/*
 * vision_process.cpp
 * IMX219(RGB) + FLIR Lepton(Thermal) 영상 캡처 및 스트리밍 프로세스
 *
 * 스레드 구조:
 *   thread_ipc           : FIFO에서 제어 명령 수신 (control_process ↔ vision_process IPC)
 *   thread_rgb_capture   : IMX219 카메라 캡처 → rgb_shared 공유 버퍼에 저장
 *   thread_rgb_stream    : rgb_shared에서 프레임 꺼내 GStreamer RTSP로 송신
 *   thread_thermal_cap   : Lepton SPI 캡처 → LeptonRingBuffer에 enqueue
 *   thread_thermal_stream: LeptonRingBuffer에서 dequeue → JPEG 인코딩 → TCP 송신
 *
 * 동기화:
 *   RGB     : pthread_mutex + pthread_cond (생산자/소비자 패턴)
 *   Thermal : pthread_mutex으로 RingBuffer enqueue/dequeue 보호
 *   IPC 제어: atomic flag (camera_active)
 *
 * 컴파일:
 *   g++ -o vision_process vision_process.cpp thermal_camera.c \
 *       $(pkg-config --cflags --libs opencv4) -lpthread -ljson-c
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <atomic>
#include <pthread.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <json-c/json.h>

#include "../include/thermal_camera.h"

/* ===== 설정 상수 ===== */
#define FIFO_PATH       "/tmp/vision_cmd"       // control_process와의 IPC 경로
#define RGB_WIDTH       1920
#define RGB_HEIGHT      1080
#define RGB_FPS         30
#define THERMAL_TCP_PORT 12346                  // 열화상 데이터 전송 TCP 포트
#define TCP_BUFFER_SIZE  1024

/* ===== 전역 제어 플래그 ===== */
static std::atomic<int> is_running(1);
static std::atomic<int> camera_active(1);

/* ===================================================================
 * RGB 공유 버퍼 (생산자/소비자 패턴)
 * thread_rgb_capture  → [rgb_shared] → thread_rgb_stream
 * mutex + cond_var로 Race Condition 방지
 * =================================================================== */
typedef struct {
    cv::Mat   frame;            // 공유 프레임
    int       has_new_frame;    // 새 프레임 준비 여부
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
} rgb_shared_t;

static rgb_shared_t rgb_shared = {
    .has_new_frame = 0,
    .mutex         = PTHREAD_MUTEX_INITIALIZER,
    .cond          = PTHREAD_COND_INITIALIZER,
};

/* ===================================================================
 * Thermal 링버퍼 + mutex (P4 fix: 기존 LeptonRingBuffer에 mutex 추가)
 * thread_thermal_cap    → [ring_buf] → thread_thermal_stream
 * =================================================================== */
static LeptonRingBuffer thermal_ring_buf = {0};
static pthread_mutex_t  thermal_mutex    = PTHREAD_MUTEX_INITIALIZER;


/* ===================================================================
 * thread_ipc: FIFO에서 제어 명령 수신
 *
 * IPC 설계 (자소서 포인트):
 *   - Named FIFO(/tmp/vision_cmd)를 통해 프로세스 간 명령 전달
 *   - control_process가 {"type":"COMMAND","payload":{"target":"CAMERA","value":true}} 전송
 *   - signal 대신 FIFO를 선택한 이유: 데이터 전달 가능, EINTR 부작용 없음
 * =================================================================== */
static void* thread_ipc(void* arg) {
    printf("* IPC Thread Started. FIFO: %s\n", FIFO_PATH);

    // FIFO 생성 (없으면 생성, 있으면 무시)
    mkfifo(FIFO_PATH, 0666);

    while (is_running.load()) {
        // O_RDONLY | O_NONBLOCK: 쓰는 쪽(control_process)이 없어도 블로킹 안 함
        int fd = open(FIFO_PATH, O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
            usleep(100000); // 100ms 후 재시도
            continue;
        }

        char buf[TCP_BUFFER_SIZE] = {0};
        ssize_t n = read(fd, buf, sizeof(buf) - 1);
        close(fd);

        if (n <= 0) {
            usleep(100000);
            continue;
        }

        // JSON 파싱
        struct json_object *root = json_tokener_parse(buf);
        if (!root) continue;

        struct json_object *type_obj, *payload_obj, *target_obj, *value_obj;
        if (json_object_object_get_ex(root, "type", &type_obj) &&
            strcmp(json_object_get_string(type_obj), "COMMAND") == 0 &&
            json_object_object_get_ex(root, "payload", &payload_obj) &&
            json_object_object_get_ex(payload_obj, "target", &target_obj) &&
            strcmp(json_object_get_string(target_obj), "CAMERA") == 0 &&
            json_object_object_get_ex(payload_obj, "value", &value_obj))
        {
            int val = json_object_get_boolean(value_obj);
            camera_active.store(val);
            printf("[IPC] Camera %s\n", val ? "ON" : "OFF");
        }

        json_object_put(root);
    }

    printf("* IPC Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * thread_rgb_capture: IMX219 캡처 → rgb_shared에 저장 (생산자)
 *
 * 동기화 포인트 (자소서 포인트):
 *   1. mutex lock → frame 복사 → has_new_frame = 1 → cond_signal → unlock
 *   2. stream 스레드가 cond_wait으로 대기 중이다가 signal 받고 깨어남
 *   → busy-wait 없이 CPU 효율적으로 생산자/소비자 동기화
 * =================================================================== */
/* ===================================================================
 * thread_rgb_capture: rpicam-vid stdout → OpenCV imdecode → rgb_shared 저장
 *
 * RPI5 PiSP 환경에서 libcamerasrc GStreamer 파이프라인은 format negotiation
 * 실패로 동작하지 않음. rpicam-vid를 subprocess로 띄워 MJPEG stdout을
 * OpenCV imdecode로 읽는 방식이 가장 안정적.
 *
 * rpicam-vid 출력 형식: MJPEG → stdout
 * 각 프레임: JPEG SOI(0xFFD8) ~ EOI(0xFFD9) 바이트 스트림
 * =================================================================== */
static void* thread_rgb_capture(void* arg) {
    printf("* RGB Capture Thread Started.\n");

    // rpicam-vid subprocess 시작
    // --nopreview: 디스플레이 출력 없음
    // --codec mjpeg: MJPEG 인코딩
    // -o -: stdout으로 출력
    // -t 0: 무한 실행
    std::string cmd =
        "rpicam-vid"
        " --nopreview"
        " --codec mjpeg"
        " --width "  + std::to_string(RGB_WIDTH)  +
        " --height " + std::to_string(RGB_HEIGHT) +
        " --framerate " + std::to_string(RGB_FPS) +
        " -t 0"
        " -o -"
        " 2>/dev/null";  // libcamera INFO 로그 억제

    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) {
        printf("[RGB] rpicam-vid 실행 실패\n");
        return NULL;
    }
    printf("[RGB] rpicam-vid 시작됨\n");

    // MJPEG 프레임 파싱용 버퍼
    std::vector<uint8_t> buf;
    buf.reserve(1024 * 1024);  // 1MB 예약

    uint8_t byte = 0, prev = 0;

    while (is_running.load()) {
        if (!camera_active.load()) {
            usleep(100000);
            continue;
        }

        // 바이트 단위로 읽어 JPEG SOI(FFD8) 탐색
        if (fread(&byte, 1, 1, pipe) != 1) break;

        if (prev == 0xFF && byte == 0xD8) {
            // JPEG 시작 감지 → 버퍼 초기화 후 SOI 저장
            buf.clear();
            buf.push_back(0xFF);
            buf.push_back(0xD8);
            prev = 0; byte = 0;

            // JPEG EOI(FFD9)까지 읽기
            while (is_running.load()) {
                if (fread(&byte, 1, 1, pipe) != 1) goto cleanup;
                buf.push_back(byte);
                if (prev == 0xFF && byte == 0xD9) break;  // EOI 감지
                prev = byte;
            }

            // JPEG 디코딩
            cv::Mat jpeg_data(1, (int)buf.size(), CV_8UC1, buf.data());
            cv::Mat frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
            if (frame.empty()) continue;

            // [생산자] mutex lock → 공유 버퍼에 복사 → cond_signal
            pthread_mutex_lock(&rgb_shared.mutex);
            rgb_shared.frame         = frame.clone();
            rgb_shared.has_new_frame = 1;
            pthread_cond_signal(&rgb_shared.cond);
            pthread_mutex_unlock(&rgb_shared.mutex);
        }
        prev = byte;
    }

cleanup:
    pclose(pipe);
    printf("* RGB Capture Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * thread_rgb_stream: rgb_shared에서 프레임 읽어 GStreamer RTSP 송신 (소비자)
 *
 * 동기화 포인트:
 *   cond_wait으로 새 프레임이 생길 때까지 sleep → CPU 낭비 없음
 *   프레임 복사 후 즉시 unlock → capture 스레드 블로킹 최소화
 * =================================================================== */
static void* thread_rgb_stream(void* arg) {
    printf("* RGB Stream Thread Started.\n");

    // GStreamer RTSP 송신 파이프라인
    // host IP는 실제 모니터링 PC IP로 변경 필요
    std::string out_pipeline =
        "appsrc ! videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
        "rtph264pay ! udpsink host=192.168.0.100 port=5004";

    cv::VideoWriter writer;
    bool writer_opened = false;

    while (is_running.load()) {
        cv::Mat local_frame;

        // [소비자] 새 프레임이 올 때까지 cond_wait으로 sleep
        pthread_mutex_lock(&rgb_shared.mutex);
        while (!rgb_shared.has_new_frame && is_running.load()) {
            pthread_cond_wait(&rgb_shared.cond, &rgb_shared.mutex);
        }
        if (!is_running.load()) {
            pthread_mutex_unlock(&rgb_shared.mutex);
            break;
        }
        local_frame              = rgb_shared.frame.clone(); // 로컬로 복사
        rgb_shared.has_new_frame = 0;
        pthread_mutex_unlock(&rgb_shared.mutex);             // 즉시 unlock

        // Writer 초기화 (첫 프레임 수신 시)
        if (!writer_opened) {
            writer.open(out_pipeline, cv::CAP_GSTREAMER, 0,
                        RGB_FPS, cv::Size(RGB_WIDTH, RGB_HEIGHT));
            if (!writer.isOpened()) {
                printf("[RGB Stream] GStreamer writer 열기 실패\n");
                break;
            }
            writer_opened = true;
            printf("[RGB Stream] RTSP 송신 시작\n");
        }

        writer.write(local_frame);
    }

    writer.release();
    printf("* RGB Stream Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * thread_thermal_cap: Lepton SPI 캡처 → LeptonRingBuffer enqueue (생산자)
 *
 * 동기화 포인트 (P4 fix):
 *   enqueue 전후로 thermal_mutex lock/unlock → stream 스레드와 Race Condition 방지
 * =================================================================== */
static void* thread_thermal_cap(void* arg) {
    printf("* Thermal Capture Thread Started.\n");

    int lepton_fd = init_lepton();
    if (lepton_fd < 0) {
        printf("[Thermal] Lepton 초기화 실패\n");
        return NULL;
    }

    uint16_t image[LEPTON_HEIGHT][LEPTON_WIDTH];

    while (is_running.load()) {
        if (!camera_active.load()) {
            usleep(100000);
            continue;
        }

        // VoSPI로 60개 패킷 수신 → image 완성
        if (lepton_capture(lepton_fd) < 0) {
            printf("[Thermal] 캡처 실패, 재시도...\n");
            usleep(50000);
            continue;
        }
        get_image(image);

        // [생산자] mutex로 enqueue 보호
        pthread_mutex_lock(&thermal_mutex);
        lepton_ringbuffer_enqueue(&thermal_ring_buf, image);
        pthread_mutex_unlock(&thermal_mutex);

        // Lepton 2.5: 8fps → 125ms 주기
        usleep(125000);
    }

    cleanup_lepton(lepton_fd);
    printf("* Thermal Capture Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * thread_thermal_stream: RingBuffer dequeue → JPEG 인코딩 → TCP 송신 (소비자)
 *
 * 전송 방식: TCP 소켓으로 JPEG 바이너리 전송
 *   - 프레임 크기(4바이트) 먼저 전송 → 이후 JPEG 데이터 전송
 *   - 모니터링 PC에서 크기 먼저 읽고 그만큼 수신
 * =================================================================== */
static void* thread_thermal_stream(void* arg) {
    printf("* Thermal Stream Thread Started.\n");

    // TCP 서버 소켓 생성
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port        = htons(THERMAL_TCP_PORT);

    bind(server_fd, (struct sockaddr*)&addr, sizeof(addr));
    listen(server_fd, 1);
    printf("[Thermal Stream] 포트 %d 대기 중...\n", THERMAL_TCP_PORT);

    int client_fd = accept(server_fd, NULL, NULL);
    printf("[Thermal Stream] 클라이언트 연결됨\n");

    uint16_t image[LEPTON_HEIGHT][LEPTON_WIDTH];
    std::vector<uchar> jpeg_buf;
    std::vector<int>   jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 85};

    while (is_running.load()) {
        // [소비자] mutex로 dequeue 보호
        pthread_mutex_lock(&thermal_mutex);
        int got = lepton_ringbuffer_dequeue(&thermal_ring_buf, image);
        pthread_mutex_unlock(&thermal_mutex);

        if (!got) {
            usleep(10000); // 버퍼 비었으면 10ms 대기
            continue;
        }

        // uint16_t raw → 8bit 정규화 → OpenCV Mat → JPEG 인코딩
        cv::Mat raw(LEPTON_HEIGHT, LEPTON_WIDTH, CV_16UC1, image);
        cv::Mat norm;
        cv::normalize(raw, norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);

        // 컬러맵 적용 (열화상 시각화)
        cv::Mat colored;
        cv::applyColorMap(norm, colored, cv::COLORMAP_JET);

        cv::imencode(".jpg", colored, jpeg_buf, jpeg_params);

        // TCP 전송: [4바이트 크기][JPEG 데이터]
        uint32_t size = htonl((uint32_t)jpeg_buf.size());
        if (send(client_fd, &size, sizeof(size), 0) < 0) break;
        if (send(client_fd, jpeg_buf.data(), jpeg_buf.size(), 0) < 0) break;
    }

    close(client_fd);
    close(server_fd);
    printf("* Thermal Stream Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * main: 스레드 생성 및 관리
 * =================================================================== */
int main(void) {
    printf("---------- [Vision] Vision Process Starting... ----------\n");

    pthread_t t_ipc, t_rgb_cap, t_rgb_stream, t_thermal_cap, t_thermal_stream;

    pthread_create(&t_ipc,            NULL, thread_ipc,            NULL);
    pthread_create(&t_rgb_cap,        NULL, thread_rgb_capture,    NULL);
    pthread_create(&t_rgb_stream,     NULL, thread_rgb_stream,     NULL);
    pthread_create(&t_thermal_cap,    NULL, thread_thermal_cap,    NULL);
    pthread_create(&t_thermal_stream, NULL, thread_thermal_stream, NULL);

    pthread_join(t_ipc,            NULL);
    pthread_join(t_rgb_cap,        NULL);
    pthread_join(t_rgb_stream,     NULL);
    pthread_join(t_thermal_cap,    NULL);
    pthread_join(t_thermal_stream, NULL);

    printf("---------- [Vision] Vision Process Shutting Down... ----------\n");
    return 0;
}
