/*
 * vision_process.cpp
 * IMX219(RGB) + FLIR Lepton(Thermal) 영상 캡처 및 스트리밍 프로세스
 *
 * 스레드 구조:
 *   thread_ipc            : FIFO에서 제어 명령 수신 (control_process ↔ vision_process IPC)
 *   thread_rgb_rtsp       : mediamtx + rpicam-vid로 H.264 RTSP 송출 (port 8554)
 *   thread_thermal_cap    : Lepton SPI 캡처 → LeptonRingBuffer에 enqueue
 *   thread_thermal_stream : LeptonRingBuffer에서 dequeue → JPEG 인코딩 → TCP 송신
 *
 * 동기화:
 *   Thermal : pthread_mutex으로 RingBuffer enqueue/dequeue 보호
 *   IPC 제어: atomic flag (camera_active)
 *
 * 컴파일:
 *   g++ -o vision_process vision_process.cpp thermal_camera.c \
 *       $(pkg-config --cflags --libs opencv4) -lpthread -ljson-c
 */

#include <opencv2/opencv.hpp>   // thermal stream JPEG 인코딩에 필요
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
 * thread_rgb_rtsp: mediamtx 실행 후 rpicam-vid --codec libav로 RTSP 송출
 *
 * 기존 thread_rgb_capture + thread_rgb_stream 두 스레드를 하나로 통합.
 *
 * 구조:
 *   1. mediamtx 실행 (RTSP 서버, port 8554)
 *   2. rpicam-vid --codec libav --libav-format rtsp 실행
 *      → rpicam-vid가 직접 H.264 인코딩 + RTSP publish 수행
 *   3. GUI(Qt QMediaPlayer): rtsp://RPI_IP:8554/rgb 로 수신
 *
 * TCP JPEG 대비 장점:
 *   - H.264: 프레임당 200~400KB → 10~50KB (대역폭 ~10배 절감)
 *   - 표준 RTSP 프로토콜: VLC 등 범용 클라이언트로도 확인 가능
 *   - 코드 단순화: 캡처/인코딩/송출을 rpicam-vid 하나가 담당
 * =================================================================== */
#define MEDIAMTX_PATH "/home/jh/mediamtx"
#define RTSP_RGB_URL  "rtsp://0.0.0.0:8554/rgb"

static void* thread_rgb_rtsp(void* arg) {
    printf("* RGB RTSP Thread Started.\n");

    // 1. mediamtx 실행
    system(MEDIAMTX_PATH " /home/jh/mediamtx.yml &>/dev/null &");

    // 2. mediamtx가 실제로 8554 포트를 열 때까지 대기 (최대 10초)
    printf("[RGB RTSP] mediamtx 포트 대기 중...\n");
    int waited = 0;
    while (waited < 100 && is_running.load()) {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr = {0};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(8554);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

        int ret = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
        close(sock);

        if (ret == 0) break;  // 포트 열림 확인
        usleep(100000);       // 100ms 대기 후 재시도
        waited++;
    }
    if (!is_running.load()) return NULL;
    printf("[RGB RTSP] mediamtx 준비 완료\n");

    // 3. rpicam-vid RTSP 송출 명령
    char cmd[512];
    snprintf(cmd, sizeof(cmd),
        "rpicam-vid -t 0 --nopreview"
        " --codec libav"
        " --libav-format rtsp"
        " --libav-audio 0"
        " --width %d --height %d --framerate %d"
        " -o %s"
        " 2>/dev/null",
        RGB_WIDTH, RGB_HEIGHT, RGB_FPS, RTSP_RGB_URL);

    // 재시작 루프
    while (is_running.load()) {
        if (!camera_active.load()) {
            usleep(100000);
            continue;
        }

        printf("[RGB RTSP] 송출 시작: %s\n", RTSP_RGB_URL);
        int ret = system(cmd);

        if (is_running.load()) {
            printf("[RGB RTSP] rpicam-vid 종료 (ret=%d), 2초 후 재시작...\n", ret);
            usleep(2000000);
        }
    }

    system("pkill rpicam-vid 2>/dev/null");
    system("pkill mediamtx 2>/dev/null");
    printf("* RGB RTSP Thread Exiting...\n");
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

    // Lepton 하드웨어 연결 시도
    int lepton_fd = init_lepton();

    // Lepton 없으면 더미 그라디언트로 대체
    // → GUI 수신 파이프라인 정상 동작 확인용
    // → 실제 Lepton 연결 시 아래 #define 제거
#define THERMAL_DUMMY_MODE

#ifdef THERMAL_DUMMY_MODE
    if (lepton_fd < 0) {
        printf("[Thermal] Lepton 없음 → 더미 모드로 동작\n");
    }
    uint16_t dummy[LEPTON_HEIGHT][LEPTON_WIDTH];
    int phase = 0;
    while (is_running.load()) {
        if (!camera_active.load()) { usleep(100000); continue; }

        // 그라디언트 패턴 생성 (프레임마다 phase 이동)
        for (int r = 0; r < LEPTON_HEIGHT; r++)
            for (int c = 0; c < LEPTON_WIDTH; c++)
                dummy[r][c] = (uint16_t)(((r * 68 + c * 40 + phase)) % 65535);
        phase = (phase + 200) % 65535;

        pthread_mutex_lock(&thermal_mutex);
        lepton_ringbuffer_enqueue(&thermal_ring_buf, dummy);
        pthread_mutex_unlock(&thermal_mutex);

        usleep(125000); // 8fps
    }
    if (lepton_fd >= 0) cleanup_lepton(lepton_fd);
    printf("* Thermal Capture Thread Exiting...\n");
    return NULL;
#endif

    // --- 실제 Lepton 코드 (THERMAL_DUMMY_MODE 해제 시 사용) ---
    if (lepton_fd < 0) {
        printf("[Thermal] Lepton 초기화 실패\n");
        return NULL;
    }

    uint16_t image[LEPTON_HEIGHT][LEPTON_WIDTH];
    while (is_running.load()) {
        if (!camera_active.load()) { usleep(100000); continue; }

        if (lepton_capture(lepton_fd) < 0) {
            printf("[Thermal] 캡처 실패, 재시도...\n");
            usleep(50000);
            continue;
        }
        get_image(image);

        pthread_mutex_lock(&thermal_mutex);
        lepton_ringbuffer_enqueue(&thermal_ring_buf, image);
        pthread_mutex_unlock(&thermal_mutex);

        usleep(125000);
    }

    cleanup_lepton(lepton_fd);
    printf("* Thermal Capture Thread Exiting...\n");
    return NULL;
}


/* send() 완전 전송 보장 헬퍼 (thermal stream에서 사용) */
static int send_all(int fd, const void* buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = send(fd, (const char*)buf + sent, len - sent, MSG_NOSIGNAL);
        if (n <= 0) return -1;
        sent += n;
    }
    return 0;
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

    std::vector<uchar> jpeg_buf;
    std::vector<int>   jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 85};

    // 재연결 루프
    while (is_running.load()) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) break;
        printf("[Thermal Stream] 클라이언트 연결됨\n");

        uint16_t image[LEPTON_HEIGHT][LEPTON_WIDTH];
        while (is_running.load()) {
            pthread_mutex_lock(&thermal_mutex);
            int got = lepton_ringbuffer_dequeue(&thermal_ring_buf, image);
            pthread_mutex_unlock(&thermal_mutex);

            if (!got) { usleep(10000); continue; }

            cv::Mat raw(LEPTON_HEIGHT, LEPTON_WIDTH, CV_16UC1, image);
            cv::Mat norm;
            cv::normalize(raw, norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::Mat colored;
            cv::applyColorMap(norm, colored, cv::COLORMAP_JET);
            cv::imencode(".jpg", colored, jpeg_buf, jpeg_params);

            uint32_t size = htonl((uint32_t)jpeg_buf.size());
            if (send_all(client_fd, &size, sizeof(size)) < 0) break;
            if (send_all(client_fd, jpeg_buf.data(), jpeg_buf.size()) < 0) break;
        }
        close(client_fd);
        printf("[Thermal Stream] 클라이언트 연결 끊김, 재연결 대기...\n");
    }

    close(server_fd);
    printf("* Thermal Stream Thread Exiting...\n");
    return NULL;
}


/* ===================================================================
 * main: 스레드 생성 및 관리
 * =================================================================== */
int main(void) {
    printf("---------- [Vision] Vision Process Starting... ----------\n");

    pthread_t t_ipc, t_rgb_rtsp, t_thermal_cap, t_thermal_stream;

    pthread_create(&t_ipc,            NULL, thread_ipc,          NULL);
    pthread_create(&t_rgb_rtsp,       NULL, thread_rgb_rtsp,     NULL);
    pthread_create(&t_thermal_cap,    NULL, thread_thermal_cap,  NULL);
    pthread_create(&t_thermal_stream, NULL, thread_thermal_stream, NULL);

    pthread_join(t_ipc,            NULL);
    pthread_join(t_rgb_rtsp,       NULL);
    pthread_join(t_thermal_cap,    NULL);
    pthread_join(t_thermal_stream, NULL);

    printf("---------- [Vision] Vision Process Shutting Down... ----------\n");
    return 0;
}
