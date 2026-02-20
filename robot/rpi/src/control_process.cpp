/*
26.02.20 - Baseline Control Process 코드 개발완료

*/
#include <iostream>
#include <string>
#include <thread>
#include <atomic>           // 플래그 변수를 위한 헤더
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <pigpio.h>
#include <nlohmann/json.hpp>
#include <cstdint>

using json = nlohmann::json;
using namespace std;

// --- 설정 ---
const int TCP_PORT = 12345;
const int INTERRUPT_PIN = 13;       // 1. SPI Slave 인터럽트 핀을 13번으로 변경
const char* SPI_DEVICE = "/dev/spidev1.0"; 

int spi_fd = -1;
int global_client_sock = -1;

// 2. Rising Edge 발생 시 set될 플래그 변수 (Thread-safe)
std::atomic<bool> sensor_event_flag(false);

// --- SPI 데이터 읽기 및 전송 ---
void read_and_send_spi_data() {
    if (spi_fd < 0) return;

    static uint8_t rx[7] = {0, };
    struct spi_ioc_transfer tr = {
        .tx_buf = 0,
        .rx_buf = (unsigned long)rx,
        .len = 7,
        .speed_hz = 1000000,
        .delay_usecs = 0,
        .bits_per_word = 8,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) >= 1) {
        
        // 하드웨어 데이터 파싱
        int co_val = static_cast<int>((static_cast<uint16_t>(rx[0]) << 8) | rx[1]);
        int sonar_val = (static_cast<int>(rx[2]) << 24) |
                        (static_cast<int>(rx[3]) << 16) |
                        (static_cast<int>(rx[4]) << 8)  |
                        rx[5];
        bool rollover_val = (rx[6] == 1);

        json telemetry_data = {
            {"type", "TELEMETRY"},
            {"payload", {
                {"co_ppm", co_val},             // int 일산화탄소 센서 측정값  
                {"obstacle_cm", sonar_val},     // int 초음파 센서 거리값
                {"rollover", rollover_val}      // Bool 전복 감지 여부 (true/false)
            }}
        };

        // 문자열 변환 및 개행 문자(\n) 추가 (TCP 스트림 구분용)
        string msg = telemetry_data.dump() + "\n";
        
        // 클라이언트로 전송
        if (global_client_sock != -1) {
            send(global_client_sock, msg.c_str(), msg.length(), 0);
        }
    }
}

// --- GPIO 인터럽트 핸들러 (GPIO 13) ---
void gpio_callback(int gpio, int level, uint32_t tick) {
    if (level == 1) { // Rising Edge 감지
        // 특정 변수를 즉시 set
        sensor_event_flag.store(true);
        
        // 기존 SPI 읽기 로직 실행
        read_and_send_spi_data();
        
        // 필요 시 여기서 flag를 다시 false로 낮추거나, 
        // 다른 쓰레드에서 처리 후 낮추도록 설계할 수 있습니다.
        cout << "SPI 인터럽트 처리 완료" << endl;
    }
}

// --- SPI 초기화 ---
void init_spi() {
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) return;
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 1000000;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
}

// --- 명령 처리 및 서버 로직 (기존과 동일) ---
void process_command(json payload) {
    string target = payload.value("target", "");
    string value = payload.value("value", "");
    cout << "명령 처리: " << target << " -> " << value << endl;
}

void start_server() {
    if (gpioInitialise() < 0) return;
    init_spi();
    
    // 1. GPIO 13번 설정
    gpioSetMode(INTERRUPT_PIN, PI_INPUT);
    gpioSetPullUpDown(INTERRUPT_PIN, PI_PUD_DOWN);
    gpioSetAlertFunc(INTERRUPT_PIN, gpio_callback);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(TCP_PORT);

    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 1);

    cout << "서버 가동 중 (Interrupt Pin: 13)" << endl;

    while (true) {
        int client_sock = accept(server_fd, nullptr, nullptr);
        global_client_sock = client_sock;

        char buffer[1024];
        while (true) {
            ssize_t valread = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
            if (valread <= 0) break;
            buffer[valread] = '\0';
            try {
                auto j = json::parse(buffer);
                if (j["type"] == "COMMAND") {
                    process_command(j["payload"]);
                }
                else{
                    cout << "알 수 없는 메시지 타입: " << j["type"] << endl;
                }
                
            } catch (...) {}
        }
        close(client_sock);
        global_client_sock = -1;
    }
    gpioTerminate();
}

int main() {
    start_server();
    return 0;
}