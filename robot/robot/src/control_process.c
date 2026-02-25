#define DEBUG

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <gpiod.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "../include/sensor.h"

#define PORT 12345  // TCP 포트 번호


static int is_running_flag = 1;

uint8_t motor_forward_blocked_flag = CLEAR;     // sensor_threa와 motor_thread 공유자원 -> mutex로 보호 필요

static void* thread_control(void* arg);
static void* thread_sensor(void* arg);
static void* thread_motor(void* arg);

int main() {
    /* TCP Socket 활성화 */
    // 소켓 생성
    struct sockaddr_in serv_addr;
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);    // TCP 소켓 생성(IPv4, TCP)
    if (socket_fd < 0) {
        printf("소켓 생성 실패.\n");
        return EXIT_FAILURE;
    }
    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(PORT);

    if (bind(socket_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("bind 실패");
        close(socket_fd);
        return EXIT_FAILURE;
    }
    if (listen(socket_fd, 5) < 0) {
        perror("listen 실패");
        close(socket_fd);
        return EXIT_FAILURE;
    }
    printf("* TCP 서버가 포트 %d에서 대기 중...\n", PORT);
    
    /* Thread 생성 */
    pthread_t thread_control_handle, thread_sensor_handle, thread_motor_handle;
    pthread_create(&thread_control_handle, NULL, thread_control, (void*)&socket_fd);
    pthread_create(&thread_sensor_handle, NULL, thread_sensor, (void*)&socket_fd);
    pthread_create(&thread_motor_handle, NULL, thread_motor, NULL);

    /* Cleanup */
    pthread_join(thread_control_handle, NULL);
    pthread_join(thread_sensor_handle, NULL);
    pthread_join(thread_motor_handle, NULL);
    
    return 0;
}


/* -------------------------- threads ------------------------------ */
static void* thread_control(void* arg) {
    printf("* Control Thread Started.\n");

    /* Initialization */
    int socket_fd = *(int*)arg;

    /* Loop */
    while(is_running_flag) {
        //TODO TCP 명령 수신 체크 
        /** 
         * - 연결 후 5초 이상 Connection 끊어지면, Disconnected Mode로 변경, Process별 IPC 전달로 동작모드 전환
         * - 다시 연결되면 Connected Mode로 변경, Process별 IPC 전달로 동작모드 전환
         */


        //TODO TCP 명령 JSON Parsing
        /** 
         * 1. 모터 제어
         * 2. Audio 마이크 IPC 보내기
         */

        usleep(10000); // 10ms 주기로 명령 수신 체크
    }
}

static void* thread_sensor(void* arg) {
    printf("* Sensor Thread Started.\n");

    // Initialization
    int socket_fd = *(int*)arg;
    uint8_t spi_buf[SPI_DATA_SIZE];     // sensor receive thread에서 받아오는 MCU 센서 데이터 저장 버퍼
    struct gpiod_edge_event_buffer *gpio_event_buffer = gpiod_edge_event_buffer_new(16);
    struct gpiod_line_request *gpio_request = initialize_gpio("/dev/gpiochip0");
    if(gpio_request == NULL) {
        printf("[sensor thread] GPIO 초기화 실패\n");
        return NULL;
    }
    if ((initialize_spi() == EXIT_FAILURE)) {
        printf("[sensor thread] SPI 초기화 실패\n");
        return NULL;
    }

    // Sensor Loop: MCU GPIO 인터럽트 처리(Sonar, SPI)
    while (is_running_flag) {
        if (gpiod_line_request_wait_edge_events(gpio_request, -1) > 0) {    // sleep state로 인터럽트 발생 대기
            int num_events = gpiod_line_request_read_edge_events(gpio_request, gpio_event_buffer, 16);

            for (int i=0; i<num_events; i++) {
                struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(gpio_event_buffer, i);
                uint32_t offset = gpiod_edge_event_get_line_offset(event);
                enum gpiod_edge_event_type type = gpiod_edge_event_get_event_type(event);

                // GPIO12 (Sonar 인터럽트 핀)
                if (offset == 12) { 
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) {
                        motor_forward_blocked_flag = SET;
                    }
                    else if (type == GPIOD_EDGE_EVENT_FALLING_EDGE) {
                        motor_forward_blocked_flag = CLEAR;
                    }
                }
                // GPIO13 (SPI 인터럽트 핀)
                else if (offset == 13) {
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) {
                        fetch_sensor_data(spi_buf);    // SPI로 센서 데이터 수신
                        transmit_sensor_data(spi_buf, socket_fd);  // 수신 데이터 JSON 가공 후 TCP 전송
                    }
                }
            }// Event loop end
        }
    }// While Loop End

    /* Cleanup */
    cleanup_spi();
    gpiod_edge_event_buffer_free(gpio_event_buffer);
    gpiod_line_request_release(gpio_request);

    return NULL;
}

static void* thread_motor(void* arg) {
    printf("* Motor Thread Started.\n");

    /* Initialization */
    //TODO 모터 제어 초기화

    /* Loop */
    while(is_running_flag) {
        //TODO motor_forward_blocked_flag 확인하여 모터 제어 수행
        //TODO PC에서 수신한 명령에 따라 모터 제어 수행

        usleep(10000); // 10ms 주기로 모터 제어 상태 체크
    }
}