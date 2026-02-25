#define DEBUG

#include <stdio.h>
#include <stdint.h>
#include <stdatomic.h>
#include <pthread.h>
#include <stdlib.h>
#include <gpiod.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "../include/control.h"



static atomic_int sonar_rising_flag = 0;     // 동기화를 위해 stdatomic 사용(C11 이상)
static atomic_int sonar_falling_flag = 0;
static atomic_int spi_rising_flag = 0;
static is_running_flag = 1;
uint8_t motor_forward_blocked_flag = 0;


void* thread_get_socket(void* arg);
void* thread_send_socket(void* arg);
void* thread_mcu_process(void* arg);


int main() {
    /* Process 생성 */
    //TODO Motor Process 생성
    //TODO Vison Process 생성
    //TODO Audio Process 생성

    pthread_t thread_mcu_handle, thread_get_socket_handle, thread_send_socket_handle;
    pthread_create(&thread_mcu_handle, NULL, thread_mcu_process, NULL);
    pthread_create(&thread_get_socket_handle, NULL, thread_get_socket, NULL);
    pthread_create(&thread_send_socket_handle, NULL, thread_send_socket, NULL);

    while (is_running_flag) {
        //TODO 

        usleep(100000); // 100ms
    }
    
    /* Cleanup */
    pthread_join(thread_mcu_handle, NULL);
    pthread_join(thread_get_socket_handle, NULL);
    pthread_join(thread_send_socket_handle, NULL);
    
    return 0;
}

// PC(Client) -> RPI(Server) : 명령 수신
void* thread_get_socket(void* arg) {
    printf("Server Thread Started.\n");

    /* Initialization */
    //TODO 서버 초기화

    /* Loop */
    while(is_running_flag) {
        //TODO 10ms 주기로 소켓 체크, 명령 획득

        usleep(10000); // 10ms 주기로 소켓 상태 체크
    }
}

// RPI(Server) -> PC(Client) : 센서 데이터 송신
void* thread_send_socket(void* arg) {
    printf("Socket Send Thread Started.\n");

    /* Initialization */
    //TODO 서버 초기화

    /* Loop */
    while(is_running_flag) {
        //TODO 100ms 주기로 센서 데이터 송신

        usleep(100000); // 100ms 주기로 센서 데이터 송신
    }
    return NULL;
}

// MCU(Slave) -> RPI(Master) : GPIO 인터럽트 처리 및 SPI 통신
void* thread_mcu_process(void* arg) {
    printf("MCU Process Thread Started.\n");

    // Initialization
    struct gpiod_edge_event_buffer *buffer = gpiod_edge_event_buffer_new(16);
    struct gpiod_line_request *request = initialize_gpio("/dev/gpiochip0");
    if(request == NULL) {
        printf("GPIO 초기화 실패\n");
        return EXIT_FAILURE;
    }
    if ((initialize_spi())) {
        printf("SPI 초기화 실패\n");
        return EXIT_FAILURE;
    }

    // MCU Loop: MCU에서 발생하는 인터럽트 처리(Sonar, SPI)
    while (is_running_flag) {
        if (gpiod_line_request_wait_edge_events(request, -1) > 0) {
            int num_events = gpiod_line_request_read_edge_events(request, buffer, 16);

            for (int i=0; i<num_events; i++) {
                struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(buffer, i);
                uint32_t offset = gpiod_edge_event_get_line_offset(event);
                enum gpiod_edge_event_type type = gpiod_edge_event_get_event_type(event);

                if (offset == 12) { // Pin32 (Sonar IRQ)
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) sonar_rising_flag = SET;
                    else if (type == GPIOD_EDGE_EVENT_FALLING_EDGE) sonar_falling_flag = SET;
                }
                else if (offset == 13) { // Pin33 (SPI IRQ)
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) spi_rising_flag = SET;
                }
            }
        }
        if (sonar_rising_flag) {
            sonar_rising_flag = CLEAR;
            sonar_irq_callback(1);  // 1: Rising Edge
        }
        if (sonar_falling_flag) {
            sonar_falling_flag = CLEAR;
            sonar_irq_callback(0);  // 0: Falling Edge
        }
        if (spi_rising_flag) {
            spi_rising_flag = CLEAR;
            spi_irq_callback();
        }
    }// MCU Loop End


    /* Cleanup */
    cleanup_spi();
    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);

    return NULL;
}