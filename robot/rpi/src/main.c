#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <gpiod.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <pthread.h>
#include <fcntl.h>          
#include <sys/ioctl.h>         
#include <linux/spi/spidev.h> 

#include <json-c/json.h>    // JSON 처리 커스텀 라이브러리 (json-c)

#include "control.h"

atomic_int sonar_rising_flag = 0;     // 동기화를 위해 stdatomic 사용(C11 이상)
atomic_int sonar_falling_flag = 0;
atomic_int spi_rising_flag = 0;



int main() {
    /* Process 생성 */
    //TODO Vison Process 생성
    //TODO Audio Process 생성

    printf(" - Control Process Version %s - \n", VERSION);
    printf("------------- Control Process Started! -------------\n");

    /* Initialization */
    printf("[C] GPIO Initialization Started...\n");
    struct gpiod_line_request *request = initialize_gpio("/dev/gpiochip0");
    if(request == NULL) {
        printf("GPIO 초기화 실패\n");
        return EXIT_FAILURE;
    }
    struct gpiod_edge_event_buffer *buffer = gpiod_edge_event_buffer_new(16);
    
    if ((initialize_spi())) {
        printf("SPI 초기화 실패\n");
        return EXIT_FAILURE;
    }

    if (initialize_server()) {
        printf("서버 초기화 실패\n");
        return EXIT_FAILURE;
    }

    if (initialize_motors()) {
        printf("모터 초기화 실패\n");
        return EXIT_FAILURE;
    }
    printf("[C] Control Process Initialization Completed!\n");

    /* 나중에 Thread 생성! */
    // TODO socket 통신을 위한 쓰레드 생성
    // TODO sonar irq 처리 쓰레드 생성
    // TODO spi irq 처리 쓰레드 생성

    /* Main Loop */
    while(1) {
        // 1. Socket 통신 처리
        // TODO 클라이언트로부터 명령 수신 및 처리 코드 작성

        // 2. 모터 제어 처리
        // TODO 모터 제어 코드 작성

        // 3. GPIO 인터럽트 처리: SPI통신(1st), Sonar경고(2nd)
        if (gpiod_line_request_wait_edge_events(request, -1) > 0) {     //TODO: param(-1) 무한대기 therad로 분리해서 반드시 수정하기
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
    }// Main Loop End

    /* Cleanup */
    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);
    return 0;
}