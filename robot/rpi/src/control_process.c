// compile: $ gcc control_process.c -lgpiod
/*
TODO List:
    1. main loop에서 socket, motor, GPIO IRQ thread로 분리하여 처리해보기 (26.02.23 ~ )
*/
#include <stdio.h>
#include <stdint.h>         // uint8_t, uint32_t 등 고정 크기 정수형 정의
#include <unistd.h>         // UNIX 표준 함수 (read, write, close 등)
#include <gpiod.h>          // Linux GPIO 제어 라이브러리 (libgpiod)
#include <json-c/json.h>    // JSON 처리 라이브러리 (json-c)
#include <stdatomic.h>        // atomic 변수 사용을 위한 헤더
#include <pthread.h>

#define SET 1
#define CLEAR 0

int sonar_irq_flag = CLEAR; // 초음파 센서 인터럽트 플래그
int spi_irq_flag = CLEAR;   // SPI 인터럽트 플래그
atomic_int sonar_rising_flag = 0;     // 동기화를 위해 stdatomic 사용(C11 이상)
atomic_int sonar_falling_flag = 0;
atomic_int spi_falling_flag = 0;

uint8_t spi_buffer[7];



/* Initialization Functions */

/**
 * @brief Sonar, SPI 인터럽트를 위한 GPIO 초기화 함수.
 * @param chip_path Path to the GPIO chip device (e.g., "/dev/gpiochip4").
 * @return Pointer to the line request struct on success, NULL on failure.
 */
struct gpiod_line_request* initialize_gpio(const char *chip_path) {
    struct gpiod_chip *chip;
    struct gpiod_line_settings *settings_sonar;
    struct gpiod_line_settings *settings_spi;
    struct gpiod_line_config *line_cfg;
    struct gpiod_request_config *req_cfg;
    struct gpiod_line_request *request;

    uint32_t sonar_offset = 12;     // GPIO12 == Pin32
    uint32_t spi_offset = 13;       // GPIO13 == Pin33

    chip = gpiod_chip_open(chip_path);
    if (!chip) {
        perror("Failed to open GPIO chip");
        return NULL;
    }
    line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        perror("Failed to create line config");
        gpiod_chip_close(chip);
        return NULL;
    }

    // Pin32 (초음파 센서) 설정: Input + Both Edges
    settings_sonar = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings_sonar, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings_sonar, GPIOD_LINE_EDGE_BOTH);
    gpiod_line_config_add_line_settings(line_cfg, &sonar_offset, 1, settings_sonar);

    // Pin33 (SPI) 설정: Input + Falling Edge
    settings_spi = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings_spi, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings_spi, GPIOD_LINE_EDGE_FALLING);
    gpiod_line_config_add_line_settings(line_cfg, &spi_offset, 1, settings_spi);

    // request 설정, 라인 요청
    req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "RPI_Control_Process");
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);

    // Cleanup
    gpiod_line_settings_free(settings_sonar);
    gpiod_line_settings_free(settings_spi);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip);

    return request;
}

int initialize_spi() {
    // TODO SPI 초기화 코드 작성
    printf("SPI Initialized.\n");

    return EXIT_SUCCESS;
}

int initialize_server() {
    // TODO 소켓 초기화 코드 작성
    printf("Server Initialized.\n");

    return EXIT_SUCCESS;
}

int initialize_motors() {
    // TODO 모터 초기화 코드 작성
    printf("Motors Initialized.\n");

    return EXIT_SUCCESS;
}


/* Callback Functions */
void sonar_irq_callback(int polarity) {
    // Interrupt 우선순위 높음
    // TODO 초음파 센서 인터럽트 처리 코드 작성
    if (polarity) {
        printf("Sonar Rising Edge Detected.\n");
    } 
    else {
        printf("Sonar Falling Edge Detected.\n");
    }

}

void spi_irq_callback() {
    // Interrupt 우선순위 낮음
    // TODO SPI 인터럽트 처리 코드 작성

    printf("SPI IRQ Callback executed.\n");
}



/* Main */
int main() {
    /* Process 생성 */
    //TODO Vison Process 생성
    //TODO Audio Process 생성
    
    printf("------------- Control Process Started! -------------\n");

    /* Initialization */
    printf("[C] GPIO Initialization Started...\n");
    struct gpiod_line_request *request = initialize_gpio("/dev/gpiochip0");
    if(request == NULL){
        printf("GPIO 초기화 실패\n");
        return EXIT_FAILURE;
    }
    struct gpiod_edge_event_buffer *buffer = gpiod_edge_event_buffer_new(16);
    
    if ((initialize_spi())){
        printf("SPI 초기화 실패\n");
        return EXIT_FAILURE;
    }

    if (initialize_server()){
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

        // 3. GPIO 인터럽트 처리
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
                    if (type == GPIOD_EDGE_EVENT_FALLING_EDGE) spi_falling_flag = SET;
                }
            }
        }
        if (sonar_rising_flag) {
            sonar_rising_flag = CLEAR;
            sonar_irq_callback(1);
        }
        if (sonar_falling_flag) {
            sonar_falling_flag = CLEAR;
            sonar_irq_callback(0);
        }
        if (spi_falling_flag) {
            spi_falling_flag = CLEAR;
            spi_irq_callback();
        }
    }// Main Loop End

    /* Cleanup */
    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);
    return 0;
}