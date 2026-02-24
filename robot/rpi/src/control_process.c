/*
compile: 
    $ gcc control_process.c -lgpiod

TODO List:
    1. main loop에서 socket, motor, GPIO IRQ thread로 분리하여 처리해보기 (26.02.23 ~ )
*/
#define DEBUG
#define VERSION "1.0"


#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <gpiod.h>
#include <stdatomic.h>
#include <stdlib.h>

#include <fcntl.h>          
#include <sys/ioctl.h>         
#include <linux/spi/spidev.h> 
#include <pthread.h>

#include <json-c/json.h>    // JSON 처리 라이브러리 (json-c)

#define SET     1
#define CLEAR   0
#define SPI_MODE        SPI_MODE_0
#define SPI_BITS        8
#define SPI_SPEED       1000000 // 1 MHz

int spi_fd = -1;
int sonar_irq_flag = CLEAR; // 초음파 센서 인터럽트 플래그
int spi_irq_flag = CLEAR;   // SPI 인터럽트 플래그
atomic_int sonar_rising_flag = 0;     // 동기화를 위해 stdatomic 사용(C11 이상)
atomic_int sonar_falling_flag = 0;
atomic_int spi_rising_flag = 0;

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

    // Pin33 (SPI) 설정: Input + Rising Edge
    settings_spi = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings_spi, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings_spi, GPIOD_LINE_EDGE_RISING);
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

    // SPI CE0핀 제어 위해 Pin18을 a0(alternative function 0)으로 설정
    int result = system("pinctrl set 18 a0");
    if (result == -1) {
        // system 함수 자체의 호출 실패
        printf("GPIO 초기화중 SPI1 CE0핀 초기화 실패.\n");
    }

    return request;
}

int initialize_spi() {
    const char *device = "/dev/spidev10.0";  // SPI10: 센서 허브 데이터 받기
    uint8_t mode = SPI_MODE; 
    uint8_t bits = SPI_BITS;
    uint32_t speed = SPI_SPEED; // 일단 1 MHz
    
    spi_fd = open(device, O_RDWR);
    if (spi_fd < 0) {
        perror("Failed to open SPI device");
        return EXIT_FAILURE;
    }

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    
    printf("SPI Initialized: %s at %d Hz\n", device, speed);
    return EXIT_SUCCESS;
}

int initialize_server() {
    // TODO 소켓 초기화 코드 작성
    printf("Server 미구현.\n");

    return EXIT_SUCCESS;
}

int initialize_motors() {
    // TODO 모터 초기화 코드 작성
    printf("Motors 미구현.\n");

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
    if (spi_fd < 0) {
        perror("SPI device not initialized\n");
        return;
    }

    uint8_t rx[7] = {0,};
    uint8_t tx_dummy[7] = {0,};

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_dummy,
        .rx_buf = (unsigned long)rx,
        .len = 7,
        .delay_usecs = 0,
        .speed_hz = SPI_SPEED,
        .bits_per_word = SPI_BITS,      // word당 비트 수
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("Can't sent SPI message\n");
        return;
    }

    // 읽어온 데이터 처리
    for (int i = 0; i < 7; i++) {
        spi_buffer[i] = rx[i];
    }

#ifdef DEBUG
    printf("SPI Read: %02X %02X %02X %02X %02X %02X %02X\n", 
            rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6]);
#endif

}



/* Main */
int main() {
    /* Process 생성 */
    //TODO Vison Process 생성
    //TODO Audio Process 생성
    printf(" - Control Process Version %s - \n", VERSION);
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
            printf("SPI Falling Edge Detected.\n");
            gpiod_line_request_set_value(request, 18, GPIOD_LINE_VALUE_ACTIVE);    // SPI CE0을 LOW로 설정하여 SPI 통신 시작
            spi_irq_callback();
            gpiod_line_request_set_value(request, 18, GPIOD_LINE_VALUE_INACTIVE);  // SPI CE0을 HIGH로 설정하여 SPI 통신 종료
        }
    }// Main Loop End

    /* Cleanup */
    gpiod_edge_event_buffer_free(buffer);
    gpiod_line_request_release(request);
    return 0;
}