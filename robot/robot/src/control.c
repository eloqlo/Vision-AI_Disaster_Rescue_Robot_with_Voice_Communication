/*
compile: 
    gcc control.c -lgpiod -o control
*/
// #define DEBUG

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <gpiod.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <fcntl.h>          
#include <sys/ioctl.h>         
#include <linux/spi/spidev.h> 

#include <json-c/json.h>    // JSON 처리 라이브러리 (json-c)
#include "../include/control.h"


/* Constants */
#define SPI_MODE        SPI_MODE_0
#define SPI_BITS        8
#define SPI_SPEED       1000000 // 1 MHz


/* Variables */
static int spi_fd = -1;
static uint8_t spi_buf[7];     // spi_irq_callback()에서 받아온 데이터 저장 버퍼
extern motor_forward_blocked_flag;   // main.c에서 정의된 모터 Forward 명령 차단 플래그


/* Initialization Functions */
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
    const char *device = "/dev/spidev1.0";  // SPI1: 센서 허브 데이터 받기
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




/* Callback Functions */
// Interrupt 우선순위 높음
// TODO Motor Thread 생성 후 구현, thread로 분리해서 수정
void sonar_irq_callback(int polarity) {
    if (polarity) {
#ifdef DEBUG
        printf("Sonar Rising Edge Detected.\n");
#endif
        motor_forward_blocked_flag = 1;
    } 
    else {
#ifdef DEBUG
        printf("Sonar Falling Edge Detected.\n");
#endif
        motor_forward_blocked_flag = 0;
    }
}


// Interrupt 우선순위 낮음
void spi_irq_callback() {
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

    // SPI 메시지 수신
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("Can't sent SPI message\n");
        return;
    }

    // 읽어온 데이터 처리
    for (int i = 0; i < 7; i++) {
        spi_buf[i] = rx[i];
    }

#ifdef DEBUG
    printf("< 받은 데이터 >\n");
    printf("IMU: %u\n", spi_buf[6]);
    printf("Sonar: %d cm\n", (spi_buf[2] << 24 | spi_buf[3] << 16 | spi_buf[4] << 8 | spi_buf[5]));
    printf("CO : %f ppm\n\n", ((float)(spi_buf[0] << 8 | spi_buf[1])) / 10.0);
#endif
}


/* Cleanup */
int cleanup_spi() {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
    return EXIT_SUCCESS;
}