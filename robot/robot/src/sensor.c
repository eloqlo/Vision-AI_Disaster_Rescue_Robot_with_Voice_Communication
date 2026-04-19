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
#include <string.h>
#include <sys/socket.h>
#include "../include/sensor.h"


/* Constants */
#define SPI_MODE        SPI_MODE_0
#define SPI_BITS        8
#define SPI_SPEED       1000000 // 1 MHz

#define TCP_BUFFER_SIZE 1024

/* Variables */
static int spi_fd = -1;


/* Initialization Functions */
struct gpiod_line_request* initialize_sensor_gpio(const char *chip_path) {
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

    printf("[sensor thread] SPI Initialized: %s at %d Hz\n", device, speed);
    return EXIT_SUCCESS;
}



void fetch_sensor_data(uint8_t* buffer) {
    if (spi_fd < 0) {
        perror("SPI device not initialized\n");
        return;
    }
    uint8_t rx[SPI_DATA_SIZE] = {0,};
    uint8_t tx_dummy[SPI_DATA_SIZE] = {0,};
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_dummy,
        .rx_buf = (unsigned long)rx,
        .len = SPI_DATA_SIZE,
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
    for (int i = 0; i < SPI_DATA_SIZE; i++) {
        buffer[i] = rx[i];
    }

#ifdef DEBUG
    printf("< 받은 데이터 >\n");
    printf("IMU: %u\n", buffer[6]);
    printf("Sonar: %d cm\n", (buffer[2] << 24 | buffer[3] << 16 | buffer[4] << 8 | buffer[5]));
    printf("CO : %f ppm\n\n", ((float)(buffer[0] << 8 | buffer[1])) / 10.0);
#endif
}

int transmit_sensor_data(uint8_t *spi_buf, int socket_fd) {
    
    struct json_object *root = json_object_new_object();
    struct json_object *payload = json_object_new_object();
    
    // spi_buf을 JSON 형태로 가공한다.
    float co_ppm = (float)((spi_buf[0] << 8) | spi_buf[1]) / 10.0f;  // 10으로 나누어서 ppm 단위로 변환.
    int obstacle_cm = (spi_buf[2] << 24 | spi_buf[3] << 16 | spi_buf[4] << 8 | spi_buf[5]);
    char rollover = spi_buf[6];

    // JSON 객체 구성
    json_object_object_add(payload, "co_ppm", json_object_new_double(co_ppm));
    json_object_object_add(payload, "obstacle_cm", json_object_new_int(obstacle_cm));
    json_object_object_add(payload, "rollover", json_object_new_boolean(rollover));
    json_object_object_add(root, "type", json_object_new_string("TELEMETRY"));
    json_object_object_add(root, "payload", payload);

    // JSON 문자열로 변환
    const char *json_str = json_object_to_json_string(root);
    char send_buf[TCP_BUFFER_SIZE];
    snprintf(send_buf, sizeof(send_buf), "%s\n", json_str); //TODO 이게 뭐야?

    // TCP로 전송
    ssize_t sent_bytes = send(socket_fd, send_buf, strlen(send_buf), 0);
    if (sent_bytes < 0) {
        perror("[Control Process] Failed to send sensor data over TCP");
        json_object_put(root);
        return EXIT_FAILURE;
    }

    json_object_put(root); // JSON 객체 메모리 해제
    
    return EXIT_SUCCESS;
}


/* Cleanup */
int cleanup_spi() {
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
    return EXIT_SUCCESS;
}