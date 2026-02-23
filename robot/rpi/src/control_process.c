#include <stdio.h>
#include <stdint.h>
#include <gpiod.h>          // Linux GPIO 제어 라이브러리 (libgpiod)
#include <json-c/json.h>    // JSON 처리 라이브러리 (json-c)

#define SET 1
#define CLEAR 0


int sonar_irq_flag = CLEAR; // 초음파 센서 인터럽트 플래그
int spi_irq_flag = CLEAR;   // SPI 인터럽트 플래그

uint8_t spi_buffer[7];

/* 디바이스 파일 초기화 함수 */
int initialize_gpio() {
    // TODO GPIO 초기화 코드 작성

    return 1;
}

int initialize_spi() {
    // TODO SPI 초기화 코드 작성

    return 1;
}

int initialize_server() {
    // TODO 소켓 초기화 코드 작성

    return 1;
}

/* Callback 함수 */
void sonar_irq_callback() {
    // Interrupt 우선순위 높음
    // TODO 초음파 센서 인터럽트 처리 코드 작성

    printf("Sonar IRQ Callback executed.\n");
}

void spi_irq_callback() {
    // Interrupt 우선순위 낮음
    // TODO SPI 인터럽트 처리 코드 작성

    printf("SPI IRQ Callback executed.\n");
}


/* Main loop */
int main() {
    
    //TODO Vison Process 생성
    //TODO Audio Process 생성

    printf("Control Process Initialization Start...\n");
    if(initialize_gpio() == -1){
        printf("GPIO 초기화 실패\n");
        return -1;
    }
    if((initialize_spi() == -1)){
        printf("SPI 초기화 실패\n");
        return -1;
    }
    if(initialize_server() == -1){
        printf("서버 초기화 실패\n");
        return -1;
    }
    printf("Control Process Initialization Completed.\n");

    // TODO socket 통신을 위한 쓰레드 생성
    // TODO sonar irq 처리 쓰레드 생성
    // TODO spi irq 처리 쓰레드 생성

    return 0;
}