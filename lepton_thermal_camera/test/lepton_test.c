// gcc lepton_test.c -o lepton_test -lgpiod -lpigpio
// 2025-12-09 작성
// Lepton 2.5 Thermal Camera의 전원 제어와 SPI 통신 테스트 코드


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pigpio.h>


#define GPIO_CHIP "/dev/gpiochip0"
#define PWR_DWN_L   21      // gpio 핀 설정
#define RESET_L     20
#define MASTER_CLK  4

#define SPI_DEVICE "/dev/spidev0.0"  // SPI0, CE0 사용
#define SPI_MODE   SPI_MODE_3       
#define SPI_BITS   8
#define SPI_SPEED  10000000  // 10 MHz
#define SPI_DELAY  0

/* 
Lepton 2.5 Thermal Camera Start-up
    1. PWR_DWN_L 핀(GPIO21)을 High로 설정
    2. RESET_L 핀(GPIO20)을 Low로 설정
    3. Enable MASTER_CLK(25MHz)
    4. 5000+ clk 사이클 대기
    5. RESET_L 핀을 High로 설정
*/
int lepton_startup(void) 
{
    //1. PWR_DWN_L 핀(GPIO21)을 High로 설정 - 하얀선
    //2. RESET_L 핀(GPIO20)을 Low로 설정 - 검정선
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio 초기화 실패\n");
        return 1;
    }
    gpioSetMode(PWR_DWN_L, PI_OUTPUT);
    gpioSetMode(RESET_L, PI_OUTPUT);
    gpioWrite(PWR_DWN_L, 1);
    gpioWrite(RESET_L, 0);

    //3. MASTER_CLk enable (25MHz)
    int ret = gpioHardwarePWM(12, 25000000, 500000);
    if (ret!=0) {
        fprintf(stderr, "GPIO12 PWM 설정 실패 (에러: %d)\n", ret);
        gpioTerminate();
        return 1;
    }
    usleep(1000);   //4. 충분하게 1ms 대기 (25MHz의 5000 clk 사이클은 200us)
    gpioWrite(RESET_L, 1);  //5. RESET_L 핀을 High로 설정

    printf("=== Lepton 2.5 Thermal Camera Start-up Complete ===\n\n");
}


int main(void)
{
    // Lepton Thermal Camera Start-up
    lepton_startup();

    // Lepton VoSPI로 raw data 받아오기
    
    // Terminal에 raw data 출력



    return 0;
}