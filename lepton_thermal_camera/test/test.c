/*
배선도
Lepton Thermal Camera Module    Raspberry Pi 4B
        VCC  ----------------->   3.3V
        GND  ----------------->   GND
        SCK  ----------------->   SPI0 SCLK (GPIO 11)
        MISO ----------------->   SPI0 MISO (GPIO 09)
        MOSI ----------------->   SPI0 MOSI (GPIO 10)
        CS   ----------------->   SPI0 CE0 (GPIO 08)
        SDA  ----------------->   I2C SDA0 (GPIO 00)
        SCL  ----------------->   I2C SCL0 (GPIO 01)
        PWR_DWN_L ------------>   GPIO 21
        RESET_L -------------->   GPIO 20
        MASTER_CLK ----------->   GPIO13 (PWM1)
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <limits.h>
#include <string.h>
#include <sys/select.h>
#include <pigpio.h>


#define GPIO_CHIP "/dev/gpiochip0"
#define PWR_DWN_L   21      // gpio 핀 설정
#define RESET_L     20
#define PWM1_PIN    13
#define MASTER_CLK  4


// > 하나하나 검증결과
// 1. PWR_DWN_L 핀 High로 잘 설정됨.
// 2. RESET_L 핀 Low -> High로 잘 설정됨.
// 3. GPIO13 PWM1 잘 설정됨.
int lepton_startup(void) 
{
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio 초기화 실패\n");
        return 1;
    }

    //1. De-assert PWR_DWN_L (should be High)
    gpioSetMode(PWR_DWN_L, PI_OUTPUT);
    gpioWrite(PWR_DWN_L, 1);
    //2. Assert RESET_L (should be Low)
    gpioSetMode(RESET_L, PI_OUTPUT);
    gpioWrite(RESET_L, 0);
    //3. Enable MASTER_CLK (25MHz) -- PWM1 (GPIO13) 사용
    int ret = gpioHardwarePWM(PWM1_PIN, 25000000, 500000);
    if (ret!=0) {
        fprintf(stderr, "GPIO13 PWM1 설정 실패 (에러: %d)\n", ret);
        gpioTerminate();
        return 1;
    }
    //4. Wait > 5000 clk periods (200us 이상)
    usleep(500); // 500us 대기

    //5. De-assert RESET_L
    gpioWrite(RESET_L, 1);

    printf("=== Lepton 2.5 Thermal Camera Start-up Complete ===\n");

    return 0;
}


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static const char *device = "/dev/spidev0.0";
static uint8_t mode = SPI_MODE_3;
static uint8_t bits = 8;
static uint32_t speed = 16000000;   // 16MHz
static uint16_t delay = 0;

#define VOSPI_FRAME_SIZE (164)
uint8_t lepton_frame_packet[VOSPI_FRAME_SIZE];
static unsigned int lepton_image[80][80];

int transfer(int fd)
{
    int ret;
    int i;
    int frame_number;
    uint8_t tx[VOSPI_FRAME_SIZE] = {0, };
    lepton_frame_packet[0] = 0x23; // dummy data
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)lepton_frame_packet,
		.len = VOSPI_FRAME_SIZE,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if(ret < 1)
        pabort("can't send spi message");

    if(((lepton_frame_packet[0] & 0xf) != 0x0f))
    {
        frame_number = lepton_frame_packet[1];

        if(frame_number < 60)
        {
            for(i=0;i<80;i++)
            {
                lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);
            }
        }
    }

    // 받아온 프레임 찍어보자...
    printf("받아온 프레임 일부를 찍어보자...\n");
    printf("%d번 프레임 -- ", frame_number);
    for (i=0;i<10;i++){
        printf("%02X ", lepton_frame_packet[i]);
    }
    printf("\n");

    return frame_number;
}

int main(int argc, char *argv[]){
    int ret = 0;
    int fd;
    int i;

    // Lepton Thermal Camera Start-up
    if(lepton_startup() != 0){
        printf("Somethings wrong in Lepton startup...\n");
        return -1;
    }

    // datasheet 4.2.2.3.1) Establishing Sync
    //1. Deassert /CS and idle SCK for at least 185ms(5 frame periods)
    usleep(300);

    fd = open(device, O_RDWR);
    if (fd < 0)
    {
        pabort("can't open device");
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret == -1)
    {
        pabort("can't set spi mode");
    }

	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)  
	{
		pabort("can't set bits per word");
	}

	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		pabort("can't set max speed hz");
	}

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d MHz)\n", speed, speed/1000000);
    
    //2. Assert /CS and enable SCLK. This action causes the Lepton to start trasmission of a first packet.
    transfer(fd);

    //3. Examine the ID field of the packet, identifying a discard packet.
    while(transfer(fd) != 59){}

    //4. Continue reading packets.

    // while(transfer(fd)!=59){}
    
    close(fd);

    return 0;
}