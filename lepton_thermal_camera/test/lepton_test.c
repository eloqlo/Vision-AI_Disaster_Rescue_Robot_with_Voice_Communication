// gcc lepton_test.c -o lepton_test -lgpiod -lpigpio
// 2025-12-09 작성
// Lepton 2.5 Thermal Camera의 전원 제어와 SPI 통신 테스트 코드
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>          // open() 함수
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/time.h>
#include <sys/select.h>
#include <pigpio.h>

#define GPIO_CHIP "/dev/gpiochip0"
#define PWR_DWN_L   21      // gpio 핀 설정
#define RESET_L     20
#define MASTER_CLK  4

#define SPI_DEVICE "/dev/spidev0.0"
#define SPI_MODE SPI_MODE_3
#define SPI_BITS 8
#define SPI_SPEED 10000000   // 10MHz

#define WIDTH 80        // 160 pixels & 2 bytes per pixel
#define HEIGHT 60
#define FRAME_SIZE (WIDTH * HEIGHT * 2)
#define PACKET_SIZE 164
#define PACKETS_PER_FRAME 60

#define DEASSERT_CS_TIME 200000    // >185ms
#define FRAME_AVAILABLE_TIME 39000 // <39ms after /CS assert
#define INTRA_PACKET_TIMEOUT 5000  // 3ms + margin = 5ms


#define CRC_POLY 0x1021 // Lepton CRC polynomial: x^16 + x^12 + x^5 + 1



uint16_t calculate_crc(uint8_t *data, int len) {
    uint16_t crc = 0;
    
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x10000) {
                crc ^= CRC_POLY;
                crc &= 0xFFFF;
            }
        }
    }
    
    return crc;
}

int spi_read_packet(int spi_fd, uint8_t *rx_buf) {
    uint8_t tx_buf[PACKET_SIZE] = {0}; // 더미 전송 버퍼

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx_buf,
        .rx_buf = (unsigned long)rx_buf,
        .len = PACKET_SIZE,
        .speed_hz = SPI_SPEED,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS,
        .cs_change = 0,
    };
    
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("SPI 패킷 읽기 실패\n");
        return -1;
    }
    
    return 0;
}

int vospi_sync(int spi_fd) {
    uint8_t rx_buf[PACKET_SIZE];
    
    // 1. /CS deassert & idle SCK >185ms 대기 (VoSPI timeout 만들기)
    usleep(DEASSERT_CS_TIME);   // 200ms 대기
    
    // 2. /CS assert & SCLK enable
    printf("\n[Step 2, 3] /CS assert & SCLK enable --> Discard packet 읽기\n");
    memset(rx_buf, 0, PACKET_SIZE);
    
    // 3. Discard packet 읽기 (ID field가 xFxx)
    if (spi_read_packet(spi_fd, rx_buf) < 0){
        fprintf(stderr, "packet 읽기 실패\n");
        return -1;
    }
    
    // 4. 첫 packet_id=0 감지(<39ms)
    printf("  VoSPI 헤더: 0x%02X\n", rx_buf[0]);
    if ((rx_buf[0] & 0x0F) != 0x0F) {
        fprintf(stderr, "  discard packet 안읽힘!!\n");
        return -1;
    }

    printf("  Discard packet 읽음 (ID: 0x%04X)\n", rx_buf[1]);
    return 0;
}

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

    printf("[Lepton 2.5 Thermal Camera Start-up Complete...]\n\n");

    return 0;
}

uint8_t lepton_frame_packet[PACKET_SIZE];

int trasnfer(int fd)
{
    int ret;
    int i;
    int frame_number;
    uint8_t tx[PACKET_SIZE] = {0, };
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)lepton_frame_packet,
		.len = PACKET_SIZE,
		.delay_usecs = 0,
		.speed_hz = SPI_SPEED,
		.bits_per_word = SPI_BITS,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1)
        perror("can't send spi message\n");

    if(((lepton_frame_packet[0]&0x0f) != 0x0f))
    {
        frame_number = lepton_frame_packet[1];

        if(frame_number < 60)
        {
            for(i=0;i<80;i++)
            {
                //TODO  image 받아오기
                // lepton_image[frame_number][i] = (lepton_frame_packet[2*i+4] << 8 | lepton_frame_packet[2*i+5]);

            }
        }
    }

}


int main(void)
{
    int ret = 0;

    // Lepton Thermal Camera Start-up
    if(lepton_startup() != 0){
        printf("Somethings wrong in Lepton startup...\n");
        return -1;
    }
    
    int spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        perror("SPI device open 실패");
        return 1;
    }
    
    uint8_t mode = SPI_MODE;        // Mode 3
    uint32_t speed = SPI_SPEED;     // 10MHz
    uint8_t bits = SPI_BITS;        // 8 bits

    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_RD_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
    
    // VoSPI 동기화
    if (vospi_sync(spi_fd) != 0) {
        fprintf(stderr, "\n❌ 초기 동기화 실패\n");
        return 1;
    }

    

    close(spi_fd);

    return 0;
}