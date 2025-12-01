#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int main() {
    int fd;
    unsigned char mode = 0;
    unsigned char bits = 8;
    unsigned int speed = 10000000; // 10MHz
    unsigned char buffer[164];

    // Open SPI device
    fd = open("/dev/spidev0.0", O_RDWR);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    // Set SPI mode
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("ioctl mode");
        close(fd);
        return 1;
    }

    // Set bits per word
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("ioctl bits");
        close(fd);
        return 1;
    }

    // Set speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("ioctl speed");
        close(fd);
        return 1;
    }

    // Read loop
    while (1) {
        if (read(fd, buffer, 164) < 0) {
            perror("read");
            break;
        }

        // Print first 4 bytes in hex
        printf("%02X %02X %02X %02X\n", buffer[0], buffer[1], buffer[2], buffer[3]);
    }

    close(fd);
    return 0;
}