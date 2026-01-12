/*
<배선>
    Lepton Thermal Camera Module    Raspberry Pi 4B
            VCC  ----------------->   3.3V
            GND  ----------------->   GND
            SCK  ----------------->   SPI0 SCLK (GPIO 11)
            MISO ----------------->   SPI0 MISO (GPIO 09)
            MOSI ----------------->   SPI0 MOSI (GPIO 10)
            CS   ----------------->   SPI0 CE0 (GPIO 08)
            SDA  ----------------->   I2C SDA0 (GPIO 00)
            SCL  ----------------->   I2C SCL0 (GPIO 01)
*/
#ifndef LEPTON_H
#define LEPTON_H

#include <stdint.h>
#include <stdlib.h>

#define VOSPI_FRAME_SIZE (164)
#define MAX_LOOP_COUNT (1000000000)

#define LEPTON_WIDTH 80
#define LEPTON_HEIGHT 60
#define DEBUG_ID_CRC 2  // 2: ID 및 CRC 포함, 0: 순수 이미지 데이터만
#define OFFSET_SIZE 2
#define BUFFER_SIZE 100

typedef struct {
    uint16_t buffer[OFFSET_SIZE * BUFFER_SIZE][LEPTON_HEIGHT][LEPTON_WIDTH];
    size_t head;
    size_t tail;
    size_t count;
} LeptonRingBuffer;


int init_lepton(void);
int cleanup_lepton(int fd);
int lepton_capture(int fd);
void get_image(uint16_t (*cpy_image)[LEPTON_WIDTH]);
void print_image(int fd);

int lepton_ringbuffer_is_available(LeptonRingBuffer* rb);
int lepton_ringbuffer_is_empty(LeptonRingBuffer* rb);
int lepton_ringbuffer_enqueue(LeptonRingBuffer* rb, const uint16_t image[][LEPTON_WIDTH]);
int lepton_ringbuffer_dequeue(LeptonRingBuffer* rb, uint16_t image[][LEPTON_WIDTH]);


#endif