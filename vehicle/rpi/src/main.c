#include <stdio.h>
#include <pthread.h>

#include "lepton.h"
#include "ringbuffer.h"


static void* lepton_capture_thread(void* arg) {
    uint16_t (*image_ptr)[80];
    while(1)
    {
        lepton_capture();
        image_ptr = get_image();

        // TODO Ring Buffer에 image 추가
    }
}

static void* lepton_transmit_thread(void* arg) {
    // TODO Ring Buffer에서 image 꺼내서 전송
}

int main(void){
    int lepton_fd = init_lepton();
    int ret = 0;
    LeptonRingBuffer lepton_ring_buffer = { .head = 0, .tail = 0, .count = 0 };

    pthread_t lepton_capture_thread_id;
    pthread_t lepton_transmit_thread_id;
    pthread_create(&lepton_capture_thread_id, NULL, lepton_capture_thread, NULL);
    pthread_create(&lepton_transmit_thread_id, NULL, lepton_transmit_thread, NULL);

    // TODO 적절한 종료 조건 추가 필요

    pthread_join(lepton_capture_thread_id, NULL);
    pthread_join(lepton_transmit_thread_id, NULL);
    cleanup_lepton(lepton_fd);
    return 0;
}