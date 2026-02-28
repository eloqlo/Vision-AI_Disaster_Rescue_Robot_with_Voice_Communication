#define DEBUG

#include <stdio.h>
#include <stdint.h>
#include <pthread.h>
#include <stdlib.h>
#include <gpiod.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <json-c/json.h>
#include <sys/msg.h>

#include "../include/sensor.h"
#include "../include/motor.h"

#define PORT 12345  // TCP 포트 번호
#define TCP_BUFFER_SIZE 1024
#define MSG_TYPE_MOTOR_CONTROL 1

typedef struct {
    long msg_type;
    char direction;
} motor_msg_t;
int motor_queue_id; // 모터 제어 메시지 큐 ID


static int is_running_flag = 1;

uint8_t motor_forward_blocked_flag = CLEAR;     // sensor_threa와 motor_thread 공유자원 -> mutex로 보호 필요

static void* thread_control(void* arg);
static void* thread_sensor(void* arg);
static void* thread_motor(void* arg);

int main() {
    /* TCP Socket 활성화 */
    // 소켓 생성
    struct sockaddr_in serv_addr;
    int socket_fd = socket(AF_INET, SOCK_STREAM, 0);    // TCP 소켓 생성(IPv4, TCP)
    if (socket_fd < 0) {
        printf("소켓 생성 실패.\n");
        return EXIT_FAILURE;
    }
    int opt = 1;
    setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(PORT);

    if (bind(socket_fd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("bind 실패");
        close(socket_fd);
        return EXIT_FAILURE;
    }
    if (listen(socket_fd, 5) < 0) {
        perror("listen 실패");
        close(socket_fd);
        return EXIT_FAILURE;
    }
    printf("* TCP 서버가 포트 %d에서 대기 중...\n", PORT);
    
    /* 메시지 큐 생성 */
    motor_queue_id = msgget(IPC_PRIVATE, IPC_CREAT | 0666);
    if (motor_queue_id == -1) {
        perror("메시지 큐 생성 실패");
        close(socket_fd);
        return EXIT_FAILURE;
    }

    /* Thread 생성 */
    pthread_t thread_control_handle, thread_sensor_handle, thread_motor_handle;
    pthread_create(&thread_control_handle, NULL, thread_control, (void*)&socket_fd);
    pthread_create(&thread_sensor_handle, NULL, thread_sensor, (void*)&socket_fd);
    pthread_create(&thread_motor_handle, NULL, thread_motor, NULL);

    /* Cleanup */
    pthread_join(thread_control_handle, NULL);
    pthread_join(thread_sensor_handle, NULL);
    pthread_join(thread_motor_handle, NULL);
    
    return 0;
}


/* -------------------------- threads ------------------------------ */
static void* thread_control(void* arg) {
    printf("* Control Thread Started.\n");

    /* Initialization */
    int socket_fd = *(int*)arg;

    /* Loop */
    while(is_running_flag) {
        // Client Connection Handling
        printf("[control thread] Waiting for client connection...\n");
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client_sock = accept(socket_fd, (struct sockaddr*)&client_addr, &addr_len);
        if (client_sock < 0) continue;
        printf("[control thread] Client connected: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        while(is_running_flag) {
            char buffer[TCP_BUFFER_SIZE];
            ssize_t bytes_received = recv(client_sock, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received <= 0) { // 0(종료), -1(에러)
                if(bytes_received == 0) {
                    printf("[control thread] Client disconnected: %s:%d\n", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
                } else {
                    perror("[control thread] recv 실패");
                }
                close(client_sock);
                break;
            }

            // JSON Pasring Logic
            buffer[bytes_received] = '\0'; 

            struct json_object *parsed_json = json_tokener_parse(buffer);
            if (parsed_json == NULL) {
                printf("[control] JSON 파싱 실패: %s\n", buffer);
                continue; 
            }

            struct json_object *type_obj, *payload_obj;

            // 1. 최상위 "type" 확인
            if (json_object_object_get_ex(parsed_json, "type", &type_obj)) {
                const char *type_str = json_object_get_string(type_obj);

                // 2. COMMAND 타입일 때만 처리
                if (strcmp(type_str, "COMMAND") == 0 && json_object_object_get_ex(parsed_json, "payload", &payload_obj)) {
                    
                    struct json_object *target_obj, *value_obj;
                    
                    if (json_object_object_get_ex(payload_obj, "target", &target_obj)) {
                        const char *target = json_object_get_string(target_obj);

                        if (json_object_object_get_ex(payload_obj, "value", &value_obj)) {
                            if (strcmp(target, "DRIVE") == 0) {
                                // DRIVE 명령 처리
                                const char *drive_val = json_object_get_string(value_obj);
                                motor_msg_t motor_msg;
                                motor_msg.msg_type = MSG_TYPE_MOTOR_CONTROL;

                                printf("[COMMAND] 주행 명령: %s\n", drive_val);
                                
                                if(strcmp(drive_val, "F") == 0) motor_msg.direction = 'F';
                                else if(strcmp(drive_val, "B") == 0) motor_msg.direction = 'B';
                                else if(strcmp(drive_val, "R") == 0) motor_msg.direction = 'R';
                                else if(strcmp(drive_val, "L") == 0) motor_msg.direction = 'L';
                                else  motor_msg.direction = 'S';    // 예외 처리도 정지로 처리
                                
                                // 메시지 큐로 전송
                                if (msgsnd(motor_queue_id, &motor_msg, sizeof(char), 0) == -1) {
                                    perror("[control thread] 메시지 큐 전송 실패");
                                }
                            }
                            else if (strcmp(target, "MIC") == 0) {
                                // TODO 오디오 스레드 제어 로직 연결
                                int mic_on = json_object_get_boolean(value_obj);    // boolean 타입으로 변환
                                printf("[COMMAND] 마이크 상태 변경: %s\n", mic_on ? "ON" : "OFF");

                            }
                        }
                    }
                }
            }

            json_object_put(parsed_json); // 메모리 해제

            //TODO JSON Parsing Logic END
            
        }
    }

    /* Cleanup */
    close(socket_fd);

    return NULL;
}

static void* thread_sensor(void* arg) {
    printf("* Sensor Thread Started.\n");

    // Initialization
    int socket_fd = *(int*)arg;
    uint8_t spi_buf[SPI_DATA_SIZE];     // sensor receive thread에서 받아오는 MCU 센서 데이터 저장 버퍼
    struct gpiod_edge_event_buffer *sensor_event_buffer = gpiod_edge_event_buffer_new(16);
    struct gpiod_line_request *sensor_request = initialize_sensor_gpio("/dev/gpiochip0");
    if(sensor_request == NULL) {
        printf("[sensor thread] GPIO 초기화 실패\n");
        return NULL;
    }
    if ((initialize_spi() == EXIT_FAILURE)) {
        printf("[sensor thread] SPI 초기화 실패\n");
        return NULL;
    }

    // Sensor Loop: MCU GPIO 인터럽트 처리(Sonar, SPI)
    while (is_running_flag) {
        if (gpiod_line_request_wait_edge_events(sensor_request, -1) > 0) {    // sleep state로 인터럽트 발생 대기
            int num_events = gpiod_line_request_read_edge_events(sensor_request, sensor_event_buffer, 16);

            for (int i=0; i<num_events; i++) {
                struct gpiod_edge_event *event = gpiod_edge_event_buffer_get_event(sensor_event_buffer, i);
                uint32_t offset = gpiod_edge_event_get_line_offset(event);
                enum gpiod_edge_event_type type = gpiod_edge_event_get_event_type(event);

                // GPIO12 (Sonar 인터럽트 핀)
                if (offset == 12) { 
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) {
                        motor_forward_blocked_flag = SET;
                    }
                    else if (type == GPIOD_EDGE_EVENT_FALLING_EDGE) {
                        motor_forward_blocked_flag = CLEAR;
                    }
                }
                // GPIO13 (SPI 인터럽트 핀)
                else if (offset == 13) {
                    if (type == GPIOD_EDGE_EVENT_RISING_EDGE) {
                        fetch_sensor_data(spi_buf);    // SPI로 센서 데이터 수신
                        transmit_sensor_data(spi_buf, socket_fd);  // 수신 데이터 JSON 가공 후 TCP 전송
                    }
                }
            }// Event loop end
        }
    }// While Loop End

    /* Cleanup */
    cleanup_spi();
    gpiod_edge_event_buffer_free(sensor_event_buffer);
    gpiod_line_request_release(sensor_request);

    return NULL;
}

static void* thread_motor(void* arg) {
    printf("* Motor Thread Started.\n");

    /* Initialization */
    struct gpiod_line_request *motor_request = initialize_motor_gpio("/dev/gpiochip0");
    if (motor_request == NULL) {
        printf("[motor thread] GPIO 초기화 실패\n");
        return NULL;
    }
    motor_msg_t received_data;

    enum gpiod_line_value forward_gpio_line_values[] = {
        GPIOD_LINE_VALUE_ACTIVE, // IN2
        GPIOD_LINE_VALUE_INACTIVE, // IN1
        GPIOD_LINE_VALUE_ACTIVE, // IN3
        GPIOD_LINE_VALUE_INACTIVE // IN4
    };
    enum gpiod_line_value right_gpio_line_values[] = {
        GPIOD_LINE_VALUE_INACTIVE,
        GPIOD_LINE_VALUE_ACTIVE,
        GPIOD_LINE_VALUE_ACTIVE,
        GPIOD_LINE_VALUE_INACTIVE, 
    };
    enum gpiod_line_value backward_gpio_line_values[] = {
        GPIOD_LINE_VALUE_INACTIVE, 
        GPIOD_LINE_VALUE_ACTIVE, 
        GPIOD_LINE_VALUE_INACTIVE,
        GPIOD_LINE_VALUE_ACTIVE
    };
    enum gpiod_line_value left_gpio_line_values[] = {
        GPIOD_LINE_VALUE_ACTIVE, 
        GPIOD_LINE_VALUE_INACTIVE, 
        GPIOD_LINE_VALUE_INACTIVE,
        GPIOD_LINE_VALUE_ACTIVE
    };
    enum gpiod_line_value stop_gpio_line_values[] = {
        GPIOD_LINE_VALUE_ACTIVE,
        GPIOD_LINE_VALUE_ACTIVE,
        GPIOD_LINE_VALUE_ACTIVE,
        GPIOD_LINE_VALUE_ACTIVE
    };

    /* Loop */
    while(is_running_flag) {
        if (msgrcv(motor_queue_id, &received_data, sizeof(char), MSG_TYPE_MOTOR_CONTROL, 0) == -1) {
            perror("[motor thread] msgrcv 실패");
            continue;
        }

        switch(received_data.direction) {
            case 'F':
                // if(motor_forward_blocked_flag == CLEAR) {
                //     gpiod_line_request_set_values(motor_request, forward_gpio_line_values);
                //     printf("[motor thread] Forward\n");
                // } 
                // else if (motor_forward_blocked_flag == SET) {
                //     printf("[motor thread] Forward blocked by sonar sensor\n");
                // }
                // break;
            case 'B':
                gpiod_line_request_set_values(motor_request, backward_gpio_line_values);
                printf("[motor thread] Backward\n");
                break;
            case 'R':
                gpiod_line_request_set_values(motor_request, right_gpio_line_values);
                printf("[motor thread] Right\n");
                break;
            case 'L':
                gpiod_line_request_set_values(motor_request, left_gpio_line_values);
                printf("[motor thread] Left\n");
                break;
            case 'S':
            default:
                gpiod_line_request_set_values(motor_request, stop_gpio_line_values);
                printf("[motor thread] Stop Command\n");
                break;
        }
    }

    /* Cleanup */
    gpiod_line_request_release(motor_request);

    return NULL;
}