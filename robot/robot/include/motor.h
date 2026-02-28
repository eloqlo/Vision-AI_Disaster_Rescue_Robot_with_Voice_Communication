#define SET     1
#define CLEAR   0

#define IN1   22
#define IN2   27
#define IN3   23
#define IN4   24


/**
 * @brief   모터 제어를 위한 GPIO 초기화 함수.
 * @param   chip_path GPIO 칩 디바이스 경로 (예: "/dev/gpiochip0")
 * @return  라인 요청 구조체 포인터 (성공 시), NULL (실패 시)
 */
struct gpiod_line_request* initialize_motor_gpio(const char *chip_path);