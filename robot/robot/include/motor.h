#ifndef MOTOR_H
#define MOTOR_H

#include <gpiod.h>

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


/**
 * @brief   모터 제어를 위한 GPIO 초기화 함수.
 * @param   chip_path GPIO 칩 디바이스 경로 (예: "/dev/gpiochip0")
 * @return  라인 요청 구조체 포인터 (성공 시), NULL (실패 시)
 */
struct gpiod_line_request* initialize_motor_gpio(const char *chip_path);

#endif // MOTOR_H