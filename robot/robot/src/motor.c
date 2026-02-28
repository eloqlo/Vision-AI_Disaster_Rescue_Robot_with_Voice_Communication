#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <gpiod.h>

#define SET     1
#define CLEAR   0

#define IN1   22
#define IN2   27
#define IN3   23
#define IN4   24



struct gpiod_line_request* initialize_motor_gpio(const char *chip_path) {
    struct gpiod_chip *chip;
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *line_cfg;
    struct gpiod_request_config *req_cfg;
    struct gpiod_line_request *request;

    uint32_t offsets[] = {IN1, IN2, IN3, IN4};
    size_t num_lines = sizeof(offsets) / sizeof(offsets[0]);

    chip = gpiod_chip_open(chip_path);
    if (!chip) {
        perror("Failed to open GPIO chip");
        return NULL;
    }
    line_cfg = gpiod_line_config_new();
    if (!line_cfg) {
        perror("Failed to create line config");
        gpiod_chip_close(chip);
        return NULL;
    }
    settings = gpiod_line_settings_new();
    if (!settings) {
        perror("Failed to create line settings");
        gpiod_line_config_free(line_cfg);
        gpiod_chip_close(chip);
        return NULL;
    }
    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    if (gpiod_line_config_add_line_settings(line_cfg, offsets, num_lines, settings) < 0) {
        perror("Failed to add line settings");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_chip_close(chip);
        return NULL;
    }
    req_cfg = gpiod_request_config_new();
    if (!req_cfg) {
        perror("Failed to create request config");
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(line_cfg);
        gpiod_chip_close(chip);
        return NULL;
    }
    gpiod_request_config_set_consumer(req_cfg, "Motor_Driver_Control");
    request = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!request) {
        perror("Failed to request lines");
    }

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    gpiod_chip_close(chip);

    return request;
}