#define VERSION "1.0"

#define SET     1
#define CLEAR   0


/* Initialization Functions */
/**
 * @brief Sonar, SPI 인터럽트를 위한 GPIO 초기화 함수.
 * @param chip_path Path to the GPIO chip device (e.g., "/dev/gpiochip4").
 * @return Pointer to the line request struct on success, NULL on failure.
 */
struct gpiod_line_request* initialize_gpio(const char *chip_path);

/**
 * @brief SPI 인터페이스 초기화 함수.
 * @param None
 * @return EXIT_SUCCESS on success, EXIT_FAILURE on failure.
 */
int initialize_spi();

/**
 * @brief 서버 초기화 함수(소켓 통신 준비).
 * @param None
 * @return EXIT_SUCCESS on success, EXIT_FAILURE on failure.
 */
int initialize_server();



/* Callback Functions */
/**
 * @brief 초음파 센서 인터럽트 콜백 함수.
 * @param polarity 인터럽트의 극성 (Rising Edge: 1, Falling Edge: 0).
 * @return None
 */
void sonar_irq_callback(int polarity);

/**
 * @brief SPI 인터럽트 콜백 함수.
 * @param None
 * @return None
 */
void spi_irq_callback();



/* Cleanup */
/**
 * @brief GPIO 및 SPI 리소스 정리 함수.
 * @param None
 * @return None
 */
int cleanup_spi();