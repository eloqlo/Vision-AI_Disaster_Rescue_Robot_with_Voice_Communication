#define SET     1
#define CLEAR   0
#define SPI_DATA_SIZE 7

/* Initialization Functions */
/**
 * @brief Sonar, SPI 인터럽트를 위한 GPIO 초기화 함수.
 * @param chip_path Path to the GPIO chip device (e.g., "/dev/gpiochip4").
 * @return Pointer to the line request struct on success, NULL on failure.
 */
struct gpiod_line_request* initialize_sensor_gpio(const char *chip_path);

/**
 * @brief SPI 인터페이스 초기화 함수.
 * @param None
 * @return EXIT_SUCCESS on success, EXIT_FAILURE on failure.
 */
int initialize_spi();

/**
 * @brief SPI 인터럽트 콜백 함수.
 * @details SPI 버퍼에서 데이터를 가져와 buffer에 저장.
 * @param None
 * @return None
 */
void fetch_sensor_data(uint8_t* buffer);

/**
 * @brief   SPI 인터럽트 콜백 함수.
 * @details 읽어온 Sensor 데이터를 JSON 형태로 가공해 TCP 소켓으로 전송.
 * @param   spi_buf SPI로 수신한 센서 데이터 버퍼주소
 * @param   socket_fd 센서 데이터를 전송할 TCP 소켓 파일 디스크립터
 * @return  EXIT_SUCCESS on success, EXIT_FAILURE on failure.
 */
int transmit_sensor_data(uint8_t *spi_buf, int socket_fd);


/* Cleanup */
/**
 * @brief GPIO 및 SPI 리소스 정리 함수.
 * @param None
 * @return None
 */
int cleanup_spi();