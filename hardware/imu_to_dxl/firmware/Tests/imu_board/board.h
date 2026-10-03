#ifndef IMU_TEST_BOARD_H
#define IMU_TEST_BOARD_H
#include <stdint.h>
int32_t board_spi_read(uint8_t reg, uint8_t *data, uint16_t len);
int32_t board_spi_write(uint8_t reg, const uint8_t *data, uint16_t len);
void board_delay_ms(uint32_t ms);
uint32_t board_millis(void);
#endif
