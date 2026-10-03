#ifndef BOARD_H
#define BOARD_H
#include <stdint.h>
#include "board_config.h"

typedef struct {
    volatile uint32_t rx_bytes, rx_overflows, uart_errors, tx_packets;
    volatile uint32_t tx_timeouts, spi_timeouts, log_dropped, tx_busy_drops;
    uint32_t reset_flags, clock_hz;
    uint8_t clock_status; /* 0 HSI PLL, 1 HSE PLL, 2 HSE failed/HSI PLL,
                            3 PLL failed/HSI16, 4 clock switch fallback */
} board_diagnostics_t;
extern board_diagnostics_t board_diagnostics;

void board_init(void);
uint32_t board_millis(void);
uint32_t board_micros(void);
void board_delay_ms(uint32_t ms);
void board_watchdog_feed(void);
void board_reboot(void);
int board_spi_read(uint8_t reg, uint8_t *data, uint16_t len);
int board_spi_write(uint8_t reg, const uint8_t *data, uint16_t len);
int board_bus_pop(uint8_t *byte, uint32_t *received_us);
int board_bus_take_fault(void);
int board_bus_send(const uint8_t *data, uint16_t len);
void board_log(const char *text);
int board_log_getchar(void);
#endif
