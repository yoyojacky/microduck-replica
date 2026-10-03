#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* a1f4979c / 2026-09-08 imu_to_dxl, STM32G031F8P6 TSSOP20.
 * All build-time board options live here. No changes to option bytes or Flash.
 * First power-up uses HSI16 -> PLL -> 64 MHz, independent of X1 population. */
#define BOARD_USE_HSE_BYPASS       0
#define BOARD_HSE_HZ               16000000UL
#define BOARD_ENABLE_WATCHDOG      1
#define BOARD_LOG_UART_SWAP        0
#define BOARD_LOG_BAUD             115200UL
#define BOARD_BUS_BAUD             1000000UL
#define BOARD_BUS_RX_SIZE          256u /* power of two, timestamp per byte */
#define BOARD_LOG_TX_SIZE          512u
#define BOARD_RX_EN_PIN            7u   /* PB7; PB8 shares package pin: ANALOG */
#define BOARD_SPI_BR               4u   /* PCLK/32: 2 MHz at 64 MHz */
#define BOARD_SPI_MODE             3u   /* LSM6DSV16X supports mode 0 and 3 */
#define BOARD_FIRMWARE_STRING      "imu_to_dxl 0.2.0"

#if BOARD_HSE_HZ != 16000000UL
#error Update PLL settings before using an oscillator other than 16 MHz
#endif
#if (BOARD_BUS_RX_SIZE & (BOARD_BUS_RX_SIZE - 1u)) != 0
#error BOARD_BUS_RX_SIZE must be a power of two
#endif
#if BOARD_SPI_MODE != 0 && BOARD_SPI_MODE != 3
#error BOARD_SPI_MODE must be 0 or 3
#endif
#endif
