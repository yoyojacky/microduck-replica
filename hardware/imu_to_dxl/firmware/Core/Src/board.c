#include "board.h"
#include "stm32g0xx.h"

board_diagnostics_t board_diagnostics;
static volatile uint32_t tick_ms;
static volatile uint8_t bus_rx[BOARD_BUS_RX_SIZE];
static volatile uint32_t bus_rx_time[BOARD_BUS_RX_SIZE];
static volatile uint16_t bus_head, bus_tail;
static volatile uint8_t bus_fault;
static volatile uint8_t log_tx[BOARD_LOG_TX_SIZE];
static volatile uint16_t log_head, log_tail;
static volatile uint8_t log_command;

static uint32_t lock_irq(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void unlock_irq(uint32_t p) { __set_PRIMASK(p); }
static int wait_bits(volatile uint32_t *reg, uint32_t mask, uint32_t want)
{
    uint32_t n = 2000000UL; /* Bounded even before SysTick is available. */
    while ((*reg & mask) != want) if (--n == 0u) return -1;
    return 0;
}
static void pin_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{ port->MODER = (port->MODER & ~(3UL << (pin * 2u))) | (mode << (pin * 2u)); }
static void pin_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
    uint32_t i = pin >> 3, shift = (pin & 7u) * 4u;
    port->AFR[i] = (port->AFR[i] & ~(15UL << shift)) | (af << shift);
    pin_mode(port, pin, 2u);
}

static void clock_init(void)
{
    uint32_t source = RCC_PLLCFGR_PLLSRC_HSI;
    RCC->CR |= RCC_CR_HSION;
    (void)wait_bits(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY);
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSISYS;
    (void)wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSISYS);
    RCC->CR &= ~RCC_CR_HSIDIV;
    RCC->APBENR1 |= RCC_APBENR1_PWREN;
    (void)RCC->APBENR1;
    PWR->CR1 = (PWR->CR1 & ~PWR_CR1_VOS) | PWR_CR1_VOS_0;
    (void)wait_bits(&PWR->SR2, PWR_SR2_VOSF, 0u);
    /* At VOS range 1, 64 MHz requires TWO wait states (encoded value 2). */
    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_1;
    (void)wait_bits(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_1);
    RCC->CR &= ~RCC_CR_PLLON;
    (void)wait_bits(&RCC->CR, RCC_CR_PLLRDY, 0u);
#if BOARD_USE_HSE_BYPASS
    /* TSSOP20: active clock into package pin 2/PC14. No crystal OSC_OUT. */
    RCC->CR |= RCC_CR_HSEBYP;
    RCC->CR |= RCC_CR_HSEON;
    if (wait_bits(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY) == 0) {
        source = RCC_PLLCFGR_PLLSRC_HSE;
        board_diagnostics.clock_status = 1u;
    } else {
        RCC->CR &= ~RCC_CR_HSEON;
        board_diagnostics.clock_status = 2u;
    }
#endif
    /* G0 PLL R divider is (PLLR + 1), unlike some other STM32 families.
       16 MHz / M=1 * N=8 / R=2 = 64 MHz; encoded PLLR=1. */
    RCC->PLLCFGR = source | (8UL << RCC_PLLCFGR_PLLN_Pos)
        | (1UL << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN;
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE);
    RCC->CR |= RCC_CR_PLLON;
    if (wait_bits(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY) == 0) {
        RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLLRCLK;
        if (wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLLRCLK) != 0) {
            RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSISYS;
            (void)wait_bits(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSISYS);
            board_diagnostics.clock_status = 4u;
        }
    } else board_diagnostics.clock_status = 3u;
    SystemCoreClockUpdate();
    board_diagnostics.clock_hz = SystemCoreClock;
}

static void spi_init(void)
{
    RCC->APBRSTR2 |= RCC_APBRSTR2_SPI1RST;
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_SPI1RST;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI
        | (BOARD_SPI_BR << SPI_CR1_BR_Pos)
#if BOARD_SPI_MODE == 3
        | SPI_CR1_CPOL | SPI_CR1_CPHA
#endif
        ;
    SPI1->CR2 = (7UL << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;
}

void board_init(void)
{
    board_diagnostics.reset_flags = RCC->CSR;
    RCC->CSR |= RCC_CSR_RMVF;
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
    (void)RCC->IOPENR;
    /* Preload safe levels BEFORE switching the pins to outputs. */
    GPIOA->BSRR = (1UL << 1) | (1UL << 4); /* DE# OFF, CS# HIGH */
    GPIOB->BSRR = 1UL << BOARD_RX_EN_PIN; /* receiver ON */
    pin_mode(GPIOA, 1u, 1u);
    pin_mode(GPIOA, 4u, 1u);
    pin_mode(GPIOB, 8u, 3u); /* tied PB8 pad must never fight PB7 */
    pin_mode(GPIOB, BOARD_RX_EN_PIN, 1u);
    clock_init();
    RCC->APBENR1 |= RCC_APBENR1_TIM2EN | RCC_APBENR1_USART2EN;
    RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN | RCC_APBENR2_SPI1EN | RCC_APBENR2_USART1EN;
    (void)RCC->APBENR2;
    TIM2->PSC = (SystemCoreClock / 1000000UL) - 1u;
    TIM2->ARR = 0xFFFFFFFFUL;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0;
    TIM2->CR1 = TIM_CR1_CEN;
    (void)SysTick_Config(SystemCoreClock / 1000u);
    NVIC_SetPriority(SysTick_IRQn, 2u);
    /* SPI1 AF0: PA5=SCK, PA6=MISO, PA7=MOSI. */
    pin_af(GPIOA, 5u, 0u); pin_af(GPIOA, 6u, 0u); pin_af(GPIOA, 7u, 0u);
    spi_init();
    /* USART2 AF1, external single-wire buffers: do NOT set UART HDSEL. */
    GPIOA->PUPDR = (GPIOA->PUPDR & ~(3UL << 6)) | (1UL << 6); /* PA3 pull-up */
    pin_af(GPIOA, 2u, 1u); pin_af(GPIOA, 3u, 1u);
    USART2->CR1 = 0;
    USART2->BRR = (SystemCoreClock + BOARD_BUS_BAUD / 2u) / BOARD_BUS_BAUD;
    USART2->CR2 = 0; USART2->CR3 = USART_CR3_EIE;
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    USART2->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART2_IRQn, 0u); NVIC_EnableIRQ(USART2_IRQn);
    /* TSSOP20 package pins 16/17: route PA9/10 onto PA11/12 pads. */
    SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP | SYSCFG_CFGR1_PA12_RMP;
    pin_af(GPIOA, 9u, 1u); pin_af(GPIOA, 10u, 1u);
    USART1->CR1 = 0;
    USART1->BRR = (SystemCoreClock + BOARD_LOG_BAUD / 2u) / BOARD_LOG_BAUD;
    USART1->CR2 = BOARD_LOG_UART_SWAP ? USART_CR2_SWAP : 0u;
    USART1->CR3 = 0;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE;
    NVIC_SetPriority(USART1_IRQn, 2u); NVIC_EnableIRQ(USART1_IRQn);
#if BOARD_ENABLE_WATCHDOG
    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_IWDG_STOP;
    IWDG->KR = 0xCCCCu;
    IWDG->KR = 0x5555u;
    IWDG->PR = 4u; /* LSI / 64; approximately 2 seconds, LSI tolerance applies. */
    IWDG->RLR = 999u;
    (void)wait_bits(&IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU, 0u);
    IWDG->KR = 0xAAAAu;
#endif
}

void SysTick_Handler(void) { ++tick_ms; }
uint32_t board_millis(void) { return tick_ms; }
void board_reboot(void) { NVIC_SystemReset(); }
uint32_t board_micros(void) { return TIM2->CNT; }
void board_delay_ms(uint32_t ms)
{ uint32_t start = tick_ms; while ((uint32_t)(tick_ms - start) < ms) { __NOP(); } }
void board_watchdog_feed(void)
{
#if BOARD_ENABLE_WATCHDOG
    IWDG->KR = 0xAAAAu;
#endif
}

static int spi_byte(uint8_t out, uint8_t *in)
{
    uint32_t start = board_micros();
    while ((SPI1->SR & SPI_SR_TXE) == 0u)
        if ((uint32_t)(board_micros() - start) > 100u) return -1;
    *(__IO uint8_t *)&SPI1->DR = out;
    while ((SPI1->SR & SPI_SR_RXNE) == 0u)
        if ((uint32_t)(board_micros() - start) > 100u) return -1;
    *in = *(__IO uint8_t *)&SPI1->DR;
    return 0;
}
static int spi_transfer(uint8_t reg, uint8_t *read, const uint8_t *write, uint16_t len)
{
    uint8_t value;
    int result = 0;
    GPIOA->BRR = 1UL << 4;
    __DSB();
    if (spi_byte(read ? (uint8_t)(reg | 0x80u) : (uint8_t)(reg & 0x7Fu), &value) != 0) result = -1;
    for (uint16_t i = 0; result == 0 && i < len; ++i) {
        if (spi_byte(read ? 0u : write[i], &value) != 0) result = -1;
        else if (read) read[i] = value;
    }
    uint32_t start = board_micros();
    while ((SPI1->SR & SPI_SR_BSY) != 0u) {
        if ((uint32_t)(board_micros() - start) > 100u) { result = -1; break; }
    }
    GPIOA->BSRR = 1UL << 4;
    if (result != 0) { ++board_diagnostics.spi_timeouts; spi_init(); }
    return result;
}
int board_spi_read(uint8_t reg, uint8_t *data, uint16_t len)
{ return data && len ? spi_transfer(reg, data, 0, len) : -1; }
int board_spi_write(uint8_t reg, const uint8_t *data, uint16_t len)
{ return data && len ? spi_transfer(reg, 0, data, len) : -1; }

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->ISR;
    if (status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        ++board_diagnostics.uart_errors;
        /* Only an overrun loses bytes: drop the partial frame. A noise / framing
         * flag keeps its byte in RDR; the frame checksum rejects it if it is
         * wrong. Treating those as faults would also flush the good bytes of
         * the same tick and silence the board for a whole Sync Read. */
        if (status & USART_ISR_ORE) bus_fault = 1u;
    }
    if (status & USART_ISR_RXNE_RXFNE) {
        uint8_t byte = (uint8_t)USART2->RDR;
        uint16_t next = (uint16_t)((bus_head + 1u) & (BOARD_BUS_RX_SIZE - 1u));
        ++board_diagnostics.rx_bytes;
        if (next == bus_tail) { ++board_diagnostics.rx_overflows; bus_fault = 1u; }
        else { bus_rx[bus_head] = byte; bus_rx_time[bus_head] = board_micros(); __DMB(); bus_head = next; }
    }
}
int board_bus_pop(uint8_t *byte, uint32_t *received_us)
{
    if (bus_tail == bus_head) return 0;
    *byte = bus_rx[bus_tail]; *received_us = bus_rx_time[bus_tail];
    __DMB(); bus_tail = (uint16_t)((bus_tail + 1u) & (BOARD_BUS_RX_SIZE - 1u));
    return 1;
}
int board_bus_take_fault(void)
{
    uint32_t p = lock_irq(); int fault = bus_fault;
    if (fault) { bus_tail = bus_head; bus_fault = 0; }
    unlock_irq(p); return fault;
}
int board_bus_send(const uint8_t *data, uint16_t len)
{
    if (!data || !len) return -1;
    uint32_t irq = lock_irq();
    NVIC_DisableIRQ(USART2_IRQn);
    /* A new request can arrive after main drained the ring. Recheck with no
     * preemption immediately before enabling the driver; never erase that RX. */
    if (bus_fault || bus_head != bus_tail ||
        (USART2->ISR & (USART_ISR_BUSY | USART_ISR_RXNE_RXFNE | USART_ISR_ORE |
                       USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))) {
        ++board_diagnostics.tx_busy_drops;
        NVIC_EnableIRQ(USART2_IRQn); unlock_irq(irq); return -2;
    }
    GPIOB->BRR = 1UL << BOARD_RX_EN_PIN; /* suppress local echo; PA3 pull-up stays on */
    USART2->ICR = USART_ICR_TCCF;
    GPIOA->BRR = 1UL << 1; /* 1OE# LOW -> drive bus */
    unlock_irq(irq);
    uint32_t start = board_micros();
    uint32_t limit = ((uint32_t)len * 10000000UL) / BOARD_BUS_BAUD + 1000u;
    int result = 0;
    for (uint16_t i = 0; i < len; ++i) {
        while (!(USART2->ISR & USART_ISR_TXE_TXFNF)) {
            if ((uint32_t)(board_micros() - start) > limit) { result = -1; break; }
        }
        if (result) break;
        USART2->TDR = data[i];
    }
    while (!(USART2->ISR & USART_ISR_TC)) {
        if ((uint32_t)(board_micros() - start) > limit) { result = -1; break; }
    }
    GPIOA->BSRR = 1UL << 1; /* only release after final stop bit / timeout */
    while (USART2->ISR & USART_ISR_RXNE_RXFNE) (void)USART2->RDR;
    USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    GPIOB->BSRR = 1UL << BOARD_RX_EN_PIN;
    NVIC_ClearPendingIRQ(USART2_IRQn); NVIC_EnableIRQ(USART2_IRQn);
    if (result) { ++board_diagnostics.tx_timeouts; bus_fault = 1u; }
    else ++board_diagnostics.tx_packets;
    return result;
}

void USART1_IRQHandler(void)
{
    uint32_t status = USART1->ISR;
    if (status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
        USART1->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
    if (status & USART_ISR_RXNE_RXFNE) log_command = (uint8_t)USART1->RDR;
    if ((status & USART_ISR_TXE_TXFNF) && (USART1->CR1 & USART_CR1_TXEIE_TXFNFIE)) {
        if (log_head != log_tail) {
            USART1->TDR = log_tx[log_tail];
            log_tail = (uint16_t)((log_tail + 1u) % BOARD_LOG_TX_SIZE);
        } else USART1->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
    }
}
void board_log(const char *text)
{
    while (*text) {
        uint16_t next = (uint16_t)((log_head + 1u) % BOARD_LOG_TX_SIZE);
        if (next == log_tail) { ++board_diagnostics.log_dropped; break; }
        log_tx[log_head] = (uint8_t)*text++; __DMB(); log_head = next;
    }
    /* ISR can clear TXEIE between a main-thread read and write of CR1. */
    uint32_t p = lock_irq(); USART1->CR1 |= USART_CR1_TXEIE_TXFNFIE; unlock_irq(p);
}
int board_log_getchar(void)
{
    uint32_t p = lock_irq(); int c = log_command; log_command = 0; unlock_irq(p); return c;
}
