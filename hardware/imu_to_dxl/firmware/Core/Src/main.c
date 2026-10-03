#include "board.h"
#include "imu.h"
#include "protocol.h"
#include "control_table.h"
#include <stdio.h>

static proto_context protocol;
static void send_packet(void *user, const uint8_t *data, uint16_t length)
{
    proto_context *context = (proto_context *)user;
    if (board_bus_send(data, length) != 0) ++context->cancelled_replies;
}

static void print_status(void)
{
    imu_snapshot_t s;
    char text[230];
    imu_get_snapshot(&s);
    (void)snprintf(text, sizeof(text),
        "t=%lu ready=%u id=0x%02X err=%u n=%lu gyro=%d,%d,%d q16=%04X,%04X,%04X spierr=%lu fifo=%lu uart=%lu crc=%lu\r\n",
        (unsigned long)board_millis(), (unsigned)s.ready, (unsigned)s.who_am_i, (unsigned)s.error,
        (unsigned long)s.sample_count, (int)s.gyro[0], (int)s.gyro[1], (int)s.gyro[2],
        (unsigned)s.quat_fp16[0], (unsigned)s.quat_fp16[1], (unsigned)s.quat_fp16[2],
        (unsigned long)s.spi_errors, (unsigned long)s.fifo_overflows,
        (unsigned long)board_diagnostics.uart_errors, (unsigned long)protocol.crc_errors);
    board_log(text);
}

int main(void)
{
    uint32_t last_log = 0u;
    uint8_t periodic_log = 1u;
    board_init();
    proto_init(&protocol, control_table_read, send_packet, &protocol);
    imu_init(board_millis());
    board_log("\r\n" BOARD_FIRMWARE_STRING " | STM32G031F8 | Feetech ID=200 1000000 8N1 block@56\r\n");
    board_log("LOG=115200 8N1; ?:help s:status l:toggle logs r:restart IMU\r\n");
    for (;;) {
        uint8_t byte;
        uint32_t received_us;
        if (board_bus_take_fault()) proto_reset_receiver(&protocol);
        while (board_bus_pop(&byte, &received_us)) {
            proto_feed(&protocol, byte, received_us);
        }
        proto_poll(&protocol, board_micros());
        if (protocol.reboot_requested) {
            board_reboot();               /* REBOOT 0x08: no reply, back within 800 ms (sec.8). */
        }
        /* Do not begin sensor/log work while an instruction is being received
         * or a synchronized reply is queued. Protocol traffic has priority. */
        if (protocol.rx_length == 0u && protocol.pending == 0u) {
            imu_poll(board_millis());
            int command = board_log_getchar();
            if (command == 'r' || command == 'R') { imu_request_reinit(); board_log("IMU restart requested\r\n"); }
            else if (command == 'l' || command == 'L') { periodic_log ^= 1u; board_log(periodic_log ? "logs on\r\n" : "logs off\r\n"); }
            else if (command == '?' || command == 'h') board_log("s=status, l=toggle periodic logs, r=reinitialize IMU\r\n");
            if (command == 's' || command == 'S' ||
                (periodic_log && (uint32_t)(board_millis() - last_log) >= 1000u)) {
                last_log = board_millis(); print_status();
            }
        }
        board_watchdog_feed();
    }
}
