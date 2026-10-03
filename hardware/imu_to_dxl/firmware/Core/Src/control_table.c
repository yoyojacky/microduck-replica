#include "control_table.h"
#include "board.h"
#include "imu.h"
#include <string.h>

/* Register area 9..255 (0..8 are answered by protocol.c), the bus protocol spec sec.9.
 * 56..70: the 15-byte block the controller Sync Reads every tick (sec.3).
 * 124..247: OUR diagnostic extension, not part of the controller interface. */

static uint32_t samples_at_block_read;
static uint8_t block_read_before;

static void put16(uint8_t *p, uint16_t value) { p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }
static void put32(uint8_t *p, uint32_t value)
{ for (unsigned i = 0; i < 4u; ++i) p[i] = (uint8_t)(value >> (8u * i)); }

uint8_t control_table_status(const imu_snapshot_t *s, uint32_t new_samples)
{
    uint8_t status = 0u;
    int quat_zero = s->quat_fp16[0] == 0u && s->quat_fp16[1] == 0u && s->quat_fp16[2] == 0u;
    if (!s->ready || quat_zero) status |= CT_STATUS_FUSION_NOT_READY;
    if (s->error == IMU_ERROR_SPI || s->error == IMU_ERROR_RESET_TIMEOUT ||
        s->error == IMU_ERROR_CONFIG || s->error == IMU_ERROR_STALE ||
        s->error == IMU_ERROR_FIFO) status |= CT_STATUS_IMU_COMM_FAIL;
    if (s->error == IMU_ERROR_ID) status |= CT_STATUS_SELF_TEST_FAIL;
    if (new_samples > CT_SLOW_READER_SAMPLES) status |= CT_STATUS_READER_TOO_SLOW;
    return status;
}

void control_table_block(const imu_snapshot_t *s, uint8_t status, uint8_t out[CT_BLOCK_LENGTH])
{
    /* Raw sensor axes, no remapping or sign change (sec.3): the controller applies
     * the mount rotation. Unready snapshots already carry zero gyro/quaternion. */
    for (unsigned i = 0; i < 3u; ++i) {
        put16(&out[i * 2u], (uint16_t)s->gyro[i]);
        put16(&out[6u + i * 2u], s->quat_fp16[i]);
    }
    out[12] = (uint8_t)s->sample_count;   /* Wraps 255 -> 0. */
    out[13] = status;
    out[14] = 0u;                         /* Reserved, must be 0. */
}

uint8_t control_table_read(void *user, uint16_t address, uint16_t length, uint8_t *out)
{
    proto_context *protocol = (proto_context *)user;
    uint8_t table[256] = {0};
    imu_snapshot_t s;
    if (!out || !length || address >= 256u || length > 256u - address) return PROTO_ERR_RANGE;
    /* Both acquisition and this callback run in main, so this is an untorn snapshot. */
    imu_get_snapshot(&s);
    if (address < CT_BLOCK_ADDRESS + CT_BLOCK_LENGTH && address + length > CT_BLOCK_ADDRESS) {
        /* BIT3 counts refreshes since the previous read that included the block.
         * A smaller count means imu_init cleared it (not a 414-day wrap): start over. */
        uint32_t fresh = (block_read_before && s.sample_count >= samples_at_block_read)
            ? s.sample_count - samples_at_block_read : 0u;
        control_table_block(&s, control_table_status(&s, fresh), &table[CT_BLOCK_ADDRESS]);
        samples_at_block_read = s.sample_count;
        block_read_before = 1u;
    }
    for (unsigned i = 0; i < 3u; ++i) {
        put16(&table[124u + i * 2u], (uint16_t)s.gyro[i]);
        put16(&table[130u + i * 2u], s.quat_fp16[i]);
        put16(&table[136u + i * 2u], (uint16_t)s.raw_accel[i]);
    }
    table[142] = (uint8_t)s.sample_count;
    table[143] = (s.ready ? 1u : 0u) | (s.configured ? 2u : 0u) | (s.error ? 4u : 0u);
    table[144] = s.who_am_i; table[145] = s.error;
    table[146] = board_diagnostics.clock_status;
    table[147] = BOARD_ENABLE_WATCHDOG;
    put32(&table[148], s.sample_count);
    put32(&table[152], s.gyro_count);
    put32(&table[156], s.accel_count);
    put32(&table[160], board_millis());
    put32(&table[164], board_millis() - s.last_gyro_ms);
    put32(&table[168], board_millis() - s.last_quat_ms);
    put32(&table[172], s.spi_errors);
    put32(&table[176], s.fifo_overflows);
    put32(&table[180], s.invalid_quaternions);
    put32(&table[184], s.recovery_count);
    put32(&table[188], board_diagnostics.rx_overflows);
    put32(&table[192], board_diagnostics.uart_errors);
    put32(&table[196], protocol->crc_errors);
    put32(&table[200], protocol->malformed_packets);
    put32(&table[204], protocol->receive_timeouts);
    put32(&table[208], protocol->rx_packets);
    put32(&table[212], board_diagnostics.tx_packets);
    put32(&table[216], protocol->cancelled_replies);
    put32(&table[220], board_diagnostics.reset_flags);
    put32(&table[224], board_diagnostics.clock_hz);
    put32(&table[228], board_diagnostics.tx_timeouts);
    put32(&table[232], board_diagnostics.log_dropped);
    table[236] = 2u; /* Diagnostic map revision: 2 = Feetech protocol, block at 56. */
    put32(&table[240], protocol->skipped_slots);
    put32(&table[244], board_diagnostics.tx_busy_drops);
    memcpy(out, &table[address], length);
    return PROTO_OK;
}
