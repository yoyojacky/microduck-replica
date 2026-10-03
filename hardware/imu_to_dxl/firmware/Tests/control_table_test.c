/* Host tests for the register area (Core/Src/control_table.c): the 15-byte
 * block at 56 (bus protocol spec sec.3/sec.4) and the diagnostic extension. */
#include "control_table.h"
#include "imu.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

board_diagnostics_t board_diagnostics;
static imu_snapshot_t sensor;
static unsigned checks;
void imu_get_snapshot(imu_snapshot_t *out) { *out = sensor; }
void imu_request_reinit(void) { }
uint32_t board_millis(void) { return 123456u; }
#define CHECK(x) do { assert(x); ++checks; } while (0)

static void live_sensor(void)
{
    memset(&sensor, 0, sizeof(sensor));
    sensor.ready = sensor.configured = 1;
    sensor.who_am_i = 0x70;
    sensor.gyro[0] = -32768; sensor.gyro[1] = 32767; sensor.gyro[2] = -1;
    sensor.quat_fp16[0] = 0x3800; sensor.quat_fp16[1] = 0xb800; sensor.quat_fp16[2] = 0x8000;
    sensor.sample_count = 0x12345678;
}

int main(void)
{
    proto_context protocol;
    uint8_t out[256] = {0};
    /* gyro x/y/z i16 LE, quat x/y/z fp16 LE, counter, status, reserved */
    const uint8_t expected[15] = {0x00,0x80,0xff,0x7f,0xff,0xff,0x00,0x38,0x00,0xb8,0x00,0x80,
                                  0x78,0x00,0x00};
    memset(&protocol, 0, sizeof(protocol));

    /* Block layout, raw axes, counter is the low byte of sample_count. */
    live_sensor();
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    CHECK(memcmp(out, expected, sizeof(expected)) == 0);

    /* Not ready: gyro/quaternion zero (imu_get_snapshot does that), BIT0 set,
     * reserved stays 0 -> matches the sec.10 "not ready" vector's data field. */
    memset(&sensor, 0, sizeof(sensor));
    sensor.error = IMU_ERROR_STARTING;
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    {
        const uint8_t not_ready[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0x01,0};
        CHECK(memcmp(out, not_ready, 15) == 0);
    }

    /* Status bits. */
    live_sensor();
    CHECK(control_table_status(&sensor, 2u) == 0u);
    sensor.quat_fp16[0] = sensor.quat_fp16[1] = sensor.quat_fp16[2] = 0u;
    CHECK(control_table_status(&sensor, 2u) == CT_STATUS_FUSION_NOT_READY);
    live_sensor();
    sensor.ready = 0; sensor.error = IMU_ERROR_SPI;
    CHECK(control_table_status(&sensor, 2u) == (CT_STATUS_FUSION_NOT_READY | CT_STATUS_IMU_COMM_FAIL));
    sensor.error = IMU_ERROR_FIFO;
    CHECK(control_table_status(&sensor, 2u) & CT_STATUS_IMU_COMM_FAIL);
    sensor.error = IMU_ERROR_ID;
    CHECK(control_table_status(&sensor, 2u) == (CT_STATUS_FUSION_NOT_READY | CT_STATUS_SELF_TEST_FAIL));
    live_sensor();
    CHECK(control_table_status(&sensor, CT_SLOW_READER_SAMPLES) == 0u);
    CHECK(control_table_status(&sensor, CT_SLOW_READER_SAMPLES + 1u) == CT_STATUS_READER_TOO_SLOW);

    /* BIT3 across real reads: 3 new samples between reads is normal, 9 is slow. */
    live_sensor();
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    sensor.sample_count += 3u;
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    CHECK((out[13] & CT_STATUS_READER_TOO_SLOW) == 0u);
    sensor.sample_count += 9u;
    CHECK(control_table_read(&protocol, 60u, 11u, out) == PROTO_OK);   /* Partial read covering 60..70. */
    CHECK((out[9] & CT_STATUS_READER_TOO_SLOW) != 0u);
    /* Reads that miss the block do not reset the BIT3 reference. */
    sensor.sample_count += 2u;
    CHECK(control_table_read(&protocol, 124u, 12u, out) == PROTO_OK);
    sensor.sample_count += 2u;
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    CHECK((out[13] & CT_STATUS_READER_TOO_SLOW) == 0u);                /* 4 since last block read. */

    /* IMU re-init clears sample_count: no false BIT3 on the next read. */
    sensor.sample_count = 5u;
    CHECK(control_table_read(&protocol, CT_BLOCK_ADDRESS, CT_BLOCK_LENGTH, out) == PROTO_OK);
    CHECK((out[13] & CT_STATUS_READER_TOO_SLOW) == 0u);

    /* Diagnostic extension unchanged from map revision 1, plus revision byte 2. */
    live_sensor();
    CHECK(control_table_read(&protocol, 124u, 12u, out) == PROTO_OK);
    CHECK(memcmp(out, expected, 12u) == 0);
    CHECK(control_table_read(&protocol, 142u, 10u, out) == PROTO_OK);
    CHECK(out[0] == 0x78 && out[1] == 3 && out[2] == 0x70);
    CHECK(out[6] == 0x78 && out[7] == 0x56 && out[8] == 0x34 && out[9] == 0x12);
    protocol.skipped_slots = 0x01020304u;
    CHECK(control_table_read(&protocol, 236u, 8u, out) == PROTO_OK);
    CHECK(out[0] == 2u && out[4] == 0x04 && out[7] == 0x01);

    /* Range checks. */
    CHECK(control_table_read(&protocol, 255u, 2u, out) == PROTO_ERR_RANGE);
    CHECK(control_table_read(&protocol, 9u, 0u, out) == PROTO_ERR_RANGE);
    CHECK(control_table_read(&protocol, 0u, 256u, out) == PROTO_OK);
    CHECK(out[56 + 14] == 0u);
    printf("control table: %u checks passed\n", checks);
    return 0;
}
