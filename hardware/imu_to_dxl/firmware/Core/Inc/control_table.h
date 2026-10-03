#ifndef CONTROL_TABLE_H
#define CONTROL_TABLE_H
#include <stdint.h>
#include "protocol.h"
#include "imu.h"

/* Register map for the Feetech protocol, see the bus protocol spec sec.3/sec.4/sec.9 and README. */
#define CT_BLOCK_ADDRESS            56u
#define CT_BLOCK_LENGTH             15u
#define CT_STATUS_FUSION_NOT_READY  0x01u  /* BIT0: SFLP not ready, quaternion is all zero. */
#define CT_STATUS_IMU_COMM_FAIL     0x02u  /* BIT1: SPI / reset / config / stale / FIFO fault. */
#define CT_STATUS_SELF_TEST_FAIL    0x04u  /* BIT2: WHO_AM_I mismatch. */
#define CT_STATUS_READER_TOO_SLOW   0x08u  /* BIT3: > CT_SLOW_READER_SAMPLES refreshes since last read. */
/* 120 Hz samples read at 50 Hz give 2-3 new blocks per read; more than 4
 * means the controller reads slower than about 30 Hz. */
#define CT_SLOW_READER_SAMPLES      4u

uint8_t control_table_read(void *user, uint16_t address, uint16_t length, uint8_t *out);
/* Exposed for host tests. */
uint8_t control_table_status(const imu_snapshot_t *s, uint32_t new_samples);
void control_table_block(const imu_snapshot_t *s, uint8_t status, uint8_t out[CT_BLOCK_LENGTH]);
#endif
