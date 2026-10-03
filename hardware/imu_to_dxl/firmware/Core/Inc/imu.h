#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* All rates match: 120 Hz gives more than two sensor updates per 50 Hz host tick.
 * These constants are also used by host-side tests. No mount rotation is applied:
 * upstream duck-control already rotates sensor axes into the trunk frame. */
#define IMU_SAMPLE_RATE_HZ       120u
#define IMU_STALE_MS             100u
#define IMU_RECOVERY_MS         1000u
#define IMU_STARTUP_TIMEOUT_MS  1500u
#define IMU_FIFO_BUDGET           16u

typedef enum {
    IMU_ERROR_NONE = 0,
    IMU_ERROR_STARTING = 1,
    IMU_ERROR_SPI = 2,
    IMU_ERROR_ID = 3,
    IMU_ERROR_RESET_TIMEOUT = 4,
    IMU_ERROR_CONFIG = 5,
    IMU_ERROR_STALE = 6,
    IMU_ERROR_FIFO = 7,
    IMU_ERROR_QUAT = 8
} imu_error_t;

typedef struct {
    uint8_t ready;
    uint8_t who_am_i;
    uint8_t configured;
    uint8_t error;
    int16_t gyro[3];          /* Sensor-frame counts; +/-500 dps, 17.5 mdps/LSB. */
    int16_t raw_accel[3];     /* Sensor-frame counts; +/-4 g, 0.122 mg/LSB. */
    uint16_t quat_fp16[3];    /* SFLP x,y,z; host reconstructs nonnegative w. */
    uint32_t sample_count;    /* Accepted SFLP records, not number of host reads. */
    uint32_t gyro_count;
    uint32_t accel_count;
    uint32_t last_gyro_ms;
    uint32_t last_quat_ms;
    uint32_t spi_errors;
    uint32_t fifo_overflows;
    uint32_t invalid_quaternions;
    uint32_t recovery_count;
} imu_snapshot_t;

/* Non-blocking lifecycle. Call poll regularly in the foreground (ideally <1 ms).
 * Init clears diagnostics; automatic recovery preserves lifetime counters.
 * Snapshot is for foreground callers only; do not call in an ISR. It checks
 * board_millis even when poll has been postponed, masking stale live data while
 * preserving configured and counters. Only poll changes the recovery state.
 * Unready output has zero gyro/quaternion bytes (upstream's not-ready marker).
 * Raw accelerometer and counters remain available for fault diagnosis. */
void imu_init(uint32_t now_ms);
void imu_poll(uint32_t now_ms);
void imu_get_snapshot(imu_snapshot_t *out);
void imu_request_reinit(void);

#ifdef __cplusplus
}
#endif
#endif
