#include "imu.h"
#include "board.h"
#include "lsm6dsv16x_reg.h"
#include <string.h>

/* Register operations are bounded by board_spi_* timeouts. Configuration is
 * split across foreground iterations so a missing IMU cannot starve the bus. */
typedef enum { STATE_BOOT, STATE_POR_WAIT, STATE_CONFIG, STATE_RUN, STATE_RETRY } state_t;
static stmdev_ctx_t ctx;
static imu_snapshot_t sample;
static state_t state;
static uint32_t state_since;
static uint32_t run_since;
static uint32_t last_poll_ms;
static uint8_t config_step;
static uint8_t have_gyro;
static uint8_t have_quat;
static uint8_t force_reinit;

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    int32_t result;
    (void)handle;
    result = board_spi_read(reg, data, len);
    if (result != 0) { sample.spi_errors++; }
    return result;
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    int32_t result;
    (void)handle;
    result = board_spi_write(reg, data, len);
    if (result != 0) { sample.spi_errors++; }
    return result;
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void clear_live_data(void)
{
    sample.ready = 0u;
    sample.configured = 0u;
    have_gyro = 0u;
    have_quat = 0u;
    memset(sample.gyro, 0, sizeof(sample.gyro));
    memset(sample.quat_fp16, 0, sizeof(sample.quat_fp16));
}

static void fail(imu_error_t error, uint32_t now_ms)
{
    clear_live_data();
    sample.error = (uint8_t)error;
    state = STATE_RETRY;
    state_since = now_ms;
}

static void start_boot(uint32_t now_ms)
{
    clear_live_data();
    sample.error = IMU_ERROR_STARTING;
    sample.who_am_i = 0u;
    config_step = 0u;
    state = STATE_BOOT;
    state_since = now_ms;
}

void imu_init(uint32_t now_ms)
{
    memset(&sample, 0, sizeof(sample));
    memset(&ctx, 0, sizeof(ctx));
    ctx.read_reg = platform_read;
    ctx.write_reg = platform_write;
    ctx.mdelay = board_delay_ms;
    force_reinit = 0u;
    last_poll_ms = now_ms - 1u;
    start_boot(now_ms);
}

void imu_request_reinit(void)
{
    force_reinit = 1u;
}

/* No calibration bias is persisted until actual board characterization exists.
 * SFLP estimates gyro bias internally; zero is only its initial estimate.
 * LPF1/LPF2 remain at ST reset defaults for the first hardware baseline. */
static int32_t configure_one_step(void)
{
    lsm6dsv16x_fifo_sflp_raw_t batch = {0};
    lsm6dsv16x_data_rate_t odr;
    lsm6dsv16x_gy_full_scale_t gy_scale;
    lsm6dsv16x_xl_full_scale_t xl_scale;
    uint8_t enabled;
    int32_t result;
    switch (config_step) {
    case 0: return lsm6dsv16x_block_data_update_set(&ctx, PROPERTY_ENABLE);
    case 1: return lsm6dsv16x_auto_increment_set(&ctx, PROPERTY_ENABLE);
    case 2: return lsm6dsv16x_fifo_mode_set(&ctx, LSM6DSV16X_BYPASS_MODE);
    case 3: return lsm6dsv16x_xl_full_scale_set(&ctx, LSM6DSV16X_4g);
    case 4: return lsm6dsv16x_gy_full_scale_set(&ctx, LSM6DSV16X_500dps);
    case 5: return lsm6dsv16x_fifo_watermark_set(&ctx, 3u);
    case 6: return lsm6dsv16x_fifo_xl_batch_set(&ctx, LSM6DSV16X_XL_BATCHED_AT_120Hz);
    case 7: return lsm6dsv16x_fifo_gy_batch_set(&ctx, LSM6DSV16X_GY_BATCHED_AT_120Hz);
    case 8:
        batch.game_rotation = 1u;
        return lsm6dsv16x_fifo_sflp_batch_set(&ctx, batch);
    case 9: return lsm6dsv16x_sflp_data_rate_set(&ctx, LSM6DSV16X_SFLP_120Hz);
    case 10: return lsm6dsv16x_xl_data_rate_set(&ctx, LSM6DSV16X_ODR_AT_120Hz);
    case 11: return lsm6dsv16x_gy_data_rate_set(&ctx, LSM6DSV16X_ODR_AT_120Hz);
    case 12: return lsm6dsv16x_sflp_game_rotation_set(&ctx, PROPERTY_ENABLE);
    /* Do not call sflp_game_gbias_set: this revision contains unbounded polls
     * for DRDY/ENDOP. No saved bias exists, so retain the POR default estimate. */
    case 13: return lsm6dsv16x_fifo_mode_set(&ctx, LSM6DSV16X_STREAM_MODE);
    case 14:
        result = lsm6dsv16x_xl_data_rate_get(&ctx, &odr);
        return result != 0 ? result : (odr == LSM6DSV16X_ODR_AT_120Hz ? 0 : -1);
    case 15:
        result = lsm6dsv16x_gy_data_rate_get(&ctx, &odr);
        return result != 0 ? result : (odr == LSM6DSV16X_ODR_AT_120Hz ? 0 : -1);
    case 16:
        result = lsm6dsv16x_gy_full_scale_get(&ctx, &gy_scale);
        return result != 0 ? result : (gy_scale == LSM6DSV16X_500dps ? 0 : -1);
    case 17:
        result = lsm6dsv16x_xl_full_scale_get(&ctx, &xl_scale);
        return result != 0 ? result : (xl_scale == LSM6DSV16X_4g ? 0 : -1);
    case 18:
        result = lsm6dsv16x_sflp_game_rotation_get(&ctx, &enabled);
        return result != 0 ? result : (enabled != 0u ? 0 : -1);
    default: return -1;
    }
}

/* Validate, but keep accepted sensor binary16 values unchanged. ST reconstructs
 * w >= 0, normalizing xyz if rounding makes sumsq > 1. The upstream host accepts
 * sumsq <= 1.02 then normalizes its reconstructed quaternion, so use that same
 * bound here. Never apply a second mounting rotation or an independent sign flip.
 * A live identity sample has xyz = +0,+0,+0. Encode x as IEEE -0 to distinguish
 * this valid identity from the all-zero not-ready marker without altering value. */
static uint8_t accept_quaternion(const uint8_t *data)
{
    uint16_t packed[3];
    float sumsq = 0.0f;
    unsigned i;
    for (i = 0u; i < 3u; ++i) {
        uint32_t bits;
        float value;
        packed[i] = le16(&data[i * 2u]);
        if ((packed[i] & 0x7c00u) == 0x7c00u) { return 0u; }
        bits = lsm6dsv16x_from_f16_to_f32(packed[i]);
        memcpy(&value, &bits, sizeof(value));
        sumsq += value * value;
    }
    if (!(sumsq <= 1.02f)) { return 0u; }
    if ((packed[0] | packed[1] | packed[2]) == 0u) { packed[0] = 0x8000u; }
    memcpy(sample.quat_fp16, packed, sizeof(packed));
    return 1u;
}

static void poll_fifo(uint32_t now_ms)
{
    lsm6dsv16x_fifo_status_t status = {0};
    uint16_t count;
    unsigned i;
    if (lsm6dsv16x_fifo_status_get(&ctx, &status) != 0) {
        fail(IMU_ERROR_SPI, now_ms);
        return;
    }
    if (status.fifo_ovr != 0u || status.fifo_full != 0u) {
        sample.fifo_overflows++;
        fail(IMU_ERROR_FIFO, now_ms);
        return;
    }
    count = status.fifo_level;
    if (count > IMU_FIFO_BUDGET) { count = IMU_FIFO_BUDGET; }
    while (count-- != 0u) {
        lsm6dsv16x_fifo_out_raw_t item = {0};
        if (lsm6dsv16x_fifo_out_raw_get(&ctx, &item) != 0) {
            fail(IMU_ERROR_SPI, now_ms);
            return;
        }
        switch (item.tag) {
        case LSM6DSV16X_GY_NC_TAG:
            for (i = 0u; i < 3u; ++i) { sample.gyro[i] = (int16_t)le16(&item.data[i * 2u]); }
            sample.gyro_count++;
            sample.last_gyro_ms = now_ms;
            have_gyro = 1u;
            break;
        case LSM6DSV16X_XL_NC_TAG:
            for (i = 0u; i < 3u; ++i) { sample.raw_accel[i] = (int16_t)le16(&item.data[i * 2u]); }
            sample.accel_count++;
            break;
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            if (accept_quaternion(item.data) != 0u) {
                sample.sample_count++;
                sample.last_quat_ms = now_ms;
                have_quat = 1u;
            } else {
                sample.invalid_quaternions++;
                /* Keep the last good sample briefly, but never refresh its age. */
            }
            break;
        default:
            break; /* Time/config tags may be interleaved with sensor records. */
        }
    }

    /* A stationary sensor is allowed to repeat identical values. Fresh FIFO tags,
     * not changing numbers, determine liveness. Gyro and SFLP arrive independently. */
    if (have_gyro != 0u && have_quat != 0u) {
        if ((uint32_t)(now_ms - sample.last_gyro_ms) >= IMU_STALE_MS ||
            (uint32_t)(now_ms - sample.last_quat_ms) >= IMU_STALE_MS) {
            fail(IMU_ERROR_STALE, now_ms);
            return;
        }
        /* Do not label an old FIFO backlog as the current orientation. Keep the
         * not-ready marker until the bounded poll has caught up with the queue. */
        sample.ready = (status.fifo_level <= IMU_FIFO_BUDGET) ? 1u : 0u;
        sample.error = IMU_ERROR_NONE;
    } else if ((uint32_t)(now_ms - run_since) >= IMU_STARTUP_TIMEOUT_MS) {
        fail(IMU_ERROR_STALE, now_ms);
    }
}

void imu_poll(uint32_t now_ms)
{
    if (force_reinit != 0u) {
        force_reinit = 0u;
        sample.recovery_count++;
        start_boot(now_ms);
    }
    switch (state) {
    case STATE_BOOT:
        if ((uint32_t)(now_ms - state_since) >= 10u) {
            /* FUNC_CFG_ACCESS is reachable from all banks; write SW_POR directly
             * as the official sw_por helper does, and wait without blocking. */
            uint8_t access = 0u;
            if (platform_write(NULL, LSM6DSV16X_FUNC_CFG_ACCESS, &access, 1u) != 0 ||
                lsm6dsv16x_device_id_get(&ctx, &sample.who_am_i) != 0) {
                fail(IMU_ERROR_SPI, now_ms);
                break;
            }
            if (sample.who_am_i != LSM6DSV16X_ID) {
                fail(IMU_ERROR_ID, now_ms);
                break;
            }
            access = 0x04u; /* FUNC_CFG_ACCESS.SW_POR, official driver bit 2. */
            if (platform_write(NULL, LSM6DSV16X_FUNC_CFG_ACCESS, &access, 1u) != 0) {
                fail(IMU_ERROR_SPI, now_ms);
                break;
            }
            state = STATE_POR_WAIT;
            state_since = now_ms;
        }
        break;
    case STATE_POR_WAIT:
        if ((uint32_t)(now_ms - state_since) >= 30u) {
            uint8_t access = 0u;
            if (platform_read(NULL, LSM6DSV16X_FUNC_CFG_ACCESS, &access, 1u) != 0) {
                fail(IMU_ERROR_SPI, now_ms);
            } else if ((access & 0x04u) != 0u) {
                if ((uint32_t)(now_ms - state_since) >= 100u) { fail(IMU_ERROR_RESET_TIMEOUT, now_ms); }
            } else {
                state = STATE_CONFIG;
                config_step = 0u;
            }
        }
        break;
    case STATE_CONFIG:
        if (configure_one_step() != 0) {
            fail(IMU_ERROR_CONFIG, now_ms);
        } else if (++config_step == 19u) {
            sample.configured = 1u;
            state = STATE_RUN;
            run_since = now_ms;
            last_poll_ms = now_ms - 1u;
        }
        break;
    case STATE_RUN:
        if (now_ms != last_poll_ms) {
            last_poll_ms = now_ms;
            poll_fifo(now_ms);
        }
        break;
    case STATE_RETRY:
        if ((uint32_t)(now_ms - state_since) >= IMU_RECOVERY_MS) {
            sample.recovery_count++;
            start_boot(now_ms);
        }
        break;
    default:
        fail(IMU_ERROR_CONFIG, now_ms);
        break;
    }
}

void imu_get_snapshot(imu_snapshot_t *out)
{
    uint32_t now_ms;
    if (out == NULL) { return; }
    *out = sample;
    /* Bus service can intentionally postpone imu_poll. Recheck age at the point
     * of every control-table read so a busy bus cannot make cached data immortal.
     * This read-only check preserves configured and diagnostics; poll owns resets. */
    now_ms = board_millis();
    if (out->ready != 0u &&
        ((uint32_t)(now_ms - out->last_gyro_ms) >= IMU_STALE_MS ||
         (uint32_t)(now_ms - out->last_quat_ms) >= IMU_STALE_MS)) {
        out->ready = 0u;
        out->error = IMU_ERROR_STALE;
    }
    if (out->ready == 0u) {
        memset(out->gyro, 0, sizeof(out->gyro));
        memset(out->quat_fp16, 0, sizeof(out->quat_fp16));
    }
}
