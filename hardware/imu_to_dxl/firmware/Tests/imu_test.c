/* Host-only tests of the actual imu.c + unmodified ST register driver. */
#include "imu.h"
#include "lsm6dsv16x_reg.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
static uint8_t regs[3][256];
static uint8_t bank;
static uint8_t fifo[512][7];
static unsigned fifo_read, fifo_write;
static uint8_t absent, bad_id, stuck_reset, overrun, ignore_writes;
static uint32_t now;
static unsigned fifo_reads;
static unsigned tests;

static void reset_registers(void)
{
    memset(regs, 0, sizeof(regs));
    regs[0][LSM6DSV16X_WHO_AM_I] = bad_id ? 0x6bu : LSM6DSV16X_ID;
    regs[0][LSM6DSV16X_CTRL3] = 0x44u;
    bank = 0u;
    fifo_read = fifo_write = 0u;
}

int32_t board_spi_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    unsigned i;
    if (absent) { return -1; }
    if (reg == LSM6DSV16X_FUNC_CFG_ACCESS && len == 1u) {
        data[0] = (uint8_t)((bank == 1u ? 0x80u : (bank == 2u ? 0x40u : 0u)) | (stuck_reset ? 4u : 0u));
        return 0;
    }
    if (bank == 0u && reg == LSM6DSV16X_FIFO_STATUS1 && len == 2u) {
        unsigned count = fifo_write - fifo_read;
        data[0] = (uint8_t)count;
        data[1] = (uint8_t)(((count >> 8) & 1u) | (overrun ? 0x40u : 0u));
        return 0;
    }
    if (bank == 0u && reg == LSM6DSV16X_FIFO_DATA_OUT_TAG && len == 7u) {
        CHECK(fifo_read < fifo_write);
        memcpy(data, fifo[fifo_read++ % 512u], 7u);
        fifo_reads++;
        return 0;
    }
    for (i = 0; i < len; ++i) { data[i] = regs[bank][(uint8_t)(reg + i)]; }
    return 0;
}

int32_t board_spi_write(uint8_t reg, const uint8_t *data, uint16_t len)
{
    unsigned i;
    if (absent) { return -1; }
    if (reg == LSM6DSV16X_FUNC_CFG_ACCESS && len == 1u) {
        if ((data[0] & 4u) != 0u) { reset_registers(); }
        else { bank = (data[0] & 0x80u) ? 1u : ((data[0] & 0x40u) ? 2u : 0u); }
        return 0;
    }
    if (ignore_writes) { return 0; }
    for (i = 0; i < len; ++i) { regs[bank][(uint8_t)(reg + i)] = data[i]; }
    return 0;
}

void board_delay_ms(uint32_t ms) { (void)ms; CHECK(0 && "IMU must never busy-wait through board_delay_ms"); }
uint32_t board_millis(void) { return now; }

static imu_snapshot_t snapshot(void)
{
    imu_snapshot_t result;
    imu_get_snapshot(&result);
    return result;
}

static void tick(uint32_t count)
{
    while (count-- != 0u) { ++now; imu_poll(now); }
}

static void fresh(void)
{
    absent = bad_id = stuck_reset = overrun = ignore_writes = 0u;
    now = 0u;
    fifo_reads = 0u;
    reset_registers();
    imu_init(now);
}

static void configure(void)
{
    unsigned i;
    for (i = 0; i < 120u && !snapshot().configured; ++i) { tick(1u); }
    CHECK(snapshot().configured);
    CHECK(!snapshot().ready);
    CHECK(snapshot().who_am_i == LSM6DSV16X_ID);
}

static void record(uint8_t tag, uint16_t x, uint16_t y, uint16_t z)
{
    uint8_t *p = fifo[fifo_write++ % 512u];
    CHECK(fifo_write - fifo_read < 512u);
    p[0] = (uint8_t)(tag << 3);
    p[1] = (uint8_t)x; p[2] = (uint8_t)(x >> 8);
    p[3] = (uint8_t)y; p[4] = (uint8_t)(y >> 8);
    p[5] = (uint8_t)z; p[6] = (uint8_t)(z >> 8);
}

static void live(void)
{
    record(LSM6DSV16X_GY_NC_TAG, 12u, (uint16_t)-34, 56u);
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x3400u, 0u, 0u);
    tick(1u);
    CHECK(snapshot().ready);
}

static void test_startup_and_async_records(void)
{
    imu_snapshot_t s;
    fresh();
    CHECK(!snapshot().ready);
    CHECK(snapshot().quat_fp16[0] == 0u);
    configure();
    CHECK((regs[0][LSM6DSV16X_CTRL1] & 0x0fu) == 6u);
    CHECK((regs[0][LSM6DSV16X_CTRL2] & 0x0fu) == 6u);
    CHECK((regs[0][LSM6DSV16X_FIFO_CTRL3] & 0xffu) == 0x66u);
    record(LSM6DSV16X_GY_NC_TAG, 0x8000u, 0x7fffu, 0xffffu);
    tick(1u);
    CHECK(!snapshot().ready);
    CHECK(snapshot().gyro_count == 1u);
    CHECK(snapshot().gyro[0] == 0);
    record(LSM6DSV16X_XL_NC_TAG, 0x1111u, 0x2222u, 0x3333u);
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x3400u, 0xb800u, 1u);
    tick(1u);
    s = snapshot();
    CHECK(s.ready && s.error == IMU_ERROR_NONE);
    CHECK(s.gyro[0] == -32768 && s.gyro[1] == 32767 && s.gyro[2] == -1);
    CHECK(s.quat_fp16[0] == 0x3400u && s.quat_fp16[1] == 0xb800u && s.quat_fp16[2] == 1u);
    CHECK(s.raw_accel[2] == 0x3333 && s.accel_count == 1u && s.sample_count == 1u);
    ++tests;
}

static void test_live_identity(void)
{
    fresh(); configure();
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0u, 0u, 0u);
    tick(1u);
    CHECK(!snapshot().ready);
    record(LSM6DSV16X_GY_NC_TAG, 0u, 0u, 0u);
    tick(1u);
    CHECK(snapshot().ready);
    CHECK(snapshot().quat_fp16[0] == 0x8000u);
    CHECK(snapshot().quat_fp16[1] == 0u && snapshot().quat_fp16[2] == 0u);
    ++tests;
}

static void test_invalid_quaternions(void)
{
    fresh(); configure(); live();
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x7c00u, 0u, 0u); /* infinity */
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x7e00u, 0u, 0u); /* NaN */
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x3c00u, 0x3c00u, 0u); /* norm^2=2 */
    tick(1u);
    CHECK(snapshot().invalid_quaternions == 3u);
    CHECK(snapshot().quat_fp16[0] == 0x3400u && snapshot().sample_count == 1u);
    /* A small over-one roundoff is allowed, matching upstream's <=1.02 bound. */
    record(LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG, 0x3c01u, 0u, 0u);
    tick(1u);
    CHECK(snapshot().sample_count == 2u && snapshot().quat_fp16[0] == 0x3c01u);
    ++tests;
}

static void test_absent_and_recovery(void)
{
    fresh(); absent = 1u; tick(10u);
    CHECK(snapshot().error == IMU_ERROR_SPI && snapshot().spi_errors == 1u);
    absent = 0u; tick(IMU_RECOVERY_MS); configure(); live();
    CHECK(snapshot().recovery_count == 1u && snapshot().spi_errors == 1u);
    ++tests;
}

static void test_bad_identity(void)
{
    fresh(); bad_id = 1u; reset_registers(); tick(10u);
    CHECK(snapshot().error == IMU_ERROR_ID && snapshot().who_am_i == 0x6bu);
    CHECK(!snapshot().configured && !snapshot().ready);
    ++tests;
}

static void test_stuck_reset(void)
{
    fresh(); stuck_reset = 1u; tick(111u);
    CHECK(snapshot().error == IMU_ERROR_RESET_TIMEOUT);
    CHECK(!snapshot().configured);
    ++tests;
}

static void test_readback_detects_lost_writes(void)
{
    fresh(); ignore_writes = 1u; tick(100u);
    CHECK(!snapshot().configured && snapshot().error == IMU_ERROR_CONFIG);
    ++tests;
}

static void test_freeze_and_restart(void)
{
    uint32_t samples;
    fresh(); configure(); live(); samples = snapshot().sample_count;
    tick(IMU_STALE_MS);
    CHECK(!snapshot().ready && snapshot().error == IMU_ERROR_STALE);
    CHECK(snapshot().quat_fp16[0] == 0u && snapshot().gyro[0] == 0);
    tick(IMU_RECOVERY_MS); configure(); live();
    CHECK(snapshot().sample_count == samples + 1u && snapshot().recovery_count == 1u);
    ++tests;
}

static void test_independent_freeze(void)
{
    unsigned i;
    fresh(); configure(); live();
    for (i = 0; i < IMU_STALE_MS; ++i) {
        record(LSM6DSV16X_GY_NC_TAG, 1u, 2u, 3u); tick(1u);
    }
    CHECK(!snapshot().ready && snapshot().error == IMU_ERROR_STALE);
    ++tests;
}

static void test_constant_values_are_live(void)
{
    unsigned i;
    fresh(); configure();
    for (i = 0; i < 30u; ++i) { live(); tick(8u); }
    CHECK(snapshot().ready && snapshot().sample_count == 30u);
    ++tests;
}

static void test_fifo_budget_and_backlog(void)
{
    unsigned i;
    fresh(); configure(); live(); fifo_reads = 0u;
    for (i = 0; i < IMU_FIFO_BUDGET + 2u; ++i) { record(LSM6DSV16X_GY_NC_TAG, (uint16_t)i, 0u, 0u); }
    tick(1u);
    CHECK(fifo_reads == IMU_FIFO_BUDGET && !snapshot().ready);
    tick(1u);
    CHECK(fifo_reads == IMU_FIFO_BUDGET + 2u && snapshot().ready);
    ++tests;
}

static void test_overflow_and_runtime_spi_fault(void)
{
    fresh(); configure(); live(); overrun = 1u; tick(1u);
    CHECK(!snapshot().ready && snapshot().fifo_overflows == 1u && snapshot().error == IMU_ERROR_FIFO);
    fresh(); configure(); live(); absent = 1u; tick(1u);
    CHECK(!snapshot().ready && snapshot().error == IMU_ERROR_SPI);
    ++tests;
}

static void test_startup_without_samples(void)
{
    fresh(); configure(); tick(IMU_STARTUP_TIMEOUT_MS);
    CHECK(!snapshot().ready && snapshot().error == IMU_ERROR_STALE);
    ++tests;
}

static void test_wrap_and_manual_restart(void)
{
    fresh(); now = 0xffffffe0u; imu_init(now); configure(); live();
    CHECK(now < 0x100u);
    imu_request_reinit(); tick(1u);
    CHECK(!snapshot().ready && snapshot().recovery_count == 1u);
    configure(); live();
    CHECK(snapshot().ready);
    ++tests;
}

static void test_snapshot_ages_without_poll(void)
{
    imu_snapshot_t before, after;
    fresh(); configure(); live(); before = snapshot();
    /* Model a continuous host packet stream that has priority over IMU polling. */
    now += IMU_STALE_MS - 1u;
    CHECK(snapshot().ready);
    ++now;
    after = snapshot();
    CHECK(!after.ready && after.error == IMU_ERROR_STALE);
    CHECK(after.configured && after.who_am_i == LSM6DSV16X_ID);
    CHECK(after.gyro[0] == 0 && after.quat_fp16[0] == 0u);
    CHECK(after.sample_count == before.sample_count);
    CHECK(after.last_quat_ms == before.last_quat_ms);
    CHECK(after.recovery_count == 0u);
    /* Reading the snapshot is side-effect free; normal poll performs recovery. */
    tick(1u);
    CHECK(!snapshot().configured && snapshot().error == IMU_ERROR_STALE);
    ++tests;
}

static void test_snapshot_ages_across_wrap(void)
{
    fresh(); now = 0xffffff80u; imu_init(now); configure(); live();
    CHECK(now > 0xffffff80u);
    now += IMU_STALE_MS;
    CHECK(now < 0x80u);
    CHECK(!snapshot().ready && snapshot().configured);
    CHECK(snapshot().error == IMU_ERROR_STALE);
    ++tests;
}

int main(void)
{
    test_startup_and_async_records();
    test_live_identity();
    test_invalid_quaternions();
    test_absent_and_recovery();
    test_bad_identity();
    test_stuck_reset();
    test_readback_detects_lost_writes();
    test_freeze_and_restart();
    test_independent_freeze();
    test_constant_values_are_live();
    test_fifo_budget_and_backlog();
    test_overflow_and_runtime_spi_fault();
    test_startup_without_samples();
    test_wrap_and_manual_restart();
    test_snapshot_ages_without_poll();
    test_snapshot_ages_across_wrap();
    printf("IMU host tests passed: %u scenarios (no hardware)\n", tests);
    return 0;
}
