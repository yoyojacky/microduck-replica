/* Host shim: the REAL protocol.c + control_table.c as a DLL, so Python can put
 * them on a simulated servo bus (Tests/bus_sim_test.py) and run the acceptance
 * tool tools/imu200/imu200.py against them. Board and IMU are stubbed here. */
#include "protocol.h"
#include "control_table.h"
#include "imu.h"
#include "board.h"
#include <string.h>

#ifdef _WIN32
#define SHIM_API __declspec(dllexport)
#else
#define SHIM_API __attribute__((visibility("default")))
#endif

board_diagnostics_t board_diagnostics;
static proto_context ctx;
static imu_snapshot_t snapshot;
static uint32_t millis;
static uint8_t tx[PROTO_PACKET_CAPACITY];
static uint16_t tx_length;
static unsigned tx_count;

uint32_t board_millis(void) { return millis; }
void imu_get_snapshot(imu_snapshot_t *out) { *out = snapshot; }
void imu_request_reinit(void) { }

static void capture(void *user, const uint8_t *packet, uint16_t length)
{
    (void)user;
    memcpy(tx, packet, length);
    tx_length = length;
    ++tx_count;
}

SHIM_API void shim_init(void)
{
    memset(&snapshot, 0, sizeof(snapshot));
    tx_length = 0u;
    tx_count = 0u;
    millis = 0u;
    proto_init(&ctx, control_table_read, capture, &ctx);
}

SHIM_API void shim_feed(uint8_t byte, uint32_t now_us) { proto_feed(&ctx, byte, now_us); }
SHIM_API void shim_poll(uint32_t now_us) { proto_poll(&ctx, now_us); }
SHIM_API void shim_set_millis(uint32_t ms) { millis = ms; }
SHIM_API int shim_pending(void) { return ctx.pending; }
SHIM_API int shim_reboot_requested(void) { return ctx.reboot_requested; }

/* Returns the length of the packet sent since the last call (0 if none). */
SHIM_API int shim_take_tx(uint8_t *out, int capacity)
{
    int length = tx_length;
    if (length > capacity) {
        return -1;
    }
    memcpy(out, tx, (size_t)length);
    tx_length = 0u;
    return length;
}

SHIM_API void shim_set_snapshot(int ready, int error, int16_t gx, int16_t gy, int16_t gz,
                                uint16_t qx, uint16_t qy, uint16_t qz, uint32_t samples)
{
    snapshot.ready = (uint8_t)ready;
    snapshot.configured = (uint8_t)ready;
    snapshot.who_am_i = 0x70u;
    snapshot.error = (uint8_t)error;
    snapshot.gyro[0] = gx; snapshot.gyro[1] = gy; snapshot.gyro[2] = gz;
    snapshot.quat_fp16[0] = qx; snapshot.quat_fp16[1] = qy; snapshot.quat_fp16[2] = qz;
    snapshot.sample_count = samples;
}

/* rx_packets, tx_packets, crc, malformed, timeouts, cancelled, skipped */
SHIM_API void shim_counters(uint32_t out[7])
{
    out[0] = ctx.rx_packets; out[1] = ctx.tx_packets; out[2] = ctx.crc_errors;
    out[3] = ctx.malformed_packets; out[4] = ctx.receive_timeouts;
    out[5] = ctx.cancelled_replies; out[6] = ctx.skipped_slots;
}
