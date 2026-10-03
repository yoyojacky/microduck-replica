#ifndef IMU_TO_DXL_PROTOCOL_H
#define IMU_TO_DXL_PROTOCOL_H

#include <stdint.h>

/* Feetech SCS/STS slave: impersonates one servo, ID 200, on the servo bus.
 * Not a real servo. The interface contract is the bus protocol spec in hardware/imu_to_dxl/;
 * section numbers below (sec.N) refer to that file.
 *
 * Frame: FF FF ID LEN INSTR/ERR params... CHK
 *        LEN = params + 2, CHK = ~(ID + LEN + INSTR + sum(params)) & 0xFF
 * There is no byte stuffing, so FF FF may also appear inside a payload: the
 * receiver resynchronizes by checksum, never by header alone. */
#define PROTO_DEVICE_ID              200u
#define PROTO_BROADCAST_ID           0xFEu
#define PROTO_FW_MAJOR               0u      /* Address 0-1: our own version (sec.9). */
#define PROTO_FW_MINOR               2u
#define PROTO_MODEL_NUMBER           0x4D44u /* Address 3-4: custom model. */
#define PROTO_PACKET_CAPACITY        260u    /* FF FF ID LEN + up to 255 more bytes. */
#define PROTO_MAX_READ               250u    /* Reply is 6 + length bytes. */
#define PROTO_MAX_SYNC_IDS           64u     /* Queue position we can track (sec.5). */
/* sec.5 timing. All values are inferred from the 1 Mbps / 6 ms tick budget and
 * must be covered by bench measurements (sec.12). */
#define PROTO_RESPONSE_DELAY_US      50u     /* T_resp: last stop bit -> our first start bit. */
#define PROTO_BYTE_TIMEOUT_US        500u    /* Longer gap inside a frame: drop the partial frame. */
#define PROTO_SLOT_GAP_US            1000u   /* Silent this long while queued: device ahead is offline. */
#define PROTO_TRANSACTION_TIMEOUT_US 25000u  /* Never got our turn: give up the reply. */
#define PROTO_REPLY_EXPIRY_US        2000u   /* Could not send this late after due: give up. */

enum {
    PROTO_OK = 0,
    PROTO_ERR_RANGE = 1
};

enum {
    PROTO_IDLE = 0,       /* Nothing to send. */
    PROTO_READY = 1,      /* Reply built, send at reply_due_us. */
    PROTO_WAIT_TURN = 2   /* Sync Read: wait until the devices listed before us replied. */
};

/* read fills the complete range or returns PROTO_ERR_RANGE; it is called for
 * addresses 9..255 only (0..8 are answered here, sec.9). send is synchronous:
 * send every byte, wait for UART TC (not TXE), release the driver, re-enable
 * RX, then return. No callbacks run in an ISR. */
typedef uint8_t (*proto_read_fn)(void *user, uint16_t address,
                              uint16_t length, uint8_t *out);
typedef void (*proto_send_fn)(void *user, const uint8_t *packet, uint16_t length);

typedef struct {
    uint8_t rx[PROTO_PACKET_CAPACITY];
    uint8_t tx[PROTO_PACKET_CAPACITY];
    uint8_t before[PROTO_MAX_SYNC_IDS]; /* Sync Read IDs listed ahead of ours. */
    proto_read_fn read;
    proto_send_fn send;
    void *user;
    uint32_t last_byte_us;           /* Receive time of the newest byte. */
    uint32_t quiet_since_us;         /* Queue gap timer: last byte or last skipped slot. */
    uint32_t transaction_start_us;
    uint32_t reply_due_us;
    uint32_t rx_packets;             /* Checksum-valid frames seen on the bus (any ID). */
    uint32_t tx_packets;
    uint32_t crc_errors;             /* Checksum failures. */
    uint32_t malformed_packets;      /* Bad LEN / params, or a request we cannot serve. */
    uint32_t receive_timeouts;       /* Partial frames dropped by the byte gap. */
    uint32_t cancelled_replies;      /* Replies given up (bus taken, expired, send failed). */
    uint32_t skipped_slots;          /* Queued Sync Read: devices ahead judged offline. */
    uint16_t rx_length;
    uint16_t tx_length;
    uint8_t pending;                 /* PROTO_IDLE / PROTO_READY / PROTO_WAIT_TURN */
    uint8_t turn;                    /* Our index k in the Sync Read ID list. */
    uint8_t slot;                    /* How many listed devices ahead of us are done. */
    uint8_t sync_length;             /* Data length of the queued Sync Read. */
    uint8_t reboot_requested;        /* REBOOT 0x08 received: main performs the reset. */
} proto_context;

void proto_init(proto_context *ctx, proto_read_fn read_registers,
                proto_send_fn send, void *user);
/* Unsigned 32-bit MICROSECOND timestamps; wraparound is supported. Pass the
 * actual receive timestamp when draining an ISR ring, not the later drain time.
 * Drain RX before proto_poll, and poll frequently (target <20 us while a reply
 * is pending: T_resp is 50 us). */
void proto_feed(proto_context *ctx, uint8_t byte, uint32_t now_us);
void proto_poll(proto_context *ctx, uint32_t now_us);
/* Discard partial RX/pending TX after UART overrun or a board-level bus fault. */
void proto_reset_receiver(proto_context *ctx);
/* ~(sum of bytes) & 0xFF, over ID..last parameter. */
uint8_t proto_checksum(const uint8_t *bytes, uint16_t length);

#endif
