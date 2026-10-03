#include "protocol.h"
#include <string.h>

/* Wire format: Feetech "SCS communication protocol" manual sec.3-sec.4 (Feetech documents folder under docs/ in the repository).
 * Independent implementation; no vendor protocol source is incorporated.
 * Behaviour per instruction follows the bus protocol spec sec.8. */

#define INSTR_PING       0x01u
#define INSTR_READ       0x02u
#define INSTR_WRITE      0x03u
#define INSTR_REG_WRITE  0x04u
#define INSTR_ACTION     0x05u
#define INSTR_RESET      0x06u
#define INSTR_REBOOT     0x08u
#define INSTR_SYNC_READ  0x82u
#define INSTR_SYNC_WRITE 0x83u
#define TABLE_SIZE       256u
#define OWNED_LAST       8u     /* 0..8 answered here, sec.9. */

uint8_t proto_checksum(const uint8_t *bytes, uint16_t length)
{
    uint8_t sum = 0u;
    uint16_t i;
    for (i = 0u; i < length; ++i) {
        sum = (uint8_t)(sum + bytes[i]);
    }
    return (uint8_t)~sum;
}

static void cancel_reply(proto_context *ctx)
{
    if (ctx->pending != PROTO_IDLE) {
        ++ctx->cancelled_replies;
    }
    ctx->pending = PROTO_IDLE;
    ctx->tx_length = 0u;
}

void proto_reset_receiver(proto_context *ctx)
{
    ctx->rx_length = 0u;
    cancel_reply(ctx);
}

void proto_init(proto_context *ctx, proto_read_fn read_registers,
                proto_send_fn send, void *user)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->read = read_registers;
    ctx->send = send;
    ctx->user = user;
}

static uint8_t owned_value(uint16_t address)
{
    switch (address) {
    case 0u: return PROTO_FW_MAJOR;
    case 1u: return PROTO_FW_MINOR;
    case 2u: return 0u;                                   /* END: little endian. */
    case 3u: return (uint8_t)(PROTO_MODEL_NUMBER & 0xFFu);
    case 4u: return (uint8_t)(PROTO_MODEL_NUMBER >> 8);
    case 5u: return PROTO_DEVICE_ID;
    case 6u: return 0u;                                   /* Baud selector 0 = 1 Mbps. */
    case 8u: return 1u;                                   /* Status return level 1. */
    default: return 0u;                                   /* 7: reserved on STS. */
    }
}

static uint8_t read_table(proto_context *ctx, uint16_t address,
                          uint16_t length, uint8_t *out)
{
    uint16_t offset = 0u;
    if (length == 0u || length > PROTO_MAX_READ ||
        (uint32_t)address + length > TABLE_SIZE) {
        return PROTO_ERR_RANGE;
    }
    while (offset < length) {
        uint16_t here = (uint16_t)(address + offset);
        if (here <= OWNED_LAST) {
            out[offset++] = owned_value(here);
        } else {
            /* Everything after the owned header is one application run. */
            uint16_t run = (uint16_t)(length - offset);
            uint8_t error;
            if (ctx->read == 0) {
                return PROTO_ERR_RANGE;
            }
            error = ctx->read(ctx->user, here, run, out + offset);
            if (error != PROTO_OK) {
                return error;
            }
            offset = (uint16_t)(offset + run);
        }
    }
    return PROTO_OK;
}

/* Status frame, ERROR byte always 0x00 (sec.8). Payload must already be at tx[5]. */
static void make_status(proto_context *ctx, uint16_t data_length)
{
    ctx->tx[0] = 0xFFu;
    ctx->tx[1] = 0xFFu;
    ctx->tx[2] = PROTO_DEVICE_ID;
    ctx->tx[3] = (uint8_t)(data_length + 2u);
    ctx->tx[4] = 0x00u;
    ctx->tx[5u + data_length] = proto_checksum(ctx->tx + 2u, (uint16_t)(3u + data_length));
    ctx->tx_length = (uint16_t)(6u + data_length);
}

static void schedule(proto_context *ctx, uint32_t now_us)
{
    ctx->pending = PROTO_READY;
    ctx->reply_due_us = now_us + PROTO_RESPONSE_DELAY_US;
}

static void reply_ack(proto_context *ctx, uint32_t now_us)
{
    make_status(ctx, 0u);
    schedule(ctx, now_us);
}

/* Build the data reply now, from one consistent table read. */
static int build_read(proto_context *ctx, uint16_t address, uint16_t length)
{
    if (read_table(ctx, address, length, ctx->tx + 5u) != PROTO_OK) {
        ++ctx->malformed_packets;
        return 0;
    }
    make_status(ctx, length);
    return 1;
}

/* IDs must be real device IDs (0..253) and unique; no device answers a
 * malformed list, even when its own entry looks fine. */
static int valid_id_list(const uint8_t *ids, uint16_t count)
{
    uint16_t i;
    uint16_t j;
    if (count == 0u) {
        return 0;
    }
    for (i = 0u; i < count; ++i) {
        if (ids[i] >= PROTO_BROADCAST_ID) {
            return 0;
        }
        for (j = 0u; j < i; ++j) {
            if (ids[i] == ids[j]) {
                return 0;
            }
        }
    }
    return 1;
}

static void sync_read(proto_context *ctx, const uint8_t *p, uint16_t n, uint32_t now_us)
{
    const uint8_t *ids = p + 2u;
    uint16_t count;
    uint16_t k;
    if (n < 3u || !valid_id_list(ids, (uint16_t)(n - 2u))) {
        ++ctx->malformed_packets;
        return;
    }
    count = (uint16_t)(n - 2u);
    for (k = 0u; k < count && ids[k] != PROTO_DEVICE_ID; ++k) {
    }
    if (k == count) {
        return;                                   /* Not asked. */
    }
    if (k >= PROTO_MAX_SYNC_IDS || !build_read(ctx, p[0], p[1])) {
        if (k >= PROTO_MAX_SYNC_IDS) {
            ++ctx->malformed_packets;
        }
        return;
    }
    if (k == 0u) {                                /* Runtime case: 200 is always first. */
        schedule(ctx, now_us);
        return;
    }
    /* sec.5 queue rule: stay receiving until the k devices ahead have answered. */
    memcpy(ctx->before, ids, k);
    ctx->turn = (uint8_t)k;
    ctx->slot = 0u;
    ctx->sync_length = p[1];
    ctx->transaction_start_us = now_us;
    ctx->quiet_since_us = now_us;
    ctx->pending = PROTO_WAIT_TURN;
}

/* Byte 4 of a frame: an instruction code from the controller, or the ERROR
 * byte of a status frame. A healthy servo replies with ERROR 0. */
static int is_instruction_code(uint8_t b)
{
    switch (b) {
    case INSTR_PING: case INSTR_READ: case INSTR_WRITE: case INSTR_REG_WRITE:
    case INSTR_ACTION: case INSTR_RESET: case INSTR_REBOOT: case 0x09u: case 0x0Au:
    case 0x0Bu: case INSTR_SYNC_READ: case INSTR_SYNC_WRITE:
        return 1;
    default:
        return 0;
    }
}

/* While queued, a frame is "a reply from a device ahead of us" when its ID is
 * listed before ours, its length matches the Sync Read and byte 4 is not an
 * instruction code. The last test keeps a controller instruction that happens
 * to have the same ID and LEN (e.g. READ to that servo during a len-2 Sync
 * Read) from being taken for a reply; we would then answer on top of that
 * servo's reply. A servo whose ERROR flags equal an instruction code is taken
 * as a new instruction instead: we stay silent, which is the safe side.
 * Returns 1 if it was a queued reply. */
static int queued_reply(proto_context *ctx, uint8_t id, uint8_t len, uint8_t byte4, uint32_t now_us)
{
    uint8_t i;
    if (len != (uint8_t)(ctx->sync_length + 2u) || is_instruction_code(byte4)) {
        return 0;
    }
    for (i = 0u; i < ctx->turn; ++i) {
        if (ctx->before[i] == id) {
            if ((uint8_t)(i + 1u) > ctx->slot) {
                ctx->slot = (uint8_t)(i + 1u);
            }
            if (ctx->slot >= ctx->turn) {
                schedule(ctx, now_us);            /* Our turn: T_resp after that reply. */
            }
            return 1;
        }
    }
    return 0;
}

static void execute(proto_context *ctx, uint32_t now_us)
{
    uint8_t id = ctx->rx[2];
    uint8_t len = ctx->rx[3];
    uint8_t instruction = ctx->rx[4];
    const uint8_t *p = ctx->rx + 5u;
    uint16_t n = (uint16_t)(len - 2u);
    int unicast = id == PROTO_DEVICE_ID;
    if (ctx->pending == PROTO_WAIT_TURN && queued_reply(ctx, id, len, instruction, now_us)) {
        return;
    }
    /* Any other valid frame is a new controller instruction: the earlier
     * transaction is over, including when it is addressed to someone else. */
    cancel_reply(ctx);
    if (!unicast && id != PROTO_BROADCAST_ID) {
        return;
    }
    switch (instruction) {
    case INSTR_PING:                              /* Broadcast PING stays silent (sec.8). */
        if (unicast) {
            reply_ack(ctx, now_us);
        }
        break;
    case INSTR_READ:
        if (!unicast) {
            break;
        }
        if (n != 2u) {
            ++ctx->malformed_packets;
        } else if (build_read(ctx, p[0], p[1])) {
            schedule(ctx, now_us);
        }
        break;
    case INSTR_WRITE:                             /* ACK, contents ignored (sec.8). */
    case INSTR_REG_WRITE:
        if (unicast) {
            if (n < 2u) {
                ++ctx->malformed_packets;
            } else {
                reply_ack(ctx, now_us);
            }
        }
        break;
    case INSTR_ACTION:
    case INSTR_RESET:
    case 0x09u:
    case 0x0Au:
    case 0x0Bu:                                   /* ACK, no action (sec.8). */
        if (unicast) {
            reply_ack(ctx, now_us);
        }
        break;
    case INSTR_REBOOT:                            /* Real reboot, no reply (sec.8). */
        ctx->reboot_requested = 1u;
        break;
    case INSTR_SYNC_READ:
        if (!unicast) {
            sync_read(ctx, p, n, now_us);
        }
        break;
    case INSTR_SYNC_WRITE:                        /* Never configured by the controller. */
        break;
    default:
        if (unicast) {
            ++ctx->malformed_packets;
        }
        break;
    }
}

static void drop_front(proto_context *ctx, uint16_t count)
{
    if (count >= ctx->rx_length) {
        ctx->rx_length = 0u;
        return;
    }
    memmove(ctx->rx, ctx->rx + count, (size_t)(ctx->rx_length - count));
    ctx->rx_length = (uint16_t)(ctx->rx_length - count);
}

/* Consume every complete frame in rx; keep a possible frame start. Iterative:
 * a checksum failure drops one byte and rescans, so a false FF FF inside a
 * payload cannot hide the real frame that follows. */
static void scan(proto_context *ctx, uint32_t now_us)
{
    for (;;) {
        uint16_t skip = 0u;
        uint16_t total;
        while (skip < ctx->rx_length && !(ctx->rx[skip] == 0xFFu &&
               (skip + 1u >= ctx->rx_length || ctx->rx[skip + 1u] == 0xFFu))) {
            ++skip;
        }
        drop_front(ctx, skip);
        if (ctx->rx_length < 4u) {
            return;                               /* Need more bytes. */
        }
        if (ctx->rx[2] == 0xFFu) {                /* FF FF FF: one extra preamble byte. */
            drop_front(ctx, 1u);
            continue;
        }
        if (ctx->rx[3] < 2u || (uint16_t)(ctx->rx[3] + 4u) > PROTO_PACKET_CAPACITY) {
            ++ctx->malformed_packets;
            drop_front(ctx, 1u);
            continue;
        }
        total = (uint16_t)(ctx->rx[3] + 4u);
        if (ctx->rx_length < total) {
            return;
        }
        if (proto_checksum(ctx->rx + 2u, (uint16_t)(total - 3u)) != ctx->rx[total - 1u]) {
            ++ctx->crc_errors;
            drop_front(ctx, 1u);
            continue;
        }
        ++ctx->rx_packets;
        execute(ctx, now_us);
        drop_front(ctx, total);
    }
}

void proto_feed(proto_context *ctx, uint8_t byte, uint32_t now_us)
{
    if (ctx->rx_length != 0u &&
        (uint32_t)(now_us - ctx->last_byte_us) > PROTO_BYTE_TIMEOUT_US) {
        ctx->rx_length = 0u;
        ++ctx->receive_timeouts;
    }
    /* A reply waiting for T_resp loses its turn as soon as someone else talks.
     * The application MUST drain RX before calling proto_poll. */
    if (ctx->pending == PROTO_READY) {
        cancel_reply(ctx);
    }
    ctx->last_byte_us = now_us;
    ctx->quiet_since_us = now_us;
    if (ctx->rx_length >= PROTO_PACKET_CAPACITY) {
        ctx->rx_length = 0u;                      /* Unreachable: scan bounds frames. */
        ++ctx->malformed_packets;
    }
    ctx->rx[ctx->rx_length++] = byte;
    scan(ctx, now_us);
}

void proto_poll(proto_context *ctx, uint32_t now_us)
{
    if (ctx->rx_length != 0u &&
        (uint32_t)(now_us - ctx->last_byte_us) > PROTO_BYTE_TIMEOUT_US) {
        ctx->rx_length = 0u;
        ++ctx->receive_timeouts;
    }
    if (ctx->pending == PROTO_WAIT_TURN) {
        if ((uint32_t)(now_us - ctx->transaction_start_us) >= PROTO_TRANSACTION_TIMEOUT_US) {
            cancel_reply(ctx);
            return;
        }
        /* A listed device that stays silent is offline: count its slot (sec.5). */
        if (ctx->rx_length == 0u &&
            (uint32_t)(now_us - ctx->quiet_since_us) >= PROTO_SLOT_GAP_US) {
            ++ctx->skipped_slots;
            ++ctx->slot;
            ctx->quiet_since_us = now_us;
            if (ctx->slot >= ctx->turn) {
                ctx->pending = PROTO_READY;       /* Bus has been idle: go now. */
                ctx->reply_due_us = now_us;
            }
        }
    }
    if (ctx->pending == PROTO_READY && (int32_t)(now_us - ctx->reply_due_us) >= 0) {
        uint16_t length = ctx->tx_length;
        if ((uint32_t)(now_us - ctx->reply_due_us) > PROTO_REPLY_EXPIRY_US ||
            ctx->rx_length != 0u) {
            cancel_reply(ctx);
            return;
        }
        ctx->pending = PROTO_IDLE;
        ctx->tx_length = 0u;
        if (ctx->send != 0) {
            ctx->send(ctx->user, ctx->tx, length);
            ++ctx->tx_packets;
        }
    }
}
