/* Host tests for the Feetech protocol slave (Core/Src/protocol.c).
 * Vectors are copied byte for byte from the bus protocol spec in hardware/imu_to_dxl/ sec.10;
 * behaviour checks follow sec.5 (timing, queue rule) and sec.8 (per instruction).
 * Portable C11, no hardware. Run: python Tools/protocol_test.py */
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #test); } } while (0)

#define BYTE_US 10u   /* 1 Mbps, 10 bits per byte. */

typedef struct {
    proto_context protocol;
    uint8_t registers[256];
    uint8_t output[512];
    uint16_t output_length;
    unsigned sends;
    unsigned reads;
    uint32_t now;
} fixture;

static uint8_t read_regs(void *user, uint16_t address, uint16_t length, uint8_t *out)
{
    fixture *f = (fixture *)user;
    ++f->reads;
    if (address < 9u || (uint32_t)address + length > sizeof(f->registers)) {
        return PROTO_ERR_RANGE;      /* 0..8 must never reach the application. */
    }
    memcpy(out, f->registers + address, length);
    return PROTO_OK;
}

static void send_bytes(void *user, const uint8_t *packet, uint16_t length)
{
    fixture *f = (fixture *)user;
    CHECK(length <= sizeof(f->output));
    memcpy(f->output, packet, length);
    f->output_length = length;
    ++f->sends;
}

static void init(fixture *f)
{
    unsigned i;
    memset(f, 0, sizeof(*f));
    for (i = 0u; i < sizeof(f->registers); ++i) {
        f->registers[i] = (uint8_t)i;
    }
    f->now = 1000u;
    proto_init(&f->protocol, read_regs, send_bytes, f);
}

/* Feed raw bytes 10 us apart; f->now ends at the last byte's timestamp. */
static void feed(fixture *f, const uint8_t *bytes, uint16_t length)
{
    uint16_t i;
    for (i = 0u; i < length; ++i) {
        if (i != 0u) {
            f->now += BYTE_US;
        }
        proto_feed(&f->protocol, bytes[i], f->now);
    }
}

/* Build FF FF id LEN instr params CHK and feed it. */
static void feed_frame(fixture *f, uint8_t id, uint8_t instr, const uint8_t *params, uint8_t n)
{
    uint8_t frame[300];
    frame[0] = 0xFFu;
    frame[1] = 0xFFu;
    frame[2] = id;
    frame[3] = (uint8_t)(n + 2u);
    frame[4] = instr;
    if (n != 0u) {
        memcpy(frame + 5, params, n);
    }
    frame[5u + n] = proto_checksum(frame + 2, (uint16_t)(3u + n));
    feed(f, frame, (uint16_t)(6u + n));
}

/* A status frame from another device (reply to a Sync Read of `length` bytes). */
static void feed_reply(fixture *f, uint8_t id, uint8_t length)
{
    uint8_t data[250];
    memset(data, 0x5A, length);
    feed_frame(f, id, 0x00u, data, length);
}

static void poll_until(fixture *f, uint32_t until)
{
    while ((int32_t)(until - f->now) > 0) {
        f->now += 1u;
        proto_poll(&f->protocol, f->now);
    }
}

static int sent(const fixture *f, const uint8_t *expected, uint16_t length)
{
    return f->output_length == length && memcmp(f->output, expected, length) == 0;
}

static void put_block(fixture *f, const uint8_t block[15])
{
    memcpy(f->registers + 56, block, 15);
}

/* ---- sec.10 vectors ------------------------------------------------------ */

static void test_vectors(void)
{
    static const uint8_t full_sync[] = {0xff,0xff,0xfe,0x14,0x82,0x38,0x0f,0xc8,0x14,0x15,0x16,0x17,
                                        0x18,0x1e,0x1f,0x20,0x21,0x22,0x0a,0x0b,0x0c,0x0d,0x0e,0x12};
    static const uint8_t only_200[] = {0xff,0xff,0xfe,0x05,0x82,0x38,0x0f,0xc8,0x6b};
    static const uint8_t ping[] = {0xff,0xff,0xc8,0x02,0x01,0x34};
    static const uint8_t ack[] = {0xff,0xff,0xc8,0x02,0x00,0x35};
    static const uint8_t not_ready_block[15] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0x01,0};
    static const uint8_t not_ready_reply[] = {0xff,0xff,0xc8,0x11,0x00,0,0,0,0,0,0,0,0,0,0,0,0,0,0x01,0,0x25};
    static const uint8_t ready_block[15] = {0x64,0x00,0xce,0xff,0x03,0x00,0x00,0x00,0x00,0x00,0x20,0x36,0x07,0x00,0x00};
    static const uint8_t ready_reply[] = {0xff,0xff,0xc8,0x11,0x00,0x64,0x00,0xce,0xff,0x03,0x00,0x00,0x00,
                                          0x00,0x00,0x20,0x36,0x07,0x00,0x00,0x95};
    fixture f;
    uint32_t end;

    /* Controller's per-tick Sync Read, 200 first: answer T_resp after the last byte. */
    init(&f);
    put_block(&f, ready_block);
    feed(&f, full_sync, sizeof(full_sync));
    end = f.now;
    CHECK(f.protocol.pending == PROTO_READY);
    poll_until(&f, end + PROTO_RESPONSE_DELAY_US - 1u);
    CHECK(f.sends == 0u);
    poll_until(&f, end + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);
    CHECK(sent(&f, ready_reply, sizeof(ready_reply)));

    /* Sync Read of 200 alone, fusion not ready yet: all-zero block, BIT0 set. */
    init(&f);
    put_block(&f, not_ready_block);
    feed(&f, only_200, sizeof(only_200));
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(sent(&f, not_ready_reply, sizeof(not_ready_reply)));

    /* PING 200 -> status frame, ERROR 0. */
    init(&f);
    feed(&f, ping, sizeof(ping));
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(sent(&f, ack, sizeof(ack)));

    /* Checksum helper agrees with the vectors. */
    CHECK(proto_checksum(ping + 2, 3u) == 0x34u);
    CHECK(proto_checksum(full_sync + 2, (uint16_t)(sizeof(full_sync) - 3u)) == 0x12u);
}

/* ---- sec.8 instructions -------------------------------------------------- */

static void test_ping_and_broadcast(void)
{
    fixture f;
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x01u, 0, 0u);   /* Broadcast PING: silent. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
    feed_frame(&f, 20u, 0x01u, 0, 0u);                  /* PING to a servo: not us. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
    CHECK(f.protocol.rx_packets == 2u);                 /* Both frames were valid. */
}

static void test_read_registers(void)
{
    static const uint8_t head_request[] = {0x00, 0x09};
    static const uint8_t block_request[] = {56u, 15u};
    static const uint8_t bad_range[] = {250u, 10u};
    static const uint8_t zero_length[] = {60u, 0u};
    fixture f;
    uint8_t expected_head[9] = {PROTO_FW_MAJOR, PROTO_FW_MINOR, 0u,
        (uint8_t)(PROTO_MODEL_NUMBER & 0xFFu), (uint8_t)(PROTO_MODEL_NUMBER >> 8),
        PROTO_DEVICE_ID, 0u, 0u, 1u};

    init(&f);
    feed_frame(&f, PROTO_DEVICE_ID, 0x02u, head_request, 2u);
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u && f.output_length == 6u + 9u);
    CHECK(f.output[3] == 11u && f.output[4] == 0u);     /* LEN = 9 + 2, ERROR 0. */
    CHECK(memcmp(f.output + 5, expected_head, 9u) == 0);/* Version 0.2, END 0, ID 200, 1 Mbps, level 1. */
    CHECK(f.output[14] == proto_checksum(f.output + 2, 12u));
    CHECK(f.reads == 0u);                               /* Header never goes to the app. */

    /* A read crossing from the header into the app area: split at address 9. */
    init(&f);
    {
        static const uint8_t crossing[] = {7u, 4u};
        feed_frame(&f, PROTO_DEVICE_ID, 0x02u, crossing, 2u);
    }
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.output_length == 10u && f.output[5] == 0u && f.output[6] == 1u &&
          f.output[7] == 9u && f.output[8] == 10u);
    CHECK(f.reads == 1u);

    init(&f);
    feed_frame(&f, PROTO_DEVICE_ID, 0x02u, block_request, 2u);
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.output_length == 21u && f.output[5] == 56u && f.output[19] == 70u);

    init(&f);                                           /* Out of the table: silent. */
    feed_frame(&f, PROTO_DEVICE_ID, 0x02u, bad_range, 2u);
    feed_frame(&f, PROTO_DEVICE_ID, 0x02u, zero_length, 2u);
    feed_frame(&f, PROTO_DEVICE_ID, 0x02u, bad_range, 1u);  /* Wrong parameter count. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
    CHECK(f.protocol.malformed_packets == 3u);

    init(&f);                                           /* Broadcast READ: never answered. */
    feed_frame(&f, PROTO_BROADCAST_ID, 0x02u, block_request, 2u);
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
}

static void test_write_is_acked_and_ignored(void)
{
    static const uint8_t ack[] = {0xff,0xff,0xc8,0x02,0x00,0x35};
    static const uint8_t torque_off[] = {40u, 0u};
    static const uint8_t new_id[] = {5u, 7u};
    fixture f;
    init(&f);
    feed_frame(&f, PROTO_DEVICE_ID, 0x03u, torque_off, 2u);
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(sent(&f, ack, sizeof(ack)));
    feed_frame(&f, PROTO_DEVICE_ID, 0x03u, new_id, 2u);    /* Even "change ID": ignored. */
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 2u);
    {
        static const uint8_t id_request[] = {5u, 1u};
        feed_frame(&f, PROTO_DEVICE_ID, 0x02u, id_request, 2u);
        poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
        CHECK(f.output[5] == PROTO_DEVICE_ID);
    }
    f.sends = 0u;
    feed_frame(&f, PROTO_BROADCAST_ID, 0x03u, torque_off, 2u);  /* Broadcast: silent. */
    feed_frame(&f, PROTO_BROADCAST_ID, 0x83u, torque_off, 2u);  /* Sync Write: silent. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
    feed_frame(&f, PROTO_DEVICE_ID, 0x03u, torque_off, 1u);     /* No data byte. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.malformed_packets == 1u);
}

static void test_misc_instructions(void)
{
    static const uint8_t ack[] = {0xff,0xff,0xc8,0x02,0x00,0x35};
    static const uint8_t acked[] = {0x04u, 0x05u, 0x06u, 0x09u, 0x0Au, 0x0Bu};
    static const uint8_t reg[] = {42u, 0u, 8u};
    fixture f;
    unsigned i;
    init(&f);
    for (i = 0u; i < sizeof(acked); ++i) {
        f.output_length = 0u;
        feed_frame(&f, PROTO_DEVICE_ID, acked[i], reg, acked[i] == 0x04u ? 3u : 0u);
        poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
        CHECK(sent(&f, ack, sizeof(ack)));
    }
    f.sends = 0u;
    feed_frame(&f, PROTO_DEVICE_ID, 0x7Fu, 0, 0u);          /* Unknown: silent, counted. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.malformed_packets == 1u);
    CHECK(f.protocol.reboot_requested == 0u);
    feed_frame(&f, PROTO_DEVICE_ID, 0x08u, 0, 0u);          /* REBOOT: flag, no reply. */
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.reboot_requested == 1u);
}

/* ---- sec.5 queue rule ---------------------------------------------------- */

static const uint8_t queued_list[] = {56u, 15u, 20u, 21u, PROTO_DEVICE_ID, 22u};

static void test_queue_waits_for_devices_ahead(void)
{
    fixture f;
    uint32_t after_21;
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    CHECK(f.protocol.pending == PROTO_WAIT_TURN && f.protocol.turn == 2u);
    poll_until(&f, f.now + 50u);
    feed_reply(&f, 20u, 15u);
    CHECK(f.protocol.pending == PROTO_WAIT_TURN && f.protocol.slot == 1u);
    poll_until(&f, f.now + 50u);
    CHECK(f.sends == 0u);                                   /* Still 21's turn. */
    feed_reply(&f, 21u, 15u);
    after_21 = f.now;
    CHECK(f.protocol.pending == PROTO_READY);
    poll_until(&f, after_21 + PROTO_RESPONSE_DELAY_US - 1u);
    CHECK(f.sends == 0u);
    poll_until(&f, after_21 + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u && f.output_length == 21u && f.output[2] == PROTO_DEVICE_ID);
    CHECK(f.protocol.skipped_slots == 0u);
}

static void test_queue_skips_silent_devices(void)
{
    fixture f;
    uint32_t start;
    /* 21 offline: after 20 replies the bus stays quiet one slot gap, then we go. */
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    poll_until(&f, f.now + 50u);
    feed_reply(&f, 20u, 15u);
    start = f.now;
    poll_until(&f, start + PROTO_SLOT_GAP_US - 1u);
    CHECK(f.sends == 0u);
    poll_until(&f, start + PROTO_SLOT_GAP_US);
    CHECK(f.sends == 1u && f.protocol.skipped_slots == 1u);

    /* Both ahead offline: two gaps. */
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    start = f.now;
    poll_until(&f, start + 2u * PROTO_SLOT_GAP_US - 1u);
    CHECK(f.sends == 0u);
    poll_until(&f, start + 2u * PROTO_SLOT_GAP_US);
    CHECK(f.sends == 1u && f.protocol.skipped_slots == 2u);

    /* 20 offline but 21 answers late (inside its own gap): 21's frame decides. */
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    poll_until(&f, f.now + PROTO_SLOT_GAP_US + 300u);
    CHECK(f.protocol.slot == 1u && f.sends == 0u);
    feed_reply(&f, 21u, 15u);
    start = f.now;
    poll_until(&f, start + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);
}

static void test_queue_cancelled_by_other_traffic(void)
{
    fixture f;
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    poll_until(&f, f.now + 50u);
    feed_frame(&f, 20u, 0x01u, 0, 0u);                      /* Controller moved on: PING 20. */
    CHECK(f.protocol.pending == PROTO_IDLE && f.protocol.cancelled_replies == 1u);
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);

    /* Review case: Sync Read [20, 200] of 2 bytes, then the controller (not
     * waiting for us) sends READ to 20. That frame has ID 20 and LEN 4 like a
     * 2-byte reply from 20; taking it for one would make us transmit on top of
     * 20's answer to the READ. Byte 4 is an instruction code: new transaction. */
    {
        static const uint8_t short_list[] = {56u, 2u, 20u, PROTO_DEVICE_ID};
        static const uint8_t read_20[] = {56u, 2u};
        init(&f);
        feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, short_list, sizeof(short_list));
        poll_until(&f, f.now + 500u);
        feed_frame(&f, 20u, 0x02u, read_20, sizeof(read_20));
        CHECK(f.protocol.pending == PROTO_IDLE && f.protocol.cancelled_replies == 1u);
        poll_until(&f, f.now + 5000u);
        CHECK(f.sends == 0u);
        /* The same 2-byte reply from 20 with ERROR 0 is a reply: we answer. */
        init(&f);
        feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, short_list, sizeof(short_list));
        feed_reply(&f, 20u, 2u);
        poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
        CHECK(f.sends == 1u);
        /* A reply whose ERROR flags equal an instruction code (0x04, overheat):
         * treated as a new instruction, we stay silent (safe side, documented). */
        init(&f);
        feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, short_list, sizeof(short_list));
        {
            static const uint8_t data[2] = {0x5A, 0x5A};
            feed_frame(&f, 20u, 0x04u, data, sizeof(data));
        }
        poll_until(&f, f.now + 5000u);
        CHECK(f.sends == 0u);
    }

    /* A frame from a device ahead with the wrong length is not "its reply". */
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    feed_reply(&f, 20u, 4u);
    CHECK(f.protocol.pending == PROTO_IDLE);

    /* Continuous noise keeps the slot timer from firing; the transaction expires. */
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    {
        uint32_t start = f.now;
        while (f.now - start < PROTO_TRANSACTION_TIMEOUT_US + 200u) {
            f.now += 100u;
            proto_feed(&f.protocol, 0x55u, f.now);          /* Not a header: dropped. */
            proto_poll(&f.protocol, f.now);
        }
    }
    CHECK(f.sends == 0u && f.protocol.pending == PROTO_IDLE);
    CHECK(f.protocol.skipped_slots == 0u && f.protocol.cancelled_replies == 1u);
}

static void test_sync_read_validation(void)
{
    static const uint8_t duplicate[] = {56u, 15u, PROTO_DEVICE_ID, 20u, 20u};
    static const uint8_t broadcast_in_list[] = {56u, 15u, PROTO_DEVICE_ID, PROTO_BROADCAST_ID};
    static const uint8_t no_ids[] = {56u, 15u};
    static const uint8_t zero_length[] = {56u, 0u, PROTO_DEVICE_ID};
    static const uint8_t not_listed[] = {56u, 15u, 20u, 21u};
    uint8_t far[2u + 70u];
    fixture f;
    unsigned i;
    init(&f);
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, duplicate, sizeof(duplicate));
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, broadcast_in_list, sizeof(broadcast_in_list));
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, no_ids, sizeof(no_ids));
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, zero_length, sizeof(zero_length));
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, not_listed, sizeof(not_listed));
    feed_frame(&f, PROTO_DEVICE_ID, 0x82u, zero_length, sizeof(zero_length)); /* Unicast: ignored. */
    poll_until(&f, f.now + 10000u);
    CHECK(f.sends == 0u);
    CHECK(f.protocol.malformed_packets == 4u);
    /* Beyond the tracked queue depth: silent, counted, never guesses a slot. */
    init(&f);
    far[0] = 56u;
    far[1] = 15u;
    for (i = 0u; i < 70u; ++i) {
        far[2u + i] = (uint8_t)i;
    }
    far[2u + 69u] = PROTO_DEVICE_ID;
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, far, sizeof(far));
    CHECK(f.protocol.pending == PROTO_IDLE && f.protocol.malformed_packets == 1u);
}

/* ---- framing & timing ------------------------------------------------- */

static void test_framing_resync(void)
{
    static const uint8_t ping[] = {0xff,0xff,0xc8,0x02,0x01,0x34};
    static const uint8_t junk_then_ping[] = {0x00,0x13,0xff,0x42,0xff,0xff,0xff,0xc8,0x02,0x01,0x34};
    static const uint8_t bad_checksum[] = {0xff,0xff,0xc8,0x02,0x01,0x35};
    /* A frame whose LEN is corrupted swallows the start of a real PING; the
     * checksum fails and the rescan must still find that PING. */
    static const uint8_t swallowing[] = {0xff,0xff,0x05,0x08,0x03,0x01,0x02,
                                         0xff,0xff,0xc8,0x02,0x01,0x34};
    uint8_t hidden[6];
    fixture f;

    init(&f);
    feed(&f, junk_then_ping, sizeof(junk_then_ping));     /* Garbage and FF FF FF preamble. */
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);

    init(&f);
    feed(&f, bad_checksum, sizeof(bad_checksum));
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.crc_errors == 1u);
    feed(&f, ping, sizeof(ping));
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);

    init(&f);
    feed(&f, swallowing, sizeof(swallowing));
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u && f.protocol.crc_errors >= 1u);

    /* A PING-looking pattern INSIDE a valid frame for ID 5 must not be executed. */
    init(&f);
    memcpy(hidden, ping, sizeof(hidden));
    feed_frame(&f, 5u, 0x03u, hidden, sizeof(hidden));
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.rx_packets == 1u);
}

static void test_byte_gap(void)
{
    static const uint8_t ping[] = {0xff,0xff,0xc8,0x02,0x01,0x34};
    fixture f;
    uint16_t i;
    /* A gap just under the limit inside a frame is fine. */
    init(&f);
    for (i = 0u; i < sizeof(ping); ++i) {
        f.now += (i == 3u) ? PROTO_BYTE_TIMEOUT_US : BYTE_US;
        proto_feed(&f.protocol, ping[i], f.now);
    }
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);
    /* A longer gap drops the partial frame; the next full frame still works. */
    init(&f);
    feed(&f, ping, 3u);
    f.now += PROTO_BYTE_TIMEOUT_US + 1u;
    proto_poll(&f.protocol, f.now);
    CHECK(f.protocol.rx_length == 0u && f.protocol.receive_timeouts == 1u);
    feed(&f, ping + 3, 3u);                                 /* Orphan tail: ignored. */
    f.now += 2000u;
    feed(&f, ping, sizeof(ping));
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US);
    CHECK(f.sends == 1u);
}

static void test_reply_lost_turn_or_expired(void)
{
    static const uint8_t ping[] = {0xff,0xff,0xc8,0x02,0x01,0x34};
    fixture f;
    /* Someone else starts talking before our T_resp: give the turn up. */
    init(&f);
    feed(&f, ping, sizeof(ping));
    f.now += 20u;
    proto_feed(&f.protocol, 0xFFu, f.now);
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.cancelled_replies == 1u);
    /* Same with a byte that is not a frame start (dropped at once, rx stays empty):
     * the turn is still lost, the rx_length guard alone would not catch it. */
    init(&f);
    feed(&f, ping, sizeof(ping));
    f.now += 20u;
    proto_feed(&f.protocol, 0x55u, f.now);
    CHECK(f.protocol.rx_length == 0u && f.protocol.pending == PROTO_IDLE);
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u && f.protocol.cancelled_replies == 1u);
    /* Main loop stalled far past the due time: do not answer into the next transaction. */
    init(&f);
    feed(&f, ping, sizeof(ping));
    f.now += PROTO_RESPONSE_DELAY_US + PROTO_REPLY_EXPIRY_US + 1u;
    proto_poll(&f.protocol, f.now);
    CHECK(f.sends == 0u && f.protocol.cancelled_replies == 1u);
    /* Board fault reset drops pending work. */
    init(&f);
    feed(&f, ping, sizeof(ping));
    proto_reset_receiver(&f.protocol);
    poll_until(&f, f.now + 5000u);
    CHECK(f.sends == 0u);
}

static void test_timestamp_wrap(void)
{
    static const uint8_t ping[] = {0xff,0xff,0xc8,0x02,0x01,0x34};
    fixture f;
    init(&f);
    f.now = 0xFFFFFFF0u;
    feed(&f, ping, sizeof(ping));                           /* Last byte at 0x00000022. */
    poll_until(&f, f.now + PROTO_RESPONSE_DELAY_US - 1u);
    CHECK(f.sends == 0u);
    poll_until(&f, f.now + 1u);
    CHECK(f.sends == 1u);
    /* Queue gap timer across the wrap. */
    init(&f);
    f.now = 0xFFFFFE00u;
    feed_frame(&f, PROTO_BROADCAST_ID, 0x82u, queued_list, sizeof(queued_list));
    poll_until(&f, f.now + 2u * PROTO_SLOT_GAP_US);
    CHECK(f.sends == 1u);
}

int main(void)
{
    test_vectors();
    test_ping_and_broadcast();
    test_read_registers();
    test_write_is_acked_and_ignored();
    test_misc_instructions();
    test_queue_waits_for_devices_ahead();
    test_queue_skips_silent_devices();
    test_queue_cancelled_by_other_traffic();
    test_sync_read_validation();
    test_framing_resync();
    test_byte_gap();
    test_reply_lost_turn_or_expired();
    test_timestamp_wrap();
    printf("Protocol tests: %u checks, %u failures. Context: %u bytes.\n",
           checks, failures, (unsigned)sizeof(proto_context));
    return failures == 0u ? 0 : 1;
}
