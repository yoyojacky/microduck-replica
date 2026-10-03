"""Exercise real packet parsing when one sync-read reply is missing or reordered."""
import unittest

from feetech import FeetechBus, BusError


def reply(sid, data=b'\x00\x08', error=0):
    body = bytes([sid, len(data) + 2, error]) + data
    return b'\xff\xff' + body + bytes([~sum(body) & 0xff])


class ReplySerial:
    timeout = 0.02

    def __init__(self, responses):
        self.responses = responses
        self.buffer = bytearray()
        self.sent = []

    def reset_input_buffer(self):
        self.buffer.clear()

    def write(self, data):
        self.sent.append(bytes(data))
        self.buffer.extend(self.responses)
        return len(data)

    def read(self, size=1):
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result


class SyncReadTests(unittest.TestCase):
    def test_missing_reply_does_not_discard_other_servos(self):
        ids = [20, 21, 22, 23, 24, 30, 31, 32, 33, 10, 11, 12, 13, 14]
        for missing in (20, 23, 14):
            with self.subTest(missing=missing):
                port = ReplySerial(b''.join(reply(sid, bytes([sid, 8])) for sid in ids if sid != missing))
                bus = FeetechBus(None, ser=port)
                got = bus.sync_read(ids, 56, 2)
                self.assertEqual([sid for sid, data in got.items() if data is None], [missing])
                for sid in ids:
                    if sid != missing:
                        self.assertEqual(got[sid], (0, bytes([sid, 8])))
                self.assertEqual(bus.stats['bad_id'], 0)
                self.assertEqual(bus.stats['timeout'], 1)
                self.assertEqual([packet[4] for packet in port.sent], [0x82])

    def test_reordered_replies_are_assigned_to_their_actual_ids(self):
        port = ReplySerial(reply(24, b'\x24\x08') + reply(23, b'\x23\x08', error=4))
        bus = FeetechBus(None, ser=port)
        self.assertEqual(bus.sync_read([23, 24], 56, 2), {23: (4, b'\x23\x08'), 24: (0, b'\x24\x08')})
        self.assertEqual(bus.stats['bad_id'], 0)

    def test_corrupt_reply_does_not_hide_the_following_valid_reply(self):
        broken = bytearray(reply(23)); broken[-1] ^= 1
        bus = FeetechBus(None, ser=ReplySerial(bytes(broken) + reply(24)))
        self.assertEqual(bus.sync_read([23, 24], 56, 2), {23: None, 24: (0, b'\x00\x08')})
        self.assertEqual(bus.stats['bad_checksum'], 1)

    def test_unrequested_id_and_wrong_length_are_not_valid_results(self):
        bus = FeetechBus(None, ser=ReplySerial(reply(29) + reply(23, b'\x08')))
        self.assertEqual(bus.sync_read([23, 24], 56, 2), {23: None, 24: None})
        self.assertEqual(bus.stats['bad_id'], 1)

    def test_silent_bus_returns_all_missing_after_one_timeout(self):
        bus = FeetechBus(None, ser=ReplySerial(b''))
        self.assertEqual(bus.sync_read([23, 24], 56, 2), {23: None, 24: None})
        self.assertEqual(bus.stats['timeout'], 1)

    def test_single_servo_read_still_rejects_a_different_id(self):
        bus = FeetechBus(None, ser=ReplySerial(reply(24)))
        with self.assertRaises(BusError):
            bus.read(23, 56, 2)


if __name__ == '__main__':
    unittest.main()
