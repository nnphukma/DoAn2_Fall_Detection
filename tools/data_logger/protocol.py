"""Decoder for BLE protocol v2 (includes battery voltage and percentage)."""
from __future__ import annotations

import struct
from dataclasses import dataclass

VERSION = 2
PACKET_SIZE = 67
FRAME_MAGIC = 0xA5
_FORMAT = "<BII12h3hBBBIIIIIHBH"
assert struct.calcsize(_FORMAT) == PACKET_SIZE

CSV_COLUMNS = [
    "sequence", "timestamp_ms", "ax_raw_g", "ay_raw_g", "az_raw_g",
    "gx_raw_dps", "gy_raw_dps", "gz_raw_dps", "ax_ema_g", "ay_ema_g",
    "az_ema_g", "gx_ema_dps", "gy_ema_dps", "gz_ema_dps",
    "accel_magnitude_raw_g", "gyro_magnitude_raw_dps", "posture_angle_deg",
    "posture_valid", "free_fall_condition", "free_fall_trigger", "impact_trigger",
    "rotation_trigger", "posture_condition", "posture_confirmed", "stillness_condition",
    "fsm_state", "fsm_event", "state_elapsed_ms", "posture_elapsed_ms",
    "stillness_elapsed_ms", "countdown_remaining_ms", "device_ble_queue_drops",
    "battery_voltage_v", "battery_percent",
]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def decode_packet(packet: bytes) -> dict:
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"packet length {len(packet)} != {PACKET_SIZE}")
    if packet[0] != VERSION:
        raise ValueError(f"unsupported protocol version {packet[0]}")
    if crc16_ccitt(packet[:-2]) != struct.unpack_from("<H", packet, PACKET_SIZE - 2)[0]:
        raise ValueError("CRC mismatch")

    values = struct.unpack(_FORMAT, packet)
    sequence, timestamp = values[1:3]
    sensor = values[3:18]
    flags, state, event = values[18:21]
    timers = values[21:26]
    battery_mv, battery_percent = values[26:28]

    result = dict(zip(CSV_COLUMNS[:2], (sequence, timestamp)))
    result.update(dict(zip(CSV_COLUMNS[2:8], [v / 1000 for v in sensor[:3]] + [v / 10 for v in sensor[3:6]])))
    result.update(dict(zip(CSV_COLUMNS[8:14], [v / 1000 for v in sensor[6:9]] + [v / 10 for v in sensor[9:12]])))
    result.update(dict(zip(CSV_COLUMNS[14:17], (sensor[12] / 1000, sensor[13] / 10, sensor[14] / 100))))
    result.update(dict(zip(CSV_COLUMNS[17:25], [bool(flags & (1 << bit)) for bit in range(8)])))
    result.update(dict(zip(CSV_COLUMNS[25:32], (state, event, *timers))))
    result["battery_voltage_v"] = battery_mv / 1000.0
    result["battery_percent"] = battery_percent
    return result


@dataclass
class Reassembler:
    frame_id: int | None = None
    count: int = 0
    fragments: dict[int, bytes] | None = None

    def feed(self, data: bytearray) -> bytes | None:
        if len(data) < 7 or data[0] != FRAME_MAGIC or data[1] != VERSION:
            raise ValueError("invalid BLE frame")
        packet_id = data[2] | (data[3] << 8)
        index, count = data[4], data[5]
        if not count or index >= count:
            raise ValueError("invalid fragment indices")
        if packet_id != self.frame_id:
            self.frame_id, self.count, self.fragments = packet_id, count, {}
        if count != self.count:
            raise ValueError("fragment count changed")
        assert self.fragments is not None
        self.fragments[index] = bytes(data[6:])
        if len(self.fragments) != count:
            return None
        packet = b"".join(self.fragments[i] for i in range(count))
        self.frame_id, self.count, self.fragments = None, 0, None
        if len(packet) != PACKET_SIZE:
            raise ValueError("reassembled packet length invalid")
        return packet
