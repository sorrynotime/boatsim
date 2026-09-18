from __future__ import annotations

from dataclasses import dataclass
import struct


SYNC = b"\xA5\x5A"
VERSION = 1
MSG_SENSOR_STATE = 0x01
MSG_TARGET = 0x02
MSG_MOTOR_COMMAND = 0x81
FLAG_GPS_NEW = 0x0001

HEADER = struct.Struct("<2sBBHH")
SENSOR_PAYLOAD = struct.Struct("<IHfiif")
TARGET_PAYLOAD = struct.Struct("<Iiif")
MOTOR_PAYLOAD = struct.Struct("<Ihh")


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    header = HEADER.pack(SYNC, VERSION, message_type, len(payload), sequence & 0xFFFF)
    body = header + payload
    return body + struct.pack("<H", crc16_ccitt(body))


def encode_sensor(
    sequence: int,
    time_ms: int,
    gps_new: bool,
    yaw_rad: float,
    latitude_deg: float,
    longitude_deg: float,
    accuracy_m: float,
) -> bytes:
    payload = SENSOR_PAYLOAD.pack(
        time_ms & 0xFFFFFFFF,
        FLAG_GPS_NEW if gps_new else 0,
        yaw_rad,
        round(latitude_deg * 10_000_000),
        round(longitude_deg * 10_000_000),
        accuracy_m,
    )
    return encode_frame(MSG_SENSOR_STATE, sequence, payload)


def encode_target(sequence: int, time_ms: int, latitude_deg: float, longitude_deg: float, radius_m: float) -> bytes:
    return encode_frame(
        MSG_TARGET,
        sequence,
        TARGET_PAYLOAD.pack(
            time_ms & 0xFFFFFFFF,
            round(latitude_deg * 10_000_000),
            round(longitude_deg * 10_000_000),
            radius_m,
        ),
    )


@dataclass(frozen=True)
class MotorCommand:
    sequence: int
    mcu_time_ms: int
    left: int
    right: int


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, int, bytes]] = []
        while True:
            sync_index = self.buffer.find(SYNC)
            if sync_index < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer.endswith(SYNC[:1]) else b""
                break
            if sync_index:
                del self.buffer[:sync_index]
            if len(self.buffer) < HEADER.size:
                break
            _, version, message_type, payload_length, sequence = HEADER.unpack_from(self.buffer)
            if version != VERSION or payload_length > 256:
                del self.buffer[0]
                continue
            frame_length = HEADER.size + payload_length + 2
            if len(self.buffer) < frame_length:
                break
            candidate = bytes(self.buffer[:frame_length])
            expected_crc = struct.unpack_from("<H", candidate, frame_length - 2)[0]
            if crc16_ccitt(candidate[:-2]) != expected_crc:
                del self.buffer[0]
                continue
            frames.append((message_type, sequence, candidate[HEADER.size:-2]))
            del self.buffer[:frame_length]
        return frames

    def feed_motor_commands(self, data: bytes) -> list[MotorCommand]:
        commands: list[MotorCommand] = []
        for message_type, sequence, payload in self.feed(data):
            if message_type == MSG_MOTOR_COMMAND and len(payload) == MOTOR_PAYLOAD.size:
                time_ms, left, right = MOTOR_PAYLOAD.unpack(payload)
                commands.append(MotorCommand(sequence, time_ms, left, right))
        return commands
