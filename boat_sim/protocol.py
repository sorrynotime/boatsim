from __future__ import annotations

from dataclasses import dataclass
import struct


SYNC = b"\xA5\x5A"
VERSION = 1
MSG_SENSOR_STATE = 0x01
MSG_TARGET = 0x02
MSG_MOTOR_COMMAND = 0x81
MSG_EKF_POSITION = 0x82
FLAG_GPS_NEW = 0x0001

# --- Navigation extension (MCU rs_lib/navigation, see ag3335 UART2_NAV_PROTOCOL docs) ---
MSG_NAV_FRAME = 0x10   # PC -> MCU: session/origin, establishes the nav frame
MSG_NAV_RANGE = 0x11   # PC -> MCU: dual-beam sonar sample (10 Hz)
MSG_NAV_TASK = 0x12    # PC -> MCU: mission command (48 B)
MSG_NAV_CONFIG = 0x13  # PC -> MCU: sensor/boat configuration (60 B)
MSG_NAV_STATUS = 0x83  # MCU -> PC: navigation status (48 B, 10 Hz)
MSG_NAV_PLAN = 0x84    # MCU -> PC: plan waypoint segment (12 + 8N, N <= 6)

NAV_CMD_STOP = 0
NAV_CMD_CLEAR = 1
NAV_CMD_POINT = 2
NAV_CMD_START = 3
NAV_CMD_SURVEY = 4
NAV_CMD_SURVEY_PLAN = 6
NAV_CMD_PLAN_EXECUTE = 7
NAV_CMD_PLAN_RESEND = 8

NAV_COORD_NORTH_EAST = 1
NAV_PROTOCOL_REV = 1
NAV_STATUS_SIZE = 48
NAV_TASK_SIZE = 48
NAV_CONFIG_SIZE = 60
NAV_PLAN_HEADER_SIZE = 12
NAV_PLAN_POINTS_PER_FRAME = 6

HEADER = struct.Struct("<2sBBHH")
SENSOR_PAYLOAD = struct.Struct("<IHffiif")
TARGET_PAYLOAD = struct.Struct("<Iiif")
MOTOR_PAYLOAD = struct.Struct("<Ihh")
EKF_POSITION_PAYLOAD = struct.Struct("<Iii")
NAV_FRAME_PAYLOAD = struct.Struct("<IIiiHBB")
NAV_RANGE_PAYLOAD = struct.Struct("<IIHBBfff")
NAV_TASK_PAYLOAD = struct.Struct("<IIHBBI8f")
NAV_CONFIG_PAYLOAD = struct.Struct("<IIHBB12f")
NAV_STATUS_PAYLOAD = struct.Struct("<IIBBBB6H4I2f")  # 4+4+4B+12+16+8 = 48


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
    yaw_rate_rps: float,
    latitude_deg: float,
    longitude_deg: float,
    accuracy_m: float,
) -> bytes:
    payload = SENSOR_PAYLOAD.pack(
        time_ms & 0xFFFFFFFF,
        FLAG_GPS_NEW if gps_new else 0,
        yaw_rad,
        yaw_rate_rps,
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


def encode_nav_frame(sequence: int, session: int, time_ms: int, latitude_deg: float,
                     longitude_deg: float, frame_id: int = 1) -> bytes:
    return encode_frame(
        MSG_NAV_FRAME,
        sequence,
        NAV_FRAME_PAYLOAD.pack(
            session & 0xFFFFFFFF,
            time_ms & 0xFFFFFFFF,
            round(latitude_deg * 10_000_000),
            round(longitude_deg * 10_000_000),
            frame_id & 0xFFFF,
            NAV_COORD_NORTH_EAST,
            NAV_PROTOCOL_REV,
        ),
    )


def encode_range(sequence: int, session: int, time_ms: int, valid: int, hit: int,
                 left_m: float, right_m: float, max_range_m: float, frame_id: int = 1) -> bytes:
    return encode_frame(
        MSG_NAV_RANGE,
        sequence,
        NAV_RANGE_PAYLOAD.pack(
            session & 0xFFFFFFFF,
            time_ms & 0xFFFFFFFF,
            frame_id & 0xFFFF,
            valid & 0xFF,
            hit & 0xFF,
            left_m,
            right_m,
            max_range_m,
        ),
    )


def encode_nav_task(sequence: int, session: int, time_ms: int, command: int, request: int,
                    args: list[float] | tuple[float, ...] = (), flags: int = 0, frame_id: int = 1) -> bytes:
    values = list(args) + [0.0] * (8 - len(args))
    return encode_frame(
        MSG_NAV_TASK,
        sequence,
        NAV_TASK_PAYLOAD.pack(
            session & 0xFFFFFFFF,
            time_ms & 0xFFFFFFFF,
            frame_id & 0xFFFF,
            command & 0xFF,
            flags & 0xFF,
            request & 0xFFFFFFFF,
            *values[:8],
        ),
    )


def encode_nav_config(sequence: int, session: int, time_ms: int, values: list[float] | tuple[float, ...],
                      frame_id: int = 1) -> bytes:
    fields = list(values) + [0.0] * (12 - len(values))
    return encode_frame(
        MSG_NAV_CONFIG,
        sequence,
        NAV_CONFIG_PAYLOAD.pack(
            session & 0xFFFFFFFF,
            time_ms & 0xFFFFFFFF,
            frame_id & 0xFFFF,
            NAV_PROTOCOL_REV,
            0,
            *fields[:12],
        ),
    )


@dataclass(frozen=True)
class MotorCommand:
    sequence: int
    mcu_time_ms: int
    left: int
    right: int


@dataclass(frozen=True)
class EkfPosition:
    sequence: int
    mcu_time_ms: int
    ekf_latitude_e7: int
    ekf_longitude_e7: int

    @property
    def ekf_latitude_deg(self) -> float:
        return self.ekf_latitude_e7 / 10_000_000.0

    @property
    def ekf_longitude_deg(self) -> float:
        return self.ekf_longitude_e7 / 10_000_000.0


NAV_STATE_NAMES = (
    "IDLE", "READY", "SCAN_LAUNCH", "TRANSIT", "SCAN_CENTER", "FOLLOW",
    "RESCAN", "DONE", "STOPPED", "FAULT", "REVIEW", "PLAN_WAIT",
)

NAV_ACK_NAMES = (
    "OK", "FORMAT", "SESSION", "TIME", "STATE", "CAPACITY", "POSE",
    "CONFIG", "QUEUE", "DUPLICATE",
)


@dataclass(frozen=True)
class NavStatus:
    """STATUS(0x83): navigation engine snapshot (48 B)."""
    sequence: int
    session: int
    last_request: int
    ack: int
    state: int
    running: int
    flags: int          # bit0 engine owns motors, bit1 motion allowed & pose ready
    total: int
    current: int
    visited: int
    shore: int
    skipped: int
    pending: int
    rejected: int
    range_rejected: int
    replans: int
    queue_drops: int
    command_v: float
    command_w: float

    @property
    def state_name(self) -> str:
        return NAV_STATE_NAMES[self.state] if 0 <= self.state < len(NAV_STATE_NAMES) else f"?{self.state}"

    @property
    def ack_name(self) -> str:
        return NAV_ACK_NAMES[self.ack] if 0 <= self.ack < len(NAV_ACK_NAMES) else f"?{self.ack}"


@dataclass(frozen=True)
class PlanSegment:
    """PLAN(0x84): one slice of the surveyed waypoint table (12 + 8N bytes)."""
    sequence: int
    session: int
    plan_id: int
    total: int
    start_index: int
    count: int
    flags: int          # bit0 = final segment (PLAN_END)
    points: tuple[tuple[float, float], ...]   # (north_m, east_m) per point


def decode_nav_status(payload: bytes, sequence: int) -> NavStatus | None:
    if len(payload) != NAV_STATUS_SIZE:
        return None
    fields = NAV_STATUS_PAYLOAD.unpack(payload)
    return NavStatus(sequence, *fields)


def decode_plan_segment(payload: bytes, sequence: int) -> PlanSegment | None:
    if len(payload) < NAV_PLAN_HEADER_SIZE:
        return None
    count = payload[10]
    if count > NAV_PLAN_POINTS_PER_FRAME or len(payload) != NAV_PLAN_HEADER_SIZE + 8 * count:
        return None
    session, plan_id, total, start_index = struct.unpack_from("<IHHH", payload, 0)
    points: list[tuple[float, float]] = []
    for index in range(count):
        north, east = struct.unpack_from("<ff", payload, NAV_PLAN_HEADER_SIZE + 8 * index)
        points.append((north, east))
    return PlanSegment(sequence, session, plan_id, total, start_index, count, payload[11], tuple(points))


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

    def feed_messages(self, data: bytes) -> list[MotorCommand | EkfPosition | NavStatus | PlanSegment]:
        messages: list[MotorCommand | EkfPosition | NavStatus | PlanSegment] = []
        for message_type, sequence, payload in self.feed(data):
            if message_type == MSG_MOTOR_COMMAND and len(payload) == MOTOR_PAYLOAD.size:
                time_ms, left, right = MOTOR_PAYLOAD.unpack(payload)
                messages.append(MotorCommand(sequence, time_ms, left, right))
            elif message_type == MSG_EKF_POSITION and len(payload) == EKF_POSITION_PAYLOAD.size:
                time_ms, latitude_e7, longitude_e7 = EKF_POSITION_PAYLOAD.unpack(payload)
                messages.append(EkfPosition(sequence, time_ms, latitude_e7, longitude_e7))
            elif message_type == MSG_NAV_STATUS:
                message = decode_nav_status(payload, sequence)
                if message is not None:
                    messages.append(message)
            elif message_type == MSG_NAV_PLAN:
                message = decode_plan_segment(payload, sequence)
                if message is not None:
                    messages.append(message)
        return messages

    def feed_motor_commands(self, data: bytes) -> list[MotorCommand]:
        return [message for message in self.feed_messages(data) if isinstance(message, MotorCommand)]
