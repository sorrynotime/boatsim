"""Sonar synthesis for the navigation MCU.

Casts the two transducer beams against the world obstacles (stone islands =
circles, shorelines = polygons) and produces RANGE(0x11) measurements.

Geometry mirrors the firmware reference (nav_engine.c scan_observation):
  - left  beam at yaw - axis_gap/2, transducer lateral offset -sensor_lateral
  - right beam at yaw + axis_gap/2, transducer lateral offset +sensor_lateral
  - transducer origin = hull origin shifted by sensor_forward along the bow
Angles are radians, north-referenced, clockwise positive (same as the boat
model). A channel with no echo reports the configured max range with hit=0,
which is exactly the wire convention of RANGE(0x11).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .collision import CircleObstacle, PolygonObstacle
from .model import BoatState

Point = tuple[float, float]

# Distances below this are treated as "on top of the obstacle" (protocol floor).
MIN_HIT_DISTANCE_M = 0.05


@dataclass(frozen=True)
class SonarConfig:
    """The 12 f32 fields of CONFIG(0x13), in wire order."""
    sensor_range: float = 5.0
    beam_width: float = math.radians(60.0)   # full width
    axis_gap: float = math.radians(40.0)     # angle between the two beams
    sensor_forward: float = 0.15
    sensor_lateral: float = 0.005
    boat_radius: float = 0.80
    margin: float = 0.0
    max_v: float = 1.2
    max_w: float = 1.2
    acceleration: float = 0.5
    angular_acceleration: float = 1.0
    lookahead: float = 1.2


@dataclass(frozen=True)
class SonarSample:
    valid: int   # bit0 left, bit1 right
    hit: int     # subset of valid; 0 on that channel = verified no echo
    left_m: float
    right_m: float
    max_range: float


def ray_circle_first_hit(origin: Point, direction: Point, center: Point, radius: float) -> float | None:
    """Smallest positive ray parameter t hitting the circle, or None."""
    dx, dy = direction
    fx, fy = origin[0] - center[0], origin[1] - center[1]
    a = dx * dx + dy * dy
    if a <= 1e-12:
        return None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    if c <= 0.0:
        return MIN_HIT_DISTANCE_M  # origin inside the obstacle
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return None
    root = math.sqrt(discriminant)
    t = (-b - root) / (2.0 * a)
    if t > 1e-6:
        return t
    t = (-b + root) / (2.0 * a)
    if t > 1e-6:
        return t
    return None


def ray_segment_first_hit(origin: Point, direction: Point, p1: Point, p2: Point) -> float | None:
    """Ray vs one segment: smallest positive t with 0 <= u <= 1, or None."""
    dx, dy = direction
    ex, ey = p2[0] - p1[0], p2[1] - p1[1]
    denominator = dx * ey - dy * ex
    if abs(denominator) <= 1e-12:
        return None
    t = ((p1[0] - origin[0]) * ey - (p1[1] - origin[1]) * ex) / denominator
    u = ((p1[0] - origin[0]) * dy - (p1[1] - origin[1]) * dx) / denominator
    if t > 1e-6 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


def ray_polygon_first_hit(origin: Point, direction: Point, polygon: tuple[Point, ...]) -> float | None:
    best: float | None = None
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        t = ray_segment_first_hit(origin, direction, first, second)
        if t is not None and (best is None or t < best):
            best = t
    return best


def _transducer_origin(state: BoatState, forward: float, lateral: float) -> Point:
    """Hull offset rotated into world coordinates (east, north)."""
    sin_yaw, cos_yaw = math.sin(state.yaw_rad), math.cos(state.yaw_rad)
    east = state.east_m + forward * sin_yaw + lateral * cos_yaw
    north = state.north_m + forward * cos_yaw - lateral * sin_yaw
    return east, north


def transducer_beams(state: BoatState, config: SonarConfig) -> tuple[tuple[Point, float], tuple[Point, float]]:
    """(origin, bearing) of the left and right beam, for UI overlay drawing."""
    left = (_transducer_origin(state, config.sensor_forward, -config.sensor_lateral),
            state.yaw_rad - config.axis_gap / 2.0)
    right = (_transducer_origin(state, config.sensor_forward, config.sensor_lateral),
             state.yaw_rad + config.axis_gap / 2.0)
    return left, right


def _beam_direction(bearing: float) -> Point:
    """Unit vector at a north-referenced clockwise bearing (east, north)."""
    return math.sin(bearing), math.cos(bearing)


def _cast_side(
    origin: Point,
    bearing: float,
    config: SonarConfig,
    obstacles: list[CircleObstacle | PolygonObstacle],
    rays_per_beam: int,
) -> float:
    """Minimum echo distance over the beam fan; config.sensor_range if none."""
    best = config.sensor_range
    half = config.beam_width / 2.0
    for index in range(rays_per_beam):
        if rays_per_beam <= 1:
            offset = 0.0
        else:
            offset = -half + config.beam_width * index / (rays_per_beam - 1)
        direction = _beam_direction(bearing + offset)
        for obstacle in obstacles:
            if isinstance(obstacle, CircleObstacle):
                t = ray_circle_first_hit(origin, direction, (obstacle.east_m, obstacle.north_m), obstacle.radius_m)
            else:
                t = ray_polygon_first_hit(origin, direction, obstacle.points)
            if t is not None and t < best:
                best = t
    return best


def synthesize_sonar(
    state: BoatState,
    config: SonarConfig,
    obstacles: list[CircleObstacle | PolygonObstacle],
    rays_per_beam: int = 3,
) -> SonarSample:
    (left_origin, left_bearing), (right_origin, right_bearing) = transducer_beams(state, config)
    left = _cast_side(left_origin, left_bearing, config, obstacles, rays_per_beam)
    right = _cast_side(right_origin, right_bearing, config, obstacles, rays_per_beam)
    hit = 0
    if left < config.sensor_range - 1e-6:
        hit |= 1
    if right < config.sensor_range - 1e-6:
        hit |= 2
    return SonarSample(
        valid=3,
        hit=hit,
        left_m=left if hit & 1 else config.sensor_range,
        right_m=right if hit & 2 else config.sensor_range,
        max_range=config.sensor_range,
    )
