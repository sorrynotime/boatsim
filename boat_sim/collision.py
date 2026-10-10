from __future__ import annotations

from dataclasses import dataclass
import math


Point = tuple[float, float]


@dataclass(frozen=True)
class CircleObstacle:
    east_m: float
    north_m: float
    radius_m: float


@dataclass(frozen=True)
class PolygonObstacle:
    points: tuple[Point, ...]


@dataclass(frozen=True)
class Collision:
    position: Point
    normal: Point


def _distance_sq(a: Point, b: Point) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def _closest_on_segment(point: Point, start: Point, end: Point) -> Point:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return start
    t = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq))
    return start[0] + t * dx, start[1] + t * dy


def _segment_circle_contact(start: Point, end: Point, center: Point, radius: float) -> Point | None:
    dx, dy = end[0] - start[0], end[1] - start[1]
    fx, fy = start[0] - center[0], start[1] - center[1]
    a = dx * dx + dy * dy
    if a <= 1e-12:
        return end if _distance_sq(end, center) <= radius * radius else None
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - radius * radius
    discriminant = b * b - 4.0 * a * c
    if discriminant < 0.0:
        return end if c <= 0.0 else None
    root = math.sqrt(discriminant)
    candidates = [value for value in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)) if 0.0 <= value <= 1.0]
    if not candidates:
        return end if c <= 0.0 else None
    t = min(candidates)
    return start[0] + t * dx, start[1] + t * dy


def _point_in_polygon(point: Point, polygon: tuple[Point, ...]) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if (current[1] > point[1]) != (previous[1] > point[1]):
            crossing = (previous[0] - current[0]) * (point[1] - current[1]) / (previous[1] - current[1]) + current[0]
            if point[0] < crossing:
                inside = not inside
        previous = current
    return inside


def _polygon_contact(start: Point, end: Point, polygon: tuple[Point, ...], radius: float) -> Collision | None:
    if len(polygon) < 3:
        return None
    nearest = None
    nearest_distance = float("inf")
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        contact = _closest_on_segment(end, first, second)
        distance = math.sqrt(_distance_sq(end, contact))
        if distance < nearest_distance:
            nearest, nearest_distance = contact, distance
        if _segments_intersect(start, end, first, second):
            nearest = contact
            nearest_distance = 0.0
    if nearest is None or (nearest_distance > radius and not _point_in_polygon(end, polygon)):
        return None
    inside = _point_in_polygon(end, polygon)
    nx, ny = (nearest[0] - end[0], nearest[1] - end[1]) if inside else (end[0] - nearest[0], end[1] - nearest[1])
    length = math.hypot(nx, ny)
    if length <= 1e-9:
        cx = sum(point[0] for point in polygon) / len(polygon)
        cy = sum(point[1] for point in polygon) / len(polygon)
        nx, ny = end[0] - cx, end[1] - cy
        length = math.hypot(nx, ny) or 1.0
    normal = (nx / length, ny / length)
    return Collision((nearest[0] + normal[0] * (radius + 1e-4), nearest[1] + normal[1] * (radius + 1e-4)), normal)


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    def orientation(p: Point, q: Point, r: Point) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    ab_c, ab_d = orientation(a, b, c), orientation(a, b, d)
    cd_a, cd_b = orientation(c, d, a), orientation(c, d, b)
    return ((ab_c > 0) != (ab_d > 0)) and ((cd_a > 0) != (cd_b > 0))


def resolve_motion(start: Point, end: Point, boat_radius_m: float, obstacles: list[CircleObstacle | PolygonObstacle]) -> Collision | None:
    for obstacle in obstacles:
        if isinstance(obstacle, CircleObstacle):
            contact = _segment_circle_contact(start, end, (obstacle.east_m, obstacle.north_m), obstacle.radius_m + boat_radius_m)
            if contact is not None:
                nx, ny = contact[0] - obstacle.east_m, contact[1] - obstacle.north_m
                length = math.hypot(nx, ny) or 1.0
                normal = (nx / length, ny / length)
                position = (obstacle.east_m + normal[0] * (obstacle.radius_m + boat_radius_m + 1e-4), obstacle.north_m + normal[1] * (obstacle.radius_m + boat_radius_m + 1e-4))
                return Collision(position, normal)
        else:
            collision = _polygon_contact(start, end, obstacle.points, boat_radius_m)
            if collision is not None:
                return collision
    return None
