from __future__ import annotations

from dataclasses import dataclass
import math


MAX_WAYPOINTS = 20


@dataclass(frozen=True)
class Waypoint:
    east_m: float
    north_m: float


class RoutePlan:
    def __init__(self) -> None:
        self.waypoints: list[Waypoint] = []
        self.active_index: int | None = None
        self.completed = False

    @property
    def running(self) -> bool:
        return self.active_index is not None

    @property
    def current(self) -> Waypoint | None:
        return self.waypoints[self.active_index] if self.active_index is not None else None

    def add(self, east_m: float, north_m: float) -> None:
        if len(self.waypoints) >= MAX_WAYPOINTS:
            raise ValueError(f"航线最多支持 {MAX_WAYPOINTS} 个点")
        self.waypoints.append(Waypoint(east_m, north_m))
        self.completed = False

    def delete(self, index: int) -> None:
        if self.running:
            raise ValueError("请先停止航线，再删除航点")
        del self.waypoints[index]
        self.completed = False

    def clear(self) -> None:
        self.stop()
        self.waypoints.clear()
        self.completed = False

    def start(self) -> Waypoint:
        if not self.waypoints:
            raise ValueError("请先添加至少一个航点")
        self.active_index = 0
        self.completed = False
        return self.waypoints[0]

    def stop(self) -> None:
        self.active_index = None

    def observe(self, east_m: float, north_m: float, radius_m: float) -> str | None:
        waypoint = self.current
        if waypoint is None:
            return None
        if math.hypot(east_m - waypoint.east_m, north_m - waypoint.north_m) > radius_m:
            return None
        assert self.active_index is not None
        if self.active_index + 1 < len(self.waypoints):
            self.active_index += 1
            return "advanced"
        self.active_index = None
        self.completed = True
        return "completed"
