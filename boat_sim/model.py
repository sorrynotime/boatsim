from __future__ import annotations

from dataclasses import dataclass
import math
import random


EARTH_RADIUS_M = 6_378_137.0


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def local_to_gps(east_m: float, north_m: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    lat0 = math.radians(lat0_deg)
    latitude = lat0_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = lon0_deg + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(lat0)))
    return latitude, longitude


def gps_to_local(latitude_deg: float, longitude_deg: float, lat0_deg: float, lon0_deg: float) -> tuple[float, float]:
    lat0 = math.radians(lat0_deg)
    north_m = math.radians(latitude_deg - lat0_deg) * EARTH_RADIUS_M
    east_m = math.radians(longitude_deg - lon0_deg) * EARTH_RADIUS_M * math.cos(lat0)
    return east_m, north_m


@dataclass
class BoatParameters:
    mass_kg: float = 5.0
    motor_spacing_m: float = 0.15
    max_speed_mps: float = 1.2
    max_yaw_rate_rps: float = 1.2
    max_motor_thrust_n: float = 35.0
    motor_time_constant_s: float = 0.25


@dataclass
class BoatState:
    east_m: float = 0.0
    north_m: float = 0.0
    yaw_rad: float = 0.0
    speed_mps: float = 0.0
    yaw_rate_rps: float = 0.0


class DifferentialBoat:
    def __init__(self, parameters: BoatParameters | None = None) -> None:
        self.parameters = parameters or BoatParameters()
        self.state = BoatState()
        self.left_actual = 0.0
        self.right_actual = 0.0

    def reset(self) -> None:
        self.state = BoatState()
        self.left_actual = 0.0
        self.right_actual = 0.0

    def step(self, left_command: int, right_command: int, dt: float) -> BoatState:
        p = self.parameters
        left = max(-1.0, min(1.0, left_command / 10_000.0))
        right = max(-1.0, min(1.0, right_command / 10_000.0))
        alpha = 1.0 - math.exp(-dt / max(0.01, p.motor_time_constant_s))
        self.left_actual += (left - self.left_actual) * alpha
        self.right_actual += (right - self.right_actual) * alpha

        left_force = self.left_actual * p.max_motor_thrust_n
        right_force = self.right_actual * p.max_motor_thrust_n
        total_force = left_force + right_force
        linear_drag = (2.0 * p.max_motor_thrust_n / max(0.1, p.max_speed_mps)) * self.state.speed_mps
        acceleration = (total_force - linear_drag) / max(0.1, p.mass_kg)

        torque = (right_force - left_force) * p.motor_spacing_m * 0.5
        inertia = max(0.05, p.mass_kg * p.motor_spacing_m**2 / 4.0)
        max_torque = p.max_motor_thrust_n * p.motor_spacing_m
        angular_drag = max_torque / max(0.05, p.max_yaw_rate_rps)
        angular_acceleration = (torque - angular_drag * self.state.yaw_rate_rps) / inertia

        self.state.speed_mps += acceleration * dt
        self.state.yaw_rate_rps += angular_acceleration * dt
        self.state.yaw_rad = wrap_angle(self.state.yaw_rad + self.state.yaw_rate_rps * dt)
        self.state.east_m += self.state.speed_mps * math.sin(self.state.yaw_rad) * dt
        self.state.north_m += self.state.speed_mps * math.cos(self.state.yaw_rad) * dt
        return self.state


class GpsSensor:
    def __init__(self, seed: int | None = None) -> None:
        self.random = random.Random(seed)

    def sample(
        self,
        state: BoatState,
        lat0_deg: float,
        lon0_deg: float,
        accuracy_radius_m: float,
    ) -> tuple[float, float]:
        # Treat accuracy as an approximate 95% horizontal radius.
        sigma = max(0.0, accuracy_radius_m) / 2.0
        noisy_east = state.east_m + self.random.gauss(0.0, sigma)
        noisy_north = state.north_m + self.random.gauss(0.0, sigma)
        return local_to_gps(noisy_east, noisy_north, lat0_deg, lon0_deg)
