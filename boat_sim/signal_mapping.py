def map_motor_commands(
    left: int,
    right: int,
    *,
    swap: bool = False,
    invert_left: bool = False,
    invert_right: bool = False,
) -> tuple[int, int]:
    """Map MCU channel directions to the simulated port/starboard motors."""
    left = max(-10_000, min(10_000, left))
    right = max(-10_000, min(10_000, right))
    if swap:
        left, right = right, left
    if invert_left:
        left = -left
    if invert_right:
        right = -right
    return left, right


import math


def map_yaw_angle(yaw_rad: float, *, invert: bool = False) -> float:
    if not invert:
        return yaw_rad
    return (-yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
