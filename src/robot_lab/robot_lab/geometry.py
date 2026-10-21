"""Geometry helpers shared by every node in this package."""

import math
from collections import namedtuple

# One lidar return: how far, and in which direction (radians, sensor frame).
Reading = namedtuple('Reading', 'range bearing')


def normalize_angle(angle):
    """Wrap an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Extract rotation about Z from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(value, low, high):
    """Constrain value to [low, high]."""
    return max(low, min(value, high))


def sector_min(scan, center_rad, half_width_rad):
    """Nearest valid return within +/- half_width_rad of center_rad.

    Returns a Reading(range, bearing), or None when the sector holds no valid
    data. Readings outside [range_min, range_max] are discarded along with inf and
    nan.
    """
    best = None
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        bearing = normalize_angle(scan.angle_min + i * scan.angle_increment)
        if abs(normalize_angle(bearing - center_rad)) > half_width_rad:
            continue
        if best is None or r < best.range:
            best = Reading(r, bearing)
    return best



def corridor_min(scan, lookahead, half_width):
    """Nearest return inside the rectangular strip straight ahead.

    Returns Reading(range, bearing) for the obstacle with the smallest forward
    distance, or None if the strip is clear.
    """
    best = None
    best_x = None
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
            continue
        bearing = normalize_angle(scan.angle_min + i * scan.angle_increment)
        x = r * math.cos(bearing)
        y = r * math.sin(bearing)
        if 0.0 < x <= lookahead and abs(y) <= half_width:
            if best_x is None or x < best_x:
                best, best_x = Reading(r, bearing), x
    return best


def quaternion_from_yaw(yaw):
    """Build a geometry_msgs-style (x, y, z, w) quaternion for a Z rotation.

    The inverse of yaw_from_quaternion. A ground robot only rotates about Z,
    so the x and y components are always zero and the pair reduces to
    (sin(yaw/2), cos(yaw/2)) -- note the HALF angle, which is the usual
    stumbling block. A yaw of 0 therefore gives (0, 0, 0, 1), the identity.
    """
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
