"""Joint-bound speed shaping. Pure math; no trajectory or device I/O."""
import math


def limit_joint_velocity(q, velocity, lower, upper, acceleration, margin=.015, latency=.13):
    """Scale the whole request to leave braking distance before soft bounds.

    Solve v*latency + v*v/(2*a) <= remaining distance for each moving joint.
    One common factor preserves the solved joint direction. Inward commands
    remain available at a bound. This is a software model, not a stop guarantee.
    """
    vectors = [tuple(float(v) for v in values) for values in (q, velocity, lower, upper)]
    if any(len(v) != 6 or not all(math.isfinite(x) for x in v) for v in vectors):
        raise ValueError('限位速度计算需要四组六维有限数值')
    acceleration, margin, latency = float(acceleration), float(margin), float(latency)
    if (not all(math.isfinite(v) for v in (acceleration, margin, latency)) or
            acceleration <= 0 or margin < 0 or latency < 0):
        raise ValueError('制动加速度、限位余量或延迟无效')
    q, velocity, lower, upper = vectors
    scale = 1.
    for position, speed, low, high in zip(q, velocity, lower, upper):
        if high - low <= 2 * margin:
            raise ValueError('关节范围不足以容纳限位余量')
        if speed == 0.:
            continue
        distance = max(0., high - margin - position if speed > 0 else position - low - margin)
        root = math.hypot(acceleration * latency, math.sqrt(2 * acceleration * distance))
        denominator = root + acceleration * latency
        allowed = 2 * acceleration * distance / denominator if denominator else 0.
        scale = min(scale, allowed / abs(speed))
    return tuple(v * scale for v in velocity), scale
