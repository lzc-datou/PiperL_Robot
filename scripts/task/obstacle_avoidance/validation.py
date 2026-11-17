"""Sample planned splines for inspection only; never produce control commands."""
import math
from scripts.task.common.schema import JOINTS


def interpolate(a, b, t, duration):
    """Match position/velocity/acceleration Hermite interpolation used by JTC."""
    s = t / duration
    result = []
    for i, (p0, p1) in enumerate(zip(a.positions, b.positions)):
        if len(a.velocities) == len(b.velocities) == 6:
            v0, v1 = a.velocities[i]*duration, b.velocities[i]*duration
            if len(a.accelerations) == len(b.accelerations) == 6:
                a0, a1 = a.accelerations[i]*duration**2, b.accelerations[i]*duration**2
                c0, c1, c2 = p0, v0, a0/2
                d, v, acc = p1-c0-c1-c2, v1-c1-2*c2, a1-2*c2
                c3, c4, c5 = 10*d-4*v+acc/2, -15*d+7*v-acc, 6*d-3*v+acc/2
                value = c0+s*(c1+s*(c2+s*(c3+s*(c4+s*c5))))
            else:
                value = (2*s**3-3*s**2+1)*p0+(s**3-2*s**2+s)*v0+(-2*s**3+3*s**2)*p1+(s**3-s**2)*v1
        else:
            value = p0+(p1-p0)*s
        result.append(value)
    return result


def samples(trajectory, time_step=.05, joint_step=.025):
    joint = trajectory.joint_trajectory
    if len(joint.joint_names) != 6 or set(joint.joint_names) != set(JOINTS) or not joint.points:
        raise ValueError('Invalid inspection trajectory')
    order = [joint.joint_names.index(name) for name in JOINTS]
    stamp = lambda p: p.time_from_start.sec+p.time_from_start.nanosec/1e9
    first = joint.points[0]
    yield stamp(first), [first.positions[i] for i in order]
    total = 1
    for a, b in zip(joint.points, joint.points[1:]):
        duration = stamp(b)-stamp(a)
        if duration <= 0:
            raise ValueError('Trajectory time must increase')
        count = max(2, math.ceil(duration/time_step),
                    math.ceil(max(abs(x-y) for x,y in zip(a.positions,b.positions))/joint_step))
        total += count
        if total > 20000:
            raise ValueError('Trajectory exceeds inspection sample limit')
        for index in range(1, count+1):
            elapsed = duration*index/count
            q = interpolate(a, b, elapsed, duration)
            yield stamp(a)+elapsed, [q[i] for i in order]


def inspect(client, trajectory, *, stop_on_collision=False):
    checked = 0
    collisions = []
    for stamp, q in samples(trajectory):
        client.tick()
        response = client.state_validity(q)
        checked += 1
        if not response.valid:
            collisions.append(dict(time=stamp, joints_rad=q,
                contacts=[dict(body1=c.contact_body_1, body2=c.contact_body_2) for c in response.contacts]))
            if stop_on_collision: break
    return dict(valid=not collisions, checked_samples=checked, collisions=collisions,
                time_step_seconds=.05, joint_step_rad=.025, continuous_collision_proof=False)
