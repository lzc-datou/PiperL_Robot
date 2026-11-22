"""Differential IK with per-direction joint travel and braking constraints."""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from servo_tracking.core import damped_joint_velocity, joint_speed_limits
from scripts.tools.gamepad_control.motion import braking_speed
from .geometry import joint_boundary


@dataclass(frozen=True)
class StoppingEnvelope:
    lower: np.ndarray
    upper: np.ndarray
    braking: str
    hold: str


def stopping_envelope(q, model, config, *, reference=None, motion=()):
    """Estimate the remaining travel from measured and queued joint motion.

    Each motion sample is (position, velocity, remaining reaction time). Servo's
    acceleration filter can scale all braking accelerations together. Use the
    slowest stopping time for the whole velocity vector in that case.
    """
    lower = q if reference is None else np.minimum(q, reference)
    upper = q if reference is None else np.maximum(q, reference)
    acceleration, margin = config['joint_acceleration'], config['joint_margin']
    moving_threshold = config.get('hold_joint_stability', .003)/config.get('hold_settle_time', .30)
    braking, hold = [], []
    for position, velocity, reaction_time in motion:
        stop_time = np.max(np.abs(velocity))/acceleration
        immediate = position + velocity*(.5*stop_time)
        delayed = immediate + velocity*reaction_time
        lower, upper = np.minimum(lower, delayed), np.maximum(upper, delayed)
        moving = np.where(abs(velocity) >= moving_threshold, velocity, 0.)
        braking.extend(joint_boundary(delayed, model.lower, model.upper, margin, moving).split('、'))
        hold.extend(joint_boundary(immediate, model.lower, model.upper, 0., moving).split('、'))
    return StoppingEnvelope(lower, upper, '、'.join(dict.fromkeys(filter(None, braking))),
                            '、'.join(dict.fromkeys(filter(None, hold))))


def tracking_step(q, twist, model, config, scale, verified_joints=None, *, reference=None,
                  position_first=False, envelope=None):
    nominal = damped_joint_velocity(q, twist, model, config, scale)
    if not np.any(nominal):
        return nominal, 1.

    # Budget the current motion before allowing any additional outward speed.
    if envelope is None:
        envelope = stopping_envelope(q, model, config, reference=reference)
    q_low, q_high = envelope.lower, envelope.upper
    margin = config['joint_margin']
    low, high = model.lower+margin, model.upper-margin
    latency, acceleration = config['command_timeout'], config['joint_acceleration']
    limits = joint_speed_limits(config, scale)
    backward = np.minimum(limits, [braking_speed(d, acceleration, latency) for d in q_low-low])
    forward = np.minimum(limits, [braking_speed(d, acceleration, latency) for d in high-q_high])
    if verified_joints is not None:
        # Keep clearance for Servo smoothing. Taper it for an IK-verified
        # near-edge target and allow inward departure from a physical zero.
        low += np.clip((verified_joints-low)*.5, 0., margin)
        high -= np.clip((high-verified_joints)*.5, 0., margin)
        horizon = max(latency, config['trajectory_horizon'])
        backward = np.minimum(backward, np.maximum(0., q_low-low)/horizon)
        forward = np.minimum(forward, np.maximum(0., high-q_high)/horizon)

    available = np.where(nominal < 0., backward, forward)
    ratio = float(np.min(np.divide(available, abs(nominal), out=np.ones(6), where=nominal != 0.)))
    jac = model.jacobian(q)
    velocity = nominal
    if ratio < 1. and (verified_joints is not None or reference is not None):
        lower, upper = -backward, forward.copy()
        if verified_joints is not None:
            # Prefer departure toward the verified joint branch at a physical
            # edge. A zero-margin joint may otherwise remain stuck at zero.
            inward = np.minimum(abs(verified_joints-q)/horizon, np.max(abs(nominal)))
            toward_high = np.where(verified_joints > q, inward, 0.)
            toward_low = np.where(verified_joints < q, inward, 0.)
            lower = np.maximum(lower, np.minimum.reduce(
                [(low+1e-7-q)/horizon, toward_high, forward*.999999]))
            upper = np.minimum(upper, np.maximum.reduce(
                [(high-1e-7-q)/horizon, -toward_low, -backward*.999999]))
        free = upper > lower
        velocity = np.zeros(6)
        if np.any(free):
            angular_weight = config['dls_rotation_weight']
            weights = np.r_[np.ones(3), np.full(3, angular_weight)]
            desired = weights*twist
            damping = config['dls_damping']*np.eye(np.count_nonzero(free))

            def solve(weights):
                system = np.vstack((weights[:, None]*jac[:, free], damping))
                result = lsq_linear(system, np.r_[weights*twist, np.zeros(damping.shape[0])],
                                    bounds=(lower[free], upper[free]), method='bvls')
                step = np.zeros(6)
                if result.success:
                    step[free] = result.x
                return step

            velocity = solve(weights)
            if position_first and np.any(twist[:3]):
                priority = solve(weights*np.array([1., 1., 1., .35, .35, .35]))
                realized = jac @ priority
                # Prioritize translation only if it gains progress without
                # reversing the requested correction direction.
                if (realized[:3] @ twist[:3] > (jac[:3] @ velocity) @ twist[:3] and
                        realized[3:] @ twist[3:] >= -1e-10):
                    velocity = priority
            if (weights*(jac @ velocity)) @ desired <= 0.:
                velocity[:] = 0.

    linear, angular = np.split(jac @ velocity, 2)
    limits = config['control']
    factor = max(1., np.linalg.norm(linear) / (scale * limits['linear_speed']),
                 np.linalg.norm(angular) / (scale * limits['angular_speed']))
    return velocity/factor, min(1., ratio)


def tracking_velocity(q, twist, model, config, scale, verified_joints=None):
    return tracking_step(q, twist, model, config, scale, verified_joints)[0]
