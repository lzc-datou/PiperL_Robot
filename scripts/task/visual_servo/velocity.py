"""Differential IK for a verified visual target, including departure from a soft limit."""
import numpy as np
from scipy.optimize import lsq_linear

from servo_tracking.core import damped_joint_velocity, joint_speed_limits
from .geometry import joint_boundary


def tracking_velocity(q, twist, model, config, scale, verified_joints=None):
    velocity = damped_joint_velocity(q, twist, model, config, scale)
    if not np.any(velocity):
        return velocity
    margin = config['joint_margin']
    horizon = max(config['command_timeout'], config['trajectory_horizon'])
    low, high = model.lower + margin, model.upper - margin
    if verified_joints is not None:
        # Leave room for Servo's acceleration smoothing and the controller's
        # queued motion. Taper the extra clearance for verified near-edge goals.
        low = low + np.clip((verified_joints - low) * .5, 0., margin)
        high = high - np.clip((high - verified_joints) * .5, 0., margin)
    jac = model.jacobian(q)
    if (verified_joints is not None and
            joint_boundary(q, low, high, 0., velocity, horizon)):
        # Respect the same predictive soft faces as the final command guard.
        # A joint already on an edge can only leave it toward the verified IK.
        limits = joint_speed_limits(config, scale)
        travel = np.asarray(verified_joints) - q
        inward = np.minimum.reduce([abs(travel) / horizon, limits, np.full(6, max(abs(velocity)))])
        lower = np.maximum(-limits, np.minimum((low + 1e-7 - q) / horizon, inward))
        upper = np.minimum(limits, np.maximum((high - 1e-7 - q) / horizon, -inward))
        weights = np.array([1.] * 3 + [config['dls_rotation_weight']] * 3)
        weighted = weights[:, None] * jac
        desired = weights * twist
        damping = config['dls_damping']
        system = np.vstack((weighted, damping * np.eye(6)))
        if np.all(lower < upper):
            result = lsq_linear(system, np.r_[desired, np.zeros(6)], bounds=(lower, upper), method='bvls')
            if result.success and (weighted @ result.x) @ desired > 0:
                velocity = result.x
    # Joint constraints can trade angular for linear accuracy. Bound the actual
    # FK twist as well as the Cartesian controller's requested twist.
    linear, angular = np.split(jac @ velocity, 2)
    limits = config['control']
    factor = max(1., np.linalg.norm(linear) / (scale * limits['linear_speed']),
                 np.linalg.norm(angular) / (scale * limits['angular_speed']))
    return velocity / factor
