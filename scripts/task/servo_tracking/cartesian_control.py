"""ROS-independent verified-pose controller; output is base-frame TCP twist.

Inputs are pose7 [x,y,z,qx,qy,qz,qw] in metres and monotonic seconds.
Output [vx,vy,vz,wx,wy,wz] is in m/s and rad/s about the TCP, expressed in
base_link. This module neither solves IK nor generates joint trajectories.
"""
from copy import deepcopy
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


ALGORITHMS = ('p', 'pi', 'pd', 'feedforward_p', 'feedforward_pd')


@dataclass(frozen=True)
class ExecutionFeedback:
    """Timestamp-aligned downstream capability, never a guessed delay error.

    stamp is the execution-side monotonic receive time. command_time identifies
    the original controller update; requested_twist must match that output.
    effective_twist is its post-limit command, or an explicitly time-aligned
    measured TCP twist. All twists are base_link/TCP [m/s, rad/s]. limited must
    explicitly distinguish linear/angular saturation. A measured velocity lag
    alone is NOT evidence of saturation: the caller must qualify limited.
    """
    stamp: float
    command_time: float
    requested_twist: Sequence[float]
    limited: tuple[bool, bool]
    effective_twist: Sequence[float] | None = None
    source: str = 'command'
    valid: bool = True


def _gains(p, i=0., d=0.):
    return dict(position_gain=p, orientation_gain=p,
                position_i_gain=i, orientation_i_gain=i,
                position_d_gain=d, orientation_d_gain=d)


DEFAULT_CONFIG = dict(
    algorithm='feedforward_p', linear_speed=.30, angular_speed=1.50,
    position_tolerance=.001, orientation_tolerance=.01,
    velocity_filter_time_constant=.04, derivative_filter_time_constant=.04,
    feedforward_hold_time=.04, feedforward_validity=.10,
    integral_position_limit=.05, integral_orientation_limit=.20,
    execution_feedback_timeout=.10, execution_command_timeout=.50,
    observer_stable_window=.16, observer_saturation_window=.08,
    observer_linear_noise=.005, observer_angular_noise=.025, observer_relative_noise=.10,
    antiwindup_tracking_time_constant=.20, antiwindup_mode='clamped_tracking',
    settling_linear_prediction_time=.18, settling_angular_prediction_time=.18,
    measured_velocity_filter_time_constant=.03, measured_velocity_timeout=.10,
    hold_linear_velocity_tolerance=.003, hold_angular_velocity_tolerance=.02,
    linear_acceleration=None, angular_acceleration=None,
    min_dt=.001, max_dt=.10,
    gains=dict(p=_gains(2.), pi=_gains(2., i=.8), pd=_gains(2., d=.08),
               feedforward_p=_gains(4.), feedforward_pd=_gains(4., d=.08)))


def _number(value, name, minimum=0., positive=False):
    if (isinstance(value, (bool, np.bool_)) or
            not isinstance(value, (int, float, np.integer, np.floating)) or
            not math.isfinite(value) or value < minimum or (positive and value == minimum)):
        raise ValueError(f'{name} must be a finite number ' + ('>' if positive else '>=') + f' {minimum}')
    return float(value)


def validate_control_config(config):
    """Return a complete independent configuration, rejecting typo/NaN settings."""
    if not isinstance(config, dict):
        raise ValueError('Cartesian control configuration must be a mapping')
    unknown = set(config)-set(DEFAULT_CONFIG)
    if unknown:
        raise ValueError('Unknown Cartesian control settings: '+', '.join(sorted(map(str, unknown))))
    result = deepcopy(DEFAULT_CONFIG)
    for key, value in config.items():
        if key != 'gains':
            result[key] = deepcopy(value)
    if result['algorithm'] not in ALGORITHMS:
        raise ValueError('Unknown Cartesian control algorithm')
    if result['antiwindup_mode'] not in ('tracking', 'clamped_tracking'):
        raise ValueError('antiwindup_mode must be tracking or clamped_tracking')
    supplied = config.get('gains', {})
    if not isinstance(supplied, dict) or set(supplied)-set(ALGORITHMS):
        raise ValueError('gains must contain known algorithms only')
    for algorithm, values in supplied.items():
        if not isinstance(values, dict) or set(values)-set(result['gains'][algorithm]):
            raise ValueError(f'Invalid gains for {algorithm}')
        result['gains'][algorithm].update(values)
    for algorithm, gains in result['gains'].items():
        for key, value in gains.items():
            gains[key] = _number(value, f'gains.{algorithm}.{key}',
                                 positive=key in ('position_gain', 'orientation_gain'))
    for key in ('linear_speed', 'angular_speed', 'integral_position_limit',
                'integral_orientation_limit', 'min_dt', 'max_dt', 'feedforward_validity',
                'execution_feedback_timeout', 'execution_command_timeout', 'antiwindup_tracking_time_constant',
                'observer_stable_window', 'observer_saturation_window', 'observer_linear_noise',
                'observer_angular_noise', 'observer_relative_noise',
                'measured_velocity_timeout', 'hold_linear_velocity_tolerance', 'hold_angular_velocity_tolerance'):
        result[key] = _number(result[key], key, positive=True)
    for key in ('position_tolerance', 'orientation_tolerance',
                'velocity_filter_time_constant', 'derivative_filter_time_constant', 'feedforward_hold_time',
                'settling_linear_prediction_time', 'settling_angular_prediction_time',
                'measured_velocity_filter_time_constant'):
        result[key] = _number(result[key], key)
    if not result['feedforward_hold_time'] < result['feedforward_validity'] <= .10:
        raise ValueError('feedforward_hold_time must be below feedforward_validity <= 0.1s')
    if result['min_dt'] > result['max_dt']:
        raise ValueError('min_dt must not exceed max_dt')
    if result['orientation_tolerance'] > math.pi:
        raise ValueError('orientation_tolerance must not exceed pi')
    if result['execution_command_timeout'] < result['execution_feedback_timeout']:
        raise ValueError('execution_command_timeout must cover execution_feedback_timeout')
    if result['observer_stable_window'] < result['execution_feedback_timeout']:
        raise ValueError('observer_stable_window must cover execution_feedback_timeout')
    if result['observer_saturation_window'] > result['execution_feedback_timeout']:
        raise ValueError('observer_saturation_window must not exceed execution_feedback_timeout')
    for key in ('linear_acceleration', 'angular_acceleration'):
        if result[key] is not None:
            result[key] = _number(result[key], key, positive=True)
    return result


def load_control_config(path=None):
    path = Path(path) if path is not None else Path(__file__).with_name('config')/'control.yaml'
    return validate_control_config(yaml.safe_load(path.read_text()))


def _pose(value):
    try:
        pose = np.asarray(value, dtype=float).copy()
    except (TypeError, ValueError) as exc:
        raise ValueError('Pose must contain seven finite values') from exc
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError('Pose must contain seven finite values')
    length = float(np.linalg.norm(pose[3:]))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError('Pose quaternion must be nonzero and finite')
    pose[3:] /= length
    return pose


def _cap(value, maximum):
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm):
        raise ValueError('Cartesian controller arithmetic overflow')
    return value * min(1., maximum/max(norm, 1e-15))


def _alpha(dt, time_constant):
    return 1. if time_constant == 0. else -math.expm1(-dt/time_constant)


class CartesianController:
    """Stateful P/PI/PD with optional filtered target-velocity feedforward.

    Feed only the most recent verified target. target_time is its original
    receipt time, not the IK completion time or the current control tick.
    Call reset after stop, target alignment, mode change or a control restart.
    """
    def __init__(self, config):
        self.config = validate_control_config(config)
        self.reset()

    def reset(self):
        self._last_now = self._target_time = None
        self._target_pose = None
        self._target_velocity = np.zeros(6)
        self._integral = np.zeros(6)
        self._derivative = np.zeros(6)
        self._previous_error = None
        self._previous_rotation_error = None
        self._last_output = np.zeros(6)
        self._command_history = deque(maxlen=512)
        self._session_start = None
        self._execution_stamp = self._execution_command_time = None
        self.antiwindup_state = 'waiting_feedback'
        self.antiwindup_limited = (False, False)
        self._actual_time = self._actual_pose = self._actual_received = None
        self._measured_velocity = np.zeros(6)
        self._motion_valid = False
        self.settling_braking = False

    def pause_updates(self):
        """Keep the integral but exclude time with no authorized tracking update."""
        self._last_now = self._previous_error = None

    def reset_angular_reference(self, orientation):
        """Retire the prior tilt reference without discarding translation control."""
        for state in (self._integral, self._derivative, self._last_output, self._target_velocity):
            state[3:] = 0.
        if self._previous_error is not None:
            self._previous_error[3:] = 0.
            self._previous_rotation_error = Rotation.identity()
        if self._target_pose is not None:
            self._target_pose[3:] = orientation

    @property
    def measured_velocity(self):
        return self._measured_velocity.copy() if self._motion_valid else None

    @property
    def motion_settled(self):
        return bool(self._motion_valid and
            np.linalg.norm(self._measured_velocity[:3]) <= self.config['hold_linear_velocity_tolerance'] and
            np.linalg.norm(self._measured_velocity[3:]) <= self.config['hold_angular_velocity_tolerance'])

    def _observe_motion(self, actual, actual_time, now):
        """Estimate actual TCP velocity, never a derivative of the target.

        actual_time is the original measurement's source time. Only differences
        within that clock are used; it may be wall/ROS time, unlike ``now``.
        Repeating a source sample cannot create velocity or refresh its age.
        """
        self.settling_braking = False
        if actual_time is None:
            self._motion_valid = False
            return
        stamp = _number(actual_time, 'actual_time')
        if self._actual_time is not None and stamp < self._actual_time:
            self._actual_time = self._actual_pose = self._actual_received = None
            self._measured_velocity[:] = 0.
            self._motion_valid = False
        if self._actual_time is None:
            self._actual_time, self._actual_pose, self._actual_received = stamp, actual.copy(), now
        elif stamp > self._actual_time:
            dt = stamp-self._actual_time
            if dt > self.config['max_dt']:
                self._measured_velocity[:] = 0.
                self._motion_valid = False
            elif dt >= self.config['min_dt']:
                change = np.r_[actual[:3]-self._actual_pose[:3],
                    (Rotation.from_quat(actual[3:])*Rotation.from_quat(self._actual_pose[3:]).inv()).as_rotvec()]
                velocity = change/dt
                if not np.isfinite(velocity).all():
                    raise ValueError('Measured velocity arithmetic overflow')
                self._measured_velocity += _alpha(dt, self.config['measured_velocity_filter_time_constant'])*(velocity-self._measured_velocity)
                self._motion_valid = True
            else:
                # Accumulate very short intervals against the same anchor,
                # without allowing a slowly advancing source to renew old speed.
                if now-self._actual_received > self.config['measured_velocity_timeout']:
                    self._motion_valid = False
                    self._measured_velocity[:] = 0.
                return
            self._actual_time, self._actual_pose, self._actual_received = stamp, actual.copy(), now
        if now-self._actual_received > self.config['measured_velocity_timeout']:
            self._motion_valid = False
            self._measured_velocity[:] = 0.

    @property
    def last_command_time(self):
        return self._last_now

    @property
    def last_command_twist(self):
        return self._last_output.copy()

    def _execution_feedback(self, feedback, now):
        """Validate telemetry without letting a bad sample disable P/D control."""
        if feedback is None:
            return None, 'feedback_missing'
        if not isinstance(feedback, ExecutionFeedback):
            return None, 'feedback_invalid'
        try:
            if not isinstance(feedback.valid, (bool, np.bool_)) or not feedback.valid:
                raise ValueError('invalid')
            stamp = _number(feedback.stamp, 'execution stamp')
            command_time = _number(feedback.command_time, 'execution command_time')
            if (stamp > now+1e-6 or command_time > stamp+1e-6 or
                    feedback.source not in ('command', 'measured')):
                raise ValueError('invalid execution clock/source')
            if len(feedback.limited) != 2 or not all(isinstance(x, (bool, np.bool_)) for x in feedback.limited):
                raise ValueError('invalid saturation flags')
            def twist(value):
                result = np.asarray(value, dtype=float)
                if result.shape != (6,) or not np.isfinite(result).all() or not math.isfinite(np.linalg.norm(result)):
                    raise ValueError('invalid execution twist')
                return result.copy()
            requested = twist(feedback.requested_twist)
            effective = None if feedback.effective_twist is None else twist(feedback.effective_twist)
            if now-stamp > self.config['execution_feedback_timeout']:
                return None, 'feedback_stale'
            if now-command_time > self.config['execution_command_timeout']:
                return None, 'feedback_command_expired'
            if self._session_start is not None and command_time < self._session_start:
                return None, 'feedback_before_reset'
            if (self._execution_stamp is not None and stamp < self._execution_stamp-1e-6 or
                    self._execution_command_time is not None and command_time < self._execution_command_time-1e-6):
                return None, 'feedback_out_of_order'
            while (self._command_history and
                   now-self._command_history[0][0] > self.config['execution_command_timeout']+1e-6):
                self._command_history.popleft()
            if not self._command_history:
                return None, 'feedback_unmatched'
            matched_time, matched_output = min(self._command_history, key=lambda item: abs(item[0]-command_time))
            if (abs(matched_time-command_time) > 1e-6 or
                    not np.allclose(requested, matched_output, rtol=1e-5, atol=1e-7)):
                return None, 'feedback_unmatched'
            self._execution_stamp, self._execution_command_time = stamp, command_time
            return dict(requested=requested, effective=effective,
                        limited=tuple(bool(x) for x in feedback.limited)), 'feedback_valid'
        except (ValueError, TypeError, OverflowError):
            return None, 'feedback_invalid'

    @staticmethod
    def _unwind_only(integral, step):
        """Unknown capacity may unwind old I toward zero, never create new I."""
        decreasing = integral*step < 0.
        candidate = integral+step
        return np.where(decreasing, np.sign(integral)*np.maximum(0., np.sign(integral)*candidate), integral)

    def _integral_step(self, error, elapsed, feedback):
        step = error*elapsed
        if feedback is None:
            return self._unwind_only(self._integral, step)
        integral = self._integral+step
        for index, block in enumerate((slice(0, 3), slice(3, 6))):
            if not feedback['limited'][index]:
                continue
            effective = feedback['effective']
            residual = None if effective is None else feedback['requested'][block]-effective[block]
            if residual is None or np.linalg.norm(residual) < 1e-12:
                integral[block] = self._unwind_only(self._integral[block], step[block])
            elif np.dot(error[block], residual) > 0.:
                integral[block] = self._integral[block]
        return integral

    def _feedforward(self, target, now, target_time):
        config = self.config
        if self._target_time is not None and target_time > self._target_time:
            dt = target_time-self._target_time
            if dt <= config['max_dt']:
                change = np.r_[target[:3]-self._target_pose[:3],
                    (Rotation.from_quat(target[3:]) * Rotation.from_quat(self._target_pose[3:]).inv()).as_rotvec()]
                alpha = _alpha(dt, config['velocity_filter_time_constant'])
                self._target_velocity += alpha*(change/max(dt, config['min_dt'])-self._target_velocity)
            else:
                self._target_velocity[:] = 0.
        if self._target_time is None or target_time > self._target_time:
            self._target_time, self._target_pose = target_time, target.copy()
        age = max(0., now-target_time)
        hold, expiry = config['feedforward_hold_time'], config['feedforward_validity']
        if age <= hold:
            weight = 1.
        elif age >= expiry-1e-12:
            weight = 0.
        else:
            weight = .5*(1.+math.cos(math.pi*(age-hold)/(expiry-hold)))
        return self._target_velocity*weight

    def _limit(self, values, dt, scale):
        config = self.config
        output = np.r_[_cap(values[:3], config['linear_speed']*scale),
                       _cap(values[3:], config['angular_speed']*scale)]
        for block, name in ((slice(0, 3), 'linear_acceleration'), (slice(3, 6), 'angular_acceleration')):
            maximum = config[name]
            if maximum is not None:
                output[block] = self._last_output[block] + _cap(
                    output[block]-self._last_output[block], maximum*scale*dt)
        # A reduced operator speed cap takes priority over a previous output.
        return np.r_[_cap(output[:3], config['linear_speed']*scale),
                     _cap(output[3:], config['angular_speed']*scale)]

    def update(self, target_pose7, actual_pose7, now, target_time, speed_scale,
               feedforward_active=True, execution_feedback=None, actual_time=None, *, linear_feedforward=None):
        """Compute a twist with optional source-timed release braking.

        feedforward_active=False immediately clears the velocity estimate while
        retaining target/time validation and normal P/I/D feedback correction.
        With actual_time, actual pose differences estimate measured velocity;
        release enables terminal damping, without target-derivative kick.
        PI needs fresh ExecutionFeedback to grow I. Missing/stale telemetry
        freezes growth while still allowing existing I to unwind toward zero.
        """
        target, actual = _pose(target_pose7), _pose(actual_pose7)
        now, target_time = _number(now, 'now'), _number(target_time, 'target_time')
        scale = _number(speed_scale, 'speed_scale')
        if not isinstance(feedforward_active, (bool, np.bool_)):
            raise ValueError('feedforward_active must be a boolean')
        if linear_feedforward is not None:
            linear_feedforward = np.asarray(linear_feedforward, dtype=float)
            if linear_feedforward.shape != (3,) or not np.isfinite(linear_feedforward).all():
                raise ValueError('linear_feedforward must have three finite components')
        if scale > 1. or target_time > now+1e-9:
            raise ValueError('speed_scale must be <= 1 and target_time must not be in the future')
        if self._last_now is not None and now <= self._last_now:
            raise ValueError('Control time must advance monotonically')
        if self._target_time is not None and target_time < self._target_time:
            raise ValueError('Verified target time must not move backwards')
        if self._target_time == target_time:
            distance = np.linalg.norm(target[:3]-self._target_pose[:3])
            angle = np.linalg.norm((Rotation.from_quat(target[3:]) *
                                   Rotation.from_quat(self._target_pose[3:]).inv()).as_rotvec())
            if distance > 1e-12 or angle > 1e-12:
                raise ValueError('A changed verified target requires a new target_time')
        elapsed = 0. if self._last_now is None else now-self._last_now
        if elapsed > self.config['max_dt']:
            self.reset()  # No old integral, derivative or velocity can bridge a timing gap.
            elapsed = 0.
        if self._session_start is None:
            self._session_start = now
        self._observe_motion(actual, actual_time, now)
        config, algorithm = self.config, self.config['algorithm']
        rotation_error = Rotation.from_quat(target[3:])*Rotation.from_quat(actual[3:]).inv()
        raw_error = np.r_[target[:3]-actual[:3], rotation_error.as_rotvec()]
        error = raw_error.copy()
        arrived = (np.linalg.norm(error[:3]) <= config['position_tolerance'],
                   np.linalg.norm(error[3:]) <= config['orientation_tolerance'])
        if arrived[0]:
            error[:3] = 0.
            self._integral[:3] = 0.
        if arrived[1]:
            error[3:] = 0.
            self._integral[3:] = 0.
        if self._previous_error is not None and elapsed > 0.:
            dt = max(elapsed, config['min_dt'])
            derivative = np.r_[(raw_error[:3]-self._previous_error[:3])/dt,
                (rotation_error*self._previous_rotation_error.inv()).as_rotvec()/dt]
            self._derivative += _alpha(elapsed, config['derivative_filter_time_constant'])*(derivative-self._derivative)
        feedforward = self._feedforward(target, now, target_time)
        if not feedforward_active:
            self._target_velocity[:] = 0.
            feedforward = np.zeros(6)
        gains = config['gains'][algorithm]
        kp = np.array([gains['position_gain']]*3 + [gains['orientation_gain']]*3)
        ki = np.array([gains['position_i_gain']]*3 + [gains['orientation_i_gain']]*3)
        kd = np.array([gains['position_d_gain']]*3 + [gains['orientation_d_gain']]*3)
        feedback, feedback_state = self._execution_feedback(execution_feedback, now)
        self.antiwindup_limited = feedback['limited'] if feedback is not None else (False, False)
        self.antiwindup_state = feedback_state if algorithm == 'pi' else 'inactive'
        integral = self._integral_step(error, elapsed, feedback) if algorithm == 'pi' and scale else np.zeros(6)
        integral = np.r_[_cap(integral[:3], config['integral_position_limit']),
                         _cap(integral[3:], config['integral_orientation_limit'])]
        base = kp*error
        if algorithm in ('pd', 'feedforward_pd'):
            base += kd*self._derivative
        if algorithm.startswith('feedforward_'):
            base += feedforward
        if linear_feedforward is not None:
            base[:3] += linear_feedforward
        if not feedforward_active and self._motion_valid:
            # On release, use actual motion to brake before the measured pose
            # crosses the target: Kp*(error - T*measured_velocity). Unlike a
            # target-error derivative, a target step cannot cause a D kick.
            correction = np.zeros(6)
            for block, name, tolerance, velocity_tolerance in (
                    (slice(0, 3), 'settling_linear_prediction_time', 'position_tolerance', 'hold_linear_velocity_tolerance'),
                    (slice(3, 6), 'settling_angular_prediction_time', 'orientation_tolerance', 'hold_angular_velocity_tolerance')):
                velocity = self._measured_velocity[block]
                if block.start == 0 and linear_feedforward is not None:
                    # Brake relative to the moving board. On reversal the old
                    # direction needs damping even while feedforward is active.
                    velocity = velocity-linear_feedforward
                speed = float(np.linalg.norm(velocity))
                horizon = config[name]
                if not horizon or speed <= config[velocity_tolerance]:
                    continue
                distance = float(np.linalg.norm(raw_error[block]))
                risk_distance = horizon*speed + config[tolerance]
                # Full damping near the predicted crossing; blend out over a
                # second stopping distance. Far-goal pursuit remains unchanged.
                weight = (1. if np.dot(raw_error[block], velocity) < 0. else
                          float(np.clip(2.-distance/max(risk_distance, 1e-15), 0., 1.)))
                correction[block] = weight*kp[block]*horizon*velocity
            base -= correction
            self.settling_braking = bool(np.any(correction))
        proposed = base + ki*integral
        dt = max(elapsed, config['min_dt'])
        output = self._limit(proposed, dt, scale)
        # Conditional integration: do not accumulate error that pushes further
        # into either a velocity or optional Cartesian acceleration saturation.
        if algorithm == 'pi':
            for block in (slice(0, 3), slice(3, 6)):
                if np.dot(error[block], proposed[block]-output[block]) > 1e-15:
                    integral[block] = self._integral[block]
            if feedback is not None:
                self.antiwindup_state = ('tracking' if feedback['effective'] is not None else
                                        'limited_freeze') if any(feedback['limited']) else 'normal'
                if feedback['effective'] is not None and scale:
                    alpha = _alpha(elapsed, config['antiwindup_tracking_time_constant'])
                    for index, block in enumerate((slice(0, 3), slice(3, 6))):
                        if feedback['limited'][index]:
                            delta = alpha*(feedback['effective'][block]-feedback['requested'][block])
                            correction = np.divide(delta, ki[block], out=np.zeros(3), where=ki[block]>0.)
                            if config['antiwindup_mode'] == 'clamped_tracking':
                                # Back-calculation only removes existing I that
                                # pushes into a reported limit. It cannot invent
                                # opposite I to cancel a saturated P term. Normal
                                # reverse-error integration above may cross zero.
                                integral[block] = self._unwind_only(integral[block], correction)
                            else:
                                integral[block] += correction
            integral[ki == 0.] = 0.
            for inside, block in zip(arrived, (slice(0, 3), slice(3, 6))):
                if inside:
                    integral[block] = 0.
            integral = np.r_[_cap(integral[:3], config['integral_position_limit']),
                             _cap(integral[3:], config['integral_orientation_limit'])]
            output = self._limit(base+ki*integral, dt, scale)
        if not np.all(np.isfinite(output)):
            raise ValueError('Cartesian controller produced nonfinite output')
        self._integral = integral
        self._previous_error, self._previous_rotation_error = raw_error.copy(), rotation_error
        self._last_now, self._last_output = now, output.copy()
        self._command_history.append((now, output.copy()))
        return output
