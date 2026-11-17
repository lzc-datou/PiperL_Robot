"""Deterministic software comparison, NOT evidence of real-arm performance.

Run from the repository root: python -m tests.servo_control_benchmark --output FILE
The plant is a noise-free 6D TCP velocity actuator: 40 ms transport delay followed
by a 70 ms first-order velocity response. It has no joints, IK, singularities,
collisions, firmware quantization, or physical robot dynamics.
"""
import argparse
from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.task.servo_tracking.cartesian_control import (
    ALGORITHMS, CartesianController, ExecutionFeedback, load_control_config)


PLANT = dict(dt=.01, transport_delay=.04, velocity_time_constant=.07,
             verified_target_rate=30., verification_delay=.02,
             release_time=8., post_release_seconds=6., speed_scale=.5,
             settle_position_m=.002, settle_orientation_rad=math.radians(1.),
             settle_duration=.4)


def target_pose(t):
    """Continuous XYZ and coupled XYZ rotation; final target is held after 8 s."""
    t = max(0., min(float(t), PLANT['release_time']))
    xyz = [.4+.05*math.sin(.9*t), .05*math.sin(.65*t), .35+.03*math.sin(.75*t)]
    quaternion = Rotation.from_euler('xyz', [.2*math.sin(.7*t),
                .2*math.sin(.5*t), .4*math.sin(.8*t)]).as_quat()
    return np.r_[xyz, quaternion]


def pose_error(target, actual):
    return np.r_[target[:3]-actual[:3],
        (Rotation.from_quat(target[3:])*Rotation.from_quat(actual[3:]).inv()).as_rotvec()]


def reversals(values, deadband):
    signs = np.sign(np.asarray(values)[np.abs(values)>deadband])
    return int(np.sum(signs[1:] != signs[:-1]))


def simulate(algorithm, config, plant=None, release_gated=False):
    settings = dict(PLANT, **(plant or {}))
    configuration = deepcopy(config)
    configuration['algorithm'] = algorithm
    controller = CartesianController(configuration)
    dt, release = settings['dt'], settings['release_time']
    delay_steps = round(settings['transport_delay']/dt)
    if abs(delay_steps*dt-settings['transport_delay']) > 1e-12:
        raise ValueError('Benchmark transport delay must be an integer number of samples')
    commands = deque([np.zeros(6) for _ in range(delay_steps)])
    velocity = np.zeros(6)
    actual = target_pose(0.)
    rows = []
    alpha = -math.expm1(-dt/settings['velocity_time_constant'])
    count = round((release+settings['post_release_seconds'])/dt)+1
    for index in range(count):
        now = index*dt
        original_time = max(0., min(release,
            math.floor((now-settings['verification_delay'])*settings['verified_target_rate']+1e-9)
            /settings['verified_target_rate']))
        verified = target_pose(original_time)
        # This plant has no downstream limiter. Report that fact explicitly;
        # its transport delay / first-order response is NOT a saturation flag.
        feedback = None if controller.last_command_time is None else ExecutionFeedback(
            stamp=now, command_time=controller.last_command_time,
            requested_twist=controller.last_command_twist, limited=(False, False),
            effective_twist=controller.last_command_twist)
        command = controller.update(verified, actual, now, original_time, settings['speed_scale'],
            feedforward_active=not release_gated or now < release, execution_feedback=feedback,
            actual_time=now)
        current_target = target_pose(now)
        error = pose_error(current_target, actual)
        rows.append(dict(time=now, actual=actual.copy(), target=current_target,
                         command=command.copy(), error=error))
        commands.append(command)
        delayed = commands.popleft()
        velocity += alpha*(delayed-velocity)
        actual[:3] += velocity[:3]*dt
        actual[3:] = (Rotation.from_rotvec(velocity[3:]*dt)*Rotation.from_quat(actual[3:])).as_quat()
    return rows, settings


def summarize(rows, settings):
    times = np.asarray([r['time'] for r in rows])
    errors = np.asarray([r['error'] for r in rows])
    command = np.asarray([r['command'] for r in rows])
    actual = np.asarray([r['actual'] for r in rows])
    release = settings['release_time']
    tracking = (times >= .75) & (times < release)
    position = np.linalg.norm(errors[:, :3], axis=1)
    orientation = np.linalg.norm(errors[:, 3:], axis=1)
    after = times >= release
    inside = ((position <= settings['settle_position_m']) &
              (orientation <= settings['settle_orientation_rad']))
    stays_inside = np.logical_and.accumulate(inside[::-1])[::-1]
    eligible = np.flatnonzero(after & stays_inside &
        (times <= times[-1]-settings['settle_duration']))
    settling = None if not len(eligible) else float(times[eligible[0]]-release)
    final_pose, previous = target_pose(release), target_pose(release-settings['dt'])
    incoming = pose_error(final_pose, previous)/settings['dt']
    linear_axis = incoming[:3]/max(np.linalg.norm(incoming[:3]), 1e-12)
    angular_axis = incoming[3:]/max(np.linalg.norm(incoming[3:]), 1e-12)
    position_projection = (actual[after, :3]-final_pose[:3])@linear_axis
    orientation_projection = np.asarray([
        (Rotation.from_quat(q)*Rotation.from_quat(final_pose[3:]).inv()).as_rotvec()
        for q in actual[after, 3:]])@angular_axis
    # A fitted temporal offset is a second, explicit measure of following lag.
    fit_rows = [r for r in rows if 1. <= r['time'] < release]
    best = None
    for delay in np.arange(-.1, .801, .01):
        squared = []
        for r in fit_rows[::4]:
            offset_error = pose_error(target_pose(r['time']-delay), r['actual'])
            squared.append(float(offset_error[:3]@offset_error[:3] +
                                 .15**2*(offset_error[3:]@offset_error[3:])))
        candidate = float(np.mean(squared)), float(delay)
        if best is None or candidate < best:
            best = candidate
    result = dict(
        position_lag_rms_mm=float(np.sqrt(np.mean(position[tracking]**2))*1000.),
        position_lag_p95_mm=float(np.percentile(position[tracking], 95)*1000.),
        orientation_lag_rms_deg=float(np.rad2deg(np.sqrt(np.mean(orientation[tracking]**2)))),
        orientation_lag_p95_deg=float(np.rad2deg(np.percentile(orientation[tracking], 95))),
        fitted_pose_delay_ms=round(best[1]*1000., 6),
        release_settling_seconds=settling,
        release_position_overshoot_mm=float(max(0., np.max(position_projection))*1000.),
        release_orientation_overshoot_deg=float(np.rad2deg(max(0., np.max(orientation_projection)))),
        release_position_direction_reversals=reversals(position_projection, .0005),
        release_orientation_direction_reversals=reversals(orientation_projection, math.radians(.1)),
        final_position_error_mm=float(position[-1]*1000.),
        final_orientation_error_deg=float(np.rad2deg(orientation[-1])),
        max_linear_command_mps=float(np.max(np.linalg.norm(command[:, :3], axis=1))),
        max_angular_command_radps=float(np.max(np.linalg.norm(command[:, 3:], axis=1))))
    # Transparent software-only score, used only to choose an initial preset.
    result['selection_score'] = (result['position_lag_rms_mm'] +
        2.*result['orientation_lag_rms_deg'] +
        2.*(settling if settling is not None else settings['post_release_seconds']) +
        result['release_position_overshoot_mm'] + 2.*result['release_orientation_overshoot_deg'])
    return result


def benchmark(config=None):
    config = load_control_config() if config is None else config
    metrics, release_gated_metrics = {}, {}
    for algorithm in ALGORITHMS:
        rows, settings = simulate(algorithm, config)
        metrics[algorithm] = summarize(rows, settings)
        rows, settings = simulate(algorithm, config, release_gated=True)
        release_gated_metrics[algorithm] = summarize(rows, settings)
    matched_config = deepcopy(config)
    for algorithm in ('p', 'feedforward_p'):
        matched_config['gains'][algorithm].update(position_gain=4., orientation_gain=4.,
            position_i_gain=0., orientation_i_gain=0., position_d_gain=0., orientation_d_gain=0.)
    matched_gain = dict(position_gain=4., orientation_gain=4., open_fade={}, release_gated={})
    for algorithm in ('p', 'feedforward_p'):
        for mode, gated in (('open_fade', False), ('release_gated', True)):
            rows, settings = simulate(algorithm, matched_config, release_gated=gated)
            matched_gain[mode][algorithm] = summarize(rows, settings)
    best = min(release_gated_metrics, key=lambda key: release_gated_metrics[key]['selection_score'])
    # Prefer the smaller-state FF+P preset when it is within 5% of the best
    # deterministic score; derivative behavior in a noise-free plant is not a
    # hardware noise/stability evaluation.
    recommended = ('feedforward_p' if release_gated_metrics['feedforward_p']['selection_score'] <=
                   1.05*release_gated_metrics[best]['selection_score'] else best)
    return dict(report_version=4, software_only=True, hardware_validated=False,
        plant=PLANT.copy(), target_path='XYZ sinusoids and XYZ Euler sinusoids in target_pose(); freeze at 8s',
        limitations=['Ideal Cartesian velocity plant; no joint geometry, singularity, collisions or firmware.',
                     'Deterministic noiseless feedback; real-arm stability and latency require separate measurement.',
                     'Each algorithm uses its YAML preset gains; comparisons include those gain differences.'],
        metrics=metrics, metrics_mode='Original five-preset open-fade results retained; no mouse-release gate.',
        release_gated_metrics=release_gated_metrics,
        release_gate='feedforward_active=False at release_time; clears only target velocity, PID remains active.',
        matched_gain=matched_gain,
        matched_gain_note='P and FF+P both use kp=4 for position/orientation, identical speed caps/filter/plant; only feedforward differs.',
        antiwindup=antiwindup_benchmark(config),
        normal_plant_execution_feedback='Fresh matched post-limit command, explicitly unlimited; natural plant delay is not saturation.',
        recommended_algorithm=recommended, recommendation_mode='release_gated_metrics',
        selection_rule='RMS position mm + 2*RMS orientation deg + 2*settling s + overshoot mm + 2*overshoot deg; prefer FF+P within 5%.',
        configuration=deepcopy(config))


def antiwindup_benchmark(config):
    """An explicit downstream limiter demonstrates the PI interface boundary.

    For 2.5s an independent actuator can produce only 4mm/s, below this
    controller's own TCP cap. The limiter then releases. The local-only
    reference deliberately hides that limiter, reproducing the old knowledge
    boundary; it is not a recommended execution-feedback implementation.
    """
    metrics = {}
    for mode in ('local_only_reference', 'standard_tracking', 'clamped_tracking', 'missing_feedback_freeze'):
        configuration = deepcopy(config); configuration['algorithm'] = 'pi'
        configuration['antiwindup_mode'] = 'tracking' if mode == 'standard_tracking' else 'clamped_tracking'
        controller = CartesianController(configuration)
        actual = np.array([0., 0., 0., 0., 0., 0., 1.])
        target = actual.copy(); target[0] = .03
        dt, release = .01, 2.5
        commands = deque([np.zeros(6) for _ in range(4)])
        velocity, previous_effective, previous_limited = np.zeros(6), None, (False, False)
        rows = []
        for index in range(851):
            now = index*dt
            feedback = None
            if controller.last_command_time is not None and mode != 'missing_feedback_freeze':
                reference = controller.last_command_twist
                feedback = ExecutionFeedback(stamp=now, command_time=controller.last_command_time,
                    requested_twist=reference,
                    limited=previous_limited if mode in ('standard_tracking', 'clamped_tracking') else (False, False),
                    effective_twist=previous_effective if mode in ('standard_tracking', 'clamped_tracking') else reference)
            command = controller.update(target, actual, now, 0., .5,
                feedforward_active=False, execution_feedback=feedback)
            effective = command.copy()
            if now < release:
                speed = np.linalg.norm(effective[:3])
                effective[:3] *= min(1., .004/max(speed, 1e-15))
            previous_limited = (np.linalg.norm(effective[:3]-command[:3]) > 1e-12, False)
            previous_effective = effective.copy()
            rows.append((now, float(actual[0]), float(controller._integral[0]), float(command[0])))
            commands.append(effective); delayed = commands.popleft()
            velocity += (-math.expm1(-dt/.07))*(delayed-velocity)
            actual[:3] += velocity[:3]*dt
        trace = np.asarray(rows)
        after = trace[:, 0] >= release
        inside = np.abs(trace[:, 1]-.03) <= .002
        stays = np.logical_and.accumulate(inside[::-1])[::-1]
        settled = np.flatnonzero(after & stays & (trace[:, 0] <= trace[-1, 0]-.4))
        first = np.flatnonzero(after)[0]
        metrics[mode] = dict(
            maximum_positive_integral_while_limited_m_s=float(max(0., np.max(trace[~after, 2]))),
            integral_at_release_m_s=float(trace[first, 2]),
            requested_speed_at_release_mps=float(trace[first, 3]),
            release_position_overshoot_mm=float(max(0., np.max(trace[after, 1]-.03))*1000.),
            release_settling_seconds=None if not len(settled) else float(trace[settled[0], 0]-release),
            final_position_error_mm=float(abs(trace[-1, 1]-.03)*1000.))
    return dict(software_only=True, default_mode='clamped_tracking', own_linear_cap_mps=config['linear_speed']*.5,
        downstream_cap_mps=.004, downstream_cap_release_seconds=2.5,
        plant='40ms transport delay + 70ms first-order velocity response; stationary 30mm position step',
        modes=metrics,
        note='Local-only reference receives a deliberately incomplete unlimited report; tracking receives the real post-limit command. Missing telemetry never permits new I.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config')
    parser.add_argument('--output')
    args = parser.parse_args()
    report = benchmark(load_control_config(args.config))
    rendered = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        Path(args.output).write_text(rendered+'\n')
    print(rendered)


if __name__ == '__main__':
    main()
