"""Release-only measured-velocity braking under delayed TCP response.

Software sensitivity analysis, not a PiPER-L dynamics identification. No ROS,
SDK or CAN. Includes delay/response sweeps rather than one favourable plant.
"""
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.task.servo_tracking.cartesian_control import load_control_config
from tests.servo_control_benchmark import simulate, pose_error, target_pose


def metrics(rows, settings):
    times = np.array([r['time'] for r in rows])
    actual = np.array([r['actual'] for r in rows])
    error = np.array([r['error'] for r in rows])
    release = settings['release_time']
    incoming = pose_error(target_pose(release), target_pose(release-settings['dt']))
    axes = [block/max(np.linalg.norm(block), 1e-12) for block in (incoming[:3], incoming[3:])]
    final = target_pose(release)
    after = times >= release
    linear = (actual[after, :3]-final[:3]) @ axes[0]
    angular = np.array([(Rotation.from_quat(q)*Rotation.from_quat(final[3:]).inv()).as_rotvec()
                        for q in actual[after, 3:]]) @ axes[1]
    position = np.linalg.norm(error[:, :3], axis=1)
    orientation = np.linalg.norm(error[:, 3:], axis=1)
    inside = (position <= .002) & (orientation <= np.deg2rad(1.))
    stays_inside = np.logical_and.accumulate(inside[::-1])[::-1]
    indices = np.flatnonzero(after & stays_inside & (times <= times[-1]-.4))
    return dict(position_overshoot_mm=float(max(0., np.max(linear))*1000),
        orientation_overshoot_deg=float(np.rad2deg(max(0., np.max(angular)))),
        settle_s=None if not len(indices) else float(times[indices[0]]-release),
        final_position_error_mm=float(position[-1]*1000),
        final_orientation_error_deg=float(np.rad2deg(orientation[-1])))


def main():
    config = load_control_config()
    results = []
    for algorithm in ('p', 'feedforward_p'):
        for delay in (.04, .10, .16):
            for response in (.07, .15, .25):
                variants, before = {}, {}
                for name, prediction in (('previous', 0.), ('measured_braking', .18)):
                    variant = deepcopy(config)
                    variant['settling_linear_prediction_time'] = prediction
                    variant['settling_angular_prediction_time'] = prediction
                    rows, settings = simulate(algorithm, variant,
                        plant=dict(transport_delay=delay, velocity_time_constant=response), release_gated=True)
                    variants[name] = metrics(rows, settings)
                    before[name] = np.array([r['command'] for r in rows if r['time'] < settings['release_time']])
                np.testing.assert_array_equal(before['previous'], before['measured_braking'])
                results.append(dict(algorithm=algorithm, transport_delay_s=delay,
                    velocity_response_s=response, identical_commands_while_dragging=True, **variants))
    report = dict(software_only=True, hardware_validated=False,
        limitations=['Ideal 6D TCP velocity plant; not MOVE_J firmware dynamics.',
                     'No collision, friction, quantization or link flexibility.',
                     'Delay/response choices are sensitivity cases, not measured robot parameters.'],
        prediction_time_s=.18, results=results)
    output = Path('outputs/servo_tracking/validation/release_measured_braking_20261114.json')
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
