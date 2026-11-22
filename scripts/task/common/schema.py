"""Shared task/snapshot schema; no ROS or hardware access."""
from copy import deepcopy
import math
from pathlib import Path
import re
import yaml
from .pose import orientation_error_rad, quaternion_from_rpy_deg

JOINTS = [f'joint{i}' for i in range(1, 7)]
# Covers the combined rounding of a copied monitor point (6 quaternion and 4 RPY decimals).
ORIENTATION_ROUNDING_TOLERANCE_DEG = 0.001
DEFAULTS = dict(frame_id='base_link', tcp_link='tcp_link', group='arm',
                velocity_scaling=0.1, acceleration_scaling=0.1, hold_seconds=0.5,
                return_to_initial=False, planning_time=5.0, planning_attempts=5,
                feedback_timeout=0.5, diagnostic_timeout=1.5, arrival_timeout=15.0, settle_seconds=0.5,
                joint_tolerance_rad=0.02, stable_range_rad=0.003,
                position_tolerance_m=0.003, orientation_tolerance_rad=0.02,
                execution_margin_seconds=10.0, start_tolerance_rad=0.01,
                approach_offset_m=[0.0, 0.0, 0.0])


def number(value, label, *, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label}: expected a finite number')
    try:
        value = float(value)
    except OverflowError as exc:
        raise ValueError(f'{label}: expected a finite number') from exc
    if not math.isfinite(value):
        raise ValueError(f'{label}: expected a finite number')
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f'{label}: outside [{minimum}, {maximum}]')
    return value


def vector(value, count, label):
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f'{label}: expected {count} numbers')
    return [number(x, label) for x in value]


def quaternion(value, label='orientation_xyzw'):
    q = vector(value, 4, label)
    norm = math.sqrt(sum(x*x for x in q))
    if abs(norm-1) > 0.01:
        raise ValueError(f'{label} must be a unit quaternion (x,y,z,w)')
    return [x/norm for x in q]


def mapping(value, label, allowed):
    if not isinstance(value, dict):
        raise ValueError(f'{label}: expected a mapping')
    unknown = set(value)-set(allowed)
    if unknown:
        raise ValueError(f'{label}: unknown fields {sorted(unknown)}')
    return value


def _resolve_pose_orientation(point):
    name = point['name']
    xyzw = None
    if 'orientation_xyzw' in point:
        xyzw = quaternion(point['orientation_xyzw'], f'{name}.orientation_xyzw')
    if 'rpy' not in point:
        if xyzw is None:
            raise ValueError(f'{name}: pose requires orientation_xyzw or rpy')
        return xyzw

    rpy = vector(point.pop('rpy'), 3, f'{name}.rpy (deg)')
    from_rpy = quaternion_from_rpy_deg(rpy)
    if xyzw is None:
        return from_rpy
    difference = math.degrees(orientation_error_rad(xyzw, from_rpy))
    if difference > ORIENTATION_ROUNDING_TOLERANCE_DEG:
        raise ValueError(f'{name}: orientation_xyzw and rpy disagree by {difference:.6f} deg '
                         f'(maximum {ORIENTATION_ROUNDING_TOLERANCE_DEG:g} deg)')
    return xyzw


def validate_config(document):
    data = deepcopy(mapping(document, 'task', ['version', 'settings', 'points', 'capture']))
    if type(data.get('version', 1)) is not int or data.get('version', 1) != 1:
        raise ValueError('Only task version 1 is supported')
    settings = deepcopy(DEFAULTS)
    settings.update(mapping(data.get('settings', {}), 'settings', DEFAULTS))
    for field in ('frame_id', 'tcp_link', 'group'):
        if not isinstance(settings[field], str) or not settings[field].strip():
            raise ValueError(f'{field} must be nonempty')
    if type(settings['return_to_initial']) is not bool:
        raise ValueError('return_to_initial must be true or false')
    for field in ('velocity_scaling', 'acceleration_scaling'):
        settings[field] = number(settings[field], field, minimum=0.000001, maximum=1)
    for field in ('planning_time', 'feedback_timeout', 'diagnostic_timeout', 'arrival_timeout', 'settle_seconds',
                  'joint_tolerance_rad', 'stable_range_rad', 'position_tolerance_m',
                  'orientation_tolerance_rad', 'execution_margin_seconds', 'start_tolerance_rad'):
        settings[field] = number(settings[field], field, minimum=0.000001)
    settings['hold_seconds'] = number(settings['hold_seconds'], 'hold_seconds', minimum=0)
    attempts = settings['planning_attempts']
    if type(attempts) is not int or not 1 <= attempts <= 100:
        raise ValueError('planning_attempts must be an integer in [1,100]')
    settings['approach_offset_m'] = vector(settings['approach_offset_m'], 3, 'approach_offset_m')
    points = data.get('points')
    if not isinstance(points, list) or not points:
        raise ValueError('points is empty: first record or configure your own target points')
    names = set()
    for point in points:
        mapping(point, 'point', ['name', 'type', 'joints_rad', 'position_m', 'orientation_xyzw',
                                 'rpy', 'frame_id', 'hold_seconds'])
        name = point.get('name')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', name) or name in names:
            raise ValueError('Point names must be unique letters/digits/_/-')
        names.add(name)
        if settings['return_to_initial'] and name=='return_to_initial':
            raise ValueError('return_to_initial is reserved when automatic return is enabled')
        point['hold_seconds'] = number(point.get('hold_seconds', settings['hold_seconds']), 'hold_seconds', minimum=0)
        if point.get('type') == 'joint':
            if set(point) & {'position_m', 'orientation_xyzw', 'rpy', 'frame_id'}:
                raise ValueError(f'{name}: joint point contains pose fields')
            point['joints_rad'] = vector(point.get('joints_rad'), 6, 'joints_rad (joint1..joint6)')
        elif point.get('type') == 'pose':
            if 'joints_rad' in point:
                raise ValueError(f'{name}: pose point contains joints_rad')
            point['frame_id'] = point.get('frame_id', settings['frame_id'])
            if point['frame_id'] != settings['frame_id']:
                raise ValueError(f'{name}: frame_id must match task frame_id')
            point['position_m'] = vector(point.get('position_m'), 3, f'{name}.position_m (m)')
            point['orientation_xyzw'] = _resolve_pose_orientation(point)
        else:
            raise ValueError(f'{name}: type must be joint or pose')
    capture = data.get('capture')
    if capture is not None:
        if not isinstance(capture, dict) or capture.get('valid_at_capture') is not True:
            raise ValueError('Snapshot capture metadata is incomplete or invalid')
        if capture.get('joint_names') != JOINTS:
            raise ValueError('Snapshot joint order mismatch')
        if capture.get('frame_id') != settings['frame_id'] or capture.get('tcp_link') != settings['tcp_link']:
            raise ValueError('Snapshot frame/TCP differs from task settings')
    return dict(version=1, settings=settings, points=points, **({'capture': capture} if capture else {}))


def validate_snapshot_model(config, urdf, model_sha256):
    """Check model identity when a whole monitoring snapshot is loaded."""
    capture = config.get('capture', {})
    mismatch = 'Snapshot was recorded with a different robot model/TCP; re-record or explicitly review configuration'
    if capture.get('model_sha256') and capture['model_sha256'] != model_sha256:
        raise RuntimeError(mismatch)
    if capture.get('kinematics_sha256'):
        from scripts.task.servo_tracking.core import RobotModel

        settings = config['settings']
        model = RobotModel(urdf, base=settings['frame_id'], tip=settings['tcp_link'])
        if capture['kinematics_sha256'] != model.kinematics_sha256():
            raise RuntimeError(mismatch)


def read_yaml(path):
    try: return yaml.safe_load(Path(path).read_text())
    except yaml.YAMLError as exc: raise ValueError(f'Invalid YAML: {exc}') from exc


def load_config(path):
    return validate_config(read_yaml(path))


def load_scene(path, frame):
    if path is None:
        return dict(enabled=False, objects=[])
    return validate_scene(read_yaml(path), frame)


def validate_scene(document, frame):
    data = deepcopy(mapping(document, 'scene', ['enabled', 'objects']))
    if type(data.get('enabled')) is not bool:
        raise ValueError('scene.enabled must be true or false')
    objects = data.get('objects', [])
    if not isinstance(objects, list):
        raise ValueError('scene.objects must be a list')
    names = set()
    for obj in objects:
        mapping(obj, 'obstacle', ['id', 'shape', 'dimensions_m', 'position_m', 'orientation_xyzw', 'frame_id'])
        name = obj.get('id')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', name) or name in names:
            raise ValueError('Obstacle ids must be unique letters/digits/_/-')
        names.add(name)
        count = {'box': 3, 'sphere': 1, 'cylinder': 2}.get(obj.get('shape'))
        if count is None:
            raise ValueError('Obstacle shape must be box/sphere/cylinder')
        obj['dimensions_m'] = vector(obj.get('dimensions_m'), count, 'dimensions_m')
        if min(obj['dimensions_m']) <= 0:
            raise ValueError('Obstacle dimensions must be positive')
        obj['position_m'] = vector(obj.get('position_m'), 3, 'position_m')
        obj['orientation_xyzw'] = quaternion(obj.get('orientation_xyzw', [0, 0, 0, 1]))
        obj['frame_id'] = obj.get('frame_id', frame)
        if obj['frame_id'] != frame:
            raise ValueError('Obstacle frame must match task frame')
    return dict(enabled=data['enabled'], objects=objects)
