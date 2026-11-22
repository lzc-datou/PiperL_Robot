#!/usr/bin/env python3
"""Gamepad acceptance against isolated ROS mock/Servo/guard; never a real driver.

Run through scripts/piperl/run.sh with an unused non-default ROS domain. Results
and owned-process cleanup reports are kept in a unique outputs directory.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from piper_servo_tracking.msg import GamepadInput
from piper_servo_tracking.srv import SelectGamepadControl
from scripts.task.servo_tracking.core import JOINTS, RobotModel
from scripts.tools.gamepad_control.core import CARTESIAN_AXES, CONTROL_MODES
from scripts.tools.gamepad_control.stack_supervisor import start_stack, stop_stack

DOMAIN = os.environ.get('ROS_DOMAIN_ID', '43')
ZERO = (0.,)*6


class Probe(Node):
    def __init__(self):
        super().__init__('gamepad_sim_integration_probe')
        self.fixture = self.create_publisher(GamepadInput, '/piper_servo/gamepad/fixture', 10)
        self.ui = self.create_publisher(Float64, '/piper_servo/gamepad/ui_heartbeat', 1)
        self.speed = self.create_publisher(Float64, '/piper_servo/gamepad/speed_scale', 1)
        self.service_clients = {name: self.create_client(Trigger, f'/piper_servo/gamepad/{name}')
                                for name in ('start', 'end', 'stop', 'recover')}
        self.selection_client = self.create_client(SelectGamepadControl, '/piper_servo/gamepad/select_control')
        self.controller_list = self.create_client(ListControllers, '/piper_servo/controller_manager/list_controllers')
        self.parameters = self.create_client(GetParameters, '/piper_servo/gamepad_backend/get_parameters')
        self.backend = {}; self.guard = {}; self.jog = None
        self.nonzero_jogs = 0; self.trajectory_count = 0
        self.jogs = []; self.trajectories = []; self.joints = []; self.statuses = []
        self.command = {'axes': (0.,)*4, 'dpad': (0,)*4, 'lb': False, 'b': False}
        self.publish_fixture = True; self.last_ui = 0.; self.sequence = 0
        self.create_subscription(String, '/piper_servo/gamepad/backend_status', self.on_backend, 10)
        self.create_subscription(String, '/piper_servo/guard/status', self.on_guard, 10)
        self.create_subscription(JointJog, '/piper_servo/servo_node/delta_joint_cmds', self.on_jog, 100)
        # Observe pre-guard Servo output; the protected actuator topic has one sink.
        self.create_subscription(JointTrajectory, '/piper_servo/servo/trajectory', self.on_trajectory, 100)
        self.create_subscription(JointState, '/piper_servo/commanded_joint_states', self.on_joints, 100)
        self.create_timer(.02, self.publish_inputs)

    def on_backend(self, msg):
        self.backend = json.loads(msg.data)
        self.statuses.append((time.monotonic(), self.backend))

    def on_guard(self, msg):
        self.guard = json.loads(msg.data)

    def on_jog(self, msg):
        mapping = dict(zip(msg.joint_names, msg.velocities))
        self.jog = tuple(mapping[name] for name in JOINTS)
        self.jogs.append((time.monotonic(), self.jog))
        if np.linalg.norm(self.jog) > .0001:
            self.nonzero_jogs += 1

    def on_trajectory(self, msg):
        if msg.points:
            self.trajectory_count += 1
            self.trajectories.append((time.monotonic(), msg))

    def on_joints(self, msg):
        mapping = dict(zip(msg.name, msg.position))
        if all(name in mapping for name in JOINTS):
            self.joints.append((time.monotonic(), np.array([mapping[name] for name in JOINTS])))

    def publish_inputs(self):
        now = time.monotonic()
        if now-self.last_ui >= .04:
            self.ui.publish(Float64(data=time.time())); self.last_ui = now
        if self.publish_fixture:
            msg = GamepadInput(); msg.header.stamp = self.get_clock().now().to_msg()
            msg.guid = 'gamepad-integration-fixture'; msg.source = 'fixture'
            msg.device_name = 'isolated mock fixture'; msg.connection_type = 'fixture'
            self.sequence += 1
            msg.generation = 1; msg.sequence = self.sequence; msg.connected = True
            msg.link_verified = False; msg.mapping_verified = False
            msg.axes = list(self.command['axes']); msg.dpad = list(self.command['dpad'])
            msg.lb = self.command['lb']; msg.b = self.command['b']
            self.fixture.publish(msg)

    def set_input(self, axes=(0.,)*4, dpad=(0,)*4, lb=False, b=False):
        self.command = dict(axes=tuple(axes), dpad=tuple(dpad), lb=bool(lb), b=bool(b))

    def pump(self, duration=.01):
        # Drain high-rate Jog/trajectory/200 Hz feedback together; one callback
        # per sleep can starve the feedback reader and manufacture zero motion.
        for _ in range(12):
            rclpy.spin_once(self, timeout_sec=0.)
        time.sleep(duration)

    def dwell(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            self.pump(.002)

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump(.002)
            if predicate():
                return
        raise RuntimeError(f'timeout: {description}; backend={self.backend}; guard={self.guard}')

    def service(self, client, request, timeout=6.):
        self.wait_for(client.service_is_ready, timeout, 'service ready')
        future = client.call_async(request)
        self.wait_for(future.done, timeout, 'service response')
        return future.result()

    def call(self, name, timeout=6., expect_success=True):
        response = self.service(self.service_clients[name], Trigger.Request(), timeout)
        if bool(response.success) != expect_success:
            raise AssertionError(f'{name}: {response.success}, {response.message}')
        return response.message

    def select(self, mode, axis=None, expect_success=True):
        field = 'selected_cartesian_axis' if mode == 'cartesian_axis' else 'selected_joint'
        if axis is None:
            axis = self.backend[field]
        response = self.service(self.selection_client, SelectGamepadControl.Request(control_mode=mode, joint_index=axis))
        if bool(response.success) != expect_success:
            raise AssertionError(f'select {mode}/{axis}: {response.success}, {response.message}')
        if expect_success:
            self.wait_for(lambda: self.backend.get('control_mode') == mode and
                          self.backend.get(field) == axis and
                          self.backend.get('selection_revision') == response.revision, 2., 'selection ack snapshot')
        return response

    def neutral(self):
        self.set_input()
        self.wait_for(lambda: self.backend.get('state') in ('WAITING', 'ARMED') and
                      self.backend.get('selection_ready'), 5., 'stopped neutral selection barrier')
        self.dwell(.20)

    def activate(self, axes, dpad=(0,)*4):
        self.neutral()
        if self.backend['state'] == 'WAITING':
            self.call('start'); self.dwell(.25)
        self.set_input(axes=axes, dpad=dpad, lb=True)
        self.wait_for(lambda: self.backend.get('state') == 'ACTIVE' and
                      self.guard.get('state') == 'running', 10., 'new LB motion permit')
        # Throw away queued status and startup trajectories before measuring.
        self.dwell(.16)

    def release(self):
        released = time.monotonic()
        self.set_input()
        self.wait_for(lambda: self.backend.get('state') == 'ARMED' and
                      self.guard.get('stop_confirmed') is True, 5., 'LB release measured stop')
        self.wait_for(lambda: any(t >= released and all(v == 0. for v in qd) for t, qd in self.jogs),
                      1., 'zero Jog bypasses intent filter')
        first_zero = next(t for t, qd in self.jogs if t >= released and all(v == 0. for v in qd))
        require(first_zero-released < .12, 'LB release retained a filtered velocity tail')
        for field in ('requested_joint_velocity', 'filtered_joint_velocity',
                      'requested_tcp_velocity', 'filtered_tcp_velocity'):
            if self.backend.get(field) != list(ZERO):
                raise AssertionError(f'{field} not reset by release')
        return time.monotonic()-released


def require(condition, description):
    if not condition:
        raise AssertionError(description)


def audit_mock(probe):
    response = probe.service(probe.parameters, GetParameters.Request(names=['robot_description']))
    urdf = response.values[0].string_value
    plugins = [item.text for item in ET.fromstring(urdf).findall('.//ros2_control/hardware/plugin')]
    require(plugins == ['mock_components/GenericSystem'], f'not pure mock: {plugins}')
    names = probe.get_node_names_and_namespaces()
    require(not any('agx_arm_ctrl' in name or 'driver' in name for name, ns in names),
            f'real driver discovered: {names}')
    require(probe.backend.get('mode') == 'sim' and probe.backend.get('input_source') == 'fixture',
            'backend must confirm sim and fixture')
    return RobotModel(urdf), {'hardware_plugins': plugins, 'nodes': names, 'driver_absent': True}


def start_mock(probe, logs, config=None):
    logs.mkdir(parents=True)
    stream = (logs/'stack.log').open('w', encoding='utf-8')
    command = [sys.executable, '/opt/ros/jazzy/bin/ros2', 'launch', 'piper_servo_tracking', 'task.launch.py',
               'mode:=sim', 'gamepad:=true', 'rviz:=false', 'gamepad_input_source:=fixture',
               f'gamepad_config:={ROOT}/scripts/tools/gamepad_control/config/config.yaml']
    if config is not None:
        command.append(f'config:={config}')
    process, lifeline = start_stack(command, stream, logs)
    return process, lifeline, stream


def ready(probe):
    probe.wait_for(lambda: bool(probe.backend.get('feedback')) and probe.guard.get('state') == 'idle' and
                   probe.backend.get('servo_switched') is True and bool(probe.joints), 40., 'mock ready')
    def controller_ready():
        result = probe.service(probe.controller_list, ListControllers.Request())
        return any(item.name == 'arm_controller' and item.state == 'active' for item in result.controller)
    probe.wait_for(controller_ready, 30., 'arm_controller active')
    probe.neutral()


def measure_direction(probe, model, name, axes, dpad, axis, sign, single=False):
    probe.activate(axes, dpad)
    mode = probe.backend['control_mode']
    started = time.monotonic(); q0 = probe.joints[-1][1].copy()
    probe.dwell(.30)
    require(probe.backend.get('state') == 'ACTIVE', f'{name}: lost active session')
    samples = [qd for t, qd in probe.jogs if t >= started]
    require(len(samples) >= 10, f'{name}: too few fresh Jogs')
    q1 = probe.joints[-1][1].copy()
    jog = np.median(samples, axis=0)
    trajectories = [msg for t, msg in probe.trajectories if t >= started]
    require(trajectories, f'{name}: no fresh Servo trajectory')
    if single:
        require(sign*jog[axis] > .001, f'{name}: wrong selected Jog sign {jog}')
        ignored = [i for i in range(6) if i != axis]
        require(all(qd[i] == 0. for qd in samples for i in ignored), f'{name}: another Jog axis nonzero')
        require(sign*(q1-q0)[axis] > .0002, f'{name}: mock joint did not move in requested direction {q1-q0}')
        span = np.max(np.abs([q-q0 for t, q in probe.joints if t >= started]), axis=0)
        require(np.max(span[ignored]) < 2e-4, f'{name}: nonselected mock feedback drift {span}')
        for msg in trajectories:
            indices = [msg.joint_names.index(JOINTS[i]) for i in ignored]
            for point in msg.points:
                require(all(abs(point.positions[index]-q0[i]) < 2e-4 for i, index in zip(ignored, indices)),
                        f'{name}: nonselected Servo positions retained a previous command')
                if point.velocities:
                    require(all(abs(point.velocities[index]) < 1e-7 for index in indices),
                            f'{name}: nonselected Servo velocity retained a previous command')
        displacement = (q1-q0).tolist()
    else:
        twist = model.jacobian(q0) @ jog
        require(sign*twist[axis] > 1e-5, f'{name}: wrong Jacobian velocity sign {twist}')
        for field in ('requested_tcp_velocity', 'filtered_tcp_velocity'):
            intent = probe.backend[field]
            require(sign*intent[axis] > 0. and all(v == 0. for i, v in enumerate(intent) if i != axis),
                    f'{name}: wrong {field} {intent}')
        p0, p1 = model.pose(q0), model.pose(q1)
        delta = np.r_[p1[:3]-p0[:3],
                      (Rotation.from_quat(p1[3:])*Rotation.from_quat(p0[3:]).inv()).as_rotvec()]
        require(sign*delta[axis] > (2e-5 if axis < 3 else 1e-4), f'{name}: wrong actual FK delta {delta}')
        weights = np.array([1., 1., 1., .25, .25, .25])
        require(sign*(weights*twist)[axis]/np.linalg.norm(weights*twist) > .95,
                f'{name}: solved velocity is not aligned with requested TCP axis {twist}')
        require(sign*(weights*delta)[axis]/np.linalg.norm(weights*delta) > .95,
                f'{name}: actual FK delta is not aligned with requested TCP axis {delta}')
        displacement = delta.tolist()
    stop_time = probe.release()
    print(f'PASS {name}', flush=True)
    return {'mode': mode, 'direction': name, 'jog_samples': len(samples), 'median_jog_rad_s': jog.tolist(),
            'measured_delta': displacement, 'servo_samples': len(trajectories), 'release_stop_s': stop_time}


def navigation_and_switching(probe):
    probe.neutral(); probe.select('single_axis', 1); probe.neutral()
    before = probe.backend['selection_revision']
    probe.set_input(dpad=(0, 0, 0, 1)); probe.dwell(.40)
    require(probe.backend['selected_joint'] == 2 and probe.backend['selection_revision'] == before+1,
            'held DPad must select exactly one joint')
    probe.dwell(.25)
    require(probe.backend['selected_joint'] == 2, 'held DPad repeated')
    probe.neutral(); probe.set_input(dpad=(0, 0, 0, 1)); probe.dwell(.25)
    require(probe.backend['selected_joint'] == 3, 'second DPad press did not select')
    for joint, dpad in ((6, (0, 0, 0, 1)), (1, (0, 0, 1, 0))):
        probe.neutral(); probe.select('single_axis', joint); probe.neutral()
        revision = probe.backend['selection_revision']
        probe.set_input(dpad=dpad); probe.dwell(.30)
        require(probe.backend['selected_joint'] == joint and probe.backend['selection_revision'] == revision,
                'navigation wrapped at endpoint')
    probe.neutral(); probe.select('single_axis', 1)
    probe.speed.publish(Float64(data=.25)); probe.dwell(.20)
    probe.neutral(); probe.select('cartesian', 1); probe.neutral()
    probe.speed.publish(Float64(data=.35)); probe.dwell(.20)
    probe.neutral(); probe.select('single_axis', 1)
    require(probe.backend['speed_scale'] == .25, 'joint speed was not remembered')
    probe.activate((0., -.65, 0., 0.))
    revision = probe.backend['selection_revision']
    probe.select('cartesian', 1, expect_success=False)
    probe.set_input(axes=(0., -.65, 0., 0.), dpad=(0, 0, 0, 1), lb=True); probe.dwell(.20)
    require(probe.backend['selection_revision'] == revision and probe.backend['selected_joint'] == 1,
            'active selection or navigation changed the joint')
    probe.release(); probe.neutral(); probe.select('cartesian', 1)
    require(probe.backend['speed_scale'] == .35, 'Cartesian speed was not remembered')
    quiet = time.monotonic(); q0 = probe.joints[-1][1].copy(); probe.dwell(.35)
    require(all(all(v == 0. for v in qd) for t, qd in probe.jogs if t >= quiet), 'switch replayed a Jog')
    require(np.max(np.abs(probe.joints[-1][1]-q0)) < 2e-4, 'switch moved mock without a new LB')
    probe.neutral(); probe.select('cartesian_axis')
    axis_field = 'selected_cartesian_axis'
    require(probe.backend[axis_field] == 1 and probe.backend['speed_scale'] == .20,
            'Cartesian single-axis must initially select X at its own default speed')
    for axis, dpad in ((2, (0, 0, 0, 1)), (5, (0, 1, 0, 0))):
        probe.neutral()
        revision = probe.backend['selection_revision']
        probe.set_input(dpad=dpad); probe.dwell(.40)
        require(probe.backend[axis_field] == axis and probe.backend['selection_revision'] == revision+1,
                f'Cartesian single-axis DPad did not select and hold {CARTESIAN_AXES[axis-1]}')
    probe.neutral(); probe.speed.publish(Float64(data=.15)); probe.dwell(.20)
    probe.activate((0., -.65, 0., 0.))
    revision = probe.backend['selection_revision']
    probe.select('single_axis', 1, expect_success=False)
    probe.set_input(axes=(.55, -.65, -.75, .55), dpad=(0, 0, 0, 1), lb=True); probe.dwell(.20)
    twist = probe.backend['requested_tcp_velocity']
    require(probe.backend[axis_field] == 5 and probe.backend['selection_revision'] == revision
            and twist[4] > 0 and sum(v != 0. for v in twist) == 1, 'held LB or other controls changed Pitch jog')
    centered = time.monotonic()
    probe.set_input(axes=(.55, 0., -.75, .55), dpad=(0, 0, 0, 1), lb=True)
    probe.wait_for(lambda: probe.jog == ZERO, .12, 'centered vertical stick emits zero Jog while holding LB')
    probe.dwell(.20)
    require(probe.backend['state'] == 'ACTIVE' and probe.backend['filtered_tcp_velocity'] == list(ZERO)
            and all(qd == ZERO for t, qd in probe.jogs if t >= centered+.12),
            'stick center retained velocity or canceled LB permit')
    probe.release()
    for mode, scale in (('single_axis', .25), ('cartesian', .35), ('cartesian_axis', .15)):
        probe.neutral(); probe.select(mode)
        require(probe.backend['speed_scale'] == scale, f'{mode} did not remember its speed')
        require(probe.backend[axis_field] == 5 and probe.backend['selected_joint'] == 1,
                'switching modes lost one of the selections')
        probe.speed.publish(Float64(data=.20)); probe.dwell(.20)
    return {'navigation_edges': True, 'no_wrap': True, 'active_selection_rejected': True,
            'independent_speeds': True, 'switch_quiet_feedback': True,
            'cartesian_axis_memory': True, 'cartesian_axis_centered_zero': True}


def lock_and_dropout(probe, mode):
    probe.neutral(); probe.select(mode, 1)
    probe.activate((0., -.65, 0., 0.))
    probe.set_input(b=True)
    probe.wait_for(lambda: probe.backend.get('state') == 'LOCKED' and probe.guard.get('stop_confirmed'), 5., 'B locks')
    revision = probe.backend['selection_revision']
    probe.select('cartesian' if mode == 'single_axis' else 'single_axis', 1, expect_success=False)
    require(probe.backend['selection_revision'] == revision, 'mode selection unlocked B')
    probe.set_input(); probe.dwell(.25); probe.call('recover'); probe.dwell(.25)
    require(probe.backend['state'] == 'ARMED', 'recover must require a new LB')
    probe.activate((0., -.65, 0., 0.))
    probe.publish_fixture = False
    probe.wait_for(lambda: probe.backend.get('state') == 'LOCKED' and probe.guard.get('stop_confirmed'),
                   5., 'input loss locks and confirms stop')
    probe.publish_fixture = True; probe.set_input(); probe.dwell(.30)
    require(probe.backend['state'] == 'LOCKED', 'reconnection unlocked automatically')
    probe.call('recover'); probe.dwell(.25)
    return {'mode': mode, 'B_locked': True, 'selection_cannot_unlock': True,
            'dropout_stopped': True, 'reconnection_stays_locked': True, 'recover_requires_new_LB': True}


def boundary_case(probe):
    probe.neutral(); probe.select('single_axis', 1)
    initial = probe.joints[-1][1].copy()
    probe.activate((0., -1., 0., 0.)); probe.dwell(.35)
    require(probe.backend['state'] == 'ACTIVE', 'outward bound request locked session')
    require(probe.backend['joint_bound_limited'], 'outward request was not limited')
    require(np.max(np.abs(probe.joints[-1][1]-initial)) < .001, 'outward request crossed configured soft margin')
    outward_scale = probe.backend['bound_velocity_scale']
    # Reverse while keeping LB held: no recovery or new session is allowed here.
    start = time.monotonic(); probe.set_input(axes=(0., 1., 0., 0.), lb=True); probe.dwell(.55)
    require(probe.backend['state'] == 'ACTIVE', 'reverse lost active session')
    require(probe.joints[-1][1][0] < initial[0]-.003, 'inward request did not retreat')
    require(any(t >= start and qd[0] < -.001 for t, qd in probe.jogs), 'no inward Jog after boundary')
    probe.set_input(b=True)
    probe.wait_for(lambda: probe.backend.get('state') == 'LOCKED' and probe.guard.get('stop_confirmed'), 5., 'B after bound')
    return {'outward_limited_session_active': True, 'outward_scale': outward_scale,
            'reverse_without_recovery': True, 'B_still_locks': True,
            'initial_joints_rad': initial.tolist(), 'final_joints_rad': probe.joints[-1][1].tolist()}


def run_stack(logs, summary, callback, config=None):
    probe = Probe(); process = lifeline = stream = None
    try:
        probe.dwell(1.)
        require(not any(ns == '/piper_servo' for name, ns in probe.get_node_names_and_namespaces()),
                'domain already contains /piper_servo; refusing to launch')
        process, lifeline, stream = start_mock(probe, logs, config)
        ready(probe); model, audit = audit_mock(probe); summary['mock_audit'] = audit
        callback(probe, model)
    finally:
        probe.destroy_node()
        if process is not None:
            summary['cleanup'] = stop_stack(process, lifeline, logs)
        if stream is not None:
            stream.close()


def main():
    require(DOMAIN.isdigit() and 0 <= int(DOMAIN) <= 232 and DOMAIN != '43', 'use an unused non-default ROS domain')
    outputs = ROOT/'outputs/gamepad_control'
    outputs.mkdir(parents=True, exist_ok=True)
    lockfile = (outputs/f'mock_domain_{DOMAIN}.lock').open('a')
    fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
    logs = outputs/f'three_modes_{time.strftime("%Y%m%d_%H%M%S")}_d{DOMAIN}_{os.getpid()}'
    logs.mkdir(parents=True)
    summary = {'domain': DOMAIN, 'fixture_only': True, 'real_hardware': False, 'passed': False,
               'logs': str(logs), 'directions': []}
    sources = ['tests/gamepad_sim_integration.py', 'scripts/tools/gamepad_control/core.py',
               'scripts/tools/gamepad_control/motion.py', 'scripts/tools/gamepad_control/config/config.yaml',
               'scripts/task/servo_tracking/gamepad_backend.py', 'scripts/task/servo_tracking/guard.py',
               'scripts/task/servo_tracking/task.launch.py', 'scripts/task/servo_tracking/core.py']
    summary['source_sha256'] = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
    rclpy.init()
    try:
        def exercise(probe, model):
            summary['switching'] = navigation_and_switching(probe)
            for joint in range(1, 7):
                for sign in (1, -1):
                    probe.neutral(); probe.select('single_axis', joint)
                    summary['directions'].append(measure_direction(probe, model, f'J{joint}{sign:+}',
                        (0., -.65*sign, 0., 0.), (0,)*4, joint-1, sign, single=True))
            probe.neutral(); probe.select('cartesian', 1)
            cases = [('X', (0., -.65, 0., 0.), (0,)*4), ('Y', (-.65, 0., 0., 0.), (0,)*4),
                     ('Z', (0., 0., 0., -.65), (0,)*4), ('Rx', (0.,)*4, (0, 0, 1, 0)),
                     ('Ry', (0.,)*4, (1, 0, 0, 0)), ('Rz', (0., 0., -.65, 0.), (0,)*4)]
            for axis, (name, axes, dpad) in enumerate(cases):
                for sign in (1, -1):
                    opposite = tuple(reversed(dpad)) if axis in (3, 4) and sign < 0 else dpad
                    if axis == 3 and sign < 0: opposite = (0, 0, 0, 1)
                    if axis == 4 and sign < 0: opposite = (0, 1, 0, 0)
                    summary['directions'].append(measure_direction(probe, model, f'{name}{sign:+}',
                        tuple(sign*v for v in axes), opposite, axis, sign))
            for axis, name in enumerate(CARTESIAN_AXES, 1):
                for sign in (1, -1):
                    probe.neutral(); probe.select('cartesian_axis', axis)
                    summary['directions'].append(measure_direction(probe, model, f'{name} single-axis {sign:+}',
                        (0., -.65*sign, 0., 0.), (0,)*4, axis-1, sign))
            probe.neutral(); probe.select('single_axis', 2)
            summary['cartesian_to_single_axis'] = measure_direction(probe, model,
                'Cartesian-to-J2 switch', (0., -.65, 0., 0.), (0,)*4, 1, 1, single=True)
            summary['stops'] = [lock_and_dropout(probe, mode) for mode in CONTROL_MODES]
            summary['boundary_upper'] = probe.backend['joint_limits_rad']['upper'][0]
            probe.call('stop')
            probe.wait_for(lambda: probe.guard.get('stop_confirmed'), 5., 'final stop')
        run_stack(logs/'directions', summary, exercise)
        # Only this generated copy changes the mock start; user task points stay untouched.
        config = yaml.safe_load((ROOT/'scripts/task/servo_tracking/config/config.yaml').read_text())
        config['initial_joints'][0] = summary['boundary_upper']-.014
        config['control_config'] = str(ROOT/'scripts/task/servo_tracking/config/control.yaml')
        path = logs/'boundary_task.yaml'; path.write_text(yaml.safe_dump(config))
        boundary = {}; summary['boundary_run'] = boundary
        run_stack(logs/'boundary', boundary,
                  lambda probe, model: boundary.update(result=boundary_case(probe)), path)
        summary['passed'] = True
    except BaseException as exc:
        summary['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        if rclpy.ok(): rclpy.shutdown()
        (logs/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps({'passed': summary['passed'], 'logs': str(logs),
                          'directions': len(summary['directions'])}, ensure_ascii=False), flush=True)
        lockfile.close()


if __name__ == '__main__':
    main()
