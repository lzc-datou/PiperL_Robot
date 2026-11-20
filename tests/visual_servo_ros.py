"""Opt-in launch/measure test in dedicated Domain 91. Owns a mock-only ROS stack."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import ServoStatus
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger
import yaml

from servo_tracking.core import JOINTS, RobotModel, load_config
from servo_tracking.node import pose_message
from scripts.task.visual_servo.geometry import DEFAULT, matrix, settings
from tests.visual_servo_kinematics import ROOT, URDF


class Probe(Node):
    def __init__(self, child):
        super().__init__('visual_servo_sim_test')
        self.child = child
        self.status, self.guard = {}, {}
        self.q = None
        self.owns = False
        ns = '/piper_servo/'
        self.create_subscription(String, ns+'status', self.status_received, 1)
        self.create_subscription(String, ns+'guard/status', self.guard_received, 1)
        self.create_subscription(JointState, ns+'commanded_joint_states', self.joints, 1)
        self.beat = self.create_publisher(Float64, ns+'panel/heartbeat', 1)
        self.board = self.create_publisher(Pose, ns+'sim/board_offset', 1)
        self.visible = self.create_publisher(Bool, ns+'sim/visible', 1)
        self.start = self.create_client(Trigger, ns+'start')
        self.stop = self.create_client(Trigger, ns+'stop')
        self.recording = None
        self.motion_started = None
        self.goal = None
        self.show = True
        self.create_timer(.04, self.fixture)

    def status_received(self, msg):
        self.status = json.loads(msg.data)

    def guard_received(self, msg):
        self.guard = json.loads(msg.data)

    def joints(self, msg):
        self.q = np.array([msg.position[msg.name.index(n)] for n in JOINTS])

    def fixture(self):
        if self.owns:
            if not (self.status.get('mode') == self.guard.get('mode') == 'sim'):
                raise RuntimeError('当前模拟执行栈身份发生变化')
            self.beat.publish(Float64(data=time.time()))
            self.visible.publish(Bool(data=self.show))
            if self.recording is not None:
                times, poses, since = self.recording
                idx = np.searchsorted(times, time.monotonic()-since, side='right') - 1
                self.goal = poses[max(0, idx)]
            elif self.motion_started is not None:
                offset = .035 + .012*np.sin(4.*(time.monotonic()-self.motion_started))
                self.set_pose([offset, 0., 0.], 3)
            if self.goal is not None:
                self.board.publish(self.goal)

    def wait(self, condition, seconds, label):
        end = time.monotonic()+seconds
        while time.monotonic() < end and self.child.poll() is None:
            rclpy.spin_once(self, timeout_sec=.03)
            if condition():
                return
        raise RuntimeError(f'{label} 超时或执行栈退出，状态 {self.status} / Guard {self.guard}')

    def set_pose(self, xyz, tilt, roll=0.):
        orientation = Rotation.from_euler('x', tilt, degrees=True)*Rotation.from_euler('z', roll, degrees=True)
        self.goal = pose_message(np.r_[xyz, orientation.as_quat()])

    def check_native_bound(self, model):
        # Inject the same axis-free status as the real J4 incident. This probe
        # owns a mock-only stack in Domain 91, with no hardware inputs or gate.
        assert self.owns and self.status['mode'] == self.guard['mode'] == 'sim'
        origin = model.pose(self.q)[:3]
        self.set_pose([.035, 0., 0.], 3, 30)
        self.wait(lambda: np.linalg.norm(model.pose(self.q)[:3]-origin) > .004 and
                  self.status.get('position_error_mm', 0.) > 12, 5, '边界注入前正在追踪')
        pub = self.create_publisher(ServoStatus, '/piper_servo/servo_node/status', 1)
        try:
            self.wait(lambda: pub.get_subscription_count() > 0, 2, '测试状态匹配 Guard')
            timer = self.create_timer(.02, lambda: pub.publish(ServoStatus(code=6, message='joint bound')))
            try:
                self.wait(lambda: self.guard.get('pause_cause') == 'joint_bound', 2, '原生关节边界暂停')
            finally:
                self.destroy_timer(timer)
        finally:
            self.destroy_publisher(pub)
        self.wait(lambda: self.guard.get('stop_confirmed') and self.status.get('boundary', '').startswith('Servo'),
                  4, '边界停稳且会话保留')
        generation, held = self.guard['stop_generation'], self.q.copy()
        self.set_pose([.045, 0., 0.], 3, 30)
        until = time.monotonic()+.6
        self.wait(lambda: time.monotonic() >= until, 2, '继续向外时保持')
        assert self.status['active'] and self.guard['gate_closed'] and np.max(abs(self.q-held)) < .003
        self.set_pose([-.018, 0., 0.], 3, 30)
        self.wait(lambda: self.status['active'] and not self.status.get('boundary') and
                  self.guard['state'] == 'running' and np.max(abs(self.q-held)) > .005,
                  8, '反向移板恢复实际运动')
        assert self.guard['stop_generation'] == generation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim-input', choices=('pose', 'image'), default='pose')
    parser.add_argument('--replay-start', type=Path, help='实机 tracking.jsonl，用模拟关节重放起步时的板中心与法线')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '91':
        parser.error('此自动验收专用 PIPERL_ROS_DOMAIN_ID=91')
    cfg = settings()
    model = RobotModel(URDF.read_text(), tip='link6')
    home = matrix(model.pose(load_config(cfg['servo_config'])['initial_joints']))
    # A test workspace face only 4 cm away lets the board stay in frame even
    # during boundary entry. All other limits remain the production limits.
    data = yaml.safe_load(DEFAULT.read_text())
    data.update(servo_config=cfg['servo_config'], board_config=cfg['board_config'])
    data['workspace_max_m'][0] = float(home[0, 3]+.04)
    directory = ROOT/'outputs/tasks/visual_servo'/f'ros_test_{time.time_ns()}'
    directory.mkdir(parents=True)
    recording = None
    if args.replay_start:
        with args.replay_start.open() as log:
            rows = [json.loads(line) for line in log]
        start = next(row for row in rows if 'planning_start_rad' in row)
        servo = yaml.safe_load(Path(cfg['servo_config']).read_text())
        servo['control_config'] = load_config(cfg['servo_config'])['control_config']
        servo['initial_joints'] = start['planning_start_rad']
        home = matrix(model.pose(servo['initial_joints']))
        servo_path = directory/'servo.yaml'
        servo_path.write_text(yaml.safe_dump(servo))
        data.update(servo_config=str(servo_path), workspace_max_m=cfg['workspace_max_m'])
        times, poses = [], []
        for row in rows:
            if 'raw_center_m' not in row or not row.get('raw_normal'):
                continue
            t = row['source_stamp'] - start['source_stamp']
            if t < 0 or t > 12 or (times and t <= times[-1]):
                continue
            center = home[:3, :3].T @ (row['raw_center_m'] - home[:3, 3])
            normal = home[:3, :3].T @ row['raw_normal']
            rotation, _ = Rotation.align_vectors([normal], [[0., 0., 1.]])
            poses.append(pose_message(np.r_[center - [0., 0., cfg['distance_m']], rotation.as_quat()]))
            times.append(t)
        assert len(times) >= 2
        recording = (times, poses)
    path = directory/'config.yaml'
    path.write_text(yaml.safe_dump(data))
    rclpy.init(args=[])
    graph = rclpy.create_node('visual_test_discovery')
    deadline = time.monotonic()+1.5
    while time.monotonic() < deadline:
        rclpy.spin_once(graph, timeout_sec=.1)
        if any(ns == '/piper_servo' for _, ns in graph.get_node_names_and_namespaces()):
            raise RuntimeError('专用 domain 已有运行栈')
    graph.destroy_node()
    with (directory/'launcher.log').open('w') as log:
        child = subprocess.Popen([sys.executable, 'scripts/task/visual_servo/run.py', '--mode', 'sim',
            '--config', str(path), '--no-rviz', '--sim-input', args.sim_input], cwd=ROOT,
            stdout=log, stderr=subprocess.STDOUT)
        p = Probe(child)
        results, measurements = [], {}
        try:
            p.wait(lambda: p.status.get('mode') == p.guard.get('mode') == 'sim' and
                   'visual_branch' in p.status and p.q is not None, 45, '模拟反馈与后端连接')
            p.show = recording is not None
            p.owns = True
            p.set_pose([0., 0., 0.], 3)
            if recording:
                p.recording = (*recording, time.monotonic())
            p.wait(lambda: p.status.get('source_ok') and p.status.get('servo_ready')
                   and p.count_subscribers('/piper_servo/arm_controller/joint_trajectory') == 1
                   and p.start.service_is_ready()
                   and (p.status.get('last_missing_exposure') if recording is None else p.status.get('angle_deg') is not None), 12, '相机与控制器就绪')
            p.motion_started = time.monotonic()
            q_start = p.q.copy()
            until = time.monotonic()+.5
            p.wait(lambda: time.monotonic() >= until, 1., '面板心跳')
            future = p.start.call_async(Trigger.Request())
            p.wait(future.done, 3, '开启跟踪')
            assert future.result().success, future.result().message
            if not recording:
                until = time.monotonic()+1.8
                p.wait(lambda: time.monotonic() >= until and p.status.get('waiting_for_target'), 3, '空画面等待')
                assert p.status['active'] and p.guard['gate_closed'] and np.max(abs(p.q-q_start)) < .003
                results.append('start_without_board')
                p.show = True
            p.wait(lambda: p.guard.get('state') == 'running', 6, 'Guard 开门')
            generation = p.guard['stop_generation']
            if recording:
                assert np.max(abs(q_start - servo['initial_joints'])) < .003
                p.wait(lambda: np.linalg.norm(model.pose(p.q)[:3]-home[:3, 3]) > .02, 8, '实测起点出发')
                p.wait(lambda: time.monotonic() >= p.recording[2] + 12. and
                       p.status.get('position_error_mm', 999.) < 5 and p.status['active'] and
                       p.guard['state'] == 'running', 45, '反馈 TCP 收敛到重放目标')
                assert p.guard['stop_generation'] == generation
                results.append('recorded_start_converged')
                print('通过', args.replay_start, '位置误差 mm', p.status['position_error_mm'])
                return
            p.wait(lambda: np.linalg.norm(model.pose(p.q)[:3]-model.pose(q_start)[:3]) > .012,
                   6, '棋盘持续移动时从远处起步')
            assert p.guard['stop_generation'] == generation
            results.append('moving_target_start')
            p.motion_started = None
            p.set_pose([0., 0., 0.], 3)
            p.wait(lambda: np.linalg.norm(model.pose(p.q)[:3]-home[:3, 3]) < .002, 8, '跟随棋盘回到起点')
            q0 = p.q.copy()
            p.set_pose([0., 0., 0.], 3, 30)
            deadline = time.monotonic()+1.5
            p.wait(lambda: time.monotonic() >= deadline, 2, '纯滚转观察')
            assert p.status['active'] and np.max(abs(p.q-q0)) < .012
            results.append('pure_roll')
            p.check_native_bound(model)
            results.append('native_joint_bound_stop_and_reverse')
            p.set_pose([0., .018, 0.], 8)
            p.wait(lambda: p.status.get('visual_branch') == '位置跟踪 · 姿态纠偏', 3, '带外纠偏')
            p.wait(lambda: (p.status.get('angle_deg') or 180) <= 5 and p.status['active'] and
                   np.linalg.norm(model.pose(p.q)[:3]-model.pose(q0)[:3]) > .007, 10, '反馈 FK 纠偏与平移')
            results.append('tilt_and_translation')
            p.set_pose(home[:3, :3].T @ [.065, 0., 0.], 3)
            p.wait(lambda: bool(p.status.get('boundary')) and p.guard.get('stop_confirmed') and
                   p.guard.get('gate_closed'), 5, '边界保持与停稳')
            held = p.q.copy()
            generation = p.guard['stop_generation']
            deadline = time.monotonic()+.5
            p.wait(lambda: time.monotonic() >= deadline, 1, '边界保持观测')
            assert np.max(abs(p.q-held)) <= .003
            p.set_pose([0., -.02, 0.], 8)
            p.wait(lambda: p.status['active'] and not p.status['boundary'] and
                   p.guard['state'] == 'running' and np.max(abs(p.q-held)) > .008, 10, '退回同会话恢复')
            assert p.guard['stop_generation'] == generation
            results.append('boundary_stop_and_retreat')
            # Move A while visible, hide during transit, compare FK to the
            # unchanged last vision revision rather than to an assumed ideal board pose.
            p.set_pose([.045, -.012, 0.], 12)
            p.wait(lambda: p.status.get('position_error_mm', 0) > 30 and
                   (p.status.get('orientation_error_deg') or 0) > 2, 5, '遮挡前尚有位置及姿态误差')
            p.show = False
            p.wait(lambda: p.status.get('frozen_target'), 3, '已锁存最后目标')
            revision = p.status['frozen_revision']
            start_error = p.status['position_error_mm']
            measurements.update(occluded_start_mm=start_error, occluded_start_deg=p.status['orientation_error_deg'])
            q_lost = p.q.copy()
            p.wait(lambda: p.status.get('waiting_for_target') and p.status['active']
                   and p.guard.get('stop_confirmed') and p.guard.get('gate_closed'), 12, '遮挡中完成最后目标')
            assert revision == p.status['target_revision'] and start_error > 8
            assert p.status['position_error_mm'] < 3 and p.status['orientation_error_deg'] < 1.15
            assert np.max(abs(p.q-q_lost)) > .005 and '已到达' in p.status['state']
            measurements.update(arrived_mm=p.status['position_error_mm'], arrived_deg=p.status['orientation_error_deg'])
            held = p.q.copy()
            until = time.monotonic()+.7
            p.wait(lambda: time.monotonic() >= until, 2, '到位停稳')
            assert np.max(abs(p.q-held)) <= .003 and p.guard['gate_closed']
            results.append('occluded_target_reached_and_held')
            p.set_pose([.012, -.012, 0.], 12)
            p.show = True
            p.wait(lambda: p.status['active'] and p.guard['state'] == 'running'
                   and home[:3, 0] @ (model.pose(held)[:3]-model.pose(p.q)[:3]) > .007
                   and not p.status.get('waiting_for_target'),
                   8, '重获棋盘自动续跟')
            results.append('reacquire_after_arrival')
            p.show = False
            p.wait(lambda: p.status.get('frozen_target') and p.status['position_error_mm'] > 10,
                   4, '第二次遮挡且仍在前往最后目标')
            frozen = p.status['frozen_revision']
            generation = p.guard['stop_generation']
            at_turn = model.pose(p.q)[:3].copy()
            p.set_pose([.050, -.012, 0.], 12)
            p.show = True
            p.wait(lambda: p.status['active'] and p.guard['state'] == 'running'
                   and not p.status['frozen_target'] and p.status['target_revision'] > frozen
                   and home[:3, 0] @ (model.pose(p.q)[:3]-at_turn) > .007,
                   8, '到达前重获棋盘并反向运动')
            assert p.guard['stop_generation'] == generation
            results.append('reacquire_during_frozen_motion')
            p.show = False
            p.wait(lambda: p.status.get('frozen_target'), 4, '再次遮挡')
            future = p.stop.call_async(Trigger.Request())
            p.wait(lambda: future.done() and not p.status['active'] and p.guard.get('state') == 'idle',
                   4, '手动停止撤销最后目标')
            assert future.result().success and p.guard['gate_closed']
            future = p.start.call_async(Trigger.Request())
            p.wait(lambda: future.done() and p.status['active'] and p.status['waiting_for_target'],
                   4, '重新开启时空画面等待')
            assert future.result().success and p.guard['gate_closed'] and not p.status['frozen_target']
            future = p.stop.call_async(Trigger.Request())
            p.wait(lambda: future.done() and not p.status['active'], 3, '空画面等待时可停止')
            p.show = True
            until = time.monotonic()+.6
            p.wait(lambda: time.monotonic() >= until and p.status.get('angle_deg') is not None,
                   4, '停止后再次看到棋盘只预览')
            assert p.guard['gate_closed'] and not p.status['active']
            results.append('stop_and_restart_without_stale_target')
            print('通过', ', '.join(results))
        finally:
            (directory/'results.json').write_text(json.dumps(dict(passed=results, measurements=measurements, status=p.status,
                guard=p.guard), ensure_ascii=False, indent=2))
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                child.terminate()
                child.wait(timeout=10)
            p.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    main()
