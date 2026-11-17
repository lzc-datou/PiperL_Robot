"""Dedicated DLS -> JointJog backend for gamepad velocity control."""
import json
import math
import os
import sys
import time
import yaml
ROOT = os.environ.get('PIPERL_ROOT')
if ROOT and ROOT not in sys.path: sys.path.insert(0, ROOT)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger, SetBool
from moveit_msgs.srv import ServoCommandType
from control_msgs.msg import JointJog
from piper_servo_tracking.msg import GamepadInput
from .core import JOINTS, FreshJoints, RobotModel, load_config, damped_joint_velocity
from scripts.tools.gamepad_control.core import InputFrame, InputGate, map_velocity, ControlState, validate_tool_config


class GamepadBackend(Node):
    def __init__(self):
        super().__init__('gamepad_backend')
        for name, default in (('config_file',''),('robot_description',''),('mode','sim'),('gamepad_config','')):
            self.declare_parameter(name, default)
        self.mode = self.get_parameter('mode').value
        self.config = load_config(self.get_parameter('config_file').value)
        gp_path = self.get_parameter('gamepad_config').value
        with open(gp_path, encoding='utf-8') as stream: self.gp = validate_tool_config(yaml.safe_load(stream))
        self.model = RobotModel(self.get_parameter('robot_description').value)
        self.feedback = FreshJoints()
        self.input_gate = InputGate(self.gp['input_timeout_s'])
        self.control = ControlState(); self.control.neutral_dwell_s = self.gp['neutral_dwell_s']
        self.input = None; self.input_seen = 0.; self.input_error = '等待手柄输入'
        self.device_name=''; self.connection_type='unknown'; self.input_source=''
        self.ui_seen = 0.; self.speed_scale = self.gp['default_speed_scale']
        self.session = 0; self.epoch = 0; self.permit_future = None
        self.phase, self.reason = 'WAITING', '等待手柄与反馈'
        self.guard_state, self.guard_status = 'unknown', {}
        self.guard_seen=0.
        self.after_stop = 'ARMED'; self.stop_started = 0.; self.disable_future = None
        self.stop_generation_required = 0
        self.velocity_config = dict(self.config)
        self.velocity_config['joint_speed'] = self.gp['joint_speed']
        self.velocity_config['joint_acceleration'] = self.gp['joint_acceleration']
        self.velocity_config['default_speed_scale'] = self.speed_scale
        self.jog_pub = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 1)
        self.heartbeat = self.create_publisher(Float64, 'backend/heartbeat', 1)
        self.status_pub = self.create_publisher(String, 'gamepad/backend_status', 10)
        self.guard_client = self.create_client(SetBool, 'guard/enable')
        self.servo_switch = self.create_client(ServoCommandType, 'servo_node/switch_command_type')
        self.servo_switch_future = None
        self.servo_switched = False
        self.create_subscription(GamepadInput, 'gamepad/input', self.on_input, 10)
        self.create_subscription(JointState, 'feedback/joint_states' if self.mode == 'real' else 'commanded_joint_states', self.on_feedback, 1)
        self.create_subscription(String, 'guard/status', self.on_guard, 1)
        self.create_subscription(Float64, 'gamepad/ui_heartbeat', self.on_ui_heartbeat, 1)
        self.create_subscription(Float64, 'gamepad/speed_scale', self.on_speed, 1)
        self.create_service(Trigger, 'gamepad/start', self.start_session)
        self.create_service(Trigger, 'gamepad/end', self.end_session)
        self.create_service(Trigger, 'gamepad/stop', self.stop_session)
        self.create_service(Trigger, 'gamepad/recover', self.recover_session)
        self.create_timer(1./self.gp['control_rate_hz'], self.tick)
        self.create_timer(.1, self.publish_status)

    def on_feedback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        self.feedback.update(msg.name, msg.position, stamp)

    def on_ui_heartbeat(self, msg):
        if 0 <= time.time()-msg.data < self.gp['gui_timeout_s']:
            self.ui_seen = time.monotonic()

    def on_speed(self, msg):
        if self.phase in ('ACTIVE','PREPARING','STOPPING'):
            return
        if math.isfinite(msg.data) and .05 <= msg.data <= 1.:
            self.speed_scale = float(msg.data)

    def on_guard(self, msg):
        try: self.guard_status = json.loads(msg.data)
        except (ValueError, TypeError): return
        self.guard_seen=time.monotonic()
        self.guard_state = self.guard_status.get('state','unknown')
        if self.phase == 'PREPARING' and self.guard_state == 'running':
            if self._current_permit():
                self.phase, self.reason = 'ACTIVE', '控制中 · 松开 LB 停止'
            else:
                self._begin_stop('LOCKED','准备完成前许可已撤销')
        stop_generation = int(self.guard_status.get('stop_generation', 0))
        stop_requested_at = float(self.guard_status.get('stop_requested_at', 0.))
        stop_is_current = (stop_generation >= self.stop_generation_required and
                           stop_requested_at >= self.stop_started-.10)
        if self.phase == 'STOPPING' and self.guard_status.get('stop_confirmed') and stop_is_current:
            self.phase, self.reason = self.after_stop, self.guard_status.get('reason','已停止')
            if self.phase == 'ARMED':
                self.control.start()
            elif self.phase == 'WAITING':
                self.control = ControlState()
        if self.phase == 'STOP_UNKNOWN' and self.guard_status.get('stop_confirmed') and stop_is_current:
            self.phase, self.reason = 'LOCKED', '停止已确认；请检查后手动恢复'
        # A transient idle status can still be queued when the prepare service
        # has been accepted but its first pausing status has not arrived.  Do
        # not revoke that in-flight request; a rejected service is handled by
        # _prepared(), and idle after ACTIVE remains a hard stop.
        if ((self.phase == 'PREPARING' and self.guard_state in ('fault','paused','holding')) or
                (self.phase == 'ACTIVE' and self.guard_state in ('fault','paused','holding','idle'))):
            self._begin_stop('LOCKED','独立 guard 已撤销执行许可：'+str(self.guard_status.get('reason','状态异常')),
                             guard_already_stopped=self.guard_state in ('fault','paused','holding'))

    def _fresh_input(self):
        return (self.input is not None and self.input.connected and
                time.monotonic()-self.input_seen <= self.gp['input_timeout_s'])

    def _healthy(self):
        try:
            q = self.feedback.get(self.config['feedback_timeout'])
            self.model.check_feedback(q, self.config.get('zero_feedback_tolerance',0.) if self.mode == 'real' else 0.)
            return True
        except ValueError:
            return False

    def _current_permit(self):
        return (self._fresh_input() and self.input.lb and not self.input.b and
                self.ui_seen > 0 and time.monotonic()-self.ui_seen <= self.gp['gui_timeout_s'])

    def on_input(self, msg):
        now = time.monotonic()
        try:
            frame = InputFrame(msg.guid, int(msg.generation), int(msg.sequence), now,
                tuple(msg.axes), tuple(msg.dpad), bool(msg.lb), bool(msg.b),
                bool(msg.connected), bool(msg.link_verified), bool(msg.mapping_verified))
            self.input_gate.accept(frame, now=now)
            self.input, self.input_seen, self.input_error = frame, now, ''
            self.device_name=msg.device_name; self.connection_type=msg.connection_type; self.input_source=msg.source
        except ValueError as exc:
            self.input_error = str(exc)
            if '连接已变化' in str(exc):
                self.input = frame
                self.input_seen = now
                if self.phase in ('ACTIVE','PREPARING','ARMED'):
                    self._begin_stop('LOCKED', self.input_error)
            return
        if self.mode == 'real' and (msg.source == 'fixture' or not msg.link_verified or not msg.mapping_verified):
            if self.phase in ('ACTIVE','PREPARING','ARMED'):
                self._begin_stop('LOCKED','该输入链路未通过实机失联验证')
            return
        if frame.b:
            self._begin_stop('LOCKED','手柄 B 停止锁定')
            return
        if self.phase in ('ACTIVE','PREPARING'):
            if not frame.lb:
                self._begin_stop('ARMED','已松开 LB，等待停稳确认')
            return
        if self.phase == 'ARMED':
            self.control.update(frame, healthy=self._healthy(), speed_scale=self.speed_scale, now=now)
            if not frame.lb and ControlState.neutral(frame):
                self.reason = '等待按住 LB' if self.control.neutral_seen else '请先松开 LB 并保持摇杆回中'
            elif frame.lb:
                if self.control.state == 'ACTIVE':
                    self._begin_prepare()
                else:
                    self.reason = '请松开 LB 并让摇杆回中至少 150 ms'

    def _begin_prepare(self):
        if not self._healthy() or not self._fresh_input() or not self.input.lb:
            self.reason = '准备条件未满足：检查反馈、输入和 LB'
            return
        if self.mode == 'real' and (not self.input.link_verified or not self.input.mapping_verified):
            self._begin_stop('LOCKED','实机模式拒绝未验证的输入链路或映射')
            return
        if self.guard_state not in ('idle','fault','paused','holding'):
            self.reason = f'等待看门狗可准备（当前 {self.guard_state}）'
            return
        if not self.guard_client.service_is_ready():
            self.reason = '独立看门狗服务尚未就绪'
            return
        self.phase, self.reason = 'PREPARING','正在同步控制器；不会重放旧输入'
        self.epoch += 1; epoch=self.epoch
        future = self.guard_client.call_async(SetBool.Request(data=True))
        self.permit_future = future
        future.add_done_callback(lambda f, e=epoch: self._prepared(f,e))

    def _prepared(self, future, epoch):
        if epoch != self.epoch or self.phase != 'PREPARING':
            try:
                if future.result().success: self._request_guard(False)
            except Exception: pass
            return
        try: response=future.result()
        except Exception as exc:
            self._begin_stop('LOCKED',f'看门狗准备请求失败：{exc}'); return
        if not response.success:
            # Guard rejections are fail-closed and happen before the executor
            # opens its gate. Keep the session armed so transient startup
            # readiness (for example, the JTC subscriber coming online) does
            # not force a recovery cycle. Require a fresh neutral + LB press.
            self.control.start()
            self.get_logger().warning('看门狗拒绝准备：'+response.message)
            self.phase, self.reason = 'ARMED', '看门狗暂不可准备：'+response.message+'；松开 LB、回中后重试'
            return
        self.reason='等待看门狗同步完成'

    def _request_guard(self, enabled):
        if self.guard_client.service_is_ready():
            return self.guard_client.call_async(SetBool.Request(data=enabled))
        return None

    def _begin_stop(self, destination, reason, guard_already_stopped=False):
        if self.phase == 'WAITING' and destination == 'WAITING': return
        if self.phase == 'LOCKED':
            generation = int(self.guard_status.get('stop_generation', 0))
            stopped_at = float(self.guard_status.get('stop_requested_at', 0.))
            if (self.guard_status.get('stop_confirmed') and
                    generation >= self.stop_generation_required and
                    stopped_at >= self.stop_started-.10):
                return
        if self.phase in ('STOPPING','STOP_UNKNOWN'):
            if destination in ('LOCKED','WAITING'):
                self.after_stop=destination
                self.reason=reason
            return
        self.epoch += 1
        self.phase, self.reason, self.after_stop = 'STOPPING', reason, destination
        self.stop_started = time.monotonic()
        generation = int(self.guard_status.get('stop_generation', 0))
        self.stop_generation_required = generation if guard_already_stopped else generation+1
        self.get_logger().warning(f'手柄控制请求停止 [{destination}] {reason}')
        self._publish_zero()
        self.disable_future = None if guard_already_stopped else self._request_guard(False)

    def _publish_zero(self):
        msg=JointJog(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.joint_names=JOINTS; msg.velocities=[0.]*6
        msg.duration=.10
        self.jog_pub.publish(msg)

    def start_session(self, request, response):
        if self.phase != 'WAITING': response.success=False; response.message='当前已有会话或停止锁定'; return response
        if not self.servo_switched: response.success=False; response.message='等待 MoveIt Servo 切换至 JOINT_JOG'; return response
        if not self._healthy(): response.success=False; response.message='等待新鲜关节反馈'; return response
        self.session += 1; self.control.start(); self.phase='ARMED'
        self.reason='请松开 LB 并让摇杆回中至少 150 ms'
        response.success=True; response.message=self.reason; return response

    def end_session(self, request, response):
        if self.phase == 'WAITING': response.success=True; response.message='没有活动会话'; return response
        self._begin_stop('WAITING','正在结束会话并确认停止')
        response.success=True; response.message=self.reason; return response

    def stop_session(self, request, response):
        self._begin_stop('LOCKED','软件停止锁定；等待实测停稳确认')
        response.success=True; response.message=self.reason; return response

    def recover_session(self, request, response):
        frame=self.input
        if self.phase not in ('LOCKED','STOP_UNKNOWN'):
            response.success=False; response.message='当前不是停止锁定状态'; return response
        if not self.guard_status.get('stop_confirmed',False):
            response.success=False; response.message='停止尚未得到新鲜实测反馈确认'; return response
        if (not self._healthy() or not self._fresh_input() or frame is None or frame.b or
                not ControlState.neutral(frame)):
            response.success=False; response.message='请恢复新鲜反馈，松开 B/LB 并让摇杆回中'; return response
        if self.mode == 'real' and not frame.link_verified:
            response.success=False; response.message='当前输入链路未通过实机失联验证'; return response
        self.epoch += 1; self.control.start(); self.phase='ARMED'; self.reason='恢复完成，等待 LB 松开回中后重新按下'
        response.success=True; response.message=self.reason; return response

    def tick(self):
        now=time.monotonic()
        self.heartbeat.publish(Float64(data=time.time()))
        if not self.servo_switched:
            if self.servo_switch_future is None and self.servo_switch.service_is_ready():
                self.servo_switch_future = self.servo_switch.call_async(ServoCommandType.Request(
                    command_type=ServoCommandType.Request.JOINT_JOG))
            if self.servo_switch_future is not None and self.servo_switch_future.done():
                try:
                    result = self.servo_switch_future.result()
                    self.servo_switched = bool(result.success)
                    self.reason = ('MoveIt Servo 已切换至 JOINT_JOG' if self.servo_switched else
                                   'MoveIt Servo 拒绝 JOINT_JOG 模式')
                except Exception as exc:
                    self.reason = f'MoveIt Servo 模式切换失败：{exc}'
                self.servo_switch_future = None
            if not self.servo_switched:
                return
        if self.phase in ('PREPARING','ACTIVE'):
            if not self.guard_seen or now-self.guard_seen>.35: self._begin_stop('LOCKED','独立 guard 状态/心跳中断')
            elif self.guard_state!='running' and self.phase=='ACTIVE': self._begin_stop('LOCKED','guard 未处于 running 状态')
            elif not self._fresh_input(): self._begin_stop('LOCKED','手柄输入超过 200 ms 未更新')
            elif not self._healthy(): self._begin_stop('LOCKED','真实关节反馈缺失或超时')
            elif not self.ui_seen or now-self.ui_seen>self.gp['gui_timeout_s']:
                self._begin_stop('LOCKED','Qt 界面心跳超时')
            elif not self.input.lb or self.input.b:
                self._begin_stop('ARMED' if not self.input.lb else 'LOCKED','LB 许可撤销或按下 B')
        if self.phase == 'STOPPING':
            self._publish_zero()
            if now-self.stop_started > self.gp['stop_confirm_timeout_s']:
                self.phase='STOP_UNKNOWN'; self.reason='无法在期限内确认停稳；禁止恢复'
        if self.phase == 'ACTIVE' and self.guard_state == 'running':
            try:
                q=self.feedback.get(self.config['feedback_timeout'])
                self.model.check_feedback(q, self.config.get('zero_feedback_tolerance',0.) if self.mode=='real' else 0.)
                twist=map_velocity(self.input, self.speed_scale,
                    self.gp['deadzone'], self.gp['linear_speed_max'],
                    self.gp['angular_speed_max'], self.gp['dpad_scale'], self.gp['response_exponent'])
                velocity=damped_joint_velocity(q, twist, self.model, self.velocity_config, self.speed_scale)
                msg=JointJog(); msg.header.stamp=self.get_clock().now().to_msg()
                msg.joint_names=JOINTS; msg.velocities=velocity.tolist()
                msg.duration=float(self.gp['command_timeout_s'])
                self.jog_pub.publish(msg)
            except (ValueError, np.linalg.LinAlgError) as exc:
                self._begin_stop('LOCKED',f'速度映射/反馈校验失败：{exc}')
        elif self.phase not in ('ACTIVE','PREPARING'):
            self._publish_zero()

    def publish_status(self):
        now=time.monotonic(); data={'mode':self.mode,'state':self.phase,'reason':self.reason,
            'servo_switched':self.servo_switched,
            'session':self.session,'input_age_s':None if not self.input_seen else now-self.input_seen,
            'input_error':self.input_error,'device':None if self.input is None else self.input.guid,
            'device_name':self.device_name,'connection_type':self.connection_type,'input_source':self.input_source,
            'generation':None if self.input is None else self.input.generation,
            'stop_generation_required':self.stop_generation_required,
            'input_lb':bool(self.input and self.input.lb),'input_b':bool(self.input and self.input.b),
            'input_axes':None if self.input is None else [float(v) for v in self.input.axes],
            'input_dpad':None if self.input is None else [int(v) for v in self.input.dpad],
            'control_state':self.control.state,'neutral_seen':self.control.neutral_seen,
            'link_verified':bool(self.input and self.input.link_verified),
            'mapping_verified':bool(self.input and self.input.mapping_verified),
            'guard_state':self.guard_state,'stop_confirmed':bool(self.guard_status.get('stop_confirmed',False)),
            'speed_scale':self.speed_scale,'feedback':None}
        try:
            q=self.feedback.get(self.config['feedback_timeout']); pose=self.model.pose(q)
            from scipy.spatial.transform import Rotation
            data['feedback']={'xyz_m':pose[:3].tolist(),'rpy_rad':Rotation.from_quat(pose[3:]).as_euler('xyz').tolist(),
                'joints_rad':q.tolist(),'source_stamp':self.feedback.stamp,'age_s':now-self.feedback.received}
        except ValueError: data['feedback_error']='反馈中断 / 停稳未知'
        self.status_pub.publish(String(data=json.dumps(data,ensure_ascii=False,allow_nan=False)))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=GamepadBackend()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node._begin_stop('WAITING','后端退出，正在请求独立看门狗停止')
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=='__main__': main()
