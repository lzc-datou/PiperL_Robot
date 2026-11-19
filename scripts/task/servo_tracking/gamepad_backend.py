"""Backend-owned single-joint and base-frame TCP velocity teleoperation."""
import json
import os
import sys
import time
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
from piper_servo_tracking.srv import SelectGamepadControl, PlanJointTarget
from .core import JOINTS, FreshJoints, RobotModel, load_config, damped_joint_velocity, joint_speed_limits
from .stop_observer import StopObserver
from scripts.tools.gamepad_control.core import (
    InputFrame, InputGate, map_velocity, map_single_axis_velocity, input_is_neutral,
    ControlState, ControlSelection, VelocityIntentFilter, stop_confirmed_for_generation,
    home_input_neutral)
from scripts.tools.gamepad_control.motion import limit_joint_velocity
from scripts.tools.gamepad_control.settings import load_tool_config


HOME_STATES = ('HOME_ARMING', 'HOME_PLANNING', 'HOMING')
HOME_HOLD_SECONDS = 1.0
HOME_SPEED_SCALE = .10


class GamepadBackend(Node):
    def __init__(self):
        super().__init__('gamepad_backend')
        for name, default in (('config_file',''),('robot_description',''),('mode','sim'),('gamepad_config','')):
            self.declare_parameter(name, default)
        self.mode = self.get_parameter('mode').value
        self.config = load_config(self.get_parameter('config_file').value)
        gp_path = self.get_parameter('gamepad_config').value
        self.gp = load_tool_config(gp_path)
        self.intent_filter = VelocityIntentFilter(self.gp['input_filter_time_constant_s'])
        self.requested_tcp_velocity = self.filtered_tcp_velocity = (0.,)*6
        self.requested_joint_velocity = self.filtered_joint_velocity = (0.,)*6
        self.bound_scale = 1.
        self.selection = ControlSelection(speed_scales={
            name: self.gp['default_speed_scale'] for name in ('single_axis', 'cartesian')})
        self.selection_observer = StopObserver(self.gp['stop_settle_window_s'], self.gp['stop_joint_span_rad'])
        self.navigation_down = (False, False)
        self.model = RobotModel(self.get_parameter('robot_description').value)
        self.feedback = FreshJoints()
        self.input_gate = InputGate(self.gp['input_timeout_s'])
        self.control = ControlState(self.gp['neutral_dwell_s'], self.gp['deadzone'])
        self.input = None; self.input_seen = 0.; self.input_error = '等待手柄输入'
        self.device_name=''; self.connection_type='unknown'; self.input_source=''
        self.ui_seen = 0.
        self.session = 0; self.epoch = 0; self.permit_future = None
        self.phase, self.reason = 'WAITING', '等待手柄与反馈'
        self.guard_state, self.guard_status = 'unknown', {}
        self.guard_seen=0.
        self.after_stop = 'ARMED'; self.stop_started = 0.; self.disable_future = None
        self.stop_generation_required = 0
        self.stop_reason = ''
        self.home_hold_started = None
        self.home_plan_id = ''
        self.home_future = None
        self.home_requested_at = 0.
        self.home_guard_seen = False
        self.velocity_config = dict(self.config)
        # The launch file already merged task/gamepad limits. Do not replace
        # that effective minimum with the raw tool setting here.
        self.jog_pub = self.create_publisher(JointJog, 'servo_node/delta_joint_cmds', 1)
        self.heartbeat = self.create_publisher(Float64, 'backend/heartbeat', 1)
        self.status_pub = self.create_publisher(String, 'gamepad/backend_status', 10)
        self.guard_client = self.create_client(SetBool, 'guard/enable')
        self.home_client = self.create_client(PlanJointTarget, 'guard/plan_joint_target')
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
        self.create_service(SelectGamepadControl, 'gamepad/select_control', self.select_control)
        self.create_timer(1./self.gp['control_rate_hz'], self.tick)
        self.create_timer(.1, self.publish_status)

    def on_feedback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        previous = self.feedback.stamp
        self.feedback.update(msg.name, msg.position, stamp)
        if self.feedback.stamp > previous and not self.feedback.error:
            self.selection_observer.observe(self.feedback.stamp, self.feedback.received, self.feedback.q)

    @property
    def speed_scale(self):
        return self.selection.speed_scale

    def on_ui_heartbeat(self, msg):
        if 0 <= time.time()-msg.data < self.gp['gui_timeout_s']:
            self.ui_seen = time.monotonic()

    def on_speed(self, msg):
        if (self.phase not in ('WAITING', 'ARMED') or not self._fresh_input() or
                self.input.lb or self.input.b or self.input.home):
            return
        try:
            self.selection.set_speed(msg.data)
        except ValueError:
            return

    def _selection_ready(self, allow_navigation=False):
        now = time.monotonic()
        if (self.phase not in ('WAITING', 'ARMED') or not self._healthy() or
                not self._fresh_input() or not input_is_neutral(
                    self.input, self.gp['deadzone'], allow_navigation) or
                not self.ui_seen or now - self.ui_seen > self.gp['gui_timeout_s'] or
                not self.guard_seen or now - self.guard_seen > .35 or
                self.guard_state not in ('idle', 'fault', 'paused', 'holding') or
                not self.guard_status.get('gate_closed', False) or
                self.guard_status.get('other_action_active', False)):
            return False
        if self.stop_started and not (
                self.guard_status.get('stop_confirmed') and
                int(self.guard_status.get('stop_generation', 0)) >= self.stop_generation_required):
            return False
        return self.selection_observer.confirmed(now, self.config['feedback_timeout'])

    def _apply_selection(self, control_mode, joint_index, allow_navigation=False):
        try:
            ControlSelection._validate(control_mode, joint_index)
            if not self._selection_ready(allow_navigation):
                return False, '请等待停稳，松开 LB/B 并让摇杆回中后切换'
            if not self.selection.select(control_mode, joint_index):
                return True, '当前已选择该模式和关节'
        except ValueError as exc:
            return False, str(exc)
        self.epoch += 1
        self._publish_zero()
        # A selection never opens the guard. Its next enable must execute the
        # existing hold/sync/unpause barrier, which clears Servo's rolling state.
        self.control.start()
        if self.phase == 'WAITING':
            self.control = ControlState(self.gp['neutral_dwell_s'], self.gp['deadzone'])
        label = f'单轴控制 · J{joint_index}' if control_mode == 'single_axis' else '笛卡尔控制 · 基座坐标系'
        self.reason = label + '；松开方向键并回中后重新按 LB'
        return True, self.reason

    def select_control(self, request, response):
        response.success, response.message = self._apply_selection(request.control_mode, request.joint_index)
        response.revision = self.selection.revision
        self.publish_status()
        return response

    def on_guard(self, msg):
        try: self.guard_status = json.loads(msg.data)
        except (ValueError, TypeError): return
        self.guard_seen=time.monotonic()
        self.guard_state = self.guard_status.get('state','unknown')
        if self.phase in ('HOME_PLANNING', 'HOMING'):
            self._observe_home_status()
        if self.phase == 'PREPARING' and self.guard_state == 'running':
            if self._current_permit():
                self.phase, self.reason = 'ACTIVE', '控制中 · 松开 LB 停止'
            else:
                self._begin_stop('LOCKED','准备完成前许可已撤销')
        if self.phase == 'ACTIVE' and self.guard_state == 'running':
            self.reason = ('关节方向受限 · 可反向退回 · 松开 LB 停止'
                           if self.guard_status.get('joint_bound_limited') or self.bound_scale < .999 else
                           '控制中 · 松开 LB 停止')
        stop_generation = int(self.guard_status.get('stop_generation', 0))
        stop_requested_at = float(self.guard_status.get('stop_requested_at', 0.))
        stop_is_current = (stop_generation >= self.stop_generation_required and
                           stop_requested_at >= self.stop_started-.10)
        if (self.phase == 'STOPPING' and self.guard_status.get('stop_confirmed') and stop_is_current
                and not self.guard_status.get('pending_joint_actions', 0)):
            if self.after_stop == 'ARMED':
                error = self._stop_resume_error(time.monotonic())
                if error:
                    self.after_stop, self.stop_reason = 'LOCKED', error
            self.phase, self.reason = self.after_stop, self.stop_reason or self.guard_status.get('reason','已停止')
            if self.phase == 'ARMED':
                self.control.start()
            elif self.phase == 'WAITING':
                self.control = ControlState(self.gp['neutral_dwell_s'], self.gp['deadzone'])
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

    def _stop_resume_error(self, now):
        # A normal release may return to ARMED only while the observation
        # chain stays healthy. LB/Y themselves are deliberately not required.
        if not self._fresh_input():
            return '停止确认期间手柄输入中断'
        if not self._healthy():
            return '停止确认期间关节反馈中断'
        if not self.ui_seen or now-self.ui_seen > self.gp['gui_timeout_s']:
            return '停止确认期间界面心跳中断'
        if not self.guard_seen or now-self.guard_seen > .35:
            return '停止确认期间独立 guard 状态中断'
        if self.mode == 'real' and (not self.input.link_verified or
                not self.input.mapping_verified or self.input_source == 'fixture'):
            return '停止确认期间输入链路或映射未通过实机验证'
        return ''

    def on_input(self, msg):
        now = time.monotonic()
        previous_navigation = self.navigation_down
        try:
            frame = InputFrame(msg.guid, int(msg.generation), int(msg.sequence), now,
                tuple(msg.axes), tuple(msg.dpad), bool(msg.lb), bool(msg.b),
                bool(msg.connected), bool(msg.link_verified), bool(msg.mapping_verified),
                home=bool(msg.home))
            self.input_gate.accept(frame, now=now)
            self.input, self.input_seen, self.input_error = frame, now, ''
            self.navigation_down = (bool(frame.dpad[2]), bool(frame.dpad[3]))
            self.device_name=msg.device_name; self.connection_type=msg.connection_type; self.input_source=msg.source
        except ValueError as exc:
            self.input_error = str(exc)
            self.navigation_down = (False, False)
            self.input = None
            self.input_seen = 0.
            if '连接已变化' in str(exc):
                self.input = frame
                self.input_seen = now
            if self.phase in ('ACTIVE','PREPARING','ARMED','STOPPING','STOP_UNKNOWN') + HOME_STATES:
                self._begin_stop('LOCKED', self.input_error)
            return
        if self.mode == 'real' and (msg.source == 'fixture' or not msg.link_verified or not msg.mapping_verified):
            if self.phase in ('ACTIVE','PREPARING','ARMED','STOPPING','STOP_UNKNOWN') + HOME_STATES:
                self._begin_stop('LOCKED','该输入链路未通过实机失联验证')
            return
        if frame.b:
            self._begin_stop('LOCKED','手柄 B 停止锁定')
            return
        if self.phase in HOME_STATES or (self.phase == 'STOPPING' and self.after_stop == 'HOME_ARMING'):
            if not self._home_permit():
                self._begin_stop('ARMED', '回零已取消；松开 LB/Y 并回中后重新操作')
            return
        if frame.home:
            # Y consumes the input before Jog. Holding it never starts ordinary
            # motion, and a canceled/completed home needs a new neutral cycle.
            if self.phase in ('ARMED', 'ACTIVE', 'PREPARING'):
                if not home_input_neutral(frame, self.gp['deadzone']):
                    self._begin_stop('ARMED', '回零需要摇杆和方向键回中；请松开 LB/Y 后重试')
                elif frame.lb and self.control.neutral_seen:
                    self._begin_stop('HOME_ARMING', '等待停稳，随后保持 LB + Y 1 秒回零')
                elif not frame.lb and self.control.neutral_seen:
                    self.reason = '保持 Y 并按住 LB 1 秒回零；回零速度 10%'
                else:
                    self.reason = '请先松开 LB/Y 并让摇杆回中，再按住 LB + Y'
            return
        left, right = self.navigation_down
        if (self.selection.control_mode == 'single_axis' and left != right and
                not any(previous_navigation) and self._selection_ready(allow_navigation=True)):
            joint = max(1, min(6, self.selection.joint_index + (1 if right else -1)))
            self._apply_selection('single_axis', joint, allow_navigation=True)
        if self.phase in ('ACTIVE','PREPARING'):
            if not frame.lb:
                self._begin_stop('ARMED','已松开 LB，等待停稳确认')
            return
        if self.phase == 'ARMED':
            self.control.update(frame, healthy=self._healthy(), now=now)
            if self.control.state == 'LOCKED':
                self._begin_stop('LOCKED', self.control.reason)
                return
            if not frame.lb and ControlState.neutral(frame, self.gp['deadzone']):
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
        self.intent_filter.reset()
        self.selection_observer.reset()
        self.epoch += 1; epoch=self.epoch
        future = self.guard_client.call_async(SetBool.Request(data=True))
        self.permit_future = future
        future.add_done_callback(lambda f, e=epoch: self._prepared(f,e))

    def _prepared(self, future, epoch):
        # DDS may deliver running status before the enable service future.
        # A successful reply in this same ACTIVE epoch is still current.
        if epoch != self.epoch or self.phase not in ('PREPARING', 'ACTIVE'):
            try:
                if future.result().success: self._request_guard(False)
            except Exception: pass
            return
        try: response=future.result()
        except Exception as exc:
            self._begin_stop('LOCKED',f'看门狗准备请求失败：{exc}'); return
        if not response.success:
            if self.phase == 'ACTIVE':
                self._begin_stop('LOCKED', '看门狗许可响应与运行状态不一致：'+response.message)
                return
            # Guard rejections are fail-closed and happen before the executor
            # opens its gate. Keep the session armed so transient startup
            # readiness (for example, the JTC subscriber coming online) does
            # not force a recovery cycle. Require a fresh neutral + LB press.
            self.control.start()
            self.get_logger().warning('看门狗拒绝准备：'+response.message)
            self.phase, self.reason = 'ARMED', '看门狗暂不可准备：'+response.message+'；松开 LB、回中后重试'
            return
        if self.phase == 'PREPARING':
            self.reason='等待看门狗同步完成'

    def _request_guard(self, enabled):
        if self.guard_client.service_is_ready():
            return self.guard_client.call_async(SetBool.Request(data=enabled))
        return None

    def _home_permit(self):
        return (self._current_permit() and self.input.home and
                home_input_neutral(self.input, self.gp['deadzone']))

    def _tick_home(self, now):
        if not self._fresh_input():
            self._begin_stop('LOCKED', '回零输入超时或手柄断开')
        elif not self._healthy():
            self._begin_stop('LOCKED', '回零关节反馈缺失或超时')
        elif not self.guard_seen or now-self.guard_seen > .35:
            self._begin_stop('LOCKED', '回零独立 guard 状态中断')
        elif not self.ui_seen or now-self.ui_seen > self.gp['gui_timeout_s']:
            self._begin_stop('LOCKED', '回零界面心跳中断')
        elif self.mode == 'real' and (not self.input.link_verified or
                not self.input.mapping_verified or self.input_source == 'fixture'):
            self._begin_stop('LOCKED', '回零输入链路或映射未通过实机验证')
        elif self.input.b:
            self._begin_stop('LOCKED', '手柄 B 停止锁定')
        elif not self._home_permit():
            self._begin_stop('ARMED', '回零已取消；松开 LB/Y 并回中后重新操作')
        elif self.phase == 'HOME_ARMING':
            ready = (self.guard_status.get('gate_closed') and
                     'epoch' in self.guard_status and
                     stop_confirmed_for_generation(self.guard_status, self.stop_generation_required) and
                     self.guard_status.get('can_plan') and
                     not self.guard_status.get('pending_joint_actions', 0))
            if not ready:
                self.home_hold_started = None
                self.reason = '等待停稳、控制门关闭和旧动作结束；请保持 LB + Y'
            else:
                if self.home_hold_started is None:
                    self.home_hold_started = now
                remaining = max(0., HOME_HOLD_SECONDS-(now-self.home_hold_started))
                self.reason = f'保持 LB + Y {remaining:.1f} 秒后回零 · 回零速度 10%'
                if remaining == 0.:
                    self._begin_home_plan(now)
        elif self.phase in ('HOME_PLANNING', 'HOMING'):
            if self.home_future is not None and not self.home_future.done() and now-self.home_requested_at > 3.:
                self._begin_stop('LOCKED', '回零请求响应超时，已撤销执行许可')
            elif (self.home_plan_id and not self.home_guard_seen and
                  now-self.home_requested_at > 1.):
                self._begin_stop('LOCKED', '未收到本次回零计划状态，已撤销执行许可')

    def _begin_home_plan(self, now):
        if not self.home_client.service_is_ready():
            self._begin_stop('ARMED', '回零规划服务未就绪；松开 LB/Y 后重试')
            return
        self.epoch += 1
        epoch = self.epoch
        self.phase, self.reason = 'HOME_PLANNING', '正在规划回零路径；请保持 LB + Y'
        self.home_requested_at = now
        self.home_plan_id, self.home_guard_seen = '', False
        self._publish_zero()
        self.selection_observer.reset()
        request = PlanJointTarget.Request(joints=[0.]*6, speed_scale=HOME_SPEED_SCALE,
            purpose='home', target_revision=epoch, auto_execute=True,
            guard_epoch=int(self.guard_status['epoch']))
        self.home_future = self.home_client.call_async(request)
        self.home_future.add_done_callback(lambda f, e=epoch: self._home_planned(f, e))

    def _home_planned(self, future, epoch):
        if epoch != self.epoch or self.phase not in ('HOME_PLANNING', 'HOMING'):
            try:
                if future.result().success:
                    self._request_guard(False)
            except Exception:
                pass
            return
        try:
            response = future.result()
            if not response.success:
                self._begin_stop('ARMED', '回零规划请求被拒绝：'+response.message)
                return
            if not response.plan_id:
                raise ValueError('缺少回零计划身份')
            self.home_plan_id = response.plan_id
            self._observe_home_status()
        except Exception as exc:
            self._begin_stop('LOCKED', f'回零规划请求失败：{exc}')

    def _observe_home_status(self):
        if self.phase not in ('HOME_PLANNING', 'HOMING'):
            return
        state = self.guard_state
        if state == 'paused' and self.guard_status.get('pause_cause') == 'gamepad_home_canceled':
            self._begin_stop('ARMED', '回零已取消；松开 LB/Y 并回中后重新操作',
                             guard_already_stopped=True)
            return
        if state in ('fault', 'paused', 'holding'):
            self._begin_stop('LOCKED', '回零许可已撤销：'+str(self.guard_status.get('reason', state)),
                             guard_already_stopped=True)
            return
        if not self.home_plan_id:
            return
        if (self.guard_status.get('plan_id') == self.home_plan_id and
                self.guard_status.get('purpose') == 'home' and
                int(self.guard_status.get('target_revision', -1)) == self.epoch):
            self.home_guard_seen = True
            if state == 'joint_done':
                # Normalize Guard back to idle and wait a new stop generation;
                # joint_done alone cannot authorize a new Jog session.
                self._begin_stop('ARMED', '已回零；松开 LB/Y 并回中后重新操作')
            elif state.startswith('joint_'):
                self.phase = 'HOMING' if state in ('joint_executing', 'joint_settling', 'joint_finishing') else 'HOME_PLANNING'
                self.reason = str(self.guard_status.get('reason', '回零处理中'))+'；请保持 LB + Y'
        elif self.home_guard_seen or (self.guard_status.get('plan_id') and state.startswith('joint_')):
            self._begin_stop('LOCKED', '回零计划身份发生变化，已撤销执行许可')

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
            if destination in ('LOCKED','WAITING') or (self.after_stop == 'HOME_ARMING' and destination == 'ARMED'):
                self.after_stop=destination
                self.reason=self.stop_reason=reason
                self.home_hold_started = None
            return
        self.epoch += 1
        self.phase, self.reason, self.after_stop = 'STOPPING', reason, destination
        self.stop_reason = reason
        self.home_hold_started = None
        self.home_plan_id, self.home_guard_seen = '', False
        self.home_future = None
        self.stop_started = time.monotonic()
        generation = int(self.guard_status.get('stop_generation', 0))
        self.stop_generation_required = generation if guard_already_stopped else generation+1
        self.get_logger().warning(f'手柄控制请求停止 [{destination}] {reason}')
        self._publish_zero()
        self.selection_observer.reset()
        self.disable_future = None if guard_already_stopped else self._request_guard(False)

    def _publish_zero(self):
        self.intent_filter.reset()
        self.requested_tcp_velocity = self.filtered_tcp_velocity = (0.,)*6
        self.requested_joint_velocity = self.filtered_joint_velocity = (0.,)*6
        self.bound_scale = 1.
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
        if (not self.guard_seen or time.monotonic()-self.guard_seen > .35 or
                not stop_confirmed_for_generation(self.guard_status, self.stop_generation_required)):
            response.success=False; response.message='停止尚未得到新鲜实测反馈确认'; return response
        if (not self._healthy() or not self._fresh_input() or frame is None or frame.b or
                not ControlState.neutral(frame, self.gp['deadzone'])):
            response.success=False; response.message='请恢复新鲜反馈，松开 B/LB/Y 并让摇杆回中'; return response
        if self.mode == 'real' and (not frame.link_verified or not frame.mapping_verified or self.input_source == 'fixture'):
            response.success=False; response.message='当前输入链路或映射未通过实机验证'; return response
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
        if self.phase in HOME_STATES or (self.phase == 'STOPPING' and self.after_stop == 'HOME_ARMING'):
            self._tick_home(now)
        if self.phase == 'STOPPING':
            if self.after_stop == 'ARMED':
                error = self._stop_resume_error(now)
                if error:
                    self._begin_stop('LOCKED', error)
            self._publish_zero()
            if now-self.stop_started > self.gp['stop_confirm_timeout_s']:
                self.phase='STOP_UNKNOWN'; self.reason='无法在期限内确认停稳；禁止恢复'
        if self.phase == 'ACTIVE' and self.guard_state == 'running':
            try:
                q=self.feedback.get(self.config['feedback_timeout'])
                self.model.check_feedback(q, self.config.get('zero_feedback_tolerance',0.) if self.mode=='real' else 0.)
                if self.selection.control_mode == 'single_axis':
                    index = self.selection.joint_index - 1
                    maximum = float(joint_speed_limits(self.velocity_config)[index])
                    target = map_single_axis_velocity(self.input, self.selection.joint_index,
                        self.speed_scale, self.gp['deadzone'], maximum, self.gp['response_exponent'])
                    self.requested_joint_velocity = target
                    velocity = self.intent_filter.update(target, now)
                    self.filtered_joint_velocity = velocity
                else:
                    twist=map_velocity(self.input, self.speed_scale,
                        self.gp['deadzone'], self.gp['linear_speed_max'],
                        self.gp['angular_speed_max'], self.gp['dpad_scale'], self.gp['response_exponent'])
                    self.requested_tcp_velocity = twist
                    twist=self.intent_filter.update(twist, now)
                    self.filtered_tcp_velocity = twist
                    velocity=damped_joint_velocity(q, twist, self.model, self.velocity_config, self.speed_scale)
                velocity, self.bound_scale = limit_joint_velocity(q, velocity, self.model.lower,
                    self.model.upper, self.config['joint_acceleration'], self.config['joint_margin'],
                    .08 + self.gp['input_filter_time_constant_s'] + 1./self.gp['control_rate_hz'])
                msg=JointJog(); msg.header.stamp=self.get_clock().now().to_msg()
                msg.joint_names=JOINTS; msg.velocities=list(velocity)
                msg.duration=float(self.gp['command_timeout_s'])
                self.jog_pub.publish(msg)
            except (ValueError, np.linalg.LinAlgError) as exc:
                self._begin_stop('LOCKED',f'速度映射/反馈校验失败：{exc}')
        elif self.phase not in ('ACTIVE','PREPARING'):
            self._publish_zero()

    def publish_status(self):
        now=time.monotonic(); data={'mode':self.mode,'state':self.phase,'reason':self.reason,
            'control_mode':self.selection.control_mode,'selected_joint':self.selection.joint_index,
            'selection_revision':self.selection.revision,'selection_ready':self._selection_ready(),
            'speed_scales':dict(self.selection.speed_scales),'dpad_scale':self.gp['dpad_scale'],
            'joint_limits_rad':{'lower':self.model.lower.tolist(),'upper':self.model.upper.tolist()},
            'servo_switched':self.servo_switched,
            'session':self.session,'input_age_s':None if not self.input_seen else now-self.input_seen,
            'input_error':self.input_error,'device':None if self.input is None else self.input.guid,
            'device_name':self.device_name,'connection_type':self.connection_type,'input_source':self.input_source,
            'generation':None if self.input is None else self.input.generation,
            'stop_generation_required':self.stop_generation_required,
            'input_lb':bool(self.input and self.input.lb),'input_b':bool(self.input and self.input.b),
            'input_home':bool(self.input and self.input.home),
            'home_speed_scale':HOME_SPEED_SCALE, 'home_plan_id':self.home_plan_id,
            'home_hold_progress':min(1., max(0., (now-self.home_hold_started)/HOME_HOLD_SECONDS))
                if self.home_hold_started is not None else 0.,
            'input_axes':None if self.input is None else [float(v) for v in self.input.axes],
            'input_dpad':None if self.input is None else [int(v) for v in self.input.dpad],
            'control_state':self.control.state,'neutral_seen':self.control.neutral_seen,
            'link_verified':bool(self.input and self.input.link_verified),
            'mapping_verified':bool(self.input and self.input.mapping_verified),
            'guard_state':self.guard_state,'stop_confirmed':bool(self.guard_status.get('stop_confirmed',False)),
            'joint_bound_limited':bool(self.guard_status.get('joint_bound_limited',False) or self.bound_scale < .999),
            'bound_velocity_scale':self.bound_scale,
            'speed_limits':{'linear_m_s':self.gp['linear_speed_max'], 'angular_rad_s':self.gp['angular_speed_max'],
                            'joint_rad_s':float(self.config['joint_speed']), 'joint_acceleration_rad_s2':float(self.config['joint_acceleration'])},
            'input_filter_time_constant_s':self.gp['input_filter_time_constant_s'],
            'requested_tcp_velocity':list(self.requested_tcp_velocity),
            'filtered_tcp_velocity':list(self.filtered_tcp_velocity),
            'requested_joint_velocity':list(self.requested_joint_velocity),
            'filtered_joint_velocity':list(self.filtered_joint_velocity),
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
