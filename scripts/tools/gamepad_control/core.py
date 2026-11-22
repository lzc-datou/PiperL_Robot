"""Pure gamepad input, control selection, and velocity intent."""
from dataclasses import dataclass, field
import math
import time


@dataclass(frozen=True)
class InputFrame:
    guid: str
    generation: int
    sequence: int
    received_at: float
    axes: tuple[float, float, float, float]
    dpad: tuple[int, int, int, int]  # up, down, left, right
    lb: bool
    b: bool
    connected: bool = True
    link_verified: bool = False
    mapping_verified: bool = False
    home: bool = False


CONTROL_MODE_LABELS = {
    'single_axis': '单轴控制',
    'cartesian': '笛卡尔控制',
    'cartesian_axis': '笛卡尔单轴',
}
CONTROL_MODES = tuple(CONTROL_MODE_LABELS)
CARTESIAN_AXES = ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
SELECTABLE_AXES = {
    'single_axis': tuple(f'J{i}' for i in range(1, 7)),
    'cartesian_axis': CARTESIAN_AXES,
}


@dataclass
class ControlSelection:
    """Backend-owned mode, remembered axes and per-mode speed scales."""
    control_mode: str = 'single_axis'
    joint_index: int = 1
    speed_scales: dict = field(default_factory=lambda: dict.fromkeys(CONTROL_MODES, .20))
    revision: int = 0
    cartesian_axis_index: int = 1

    def __post_init__(self):
        self.validate_choice(self.control_mode, self.joint_index)
        self.validate_choice('cartesian_axis', self.cartesian_axis_index)
        self.speed_scales = dict(self.speed_scales)
        if set(self.speed_scales) != set(CONTROL_MODES):
            raise ValueError('所有控制模式均需设置速度比例')
        for value in self.speed_scales.values():
            self._validate_speed(value)

    @staticmethod
    def validate_choice(control_mode, axis_index):
        if control_mode not in CONTROL_MODES:
            raise ValueError('未知手柄控制方式')
        if isinstance(axis_index, bool) or not isinstance(axis_index, int) or not 1 <= axis_index <= 6:
            raise ValueError('选轴编号必须在 1 到 6 之间')

    @property
    def axis_index(self):
        return self.cartesian_axis_index if self.control_mode == 'cartesian_axis' else self.joint_index

    @staticmethod
    def _validate_speed(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not .05 <= value <= 1.:
            raise ValueError('速度比例必须在 5% 到 100% 之间')

    @property
    def speed_scale(self):
        return self.speed_scales[self.control_mode]

    def set_speed(self, value):
        self._validate_speed(value)
        self.speed_scales[self.control_mode] = float(value)

    def select(self, control_mode, axis_index):
        self.validate_choice(control_mode, axis_index)
        if (control_mode, axis_index) == (self.control_mode, self.axis_index):
            return False
        self.control_mode = control_mode
        if control_mode == 'cartesian_axis':
            self.cartesian_axis_index = axis_index
        else:
            self.joint_index = axis_index
        self.revision += 1
        return True


def _finite_axes(axes):
    if len(axes) != 4:
        raise ValueError('手柄输入必须包含四个摇杆轴')
    result = tuple(float(v) for v in axes)
    if not all(math.isfinite(v) and -1.001 <= v <= 1.001 for v in result):
        raise ValueError('摇杆输入不是有限的 [-1, 1] 数值')
    return tuple(max(-1., min(1., v)) for v in result)


def shape_axis(value, deadzone=.08, exponent=1.5):
    """Continuous monotonic dead-zone remap, with full scale at the edge."""
    v = max(-1., min(1., float(value)))
    a = abs(v)
    if a <= deadzone:
        return 0.
    return math.copysign(((a-deadzone)/(1.-deadzone)) ** exponent, v)


def _cap(values, limit):
    norm = math.sqrt(sum(v*v for v in values))
    factor = min(1., limit / max(norm, 1e-12))
    return tuple(v*factor for v in values)


class VelocityIntentFilter:
    """One time-based filter for six-axis velocity intent, before joint solving.

    The owner resets it whenever motion permission ends. Stop requests never
    pass through this filter. It does not scale speed or generate trajectories.
    """
    def __init__(self, time_constant_s):
        self.time_constant_s = float(time_constant_s)
        self.reset()

    def reset(self):
        self.value = (0.,)*6
        self.last_time = None

    def update(self, target, now):
        target = tuple(float(v) for v in target)
        if len(target) != 6 or not all(math.isfinite(v) for v in target) or not math.isfinite(now):
            raise ValueError('手柄速度滤波需要六维有限速度和有效时间')
        if self.last_time is None:
            self.last_time = now
            return self.value
        elapsed = now-self.last_time
        if elapsed <= 0 or elapsed > .10:
            self.reset()
            raise ValueError('手柄控制周期中断，已清除速度历史，请检查后恢复')
        self.last_time = now
        alpha = -math.expm1(-elapsed/self.time_constant_s)
        self.value = tuple(old+alpha*(new-old) for old,new in zip(self.value,target))
        return self.value


def map_velocity(frame: InputFrame, speed_scale=.20, deadzone=.08,
                 linear_max=.30, angular_max=1.50, dpad_scale=.35, response_exponent=1.5):
    """Return base_link [vx,vy,vz,wx,wy,wz], SI units per second."""
    axes = _finite_axes(frame.axes)
    if not math.isfinite(speed_scale) or not .05 <= speed_scale <= 1.:
        raise ValueError('速度比例必须在 5% 到 100% 之间')
    lx, ly, rx, ry = (shape_axis(v, deadzone, response_exponent) for v in axes)
    linear = ( -ly, -lx, -ry )
    angular = (0., 0., -rx)
    up, down, left, right = frame.dpad
    def signed(pos, neg):
        return int(bool(pos)) - int(bool(neg))
    angular = (signed(left, right)*dpad_scale, signed(up, down)*dpad_scale, angular[2])
    linear = _cap(tuple(v*linear_max*speed_scale for v in linear), linear_max*speed_scale)
    angular = _cap(tuple(v*angular_max*speed_scale for v in angular), angular_max*speed_scale)
    return linear + angular


def map_single_axis_velocity(frame: InputFrame, joint_index=1, speed_scale=.20,
                             deadzone=.08, joint_max=1.50, response_exponent=1.5):
    """Return rad/s for exactly one selected joint; other input axes are unused."""
    ControlSelection.validate_choice('single_axis', joint_index)
    return _map_vertical_jog(frame, joint_index, speed_scale, deadzone, joint_max, response_exponent)


def map_cartesian_axis_velocity(frame: InputFrame, axis_index=1, speed_scale=.20,
                                deadzone=.08, linear_max=.30, angular_max=1.50, response_exponent=1.5):
    """Return a base-frame TCP velocity with at most one nonzero component."""
    ControlSelection.validate_choice('cartesian_axis', axis_index)
    maximum = linear_max if axis_index <= 3 else angular_max
    return _map_vertical_jog(frame, axis_index, speed_scale, deadzone, maximum, response_exponent)


def _map_vertical_jog(frame, axis_index, speed_scale, deadzone, maximum, response_exponent):
    ControlSelection._validate_speed(speed_scale)
    vertical = _finite_axes(frame.axes)[1]
    if not math.isfinite(maximum) or maximum <= 0:
        raise ValueError('速度上限必须为有限正数')
    velocity = [0.] * 6
    velocity[axis_index - 1] = -shape_axis(vertical, deadzone, response_exponent) * maximum * speed_scale
    return tuple(velocity)


def input_is_neutral(frame, deadzone=.08, allow_navigation=False):
    """Selection allows at most one D-pad direction with the sticks centered and LB/B/Y released."""
    return (not frame.lb and not frame.b and not frame.home and
            all(abs(v) <= deadzone for v in frame.axes) and
            sum(frame.dpad) <= (1 if allow_navigation else 0))


def home_input_neutral(frame, deadzone=.08):
    """Home permits held LB/Y, but never B, stick motion or D-pad motion."""
    return (not frame.b and all(abs(v) <= deadzone for v in frame.axes) and
            not any(frame.dpad))


class InputGate:
    """Fail-closed frame and connection-generation validator."""
    def __init__(self, timeout=.20):
        self.timeout = timeout
        self.generation = None
        self.guid = None
        self.sequence = -1
        self.last_frame = None

    def accept(self, frame, now=None):
        now = time.monotonic() if now is None else now
        if not frame.connected:
            self.last_frame = None
            raise ValueError('手柄已断开')
        if not frame.guid or frame.generation < 1:
            raise ValueError('设备身份或连接代数无效')
        if frame.generation != self.generation or frame.guid != self.guid:
            self.generation, self.guid, self.sequence = frame.generation, frame.guid, -1
            self.last_frame = None
            raise ValueError('手柄连接已变化，请松开 LB、回中并恢复控制')
        if frame.sequence <= self.sequence:
            self.last_frame = None
            raise ValueError('手柄输入乱序或重复')
        if not math.isfinite(frame.received_at) or now-frame.received_at < 0 or now-frame.received_at > self.timeout:
            self.last_frame = None
            raise ValueError('手柄输入已过期')
        _finite_axes(frame.axes)
        if len(frame.dpad) != 4 or any(v not in (0, 1) for v in frame.dpad):
            raise ValueError('十字键输入无效')
        self.sequence = frame.sequence
        self.last_frame = frame
        return frame

    def disconnected(self):
        self.last_frame = None


def validate_tool_config(data):
    """Validate independent gamepad limits before any ROS node is launched."""
    ranges = {
        'control_rate_hz': (20., 200.), 'deadzone': (0., .4),
        'neutral_dwell_s': (.05, 1.), 'response_exponent': (.5, 3.),
        'linear_speed_max': (.001, .30), 'angular_speed_max': (.001, 1.50),
        'joint_speed': (.01, 1.50), 'joint_acceleration': (.05, 1.50),
        'default_speed_scale': (.05, 1.), 'dpad_scale': (0., 1.),
        'input_filter_time_constant_s': (.01, .10),
        'input_timeout_s': (.05, .20), 'gui_timeout_s': (.10, .30),
        'command_timeout_s': (.05, .20), 'stop_settle_window_s': (.10, 1.),
        'stop_joint_span_rad': (.0001, .003), 'stop_confirm_timeout_s': (.5, 10.),
    }
    for key,(low,high) in ranges.items():
        value=data.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'gamepad 配置 {key} 必须位于 [{low}, {high}]')
    return dict(data)


def stop_confirmed_for_generation(status, required_generation):
    """A prior confirmed stop cannot satisfy a newer stop request."""
    try:
        observed = int(status.get('stop_generation', -1))
        required = int(required_generation)
    except (AttributeError, TypeError, ValueError):
        return False
    return bool(status.get('stop_confirmed')) and required >= 0 and observed >= required


class ControlState:
    """Small UI/controller-independent permit latch."""
    def __init__(self, neutral_dwell_s=.15, deadzone=.08):
        self.state = 'WAITING'
        self.reason = '等待手柄输入'
        self.neutral_seen = False
        self.neutral_since = None
        self.lb_was_down = False
        self.session = 0
        self.neutral_dwell_s = neutral_dwell_s
        self.deadzone = deadzone

    @staticmethod
    def neutral(frame, deadzone=.08):
        return input_is_neutral(frame, deadzone)

    def start(self):
        self.session += 1
        self.state, self.reason = 'ARMED', '等待松开 LB 并回中'
        self.neutral_seen = self.lb_was_down = False
        self.neutral_since = None

    def update(self, frame, healthy=True, now=None):
        now = frame.received_at if now is None else now
        if frame.b or not healthy:
            self.state = 'LOCKED'
            self.reason = '停止锁定' if frame.b else '输入或反馈异常'
            self.neutral_seen = False
            return False
        if self.state not in ('ARMED', 'ACTIVE'):
            return False
        if self.state == 'ACTIVE' and not frame.lb:
            self.state, self.reason, self.lb_was_down = 'ARMED', '等待停稳确认', False
            self.neutral_since = None
            self.neutral_seen = False
            return False
        if self.state == 'ARMED' and not frame.lb:
            if self.neutral(frame, self.deadzone):
                if self.neutral_since is None:
                    self.neutral_since = now
                self.neutral_seen = now-self.neutral_since >= self.neutral_dwell_s
            else:
                self.neutral_since = None
                self.neutral_seen = False
            self.reason = '等待按住 LB' if self.neutral_seen else '请先松开 LB 并让摇杆回中'
            return False
        if self.state == 'ARMED':
            if not self.neutral_seen:
                self.reason = '请先松开 LB 并让摇杆回中'
                return False
            self.state, self.reason = 'ACTIVE', '控制中 · 松开 LB 停止'
            self.lb_was_down = True
        return True

    def recover(self, healthy):
        if self.state != 'LOCKED' or not healthy:
            return False
        self.start()
        return True
