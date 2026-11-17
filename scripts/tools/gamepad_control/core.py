"""Pure input validation and fixed Direwolf 2 velocity mapping."""
from dataclasses import dataclass
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


def map_velocity(frame: InputFrame, speed_scale=.20, deadzone=.08,
                 linear_max=.05, angular_max=.25, dpad_scale=.35, response_exponent=1.5):
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
        'linear_speed_max': (.001, .05), 'angular_speed_max': (.001, .25),
        'joint_speed': (.01, .30), 'joint_acceleration': (.05, .50),
        'default_speed_scale': (.05, 1.), 'dpad_scale': (0., 1.),
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
    def __init__(self):
        self.state = 'WAITING'
        self.reason = '等待手柄输入'
        self.neutral_seen = False
        self.neutral_since = None
        self.lb_was_down = False
        self.session = 0
        self.neutral_dwell_s = .15

    @staticmethod
    def neutral(frame, deadzone=.08):
        return not frame.lb and all(abs(v) <= deadzone for v in frame.axes) and not any(frame.dpad)

    def start(self):
        self.session += 1
        self.state, self.reason = 'ARMED', '等待松开 LB 并回中'
        self.neutral_seen = self.lb_was_down = False

    def update(self, frame, healthy=True, speed_scale=.20, now=None):
        now = frame.received_at if now is None else now
        if frame.b or not healthy:
            self.state = 'LOCKED'
            self.reason = '停止锁定' if frame.b else '输入或反馈异常'
            self.neutral_seen = False
            return (0.,)*6
        if self.state not in ('ARMED', 'ACTIVE'):
            return (0.,)*6
        if self.state == 'ACTIVE' and not frame.lb:
            self.state, self.reason, self.lb_was_down = 'ARMED', '等待停稳确认', False
            self.neutral_since = None
            self.neutral_seen = False
            return (0.,)*6
        if self.state == 'ARMED' and not frame.lb:
            if self.neutral(frame):
                if self.neutral_since is None:
                    self.neutral_since = now
                self.neutral_seen = now-self.neutral_since >= self.neutral_dwell_s
            else:
                self.neutral_since = None
                self.neutral_seen = False
            self.reason = '等待按住 LB' if self.neutral_seen else '请先松开 LB 并让摇杆回中'
            return (0.,)*6
        if self.state == 'ARMED':
            if not self.neutral_seen:
                self.reason = '请先松开 LB 并让摇杆回中'
                return (0.,)*6
            self.state, self.reason = 'ACTIVE', '控制中 · 松开 LB 停止'
            self.lb_was_down = True
        return map_velocity(frame, speed_scale=speed_scale)

    def recover(self, healthy):
        if self.state != 'LOCKED' or not healthy:
            return False
        self.start()
        return True
