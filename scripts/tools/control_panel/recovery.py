"""Bounded raw-CAN mode/enable recovery; no automatic reset or joint targets."""
import time

from .core import COMMANDS, home_frames

RESPONSE_TIMEOUT = 2.0
ENABLE_TIMEOUT = 3.0
RETRY_INTERVAL = 0.1
FRESHNESS = 0.5


class State:
    def __init__(self):
        self.status = None
        self.motors = {}

    def update(self, frame):
        if frame is None:
            return
        identifier, payload = frame
        now = time.monotonic()
        if identifier == 0x2A1:
            self.status = (payload, now)
        elif 0x261 <= identifier <= 0x266:
            self.motors[identifier - 0x260] = (payload[5], now)

    def status_data(self):
        if self.status and time.monotonic() - self.status[1] <= FRESHNESS:
            return self.status[0]
        return None

    def motor_data(self, joint):
        entry = self.motors.get(joint)
        return entry[0] if entry and time.monotonic() - entry[1] <= FRESHNESS else None

    def check_fault(self):
        data = self.status_data()
        if data is not None:
            if data[1] == 1:
                raise RuntimeError('控制器仍在急停：支撑机械臂后点击重置，再使能、回零')
            # Brake-not-released may clear during enable. Never move with it set.
            if data[1] not in (0, 6) or data[6] or data[7]:
                raise RuntimeError(f'控制器异常：状态 0x{data[1]:02X}，故障码 {data[6:8].hex().upper()}')
        for joint in range(1, 7):
            flags = self.motor_data(joint)
            if flags is not None and flags & 0xBF:
                raise RuntimeError(f'J{joint} 驱动器异常：状态 0x{flags:02X}')

    def enabled(self):
        return all((self.motor_data(joint) or 0) & 0x40 for joint in range(1, 7))

    def explain(self):
        data = self.status_data()
        if data is None:
            return '未收到有效的机械臂状态反馈（0x2A1）；请检查所选端口和机械臂供电'
        missing = [str(j) for j in range(1, 7) if self.motor_data(j) is None]
        if missing:
            return '缺少 J'+ '/J'.join(missing)+' 的有效使能反馈'
        disabled = [str(j) for j in range(1, 7) if not self.motor_data(j) & 0x40]
        if disabled:
            return 'J'+'/J'.join(disabled)+' 未使能，请先点击机械臂使能'
        return f'模式未就绪：控制模式 {data[0]}，运动模式 {data[2]}，状态 {data[1]}'


def prepare_motion(reader, send, check, *, speed, enable, note):
    """Confirm real state, establish CAN/MOVE J, optionally enable six motors.

    Only 0x151 / 0x471 may be repeated, for a bounded duration. There is no
    0x150 reset here: leaving emergency stop remains an explicit user action.
    """
    state = State()
    deadline = time.monotonic() + RESPONSE_TIMEOUT
    # Do not try to enable an arm whose current controller state is unknown.
    while state.status_data() is None or (not enable and any(
            state.motor_data(j) is None for j in range(1, 7))):
        check()
        if time.monotonic() >= deadline:
            raise RuntimeError(state.explain())
        state.update(reader.receive())
        state.check_fault()
    state.check_fault()
    if not enable and not state.enabled():
        raise RuntimeError(state.explain())
    note('正在恢复 CAN 运动模式并使能六轴' if enable else '正在确认回零运动模式')
    mode = home_frames(speed)[0]
    check()
    send(*mode)
    # Discard pre-command mode confirmation. Motor observations remain bounded
    # by FRESHNESS; the receiver exists only for this user-triggered operation.
    state.status = None
    deadline = time.monotonic() + (ENABLE_TIMEOUT if enable else RESPONSE_TIMEOUT)
    next_mode = time.monotonic() + RETRY_INTERVAL
    next_enable = time.monotonic() + RETRY_INTERVAL
    enable_sent = not enable
    while True:
        check()
        now = time.monotonic()
        if now >= deadline:
            raise RuntimeError('恢复超时：'+state.explain())
        state.update(reader.receive())
        check()
        state.check_fault()
        data = state.status_data()
        if enable_sent and data is not None and data[0] == 1 and data[1] == 0 and data[2] == 1 and state.enabled():
            return
        now = time.monotonic()
        if now >= next_mode:
            send(*mode)
            next_mode = now + RETRY_INTERVAL
        if enable and now >= next_enable:
            send(*COMMANDS['enable'])
            if not enable_sent:
                state.motors.clear()
            enable_sent = True
            next_enable = now + RETRY_INTERVAL
