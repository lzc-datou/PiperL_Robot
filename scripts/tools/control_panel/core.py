"""Raw SocketCAN transport and protocol frames, without ROS or SDK imports."""
import json
import shutil
import socket
import struct
import subprocess
import time

CAN_FRAME = struct.Struct('=IB3x8s')
COMMANDS = {
    'enable': (0x471, bytes.fromhex('0702000000000000')),
    'stop': (0x150, bytes.fromhex('0100000000000000')),
    'reset': (0x150, bytes.fromhex('0200000000000000')),
}


def command_frame(command):
    identifier, payload = COMMANDS[command]
    return CAN_FRAME.pack(identifier, 8, payload)


def home_frames(speed=10):
    """Ordinary MOVE J to controller zero angles, not zero calibration."""
    if not isinstance(speed, int) or not 1 <= speed <= 100:
        raise ValueError('回零速度比例必须为 1–100 的整数')
    return [(0x151, bytes([1, 1, speed, 0, 0, 0, 0, 0])),
            *[(identifier, bytes(8)) for identifier in (0x155, 0x156, 0x157)]]


def list_interfaces():
    """Map CAN names to (up, bitrate, state, ifindex).

    ifindex distinguishes a replugged adapter from its previous socket binding,
    even when the interface is recreated with the same name between scans.
    """
    result = subprocess.run(['ip', '-j', '-details', 'link', 'show'],
                            capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError('无法读取 CAN 端口：'+result.stderr.strip())
    interfaces = {}
    for link in json.loads(result.stdout):
        info = link.get('linkinfo', {})
        if info.get('info_kind') == 'can':
            data = info.get('info_data', {})
            interfaces[link['ifname']] = (
                'UP' in link.get('flags', []),
                data.get('bittiming', {}).get('bitrate'), data.get('state'),
                link.get('ifindex'))
    return dict(sorted(interfaces.items()))


def ready(status):
    return status[:3] == (True, 1000000, 'ERROR-ACTIVE')


def inspect_interface(channel):
    result = subprocess.run(['ip', '-j', '-details', 'link', 'show', 'dev', channel],
                            capture_output=True, text=True, timeout=3)
    if result.returncode:
        raise RuntimeError(f'未找到 {channel}；请检查 USB-CAN 连接和项目配置')
    link = json.loads(result.stdout)[0]
    info = link.get('linkinfo', {})
    if info.get('info_kind') != 'can':
        raise RuntimeError('所选接口不是 CAN')
    data = info.get('info_data', {})
    return ('UP' in link.get('flags', []), data.get('bittiming', {}).get('bitrate'), data.get('state'))


def activate_interface(channel, cancel=None):
    up, bitrate, state = inspect_interface(channel)
    if state == 'BUS-OFF':
        raise RuntimeError('CAN BUS-OFF；请检查接线，停止控制程序后人工恢复')
    if up:
        if bitrate != 1000000:
            raise RuntimeError('接口正在使用且不是 1 Mbps；不会自动重配，请先停止其他控制程序')
        return
    ip = shutil.which('ip')
    pkexec = shutil.which('pkexec')
    if not pkexec:
        raise RuntimeError('系统无图形授权工具；请在终端运行 scripts/piperl/activate_can.sh')
    # Only the fixed network configuration commands get elevated, never the GUI.
    for args in (['type', 'can', 'bitrate', '1000000'], ['up']):
        if cancel is not None and cancel.is_set():
            raise InterruptedError('连接操作已取消')
        with subprocess.Popen([pkexec, ip, 'link', 'set', 'dev', channel, *args],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            deadline = time.monotonic()+120
            try:
                while True:
                    if cancel is not None and cancel.is_set():
                        raise InterruptedError('连接操作已取消')
                    if time.monotonic() > deadline:
                        raise TimeoutError('CAN 授权等待超时')
                    try:
                        _, error = process.communicate(timeout=0.1)
                        break
                    except subprocess.TimeoutExpired:
                        continue
            except BaseException:
                process.kill()
                process.communicate()
                raise
            if process.returncode:
                raise RuntimeError('CAN 激活未完成：'+error.strip())
    up, bitrate, state = inspect_interface(channel)
    if not up or bitrate != 1000000 or state == 'BUS-OFF':
        raise RuntimeError('CAN 激活后的状态校验失败')


class CanTransport:
    def __init__(self, channel, socket_factory=None):
        factory = socket.socket if socket_factory is None else socket_factory
        self.tx = factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.tx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b'')
            self.tx.settimeout(0.05)
            self.tx.bind((channel,))
        except Exception:
            self.tx.close()
            raise

    def send(self, command):
        self.send_frame(*COMMANDS[command])

    def send_frame(self, identifier, payload):
        frame = CAN_FRAME.pack(identifier, len(payload), payload)
        if self.tx.send(frame) != len(frame):
            raise OSError('CAN 帧未完整发送')

    def close(self):
        self.tx.close()


class CanFeedback:
    """Temporary, filtered receiver for a single enable/home operation."""
    IDS = (0x2A1, *range(0x261, 0x267))

    def __init__(self, channel, socket_factory=None):
        factory = socket.socket if socket_factory is None else socket_factory
        self.rx = factory(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            # Match standard data frames. CAN_ERR_FLAG must NOT be in this
            # mask: SocketCAN routes such filters to the error receive list.
            # Error reception is disabled by default, independently of this.
            mask = socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG | socket.CAN_SFF_MASK
            filters = b''.join(struct.pack('=II', identifier, mask)
                               for identifier in self.IDS)
            self.rx.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.rx.settimeout(0.02)
            self.rx.bind((channel,))
        except Exception:
            self.rx.close()
            raise

    def receive(self):
        try:
            frame = self.rx.recv(CAN_FRAME.size)
        except socket.timeout:
            return None
        if len(frame) != CAN_FRAME.size:
            return None
        identifier, length, payload = CAN_FRAME.unpack(frame)
        if identifier not in self.IDS or length != 8:
            return None
        return identifier, payload

    def close(self):
        self.rx.close()
