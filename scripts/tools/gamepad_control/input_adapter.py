"""Read-only Linux joystick input using the kernel evdev ioctls (stdlib only)."""
import glob
import fcntl
import hashlib
import os
from pathlib import Path
import select
import struct
import threading
import time

EV_KEY, EV_ABS = 1, 3
ABS_X, ABS_Y, ABS_RX, ABS_RY = 0, 1, 3, 4
ABS_HAT0X, ABS_HAT0Y = 16, 17
BTN_EAST, BTN_NORTH, BTN_WEST, BTN_TL = 0x131, 0x133, 0x134, 0x136
ABS_CODES = {'ABS_X': ABS_X, 'ABS_Y': ABS_Y, 'ABS_Z': 2,
             'ABS_RX': ABS_RX, 'ABS_RY': ABS_RY, 'ABS_RZ': 5,
             'ABS_THROTTLE': 6, 'ABS_RUDDER': 7,
             'ABS_HAT0X': ABS_HAT0X, 'ABS_HAT0Y': ABS_HAT0Y}
KEY_CODES = {'BTN_EAST': BTN_EAST, 'BTN_NORTH': BTN_NORTH,
             'BTN_WEST': BTN_WEST, 'BTN_TL': BTN_TL}
AXIS_ORDER = ('left_x', 'left_y', 'right_x', 'right_y')
LEGACY_AXIS_CODES = dict(zip(AXIS_ORDER, ('ABS_X', 'ABS_Y', 'ABS_RX', 'ABS_RY')))
D_PAD_ORDER = ('up', 'down', 'left', 'right')


def _ior(size, nr):
    return (2 << 30) | (size << 16) | (ord('E') << 8) | nr


def _device_info(fd):
    namebuf = bytearray(256)
    fcntl.ioctl(fd, _ior(len(namebuf), 0x06), namebuf, True)
    idbuf = bytearray(8)
    fcntl.ioctl(fd, _ior(len(idbuf), 0x02), idbuf, True)
    bustype, vendor, product, version = struct.unpack('=HHHH', idbuf)
    def text_ioctl(nr):
        buf = bytearray(256)
        try:
            fcntl.ioctl(fd, _ior(len(buf), nr), buf, True)
            return bytes(buf).split(b'\0',1)[0].decode(errors='replace')
        except OSError:
            return ''
    name = bytes(namebuf).split(b'\0',1)[0].decode(errors='replace')
    phys, uniq = text_ioctl(0x07), text_ioctl(0x08)
    identity = f'{bustype:04x}:{vendor:04x}:{product:04x}:{version:04x}:{name}:{phys}:{uniq}'
    guid = hashlib.sha256(identity.encode()).hexdigest()[:32]
    return {'name': name, 'phys': phys, 'uniq': uniq, 'path': '', 'guid': guid,
            'vendor': vendor, 'product': product, 'connection': '未知'}


def _absinfo(fd, code):
    buf = bytearray(24)
    try:
        fcntl.ioctl(fd, _ior(len(buf), 0x40+code), buf, True)
        return struct.unpack('=iiiiii', buf)
    except OSError:
        return None


def _normalize(value, low, high):
    if high <= low:
        raise ValueError('手柄轴量程无效')
    center = (low+high)/2.
    if value >= center:
        return min(1., (value-center)/max(high-center, 1.))
    return max(-1., (value-center)/max(center-low, 1.))


def _pressed(fd, codes):
    size = 96
    buf = bytearray(size)
    fcntl.ioctl(fd, _ior(size, 0x18), buf, True)
    return {code: bool(buf[code//8] & (1 << (code % 8))) for code in codes}


def _profile_mapping(profile):
    evdev = (profile or {}).get('evdev', {})
    axes = evdev.get('axes', {})
    axis_codes, inversions = [], []
    for role in AXIS_ORDER:
        spec = axes.get(role, axes.get(LEGACY_AXIS_CODES[role]))
        if spec is None:
            spec = {'code': LEGACY_AXIS_CODES[role], 'invert': False}
        if isinstance(spec, dict):
            code_name, invert = spec.get('code', LEGACY_AXIS_CODES[role]), bool(spec.get('invert', False))
        elif isinstance(spec, str):
            code_name, invert = spec, False
        else:
            code_name = LEGACY_AXIS_CODES[role]
            invert = bool(spec[3]) if isinstance(spec, (list, tuple)) and len(spec) > 3 else False
        if code_name not in ABS_CODES or code_name in ('ABS_HAT0X', 'ABS_HAT0Y'):
            raise ValueError(f'手柄 profile 轴 {role} 无效: {code_name}')
        axis_codes.append(ABS_CODES[code_name]); inversions.append(invert)
    buttons = evdev.get('buttons', {})
    key_codes = {}
    for name, fallback in (('lb', 'BTN_TL'), ('b', 'BTN_EAST'), ('home', 'BTN_WEST')):
        code_name = buttons.get(name, fallback)
        if code_name not in KEY_CODES:
            raise ValueError(f'手柄 profile 按键 {name} 不支持: {code_name}')
        key_codes[name] = KEY_CODES[code_name]
    if len(set(key_codes.values())) != len(key_codes):
        raise ValueError('手柄 profile 的 LB、B 和回零按键不能使用同一个键码')
    dpad_profile = evdev.get('dpad', {})
    dpad = []
    for direction, fallback in zip(D_PAD_ORDER,
            ('ABS_HAT0Y:-1', 'ABS_HAT0Y:1', 'ABS_HAT0X:-1', 'ABS_HAT0X:1')):
        spec = dpad_profile.get(direction, fallback)
        try:
            code_name, value_text = spec.split(':', 1)
            code, value = ABS_CODES[code_name], int(value_text)
        except (AttributeError, KeyError, ValueError):
            raise ValueError(f'手柄 profile 十字键 {direction} 无效: {spec}')
        if code not in (ABS_HAT0X, ABS_HAT0Y) or value not in (-1, 1):
            raise ValueError(f'手柄 profile 十字键 {direction} 无效: {spec}')
        dpad.append((code, value))
    return tuple(axis_codes), tuple(inversions), key_codes, tuple(dpad)


class EvdevInput:
    """Poll kernel state and detect device-fd removal; never claims radio-link proof."""
    def __init__(self, profile=None, link_verified=False, mapping_verified=False):
        self.profile = profile or {}
        self.axis_codes, self.axis_inversions, self.key_codes, self.dpad_codes = _profile_mapping(self.profile)
        self.link_verified = bool(link_verified)
        self.mapping_verified = bool(mapping_verified)
        self.lock = threading.Lock()
        self.frame = None
        self.running = False
        self._stop_event = threading.Event()
        self.generation = 0
        self.sequence = 0
        self.error = '尚未连接手柄'
        self._thread = None
        self._fd = None
        self.device = None

    def _supports_profile(self, path):
        """Inspect capabilities without opening keyboards or requiring device access."""
        capabilities = Path('/sys/class/input')/Path(path).name/'device/capabilities'
        def read_bits(name):
            words = (capabilities/name).read_text().split()
            # sysfs prints native unsigned-long words, including bare "0"
            # words. Concatenating them loses the spacing between high bits.
            width = struct.calcsize('@L') * 8
            return sum(int(word, 16) << (index * width)
                       for index, word in enumerate(reversed(words)))
        try:
            abs_bits = read_bits('abs')
            key_bits = read_bits('key')
        except (OSError, ValueError):
            return None  # Fall back to the device ioctls if sysfs is unavailable.
        return (all(abs_bits & (1 << code) for code in self.axis_codes) and
                all(key_bits & (1 << code) for code in self.key_codes.values()))

    def devices(self, path=None):
        result = []
        denied = []
        paths = [os.path.realpath(path)] if path else sorted(glob.glob('/dev/input/event*'))
        error = ''
        for candidate in paths:
            if self._supports_profile(candidate) is False:
                if path:
                    error = f'所选设备不支持当前手柄轴/按键配置：{candidate}'
                continue
            fd = None
            try:
                fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
                info = _device_info(fd)
                axis_info = [_absinfo(fd, code) for code in self.axis_codes]
                if all(v is not None and v[2] > v[1] for v in axis_info):
                    info['path'] = candidate
                    result.append(info)
                elif path:
                    error = f'所选设备缺少当前配置需要的四个摇杆轴：{candidate}'
            except PermissionError:
                denied.append(candidate)
            except OSError as exc:
                if path:
                    error = f'无法读取指定手柄设备 {candidate}：{exc}'
            finally:
                if fd is not None:
                    os.close(fd)
        if denied:
            error = '手柄设备无读取权限：'+', '.join(denied)+'；请检查该手柄设备的访问权限'
        if not result and not error:
            error = '系统未发现兼容手柄；请检查 PC 挡位、电源和 USB 数据连接'
        return result, error

    def _choose_device(self, candidates):
        hint = str(self.profile.get('name_hint', '')).casefold()
        named = [d for d in candidates if hint and hint in d['name'].casefold()]
        if len(named) == 1:
            return named[0]
        # XInput devices may identify as an Xbox pad instead of Flydigi.
        if len(candidates) == 1:
            return candidates[0]
        if candidates:
            paths = ', '.join(d['path'] for d in candidates)
            raise ValueError('发现多个兼容手柄，请用 --device 指定其中一个：'+paths)
        return None

    def start(self, path=None):
        if self.running:
            return
        self._stop_event.clear()
        self.running = True
        self._thread = threading.Thread(target=self._read_loop, args=(path,), daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        self._stop_event.set()
        # The reader owns its descriptor. Closing it from the UI can race with
        # a read/reconnect and accidentally close a newly reused descriptor.
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=.5)
        with self.lock:
            self.frame = None

    def _read_loop(self, path):
        while self.running:
            fd = None
            try:
                candidates, error = self.devices(path)
                device = self._choose_device(candidates)
                if device is None:
                    self.error = error
                    self._stop_event.wait(.25)
                    continue
                fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
                device = dict(device)
                self._fd, self.device = fd, device
                self.generation += 1
                self.sequence = 0
                self.error = ''
                while self.running:
                    # A successful kernel query reads the current device state;
                    # it does not re-date a cached ROS/Joy message. It still
                    # cannot prove that a wireless controller is linked to a receiver.
                    axis_info = [_absinfo(fd, code) for code in self.axis_codes]
                    if any(v is None for v in axis_info):
                        raise OSError('手柄缺少所需四个摇杆轴')
                    keys = _pressed(fd, tuple(self.key_codes.values()))
                    axes = tuple((-1. if invert else 1.)*_normalize(info[0], *info[1:3])
                                 for info, invert in zip(axis_info, self.axis_inversions))
                    hats = {code: (_absinfo(fd, code) or (0,))[0] for code in (ABS_HAT0X, ABS_HAT0Y)}
                    dpad = tuple(int(hats[code] == value) for code, value in self.dpad_codes)
                    self.sequence += 1
                    snapshot = {'guid': device['guid'], 'generation': self.generation,
                                'sequence': self.sequence, 'axes': axes, 'dpad': dpad,
                                'lb': keys[self.key_codes['lb']], 'b': keys[self.key_codes['b']],
                                'home': keys[self.key_codes['home']],
                                'connected': True, 'link_verified': self.link_verified,
                                'mapping_verified': self.mapping_verified,
                                'source': 'evdev', 'received_at': time.monotonic(), 'device': device}
                    with self.lock: self.frame = snapshot
                    # Drain events to keep the kernel queue from overflowing.
                    readable, _, _ = select.select([fd], [], [], .02)
                    if readable:
                        try: os.read(fd, 24*256)
                        except BlockingIOError: pass
            except (OSError, ValueError) as exc:
                self.generation += 1
                with self.lock: self.frame = None
                self.error = f'手柄断开或输入读取失败：{exc}'
                self._stop_event.wait(.1)
            finally:
                if fd is not None:
                    os.close(fd)
                self._fd = None
        with self.lock: self.frame = None

    def latest(self):
        with self.lock:
            if self.frame and time.monotonic()-self.frame['received_at'] <= .12:
                return dict(self.frame)
            return None
