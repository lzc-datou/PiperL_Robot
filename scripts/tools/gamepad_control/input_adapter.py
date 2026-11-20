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
BUS_USB, BUS_BLUETOOTH = 0x03, 0x05
TRANSPORT_BUSES = {'usb': BUS_USB, 'bluetooth': BUS_BLUETOOTH}
INPUT_MAX_AGE = .12
INPUT_EVENT = struct.Struct('@llHHi')


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
            'bus': bustype, 'vendor': vendor, 'product': product, 'version': version,
            'connection': {BUS_USB: 'USB', BUS_BLUETOOTH: '蓝牙'}.get(bustype, '未知')}


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


def _input_sysfs_dir(event_fd):
    device_no = os.fstat(event_fd).st_rdev
    return (Path('/sys/dev/char')/f'{os.major(device_no)}:{os.minor(device_no)}'/'device').resolve()


def _native_bluetooth_link(input_dir):
    """Find the kernel connection that actually owns this input device."""
    for parent in input_dir.parents:
        if parent.name.startswith('hci') and ':' in parent.name:
            if 'DEVTYPE=link' in (parent/'uevent').read_text().splitlines():
                return parent
    return None  # Userspace HID can have no HCI ancestor. Use input reports there.


def _open_report_stream(event_fd):
    """Open only the companion hidraw node of this evdev descriptor, read-only."""
    sys_char = Path('/sys/dev/char')
    hid = (_input_sysfs_dir(event_fd)/'device').resolve()
    for node in (hid/'hidraw').glob('hidraw*'):
        try:
            fd = os.open('/dev/' + node.name, os.O_RDONLY | os.O_NONBLOCK)
        except OSError:
            continue
        raw_no = os.fstat(fd).st_rdev
        if (sys_char/f'{os.major(raw_no)}:{os.minor(raw_no)}'/'device').resolve() == hid:
            return fd
        os.close(fd)
    return None


def _wait_for_input(fd, raw_fd, controls, input_report):
    readable, _, _ = select.select([fd] if raw_fd is None else [fd, raw_fd], [], [], .02)
    changed = False
    if fd in readable:
        try:
            events = os.read(fd, INPUT_EVENT.size * 256)
        except BlockingIOError:
            events = None
        if events == b'':
            raise OSError('手柄连接已关闭')
        if controls and events:
            changed = any((kind, code) in controls and not (kind == EV_KEY and value == 2)
                          for _, _, kind, code, value in INPUT_EVENT.iter_unpack(events))
    if raw_fd in readable:
        # Battery and system reports cannot renew a motion permit.
        for _ in range(64):
            try:
                report = os.read(raw_fd, 128)
            except BlockingIOError:
                break
            if not report:
                raise OSError('蓝牙手柄报告通道已关闭')
            if (report[0], len(report)) == input_report:
                changed = True
    return changed


def _button_mapping(buttons):
    codes = {}
    for role, fallback in (('lb', 'BTN_TL'), ('b', 'BTN_EAST'), ('home', 'BTN_WEST')):
        name = buttons.get(role, fallback)
        if name not in KEY_CODES:
            raise ValueError(f'手柄 profile 按键 {role} 不支持: {name}')
        codes[role] = KEY_CODES[name]
    if len(set(codes.values())) != len(codes):
        raise ValueError('手柄 profile 的 LB、B 和回零按键不能使用同一个键码')
    return codes


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
    key_codes = _button_mapping(evdev.get('buttons', {}))
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
    def __init__(self, profile=None, link_verified=False, mapping_verified=False, *, display_only=False):
        self.profile = profile or {}
        self.axis_codes, self.axis_inversions, self.key_codes, self.dpad_codes = _profile_mapping(self.profile)
        self.bluetooth_layouts = {
            (entry['vendor'], entry['product'], entry['version'], entry['gamepad_keymask']):
                (_button_mapping(entry['buttons']), tuple(entry['input_report']))
            for entry in self.profile.get('bluetooth_layouts', ())
        }
        self.button_layouts = [self.key_codes] + [buttons for buttons, _ in self.bluetooth_layouts.values()]
        self.link_verified = bool(link_verified)
        self.mapping_verified = bool(mapping_verified)
        self.display_only = display_only
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

    def _device_layout(self, device, key_bits):
        if device['bus'] == BUS_BLUETOOTH:
            # Bits 0x130..0x13f distinguish the compact and standard evdev layouts.
            layout = (device['vendor'], device['product'], device['version'], (key_bits >> 0x130) & 0xffff)
            if layout in self.bluetooth_layouts:
                return self.bluetooth_layouts[layout]
        return self.key_codes, None

    def _supports_profile(self, path, required_bus=None):
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
            if required_bus is not None and int((capabilities.parent/'id/bustype').read_text(), 16) != required_bus:
                return False
            abs_bits = read_bits('abs')
            key_bits = read_bits('key')
        except (OSError, ValueError):
            return None  # Fall back to the device ioctls if sysfs is unavailable.
        return (all(abs_bits & (1 << code) for code in (*self.axis_codes, ABS_HAT0X, ABS_HAT0Y)) and
                any(all(key_bits & (1 << code) for code in mapping.values())
                    for mapping in self.button_layouts))

    def devices(self, path=None):
        result = []
        denied = []
        required_bus = TRANSPORT_BUSES.get(path)
        explicit = bool(path and required_bus is None)
        paths = [os.path.realpath(path)] if explicit else sorted(glob.glob('/dev/input/event*'))
        error = ''
        for candidate in paths:
            if self._supports_profile(candidate, required_bus) is False:
                if explicit:
                    error = f'所选设备不支持当前手柄轴/按键配置：{candidate}'
                continue
            fd = None
            try:
                fd = os.open(candidate, os.O_RDONLY | os.O_NONBLOCK)
                info = _device_info(fd)
                if required_bus is not None and info['bus'] != required_bus:
                    continue
                supported = bytearray(96)
                fcntl.ioctl(fd, _ior(len(supported), 0x20 + EV_KEY), supported, True)
                key_bits = int.from_bytes(supported, 'little')
                buttons, input_report = self._device_layout(info, key_bits)
                axis_info = [_absinfo(fd, code) for code in self.axis_codes]
                if (all(v is not None and v[2] > v[1] for v in axis_info) and
                        all(key_bits & (1 << code) for code in buttons.values()) and
                        all(_absinfo(fd, code) is not None for code in (ABS_HAT0X, ABS_HAT0Y))):
                    info['path'] = candidate
                    info['buttons'] = buttons
                    info['input_report'] = input_report
                    result.append(info)
                elif explicit:
                    error = f'所选设备缺少当前配置需要的摇杆、方向键或按钮：{candidate}'
            except PermissionError:
                denied.append(candidate)
            except OSError as exc:
                if explicit:
                    error = f'无法读取指定手柄设备 {candidate}：{exc}'
            finally:
                if fd is not None:
                    os.close(fd)
        if denied:
            error = '手柄设备无读取权限：'+', '.join(denied)+'；请检查该手柄设备的访问权限'
        if not result and not error:
            if not Path('/dev/input').is_dir():
                error = '系统未发现兼容手柄：当前环境看不到 /dev/input，请在主机桌面终端运行。'
            elif required_bus == BUS_BLUETOOTH:
                error = '正在等待蓝牙手柄，请确认系统显示“已连接”且手柄处于游戏手柄模式。'
            elif required_bus == BUS_USB:
                error = '正在等待 USB 手柄，请检查手柄电源和 USB 数据线。'
            else:
                error = '系统未发现兼容手柄，请检查手柄电源、蓝牙连接或 USB 数据线。'
        return result, error

    def _choose_device(self, candidates):
        if self.device and (self.device['bus'] == BUS_BLUETOOTH or self.link_verified or self.mapping_verified):
            return next((d for d in candidates if d['guid'] == self.device['guid']), None)
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
        self.device = None
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
            raw_fd = None
            try:
                candidates, error = self.devices(path)
                device = self._choose_device(candidates)
                if device is None:
                    self.error = error or '等待原手柄重新连接'
                    self._stop_event.wait(.25)
                    continue
                fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
                if _device_info(fd)['guid'] != device['guid']:
                    raise OSError('输入节点已变更，正在重新识别手柄')
                link = _native_bluetooth_link(_input_sysfs_dir(fd)) if device['bus'] == BUS_BLUETOOTH else None
                if link is None and device['input_report']:
                    raw_fd = _open_report_stream(fd)
                self._stream_device(fd, device, raw_fd, link)
            except (OSError, ValueError) as exc:
                self.generation += 1
                with self.lock: self.frame = None
                self.error = f'手柄断开或输入读取失败：{exc}'
                self._stop_event.wait(.1)
            finally:
                if raw_fd is not None:
                    os.close(raw_fd)
                if fd is not None:
                    os.close(fd)
                self._fd = None
        with self.lock: self.frame = None

    def _stream_device(self, fd, device, raw_fd, link):
        device = dict(device)
        buttons = device['buttons']
        bluetooth = device['bus'] == BUS_BLUETOOTH
        controls = {(EV_KEY, code) for code in buttons.values()} if bluetooth else set()
        if bluetooth:
            controls.update((EV_ABS, code) for code in (*self.axis_codes, ABS_HAT0X, ABS_HAT0Y))
            device['report_source'] = 'HID 控制报告及 evdev 事件' if raw_fd is not None else 'evdev 按键与摇杆事件'
        if link is not None:
            link_id = link.stat()
            device['report_source'] = 'evdev 按键与摇杆状态 + 内核蓝牙连接'
        last_report = None
        last_poll = time.monotonic()
        held_with_report = False
        release_required = False
        self._fd, self.device = fd, device
        self.generation += 1
        self.sequence = 0
        self.error = ''
        while self.running:
            changed = _wait_for_input(fd, raw_fd, controls, device['input_report'])
            now = time.monotonic()
            if bluetooth and now - last_poll > INPUT_MAX_AGE:
                raise OSError('手柄采样超时，正在重新确认输入设备')
            if link is not None and not os.path.samestat(link_id, link.stat()):
                raise OSError('原蓝牙连接已更换')
            if changed:
                last_report = now
            last_poll = now
            axis_info = [_absinfo(fd, code) for code in self.axis_codes]
            if any(v is None for v in axis_info):
                raise OSError('手柄缺少所需四个摇杆轴')
            keys = _pressed(fd, tuple(buttons.values()))
            sample_time = now
            if bluetooth and link is None and keys[buttons['lb']]:
                fresh = last_report is not None and now - last_report <= INPUT_MAX_AGE
                if held_with_report and not fresh:
                    release_required = True
                if not fresh or release_required:
                    self.error = '蓝牙运动输入报告中断，请松开 LB 后重新操作。'
                    if not self.display_only:
                        with self.lock: self.frame = None
                        continue
                else:
                    held_with_report = True
                    self.error = ''
                    sample_time = last_report
            else:
                held_with_report = False
                release_required = False
                self.error = ''
            axes = tuple((-1. if invert else 1.)*_normalize(info[0], *info[1:3])
                         for info, invert in zip(axis_info, self.axis_inversions))
            hats = {}
            for code in (ABS_HAT0X, ABS_HAT0Y):
                hat = _absinfo(fd, code)
                if hat is None:
                    raise OSError('无法读取手柄方向键')
                hats[code] = hat[0]
            dpad = tuple(int(hats[code] == value) for code, value in self.dpad_codes)
            self.sequence += 1
            snapshot = {'guid': device['guid'], 'generation': self.generation,
                        'sequence': self.sequence, 'axes': axes, 'dpad': dpad,
                        'lb': keys[buttons['lb']], 'b': keys[buttons['b']],
                        'home': keys[buttons['home']],
                        'connected': True, 'link_verified': self.link_verified,
                        'mapping_verified': self.mapping_verified,
                        'source': 'evdev', 'device': device,
                        'received_at': sample_time}
            with self.lock: self.frame = snapshot

    def latest(self):
        with self.lock:
            if self.frame and time.monotonic()-self.frame['received_at'] <= INPUT_MAX_AGE:
                return dict(self.frame)
            return None
