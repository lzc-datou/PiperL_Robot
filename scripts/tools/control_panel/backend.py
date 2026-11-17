"""Send-only CAN control with interface discovery and cancellable queued work."""
import queue
import threading

from .core import (CanTransport, CanFeedback, activate_interface, inspect_interface,
                   list_interfaces, ready, home_frames, COMMANDS)
from .recovery import prepare_motion


class Backend:
    def __init__(self, channel=None, *, demo=False, home_speed=10, discover=True):
        home_frames(home_speed)  # Validate before starting any worker.
        self.channel, self.demo, self.home_speed = channel, demo, home_speed
        self.transport = None
        self.interface_index = None
        self.link_revision = 0
        self.connected = self.busy = False
        self.epoch = 0
        self.busy_epoch = None
        # A send and a port switch are atomic with respect to each other.
        self.lock = threading.RLock()
        self.shutdown = threading.Event()
        self.cancel = threading.Event()
        self.wake_scan = threading.Event()
        self.events, self.jobs = queue.Queue(), queue.Queue()
        self.interfaces = {}
        self.scanned = False
        self.scan_error = ''
        self.auto_choice = channel is None
        self.threads = [threading.Thread(target=self.work, daemon=True)]
        if discover:
            self.threads.append(threading.Thread(target=self.scan_loop, daemon=True))
        for thread in self.threads:
            thread.start()

    def note(self, text):
        self.events.put(text)

    def invalidate(self):
        self.epoch += 1
        self.cancel.set()
        self.cancel = threading.Event()

    def detach(self):
        self.link_revision += 1
        self.interface_index = None
        self.connected = False
        if self.transport:
            self.transport.close()
            self.transport = None

    def select(self, channel):
        with self.lock:
            if channel == self.channel:
                return
            self.invalidate()
            self.detach()
            self.channel = channel
            self.scanned = False
            self.auto_choice = False
            self.busy = False
            self.busy_epoch = None
            self.note(f'已选择 {channel}，正在检查端口' if channel else '请选择 CAN 端口')
        self.wake_scan.set()

    def refresh_interfaces(self, interfaces, scan_revision=None):
        with self.lock:
            if self.shutdown.is_set():
                return
            # Ignore a scan started before a manual connection / port switch.
            if scan_revision is not None and scan_revision != self.link_revision:
                return
            changed = interfaces != self.interfaces or not self.scanned
            old_status = self.interfaces.get(self.channel)
            first_scan = not self.scanned
            self.interfaces, self.scanned, self.scan_error = interfaces, True, ''
            if self.auto_choice and interfaces:
                if len(interfaces) == 1:
                    self.channel = next(iter(interfaces))
                self.auto_choice = False
            if not self.channel:
                if changed:
                    self.note('请选择 CAN 端口' if interfaces else '未检测到 CAN 端口')
                return
            status = interfaces.get(self.channel)
            selected_changed = first_scan or status != old_status
            index = status[3] if status and len(status) > 3 else None
            if (self.connected and index is not None and self.interface_index is not None
                    and index != self.interface_index):
                self.invalidate()
                self.detach()
            if status is None or not ready(status):
                if self.connected:
                    self.invalidate()
                    self.detach()
                    selected_changed = True
                if selected_changed:
                    if status is None:
                        message = f'{self.channel} 不存在，请选择端口'
                    elif not status[0]:
                        message = f'{self.channel} 未启用，请点击 CAN 连接'
                    else:
                        message = f'{self.channel} 状态异常：{status[2]} / {status[1]} bps'
                    self.note(message)
                return
            if not self.connected:
                self.attach()
            self.interface_index = index

    def attach(self):
        # Caller holds lock. Merely binding a send-only socket emits no CAN frame.
        transport = None if self.demo else CanTransport(self.channel)
        self.transport, self.connected = transport, True
        self.link_revision += 1
        status = self.interfaces.get(self.channel)
        self.interface_index = status[3] if status and len(status) > 3 else None
        self.note(('演示 · ' if self.demo else '')+f'{self.channel} 已连接 · 1 Mbps')

    def scan_loop(self):
        while not self.shutdown.is_set():
            self.wake_scan.clear()
            with self.lock:
                scan_revision = self.link_revision
            try:
                interfaces = ({'can0': (True, 1000000, 'ERROR-ACTIVE'),
                               'can1': (False, 1000000, 'STOPPED')}
                              if self.demo else list_interfaces())
                # Preserve simulated manually activated ports between scans.
                if self.demo:
                    with self.lock:
                        if self.connected and self.channel in interfaces:
                            interfaces[self.channel] = (True, 1000000, 'ERROR-ACTIVE')
                self.refresh_interfaces(interfaces, scan_revision)
            except Exception as exc:
                with self.lock:
                    if self.shutdown.is_set() or scan_revision != self.link_revision:
                        continue
                    if self.connected:
                        self.invalidate()
                        self.detach()
                    message = str(exc)
                    if message != self.scan_error:
                        self.note('端口检查失败：'+message)
                    self.scan_error = message
                    self.scanned = True
            self.wake_scan.wait(1)

    def submit(self, action):
        if action not in ('connect', 'enable', 'reset', 'home'):
            raise ValueError('未知操作')
        with self.lock:
            if self.busy or self.shutdown.is_set():
                return
            if not self.channel:
                self.note('请先选择 CAN 端口')
                return
            self.busy = True
            self.busy_epoch = self.epoch
            self.jobs.put((action, self.epoch))

    def check_epoch(self, epoch):
        if self.shutdown.is_set() or epoch != self.epoch:
            raise InterruptedError('待发送操作已取消')

    def send_frame(self, identifier, payload, epoch):
        with self.lock:
            self.check_epoch(epoch)
            if not self.connected:
                raise RuntimeError('请先连接 CAN')
            if not self.demo:
                self.transport.send_frame(identifier, payload)

    def send(self, command, epoch):
        identifier, payload = COMMANDS[command]
        with self.lock:
            self.send_frame(identifier, payload, epoch)
            self.note(('演示 · ' if self.demo else '')+f'已发送 {identifier:03X}#{payload.hex().upper()}')

    def emergency_stop(self):
        with self.lock:
            if self.shutdown.is_set() or not self.connected:
                return
            self.invalidate()
            epoch = self.epoch
            self.threads = [t for t in self.threads if t.is_alive()]
            thread = threading.Thread(target=self._stop, args=(epoch,), daemon=True)
            self.threads.append(thread)
            thread.start()

    def _stop(self, epoch):
        try:
            self.send('stop', epoch)
        except InterruptedError:
            pass
        except Exception as exc:
            self.note('急停发送失败：'+str(exc))

    def prepare(self, channel, epoch, *, enable):
        if self.demo:
            self.send_frame(*home_frames(self.home_speed)[0], epoch)
            if enable:
                self.send('enable', epoch)
            return
        self.check_epoch(epoch)
        reader = CanFeedback(channel)
        def report(message):
            with self.lock:
                self.check_epoch(epoch)
                self.note(message)
        try:
            prepare_motion(reader, lambda identifier, payload: self.send_frame(identifier, payload, epoch),
                           lambda: self.check_epoch(epoch), speed=self.home_speed,
                           enable=enable, note=report)
        finally:
            reader.close()

    def operate(self, action, epoch):
        with self.lock:
            self.check_epoch(epoch)
            channel, cancel = self.channel, self.cancel
        if action == 'connect':
            if not self.demo:
                activate_interface(channel, cancel)
                if not ready(inspect_interface(channel)):
                    raise RuntimeError('CAN 状态异常，未接入')
            with self.lock:
                self.check_epoch(epoch)
                if not self.connected:
                    self.attach()
            self.wake_scan.set()
        elif action == 'reset':
            self.send('reset', epoch)
            # Allow the controller to leave emergency stop before accepting the
            # next click. This wait is cancellable, not a confirmation of reset.
            cancel.wait(1.0)
            with self.lock:
                self.check_epoch(epoch)
                self.note(('演示 · ' if self.demo else '')+'重置指令已发送，请再使能')
        elif action == 'enable':
            self.prepare(channel, epoch, enable=True)
            with self.lock:
                self.check_epoch(epoch)
                self.note(('演示 · ' if self.demo else '')+'CAN 运动模式和六轴使能已'+('模拟确认' if self.demo else '确认'))
        elif action == 'home':
            self.prepare(channel, epoch, enable=False)
            for identifier, payload in home_frames(self.home_speed)[1:]:
                # Release lock between frames so stop/switch can cancel the rest.
                self.send_frame(identifier, payload, epoch)
            with self.lock:
                self.check_epoch(epoch)
                self.note(('演示 · ' if self.demo else '')+'回零指令已发送')
        else:
            self.send(action, epoch)

    def work(self):
        while not self.shutdown.is_set():
            try:
                action, epoch = self.jobs.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.operate(action, epoch)
            except InterruptedError:
                pass
            except Exception as exc:
                with self.lock:
                    if epoch == self.epoch and not self.shutdown.is_set():
                        self.note(str(exc))
            finally:
                with self.lock:
                    if self.busy_epoch == epoch:
                        self.busy = False
                        self.busy_epoch = None

    def snapshot(self):
        with self.lock:
            status = self.interfaces.get(self.channel)
            return dict(connected=self.connected, busy=self.busy, channel=self.channel,
                        interfaces=dict(self.interfaces), scanned=self.scanned,
                        can_connect=bool(status and not status[0] and not self.scan_error))

    def close(self):
        with self.lock:
            self.shutdown.set()
            self.invalidate()
            self.detach()
        self.wake_scan.set()
        for thread in self.threads:
            thread.join(timeout=0.3)
