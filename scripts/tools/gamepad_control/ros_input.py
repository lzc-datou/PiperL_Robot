"""ROS transport for kernel-read gamepad snapshots, with explicit fixture isolation."""
import os
import sys
import time
ROOT = os.environ.get('PIPERL_ROOT')
if ROOT and ROOT not in sys.path: sys.path.insert(0, ROOT)
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from piper_servo_tracking.msg import GamepadInput
from .input_adapter import EvdevInput
from .settings import load_input_profile


class InputNode(Node):
    def __init__(self):
        super().__init__('gamepad_input')
        self.declare_parameter('source', 'evdev')
        self.declare_parameter('device', '')
        self.declare_parameter('link_verified', False)
        self.declare_parameter('mapping_verified', False)
        self.source = self.get_parameter('source').value
        if self.source not in ('evdev', 'fixture', 'external'):
            raise ValueError('gamepad input source must be evdev, fixture, or external')
        self.device = self.get_parameter('device').value or None
        self.link_verified = bool(self.get_parameter('link_verified').value)
        self.mapping_verified = bool(self.get_parameter('mapping_verified').value)
        self.pub = self.create_publisher(GamepadInput, '/piper_servo/gamepad/input', 10)
        self.generation, self.sequence = 1, 0
        self.last_identity = None
        self.last_published_sequence = 0
        self.fixture_override = None
        self.fixture_received = 0.
        if self.source == 'fixture':
            self.create_subscription(GamepadInput, '/piper_servo/gamepad/fixture', self.on_fixture, 10)
        self.adapter = None
        if self.source == 'evdev':
            self.adapter = EvdevInput(load_input_profile(), self.link_verified, self.mapping_verified)
            self.adapter.start(self.device)
        self.create_timer(.02, self.tick)

    def publish(self, snapshot):
        msg = GamepadInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.guid = str(snapshot['guid'])
        msg.source = str(snapshot['source'])
        msg.device_name = str(snapshot.get('device',{}).get('name',snapshot.get('name','虚拟输入 fixture')))
        msg.connection_type = str(snapshot.get('device',{}).get('connection','fixture' if snapshot['source']=='fixture' else 'unknown'))
        msg.generation = int(snapshot['generation'])
        msg.sequence = int(snapshot['sequence'])
        msg.connected = bool(snapshot['connected'])
        msg.link_verified = bool(snapshot['link_verified'])
        msg.mapping_verified = bool(snapshot.get('mapping_verified',False))
        msg.axes = list(snapshot['axes'])
        msg.dpad = list(snapshot['dpad'])
        msg.lb, msg.b = bool(snapshot['lb']), bool(snapshot['b'])
        msg.home = bool(snapshot.get('home', False))
        self.pub.publish(msg)

    def on_fixture(self,msg):
        if self.source!='fixture': return
        if (len(msg.axes)!=4 or len(msg.dpad)!=4 or any(v not in (0,1) for v in msg.dpad)):
            return
        self.fixture_override={'axes':tuple(msg.axes),'dpad':tuple(msg.dpad),
            'lb':bool(msg.lb),'b':bool(msg.b),'home':bool(msg.home),
            'name':msg.device_name or '测试输入 fixture'}
        self.fixture_received=time.monotonic()

    def tick(self):
        if self.source == 'fixture':
            if self.fixture_override and time.monotonic()-self.fixture_received>.12:
                return
            self.sequence += 1
            override=self.fixture_override or {'axes':(0.,)*4,'dpad':(0,)*4,
                'lb':False,'b':False,'home':False,'name':'模拟中立 fixture'}
            self.publish({'guid':'sim-fixture', 'source':'fixture', 'generation':self.generation,
                          'sequence':self.sequence, 'connected':True, 'link_verified':False,
                          'mapping_verified':False,
                          'device':{'name':override['name'],'connection':'fixture'},
                          'axes':override['axes'],'dpad':override['dpad'],
                          'lb':override['lb'],'b':override['b'],'home':override['home']})
            return
        if self.source == 'external':
            return
        snapshot = self.adapter.latest() if self.adapter else None
        if snapshot:
            identity = (snapshot['guid'], snapshot['generation'])
            if identity != self.last_identity:
                self.last_identity = identity
                self.get_logger().info(f"发现输入设备 {snapshot['device']['name']} ({snapshot['device']['path']})")
            if snapshot['sequence'] > self.last_published_sequence:
                self.last_published_sequence = snapshot['sequence']
                self.publish(snapshot)

    def destroy_node(self):
        if self.adapter:
            self.adapter.stop()
        return super().destroy_node()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = InputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
