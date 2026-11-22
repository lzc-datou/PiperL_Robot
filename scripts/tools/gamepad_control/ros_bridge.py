"""Nonblocking Qt-thread ROS bridge for gamepad controls and status snapshots."""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool, Empty
from piper_servo_tracking.msg import GamepadInput
from piper_servo_tracking.srv import SelectGamepadControl
from .core import ControlSelection


class GamepadBridge(Node):
    SERVICES={'start','end','stop','recover'}
    def __init__(self):
        super().__init__('gamepad_ui')
        self.pub_ui=self.create_publisher(Float64,'/piper_servo/gamepad/ui_heartbeat',1)
        self.pub_speed=self.create_publisher(Float64,'/piper_servo/gamepad/speed_scale',1)
        self.request_clients={name:self.create_client(Trigger,f'/piper_servo/gamepad/{name}') for name in self.SERVICES}
        self.selection_client=self.create_client(SelectGamepadControl,'/piper_servo/gamepad/select_control')
        self.real_gate=self.create_client(SetBool,'/piper_servo/control_enable')
        self.real_hold=self.create_client(Empty,'/piper_servo/emergency_stop')
        self.status={}; self.guard={}; self.input=None; self.last_reply=''; self.pending={}; self.guard_received=0.
        self.create_subscription(String,'/piper_servo/gamepad/backend_status',self.on_status,10)
        self.create_subscription(String,'/piper_servo/guard/status',self.on_guard,10)
        self.create_subscription(GamepadInput,'/piper_servo/gamepad/input',self.on_input,10)

    def on_status(self,msg):
        try: self.status=json.loads(msg.data)
        except (ValueError,TypeError): pass
    def on_guard(self,msg):
        try: self.guard=json.loads(msg.data)
        except (ValueError,TypeError): pass
        self.guard_received=time.monotonic()
    def on_input(self,msg):
        self.input={'guid':msg.guid,'generation':msg.generation,'sequence':msg.sequence,
            'source':msg.source,'device_name':msg.device_name,'connection_type':msg.connection_type,
            'connected':msg.connected,'link_verified':msg.link_verified,
            'mapping_verified':msg.mapping_verified,
            'axes':tuple(msg.axes),'dpad':tuple(msg.dpad),'lb':msg.lb,'b':msg.b,'home':msg.home,
            'received_at':time.monotonic()}

    def heartbeat(self):
        self.pub_ui.publish(Float64(data=time.time()))
    def set_speed(self,value):
        self.pub_speed.publish(Float64(data=float(value)))
    def real_stop_fallback(self):
        """Only official driver services; called when the shared guard disappears in real mode."""
        requested=False
        if self.real_gate.service_is_ready():
            self.real_gate.call_async(SetBool.Request(data=False)); requested=True
        if self.real_hold.service_is_ready():
            self.real_hold.call_async(Empty.Request()); requested=True
        return requested
    def request(self,name,callback=None):
        if name not in self.SERVICES: raise ValueError('unknown gamepad request')
        client=self.request_clients[name]
        if not client.service_is_ready():
            self.last_reply='专用手柄后端尚未就绪'
            if callback: callback(False,self.last_reply)
            return None
        future=client.call_async(Trigger.Request())
        self.pending[name]=future
        def completed(f):
            try:
                result=f.result(); self.last_reply=result.message
                success=bool(result.success)
            except Exception as exc:
                self.last_reply=f'请求失败：{exc}'; success=False
            self.pending.pop(name,None)
            if callback: callback(success,self.last_reply)
        future.add_done_callback(completed)
        return future

    def select_control(self,control_mode,joint_index,callback=None):
        ControlSelection.validate_choice(control_mode, joint_index)
        if not self.selection_client.service_is_ready():
            self.last_reply='控制选择服务尚未就绪'
            if callback: callback(False,self.last_reply)
            return None
        if 'select_control' in self.pending:
            self.last_reply='控制选择请求正在处理中'
            if callback: callback(False,self.last_reply)
            return None
        future=self.selection_client.call_async(SelectGamepadControl.Request(
            control_mode=control_mode,joint_index=joint_index))
        self.pending['select_control']=future
        def completed(result_future):
            try:
                result=result_future.result(); self.last_reply=result.message
                success=bool(result.success)
            except Exception as exc:
                self.last_reply=f'选择请求失败：{exc}'; success=False
            self.pending.pop('select_control',None)
            if callback: callback(success,self.last_reply)
        future.add_done_callback(completed)
        return future

    def spin(self):
        # Drain the input/status burst without letting ROS monopolize Qt.
        deadline=time.monotonic()+.004
        for _ in range(16):
            if time.monotonic()>=deadline:
                break
            rclpy.spin_once(self,timeout_sec=0.)


def init_bridge():
    if not rclpy.ok(): rclpy.init()
    return GamepadBridge()
