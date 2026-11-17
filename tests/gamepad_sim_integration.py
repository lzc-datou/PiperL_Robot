#!/usr/bin/env python3
"""Run the official mock/Servo/guard stack and exercise fixture-only gamepad I/O."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointJog
from trajectory_msgs.msg import JointTrajectory
from piper_servo_tracking.msg import GamepadInput
from scripts.task.common.process_cleanup import stop_process_group

DOMAIN=os.environ.get('ROS_DOMAIN_ID','43')


class Probe(Node):
    def __init__(self):
        super().__init__('gamepad_sim_integration_probe')
        self.fixture=self.create_publisher(GamepadInput,'/piper_servo/gamepad/fixture',10)
        self.ui=self.create_publisher(Float64,'/piper_servo/gamepad/ui_heartbeat',1)
        self.service_clients={name:self.create_client(Trigger,f'/piper_servo/gamepad/{name}') for name in ('start','stop','recover')}
        self.controller_list=self.create_client(ListControllers,'/piper_servo/controller_manager/list_controllers')
        self.backend={}; self.guard={}; self.jog=None; self.nonzero_jogs=0; self.trajectory_count=0
        self.last_input_time=0.; self.command={'axes':(0.,)*4,'dpad':(0,)*4,'lb':False,'b':False}
        self.last_fixture=time.monotonic(); self.last_ui=time.monotonic()
        self.publish_fixture=True
        self.create_subscription(String,'/piper_servo/gamepad/backend_status',self.on_backend,10)
        self.create_subscription(String,'/piper_servo/guard/status',self.on_guard,10)
        self.create_subscription(JointJog,'/piper_servo/servo_node/delta_joint_cmds',self.on_jog,10)
        # Observe Servo output before the guard; subscribing to the protected
        # JTC command topic would itself create a second actuator command sink.
        self.create_subscription(JointTrajectory,'/piper_servo/servo/trajectory',self.on_trajectory,10)
        self.create_timer(.02,self.publish_inputs)

    def on_backend(self,msg):
        try:self.backend=json.loads(msg.data)
        except ValueError:pass
    def on_guard(self,msg):
        try:self.guard=json.loads(msg.data)
        except ValueError:pass
    def on_jog(self,msg):
        self.jog=tuple(msg.velocities)
        if np.linalg.norm(self.jog)>.0001:self.nonzero_jogs+=1
    def on_trajectory(self,msg):
        if msg.points:self.trajectory_count+=1
    def publish_inputs(self):
        now=time.monotonic()
        if now-self.last_ui>=.04:
            self.ui.publish(Float64(data=time.time())); self.last_ui=now
        if self.publish_fixture:
            msg=GamepadInput(); msg.header.stamp=self.get_clock().now().to_msg()
            msg.guid='integration-fixture'; msg.source='fixture'; msg.device_name='集成 fixture'; msg.connection_type='fixture'
            msg.generation=1; msg.sequence=int(now*1000); msg.connected=True
            msg.link_verified=False; msg.mapping_verified=False
            msg.axes=list(self.command['axes']); msg.dpad=list(self.command['dpad'])
            msg.lb=self.command['lb']; msg.b=self.command['b']; self.fixture.publish(msg)
            self.last_fixture=now

    def set_input(self,axes=(0.,)*4,dpad=(0,)*4,lb=False,b=False):
        self.command={'axes':tuple(axes),'dpad':tuple(dpad),'lb':bool(lb),'b':bool(b)}
        self.last_fixture=time.monotonic()

    def pump(self,duration=.01):
        rclpy.spin_once(self,timeout_sec=0.)
        time.sleep(duration)

    def wait_for(self,predicate,timeout,description):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            self.pump(.01)
            if predicate():return
        raise RuntimeError(f'timeout waiting for {description}; backend={self.backend}; guard={self.guard}')

    def call(self,name,timeout=5.):
        client=self.service_clients[name]; deadline=time.monotonic()+timeout
        while time.monotonic()<deadline and not client.service_is_ready():self.pump(.02)
        if not client.service_is_ready():raise RuntimeError(f'service unavailable: {name}')
        future=client.call_async(Trigger.Request())
        while time.monotonic()<deadline and not future.done():self.pump(.01)
        if not future.done():raise RuntimeError(f'service timeout: {name}')
        response=future.result()
        if not response.success:raise RuntimeError(f'{name} rejected: {response.message}')
        return response.message


def main():
    if DOMAIN=='43':
        raise RuntimeError('请用 PIPERL_ROS_DOMAIN_ID=64 运行，避免与用户的默认 ROS domain 冲突')
    logs=ROOT/'outputs/gamepad_control'/'sim_integration'
    logs.mkdir(parents=True,exist_ok=True)
    stream=(logs/'stack.log').open('w',encoding='utf-8')
    command=[sys.executable,'/opt/ros/jazzy/bin/ros2','launch','piper_servo_tracking','task.launch.py',
        'mode:=sim','gamepad:=true','rviz:=false','gamepad_input_source:=fixture',
        f'gamepad_config:={ROOT}/scripts/tools/gamepad_control/config/config.yaml']
    rclpy.init(); probe=Probe()
    process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    summary={'domain':DOMAIN,'fixture_only':True,'directions':[]}
    try:
        probe.wait_for(lambda: bool(probe.backend.get('feedback')) and probe.guard.get('state')=='idle' and
                       probe.backend.get('servo_switched') is True,
                       30,'mock feedback, idle guard, and Servo JOINT_JOG mode')
        controller_deadline=time.monotonic()+30.
        while time.monotonic()<controller_deadline:
            if probe.controller_list.service_is_ready():
                future=probe.controller_list.call_async(ListControllers.Request())
                while time.monotonic()<controller_deadline and not future.done():probe.pump(.02)
                if future.done() and any(item.name=='arm_controller' and item.state=='active'
                                          for item in future.result().controller):break
            probe.pump(.1)
        else:raise RuntimeError('arm_controller activation timeout')
        probe.set_input(); probe.call('start')
        probe.wait_for(lambda:probe.backend.get('state')=='ARMED',3,'armed session')
        # The neutral dwell is measured from consecutive, newly received fixture frames.
        for _ in range(35):probe.pump(.01)
        directions=[
            ('+X',(0.,-.65,0.,0.),(0,0,0,0)),('-X',(0.,.65,0.,0.),(0,0,0,0)),
            ('+Y',(-.65,0.,0.,0.),(0,0,0,0)),('-Y',(.65,0.,0.,0.),(0,0,0,0)),
            ('+Z',(0.,0.,0.,-.65),(0,0,0,0)),('-Z',(0.,0.,0.,.65),(0,0,0,0)),
            ('+Rx',(0.,0.,0.,0.),(0,0,1,0)),('-Rx',(0.,0.,0.,0.),(0,0,0,1)),
            ('+Ry',(0.,0.,0.,0.),(1,0,0,0)),('-Ry',(0.,0.,0.,0.),(0,1,0,0)),
            ('+Rz',(0.,0.,-.65,0.),(0,0,0,0)),('-Rz',(0.,0.,.65,0.),(0,0,0,0)),
        ]
        probe.set_input(axes=directions[0][1],dpad=directions[0][2],lb=True)
        probe.wait_for(lambda:probe.backend.get('state')=='ACTIVE' and probe.guard.get('state')=='running',8,'guard running')
        for name,axes,dpad in directions:
            before=probe.nonzero_jogs
            probe.set_input(axes=axes,dpad=dpad,lb=True)
            probe.wait_for(lambda:probe.nonzero_jogs>before,.8,f'{name} nonzero DLS JointJog')
            summary['directions'].append(name)
            for _ in range(8):probe.pump(.01)
        summary['jog_nonzero_samples']=probe.nonzero_jogs
        summary['servo_trajectory_samples']=probe.trajectory_count
        if not probe.trajectory_count:raise RuntimeError('Servo produced no trajectory output')
        probe.set_input(); probe.wait_for(lambda:probe.backend.get('state')=='ARMED' and probe.guard.get('stop_confirmed') is True,4,'LB release stop confirmation')
        summary['lb_release_stop_confirmed']=True
        probe.set_input(b=True); probe.wait_for(lambda:probe.backend.get('state')=='LOCKED' and probe.guard.get('stop_confirmed') is True,4,'B lock and stop confirmation')
        summary['b_stop_locked']=True
        probe.set_input()
        probe.wait_for(lambda:probe.backend.get('input_b') is False,1,'B release reaches input relay')
        probe.call('recover')
        probe.wait_for(lambda:probe.backend.get('state')=='ARMED',2,'manual recovery to armed')
        summary['manual_recovery_requires_new_LB']=probe.backend.get('state')=='ARMED'
        for _ in range(35):probe.pump(.01)
        probe.wait_for(lambda:probe.backend.get('neutral_seen') is True,2,'neutral dwell after recovery')
        probe.set_input(axes=directions[0][1],dpad=directions[0][2],lb=True)
        probe.wait_for(lambda:probe.backend.get('state')=='ACTIVE' and probe.guard.get('state')=='running',8,
                       'new LB press after recovery')
        probe.publish_fixture=False
        probe.wait_for(lambda:probe.backend.get('state')=='LOCKED' and
                       probe.guard.get('stop_confirmed') is True,4,'fixture input loss stop confirmation')
        summary['input_dropout_lock_and_stop_confirmed']=True
        probe.publish_fixture=True; probe.set_input()
        probe.wait_for(lambda:probe.backend.get('input_b') is False and
                       probe.backend.get('input_lb') is False and
                       probe.backend.get('input_axes') == [0.,0.,0.,0.] and
                       probe.backend.get('input_dpad') == [0,0,0,0] and
                       probe.backend.get('input_age_s',1.) < .12 and
                       probe.backend.get('state')=='LOCKED',2,'fresh neutral input after dropout')
        probe.call('recover')
        probe.wait_for(lambda:probe.backend.get('state')=='ARMED',2,'recovery after input dropout')
        probe.call('stop')
        probe.wait_for(lambda:probe.guard.get('stop_confirmed') is True,4,'final measured stop confirmation')
        print(json.dumps(summary,ensure_ascii=False,indent=2))
    finally:
        probe.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        report=stop_process_group(process,timeouts=(6.,3.,2.))
        (logs/'cleanup.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        stream.close()
        if process.poll() is None: process.wait(timeout=2)


if __name__=='__main__':main()
