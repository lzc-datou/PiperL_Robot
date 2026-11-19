"""Own only processes started by this task; never start a second driver."""
import fcntl
import os
from pathlib import Path
import select
import sys
import subprocess
import time
import yaml
from rcl_interfaces.srv import GetParameters, ListParameters
from ament_index_python.packages import get_package_share_directory
from .process_cleanup import stop_process_group

class Runtime:
    def __init__(self, client, root, logdir):
        self.client, self.root, self.logdir = client, Path(root), Path(logdir)
        self.processes=[]
        self.streams=[]
        self.lock=None
        self.rviz_process=None
        self.external_rviz=False
        self.session_panel=False

    def acquire(self):
        # Same user/domain tasks share one lease even when started from different directories.
        path=Path('/tmp')/f'piper_multi_point_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID","0")}.lock'
        self.lock=path.open('a')
        try: fcntl.flock(self.lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Another multi-point task is already running in this ROS domain')

    def spawn(self,name,command):
        stream=(self.logdir/(name+'.log')).open('w')
        self.streams.append(stream)
        process=subprocess.Popen(command,cwd=self.root,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append(process)
        return process

    def names(self):
        return {name for name,_ in self.client.node.get_node_names_and_namespaces()}

    def prepare(self,mode,rviz):
        self.acquire()
        time.sleep(1.0) # Bounded ROS discovery before deciding whether to launch.
        names=self.names()
        if 'control_gate' in names:
            raise RuntimeError('Automatic control_gate conflicts with task stop/recovery; restart the official stack with auto_control_gate:=false')
        existing=bool({'move_group','controller_manager','agx_arm_ctrl_single_node'} & names)
        if mode=='start' and existing:
            raise RuntimeError('Robot stack already exists; use --stack attach/auto')
        if not existing and mode in ('auto','start'):
            self.spawn('official_stack',['bash','scripts/piperl/start_moveit.sh','auto_enable:=false','use_rviz:=false'])
        elif mode=='auto' and 'agx_arm_ctrl_single_node' in names and not ({'move_group','controller_manager'} & names):
            self.start_planning_for_existing_driver()
        elif mode=='attach' or existing:
            print('Using existing official stack; no driver will be launched.',flush=True)
        self.client.connect(timeout=40)
        names=self.names()
        if 'control_gate' in names:
            raise RuntimeError('Automatic control_gate conflicts with task stop/recovery; use auto_control_gate:=false')
        has_rviz=any('rviz' in name.lower() for name in names)
        if rviz:
            if has_rviz:
                self.external_rviz=True
                print('Using existing RViz; its window and settings remain unchanged.',flush=True)
            else:
                self.start_rviz()
            self.wait_for_display()
            print('RViz: Multi-point preview uses /multi_point/display_planned_path; official MotionPlanning stays independent.',flush=True)
            print('Optional labels: add MarkerArray /multi_point/targets, Durability = Transient Local.',flush=True)
        elif has_rviz:
            print('Existing RViz belongs to the operator and will remain open.',flush=True)

    def start_planning_for_existing_driver(self):
        """Start only official MoveIt, with the existing driver's actual model parameters."""
        service=self.client.node.create_client(GetParameters,'/agx_arm_ctrl_single_node/get_parameters')
        names=['arm_type','effector_type','revo2_type','tcp_offset']
        try:
            values=self.client.call(service,GetParameters.Request(names=names)).values
        finally:
            self.client.node.destroy_client(service)
        if len(values)!=4 or any(value.type!=4 for value in values[:3]) or values[3].type!=8:
            raise RuntimeError('Cannot read existing driver model/TCP parameters; no duplicate driver was started')
        parameters=[value.string_value for value in values[:3]]+[str(list(values[3].double_array_value))]
        arguments=[f'{key}:={value}' for key,value in zip(names,parameters)]
        print('Existing driver found; starting official MoveIt only.',flush=True)
        self.spawn('official_planning',['bash','scripts/piperl/run.sh','ros2','launch','agx_arm_moveit','demo.launch.py',
                   *arguments,'follow:=true','use_rviz:=false','auto_control_gate:=false'])

    def read_rviz_parameters(self,names):
        """An unset rclcpp parameter can empty an entire GetParameters response."""
        attrs={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',5:'byte_array_value',
               6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
        params={}
        for offset in range(0,len(names),32):
            batch=names[offset:offset+32]
            values=self.client.call(self.client.parameters,GetParameters.Request(names=batch)).values
            if len(values)!=len(batch):
                values=[]
                for name in batch:
                    single=self.client.call(self.client.parameters,GetParameters.Request(names=[name])).values
                    values.append(single[0] if single else None)
            for name,value in zip(batch,values):
                if value is None or value.type not in attrs: continue
                item=getattr(value,attrs[value.type])
                # ROS parameter YAML cannot infer the element type of []. Let the
                # plugin declare its typed empty default instead of aborting RViz.
                if value.type>=5 and not item: continue
                params[name]=list(item) if value.type>=5 else item
        return params

    def start_rviz(self):
        # Mirror the running model and settings rather than reconstructing a different TCP/model.
        service=self.client.node.create_client(ListParameters,'/move_group/list_parameters')
        request=ListParameters.Request()
        try:
            request.depth=0
            available=self.client.call(service,request).result.names
        finally:
            self.client.node.destroy_client(service)
        # Keep every configured pipeline and its default; RViz otherwise selects
        # the first returned planner (which can be Pilz with an empty planner ID).
        pipeline_names=self.client.call(self.client.parameters,
            GetParameters.Request(names=['planning_pipelines'])).values
        pipelines=list(pipeline_names[0].string_array_value) if pipeline_names and pipeline_names[0].type==9 else []
        prefixes=('robot_description','robot_description_semantic','robot_description_kinematics',
                  'robot_description_planning','planning_pipelines','default_planning_pipeline',*pipelines)
        names=[name for name in available if any(name==p or name.startswith(p+'.') for p in prefixes)]
        params=self.read_rviz_parameters(names)
        if not params.get('default_planning_pipeline') or not params.get('planning_pipelines'):
            raise RuntimeError('Live MoveIt planning pipeline parameters unavailable; RViz was not started with incomplete settings')
        params['robot_description']=self.client.urdf
        params['robot_description_semantic']=self.client.srdf
        if not any(name=='robot_description_kinematics' or name.startswith('robot_description_kinematics.') for name in params):
            # Some move_group builds do not expose undeclared solver parameters.
            # Use the unchanged official solver config so RViz single-point IK works.
            kinematics=Path(get_package_share_directory('agx_arm_moveit'))/'config/kinematics.yaml'
            params['robot_description_kinematics']=yaml.safe_load(kinematics.read_text())
        config=self.logdir/'rviz_parameters.yaml'
        config.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}},sort_keys=False))
        upstream=Path(get_package_share_directory('agx_arm_moveit'))/'config/moveit.rviz'
        display=yaml.safe_load(upstream.read_text())
        # Saved desktop geometry/dock state belongs to the upstream author's
        # screen. Start owned windows without replaying those size constraints.
        display['Window Geometry']={'Width':1100,'Height':680}
        for item in display['Visualization Manager']['Displays']:
            if item.get('Class')=='moveit_rviz_plugin/MotionPlanning':
                item.setdefault('Scene Robot',{}).update({'Robot Alpha':1.0,'Show Robot Visual':True,'Show Robot Collision':False})
                item.setdefault('Planned Path',{}).update({'Robot Alpha':0.35,'Color Enabled':False,'Loop Animation':False})
                # Keep the official colored query robot visible while dragging.
                item.setdefault('Planning Request',{}).update({
                    'Start State Alpha':0.45,'Goal State Alpha':0.45,'Query Goal State':True})
        display['Visualization Manager']['Displays'].append({
            'Class':'moveit_rviz_plugin/Trajectory','Name':'Multi-point preview','Enabled':True,'Value':True,
            'Robot Description':'robot_description','Trajectory Topic':'/multi_point/display_planned_path',
            'Loop Animation':True,'Interrupt Display':True,'State Display Time':'1x',
            'Color Enabled':False,'Robot Alpha':0.35})
        display['Visualization Manager']['Displays'].append({
            'Class':'rviz_default_plugins/MarkerArray','Name':'Task targets','Enabled':True,'Value':True,
            'Topic':{'Value':'/multi_point/targets','Reliability Policy':'Reliable','Durability Policy':'Transient Local','History Policy':'Keep Last','Depth':1}})
        if self.session_panel:
            display.setdefault('Panels',[]).append({'Class':'piper_multi_point_rviz/TaskPanel','Name':'多点任务',
                                                    'Default Left Layout':True})
        # Also fix preview timing in non-interactive sessions without the panel.
        entries=display['Visualization Manager']['Displays']
        display['Visualization Manager']['Displays']=[{
            'Class':'piper_multi_point_rviz/TimeCompatibleGroup',
            'Name':'Displays (time compatible)','Value':True,'Displays':entries,
            'Fit Window On Startup':True}]
        rviz_config=self.logdir/'task.rviz'
        rviz_config.write_text(yaml.safe_dump(display,sort_keys=False))
        self.rviz_process=self.spawn('rviz',['bash','scripts/piperl/run.sh','ros2','run','rviz2','rviz2','-d',str(rviz_config),
                          '--ros-args','--params-file',str(config),'-r','joint_states:=/feedback/joint_states'])
        time.sleep(0.5)
        if self.processes[-1].poll() is not None: raise RuntimeError('RViz exited; see rviz.log')

    def wait_for_display(self,timeout=15):
        """Wait for the trajectory subscriber, not just an RViz process name."""
        print('Waiting for RViz trajectory display...',flush=True)
        deadline=time.monotonic()+timeout
        ready_since=None
        while time.monotonic()<deadline:
            if self.rviz_process is not None and self.rviz_process.poll() is not None:
                raise RuntimeError('RViz exited before its trajectory display was ready; see rviz.log')
            if self.client.cancel_requested.is_set():
                raise InterruptedError('Preview startup cancelled')
            if self.client.display.get_subscription_count()>0:
                if ready_since is None: ready_since=time.monotonic()
                # Let the display finish loading its model after creating its subscriber.
                if time.monotonic()-ready_since>=2.0: return
            else:
                ready_since=None
            time.sleep(0.1)
        raise RuntimeError('No ready /multi_point/display_planned_path subscriber. Add the multi-point panel or a Trajectory display '
                           'in RViz with Trajectory Topic /multi_point/display_planned_path, then retry.')

    @staticmethod
    def preview_key():
        if not sys.stdin.isatty(): return None
        try:
            if select.select([sys.stdin],[],[],0)[0]:
                line=sys.stdin.readline()
                return line.strip().lower() if line else None
        except (OSError,ValueError):
            pass
        return None

    def wait_for_rviz(self,on_execute=None):
        """Keep display publishers alive for both owned and external RViz."""
        if self.rviz_process is None and not self.external_rviz:
            return
        print('Preview ready: Enter = replay only; q + Enter / Ctrl+C = exit.',flush=True)
        if self.external_rviz:
            print('External RViz will remain open when this task exits.',flush=True)
        execute_available=on_execute is not None
        if execute_available:
            print('Interactive mode: e + Enter executes the reviewed trajectory ONCE on the REAL ARM. Confirm clearance and enable manually first.',flush=True)
        try:
            while self.rviz_process is None or self.rviz_process.poll() is None:
                if self.client.cancel_requested.wait(0.2): return
                key=self.preview_key()
                if key=='q': return
                if key=='e':
                    if execute_available:
                        execute_available=False  # A failed attempt also requires a new preview.
                        try:
                            on_execute()
                        except KeyboardInterrupt as exc:
                            raise InterruptedError('Interactive execution interrupted') from exc
                        print('Execution complete. Enter replays the preview only; another execution requires a new task.',flush=True)
                    else:
                        print('Execution unavailable: use --interactive and a fresh preview.',flush=True)
                if key=='':
                    self.client.replay_preview()
                    print('Preview replayed (display only).',flush=True)
        except KeyboardInterrupt:
            return  # Viewing ended; a completed task remains successful.
        if self.rviz_process.returncode:
            raise RuntimeError(f'RViz exited with code {self.rviz_process.returncode}; see rviz.log')

    def close(self):
        exits=[]
        try:
            if self.processes:
                print('正在关闭本次启动的 RViz 和官方链路，请等待进程清理完成。',flush=True)
            for process in reversed(self.processes):
                try:
                    exits.append(stop_process_group(process))
                except Exception as exc:
                    exits.append(dict(pid=process.pid,errors=[str(exc)]))
            if exits:
                import json
                (self.logdir/'process_exit.json').write_text(json.dumps(exits,indent=2)+'\n')
            if any(item.get('remaining_pids') or item.get('errors') for item in exits):
                raise RuntimeError(f'进程清理未完成，请查看 {self.logdir / "process_exit.json"}')
            if exits:
                print('本次启动的进程已全部退出。',flush=True)
        finally:
            self.processes.clear()
            for stream in self.streams: stream.close()
            self.streams.clear()
            if self.lock:
                fcntl.flock(self.lock,fcntl.LOCK_UN)
                self.lock.close()
                self.lock=None
