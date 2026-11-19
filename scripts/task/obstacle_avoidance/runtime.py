"""Isolated MoveIt + robot_state_publisher; no driver, controller or execution."""
import fcntl
import os
import time
from pathlib import Path
import yaml
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from moveit_msgs.msg import PlannerParams
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
from ament_index_python.packages import get_package_share_directory
from scripts.task.common.runtime import Runtime


class PreviewRuntime(Runtime):
    def acquire(self):
        path = Path('/tmp')/f'piper_obstacle_{os.getuid()}_{os.environ["ROS_DOMAIN_ID"]}.lock'
        self.lock = path.open('a')
        try: fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('该 ROS 域已有避障演示运行')

    def command(self, *args):
        return ['env', 'PIPERL_ROS_DOMAIN_ID='+os.environ['ROS_DOMAIN_ID'],
                'bash', 'scripts/piperl/run.sh', *args]

    def prepare(self, rviz=True):
        self.acquire()
        time.sleep(1.)
        names = self.names()
        if names - {self.client.node.get_name()}:
            raise RuntimeError('演示需要空闲 ROS 域，请退出该域节点或改用 --domain；不会接管已有链路')
        arguments = ['arm_type:=piper_l', 'effector_type:=none', 'follow:=false',
                     'control_topic:=/obstacle_avoidance/demo_joint_states']
        self.spawn('planner', self.command('ros2','launch','agx_arm_moveit','move_group.launch.py',
            *arguments, 'allow_trajectory_execution:=false',
            'disable_capabilities:=move_group/MoveGroupExecuteTrajectoryAction'))
        self.spawn('model', self.command('ros2','launch','agx_arm_moveit','rsp.launch.py', *arguments))
        self.client.connect_model(timeout=40)
        # Limit TOTG corner deviation: large rounding can cut into a box.
        # This affects only this isolated preview planner.
        setter = self.client.node.create_client(SetParametersAtomically, '/move_group/set_parameters_atomically')
        try:
            request = SetParametersAtomically.Request(parameters=[
                Parameter(name='ompl.totg.path_tolerance', value=ParameterValue(type=3,double_value=.0001)),
                Parameter(name='ompl.totg.resample_dt', value=ParameterValue(type=3,double_value=.05))])
            reply = self.client.call(setter,request,tick=self.client.tick)
            if not reply.result.successful:
                raise RuntimeError('Cannot configure planning path tolerance: '+reply.result.reason)
        finally:
            self.client.node.destroy_client(setter)
        # Configure OMPL's motion-edge checking as well as the postprocessor.
        setter = self.client.node.create_client(SetPlannerParams, '/set_planner_params')
        getter = self.client.node.create_client(GetPlannerParams, '/get_planner_params')
        try:
            self.client.call(setter,SetPlannerParams.Request(pipeline_id='ompl',
                group=self.client.settings['group'],planner_config=self.client.planner_id,replace=True,
                params=PlannerParams(keys=['type','range','longest_valid_segment_fraction'],
                    values=['geometric::RRTConnect','0.0','0.0001'])),tick=self.client.tick)
            configured = self.client.call(getter,GetPlannerParams.Request(pipeline_id='ompl',
                group=self.client.settings['group'],planner_config=self.client.planner_id),tick=self.client.tick).params
            if dict(zip(configured.keys,configured.values)).get('longest_valid_segment_fraction') != '0.0001':
                raise RuntimeError('Cannot confirm OMPL collision checking resolution')
        finally:
            self.client.node.destroy_client(setter)
            self.client.node.destroy_client(getter)
        values = self.client.call(self.client.parameters,
            GetParameters.Request(names=['allow_trajectory_execution'])).values
        if len(values) != 1 or values[0].type != 1 or values[0].bool_value:
            raise RuntimeError('无法确认 MoveIt 执行功能已关闭')
        if rviz:
            self.start_rviz()
            self.wait_for_display()

    def start_rviz(self):
        # Parameter mirroring stays shared with the real-arm preview runtime.
        from rcl_interfaces.srv import ListParameters
        service = self.client.node.create_client(ListParameters, '/move_group/list_parameters')
        try: names = self.client.call(service, ListParameters.Request()).result.names
        finally: self.client.node.destroy_client(service)
        params = self.read_rviz_parameters([n for n in names if n.startswith(('robot_description','ompl','planning_pipelines','default_planning_pipeline'))])
        params.update(robot_description=self.client.urdf, robot_description_semantic=self.client.srdf)
        parameter_file = self.logdir/'rviz_parameters.yaml'
        parameter_file.write_text(yaml.safe_dump({'/**':{'ros__parameters':params}}))
        upstream = Path(get_package_share_directory('agx_arm_moveit'))/'config/moveit.rviz'
        config = yaml.safe_load(upstream.read_text())
        config['Panels'] = [dict(Class='rviz_common/Displays', Name='Displays')]
        config['Window Geometry'] = dict(Width=1100, Height=680)
        entries = [dict(Class='rviz_default_plugins/Grid', Name='Grid', Value=True)]
        entries.append({'Class':'moveit_rviz_plugin/PlanningScene', 'Name':'Virtual scene (solid robot = first point)',
            'Value':True, 'Enabled':True, 'Robot Description':'robot_description',
            'Planning Scene Topic':'/monitored_planning_scene',
            'Scene Geometry':{'Scene Alpha':.65, 'Scene Color':'255; 120; 50'},
            'Scene Robot':{'Robot Alpha':1.,'Show Robot Visual':True,'Show Robot Collision':False}})
        entries.append({'Class':'moveit_rviz_plugin/Trajectory','Name':'Comparison preview', 'Value':True,
            'Enabled':True, 'Robot Description':'robot_description',
            'Trajectory Topic':'/obstacle_avoidance/display_planned_path', 'Robot Alpha':.35,
            'Color Enabled':False, 'Loop Animation':True, 'Interrupt Display':True, 'State Display Time':'1x'})
        entries.append({'Class':'rviz_default_plugins/MarkerArray','Name':'TCP path and waypoints', 'Value':True,
            'Enabled':True, 'Topic':{'Value':'/obstacle_avoidance/markers','Reliability Policy':'Reliable',
            'Durability Policy':'Transient Local','History Policy':'Keep Last','Depth':1}})
        config['Visualization Manager']['Displays'] = [{'Class':'piper_multi_point_rviz/TimeCompatibleGroup',
            'Name':'Displays (time compatible)', 'Value':True, 'Displays':entries,
            'Fit Window On Startup':True}]
        config['Visualization Manager']['Global Options']['Fixed Frame'] = self.client.settings['frame_id']
        config['Visualization Manager']['Views']['Current'].update(
            {'Distance':1.65,'Focal Point':{'X':.20,'Y':0.,'Z':.40},'Yaw':.8,'Pitch':.45})
        self.configure_rviz(config)
        display_file = self.logdir/'scene.rviz'
        display_file.write_text(yaml.safe_dump(config,sort_keys=False))
        self.rviz_process = self.spawn('rviz', self.command('ros2','run','rviz2','rviz2',
            '-d',str(display_file),'--ros-args','--params-file',str(parameter_file),
            '-r','joint_states:=/obstacle_avoidance/demo_joint_states'))

    def configure_rviz(self, config):
        """Allow a preview scene to add displays without changing process ownership."""
