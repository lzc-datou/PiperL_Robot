"""Thin client of official MoveIt. No resampling, CAN writes or automatic enable."""
import math
import threading
import time
import xml.etree.ElementTree as ET
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, DisplayTrajectory
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import SetBool
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from .schema import JOINTS
from .pose import pose_errors
from .robot_feedback import Feedback, StableWindow, FeedbackUnavailableError
from .ros_context import RosContext, robot_state, model_hash
from .planning_scene import PlanningScene
from .joint_bounds import JointBoundsError


class PlanningError(RuntimeError):
    """Keep the planner's diagnostic fields, including generic FAILURE responses."""
    def __init__(self, error_code):
        code = error_code.val
        name = next((key for key in dir(MoveItErrorCodes)
                     if key.isupper() and getattr(MoveItErrorCodes, key) == code), 'UNKNOWN')
        self.details = dict(code=code, name=name, message=getattr(error_code, 'message', ''),
                            source=getattr(error_code, 'source', ''))
        detail = '; '.join(f'{key}={self.details[key]}' for key in ('source', 'message') if self.details[key])
        super().__init__(f'MoveIt planning failed: code={code} ({name})' + (f'; {detail}' if detail else ''))


class MoveItClient(RosContext):
    def __init__(self, settings):
        super().__init__()
        self.settings = settings
        self.feedback = Feedback()
        self.command_feedback = Feedback()
        self.task_gate_closed = False
        self.cancel_requested = threading.Event()
        self.active_goal = None
        self.pending_goal = None
        self.motion_submitted = False
        self.external_status = []
        self.activity_generation = 0
        self.active_status_ids = set()
        self.execution_result_future = None
        self.node.create_subscription(JointState,'/feedback/joint_states',self.feedback.joint_callback,20)
        self.node.create_subscription(JointState,'/control/joint_states',self.command_feedback.joint_callback,20)
        for kind,topic in [('motor','/feedback/motor_status'),('arm','/feedback/arm_status')]:
            self.node.create_subscription(DiagnosticArray,topic,lambda msg,k=kind:self.feedback.diagnostic_callback(k,msg),10)
        # Receive the controller's last status even when attaching during a goal.
        status_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(GoalStatusArray,'/arm_controller/follow_joint_trajectory/_action/status',self._status,status_qos)
        self.planning = self.node.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.validity = self.node.create_client(GetStateValidity,'/check_state_validity')
        self.execution = ActionClient(self.node,ExecuteTrajectory,'/execute_trajectory')
        self.gate = self.node.create_client(SetBool,'/control_enable')
        qos = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.display = self.node.create_publisher(DisplayTrajectory,'/multi_point/display_planned_path',qos)
        self.targets = self.node.create_publisher(MarkerArray,'/multi_point/targets',qos)
        self.scene = PlanningScene(self)
        self.limits = {}
        self.preview_message = None
        self.target_message = None

    def _status(self, msg):
        self.external_status = msg.status_list
        active={bytes(item.goal_info.goal_id.uuid) for item in msg.status_list if item.status in (1,2,3)}
        if active-self.active_status_ids:
            self.activity_generation += 1
        self.active_status_ids = active

    def connect(self, timeout=30):
        self.connect_model(timeout)
        deadline=time.monotonic()+timeout
        while True:
            self.tick()
            try:
                self.current()
                break
            except RuntimeError:
                if time.monotonic()>deadline: raise
                time.sleep(0.1)
        publishers=self.node.get_publishers_info_by_topic('/feedback/joint_states')
        if len(publishers)!=1 or publishers[0].node_name!='agx_arm_ctrl_single_node':
            raise RuntimeError('Expected one official real-feedback publisher')

    def connect_model(self, timeout=30):
        """Load the official planning model without requiring a hardware publisher."""
        deadline=time.monotonic()+timeout
        while not self.planning.service_is_ready():
            self.tick()
            if time.monotonic()>deadline: raise TimeoutError('Official MoveIt is not ready')
            time.sleep(0.1)
        self.urdf,self.srdf=self.model()
        self.model_sha256=model_hash(self.urdf,self.srdf)
        urdf,srdf=ET.fromstring(self.urdf),ET.fromstring(self.srdf)
        links={link.get('name') for link in urdf.findall('link')}
        if not {self.settings['frame_id'],self.settings['tcp_link']}<=links:
            raise RuntimeError('Configured reference frame/TCP absent from running model')
        group=srdf.find(f"group[@name='{self.settings['group']}']")
        chain=None if group is None else group.find('chain')
        if chain is None or chain.get('base_link')!=self.settings['frame_id'] or chain.get('tip_link')!=self.settings['tcp_link']:
            raise RuntimeError('Task must use the running arm chain base/TCP')
        for name in JOINTS:
            joint=urdf.find(f"joint[@name='{name}']")
            if joint is None or joint.get('type')!='revolute':
                raise RuntimeError(f'{name}: expected official bounded revolute joint')
            limit=joint.find('limit')
            self.limits[name]=(float(limit.get('lower')),float(limit.get('upper')))
    def begin_round(self):
        """Clear only terminal state from the previous round, never an active goal."""
        self.tick()
        if any(item.status in (1,2,3) for item in self.external_status):
            raise RuntimeError('控制器正在执行其他运动，请等待结束后再规划。')
        if self.pending_goal is not None:
            if not self.pending_goal.done():
                raise RuntimeError('上一条执行请求尚未结束，请等待取消结果。')
            if self.active_goal is None:
                handle=self.pending_goal.result()
                if handle is not None and handle.accepted:
                    self.active_goal=handle
        if self.active_goal is not None and self.active_goal.accepted:
            if self.execution_result_future is None:
                self.execution_result_future=self.active_goal.get_result_async()
            if not self.execution_result_future.done():
                raise RuntimeError('上一条执行仍未终止，请先确认停止。')
        self.pending_goal=None
        self.active_goal=None
        self.execution_result_future=None
        self.motion_submitted=False

    def tick(self, health=False):
        if self.cancel_requested.is_set(): raise InterruptedError('Task canceled')
        if health: self.feedback.health(self.settings['feedback_timeout'],self.settings['diagnostic_timeout'])

    def current(self):
        return self.feedback.get(self.settings['feedback_timeout'])

    def check_joints(self,q):
        if len(q)!=6 or not all(math.isfinite(x) for x in q): raise ValueError('Invalid six-joint target')
        violations=[]
        for name,value in zip(JOINTS,q):
            lower,upper=self.limits[name]
            if not lower<=value<=upper:
                violations.append(dict(joint=name,value_rad=value,lower_rad=lower,upper_rad=upper))
        if violations: raise JointBoundsError(violations)

    def check_state(self,q):
        self.check_joints(q)
        request=GetStateValidity.Request()
        request.group_name=self.settings['group']
        request.robot_state=robot_state(q)
        if not self.call(self.validity,request,tick=self.tick).valid:
            raise RuntimeError('State is in collision or invalid in current planning scene')

    def plan(self,point,start,*,publish=True):
        self.check_state(start)
        constraints=Constraints()
        if point['type']=='joint':
            self.check_joints(point['joints_rad'])
            self.check_state(point['joints_rad'])
            for name,value in zip(JOINTS,point['joints_rad']):
                item=JointConstraint(joint_name=name,position=float(value),weight=1.0)
                item.tolerance_above=item.tolerance_below=self.settings['joint_tolerance_rad']/2
                constraints.joint_constraints.append(item)
        else:
            position=PositionConstraint()
            position.header.frame_id=point['frame_id']
            position.link_name=self.settings['tcp_link']
            position.weight=1.0
            # Reserve half of the arrival tolerance for physical tracking error,
            # just as joint targets do. A boundary IK solution is not exact arrival.
            primitive=SolidPrimitive(type=SolidPrimitive.SPHERE,dimensions=[self.settings['position_tolerance_m']/2])
            pose=Pose()
            pose.position.x,pose.position.y,pose.position.z=point['position_m']
            pose.orientation.w=1.0
            position.constraint_region.primitives=[primitive]
            position.constraint_region.primitive_poses=[pose]
            orientation=OrientationConstraint()
            orientation.header.frame_id=point['frame_id']
            orientation.link_name=self.settings['tcp_link']
            orientation.weight=1.0
            orientation.orientation.x,orientation.orientation.y,orientation.orientation.z,orientation.orientation.w=point['orientation_xyzw']
            orientation.absolute_x_axis_tolerance=orientation.absolute_y_axis_tolerance=orientation.absolute_z_axis_tolerance=self.settings['orientation_tolerance_rad']/(2*math.sqrt(3))
            constraints.position_constraints=[position]
            constraints.orientation_constraints=[orientation]
        request=GetMotionPlan.Request()
        motion=request.motion_plan_request
        motion.group_name=self.settings['group']
        motion.pipeline_id='ompl'
        motion.planner_id=getattr(self,'planner_id','')
        motion.start_state=robot_state(start)
        motion.goal_constraints=[constraints]
        for key in ('velocity','acceleration'):
            setattr(motion,f'max_{key}_scaling_factor',self.settings[f'{key}_scaling'])
        motion.allowed_planning_time=self.settings['planning_time']
        motion.num_planning_attempts=self.settings['planning_attempts']
        response=self.call(self.planning,request,self.settings['planning_time']+10,tick=self.tick).motion_plan_response
        if response.error_code.val!=1:
            raise PlanningError(response.error_code)
        trajectory=response.trajectory
        joints=trajectory.joint_trajectory
        if trajectory.multi_dof_joint_trajectory.points or len(joints.joint_names)!=6 or set(joints.joint_names)!=set(JOINTS) or not joints.points:
            raise RuntimeError('MoveIt returned invalid arm trajectory')
        order=[joints.joint_names.index(name) for name in JOINTS]
        previous=-1.0
        for p in joints.points:
            t=p.time_from_start.sec+p.time_from_start.nanosec/1e9
            if not math.isfinite(t) or t<0 or t<=previous or len(p.positions)!=6:
                raise RuntimeError('Invalid trajectory timestamps or positions')
            if any(len(values) not in (0,6) for values in (p.velocities,p.accelerations,p.effort)):
                raise RuntimeError('Invalid trajectory derivative/effort dimensions')
            if any(not math.isfinite(v) for values in (p.positions,p.velocities,p.accelerations,p.effort) for v in values):
                raise RuntimeError('Nonfinite trajectory sample')
            self.check_joints([p.positions[i] for i in order])
            previous=t
        if max(abs(joints.points[0].positions[i]-q) for i,q in zip(order,start))>self.settings['start_tolerance_rad']:
            raise RuntimeError('Planned start does not match requested real start')
        if publish:
            self.publish_plan(start,[trajectory])
        return trajectory,[joints.points[-1].positions[i] for i in order],previous

    def publish_plan(self,start,trajectories):
        """Publish display messages only; this never submits execution."""
        display=DisplayTrajectory()
        display.trajectory_start=robot_state(start)
        display.trajectory=list(trajectories)
        self.preview_message=display
        self.display.publish(display)

    def publish_targets(self,points):
        array=MarkerArray()
        clear=Marker(action=Marker.DELETEALL)
        clear.ns='multi_point'
        array.markers.append(clear)
        for i,point in enumerate(points):
            if point['type']=='joint':
                pose=self.fk(point['joints_rad'],self.settings['frame_id'],self.settings['tcp_link'])
            else: pose=point
            marker=Marker()
            marker.header.frame_id=self.settings['frame_id']
            marker.ns='multi_point'
            marker.id=i
            marker.type=Marker.SPHERE
            marker.action=Marker.ADD
            marker.pose.position.x,marker.pose.position.y,marker.pose.position.z=pose['position_m']
            marker.pose.orientation.w=1.0
            marker.scale.x=marker.scale.y=marker.scale.z=0.015
            marker.color.r,marker.color.g,marker.color.b,marker.color.a=0.1,0.8,0.3,1.0
            array.markers.append(marker)
            import copy
            label=copy.deepcopy(marker)
            label.id=10000+i
            label.type=Marker.TEXT_VIEW_FACING
            label.text=point['name']
            label.pose.position.z+=0.025
            label.scale.z=0.02
            array.markers.append(label)
        self.target_message=array
        self.targets.publish(array)

    def replay_preview(self):
        """Republish cached visualization only, without planning or executing."""
        if self.preview_message is not None:
            self.display.publish(self.preview_message)
        if self.target_message is not None:
            self.targets.publish(self.target_message)

    def clear_preview(self):
        """Release task visualization only; never stop or reset the official controller."""
        self.preview_message=None
        self.target_message=None
        self.display.publish(DisplayTrajectory())  # Replace the transient-local cached trajectory.
        self.targets.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def stable(self,target=None,timeout=None,health=True):
        window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
        deadline=time.monotonic()+(timeout or self.settings['arrival_timeout'])
        unavailable=None
        reported=False
        while time.monotonic()<deadline:
            self.tick(health)
            try:
                sample=self.current()
            except FeedbackUnavailableError as exc:
                # Only preview acquisition may wait for a new stream. Execution,
                # arrival and recovery still fail immediately on stale feedback.
                if health or target is not None or exc.details['reason']=='future_timestamp': raise
                unavailable=exc
                window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
                if not reported:
                    print('[WAIT] 预览等待新鲜反馈恢复，并重新确认停稳。',flush=True)
                    reported=True
                time.sleep(0.02)
                continue
            unavailable=None
            if window.update(sample,time.monotonic(),target,self.settings['joint_tolerance_rad']): return sample
            time.sleep(0.02)
        if unavailable is not None: raise unavailable
        raise TimeoutError('Real feedback did not reach/stabilize at target')

    def stable_pose(self,point,endpoint):
        """Require real joint and TCP arrival over one stable window; never correct motion."""
        window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
        deadline=time.monotonic()+self.settings['arrival_timeout']
        distance=angle=float('inf')
        last_stamp=None
        while time.monotonic()<deadline:
            self.tick(True)
            sample=self.current()
            if sample.stamp==last_stamp:
                time.sleep(0.02)
                continue
            last_stamp=sample.stamp
            pose=self.fk(sample.q,point['frame_id'],self.settings['tcp_link'])
            self.tick(True)
            now=time.monotonic()
            if now>=deadline: break
            if now-sample.received>self.settings['feedback_timeout']:
                window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
                continue
            distance,angle=pose_errors(pose,point)
            if distance>self.settings['position_tolerance_m'] or angle>self.settings['orientation_tolerance_rad']:
                window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
            elif window.update(sample,now,endpoint,self.settings['joint_tolerance_rad']):
                return sample,pose
            time.sleep(0.02)
        raise TimeoutError(
            f'{point["name"]} 实测末端未在 {self.settings["arrival_timeout"]:g}s 内稳定到位：'
            f'位置误差 {distance*1000:.3f} mm（上限 {self.settings["position_tolerance_m"]*1000:g} mm），'
            f'姿态误差 {angle:.6f} rad（上限 {self.settings["orientation_tolerance_rad"]:g} rad）')

    def execute(self,trajectory,start,duration):
        self.tick(True)
        if any(item.status in (1,2,3) for item in self.external_status):
            raise RuntimeError('Arm controller already has an active goal')
        sample=self.stable(timeout=5)
        if max(abs(a-b) for a,b in zip(start,sample.q))>self.settings['start_tolerance_rad']:
            raise RuntimeError('Real arm moved after planning; replan required')
        if not self.execution.wait_for_server(timeout_sec=3): raise RuntimeError('ExecuteTrajectory action unavailable')
        # Discovery can block: recheck cancellation, feedback and external goals
        # after it, immediately before submitting any trajectory.
        self.tick(True)
        if any(item.status in (1,2,3) for item in self.external_status):
            raise RuntimeError('Arm controller already has an active goal')
        if max(abs(a-b) for a,b in zip(start,self.current().q))>self.settings['start_tolerance_rad']:
            raise RuntimeError('Real arm moved while waiting for execution service; replan required')
        goal=ExecuteTrajectory.Goal(trajectory=trajectory,controller_names=['arm_controller'])
        self.motion_submitted=True
        self.pending_goal=self.execution.send_goal_async(goal)
        self.active_goal=self.wait(self.pending_goal,5,lambda:self.tick(True))
        if not self.active_goal.accepted: raise RuntimeError('MoveIt rejected execution')
        self.execution_result_future=self.active_goal.get_result_async()
        result=self.wait(self.execution_result_future,duration+self.settings['execution_margin_seconds'],lambda:self.tick(True))
        if result.status!=4 or result.result.error_code.val!=1:
            raise RuntimeError(f'MoveIt execution failed: status={result.status}, code={result.result.error_code.val}')
        self.active_goal=None

    def hold(self,target,seconds,point=None):
        self.tick(True)
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            self.tick(True)
            sample=self.current()
            if max(abs(a-b) for a,b in zip(sample.q,target))>self.settings['joint_tolerance_rad']:
                raise RuntimeError('Real arm drifted out of tolerance during dwell')
            if point is not None:
                if point['type']=='joint':
                    if max(abs(a-b) for a,b in zip(sample.q,point['joints_rad']))>self.settings['joint_tolerance_rad']:
                        raise RuntimeError(f'{point["name"]}: joint goal drifted out of tolerance during dwell')
                else:
                    pose=self.fk(sample.q,point['frame_id'],self.settings['tcp_link'])
                    self.tick(True)
                    if time.monotonic()-sample.received>self.settings['feedback_timeout']:
                        raise RuntimeError('FK sample became stale during dwell')
                    distance,angle=pose_errors(pose,point)
                    if distance>self.settings['position_tolerance_m'] or angle>self.settings['orientation_tolerance_rad']:
                        raise RuntimeError(f'{point["name"]}: TCP drifted during dwell: distance={distance}, angle={angle}')
            time.sleep(0.02)
        self.tick(True)

    def restore_task_control(self):
        """Explicit operator recovery of a gate closed by this task, never motor enable.

        The official controller can retain its canceled trajectory's last command.
        Align that command to measured joints while forwarding remains closed.
        Only reopen after both real feedback and controller output agree.
        """
        if not self.task_gate_closed: return
        if any(name=='control_gate' for name,_ in self.node.get_node_names_and_namespaces()):
            raise RuntimeError('Automatic control_gate conflicts with isolated recovery; use auto_control_gate:=false')
        self.begin_round()  # Reject unresolved goals before changing forwarding.
        sample=self.stable(timeout=5)
        response=self.call(self.gate,SetBool.Request(data=False),3)
        if not response.success:
            raise RuntimeError('Cannot confirm forwarding gate closed; recovery refused')
        trajectory=RobotTrajectory()
        trajectory.joint_trajectory.joint_names=list(JOINTS)
        point=JointTrajectoryPoint(positions=list(sample.q),velocities=[0.0]*6)
        point.time_from_start.nanosec=500000000
        trajectory.joint_trajectory.points=[point]
        try:
            # This goes through the official MoveIt/controller action, with CAN
            # forwarding still closed. It cannot resume the canceled task.
            self.execute(trajectory,sample.q,0.5)
            self.stable(target=sample.q,timeout=5)
            self.begin_round()
            self.tick(True)
            actual=self.current().q
            command=self.command_feedback.get(self.settings['feedback_timeout']).q
            if max(abs(a-b) for a,b in zip(actual,command))>self.settings['start_tolerance_rad']:
                raise RuntimeError('Controller hold command does not match real position; gate remains closed')
            self.tick(True)
            response=self.call(self.gate,SetBool.Request(data=True),3)
            self.tick(True)  # A stop received during the open request must close it again.
            if not response.success:
                raise RuntimeError('Forwarding gate recovery rejected: '+response.message)
            self.task_gate_closed=False
            print('控制已恢复：旧轨迹已结束，控制器保持目标已对齐实测位置。',flush=True)
        except BaseException:
            # Includes an ambiguous open-service timeout: request closed again.
            self.motion_submitted=True
            self.stop()
            raise

    def stop(self):
        result={'requested': self.motion_submitted, 'cancel':'not_requested','gate':'not_requested','position_stable_observed':False}
        if not self.motion_submitted: return result
        cancel_future=None
        if self.active_goal is not None and self.active_goal.accepted:
            try: cancel_future=self.active_goal.cancel_goal_async()
            except Exception as exc: result['cancel']=str(exc)
        elif self.pending_goal is not None:
            def cancel_late(future):
                try:
                    handle=future.result()
                    if handle and handle.accepted: handle.cancel_goal_async()
                except Exception: pass
            self.pending_goal.add_done_callback(cancel_late)
            result['cancel']='pending_goal_cancel_registered'
        self.task_gate_closed=True  # Also retain recovery requirement if acknowledgement is lost.
        try:
            response=self.call(self.gate,SetBool.Request(data=False),3)
            result['gate']='closed' if response.success else response.message
        except Exception as exc: result['gate']=str(exc)
        if cancel_future is not None:
            try:
                canceled=self.wait(cancel_future,3)
                result['cancel']='accepted' if canceled.goals_canceling else 'not_accepted_or_already_terminal'
            except Exception as exc: result['cancel']=str(exc)
        # Observe only: closing a forwarding gate is not a hardware braking command.
        window=StableWindow(self.settings['settle_seconds'],self.settings['stable_range_rad'])
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            try:
                if window.update(self.current(),time.monotonic()):
                    result['position_stable_observed']=True
                    break
            except Exception as exc:
                result['observation_error']=str(exc)
                break
            time.sleep(0.02)
        return result
