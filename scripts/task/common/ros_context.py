"""ROS service plumbing; constructing this object never creates a CAN/SDK driver."""
import hashlib
import threading
import time
import uuid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from .schema import JOINTS, quaternion, vector


def robot_state(q):
    state = RobotState()
    state.is_diff = True
    state.joint_state.name = JOINTS
    state.joint_state.position = list(map(float, q))
    return state


def fk_request(q, frame, tcp):
    request = GetPositionFK.Request()
    request.header.frame_id = frame
    request.fk_link_names = [tcp]
    request.robot_state = robot_state(q)
    return request


def fk_value(response, frame, tcp):
    if response.error_code.val != 1 or response.fk_link_names != [tcp] or len(response.pose_stamped) != 1:
        raise RuntimeError('MoveIt FK failed')
    stamped = response.pose_stamped[0]
    if stamped.header.frame_id != frame:
        raise RuntimeError('FK frame mismatch')
    p, q = stamped.pose.position, stamped.pose.orientation
    return dict(frame_id=frame, position_m=vector([p.x,p.y,p.z],3,'FK position_m'),
                orientation_xyzw=quaternion([q.x,q.y,q.z,q.w]))


def model_hash(urdf, srdf):
    return hashlib.sha256((urdf+'\n'+srdf).encode()).hexdigest()

class RosContext:
    def __init__(self, prefix='piper_task'):
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node(prefix+'_'+uuid.uuid4().hex[:8], context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.parameters = self.node.create_client(GetParameters, '/move_group/get_parameters')
        self.fk_client = self.node.create_client(GetPositionFK, '/compute_fk')

    def wait(self, future, timeout=10, tick=None):
        deadline = time.monotonic()+timeout
        while not future.done():
            if tick: tick()
            if time.monotonic() > deadline:
                raise TimeoutError('ROS request timed out')
            time.sleep(0.01)
        if tick: tick()  # A completed future must not bypass cancellation/health checks.
        result = future.result()
        if result is None: raise RuntimeError('ROS request returned no result')
        return result

    def call(self, client, request, timeout=10, tick=None):
        if tick: tick()
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        if tick: tick()
        return self.wait(client.call_async(request), timeout, tick)

    def model(self):
        request = GetParameters.Request(names=['robot_description','robot_description_semantic'])
        values = self.call(self.parameters, request, tick=getattr(self,'tick',None)).values
        if len(values)!=2 or any(v.type!=4 or not v.string_value for v in values):
            raise RuntimeError('Live MoveIt URDF/SRDF unavailable')
        return [v.string_value for v in values]

    def fk(self, q, frame, tcp):
        return fk_value(self.call(self.fk_client, fk_request(q,frame,tcp),
                                  tick=getattr(self,'tick',None)), frame,tcp)

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2)
        self.node.destroy_node()
        self.context.try_shutdown()
