"""Apply named virtual objects, preserving unrelated scene content."""
import hashlib
import json
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict

PREFIX = 'multi_point/'


def obstacle_message(obj, prefix=PREFIX):
    message = CollisionObject()
    message.id = prefix+obj['id']
    message.header.frame_id = obj['frame_id']
    message.operation = CollisionObject.ADD
    message.pose.position.x, message.pose.position.y, message.pose.position.z = obj['position_m']
    (message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w) = obj['orientation_xyzw']
    primitive = SolidPrimitive()
    primitive.type = {'box': SolidPrimitive.BOX, 'sphere': SolidPrimitive.SPHERE, 'cylinder': SolidPrimitive.CYLINDER}[obj['shape']]
    primitive.dimensions = obj['dimensions_m']
    identity = Pose()
    identity.orientation.w = 1.0
    message.primitives = [primitive]
    message.primitive_poses = [identity]
    return message


def fingerprint(scene):
    value = message_to_ordereddict(scene)
    # Dynamic robot joint positions and publication stamps are not obstacle edits.
    objects = value['world']['collision_objects']
    for obj in objects:
        obj['header']['stamp'] = {'sec':0,'nanosec':0}
    value['world']['collision_objects'] = sorted(objects, key=lambda o:o['id'])
    value['world']['octomap']['header']['stamp'] = {'sec':0,'nanosec':0}
    selected = {k:value[k] for k in ('world','allowed_collision_matrix','link_padding','link_scale','fixed_frame_transforms')}
    selected['attached_objects'] = value['robot_state']['attached_collision_objects']
    for transform in selected['fixed_frame_transforms']:
        transform['header']['stamp'] = {'sec':0,'nanosec':0}
    return hashlib.sha256(json.dumps(selected,sort_keys=True).encode()).hexdigest()

class PlanningScene:
    def __init__(self, context, prefix=PREFIX):
        self.context = context
        if not prefix or not prefix.endswith('/') or prefix == '/':
            raise ValueError('Scene prefix must be nonempty and end with /')
        self.prefix = prefix
        from tf2_ros import Buffer, TransformListener
        self.transforms = Buffer(node=context.node)
        self.listener = TransformListener(self.transforms, context.node, spin_thread=False)
        self.getter = context.node.create_client(GetPlanningScene, '/get_planning_scene')
        self.setter = context.node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    def get(self):
        request = GetPlanningScene.Request()
        request.components.components = 1|4|16|32|64|128|256
        return self.context.call(self.getter,request,tick=getattr(self.context,'tick',None)).scene

    def apply(self, config):
        before = self.get()
        if not config['enabled']:
            return before
        desired = [obstacle_message(obj, self.prefix) for obj in config['objects']]
        desired_ids = {obj.id for obj in desired}
        changes = list(desired)
        for obj in before.world.collision_objects:
            if obj.id.startswith(self.prefix) and obj.id not in desired_ids:
                removal = CollisionObject()
                removal.id, removal.operation = obj.id, CollisionObject.REMOVE
                changes.append(removal)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = changes
        if not self.context.call(self.setter, request,tick=getattr(self.context,'tick',None)).success:
            raise RuntimeError('MoveIt rejected obstacle scene')
        after = self.get()
        actual = {obj.id:obj for obj in after.world.collision_objects}
        if {name for name in actual if name.startswith(self.prefix)} != desired_ids:
            raise RuntimeError('Scene object IDs do not match applied configuration')
        # Verify geometric identity in the planning frame; compare numeric poses, not timestamps.
        from math import isclose
        for obj in desired:
            found = actual[obj.id]
            if len(found.primitives)!=1 or found.primitives[0].type!=obj.primitives[0].type:
                raise RuntimeError('Scene primitive type mismatch')
            expected_pose = obj.pose
            actual_pose = found.pose
            from tf2_geometry_msgs import do_transform_pose
            from rclpy.time import Time
            from rclpy.duration import Duration
            if found.header.frame_id.lstrip('/') != obj.header.frame_id.lstrip('/'):
                transform = self.transforms.lookup_transform(found.header.frame_id, obj.header.frame_id, Time(), timeout=Duration(seconds=2))
                expected_pose = do_transform_pose(expected_pose, transform)
            if len(found.primitive_poses) != 1:
                raise RuntimeError('Scene primitive pose missing')
            from geometry_msgs.msg import TransformStamped
            placement = TransformStamped()
            placement.transform.translation.x = found.pose.position.x
            placement.transform.translation.y = found.pose.position.y
            placement.transform.translation.z = found.pose.position.z
            placement.transform.rotation = found.pose.orientation
            actual_pose = do_transform_pose(found.primitive_poses[0], placement)
            expected = [*obj.primitives[0].dimensions, expected_pose.position.x,expected_pose.position.y,expected_pose.position.z,
                        expected_pose.orientation.x,expected_pose.orientation.y,expected_pose.orientation.z,expected_pose.orientation.w]
            values = [*found.primitives[0].dimensions, actual_pose.position.x,actual_pose.position.y,actual_pose.position.z,
                      actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w]
            if len(values)!=len(expected) or not all(isclose(a,b,abs_tol=1e-6) for a,b in zip(values[:-4],expected[:-4])) or not isclose(abs(sum(a*b for a,b in zip(values[-4:],expected[-4:]))),1.0,abs_tol=1e-6):
                raise RuntimeError(f'Scene geometry mismatch: {obj.id}')
        return after
