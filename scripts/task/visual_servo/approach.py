"""Legacy path reconstruction for recordings made with planned startup."""
import numpy as np
from scipy.interpolate import BPoly
from scipy.spatial.transform import Rotation
from moveit_msgs.msg import OrientationConstraint

from servo_tracking.core import JOINTS, joint_speed_limits, validate_joint_plan
from servo_tracking.joint_motion import joint_plan_request


class ApproachTarget:
    """Freeze the observed object in base_link, independent of eye-in-hand motion."""
    def __init__(self, goal, stamp):
        self.center = goal.center.copy()
        self.normal = None if goal.normal is None else goal.normal.copy()
        self.stamp = stamp

    def matches(self, goal, cfg):
        if np.linalg.norm(goal.center - self.center) > cfg.get('approach_target_drift_m', .006):
            return False
        if self.normal is None or goal.normal is None:
            return self.normal is goal.normal
        return self.normal @ goal.normal >= np.cos(np.deg2rad(cfg.get('approach_target_drift_deg', 2.)))


def approach_request(start, goal, config, scale, actual, target_pose, visual, attempt, normal=None):
    headroom = .6**max(0, attempt-1)
    request = joint_plan_request(start, goal, float(config['planning_time']),
                                 scale*.8*headroom, scale*.25*headroom)
    motion = request.motion_plan_request
    if attempt == 0:
        # LIN connects the measured start to the FK of the approved joints in
        # Cartesian space, keeping orientation constant for a translation.
        motion.pipeline_id, motion.planner_id = 'pilz_industrial_motion_planner', 'LIN'
        motion.num_planning_attempts = 1
        return request
    # Bound intermediate tool rotation as well as the IK endpoint. Express
    # the constraint relative to the final orientation; the start is included.
    desired = Rotation.from_quat(target_pose[3:])
    tilt = (desired.inv()*Rotation.from_quat(actual[3:])).magnitude()
    tolerance = max(float(tilt), np.deg2rad(visual['normal_tolerance_deg'])) + 1e-4
    if attempt == 1 and normal is not None:
        # An XYZ rotation-vector box around the held tool orientation can extend
        # beyond the cone around the board normal (e.g. a tool already at 3.2°).
        # Prefer an inscribed box on the first detour. The final search may
        # explore a wider set of orientations; both require the same cone check.
        target_tilt = np.arccos(np.clip(desired.as_matrix()[:, 2] @ normal, -1., 1.))
        budget = (np.deg2rad(visual['normal_tolerance_deg']) - target_tilt) / np.sqrt(3.)
        if budget > tilt + 1e-4:
            tolerance = min(tolerance, budget)
    constraint = OrientationConstraint(link_name='tcp_link', weight=1.)
    constraint.header.frame_id = 'base_link'
    quat = constraint.orientation
    quat.x, quat.y, quat.z, quat.w = map(float, desired.as_quat())
    constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
    constraint.absolute_x_axis_tolerance = tolerance
    constraint.absolute_y_axis_tolerance = tolerance
    constraint.absolute_z_axis_tolerance = tolerance
    request.motion_plan_request.path_constraints.orientation_constraints = [constraint]
    return request


class ApproachPath:
    """Dense joint references on MoveIt's checked quintic. Advance from feedback."""
    def __init__(self, response, start, goal, model, config, scale, visual, normal):
        result = response.motion_plan_response
        if result.error_code.val != 1:
            raise ValueError(f'MoveIt 接近规划失败（{result.error_code.val}）')
        if result.trajectory.multi_dof_joint_trajectory.points:
            raise ValueError('接近规划包含浮动关节')
        trajectory = result.trajectory.joint_trajectory
        duration = validate_joint_plan(trajectory, start, goal, model, config, scale)
        order = [trajectory.joint_names.index(name) for name in JOINTS]
        times, derivatives = [], []
        for point in trajectory.points:
            times.append(point.time_from_start.sec + point.time_from_start.nanosec*1e-9)
            derivatives.append(np.asarray([point.positions, point.velocities, point.accelerations])[:, order])
        curve = BPoly.from_derivatives(times, derivatives)
        # Include original knots and bound the maximum single-axis gap to
        # 0.003 rad using the continuous speed bound checked above.
        period = min(.025, .003 / np.max(joint_speed_limits(config, scale)))
        grid = np.unique(np.r_[times, np.linspace(0., duration, int(np.ceil(duration/period))+1)])
        self.points = curve(grid)
        low, high = np.array(visual['workspace_min_m']), np.array(visual['workspace_max_m'])
        min_cos = None
        if normal is not None:
            initial_cos = model.transforms(start)[0, :3, 2] @ normal
            min_cos = min(initial_cos, np.cos(np.deg2rad(visual['normal_tolerance_deg'])))
        for block in np.array_split(self.points, max(1, int(np.ceil(len(grid)/1024)))):
            transforms = model.transforms(block)
            positions = transforms[:, :3, 3]
            if np.any(positions < low) or np.any(positions > high):
                raise ValueError('接近路径超出任务工作空间')
            if min_cos is not None:
                worst = (transforms[:, :3, 2] @ normal).min()
                if worst < min_cos-1e-5:
                    angle, limit = np.rad2deg(np.arccos(np.clip([worst, min_cos], -1., 1.)))
                    raise ValueError(f'接近路径倾角 {angle:.2f}° 超过 {limit:.2f}°')
        self.index = 0
        self.final = self.points[-1]
        self.arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(self.points, axis=0), axis=1))]
        self.reference_index = 0

    def step(self, q, model, config, scale):
        # Follow the first local minimum of measured distance along the route.
        # This keeps the cursor on the current branch at a turn or crossing.
        end = self.arc.searchsorted(self.arc[self.index] + .022, side='right')
        distances = np.linalg.norm(self.points[self.index:end] - q, axis=1)
        rises = np.flatnonzero(np.diff(distances) > 0.)
        self.index += int(rises[0]) if len(rises) else len(distances)-1
        if np.max(abs(self.points[self.index] - q)) > .022:
            raise ValueError('实测关节偏离接近路径，请重新开始')
        # Chase a short stretch of the checked curve instead of requiring all
        # six joints to settle at every interpolation sample. At a bend cap
        # the deviation of the intervening references from the chord.
        last = len(self.points)-1
        ref = max(self.index, int(self.arc.searchsorted(self.arc[self.index] + .016, side='right'))-1)
        while ref > self.index+1:
            arc = self.arc[self.index:ref+1] - self.arc[self.index]
            if arc[-1] == 0.:
                break
            span = self.points[ref] - self.points[self.index]
            chord = self.points[self.index] + np.outer(arc/arc[-1], span)
            if np.max(abs(chord - self.points[self.index:ref+1])) <= .002:
                break
            ref = (self.index + ref)//2
        self.reference_index = ref
        if ref == last and np.max(abs(self.final-q)) <= .002:
            return None
        velocity = (self.points[ref]-q) * 12.
        jacobian = model.jacobian(q)
        twist = jacobian @ velocity
        limits = config['control']
        ratio = max(1., float(np.max(abs(velocity)/joint_speed_limits(config, scale))),
                    np.linalg.norm(twist[:3])/(scale*limits['linear_speed']),
                    np.linalg.norm(twist[3:])/(scale*limits['angular_speed']))
        return twist/ratio, velocity/ratio
