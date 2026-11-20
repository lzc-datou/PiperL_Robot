"""ROS-independent geometry and validation. No device I/O or joint interpolation."""
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from .cartesian_control import CartesianController, load_control_config, validate_control_config

JOINTS = [f'joint{i}' for i in range(1, 7)]
# PiPER-L user manual V1.1 (January 2025), section 1.2, maximum joint speeds:
# J1/J3 180, J2 195, J4/J5/J6 225 degrees/s. These are hardware ceilings,
# separate from the task's nominal speed and operator scaling.
# https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/fvtsoyfpewxeuy5v?singleDoc
HARDWARE_JOINT_SPEED_LIMITS = np.deg2rad([180., 195., 180., 225., 225., 225.])
HARDWARE_JOINT_SPEED_LIMITS.setflags(write=False)
MAX_ZERO_FEEDBACK_TOLERANCE = math.radians(.1)


def _zero_feedback_tolerance(value):
    if (isinstance(value, bool) or not isinstance(value, (int, float, np.integer, np.floating)) or
            not math.isfinite(value) or not 0 <= value <= MAX_ZERO_FEEDBACK_TOLERANCE):
        raise ValueError('zero_feedback_tolerance 必须是 0 到 0.1° 之间的有限数值')
    return float(value)


def validate_arm_mode(values):
    # Official diagnostic OK also includes teach record/play/pause. Those are not Servo-ready.
    try:
        normal = int(values['arm_status']) == 0 and int(values['teach_status']) == 0
        control = int(values['ctrl_mode']) in (0, 1)  # standby or CAN command mode
    except (KeyError, ValueError, TypeError):
        raise ValueError('机械臂控制模式反馈无效')
    if not normal or not control:
        raise ValueError('机械臂处于示教、外部控制或异常模式')


def vector(values, size):
    result = np.asarray(values, dtype=float)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f'需要 {size} 个有限数值')
    return result


def joint_speed_limits(config, scale=1.):
    """Return each axis's nominal speed, capped by its PiPER-L hardware ceiling."""
    try:
        nominal = float(config['joint_speed'])
        limits = vector(config.get('joint_velocity_limits', HARDWARE_JOINT_SPEED_LIMITS), 6)
        if (not math.isfinite(nominal) or nominal <= 0 or
                not math.isfinite(scale) or not 0 < scale <= 1. or
                np.any(limits <= 0) or np.any(limits > HARDWARE_JOINT_SPEED_LIMITS)):
            raise ValueError('关节速度配置无效或超过 PiPER-L 官方各轴上限')
        return np.minimum(nominal, limits) * scale
    except (KeyError, TypeError, OverflowError) as exc:
        raise ValueError('关节速度配置无效') from exc


def normalized_pose(values):
    p = vector(values, 7).copy()
    length = np.linalg.norm(p[3:])
    if length < 1e-8:
        raise ValueError('无效姿态四元数')
    p[3:] /= length
    return p


def pose_error(target, actual):
    target, actual = normalized_pose(target), normalized_pose(actual)
    rotation = Rotation.from_quat(target[3:]) * Rotation.from_quat(actual[3:]).inv()
    return target[:3] - actual[:3], rotation.as_rotvec()


def capped(value, maximum):
    norm = np.linalg.norm(value)
    return value * min(1., maximum / max(norm, 1e-12))


def tracking_twist(target, actual, config, scale):
    """Compatibility entry point for the independent controller's stateless P mode."""
    if not math.isfinite(scale) or not 0 < scale <= 1:
        raise ValueError('无效速度比例')
    settings = {key: config[key] for key in
                ('linear_speed', 'angular_speed', 'position_tolerance', 'orientation_tolerance')}
    settings.update(algorithm='p', gains={'p': {key: config[key] for key in
                    ('position_gain', 'orientation_gain')}})
    return CartesianController(settings).update(target, actual, now=0., target_time=0., speed_scale=scale)


def damped_joint_velocity(q, twist, model, config, scale=1.):
    """Map a base-frame TCP twist to bounded joint velocities, without a trajectory.

    A fixed positive damping regularizes even a rank-deficient Jacobian; no
    condition-number threshold pauses or switches the command. Angular rows use
    a characteristic length (m/rad) to balance translation and rotation. The
    caller still validates fresh feedback and the complete target pose.

    These velocities enter official Servo's JOINT_JOG input. Its common output
    path retains collision scaling, joint bounds, velocity limits and smoothing:
    https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/moveit_servo/src/utils/command.cpp#L76-L130
    https://github.com/moveit/moveit2/blob/2.12.4/moveit_ros/moveit_servo/src/servo.cpp#L439-L506
    """
    q, twist = vector(q, 6), vector(twist, 6)
    damping, rotation_weight = _dls_parameters(config)
    limits = joint_speed_limits(config, scale)
    weights = np.array([1., 1., 1., rotation_weight, rotation_weight, rotation_weight])
    jacobian = np.asarray(model.jacobian(q), dtype=float)
    if jacobian.shape != (6, 6) or not np.all(np.isfinite(jacobian)):
        raise ValueError('雅可比矩阵必须为 6×6 有限数值')
    weighted_jacobian, weighted_twist = weights[:, None] * jacobian, weights * twist
    if not np.all(np.isfinite(weighted_jacobian)) or not np.all(np.isfinite(weighted_twist)):
        raise ValueError('阻尼逆解输入超出有限数值范围')
    try:
        u, singular_values, vt = np.linalg.svd(weighted_jacobian, full_matrices=False)
        # s / (s*s + lambda*lambda), written with hypot to avoid squaring
        # overflow/underflow. A zero singular value has exactly zero gain.
        denominator = np.hypot(singular_values, damping)
        gains = (singular_values / denominator) / denominator
        velocity = vt.T @ (gains * (u.T @ weighted_twist))
    except np.linalg.LinAlgError as exc:
        raise ValueError('阻尼逆解数值分解失败') from exc
    if not np.all(np.isfinite(velocity)):
        raise ValueError('阻尼逆解产生非有限关节速度')
    # A common factor preserves the solved joint direction; per-axis clipping
    # would introduce a different Cartesian direction at every active bound.
    ratio = float(np.max(np.abs(velocity) / limits))
    return velocity / max(1., ratio)


def _dls_parameters(config):
    values = []
    for key, default in (('dls_damping', .02), ('dls_rotation_weight', .25)):
        value = config.get(key, default)
        if (isinstance(value, bool) or not isinstance(value, (float, int, np.floating, np.integer)) or
                not math.isfinite(value) or value <= 0):
            raise ValueError(f'配置 {key} 必须是有限正数')
        values.append(float(value))
    return tuple(values)


class RobotModel:
    """URDF FK for the configured base→TCP chain; workspace is position-only."""
    def __init__(self, urdf, base='base_link', tip='tcp_link'):
        root = ET.fromstring(urdf)
        self.link_names = [link.get('name') for link in root.findall('link')]
        by_child = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain, link = [], tip
        while link != base:
            j = by_child[link]
            chain.append(j)
            link = j.find('parent').get('link')
        self.chain, self.lower, self.upper = [], np.zeros(6), np.zeros(6)
        names = []
        for j in reversed(chain):
            kind, name = j.get('type'), j.get('name')
            origin = j.find('origin')
            xyz = [0., 0., 0.] if origin is None else list(map(float, origin.get('xyz', '0 0 0').split()))
            rpy = [0., 0., 0.] if origin is None else list(map(float, origin.get('rpy', '0 0 0').split()))
            transform = np.eye(4)
            transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            transform[:3, 3] = xyz
            axis, index = None, None
            if kind != 'fixed':
                if kind != 'revolute' or name not in JOINTS:
                    raise ValueError(f'不支持的关节: {name}/{kind}')
                index = JOINTS.index(name)
                names.append(name)
                limit = j.find('limit')
                self.lower[index], self.upper[index] = float(limit.get('lower')), float(limit.get('upper'))
                axis = vector(list(map(float, j.find('axis').get('xyz').split())), 3)
                axis /= np.linalg.norm(axis)
            self.chain.append((transform, axis, index))
        if set(names) != set(JOINTS):
            raise ValueError('模型必须包含完整的 joint1…joint6')
        self.radius = sum(np.linalg.norm(t[:3, 3]) for t, _, _ in self.chain)

    def check_joints(self, q, margin=0.):
        q = vector(q, 6)
        violations = self._bound_violations(q, self.lower + margin, self.upper - margin)
        if violations:
            self._raise_bounds(violations)
        return q

    @staticmethod
    def _bound_violations(q, lower, upper):
        violations = []
        for name, value, low, high in zip(JOINTS, q, lower, upper):
            if value < low or value > high:
                side = 'lower' if value < low else 'upper'
                violations.append(dict(joint=name, value=float(value), lower=float(low),
                    upper=float(high), delta=float(low-value if side == 'lower' else value-high), side=side))
        return violations

    @staticmethod
    def _raise_bounds(violations):
        details = '；'.join(
            f'{v["joint"]}: {math.degrees(v["value"]):.6f}° '
            f'不在 [{math.degrees(v["lower"]):.6f}°, {math.degrees(v["upper"]):.6f}°]，'
            f'越界 {math.degrees(v["delta"]):.6f}°' for v in violations)
        raise ValueError('关节角超出模型限位：' + details)

    def joint_bound_violations(self, q):
        """Describe hard-bound violations without changing the measured joints."""
        return self._bound_violations(vector(q, 6), self.lower, self.upper)

    def check_feedback(self, q, zero_tolerance=0.):
        """Accept only narrow J2/J3 zero-edge feedback error, preserving raw q.

        This classifies measurements. Targets and command trajectories must use
        check_joints(); an accepted measurement alone never authorizes motion.
        """
        q = vector(q, 6)
        tolerance = _zero_feedback_tolerance(zero_tolerance)
        blocked = []
        for violation in self.joint_bound_violations(q):
            lower_zero = (violation['joint'] == 'joint2' and violation['side'] == 'lower' and
                          math.isclose(violation['lower'], 0., abs_tol=1e-9))
            upper_zero = (violation['joint'] == 'joint3' and violation['side'] == 'upper' and
                          math.isclose(violation['upper'], 0., abs_tol=1e-9))
            if not ((lower_zero or upper_zero) and violation['delta'] <= tolerance):
                blocked.append(violation)
        if blocked:
            self._raise_bounds(blocked)
        return q

    def planning_joints(self, q, zero_tolerance=0.):
        """Build a bounded planning reference without modifying measured joints.

        Only the explicitly accepted J2/J3 zero-edge measurement deviations
        may be normalized to their exact model boundary. This reference is not
        feedback, calibration, a motion command, or a relaxation of path bounds.
        """
        reference = self.check_feedback(q, zero_tolerance).copy()
        if reference[1] < self.lower[1]:
            reference[1] = self.lower[1]
        if reference[2] > self.upper[2]:
            reference[2] = self.upper[2]
        return self.check_joints(reference)

    def transforms(self, q):
        q = np.atleast_2d(q)
        out = np.broadcast_to(np.eye(4), (len(q), 4, 4)).copy()
        for origin, axis, index in self.chain:
            out = out @ origin
            if index is not None:
                rot = np.broadcast_to(np.eye(4), out.shape).copy()
                rot[:, :3, :3] = Rotation.from_rotvec(q[:, index, None] * axis).as_matrix()
                out = out @ rot
        return out

    def pose(self, q):
        result = self.transforms(vector(q, 6))[0]
        return np.r_[result[:3, 3], Rotation.from_matrix(result[:3, :3]).as_quat()]

    def jacobian(self, q):
        """Return the 6×6 TCP geometric Jacobian in the base frame (linear, angular)."""
        q = vector(q, 6)
        transform = np.eye(4)
        origins, axes = np.zeros((6, 3)), np.zeros((6, 3))
        for origin, axis, index in self.chain:
            transform = transform @ origin
            if index is not None:
                origins[index] = transform[:3, 3]
                axes[index] = transform[:3, :3] @ axis
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(q[index] * axis).as_matrix()
                transform = transform @ rotation
        linear = np.cross(axes, transform[:3, 3] - origins)
        return np.vstack((linear.T, axes.T))

    def condition_number(self, q):
        """Return the unweighted geometric-Jacobian condition number, or infinity."""
        singular_values = np.linalg.svd(self.jacobian(q), compute_uv=False)
        if singular_values[-1] <= 1e-12 * max(1., singular_values[0]):
            return float('inf')
        return float(singular_values[0] / singular_values[-1])

    def workspace_surface(self, samples, resolution):
        # Low-discrepancy samples cover joint limits more evenly than IID sampling.
        from scipy.stats import qmc
        q = qmc.Sobol(6, scramble=True, seed=7).random_base2(int(math.ceil(math.log2(samples))))[:samples]
        points = self.transforms(qmc.scale(q, self.lower, self.upper))[:, :3, 3]
        return workspace_surface(points, resolution)


def workspace_surface(points, resolution, max_faces=20000):
    """Smooth sampled occupancy, then extract only its boundary (not a convex hull).

    This is an approximate position envelope, never a reachability/authorization test.
    A bounded triangle count keeps both rendering and marker picking inexpensive.
    """
    from scipy.ndimage import gaussian_filter
    from skimage.measure import marching_cubes
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('工作空间采样点无效')
    if not math.isfinite(resolution) or resolution <= 0 or max_faces < 100:
        raise ValueError('工作空间曲面参数无效')
    for _ in range(16):
        origin = np.floor(points.min(axis=0)/resolution)*resolution - 3*resolution
        cells = np.floor((points-origin)/resolution).astype(int)
        shape = tuple(cells.max(axis=0)+4)
        if np.prod(shape) > 4_000_000:
            resolution *= 1.25
            continue
        occupied = np.zeros(shape, dtype=np.float32)
        occupied[tuple(cells.T)] = 1.
        field = gaussian_filter(occupied, sigma=1., mode='constant')
        # Fixed occupancy threshold, independent of sampling density per occupied cell.
        level = .35
        if field.max() <= level:
            resolution *= 1.25
            continue
        vertices, faces, _, _ = marching_cubes(field, level, spacing=(resolution,)*3,
                                              allow_degenerate=False)
        if len(faces) <= max_faces:
            # Samples occupy cells centred at index+0.5, not at their lower corners.
            return vertices[faces].astype(float) + origin + .5*resolution, resolution
        resolution *= 1.25
    raise ValueError('无法生成有界工作空间曲面')


def surface_vertex_colors(triangles):
    """Smooth baked shading avoids RViz's per-triangle lighting on a translucent shell."""
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    normals = np.zeros_like(vertices)
    face_normals = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    light = np.array([.4, -.5, .8]); light /= np.linalg.norm(light)
    shade = .65 + .35 * np.abs(normals @ light)
    return shade[indices, None] * np.array([.16, .95, .45])


class FreshJoints:
    def __init__(self):
        self.q = None
        self.stamp = 0.
        self.received = 0.
        self.error = ''

    def update(self, names, positions, stamp, now=None):
        now = time.monotonic() if now is None else now
        try:
            if len(names) != len(set(names)) or len(names) != len(positions):
                raise ValueError('关节反馈格式错误')
            q = vector([positions[names.index(name)] for name in JOINTS], 6)
            if not math.isfinite(stamp) or stamp <= 0 or stamp < self.stamp:
                raise ValueError('关节反馈时间戳无效或倒退')
            if stamp > self.stamp:
                self.q, self.stamp, self.received = q, stamp, now
            self.error = ''
        except (ValueError, IndexError, TypeError) as exc:
            self.error = str(exc)

    def get(self, timeout, now=None, wall=None):
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        if self.error:
            raise ValueError(self.error)
        if self.q is None or now-self.received > timeout or not -.1 <= wall-self.stamp <= timeout:
            raise ValueError('关节反馈缺失或超时')
        return self.q.copy()


def validate_trajectory(msg, actual, model, config, now):
    """Validate future knots; Guard checks real tracking on sampled commands.

    Future positions can legitimately be far from the current measurement while
    remaining within the configured speed and horizon. They are not current
    tracking errors. Retain actual in this interface for existing callers.
    """
    if list(msg.joint_names) != JOINTS or not msg.points:
        raise ValueError('Servo 输出缺少完整关节')
    stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    if not now - config['command_timeout'] <= stamp <= now + .15:
        raise ValueError('Servo 输出时间戳超时')
    limits = joint_speed_limits(config)
    knot_limits = np.minimum(limits * 1.05, HARDWARE_JOINT_SPEED_LIMITS)
    segment_limits = np.minimum(limits * 1.2, HARDWARE_JOINT_SPEED_LIMITS)
    previous, previous_t = None, -1.
    for point in msg.points:
        q = model.check_joints(point.positions)
        t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if not 0 <= t <= config['trajectory_horizon'] or t <= previous_t:
            raise ValueError('Servo 轨迹时间无效')
        if point.velocities and np.any(np.abs(vector(point.velocities, 6)) > knot_limits):
            raise ValueError('关节速度越限')
        if previous is not None:
            speeds = np.abs(q-previous)/(t-previous_t)
            if np.any(speeds > segment_limits):
                i = int(np.argmax(speeds/segment_limits))
                raise ValueError(f'相邻轨迹点速度越限 J{i+1}：{speeds[i]:.3f} rad/s，'
                                 f'允许 {segment_limits[i]:.3f} rad/s，点间隔 {(t-previous_t)*1000:.2f} ms')
        previous, previous_t = q, t


def validate_joint_plan(trajectory, actual, goal, model, config, speed_scale=1.0):
    """Check a complete p/v/a plan, including JTC quintic extrema; never edit it.

    The controller must be synchronized and stationary before execution. Requiring
    a zero-time first point avoids an implicit segment from an unknown controller
    state. Polynomial evaluation here is validation only, not command generation.
    """
    try:
        if not math.isfinite(speed_scale) or not 0 < speed_scale <= 1.:
            raise ValueError('规划速度比例无效')
        velocity_limits = joint_speed_limits(config, speed_scale)
        acceleration_limit = float(config['joint_acceleration']) * speed_scale
        max_duration = float(config.get('plan_max_duration', 120.))
        margin = float(config['joint_margin'])
        if (not all(math.isfinite(x) and x > 0 for x in
                    (acceleration_limit, max_duration)) or
                not math.isfinite(margin) or margin < 0):
            raise ValueError('规划限速或限位配置无效')
        names, points = list(trajectory.joint_names), trajectory.points
        if len(names) != 6 or set(names) != set(JOINTS) or len(points) < 2:
            raise ValueError('关节规划必须包含完整关节和至少两个轨迹点')
        order = [names.index(name) for name in JOINTS]
        actual, goal = model.check_joints(actual), model.check_joints(goal)
        knots = []
        for point in points:
            q = vector(point.positions, 6)[order]
            velocity = vector(point.velocities, 6)[order]
            acceleration = vector(point.accelerations, 6)[order]
            model.check_joints(q)
            duration = point.time_from_start
            if (not all(isinstance(x, (int, np.integer)) and not isinstance(x, bool)
                        for x in (duration.sec, duration.nanosec)) or
                    duration.sec < 0 or not 0 <= duration.nanosec < 1_000_000_000):
                raise ValueError('关节规划轨迹时间格式无效')
            t = duration.sec + duration.nanosec * 1e-9
            if (not math.isfinite(t) or t > max_duration or (knots and t <= knots[-1][0])):
                raise ValueError('关节规划轨迹时间无效或超出时长限制')
            if np.any(np.abs(velocity) > velocity_limits + 1e-8):
                raise ValueError('关节规划轨迹点速度越限')
            if np.max(np.abs(acceleration)) > acceleration_limit + 1e-8:
                raise ValueError('关节规划轨迹点加速度越限')
            knots.append((t, q, velocity, acceleration))
        if knots[0][0] > 1e-9:
            raise ValueError('关节规划首点必须从零时刻开始')
        if np.max(np.abs(knots[0][1] - actual)) > .005:
            raise ValueError('关节规划起点与实际反馈不一致')
        if np.max(np.abs(knots[-1][1] - goal)) > .003:
            raise ValueError('关节规划终点未到达目标')
        model.check_joints(knots[-1][1], margin)
        # Pinned MoveIt 2.12.4 TOTG getVelocity(0) uses the entire first
        # integration interval (DEFAULT_TIMESTEP=1 ms), not time-previous.time.
        # It therefore reports the interval-end velocity at the zero-time knot.
        # Permit at most one integration step of the scaled acceleration cap;
        # retain the supplied derivatives and check their full quintic below.
        # https://github.com/moveit/moveit2/blob/2.12.4/moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp#L820-L833
        if np.max(np.abs(knots[0][2])) > acceleration_limit * .001 + 1e-6:
            raise ValueError('关节规划首速度超出 TOTG 首段容差')
        if np.max(np.abs(knots[-1][2])) > 1e-6:
            raise ValueError('关节规划末速度必须为零')

        def extrema(coefficients):
            derivative = np.polynomial.polynomial.polyder(coefficients)
            roots = np.polynomial.polynomial.polyroots(derivative)
            times = [0., 1.]
            for root in roots:
                if abs(root.imag) <= 1e-7 and -1e-9 <= root.real <= 1. + 1e-9:
                    times.append(float(np.clip(root.real, 0., 1.)))
            values = np.polynomial.polynomial.polyval(times, coefficients)
            if not np.isfinite(values).all():
                raise ValueError('关节规划多项式数值无效')
            return np.min(values), np.max(values)

        for (t0, q0, v0, a0), (t1, q1, v1, a1) in zip(knots, knots[1:]):
            dt = t1 - t0
            if np.any(np.abs((q1 - q0) / dt) > velocity_limits + 1e-8):
                raise ValueError('关节规划相邻点隐含速度越限')
            # Quintic Hermite coefficients in normalized time s=(t-t0)/dt.
            c0, c1, c2 = q0, dt * v0, .5 * dt * dt * a0
            position_error = q1 - c0 - c1 - c2
            velocity_error = dt * v1 - c1 - 2. * c2
            acceleration_error = dt * dt * a1 - 2. * c2
            coefficients = np.stack((c0, c1, c2,
                10. * position_error - 4. * velocity_error + .5 * acceleration_error,
                -15. * position_error + 7. * velocity_error - acceleration_error,
                6. * position_error - 3. * velocity_error + .5 * acceleration_error))
            if not np.isfinite(coefficients).all():
                raise ValueError('关节规划多项式数值无效')
            for joint, polynomial in enumerate(coefficients.T):
                lower, upper = extrema(polynomial)
                if lower < model.lower[joint] - 1e-9 or upper > model.upper[joint] + 1e-9:
                    raise ValueError('关节规划多项式位置超出模型限位')
                velocity = np.polynomial.polynomial.polyder(polynomial) / dt
                if max(map(abs, extrema(velocity))) > velocity_limits[joint] + 1e-8:
                    raise ValueError('关节规划多项式速度越限')
                acceleration = np.polynomial.polynomial.polyder(velocity) / dt
                if max(map(abs, extrema(acceleration))) > acceleration_limit + 1e-8:
                    raise ValueError('关节规划多项式加速度越限')
        return float(knots[-1][0])
    except (AttributeError, TypeError, IndexError, KeyError, OverflowError, np.linalg.LinAlgError) as exc:
        raise ValueError('关节规划格式或数值无效') from exc


def load_config(path, control_path=None):
    import yaml
    path = Path(path).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError('任务配置必须是映射')
    selection = control_path if control_path not in (None, '') else data.get('control_config')
    explicit_control = selection not in (None, '')
    if explicit_control:
        if not isinstance(selection, (str, Path)):
            raise ValueError('control_config 必须是控制参数文件路径')
        selected = Path(selection).expanduser()
        selected = selected if selected.is_absolute() else path.parent/selected
    else:
        adjacent = path.with_name('control.yaml')
        selected = adjacent if adjacent.exists() else Path(__file__).with_name('config')/'control.yaml'
    selected = selected.resolve()
    control = load_control_config(selected)
    flat_settings = ('linear_speed', 'angular_speed', 'position_tolerance', 'orientation_tolerance')
    flat_gains = ('position_gain', 'orientation_gain')
    legacy = False
    if not explicit_control:
        for key in flat_settings:
            if key in data:
                control[key], legacy = data[key], True
        for key in flat_gains:
            if key in data:
                control['gains'][control['algorithm']][key], legacy = data[key], True
        control = validate_control_config(control)
    data['control'], data['control_config'] = control, str(selected)
    # Legacy task files remain loadable, but this retired setting has no effect.
    data.pop('max_joint_tracking_error', None)
    # Launch materializes this already-validated effective configuration only
    # when legacy flat overrides would otherwise be lost at the Node boundary.
    data['control_legacy_overrides'] = legacy
    data.update({key: control[key] for key in flat_settings})
    data.update({key: control['gains'][control['algorithm']][key] for key in flat_gains})
    for key, value in {'ready_joints': [0., 1.4, -1.2, .3, 1., 0.],
                       'prepare_on_singular_start': True, 'planning_time': 5.,
                       'plan_max_duration': 120., 'plan_settle_timeout': 5.,
                       'controller_sync_settle_time': .08,
                       'dls_damping': .02, 'dls_rotation_weight': .25,
                       'joint_velocity_limits': HARDWARE_JOINT_SPEED_LIMITS.tolist()}.items():
        data.setdefault(key, value)
    _dls_parameters(data)
    settle = data['controller_sync_settle_time']
    if (isinstance(settle, bool) or not isinstance(settle, (int, float)) or
            not math.isfinite(settle) or not .08 <= settle <= 1.):
        raise ValueError('controller_sync_settle_time 必须是 0.08 到 1 秒之间的有限数值')
    data['zero_feedback_tolerance'] = _zero_feedback_tolerance(
        data.get('zero_feedback_tolerance', MAX_ZERO_FEEDBACK_TOLERANCE))
    for key in ('rate', 'joint_speed', 'joint_acceleration', 'feedback_timeout', 'command_timeout',
                'trajectory_horizon',
                'ik_timeout', 'workspace_resolution', 'workspace_samples',
                'planning_time', 'plan_max_duration', 'plan_settle_timeout'):
        if not isinstance(data[key], (float, int)) or not math.isfinite(data[key]) or data[key] <= 0:
            raise ValueError(f'配置 {key} 必须是有限正数')
    hold_defaults = {
        'hold_position_tolerance': max(.003, data['position_tolerance']),
        'hold_orientation_tolerance': max(.02, data['orientation_tolerance']),
        'hold_joint_stability': .003,
        'hold_settle_time': .3,
    }
    for key, default in hold_defaults.items():
        value = data.setdefault(key, default)
        if (isinstance(value, bool) or not isinstance(value, (float, int)) or
                not math.isfinite(value) or value <= 0):
            raise ValueError(f'配置 {key} 必须是有限正数')
    if (data['hold_position_tolerance'] < data['position_tolerance'] or
            data['hold_orientation_tolerance'] < data['orientation_tolerance']):
        raise ValueError('保持位姿容差不能小于跟踪位姿容差')
    # Old task variants remain loadable; target displacement is no longer a fault limit.
    data.pop('max_position_error', None)
    data.pop('max_orientation_error', None)
    data.setdefault('lower_singularity_threshold', 17.)
    data.setdefault('hard_stop_singularity_threshold', 30.)
    low, hard = data['lower_singularity_threshold'], data['hard_stop_singularity_threshold']
    if not isinstance(low, (int, float)) or not isinstance(hard, (int, float)) or not 0 < low < hard < float('inf'):
        raise ValueError('奇异位形阈值必须有限且满足 0 < lower < hard_stop')
    if not 20 <= data['rate'] <= 200 or not 0 < data['default_speed_scale'] <= 1:
        raise ValueError('控制频率或速度比例无效')
    if not 0 < data['joint_margin'] < .2 or not 0 < data['workspace_alpha'] < 1:
        raise ValueError('限位余量或显示透明度无效')
    if not isinstance(data['fast_mode'], bool):
        raise ValueError('fast_mode 必须是布尔值')
    if not isinstance(data['prepare_on_singular_start'], bool):
        raise ValueError('prepare_on_singular_start 必须是布尔值')
    if any(isinstance(data[key], bool) for key in ('planning_time', 'plan_max_duration', 'plan_settle_timeout')):
        raise ValueError('规划时间配置必须是有限正数')
    if not isinstance(data['can_port'], str) or not data['can_port'].replace('_', '').replace('-', '').isalnum():
        raise ValueError('无效 CAN 接口名称')
    if not 1000 <= data['workspace_samples'] <= 500000 or not .01 <= data['workspace_resolution'] <= .1:
        raise ValueError('工作空间采样数量或分辨率无效')
    vector(data['initial_joints'], 6)
    vector(data['ready_joints'], 6)
    vector(data['tcp_offset'], 6)
    joint_speed_limits(data)
    return data
