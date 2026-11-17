import math
from pathlib import Path
import unittest
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
from scripts.task.servo_tracking.core import (JOINTS, HARDWARE_JOINT_SPEED_LIMITS, joint_speed_limits,
    RobotModel, FreshJoints, load_config,
    pose_error, normalized_pose, tracking_twist, validate_trajectory, validate_arm_mode, workspace_surface)

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT/'scripts/task/servo_tracking/config/config.yaml'
URDF = ROOT/'piperl_ws/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper_l/urdf/piper_l_description.urdf'


class ServoCoreTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(CONFIG)
        # These regression fixtures exercise the original conservative task caps.
        self.config.update(joint_speed=.3, joint_acceleration=.5)
        self.model = RobotModel(URDF.read_text(), tip='link6')
        self.q = np.asarray(self.config['initial_joints'])

    def test_quaternion_sign_and_wrap(self):
        a = np.r_[[0.,0.,0.], Rotation.from_euler('z', 179, degrees=True).as_quat()]
        b = np.r_[[0.,0.,0.], Rotation.from_euler('z', -179, degrees=True).as_quat()]
        self.assertAlmostEqual(np.linalg.norm(pose_error(a,b)[1]), math.radians(2))
        self.assertLess(np.linalg.norm(pose_error(a, np.r_[a[:3], -a[3:]])[1]), 1e-10)

    def test_invalid_pose_rejected(self):
        for p in ([0]*7, [float('nan')]*7, [0]*6):
            with self.assertRaises(ValueError): normalized_pose(p)

    def test_full_six_axis_velocity_caps(self):
        actual = self.model.pose(self.q)
        target = actual.copy()
        target[:3] += [.1,.1,.1]
        # Stay above the configured angular cap before clipping on all axes.
        target[3:] = (Rotation.from_rotvec([.6,.4,.2])*Rotation.from_quat(actual[3:])).as_quat()
        command = tracking_twist(target, actual, self.config, .5)
        self.assertTrue(np.all(np.abs(command)>0))
        self.assertAlmostEqual(np.linalg.norm(command[:3]), self.config['linear_speed']*.5)
        self.assertAlmostEqual(np.linalg.norm(command[3:]), self.config['angular_speed']*.5)
        self.assertTrue(np.all(tracking_twist(actual, actual, self.config, 1.) == 0))

    def test_piper_l_axis_speed_ceilings_and_nominal_clamping(self):
        np.testing.assert_allclose(np.rad2deg(HARDWARE_JOINT_SPEED_LIMITS),
                                   [180., 195., 180., 225., 225., 225.])
        self.assertAlmostEqual(HARDWARE_JOINT_SPEED_LIMITS[1], math.radians(195.))
        np.testing.assert_allclose(joint_speed_limits({'joint_speed': 10.}), HARDWARE_JOINT_SPEED_LIMITS)
        np.testing.assert_allclose(joint_speed_limits({'joint_speed': 10.}, .5),
                                   HARDWARE_JOINT_SPEED_LIMITS * .5)
        config = dict(joint_speed=3.5, joint_velocity_limits=[1., 2., 1.1, 3.5, 3.6, 3.7])
        np.testing.assert_allclose(joint_speed_limits(config), [1., 2., 1.1, 3.5, 3.5, 3.5])
        np.testing.assert_allclose(joint_speed_limits({'joint_speed': 1.5}), [1.5] * 6)

    def test_republished_feedback_cannot_refresh_age(self):
        f=FreshJoints()
        f.update(JOINTS, self.q, 100., now=10.)
        f.update(JOINTS, self.q, 100., now=11.)
        with self.assertRaises(ValueError): f.get(.25, now=11.01, wall=100.1)

    def test_feedback_order_and_bad_stamps(self):
        f=FreshJoints()
        f.update(JOINTS[::-1], self.q[::-1], 100., now=10.)
        np.testing.assert_allclose(f.get(.25, now=10.1, wall=100.1), self.q)
        f.update(JOINTS, self.q, 99., now=10.2)
        with self.assertRaises(ValueError): f.get(.25, now=10.2, wall=100.2)

    def test_urdf_limits_not_relaxed(self):
        q=self.q.copy();q[1]=-.001
        with self.assertRaises(ValueError): self.model.check_joints(q)
        self.assertEqual(self.model.lower[1],0.)

    def test_base_tcp_jacobian_matches_pose_finite_difference(self):
        # An offset and rotated TCP exercises the point/frame convention too.
        urdf = URDF.read_text().replace('</robot>', '''
            <link name="jacobian_test_tcp"/>
            <joint name="jacobian_test_mount" type="fixed">
                <parent link="link6"/><child link="jacobian_test_tcp"/>
                <origin xyz="0.07 -0.03 0.05" rpy="0.2 -0.3 0.4"/>
            </joint>
        </robot>''')
        model = RobotModel(urdf, tip='jacobian_test_tcp')
        step = 1e-6
        for q in (self.q, self.q + [.2, -.1, .15, -.3, .2, .4]):
            difference = np.zeros((6, 6))
            for i in range(6):
                delta = np.zeros(6); delta[i] = step
                plus, minus = model.pose(q + delta), model.pose(q - delta)
                difference[:3, i] = (plus[:3] - minus[:3]) / (2 * step)
                difference[3:, i] = (Rotation.from_quat(plus[3:]) *
                                     Rotation.from_quat(minus[3:]).inv()).as_rotvec() / (2 * step)
            np.testing.assert_allclose(model.jacobian(q), difference, rtol=1e-7, atol=1e-9)

    def test_jacobian_condition_reports_singular_and_regular_poses(self):
        self.assertIsInstance(self.model.condition_number(self.q), float)
        self.assertLess(self.model.condition_number(self.q), 17.)
        # The extended zero pose and aligned wrist both lose a motion dimension.
        for q in (np.zeros(6), [0., 1.4, -1.2, 0., 0., 0.]):
            self.assertTrue(math.isinf(self.model.condition_number(q)))
        near = self.q.copy(); near[4] = 1e-5
        self.assertGreater(self.model.condition_number(near), 1e4)
        with self.assertRaises(ValueError):
            self.model.jacobian([0.] * 5)
        with self.assertRaises(ValueError):
            self.model.condition_number([float('nan')] * 6)

    def test_teach_modes_rejected_even_when_diagnostic_is_ok(self):
        validate_arm_mode({'arm_status':'0','ctrl_mode':'1','teach_status':'0'})
        for mode in ('2','3','6','7'):
            with self.assertRaises(ValueError):
                validate_arm_mode({'arm_status':'0','ctrl_mode':mode,'teach_status':'0'})
        for status in ('11','12','13'):
            with self.assertRaises(ValueError):
                validate_arm_mode({'arm_status':status,'ctrl_mode':'1','teach_status':'0'})

    def test_workspace_surface_is_finite_bounded_and_nondegenerate(self):
        mesh,resolution=self.model.workspace_surface(32768,.04)
        self.assertEqual(mesh.shape[1:],(3,3))
        self.assertTrue(np.isfinite(mesh).all())
        self.assertLessEqual(len(mesh),20000)
        area=np.linalg.norm(np.cross(mesh[:,1]-mesh[:,0],mesh[:,2]-mesh[:,0]),axis=1)
        self.assertTrue(np.all(area>0))
        self.assertTrue(np.all(np.linalg.norm(mesh,axis=2)<=self.model.radius+2*resolution))

    def test_surface_preserves_large_internal_hole_and_render_budget(self):
        # A toroidal sampled volume must keep both its outer and inner boundary.
        grid=np.stack(np.meshgrid(*([np.arange(-.6,.61,.025)]*3),indexing='ij'),axis=-1).reshape(-1,3)
        radius=np.linalg.norm(grid[:,:2],axis=1)
        points=grid[(radius-.4)**2+grid[:,2]**2<.13**2]
        mesh,resolution=workspace_surface(points,.025,max_faces=4000)
        radial=np.linalg.norm(mesh[:,:,:2],axis=2)
        self.assertGreater(radial.min(),.2)
        self.assertLess(radial.min(),.32)
        self.assertGreater(radial.max(),.48)
        self.assertLessEqual(len(mesh),4000)

    def trajectory(self):
        return NS(joint_names=JOINTS, header=NS(stamp=NS(sec=100,nanosec=0)), points=[
            NS(positions=self.q.tolist(), velocities=[0.]*6, time_from_start=NS(sec=0,nanosec=20_000_000))])

    def test_guard_rejects_old_partial_and_out_of_bounds_trajectories(self):
        valid=self.trajectory()
        validate_trajectory(valid,self.q,self.model,self.config,100.01)
        with self.assertRaises(ValueError):validate_trajectory(valid,self.q,self.model,self.config,101.)
        valid.joint_names=JOINTS[:-1]
        with self.assertRaises(ValueError):validate_trajectory(valid,self.q,self.model,self.config,100.01)
        valid=self.trajectory();valid.points[0].positions[0]=self.model.upper[0]+.001
        with self.assertRaises(ValueError):validate_trajectory(valid,self.q,self.model,self.config,100.01)

    def test_guard_validates_every_knot(self):
        msg=self.trajectory()
        p=NS(positions=self.q.tolist(),velocities=[0.]*6,time_from_start=NS(sec=0,nanosec=30_000_000))
        p.positions[0]+=.05
        msg.points.append(p)
        with self.assertRaises(ValueError):validate_trajectory(msg,self.q,self.model,self.config,100.01)

    def test_servo_knot_velocity_uses_the_correct_hardware_axis_ceiling(self):
        config = dict(self.config, joint_speed=10.)
        for joint, velocity, allowed in ((0, 3.2, False), (1, 3.2, True),
                                         (1, 3.8, False), (3, 3.8, True)):
            msg = self.trajectory(); msg.points[0].velocities[joint] = velocity
            with self.subTest(joint=joint + 1, velocity=velocity):
                if allowed:
                    validate_trajectory(msg, self.q, self.model, config, 100.01)
                else:
                    with self.assertRaisesRegex(ValueError, '关节速度'):
                        validate_trajectory(msg, self.q, self.model, config, 100.01)

    def test_servo_implied_speed_uses_the_correct_hardware_axis_ceiling(self):
        config = dict(self.config, joint_speed=10.)
        for joint, velocity, allowed in ((0, 3.3, False), (1, 3.3, True),
                                         (1, 3.8, False), (3, 3.8, True)):
            msg = self.trajectory()
            positions = self.q.copy(); positions[joint] += velocity * .01
            msg.points.append(NS(positions=positions.tolist(), velocities=[0.] * 6,
                                 time_from_start=NS(sec=0, nanosec=30_000_000)))
            with self.subTest(joint=joint + 1, velocity=velocity):
                if allowed:
                    validate_trajectory(msg, self.q, self.model, config, 100.01)
                else:
                    with self.assertRaisesRegex(ValueError, '相邻轨迹点速度'):
                        validate_trajectory(msg, self.q, self.model, config, 100.01)


if __name__=='__main__':unittest.main()
