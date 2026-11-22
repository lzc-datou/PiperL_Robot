import copy
import tempfile
import time
import unittest
from pathlib import Path
import yaml
from scripts.task.common.pose import pose_errors
from scripts.task.common.schema import quaternion, validate_config, load_scene, DEFAULTS, JOINTS
from scripts.task.common.robot_feedback import StableWindow, Sample
from scripts.tools.monitoring.read_joint_angles import JointFeedback
from scripts.tools.monitoring.point_snapshot import Capture, documents, save_snapshot, point_text

class ConfigSnapshotTests(unittest.TestCase):
    def point(self): return dict(name='p1',type='joint',joints_rad=[0.0]*6)
    def test_joint_and_pose_formats(self):
        result=validate_config({'points':[self.point(),
            dict(name='p2',type='pose',position_m=[0.1,0.2,0.3],orientation_xyzw=[0,0,0,1])]})
        self.assertEqual(result['settings']['velocity_scaling'],0.1)
        self.assertEqual(result['points'][1]['frame_id'],'base_link')
    def test_mixed_rpy_and_quaternion_use_fixed_axis_rotation(self):
        quat = dict(name='quat', type='pose', position_m=[.1, -.2, .3],
                    orientation_xyzw=[.5, .5, -.5, .5])
        rpy = dict(name='rpy', type='pose', position_m=[.1, -.2, .3], rpy=[90, 90, 0])
        document = {'points': [self.point(), quat, rpy]}
        joint, quat_goal, rpy_goal = validate_config(document)['points']
        self.assertEqual(joint['joints_rad'], [0.] * 6)
        self.assertEqual(quat_goal['position_m'], rpy_goal['position_m'])
        for actual, expected in zip(rpy_goal['orientation_xyzw'], quat_goal['orientation_xyzw']):
            self.assertAlmostEqual(actual, expected, places=14)
        self.assertNotIn('rpy', rpy_goal)
        self.assertIn('rpy', document['points'][-1])

    def test_rounded_monitor_rpy_keeps_supplied_quaternion_even_with_opposite_sign(self):
        values = [.302392, .920589, .028120, .245529]
        for sign in (-1, 1):
            xyzw = [sign * value for value in values]
            point = dict(name='copied', type='pose', position_m=[.263916, .205623, .448602],
                         orientation_xyzw=xyzw, rpy=[167.1489, 25.7889, 140.6780])
            goal = validate_config({'points': [point]})['points'][0]
            self.assertEqual(goal['orientation_xyzw'], quaternion(xyzw))
            self.assertNotIn('rpy', goal)

    def test_orientation_conflict_reports_the_point_and_both_fields(self):
        point = dict(name='conflicting_target', type='pose', position_m=[.1, -.2, .3],
                     orientation_xyzw=[0, 0, 0, 1], rpy=[0, 0, .002])
        with self.assertRaisesRegex(ValueError, 'conflicting_target: orientation_xyzw and rpy disagree'):
            validate_config({'points': [point]})

    def test_invalid_pose_is_rejected_before_planning(self):
        base = dict(name='target', type='pose', position_m=[0] * 3, rpy=[0] * 3)
        for updates in (dict(rpy=[0] * 2), dict(rpy=[0, 0, float('inf')]), dict(rpy=[0, 0, 10**400]),
                        dict(rpy=[0, 0, True]), dict(frame_id='other'),
                        dict(position_m=None), dict(orientation_xyzw=[0, 0, 0, 0]),
                        dict(position_mm=[0] * 3), dict(xyzrpy=[0] * 6),
                        dict(orientation='initial'), dict(type='joint', joints_rad=[0] * 6)):
            with self.subTest(updates=updates), self.assertRaises(ValueError):
                validate_config({'points': [dict(base, **updates)]})
        for fields in ({'position_m': [0] * 3}, {'xyzrpy': [0] * 6}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                validate_config({'points': [dict(name='target', type='pose', **fields)]})

    def test_invalid_inputs_rejected(self):
        for doc in ({'points':[]},{'points':[self.point(),self.point()]},
                    {'points':[dict(name='../bad',type='joint',joints_rad=[0]*6)]},
                    {'points':[dict(name='bad',type='joint',joints_rad=[0]*5)]},
                    {'points':[dict(name='bad',type='joint',joints_rad=[float('nan')]*6)]},
                    {'settings':{'velocity_scaling':0},'points':[self.point()]},
                    {'settings':{'return_to_initial':'false'},'points':[self.point()]},
                    {'settings':{'unknown':5},'points':[self.point()]},
                    {'points':[dict(name='bad',type='pose',position_m=[0]*3,orientation_xyzw=[0]*4)]}):
            with self.subTest(doc=doc),self.assertRaises(ValueError): validate_config(doc)
    def test_schema_does_not_mutate_input(self):
        original={'points':[self.point()]}
        saved=copy.deepcopy(original)
        validate_config(original)
        self.assertEqual(original,saved)
    def feedback(self):
        data=JointFeedback()
        data.degrees=[0.0,10.0,-20.0,30.0,0.0,5.0]
        data.received_at=[10.0,10.0,10.001,10.001,10.002,10.002]
        return data
    def test_snapshot_import_and_copy_round_trip(self):
        capture=Capture.from_feedback(self.feedback(),10.01)
        pose=dict(frame_id='base_link',position_m=[0.1,0.2,0.3],orientation_xyzw=[.5,.5,-.5,.5])
        records=documents(capture,pose,model_sha256='abc')
        self.assertEqual(set(records), {'joint', 'pose'})
        for kind, record in records.items():
            self.assertEqual(len(validate_config(record)['points']),1)
            point=yaml.safe_load(point_text(record))[0]
            self.assertEqual(records['joint']['capture'], record['capture'])
            parsed = validate_config({'points':[point]})['points'][0]
            if kind != 'joint':
                self.assertIn('orientation_xyzw', point)
                self.assertIn('rpy', point)
                distance, angle = pose_errors(parsed, pose)
                self.assertLess(distance, 1e-6)
                self.assertLess(angle, 2e-6)
        with tempfile.TemporaryDirectory() as directory:
            saved=save_snapshot(directory,records)
            self.assertEqual({path.name for path in saved.iterdir()}, {'joint.yaml', 'pose.yaml'})
            for kind in records:
                saved_record = yaml.safe_load((saved/(kind+'.yaml')).read_text())
                self.assertEqual(saved_record, records[kind])
                validate_config(saved_record)
    def test_capture_is_immutable_and_uses_original_joint_sample(self):
        data=self.feedback()
        capture=Capture.from_feedback(data,10.01)
        data.degrees[0]=50
        self.assertEqual(capture.q[0],0)
    def test_invalid_capture_never_exported(self):
        for mode in ('missing','stale','skew','nan'):
            data=self.feedback()
            if mode=='missing': data.received_at[0]=None
            if mode=='stale': data.received_at[0]=9
            if mode=='skew': data.received_at[0]=9.9
            if mode=='nan': data.degrees[0]=float('nan')
            with self.subTest(mode=mode),self.assertRaises(ValueError): Capture.from_feedback(data,10.01)
    def test_metadata_and_frame_mismatch_rejected(self):
        records=documents(Capture.from_feedback(self.feedback(),10.01))
        record=records['joint']
        record['settings']['tcp_link']='wrong_link'
        with self.assertRaises(ValueError): validate_config(record)
    def test_old_stamp_never_proves_stability(self):
        window=StableWindow(0.3,0.003)
        sample=Sample((0,)*6,1,1)
        for now in (1,2,3): self.assertFalse(window.update(sample,now))
    def test_stability_needs_fresh_samples_and_target(self):
        window=StableWindow(0.3,0.003)
        target=(0,)*6
        for index,t in enumerate((0,0.1,0.2,0.31)):
            ok=window.update(Sample(target,index+1,t),t,target,0.02)
        self.assertTrue(ok)
        self.assertFalse(window.update(Sample((0.1,)*6,5,0.4),0.4,target,0.02))
    def test_scene_shape_units_and_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'scene.yaml'
            scene={'enabled':True,'objects':[dict(id='a',shape='cylinder',dimensions_m=[0.3,0.1],position_m=[0.3,0,0.2])]}
            path.write_text(yaml.safe_dump(scene))
            result=load_scene(path,'base_link')
            self.assertEqual(result['objects'][0]['dimensions_m'],[0.3,0.1])
            scene['objects'][0]['dimensions_m']=[-1,0.1]
            path.write_text(yaml.safe_dump(scene))
            with self.assertRaises(ValueError): load_scene(path,'base_link')

if __name__=='__main__': unittest.main()
