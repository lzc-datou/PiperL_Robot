"""Scene drafts, saved files and exact preview selection; no ROS nodes or hardware."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import yaml

try:
    from moveit_msgs.msg import PlanningScene
    from scripts.task.common.planning_scene import obstacle_message
    from scripts.task.common.schema import DEFAULTS, validate_scene
    from scripts.task.multi_point.scene_session import SceneSession
    from scripts.task.obstacle_avoidance.control_session import ObstacleControlSession
    from tests import test_task_ros_messages as helpers
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Use ROS runtime wrapper')
class SceneEditorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.client = helpers.TaskMessageTests().client()
        self.client.settings = dict(DEFAULTS)
        self.client.activity_generation = 0
        self.client.cancel_requested = threading.Event()
        self.client.begin_round = Mock()
        self.client.connect = Mock()
        self.client.clear_preview = Mock()
        self.state = PlanningScene()
        def apply(scene):
            if scene['enabled']:
                self.state.world.collision_objects = [obstacle_message(o) for o in scene['objects']]
            return deepcopy(self.state)
        self.client.scene = SimpleNamespace(apply=Mock(side_effect=apply), get=lambda:deepcopy(self.state))
        self.path = self.root/'points.yaml'
        self.path.write_text(yaml.safe_dump({'points':[
            dict(name='A',type='joint',joints_rad=[.1]*6), dict(name='B',type='joint',joints_rad=[.2]*6)]}))
        self.object = dict(id='box',shape='box',dimensions_m=[.1,.2,.3],position_m=[.3,.1,.4],
                           orientation_xyzw=[0.,0.,0.,1.],frame_id='base_link')
        self.scene_file = self.root/'scene.yaml'
        self.scene_file.write_text(yaml.safe_dump(dict(enabled=True,objects=[self.object])))
        self.session = ObstacleControlSession(self.client, self.root/'runs', self.scene_file)
        self.command('load',self.path)

    def tearDown(self):
        self.session.close()
        self.temp.cleanup()

    def command(self, name, payload=''):
        accepted, reason = self.session.submit(name, str(payload))
        self.assertTrue(accepted,reason)
        self.session.jobs.join()
        return self.session.status()

    def test_draft_drag_apply_and_save_roundtrip_preserves_original_file(self):
        original = self.scene_file.read_bytes()
        self.command('plan')
        self.assertTrue(self.session.status()['can_execute'])
        self.command('scene_edit','true')
        self.assertFalse(self.session.status()['can_execute'])
        move = dict(id='box',position_m=[.4,.2,.5],orientation_xyzw=[0.,0.,1.,0.])
        self.command('scene_pose',json.dumps(move))
        self.assertTrue(self.session.scene_dirty)
        # Draft is not collision geometry until explicitly applied.
        self.assertAlmostEqual(self.state.world.collision_objects[0].pose.position.x,.3)
        self.assertFalse(self.session.submit('plan')[0])
        self.assertFalse(self.session.submit('execute')[0])
        self.command('scene_apply')
        self.assertFalse(self.session.scene_dirty)
        self.assertFalse(self.session.scene_editing)
        self.assertAlmostEqual(self.state.world.collision_objects[0].pose.position.x,.4)
        target=self.root/'scene_new.yaml'
        self.command('scene_save',target)
        self.assertEqual(self.scene_file.read_bytes(),original)
        self.assertEqual(yaml.safe_load(target.read_text()),self.session.applied_scene)
        self.assertEqual(self.command('scene_save',target)['state'],'ERROR')
        self.assertEqual(yaml.safe_load(target.read_text()),self.session.applied_scene)
        self.command('scene_load',target)
        self.assertTrue(self.session.scene_dirty)
        self.assertEqual(self.client.executions,0)

    def test_selected_preview_executes_exact_cached_trajectory_for_either_mode(self):
        for mode in ('baseline','avoidance'):
            with self.subTest(mode=mode):
                self.client.q=(0.,)*6
                self.command('plan')
                self.assertEqual(self.session.status()['state'],'READY')
                self.assertEqual(set(self.session.cached),{'baseline','avoidance'})
                expected=deepcopy(self.session.cached[mode][0]['segments'])
                planned_count=len(self.client.starts)
                self.command('preview_'+mode)
                self.assertEqual(len(self.state.world.collision_objects),0 if mode=='baseline' else 1)
                self.assertTrue(self.session.status()['can_execute'])
                self.command('execute')
                self.assertEqual(len(self.client.starts),planned_count)  # No replanning at execution.
                self.assertEqual(self.client.sent[-2:],[s['trajectory'] for s in expected])
                self.assertFalse(self.session.status()['can_execute'])
                self.assertEqual(self.session.cached,{})

    def test_independent_planning_requires_matching_preview_before_execution(self):
        publish=Mock(wraps=self.client.publish_plan)
        self.client.publish_plan=publish
        for mode in ('baseline','avoidance'):
            status=self.command('plan_'+mode)
            self.assertEqual(status['state'],'PLANNED')
            self.assertFalse(status['preview_visible'])
            self.assertFalse(status['can_execute_'+mode])
            self.assertFalse(self.session.submit('execute_'+mode)[0])
            publish.assert_not_called()
            self.command('preview_'+mode)
            self.assertTrue(self.session.status()['can_execute_'+mode])
            publish.reset_mock()
        self.assertEqual(len(self.client.starts),4)
        self.assertTrue(self.session.status()['can_execute_baseline'])
        self.assertTrue(self.session.status()['can_execute_avoidance'])
        expected=deepcopy(self.session.cached['baseline'][0]['segments'])
        self.command('execute_baseline')  # Explicitly select baseline even after viewing avoidance.
        self.assertEqual(len(self.client.starts),4)
        self.assertEqual(self.client.sent[-2:],[s['trajectory'] for s in expected])
        self.assertEqual(len(self.state.world.collision_objects),0)
        self.assertEqual(self.session.cached,{})
        self.assertFalse(self.session.status()['can_execute_avoidance'])

    def test_failed_other_mode_preserves_valid_preview_and_manual_clears_it(self):
        self.command('plan_baseline');self.command('preview_baseline')
        with patch.object(self.client,'plan',side_effect=RuntimeError('no path')):
            self.assertEqual(self.command('plan_avoidance')['state'],'ERROR')
        self.assertEqual(set(self.session.cached),{'baseline'})
        self.assertTrue(self.session.status()['can_execute_baseline'])
        self.assertEqual(len(self.state.world.collision_objects),1)
        self.command('manual')
        self.assertEqual(self.session.cached,{})
        self.assertFalse(self.session.submit('execute_baseline')[0])

    def test_independent_cache_invalidated_by_config_change_or_motion(self):
        self.command('plan_baseline');self.command('preview_baseline')
        config=yaml.safe_load(self.path.read_text())
        config['points'][0]['joints_rad']=[.12]*6
        self.path.write_text(yaml.safe_dump(config))
        self.command('plan_avoidance')
        self.assertNotIn('baseline',self.session.cached)
        self.command('preview_avoidance')
        self.client.q=(.05,)*6
        status=self.command('execute_avoidance')
        self.assertEqual(status['state'],'ERROR')
        self.assertEqual(self.client.executions,0)
        self.assertFalse(self.session.cached)

    def test_independent_execution_rejects_editing_and_closed_gate(self):
        self.command('plan_avoidance');self.command('preview_avoidance')
        self.client.task_gate_closed=True
        self.assertFalse(self.session.submit('execute_avoidance')[0])
        self.assertFalse(self.session.status()['can_execute_avoidance'])
        self.client.task_gate_closed=False
        self.command('scene_edit','true')
        for command in ('plan_baseline','plan_avoidance','preview_avoidance','execute_avoidance'):
            self.assertFalse(self.session.submit(command)[0])

    def test_stale_start_prevents_selecting_any_cached_preview(self):
        self.command('plan')
        self.client.q=(.05,)*6
        self.assertEqual(self.command('preview_baseline')['state'],'ERROR')
        self.assertFalse(self.session.status()['can_execute'])
        self.assertEqual(self.client.executions,0)

    def test_apply_failure_keeps_dirty_draft_and_no_execution_confirmation(self):
        self.command('scene_upsert',json.dumps(dict(self.object,position_m=[.5,0.,.4])))
        self.client.scene.apply.side_effect=RuntimeError('rejected scene')
        self.assertEqual(self.command('scene_apply')['state'],'ERROR')
        self.assertTrue(self.session.scene_dirty)
        self.assertFalse(self.session.status()['can_execute'])

    def test_invalid_geometry_is_rejected_without_corrupting_draft(self):
        before=deepcopy(self.session.draft)
        self.assertEqual(self.command('scene_upsert',json.dumps(dict(self.object,dimensions_m=[-1,1,1])))['state'],'ERROR')
        self.assertEqual(self.session.draft,before)
        self.assertEqual(self.client.executions,0)

    def test_running_task_or_external_motion_rejects_scene_changes(self):
        with self.session.lock: self.session.busy=True
        self.assertFalse(self.session.submit('scene_delete','box')[0])
        with self.session.lock: self.session.busy=False
        self.client.begin_round.side_effect=RuntimeError('controller active')
        self.assertEqual(self.command('scene_delete','box')['state'],'ERROR')
        self.assertEqual(len(self.session.draft['objects']),1)

    def test_applied_unsaved_scene_survives_replanning_and_manual_control(self):
        self.command('scene_upsert',json.dumps(dict(self.object,position_m=[.5,0.,.4])))
        self.command('scene_apply')
        self.command('plan')
        self.assertAlmostEqual(self.session.applied_scene['objects'][0]['position_m'][0],.5)
        self.command('manual')
        self.assertFalse(self.session.status()['can_execute'])
        self.assertFalse(self.session.status()['preview_visible'])
        self.assertEqual(self.client.executions,0)
        self.assertEqual(self.client.stops,0)

    def test_all_shapes_and_duplicate_ids_use_shared_validation(self):
        for shape,dimensions in [('box',[.1]*3),('sphere',[.1]),('cylinder',[.2,.1])]:
            obj=dict(self.object,shape=shape,dimensions_m=dimensions)
            self.command('scene_upsert',json.dumps(obj))
            self.command('scene_apply')
            self.assertEqual(self.session.applied_scene['objects'][0]['shape'],shape)
        with self.assertRaises(ValueError):
            validate_scene(dict(enabled=True,objects=[self.object,self.object]),'base_link')

    def test_interactive_handles_publish_geometry_and_commit_only_on_mouse_release(self):
        from scripts.task.multi_point.scene_markers import SceneMarkers
        from visualization_msgs.msg import InteractiveMarkerFeedback, Marker
        self.command('scene_edit','true')
        with patch('scripts.task.multi_point.scene_markers.InteractiveMarkerServer') as server:
            markers=SceneMarkers(Mock(),self.session)
            markers.refresh()
            marker=server.return_value.insert.call_args.args[0]
            self.assertEqual(marker.header.frame_id,'base_link')
            self.assertEqual(marker.name,'box')
            self.assertEqual(len(marker.controls),7)
            self.assertEqual(marker.controls[0].markers[0].type,Marker.CUBE)
            feedback=InteractiveMarkerFeedback(marker_name='box',control_name='move_x',event_type=InteractiveMarkerFeedback.POSE_UPDATE)
            feedback.pose=deepcopy(marker.pose);feedback.pose.position.x=.6
            markers.feedback(feedback)
            self.assertFalse(self.session.scene_dirty)
            feedback.event_type=InteractiveMarkerFeedback.MOUSE_UP
            markers.feedback(feedback);self.session.jobs.join()
            self.assertTrue(self.session.scene_dirty)
            self.assertAlmostEqual(self.session.draft['objects'][0]['position_m'][0],.6)
            self.command('scene_apply')
            server.return_value.insert.reset_mock()
            markers.refresh()
            server.return_value.insert.assert_not_called()
            markers.close();server.return_value.shutdown.assert_called_once()

    def test_selection_by_dropdown_and_body_click_only_gives_one_object_handles(self):
        from scripts.task.multi_point.scene_markers import SceneMarkers
        from visualization_msgs.msg import InteractiveMarkerFeedback,InteractiveMarkerControl
        sphere=dict(self.object,id='sphere',shape='sphere',dimensions_m=[.1])
        self.command('scene_upsert',json.dumps(sphere));self.command('scene_apply')
        self.command('scene_edit','true')
        self.command('scene_select','box')
        with patch('scripts.task.multi_point.scene_markers.InteractiveMarkerServer') as server:
            markers=SceneMarkers(Mock(),self.session);markers.refresh()
            displayed={call.args[0].name:call.args[0] for call in server.return_value.insert.call_args_list}
            self.assertEqual(len(displayed['box'].controls),7)
            self.assertEqual(len(displayed['sphere'].controls),1)
            self.assertEqual(displayed['sphere'].controls[0].interaction_mode,InteractiveMarkerControl.BUTTON)
            self.assertEqual(displayed['sphere'].description,'')
            revision=self.session.scene_revision
            applied=self.client.scene.apply.call_count
            click=InteractiveMarkerFeedback(marker_name='sphere',control_name='select_object',event_type=InteractiveMarkerFeedback.BUTTON_CLICK)
            markers.feedback(click);self.session.jobs.join()
            self.assertEqual(self.session.status()['scene_selected'],'sphere')
            self.assertFalse(self.session.scene_dirty)
            self.assertEqual(self.session.scene_revision,revision)
            self.assertEqual(self.client.scene.apply.call_count,applied)
            server.return_value.insert.reset_mock();markers.refresh()
            displayed={call.args[0].name:call.args[0] for call in server.return_value.insert.call_args_list}
            self.assertEqual(len(displayed['box'].controls),1)
            self.assertEqual(len(displayed['sphere'].controls),7)
            click.event_type=InteractiveMarkerFeedback.MOUSE_UP
            markers.feedback(click);self.session.jobs.join()
            self.assertFalse(self.session.scene_dirty)  # Clicking a body must not write a pose.
            late=InteractiveMarkerFeedback(marker_name='box',control_name='move_x',event_type=InteractiveMarkerFeedback.MOUSE_UP)
            late.pose.position.x=.9;late.pose.orientation.w=1.
            markers.feedback(late);self.session.jobs.join()
            self.assertFalse(self.session.scene_dirty)
            self.command('scene_delete','sphere')
            self.assertEqual(self.session.scene_selected,'box')
            self.command('scene_delete','box')
            self.assertEqual(self.session.scene_selected,'')
            server.return_value.insert.reset_mock();markers.refresh()
            server.return_value.insert.assert_not_called()

if __name__ == '__main__': unittest.main()
