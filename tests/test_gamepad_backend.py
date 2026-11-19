"""Backend permit, selection and stop regressions without ROS init or hardware."""
import time
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from piper_servo_tracking.msg import GamepadInput
from scripts.task.servo_tracking.gamepad_backend import GamepadBackend
from scripts.tools.gamepad_control.core import (
    ControlSelection, ControlState, InputFrame, InputGate, VelocityIntentFilter,
)
from scripts.tools.gamepad_control.settings import load_tool_config


def backend(phase='ARMED'):
    now = time.monotonic()
    state = NS(phase=phase, reason='', mode='sim', gp=load_tool_config(),
               selection=ControlSelection(), selection_observer=NS(confirmed=Mock(return_value=True), reset=Mock()),
               input=InputFrame('fixture', 1, 2, now, (0.,)*4, (0,)*4, False, False),
               input_seen=now, input_error='', input_gate=InputGate(), navigation_down=(False, False),
               ui_seen=now, guard_seen=now, guard_state='idle',
               guard_status={'gate_closed': True, 'stop_confirmed': True, 'stop_generation': 3,
                             'stop_requested_at': now},
               stop_started=0., stop_generation_required=3, config={'feedback_timeout': .25},
               control=ControlState(), epoch=4, session=1, after_stop='ARMED',
               intent_filter=VelocityIntentFilter(.04), requested_tcp_velocity=(.3,)*6,
               filtered_tcp_velocity=(.1,)*6, requested_joint_velocity=(.2,)*6,
               filtered_joint_velocity=(.1,)*6, bound_scale=.3, jog_pub=Mock(),
               speed_scale=.2, servo_switched=True, heartbeat=Mock())
    for name, method in GamepadBackend.__dict__.items():
        if callable(method) and not name.startswith('__'):
            setattr(state, name, MethodType(method, state))
    state._healthy = Mock(return_value=True)
    state._request_guard = Mock(return_value=NS())
    state.get_logger = Mock(return_value=Mock())
    state.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time())))
    state.publish_status = Mock()
    state.control.start()
    # InputGate intentionally rejects the first frame to establish identity.
    try:
        state.input_gate.accept(state.input, now)
    except ValueError:
        pass
    state.input_gate.accept(state.input, now)
    return state


def message(sequence=3, *, axes=(0.,)*4, dpad=(0,)*4, lb=False, b=False, connected=True):
    msg = GamepadInput(guid='fixture', generation=1, sequence=sequence, connected=connected,
                       axes=list(axes), dpad=list(dpad), lb=lb, b=b, source='fixture')
    return msg


class GamepadBackendSelectionTests(unittest.TestCase):
    def test_selection_barrier_uses_stopped_guard_fresh_input_and_released_buttons(self):
        state = backend()
        self.assertTrue(state._selection_ready())
        for field, value in (('phase', 'ACTIVE'), ('ui_seen', 0.), ('guard_seen', 0.)):
            old = getattr(state, field); setattr(state, field, value)
            self.assertFalse(state._selection_ready(), field)
            setattr(state, field, old)
        for key, value in (('gate_closed', False), ('other_action_active', True)):
            old = state.guard_status.get(key); state.guard_status[key] = value
            self.assertFalse(state._selection_ready(), key)
            state.guard_status[key] = old
        state.selection_observer.confirmed.return_value = False
        self.assertFalse(state._selection_ready())
        state.selection_observer.confirmed.return_value = True
        state.input = InputFrame('fixture', 1, 3, time.monotonic(), (0.,)*4, (0, 0, 0, 1), False, False)
        self.assertFalse(state._selection_ready())
        self.assertTrue(state._selection_ready(allow_navigation=True))

    def test_changed_selection_resets_motion_filter_and_neutral_history(self):
        state = backend()
        state.control.neutral_since = 1.; state.control.neutral_seen = True
        state.intent_filter.update((.3,)*6, 1.)
        state.intent_filter.update((.3,)*6, 1.04)
        epoch = state.epoch
        self.assertTrue(state._apply_selection('cartesian', 4)[0])
        self.assertEqual(state.selection.revision, 1)
        self.assertEqual(state.epoch, epoch+1)
        self.assertEqual(state.intent_filter.value, (0.,)*6)
        self.assertIsNone(state.control.neutral_since)
        self.assertFalse(state.control.neutral_seen)
        self.assertEqual(tuple(state.jog_pub.publish.call_args.args[0].velocities), (0.,)*6)
        state._request_guard.assert_not_called()
        for field in ('requested_tcp_velocity', 'filtered_tcp_velocity',
                      'requested_joint_velocity', 'filtered_joint_velocity'):
            self.assertEqual(getattr(state, field), (0.,)*6)

    def test_locked_or_active_selection_cannot_change_revision_or_unlock(self):
        for phase in ('ACTIVE', 'PREPARING', 'STOPPING', 'LOCKED', 'STOP_UNKNOWN'):
            state = backend(phase)
            self.assertFalse(state._apply_selection('cartesian', 5)[0])
            self.assertEqual(state.phase, phase)
            self.assertEqual(state.selection.revision, 0)
            state.jog_pub.publish.assert_not_called()

    def test_dpad_selection_is_press_edge_clamped_and_does_not_repeat(self):
        state = backend()
        for sequence in range(3, 12):
            state.on_input(message(sequence, dpad=(0, 0, 0, 1)))
        self.assertEqual(state.selection.joint_index, 2)
        self.assertEqual(state.selection.revision, 1)
        state.on_input(message(12))
        state.on_input(message(13, dpad=(0, 0, 0, 1)))
        self.assertEqual(state.selection.joint_index, 3)
        state.selection.select('single_axis', 6)
        revision = state.selection.revision
        state.on_input(message(14)); state.on_input(message(15, dpad=(0, 0, 0, 1)))
        self.assertEqual((state.selection.joint_index, state.selection.revision), (6, revision))

    def test_active_navigation_does_not_select_and_cartesian_dpad_is_not_navigation(self):
        for phase, mode in (('ACTIVE', 'single_axis'), ('ARMED', 'cartesian')):
            state = backend(phase); state.selection.select(mode, 1)
            state._begin_prepare = Mock()
            state.on_input(message(dpad=(0, 0, 0, 1), lb=phase == 'ACTIVE'))
            self.assertEqual(state.selection.joint_index, 1)

    def test_speed_changes_only_update_current_mode_while_released_and_idle(self):
        state = backend()
        state.on_speed(NS(data=.4))
        self.assertEqual(state.selection.speed_scales, {'single_axis': .4, 'cartesian': .2})
        state.phase = 'ACTIVE'; state.on_speed(NS(data=.6))
        self.assertEqual(state.selection.speed_scale, .4)
        state.phase = 'ARMED'
        state.input = InputFrame('fixture', 1, 3, time.monotonic(), (0.,)*4, (0,)*4, True, False)
        state.on_speed(NS(data=.6))
        self.assertEqual(state.selection.speed_scale, .4)


class GamepadBackendStopTests(unittest.TestCase):
    def test_stop_clears_both_mode_histories_and_publishes_zero_without_filter_tail(self):
        state = backend('ACTIVE')
        state.intent_filter.update((.3,)*6, 1.)
        state.intent_filter.update((.3,)*6, 1.04)
        state._begin_stop('ARMED', 'LB released')
        self.assertEqual((state.phase, state.after_stop), ('STOPPING', 'ARMED'))
        self.assertEqual(tuple(state.jog_pub.publish.call_args.args[0].velocities), (0.,)*6)
        self.assertEqual(state.intent_filter.value, (0.,)*6)
        self.assertIsNone(state.intent_filter.last_time)
        self.assertEqual(state.stop_generation_required, 4)
        state._request_guard.assert_called_once_with(False)

    def test_lb_b_disconnect_each_revoke_without_waiting_for_the_filter(self):
        for msg, destination in ((message(), 'ARMED'), (message(b=True, lb=True), 'LOCKED'),
                                 (message(connected=False), 'LOCKED')):
            state = backend('ACTIVE'); state._begin_stop = Mock()
            state.on_input(msg)
            self.assertEqual(state._begin_stop.call_args.args[0], destination)

    def test_armed_bad_feedback_transitions_the_backend_to_stopping_locked(self):
        state = backend(); state._healthy.return_value = False
        state.on_input(message())
        self.assertEqual(state.phase, 'STOPPING')
        self.assertEqual(state.after_stop, 'LOCKED')

    def test_recovery_cannot_use_stale_guard_or_a_previous_stop_generation(self):
        for stale, generation in ((True, 3), (False, 2)):
            state = backend('LOCKED')
            if stale: state.guard_seen = time.monotonic()-.36
            state.guard_status['stop_generation'] = generation
            response = state.recover_session(None, NS(success=None, message=''))
            self.assertFalse(response.success)
            self.assertEqual(state.phase, 'LOCKED')

    def test_running_before_successful_same_epoch_permit_ack_stays_active(self):
        state = backend('ACTIVE')
        state._prepared(NS(result=lambda: NS(success=True)), state.epoch)
        self.assertEqual(state.phase, 'ACTIVE')
        state._request_guard.assert_not_called()

    def test_running_before_failed_same_epoch_permit_ack_locks(self):
        state = backend('ACTIVE'); state._begin_stop = Mock()
        state._prepared(NS(result=lambda: NS(success=False, message='rejected')), state.epoch)
        self.assertEqual(state._begin_stop.call_args.args[0], 'LOCKED')

    def test_old_epoch_successful_permit_ack_is_revoked(self):
        state = backend('ACTIVE')
        state._prepared(NS(result=lambda: NS(success=True)), state.epoch-1)
        state._request_guard.assert_called_once_with(False)


if __name__ == '__main__':
    unittest.main()
