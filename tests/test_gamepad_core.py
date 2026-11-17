import unittest
from scripts.tools.gamepad_control.core import (ControlState, InputFrame, InputGate, map_velocity,
    shape_axis, stop_confirmed_for_generation, validate_tool_config)
from scripts.tools.gamepad_control.input_adapter import _profile_mapping
from scripts.task.servo_tracking.stop_observer import StopObserver
import yaml
from pathlib import Path


def frame(seq=1, gen=1, axes=(0,0,0,0), dpad=(0,0,0,0), lb=False, b=False, t=10.):
    return InputFrame('flydigi-guid', gen, seq, t, axes, dpad, lb, b)


class GamepadCoreTests(unittest.TestCase):
    def test_fixed_mapping_and_vector_caps(self):
        self.assertAlmostEqual(map_velocity(frame(axes=(0,-1,0,0)), 1)[0], .05)
        self.assertAlmostEqual(map_velocity(frame(axes=(1,0,0,0)), 1)[1], -.05)
        self.assertAlmostEqual(map_velocity(frame(axes=(0,0,0,-1)), 1)[2], .05)
        v = map_velocity(frame(axes=(1,-1,1,-1), dpad=(1,0,1,0)), 1)
        self.assertLessEqual(sum(x*x for x in v[:3])**.5, .0500001)
        self.assertLessEqual(sum(x*x for x in v[3:])**.5, .2500001)

    def test_deadzone_continuous_monotonic_and_dpad_conflict(self):
        self.assertEqual(shape_axis(.08), 0.)
        self.assertGreater(shape_axis(.081), 0.)
        self.assertLess(shape_axis(.2), shape_axis(.4))
        result = map_velocity(frame(dpad=(1,1,0,0)),1)
        self.assertEqual(result[4],0.)

    def test_generation_sequence_and_expiry_fail_closed(self):
        gate=InputGate(.2)
        with self.assertRaisesRegex(ValueError,'连接已变化'):
            gate.accept(frame(), now=10.)
        self.assertEqual(gate.accept(frame(2), now=10.).sequence,2)
        with self.assertRaisesRegex(ValueError,'乱序'):
            gate.accept(frame(2), now=10.)
        with self.assertRaisesRegex(ValueError,'过期'):
            gate.accept(frame(3,t=9.), now=10.)
        with self.assertRaisesRegex(ValueError,'连接已变化'):
            gate.accept(frame(1,gen=2), now=10.)

    def test_lb_requires_neutral_and_b_latches(self):
        state=ControlState(); state.start()
        self.assertEqual(state.update(frame(axes=(0.5,0,0,0),lb=True)),(0.,)*6)
        state.update(frame(2,t=10.00))
        state.update(frame(3,t=10.16))
        output=state.update(frame(4,axes=(0.7,0,0,0),lb=True,t=10.18))
        self.assertEqual(state.state,'ACTIVE')
        self.assertLess(output[1],0.)
        state.update(frame(5,lb=False,t=10.20))
        self.assertEqual(state.state,'ARMED')
        state.update(frame(6,b=True,t=10.22))
        self.assertEqual(state.state,'LOCKED')
        self.assertFalse(state.recover(False))
        self.assertTrue(state.recover(True))

    def test_stop_confirmation_needs_unique_fresh_samples_and_full_window(self):
        observer=StopObserver(.30,.003)
        self.assertTrue(observer.observe(1.,0.,[0.]*6))
        self.assertFalse(observer.observe(1.,.1,[0.]*6))
        self.assertFalse(observer.confirmed(.20,.05))
        for i in range(1,9): observer.observe(1.+i*.04,i*.04,[0.]*6)
        self.assertTrue(observer.confirmed(.32,.05))

    def test_stop_confirmation_rejects_slow_drift_and_stale_feedback(self):
        observer=StopObserver(.30,.003)
        for i in range(4):
            q=[i*.0014]+[0.]*5
            observer.observe(2.+i*.1,i*.1,q)
        self.assertFalse(observer.confirmed(.31,.05))
        stable=StopObserver(.30,.003)
        for i in range(4): stable.observe(3.+i*.1,i*.1,[0.]*6)
        self.assertFalse(stable.confirmed(.6,.05))

    def test_gamepad_configuration_fails_closed_on_unsafe_ranges(self):
        path=Path(__file__).parents[1]/'scripts/tools/gamepad_control/config/config.yaml'
        config=validate_tool_config(yaml.safe_load(path.read_text()))
        self.assertEqual(config['default_speed_scale'],.2)
        config['linear_speed_max']=.2
        with self.assertRaisesRegex(ValueError,'linear_speed_max'):
            validate_tool_config(config)

    def test_evdev_profile_drives_axis_inversion_buttons_and_dpad(self):
        profile={'evdev':{'axes':{
                    'left_x':{'code':'ABS_Z','invert':True},'left_y':'ABS_Y',
                    'right_x':'ABS_RX','right_y':'ABS_RY'},
                  'buttons':{'lb':'BTN_TL','b':'BTN_EAST'},
                  'dpad':{'up':'ABS_HAT0Y:-1','down':'ABS_HAT0Y:1',
                          'left':'ABS_HAT0X:-1','right':'ABS_HAT0X:1'}}}
        axes,inversions,buttons,dpad=_profile_mapping(profile)
        self.assertEqual(axes,(2,1,3,4))
        self.assertEqual(inversions,(True,False,False,False))
        self.assertEqual(buttons,{'lb':0x136,'b':0x131})
        self.assertEqual(dpad,((17,-1),(17,1),(16,-1),(16,1)))
        profile['evdev']['axes']['left_x']={'code':'ABS_HAT0X'}
        with self.assertRaisesRegex(ValueError,'轴 left_x'):
            _profile_mapping(profile)

    def test_old_guard_confirmation_cannot_finish_a_new_stop(self):
        self.assertFalse(stop_confirmed_for_generation(
            {'stop_confirmed':True,'stop_generation':4},5))
        self.assertFalse(stop_confirmed_for_generation(
            {'stop_confirmed':False,'stop_generation':5},5))
        self.assertTrue(stop_confirmed_for_generation(
            {'stop_confirmed':True,'stop_generation':5},5))


if __name__ == '__main__':
    unittest.main()
