"""Independent Cartesian parameter selection and legacy task compatibility."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

from scripts.task.servo_tracking import core


ROOT = Path(__file__).resolve().parents[1]
TASK = ROOT/'scripts/task/servo_tracking'
CONFIG, CONTROL = TASK/'config/config.yaml', TASK/'config/control.yaml'


class ServoControlConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.path = self.directory/'task.yaml'
        self.data = yaml.safe_load(CONFIG.read_text())

    def write_task(self, **updates):
        data = self.data.copy()
        data.update(updates)
        self.path.write_text(yaml.safe_dump(data))
        return self.path

    def write_control(self, relative='control.yaml', **updates):
        path = self.directory/relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(updates))
        return path

    def test_adjacent_control_is_resolved_and_active_gains_map_to_compatibility_fields(self):
        source = self.write_control(algorithm='pd', linear_speed=.12, angular_speed=.8,
            gains={'pd': {'position_gain': 3., 'orientation_gain': 2.5}})
        config = core.load_config(self.write_task())
        self.assertEqual(config['control_config'], str(source.resolve()))
        self.assertEqual(config['control']['algorithm'], 'pd')
        self.assertEqual(config['linear_speed'], .12)
        self.assertEqual(config['position_gain'], 3.)
        self.assertEqual(config['orientation_gain'], 2.5)
        self.assertFalse(config['control_legacy_overrides'])

    def test_explicit_argument_precedes_task_selection_and_legacy_flat_values(self):
        first = self.write_control('profiles/first.yaml', algorithm='pi', linear_speed=.11)
        second = self.write_control('profiles/second.yaml', algorithm='p', linear_speed=.22)
        path = self.write_task(control_config='profiles/first.yaml', linear_speed=float('nan'))
        self.assertEqual(core.load_config(path)['control_config'], str(first.resolve()))
        config = core.load_config(path, 'profiles/second.yaml')
        self.assertEqual(config['control_config'], str(second.resolve()))
        self.assertEqual(config['control']['algorithm'], 'p')
        self.assertEqual(config['linear_speed'], .22)
        self.assertFalse(config['control_legacy_overrides'])

    def test_old_flat_settings_override_only_active_controller_and_are_validated(self):
        self.write_control(algorithm='feedforward_p')
        config = core.load_config(self.write_task(linear_speed=.15, position_gain=5.))
        self.assertEqual(config['control']['linear_speed'], .15)
        self.assertEqual(config['control']['gains']['feedforward_p']['position_gain'], 5.)
        self.assertEqual(config['control']['gains']['p']['position_gain'], 2.)
        self.assertTrue(config['control_legacy_overrides'])
        for update in ({'linear_speed': 0.}, {'angular_speed': float('nan')},
                       {'position_gain': True}, {'orientation_gain': -1.},
                       {'position_tolerance': -.001}, {'orientation_tolerance': float('inf')}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                core.load_config(self.write_task(**update))

    def test_temporary_task_without_neighbor_uses_module_default_and_missing_files_fail(self):
        path = self.write_task()
        config = core.load_config(path)
        self.assertEqual(config['control_config'], str(CONTROL.resolve()))
        self.assertEqual(config['control']['algorithm'], yaml.safe_load(CONTROL.read_text())['algorithm'])
        with self.assertRaises(FileNotFoundError):
            core.load_config(path, 'missing.yaml')
        with patch.object(core, '__file__', str(self.directory/'isolated/core.py')):
            with self.assertRaises(FileNotFoundError):
                core.load_config(path)

    def test_zero_controller_tolerances_keep_schema_meaning_and_hold_defaults(self):
        self.write_control(position_tolerance=0., orientation_tolerance=0.)
        config = core.load_config(self.write_task())
        self.assertEqual(config['position_tolerance'], 0.)
        self.assertEqual(config['orientation_tolerance'], 0.)
        self.assertGreater(config['hold_position_tolerance'], 0.)

    def test_cli_exposes_independent_control_configuration_without_initializing_ros(self):
        result = subprocess.run([sys.executable, str(TASK/'run.py'), '--help'],
            capture_output=True, text=True, check=True)
        self.assertIn('--control-config', result.stdout)

    def test_observer_settings_are_independent_positive_and_time_budgets_are_ordered(self):
        from scripts.task.servo_tracking.cartesian_control import validate_control_config
        config = validate_control_config({})
        self.assertEqual(config['observer_stable_window'], .16)
        self.assertEqual(config['observer_saturation_window'], .08)
        self.assertEqual(config['observer_linear_noise'], .005)
        self.assertEqual(config['observer_angular_noise'], .025)
        self.assertEqual(config['observer_relative_noise'], .10)
        for key in ('observer_stable_window', 'observer_saturation_window',
                    'observer_linear_noise', 'observer_angular_noise', 'observer_relative_noise'):
            for value in (0., -1., True, float('inf'), float('nan'), '0.1'):
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    validate_control_config({key: value})
        for update in ({'observer_stable_window': .09}, {'observer_saturation_window': .11},
                       {'execution_feedback_timeout': .20}, {'execution_feedback_timeout': .05}):
            with self.subTest(update=update), self.assertRaises(ValueError):
                validate_control_config(update)
        custom = validate_control_config(dict(execution_feedback_timeout=.15,
            observer_stable_window=.2, observer_saturation_window=.12))
        self.assertEqual(custom['observer_saturation_window'], .12)


if __name__ == '__main__':
    unittest.main()
