"""Exercise preview supervision with real isolated processes and no ROS devices."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cli


ROOT = Path(__file__).resolve().parents[3]
TOOL = Path(__file__).resolve().parent


def fixture_config():
    """Keep regression checks independent of the operator's current target settings."""
    return {'target': 'chessboard',
            'chessboard': {'columns': 7, 'rows': 10, 'square_size_mm': 22.0},
            'marker_id': 123, 'marker_size_m': .045,
            'marker_pose_topic': '/preview_fixture/aruco/pose',
            'camera_name': 'fixture_d405', 'camera_namespace': 'preview_fixture',
            'camera_serial': '', 'color_profile': '640x480x30',
            'base_frame': 'base_link', 'end_frame': 'tcp_link'}


class GraphFixture:
    """Supply graph discovery and a bounded clock without creating any ROS node."""
    def __init__(self, nodes=(), publishers=(), missing_package=None, executor_error=False):
        self.elapsed = 0.
        self.lifecycle = []
        self.context = SimpleNamespace(get_domain_id=lambda: 77)
        self.node = SimpleNamespace(
            get_node_names_and_namespaces=Mock(return_value=list(nodes)),
            get_publishers_info_by_topic=Mock(side_effect=lambda topic: [object()] if topic in publishers else []),
            destroy_node=Mock(side_effect=lambda: self.lifecycle.append('destroy_node')))
        self.rclpy = ModuleType('rclpy')
        self.rclpy.context = SimpleNamespace(Context=Mock(return_value=self.context))
        self.rclpy.init = Mock()
        self.rclpy.create_node = Mock(return_value=self.node)
        self.rclpy.try_shutdown = Mock(side_effect=lambda **kwargs: self.lifecycle.append('try_shutdown'))

        def spin_once(timeout_sec):
            self.elapsed += timeout_sec
            if self.elapsed > 3:
                raise AssertionError('Preflight graph discovery exceeded its bounded test window.')

        self.executor = SimpleNamespace(
            add_node=Mock(), spin_once=Mock(side_effect=spin_once),
            shutdown=Mock(side_effect=lambda **kwargs: self.lifecycle.append('executor_shutdown')))
        self.executors = ModuleType('rclpy.executors')
        self.executors.SingleThreadedExecutor = Mock(
            side_effect=RuntimeError('Fixture executor construction failed.') if executor_error else None,
            return_value=self.executor)
        self.rclpy.executors = self.executors
        # Deliberately provide no rclpy.spin_once shortcut: a private Context must
        # be paired with its own executor, rather than the global default one.
        self.packages = ModuleType('ament_index_python.packages')
        self.packages.PackageNotFoundError = type('PackageNotFoundError', (KeyError,), {})

        def package_prefix(name):
            if name == missing_package:
                raise self.packages.PackageNotFoundError(name)
            return '/fixture/prefix'

        self.packages.get_package_prefix = Mock(side_effect=package_prefix)

    def check(self, cfg):
        with patch.dict(os.environ, {'DISPLAY': ':preview-fixture'}), \
                patch.dict(sys.modules, {'rclpy': self.rclpy,
                                         'rclpy.executors': self.executors,
                                         'ament_index_python.packages': self.packages}), \
                patch.object(cli.time, 'monotonic', side_effect=lambda: self.elapsed):
            return cli.preview_preflight(cfg, cfg['target'])

    def assert_closed(self, executor_started=True):
        self.rclpy.init.assert_called_once_with(args=[], context=self.context)
        self.executors.SingleThreadedExecutor.assert_called_once_with(context=self.context)
        self.node.destroy_node.assert_called_once_with()
        self.rclpy.try_shutdown.assert_called_once_with(context=self.context)
        if executor_started:
            self.executor.add_node.assert_called_once_with(self.node)
            self.executor.spin_once.assert_called_with(timeout_sec=.05)
            self.executor.shutdown.assert_called_once_with(timeout_sec=1.)
            if self.lifecycle != ['executor_shutdown', 'destroy_node', 'try_shutdown']:
                raise AssertionError(f'Unexpected preflight teardown order: {self.lifecycle}')
        else:
            self.executor.add_node.assert_not_called()
            self.executor.spin_once.assert_not_called()
            self.executor.shutdown.assert_not_called()
            if self.lifecycle != ['destroy_node', 'try_shutdown']:
                raise AssertionError(f'Unexpected failed-executor teardown order: {self.lifecycle}')

# This child stays in its parent's session and resists the first two signals.
# Cleaning up only the launch process would leave it alive.
_GRANDCHILD = r'''
import json, os, signal, sys, time
from pathlib import Path
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
Path(sys.argv[1]).write_text(json.dumps({'pid': os.getpid(), 'pgid': os.getpgrp()}))
while True:
    time.sleep(.05)
'''

_WORKER = r'''
import json, os, signal, subprocess, sys, time
from pathlib import Path
spec = json.loads(sys.argv[1])
root = Path(spec['records'])
role = spec['role']
signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
if spec.get('grandchild'):
    subprocess.Popen([sys.executable, '-u', '-c', spec['grandchild_code'],
                      str(root / (role + '-grandchild.json'))])
    deadline = time.monotonic() + 5
    while not (root / (role + '-grandchild.json')).exists():
        if time.monotonic() >= deadline:
            raise RuntimeError('Fixture grandchild failed to start.')
        time.sleep(.01)
record = {'pid': os.getpid(), 'pgid': os.getpgrp(),
          'environment': {name: os.environ.get(name) for name in spec['environment']}}
(root / (role + '.json')).write_text(json.dumps(record))
print('worker:' + role, flush=True)
if spec.get('mode') == 'exit':
    deadline = time.monotonic() + 5
    while not all((root / (name + '.json')).exists() for name in ('camera', 'marker', 'view')):
        if time.monotonic() >= deadline:
            raise RuntimeError('Fixture siblings failed to start.')
        time.sleep(.01)
    time.sleep(.1)
    sys.exit(spec.get('code', 0))
while True:
    time.sleep(.05)
'''


def _worker_commands(spec):
    commands = {}
    for role in ('camera', 'marker', 'view'):
        worker = {'role': role, 'records': spec['records'],
                  'environment': spec.get('environment', []),
                  'grandchild_code': _GRANDCHILD, **spec['workers'].get(role, {})}
        commands[role] = [sys.executable, '-u', '-c', _WORKER, json.dumps(worker)]
    return commands


def _fixture_main(spec):
    """Run the actual supervisor in a separate interpreter for real SIGINT tests."""
    real_stop = cli.stop_process_group
    real_spawn = subprocess.Popen
    count = 0

    def stop_owned(process, *args, **kwargs):
        report = real_stop(process, timeouts=(.3, .2, .2))
        if spec.get('cleanup_error'):
            report['errors'].append('Injected cleanup report failure after real cleanup.')
        return report

    def spawn(*args, **kwargs):
        nonlocal count
        count += 1
        if count == spec.get('fail_spawn'):
            # Ensure an actual child exists before testing partial-start cleanup.
            deadline = time.monotonic() + 5
            while not (Path(spec['records']) / 'camera.json').exists():
                if time.monotonic() >= deadline:
                    raise AssertionError('First fixture child never started.')
                time.sleep(.01)
            raise OSError('Injected Popen failure')
        return real_spawn(*args, **kwargs)

    with patch.object(cli, 'preview_preflight'), \
            patch.object(cli, 'preview_commands', return_value=_worker_commands(spec)), \
            patch.object(cli, 'PREVIEW_ROOT', Path(spec['runs'])), \
            patch.object(cli, 'stop_process_group', side_effect=stop_owned), \
            patch.object(subprocess, 'Popen', side_effect=spawn):
        result = cli.run_preview(spec['config'], tracking=spec.get('tracking', False))
    print('PREVIEW_TEST_RESULT ' + str(result), flush=True)
    raise SystemExit(result)


def _running(pid):
    try:
        return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0] not in ('Z', 'X')
    except FileNotFoundError:
        return False


class PreviewChecks(unittest.TestCase):
    def setUp(self):
        (ROOT / 'outputs').mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / 'outputs', prefix='handeye-preview-test-')
        self.root = Path(self.temp.name)
        self.supervisors = []

    def tearDown(self):
        # Also clean up after a failed assertion, so a regression cannot leave workers.
        for process, _ in self.supervisors:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)
        for record in self.root.glob('*/processes/*.json'):
            try:
                pgid = json.loads(record.read_text())['pgid']
                if pgid != os.getpgrp():
                    os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, FileNotFoundError, json.JSONDecodeError):
                pass
        self.temp.cleanup()

    def fixture(self, name, workers=None, runs=None, **options):
        directory = self.root / name
        directory.mkdir()
        records = directory / 'processes'
        records.mkdir()
        spec = {'config': fixture_config(), 'workers': workers or {},
                'records': str(records), 'runs': str(runs or directory / 'runs'), **options}
        environment = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1',
                       'ROS_DOMAIN_ID': '77', 'HAND_EYE_TEST_TOKEN': 'inherit-me',
                       'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'AMENT_PREFIX_PATH': '/fixture/ament'}
        environment['PYTHONPATH'] = os.pathsep.join(
            [str(TOOL), str(ROOT), environment.get('PYTHONPATH', '')])
        output = directory / 'supervisor.log'
        code = 'import json, sys; from tests_preview import _fixture_main; _fixture_main(json.loads(sys.argv[1]))'
        with output.open('w') as stream:
            process = subprocess.Popen([sys.executable, '-u', '-c', code, json.dumps(spec)],
                                       stdout=stream, stderr=subprocess.STDOUT,
                                       env=environment, start_new_session=True, cwd=ROOT)
        self.supervisors.append((process, directory))
        return process, directory

    def finish(self, process, directory, expected):
        try:
            actual = process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            self.fail('Supervisor did not finish:\n' + (directory / 'supervisor.log').read_text())
        self.assertEqual(actual, expected, (directory / 'supervisor.log').read_text())
        self.assertIn(f'PREVIEW_TEST_RESULT {expected}', (directory / 'supervisor.log').read_text())
        for record in (directory / 'processes').glob('*.json'):
            pid = json.loads(record.read_text())['pid']
            self.assertFalse(_running(pid), f'Owned process {pid} survived shutdown: {record}')

    def ready(self, directory):
        deadline = time.monotonic() + 6
        while not all((directory / 'processes' / f'{role}.json').exists()
                      for role in ('camera', 'marker', 'view')):
            if time.monotonic() >= deadline:
                self.fail('Workers did not start:\n' + (directory / 'supervisor.log').read_text())
            time.sleep(.02)

    def test_view_close_stops_workers_and_resistant_grandchild_and_keeps_logs(self):
        process, directory = self.fixture('view-close', workers={
            'camera': {'grandchild': True}, 'view': {'mode': 'exit', 'code': 0}})
        self.finish(process, directory, 0)
        grandchild = json.loads((directory / 'processes/camera-grandchild.json').read_text())
        camera = json.loads((directory / 'processes/camera.json').read_text())
        self.assertEqual(grandchild['pgid'], camera['pgid'])
        logs = list((directory / 'runs').rglob('*.log'))
        for role in ('camera', 'marker', 'view'):
            self.assertTrue(any(f'worker:{role}' in log.read_text() for log in logs),
                            f'No saved {role} output among {logs}')

    def test_child_failure_returns_nonzero_and_cleans_siblings(self):
        process, directory = self.fixture('camera-failure', workers={
            'camera': {'mode': 'exit', 'code': 7}, 'marker': {'grandchild': True}})
        self.finish(process, directory, 1)
        self.assertTrue(any('worker:camera' in path.read_text()
                            for path in (directory / 'runs').rglob('*.log')))

    def test_sigint_stops_owned_groups(self):
        process, directory = self.fixture('sigint', workers={'marker': {'grandchild': True}})
        self.ready(directory)
        os.kill(process.pid, signal.SIGINT)
        self.finish(process, directory, 0)

    def test_second_session_is_rejected_without_stopping_the_first(self):
        runs = self.root / 'shared-runs'
        first, first_directory = self.fixture('first-session', runs=runs)
        self.ready(first_directory)
        second, second_directory = self.fixture('second-session', runs=runs)
        self.finish(second, second_directory, 1)
        self.assertIsNone(first.poll(), (first_directory / 'supervisor.log').read_text())
        for record in (first_directory / 'processes').glob('*.json'):
            self.assertTrue(_running(json.loads(record.read_text())['pid']))
        self.assertEqual(list((second_directory / 'processes').glob('*.json')), [])
        os.kill(first.pid, signal.SIGINT)
        self.finish(first, first_directory, 0)

    def test_partial_start_failure_cleans_the_started_group(self):
        process, directory = self.fixture('partial-start', fail_spawn=2,
                                          workers={'camera': {'grandchild': True}})
        self.finish(process, directory, 1)
        self.assertTrue((directory / 'processes/camera.json').exists())
        self.assertFalse((directory / 'processes/marker.json').exists())

    def test_children_keep_the_existing_ros_environment(self):
        names = ['ROS_DOMAIN_ID', 'HAND_EYE_TEST_TOKEN', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH']
        expected = ['77', 'inherit-me', 'rmw_fastrtps_cpp', '/fixture/ament']
        process, directory = self.fixture('environment', environment=names,
                                          workers={'view': {'mode': 'exit'}})
        self.finish(process, directory, 0)
        for role in ('camera', 'marker', 'view'):
            actual = json.loads((directory / 'processes' / f'{role}.json').read_text())['environment']
            self.assertEqual(actual, dict(zip(names, expected)))

    def test_cleanup_errors_are_reported_as_failure(self):
        process, directory = self.fixture('cleanup-error', cleanup_error=True,
                                          workers={'view': {'mode': 'exit'}})
        self.finish(process, directory, 1)

    def test_commands_reuse_the_current_interpreter_without_nested_environment_reset(self):
        commands = cli.preview_commands()
        self.assertEqual(list(commands), ['camera', 'marker', 'view'])
        for role, command in commands.items():
            expected = [sys.executable, '-u', str(TOOL / 'cli.py'), role]
            self.assertEqual(command, expected + (['--color-only'] if role == 'camera' else []))
            self.assertEqual(cli.preview_commands(with_depth=True)[role], expected)
        self.assertEqual(cli.preview_commands(tracking=True)['marker'], commands['marker'] + ['--tracking'])

    def test_tracking_camera_uses_typed_hardware_clock_option(self):
        process, directory = self.fixture('tracking-clock', tracking=True, workers={'view': {'mode': 'exit'}})
        self.finish(process, directory, 0)
        session = next((directory / 'runs').glob('*/session.json'))
        camera = json.loads(session.read_text())['processes']['camera']['command']
        options = Path(camera[camera.index('--camera-options')+1])
        self.assertEqual(options.parent, session.parent)
        for flags in (['--camera-options', str(options)], ['--tracking'], []):
            with patch.object(sys, 'argv', ['cli.py', 'camera', '--color-only', *flags]), \
                    patch.object(cli, 'PREVIEW_ROOT', self.root), patch.object(cli, 'ros_exec') as launch:
                cli.main()
            configs = [x.split(':=', 1)[1] for x in launch.call_args.args[0] if x.startswith('config_file:=')]
            if flags:
                self.assertEqual(len(configs), 1)
                self.assertIs(cli.yaml.safe_load(Path(configs[0]).read_text())['depth_module.global_time_enabled'], False)
            else:
                self.assertEqual(configs, [])

    def test_target_parameters_are_checked_without_starting_processes(self):
        cfg = fixture_config()
        kind, _, topic = cli.target_settings(cfg)
        self.assertEqual(kind, 'chessboard')
        self.assertEqual(topic, '/handeye/chessboard/result')
        pattern, size = cli.validate_target(cfg, kind)
        self.assertEqual(pattern, (7, 10))
        self.assertAlmostEqual(size, .022)
        with self.assertRaises(SystemExit):
            cli.target_settings({**cfg, 'target': 'unsupported'})
        with self.assertRaises(SystemExit):
            cli.validate_target({**cfg, 'chessboard': {'columns': 2, 'rows': 11, 'square_size_mm': 15}}, kind)
        with self.assertRaises(SystemExit):
            cli.validate_target({**cfg, 'marker_size_m': None}, 'aruco')

    def test_missing_display_is_rejected_before_starting_any_process(self):
        with patch.dict(os.environ, {'DISPLAY': ''}), \
                patch.object(cli, 'PREVIEW_ROOT', self.root / 'display-check'), \
                patch.object(cli, 'validate_target'), \
                patch.object(cli.subprocess, 'Popen') as spawn:
            with self.assertRaisesRegex(SystemExit, 'DISPLAY'):
                cli.run_preview(fixture_config())
            spawn.assert_not_called()

    def test_preflight_accepts_empty_graph_and_closes_its_context(self):
        for target in ('chessboard', 'aruco'):
            with self.subTest(target=target):
                graph = GraphFixture()
                cfg = {**fixture_config(), 'target': target}
                self.assertIsNone(graph.check(cfg))
                self.assertGreaterEqual(graph.elapsed, 1.)
                graph.assert_closed()
                required = ['realsense2_camera', 'rqt_image_view']
                if target == 'aruco':
                    required.append('aruco_ros')
                self.assertEqual([call.args[0] for call in graph.packages.get_package_prefix.call_args_list],
                                 required)

    def test_preflight_rejects_camera_or_detector_node_conflicts_and_closes_context(self):
        cfg = fixture_config()
        for node in ((cfg['camera_name'], '/' + cfg['camera_namespace']), ('handeye_chessboard', '/')):
            with self.subTest(node=node):
                graph = GraphFixture(nodes=[node])
                with self.assertRaisesRegex(SystemExit, node[0]):
                    graph.check(cfg)
                graph.assert_closed()

    def test_preflight_rejects_result_publisher_and_closes_context(self):
        result_topic = '/handeye/chessboard/result'
        graph = GraphFixture(publishers=[result_topic])
        with self.assertRaisesRegex(SystemExit, result_topic):
            graph.check(fixture_config())
        graph.assert_closed()

    def test_preflight_reports_missing_package_before_ros_initialization(self):
        graph = GraphFixture(missing_package='rqt_image_view')
        with self.assertRaisesRegex(SystemExit, '缺少 ROS 包 rqt_image_view'):
            graph.check(fixture_config())
        graph.rclpy.init.assert_not_called()
        graph.rclpy.create_node.assert_not_called()

    def test_preflight_cleans_node_and_context_when_executor_construction_fails(self):
        graph = GraphFixture(executor_error=True)
        with self.assertRaisesRegex(RuntimeError, 'Fixture executor construction failed'):
            graph.check(fixture_config())
        graph.assert_closed(executor_started=False)


def run():
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(PreviewChecks))
    if not result.wasSuccessful():
        raise AssertionError('相机检测预览进程管理回归测试失败。')


if __name__ == '__main__':
    run()
