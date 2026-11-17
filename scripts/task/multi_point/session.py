"""Persistent task session shared by the terminal and RViz panel."""
from copy import deepcopy
from pathlib import Path
import queue
import threading
from . import task as task_module
from scripts.task.common.schema import load_config, load_scene
from scripts.task.common.run_logger import RunLog
from scripts.task.common.joint_bounds import StartStateBoundsError
from scripts.task.common.robot_feedback import FeedbackUnavailableError


class TaskSession:
    extra_commands = ()
    def __init__(self, client, directory, scene_path=None, task_factory=None):
        self.client = client
        self.directory = Path(directory)
        self.scene_path = scene_path
        self.task_factory = task_factory or task_module.MultiPointTask
        self.config_path = None
        self.config = None
        self.scene = None
        self.reviewed = None
        self.preview_log = None
        self.preview_epoch = None
        self.lock = threading.RLock()
        self.jobs = queue.Queue()
        self.closing = threading.Event()
        self.state = 'IDLE'
        self.message = '可使用官方 RViz 单点控制，或加载多点配置。'
        self.summary = ''
        self.busy = False
        self.last_output = ''
        self.preview_visible = False
        self.worker = threading.Thread(target=self._work, name='piper-task-session', daemon=True)
        self.worker.start()

    def status(self):
        with self.lock:
            return dict(state=self.state, message=self.message, summary=self.summary, busy=self.busy,
                        config_path=str(self.config_path or ''), has_config=self.config is not None,
                        can_execute=self.reviewed is not None and not self.busy,
                        last_output=self.last_output, preview_visible=self.preview_visible,
                        recovery_required=getattr(self.client,'task_gate_closed',False))

    def _state(self, state, message, summary=''):
        with self.lock:
            self.state, self.message = state, message
            self.summary = summary
        print(f'[{state}] {message}', flush=True)

    def submit(self, command, path=''):
        command = command.lower().strip()
        with self.lock:
            if self.closing.is_set(): return False, '程序正在退出。'
            if command == 'stop':
                if not self.busy: return False, '本程序没有正在运行的任务；不会停止官方单点操作。'
                self.client.cancel_requested.set()
                self.message = '已请求停止当前任务，请等待处理结果。'
                return True, self.message
            if self.busy: return False, '任务处理中，请等待完成或停止当前任务。'
            if command not in ('load', 'load_plan', 'plan', 'execute', 'replay', 'manual', 'recover') + self.extra_commands:
                return False, '未知命令。'
            if command in ('load','load_plan') and not path.strip(): return False, '请选择配置文件。'
            if command in ('plan', 'execute') and self.config_path is None:
                return False, '请先加载配置。'
            if command == 'recover' and not getattr(self.client,'task_gate_closed',False):
                return False, '当前没有本任务关闭的控制门。'
            if command == 'execute' and getattr(self.client,'task_gate_closed',False):
                return False, '请先恢复控制，再重新规划并执行。'
            if command == 'execute' and self.reviewed is None:
                return False, '请先从当前位置重新规划并查看预览。'
            self.busy = True  # Both frontends reserve the same worker before acknowledgement.
            self.client.cancel_requested.clear()
            self.jobs.put((command, path))
            return True, '已接受。'

    def _load(self, path):
        candidate_path = Path(path).expanduser().resolve()
        with self.lock:
            self.reviewed = None
            self.preview_visible = False
            self.config = None
            self.config_path = None
        self.client.clear_preview()
        candidate = load_config(candidate_path)
        scene = load_scene(self.scene_path, candidate['settings']['frame_id'])
        previous = self.client.settings
        try:
            self.client.settings = deepcopy(candidate['settings'])
            self.client.connect(timeout=5)
        except BaseException:
            self.client.settings = previous
            raise
        self.config_path, self.config, self.scene = candidate_path, candidate, scene
        self._state('LOADED', f'已加载 {candidate_path.name}（{len(candidate["points"])} 个目标）。点击规划或输入 p。')

    def _plan(self, *, show_preview=True):
        with self.lock: self.reviewed = None
        self.client.begin_round()
        self._load(self.config_path)  # Reload file edits, but always sample a new real start.
        self.preview_visible = False
        self.client.clear_preview()
        self._state('PLANNING', '从当前实测位置规划；请勿同时操作官方 Execute。')
        log = RunLog(self.directory / 'rounds')
        self.last_output = str(log.directory)
        print(f'日志：{log.directory}', flush=True)
        epoch = self.client.activity_generation
        task = self.task_factory(self.client, self.config, self.scene, log, False)
        task.publish_preview = show_preview
        task.run()
        if epoch != self.client.activity_generation:
            raise RuntimeError('规划期间有其他运动请求，预览已作废，请重新规划。')
        self.reviewed, self.preview_log, self.preview_epoch = task.reviewed_plan, log, epoch
        self.preview_visible = show_preview
        self._state('READY' if show_preview else 'PLANNED',
                    '预览就绪。确认后点击“确认执行”或输入 e；Enter 仅重播。' if show_preview else '规划完成，请点击对应预览。')

    def _execute(self):
        reviewed = self.reviewed
        with self.lock: self.reviewed = None  # Consume this preview even when validation fails.
        if reviewed is None: raise RuntimeError('预览已失效，请重新规划。')
        if self.preview_epoch != self.client.activity_generation:
            raise RuntimeError('预览后有其他运动请求，请重新规划。')
        self.client.begin_round()
        self._state('EXECUTING', '正在执行实机运动；请勿同时点击官方 Execute。')
        log = RunLog(self.preview_log.directory / 'executions')
        log.write('preview_source.json', dict(directory=str(self.preview_log.directory)))
        self.preview_log.event('OPERATOR_EXECUTE', execution_directory=str(log.directory))
        self.last_output = str(log.directory)
        print(f'日志：{log.directory}', flush=True)
        self.task_factory(self.client, self.config, self.scene, log, True).run(reviewed=reviewed)
        self.client.publish_plan(reviewed['initial'],[part['trajectory'] for part in reviewed['segments']])
        self._state('IDLE', '执行完成。p 从当前终点重新规划；也可加载新配置或使用官方单点控制。')

    def poll(self):
        """Invalidate an idle preview after motion, without stopping external controls."""
        with self.lock:
            if self.busy or self.reviewed is None: return
            try:
                changed = self.client.activity_generation != self.preview_epoch
                q = self.client.current().q
                changed |= max(abs(a-b) for a,b in zip(q,self.reviewed['initial'])) > self.config['settings']['start_tolerance_rad']
            except RuntimeError:
                changed = True
            if changed:
                self.reviewed = None
                self.preview_visible = False
                self.client.clear_preview()
                self._state('IDLE', '当前位置或反馈已变化，旧预览作废。请重新规划。')

    def _work(self):
        while not self.closing.is_set():
            try: command, path = self.jobs.get(timeout=0.1)
            except queue.Empty: continue
            try:
                if command == 'load': self._load(path)
                elif command == 'load_plan':
                    self._load(path)
                    self._plan()
                elif command == 'plan': self._plan()
                elif command == 'execute': self._execute()
                elif command == 'replay':
                    self.preview_visible = self.client.preview_message is not None
                    self.client.replay_preview()
                elif command == 'manual':
                    self.reviewed = None
                    self.preview_visible = False
                    self.client.clear_preview()
                    self._state('MANUAL', '已退出多点预览。控制门仍关闭，请先恢复控制。' if getattr(self.client,'task_gate_closed',False) else '已退出多点预览，可使用官方单点控制；之后请重新规划。')
                elif command == 'recover':
                    self.reviewed = None
                    self.preview_visible = False
                    self.client.clear_preview()
                    self._state('RECOVERING', '检查旧动作和停稳反馈，在关闭控制门时同步保持目标。')
                    self.client.restore_task_control()
                    self._state('MANUAL', '控制已恢复，可使用官方单点；多点任务请重新规划。')
                else:
                    self.handle_command(command, path)
            except (Exception, KeyboardInterrupt) as exc:
                with self.lock:
                    self.reviewed = None
                    self.preview_visible = False
                try: self.client.clear_preview()
                except Exception as clear_error:
                    print(f'Preview cleanup failed: {clear_error}',flush=True)
                next_step = '控制门已锁定；检查现场后点击恢复控制，再重新规划。' if getattr(self.client,'task_gate_closed',False) else '已返回等待；检查原因后重新规划。'
                if isinstance(exc,StartStateBoundsError):
                    self._state('START_INVALID', f'{exc} 日志：{self.last_output}', exc.summary)
                elif isinstance(exc,FeedbackUnavailableError):
                    self._state('FEEDBACK_UNAVAILABLE', f'{exc} {next_step}日志：{self.last_output}', exc.summary)
                else:
                    self._state('ERROR', f'任务未完成：{exc}。{next_step}日志：{self.last_output}')
            finally:
                with self.lock: self.busy = False
                self.jobs.task_done()

    def close(self):
        self.closing.set()
        self.client.cancel_requested.set()
        self.worker.join()  # Task cancellation runs before any ROS/process teardown.
