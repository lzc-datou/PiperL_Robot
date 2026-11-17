"""Editable scene drafts and optional two-path planning in the existing task session."""
from copy import deepcopy
import json
from pathlib import Path
import yaml

from .session import TaskSession
from scripts.task.common.schema import load_scene, validate_scene
from scripts.task.common.planning_scene import fingerprint


class SceneSession(TaskSession):
    comparison_enabled = False
    extra_commands = ('scene_load', 'scene_edit', 'scene_upsert', 'scene_delete', 'scene_pose',
                      'scene_apply', 'scene_save', 'scene_select', 'preview_baseline', 'preview_avoidance',
                      'plan_baseline', 'plan_avoidance', 'execute_baseline', 'execute_avoidance')

    def __init__(self, *args, **kwargs):
        self.draft = None
        self.applied_scene = None
        self.scene_dirty = False
        self.scene_revision = 0
        self.scene_editing = False
        self.scene_file = ''
        self.scene_selected = ''
        self.selected_mode = 'avoidance'
        self.cached = {}
        self.previewed_modes = set()
        self._building = False
        super().__init__(*args, **kwargs)

    def status(self):
        with self.lock:
            state = super().status()
            state.update(scene_editor=True, comparison_enabled=self.comparison_enabled,
                         selected_mode=self.selected_mode, scene_dirty=self.scene_dirty,
                         scene_editing=self.scene_editing, scene_revision=self.scene_revision,
                         scene_selected=self.scene_selected,
                         scene_file=self.scene_file, scene_objects=deepcopy((self.draft or {}).get('objects', [])),
                         has_baseline='baseline' in self.cached, has_avoidance='avoidance' in self.cached)
            state['can_execute'] &= not self.scene_dirty and not self.scene_editing
            for mode in ('baseline','avoidance'):
                state['can_execute_'+mode] = (mode in self.cached and mode in self.previewed_modes
                    and not self.busy and not self.scene_dirty and not self.scene_editing
                    and not state['recovery_required'])
            return state

    def _state(self, state, message, summary=''):
        if self._building:
            # A baseline alone is not a completed comparison preview.
            if state in ('LOADED','READY'): return
            if state == 'PLANNING':
                label='无避障' if self.selected_mode=='baseline' else '有避障'
                message=f'正在规划{label}路径；请勿同时操作官方 Execute。'
        super()._state(state,message,summary)

    def _invalidate(self):
        self.reviewed = None
        self.preview_visible = False
        self.cached.clear()
        self.previewed_modes.clear()
        self.client.clear_preview()

    def _ensure_scene(self):
        if self.draft is None:
            scene = load_scene(self.scene_path, self.client.settings['frame_id'])
            self.draft = deepcopy(scene)
            self.applied_scene = deepcopy(scene)
            self.scene_file = str(self.scene_path or '')
            self.scene_selected = next((o['id'] for o in scene['objects']), '')
            self.scene_revision += 1

    def _mode_scene(self):
        if self.comparison_enabled and self.selected_mode == 'baseline':
            return dict(enabled=True, objects=[])
        return deepcopy(self.applied_scene)

    def _load(self, path):
        if not self._building:
            self.cached.clear()
            self.previewed_modes.clear()
        # Point reloads must not silently overwrite an unsaved, applied scene draft.
        scene_path, self.scene_path = self.scene_path, None
        try:
            super()._load(path)
        finally:
            self.scene_path = scene_path
        self._ensure_scene()
        validate_scene(self.draft, self.client.settings['frame_id'])
        self.scene = self._mode_scene()
        for mode,(reviewed,_,epoch) in list(self.cached.items()):
            if (reviewed['config']!=self.config or reviewed['model_sha256']!=self.client.model_sha256
                    or epoch!=self.client.activity_generation):
                self.cached.pop(mode)
                self.previewed_modes.discard(mode)

    def submit(self, command, path=''):
        command = command.strip().lower()
        with self.lock:
            mode_command=command in ('plan_baseline','plan_avoidance','preview_baseline','preview_avoidance',
                                     'execute_baseline','execute_avoidance')
            if command in ('plan', 'execute') or mode_command:
                if self.scene_dirty or self.scene_editing:
                    return False, '请先应用场景并关闭编辑模式，再规划或执行。'
            if mode_command:
                if not self.comparison_enabled: return False, '当前会话没有双路径对比。'
                if self.config_path is None: return False, '请先加载配置。'
            if command.startswith('execute_') and mode_command:
                mode=command.removeprefix('execute_')
                if getattr(self.client,'task_gate_closed',False): return False, '请先恢复控制，再重新规划并执行。'
                if mode not in self.cached or mode not in self.previewed_modes:
                    return False, '请先规划并预览对应路径。'
            accepted,message=super().submit(command, path)
            if accepted and command in ('manual','recover'):
                self.cached.clear()
                self.previewed_modes.clear()
            return accepted,message

    def _plan(self):
        if self.scene_dirty or self.scene_editing:
            raise RuntimeError('场景尚未应用，或仍在编辑模式')
        self.cached.clear()
        self.previewed_modes.clear()
        if not self.comparison_enabled:
            return super()._plan()
        self._building = True
        try:
            for mode in ('baseline', 'avoidance'):
                self.selected_mode = mode
                super()._plan()
                self.preview_log.write('preview_mode.json', dict(mode=mode, scene=self.scene))
                self.cached[mode] = (deepcopy(self.reviewed), self.preview_log, self.preview_epoch)
            baseline = self.cached['baseline'][0]['initial']
            if self.cached['baseline'][0]['config'] != self.cached['avoidance'][0]['config']:
                raise RuntimeError('两套规划期间点位配置变化，请重新规划')
            if max(abs(a-b) for a,b in zip(baseline, self.reviewed['initial'])) > self.config['settings']['start_tolerance_rad']:
                raise RuntimeError('两套规划期间实测起点变化，请重新规划')
            self._building = False
            self._select_preview('avoidance')
        except BaseException:
            self._invalidate()
            # Never leave an unintended obstacle-free scene after failed comparison.
            if not self.client.cancel_requested.is_set() and self.applied_scene is not None:
                self.selected_mode = 'avoidance'
                self.scene = self._mode_scene()
                self.client.scene.apply(self.scene)
            raise
        finally:
            self._building = False

    def _select_preview(self, mode):
        if mode not in self.cached:
            raise RuntimeError('该路径没有有效规划，请先规划对应路径')
        reviewed, log, epoch = self.cached[mode]
        self.reviewed = None
        self.preview_visible = False
        self.client.begin_round()
        q = self.client.current().q
        if (reviewed['config']!=self.config or reviewed['model_sha256']!=self.client.model_sha256
                or epoch != self.client.activity_generation or max(abs(a-b) for a,b in zip(q, reviewed['initial'])) > self.config['settings']['start_tolerance_rad']):
            self._invalidate()
            raise RuntimeError('实测起点或运动状态变化，请重新规划')
        self.selected_mode = mode
        self.scene = self._mode_scene()
        applied = self.client.scene.apply(self.scene)
        if any(part['scene'] != fingerprint(applied) for part in reviewed['segments']):
            self._invalidate()
            raise RuntimeError('场景已变化，请重新规划')
        self.reviewed, self.preview_log, self.preview_epoch = deepcopy(reviewed), log, epoch
        log.event('SELECT_PREVIEW', mode=mode)
        self.client.preview_mode = mode
        self.client.publish_plan(reviewed['initial'], [s['trajectory'] for s in reviewed['segments']])
        if hasattr(self.client, 'publish_comparison_path'):
            self.client.publish_comparison_path(reviewed, mode)
        self.preview_visible = True
        self.previewed_modes.add(mode)
        label = '无避障（本任务障碍物已移除）' if mode == 'baseline' else '有避障'
        self._state('READY', f'当前预览：{label}。执行按钮只执行当前这套已预览轨迹。')

    def _execute(self):
        try:
            return super()._execute()
        finally:
            self.cached.clear()  # An execution attempt consumes both previews.
            self.previewed_modes.clear()

    def _plan_mode(self, mode):
        self.cached.pop(mode,None)
        self.previewed_modes.discard(mode)
        self._building=True
        try:
            self.selected_mode=mode
            super()._plan(show_preview=False)
            self.preview_log.write('preview_mode.json',dict(mode=mode,scene=self.scene))
            for other,(reviewed,_,epoch) in list(self.cached.items()):
                if max(abs(a-b) for a,b in zip(reviewed['initial'],self.reviewed['initial']))>self.config['settings']['start_tolerance_rad']:
                    self.cached.pop(other)
                    self.previewed_modes.discard(other)
            self.cached[mode]=(deepcopy(self.reviewed),self.preview_log,self.preview_epoch)
            self.reviewed=None
            label='无避障' if mode=='baseline' else '有避障'
            self._state('PLANNED',f'{label}规划完成，请点击“预览{label}”。')
        except BaseException:
            self.reviewed=None
            self.preview_visible=False
            if self.config is None:
                self.cached.clear()
                self.previewed_modes.clear()
            if not self.client.cancel_requested.is_set() and self.applied_scene is not None:
                self.selected_mode='avoidance'
                self.scene=self._mode_scene()
                self.client.scene.apply(self.scene)
            raise
        finally:
            self._building=False

    def poll(self):
        if not self.comparison_enabled: return super().poll()
        with self.lock:
            if self.busy or not self.cached: return
            try:
                q=self.client.current().q
                changed=any(epoch!=self.client.activity_generation or
                    max(abs(a-b) for a,b in zip(q,reviewed['initial']))>self.config['settings']['start_tolerance_rad']
                    for reviewed,_,epoch in self.cached.values())
            except RuntimeError:
                changed=True
            if changed:
                self._invalidate()
                self._state('IDLE','当前位置或反馈已变化，两套缓存已作废。请重新规划。')

    def handle_command(self, command, path):
        if command in ('plan_baseline','plan_avoidance'):
            return self._plan_mode(command.removeprefix('plan_'))
        if command in ('execute_baseline','execute_avoidance'):
            mode=command.removeprefix('execute_')
            if mode not in self.previewed_modes: raise RuntimeError('请先预览对应路径')
            self._select_preview(mode)
            return self._execute()
        if command.startswith('preview_'):
            return self._select_preview(command.removeprefix('preview_'))
        self._ensure_scene()
        if command == 'scene_select':
            if path and path not in {o['id'] for o in self.draft['objects']}:
                raise ValueError('未找到该障碍物')
            # Selection only changes handles; it is not a scene/trajectory edit.
            self.scene_selected = path
            return
        if command == 'scene_save':
            if self.scene_dirty:
                raise RuntimeError('请先应用场景，再另存为')
            target = Path(path).expanduser().resolve()
            if target.suffix.lower() not in ('.yaml', '.yml'):
                raise ValueError('场景文件必须以 .yaml 或 .yml 结尾')
            # Save As never overwrites a previous scene, even outside the GUI.
            with target.open('x', encoding='utf-8') as stream:
                yaml.safe_dump(self.draft, stream, allow_unicode=True, sort_keys=False)
            self.scene_file = str(target)
            self._state('IDLE', f'场景已另存为 {target}')
            return
        self.client.begin_round()  # Reject edits during any observed controller motion.
        self._invalidate()
        if command == 'scene_load':
            draft = load_scene(Path(path).expanduser(), self.client.settings['frame_id'])
            self.draft = draft
            self.scene_selected = next((o['id'] for o in draft['objects']), '')
            self.scene_file = str(Path(path).expanduser().resolve())
            self.scene_dirty = True
            self.scene_editing = True
        elif command == 'scene_edit':
            self.scene_editing = path == 'true'
        elif command == 'scene_apply':
            scene = validate_scene(self.draft, self.client.settings['frame_id'])
            scene['enabled'] = True
            self.client.scene.apply(scene)
            self.applied_scene = deepcopy(scene)
            self.draft = deepcopy(scene)
            self.scene = deepcopy(scene)
            self.selected_mode = 'avoidance'
            self.scene_dirty = False
            self.scene_editing = False
        elif command in ('scene_upsert', 'scene_delete', 'scene_pose'):
            draft = deepcopy(self.draft)
            objects = {o['id']: o for o in draft['objects']}
            if command == 'scene_delete':
                if path not in objects: raise ValueError('未找到该障碍物')
                del objects[path]
            else:
                value = json.loads(path)
                if command == 'scene_pose':
                    if not self.scene_editing: raise RuntimeError('请先开启场景编辑模式')
                    if value['id'] != self.scene_selected: raise RuntimeError('该物体已不再选中，请拖动当前选中物体')
                    if value['id'] not in objects: raise ValueError('未找到该障碍物')
                    obj = deepcopy(objects[value['id']])
                    obj['position_m'] = value['position_m']
                    obj['orientation_xyzw'] = value['orientation_xyzw']
                else:
                    obj = value
                objects[obj['id']] = obj
            draft['objects'] = list(objects.values())
            draft['enabled'] = True
            self.draft = validate_scene(draft, self.client.settings['frame_id'])
            if command == 'scene_delete':
                if self.scene_selected not in objects:
                    self.scene_selected = next(iter(objects), '')
            else:
                self.scene_selected = obj['id']
            self.scene_dirty = True
            self.scene_editing = True
        else:
            raise ValueError('未知场景操作')
        self.scene_revision += 1
        self._state('IDLE', '场景草稿已改变，请应用后重新规划。' if self.scene_dirty else '场景已更新；旧预览已作废。')
