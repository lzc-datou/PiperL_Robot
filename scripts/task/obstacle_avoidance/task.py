"""Compare a sequential reference path with a collision-checked obstacle-avoidance path."""
from copy import deepcopy
from rosidl_runtime_py.convert import message_to_ordereddict

from scripts.task.common.planning_scene import fingerprint
from scripts.task.common.moveit_client import PlanningError
from scripts.task.common.pose import pose_errors
from scripts.task.common.run_logger import RunLog
from scripts.task.common.schema import JOINTS, validate_snapshot_model
from .validation import inspect
from .inputs import load_inputs


def with_margin(scene, margin):
    expanded = deepcopy(scene)
    for obj in expanded['objects']:
        if obj['shape'] == 'box':
            obj['dimensions_m'] = [size+2*margin for size in obj['dimensions_m']]
        elif obj['shape'] == 'sphere': obj['dimensions_m'][0] += margin
        else:
            obj['dimensions_m'][0] += 2*margin
            obj['dimensions_m'][1] += margin
    return expanded



class ObstacleTask:
    def __init__(self, client, config_path, scene_path, directory, clearance=.025):
        self.client = client
        self.config_path, self.scene_path = config_path, scene_path
        self.directory = directory
        self.clearance = clearance
        self.baseline = self.avoidance = None
        self.scene_hash = None

    def invalidate(self):
        self.baseline = self.avoidance = None
        self.scene_hash = None
        self.client.clear_preview()

    def reload(self):
        self.invalidate()
        config, scene = load_inputs(self.config_path, self.scene_path)
        self.client.settings = deepcopy(config['settings'])
        self.client.connect_model()
        if 'capture' in config:
            validate_snapshot_model(config, self.client.urdf, self.client.model_sha256)
        for point in config['points']:
            if point['type'] == 'joint':
                self.client.check_joints(point['joints_rad'])
        if config['points'][0]['type'] == 'joint':
            self.client.start = tuple(config['points'][0]['joints_rad'])
        self.client.waypoint_names = [p['name'] for p in config['points']]
        self.config, self.scene = config, scene

    def tcp_pose(self, q):
        settings = self.config['settings']
        return self.client.fk(q, settings['frame_id'], settings['tcp_link'])

    def prepare_start(self):
        first = self.config['points'][0]
        if first['type'] == 'pose':
            start = self.client.solve_start_pose(first)
            self.check_endpoint(first, start)
            self.client.start = start

    def load_scene(self):
        self.reload()
        self.client.scene.apply(self.scene)
        self.prepare_start()
        print('场景已更新；r 重新规划对比。', flush=True)

    def clear_scene(self):
        self.invalidate()
        self.client.scene.apply(dict(enabled=True, objects=[]))
        print('已清除本任务障碍物，其他场景物体保留。', flush=True)

    def path(self, trajectory):
        joint = trajectory.joint_trajectory
        order = [joint.joint_names.index(n) for n in JOINTS]
        stride = max(1, len(joint.points)//80)
        selected = list(joint.points[::stride])
        if selected[-1] is not joint.points[-1]: selected.append(joint.points[-1])
        return [self.client.fk([p.positions[i] for i in order],
                    self.config['settings']['frame_id'], self.config['settings']['tcp_link'])['position_m']
                for p in selected]

    def plan(self, log, start, stage, point=None):
        # OMPL may find a geometric path which its response adapters reject.
        # Retry planning only, with unchanged goals, scene and tolerances.
        for attempt in range(1,6):
            self.client.tick()
            try:
                return self.client.plan(point or self.config['points'][1], start, publish=False)
            except PlanningError as exc:
                log.event('PLAN_REJECTED',stage=stage,attempt=attempt,error=exc.details)
                if attempt == 5: raise

    def plan_sequence(self, log, stage):
        start = list(self.client.start)
        trajectories = []
        for index, point in enumerate(self.config['points'][1:], 1):
            source = self.config['points'][index-1]['name']
            print(f'{stage}: {index}/{len(self.config["points"])-1} {source} → {point["name"]}', flush=True)
            log.event('SEGMENT', stage=stage, index=index, source=source, target=point['name'], start=start)
            try:
                trajectory, endpoint, _ = self.plan(log, start, stage, point)
                joint = trajectory.joint_trajectory
                order = [joint.joint_names.index(name) for name in JOINTS]
                actual_start = [joint.points[0].positions[i] for i in order]
                if max(abs(a-b) for a, b in zip(start, actual_start)) > 1e-6:
                    raise RuntimeError('规划段起点不连续')
                self.check_endpoint(point, endpoint)
            except InterruptedError:
                raise
            except Exception as exc:
                raise RuntimeError(f'{stage} 第 {index} 段 {source} → {point["name"]}: {exc}') from exc
            trajectories.append(trajectory)
            # Use the actual planned endpoint, never reset to the nominal waypoint.
            start = list(endpoint)
        saved = dict(
            point_names=[p['name'] for p in self.config['points']],
            trajectories=[message_to_ordereddict(t) for t in trajectories])
        # Retain the original single-trajectory artifact for two-point callers.
        log.write(f'{stage}_trajectory.json', saved if len(trajectories) > 1 else saved['trajectories'][0])
        return trajectories

    def check_endpoint(self, point, endpoint):
        settings = self.config['settings']
        if point['type'] == 'joint':
            if max(abs(a-b) for a, b in zip(point['joints_rad'], endpoint)) > settings['joint_tolerance_rad']:
                raise RuntimeError('规划终点超出目标关节容差')
            return
        distance, angle = pose_errors(self.tcp_pose(endpoint), point)
        if distance > settings['position_tolerance_m'] or angle > settings['orientation_tolerance_rad']:
            raise RuntimeError(f'{point["name"]}: TCP 终点超出位姿容差 ({distance*1000:.3f} mm, {angle:.6f} rad)')

    def inspect_sequence(self, trajectories):
        reports = []
        for index, trajectory in enumerate(trajectories, 1):
            report = inspect(self.client, trajectory, stop_on_collision=True)
            reports.append(dict(index=index, source=self.config['points'][index-1]['name'],
                                target=self.config['points'][index]['name'], **report))
        return dict(valid=all(r['valid'] for r in reports), segments=reports,
                    checked_samples=sum(r['checked_samples'] for r in reports),
                    continuous_collision_proof=False)

    def check_waypoints(self, stage):
        for index, point in enumerate(self.config['points']):
            q = self.client.start if index == 0 else point.get('joints_rad')
            if q is not None and not self.client.state_validity(q).valid:
                raise RuntimeError(f'{stage}: {point["name"]} 点与障碍物碰撞；请调整场景或点位')

    def baseline_scene(self):
        return dict(enabled=True, objects=[])

    def compare(self, invalid_goal_test=False):
        self.reload()
        log = RunLog(self.directory/'rounds')
        print(f'日志：{log.directory}', flush=True)
        result = dict(success=False, mode='preview_only', execution_available=False)
        client = self.client
        goal = self.config['points'][-1]
        result['point_names'] = [p['name'] for p in self.config['points']]
        result['segment_count'] = len(self.config['points'])-1
        try:
            log.write('config.yaml', self.config)
            log.write('scene.yaml', self.scene)
            log.write('model.json', dict(sha256=client.model_sha256, urdf=client.urdf, srdf=client.srdf))
            before = client.scene.apply(self.baseline_scene())
            clear_hash = fingerprint(before)
            self.prepare_start()
            log.event('BASELINE', message='在基准场景中按相同点序规划整条路径')
            baseline = self.plan_sequence(log, 'baseline')
            baseline_check = self.inspect_sequence(baseline)
            if not baseline_check['valid']:
                raise RuntimeError('无本任务障碍物时，基准路径仍碰撞；请检查其他场景物体')
            if fingerprint(client.scene.get()) != clear_hash:
                raise RuntimeError('基准规划期间场景变化，请重试')
            scene = client.scene.apply(self.scene)
            scene_hash = fingerprint(scene)
            log.write('scene_applied.json', message_to_ordereddict(scene))
            self.check_waypoints('原始场景')
            log.event('CHECK_BLOCKED_BASELINE')
            blocked = self.inspect_sequence(baseline)
            result['baseline_with_obstacle'] = blocked
            if blocked['valid']:
                raise RuntimeError('障碍物没有挡住本次基准路径，不能证明绕障；请调整场景后按 r')
            log.event('AVOIDANCE', message='按相同点序逐段自动规划绕行')
            planning_scene = with_margin(self.scene, self.clearance)
            log.write('planning_scene_with_margin.yaml', planning_scene)
            result['planning_margin_m'] = self.clearance
            client.scene.apply(planning_scene)
            margin_hash = fingerprint(client.scene.get())
            self.check_waypoints('增加留边后的场景')
            avoidance = self.plan_sequence(log, 'avoidance')
            if fingerprint(client.scene.get()) != margin_hash:
                raise RuntimeError('绕障规划期间场景变化，请重试')
            client.scene.apply(self.scene)
            avoided = self.inspect_sequence(avoidance)
            result['avoidance'] = avoided
            if not avoided['valid']:
                raise RuntimeError('绕障轨迹复查发现碰撞，拒绝展示为有效路径')
            endpoints = [p['position_m'] if p['type'] == 'pose' else self.tcp_pose(p['joints_rad'])['position_m']
                         for p in self.config['points']]
            if invalid_goal_test:
                goal_joints = goal.get('joints_rad')
                if goal_joints is None:
                    joint = avoidance[-1].joint_trajectory
                    goal_joints = [joint.points[-1].positions[joint.joint_names.index(name)] for name in JOINTS]
                result['invalid_goal_rejected'] = self.check_invalid_goal(goal_joints, endpoints[-1])
            baseline_path = [xyz for trajectory in baseline for xyz in self.path(trajectory)]
            avoidance_path = [xyz for trajectory in avoidance for xyz in self.path(trajectory)]
            if fingerprint(client.scene.get()) != scene_hash:
                raise RuntimeError('规划或复查期间场景变化，结果作废')
            self.baseline = (baseline if len(baseline) > 1 else baseline[0], baseline_path, endpoints)
            self.avoidance = (avoidance if len(avoidance) > 1 else avoidance[0], avoidance_path, endpoints)
            self.scene_hash = scene_hash
            self.show(False)
            result.update(success=True, baseline_without_obstacle=baseline_check,
                          endpoints_m=endpoints, scene_sha256=scene_hash)
            log.event('READY', checked_samples=avoided['checked_samples'])
            print('对比通过：基准路径被挡住，绕障路径复查通过。绿色为绕障 TCP 路径。', flush=True)
            return result
        except BaseException as exc:
            self.invalidate()
            result['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            # Even a failed comparison leaves the configured obstacles visible.
            try:
                if not client.cancel_requested.is_set(): client.scene.apply(self.scene)
            finally:
                log.write('result.json', result)

    def check_invalid_goal(self, goal, xyz):
        blocked_scene = deepcopy(self.scene)
        blocked_scene['objects'].append(dict(id='negative_test_goal', shape='box',
            dimensions_m=[.20, .20, .20], frame_id=self.config['settings']['frame_id'],
            position_m=list(xyz), orientation_xyzw=[0.,0.,0.,1.]))
        try:
            self.client.scene.apply(blocked_scene)
            if self.client.state_validity(goal).valid:
                raise RuntimeError('负例箱体未与目标机器人碰撞，验证未成立')
            try:
                point = self.config['points'][-1]
                if point['type'] == 'pose':
                    point = dict(name=point['name'], type='joint', joints_rad=goal)
                self.client.plan(point, self.client.start, publish=False)
            except RuntimeError as exc:
                if 'State is in collision' not in str(exc): raise
                return True
            raise RuntimeError('碰撞目标未被拒绝')
        finally:
            self.client.scene.apply(self.scene)

    def show(self, baseline=False):
        selected = self.baseline if baseline else self.avoidance
        if selected is None:
            raise RuntimeError('没有有效对比结果，请先按 r 规划')
        if fingerprint(self.client.scene.get()) != self.scene_hash:
            self.invalidate()
            raise RuntimeError('场景已变化，旧预览作废，请按 r')
        self.client.show(*selected, baseline=baseline)
        if baseline: print('红色：碰撞基准路径，仅用于对照，不可执行。', flush=True)
