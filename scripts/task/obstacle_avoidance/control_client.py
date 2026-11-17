"""Use the existing real-feedback task client, with extra path collision inspection."""
from scripts.task.common.moveit_client import MoveItClient, PlanningError
from scripts.task.common.planning_scene import fingerprint
from .client import PlanningClient
from .validation import inspect
from scripts.task.common.schema import JOINTS
from types import SimpleNamespace
from moveit_msgs.msg import PlannerParams
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams


class ObstacleControlClient(MoveItClient):
    state_validity = PlanningClient.state_validity
    planner_id = 'ObstacleControl'
    max_inspected_attempts = 5

    def __init__(self, settings):
        super().__init__(settings)
        # Do not replace another multi_point session's virtual objects.
        self.scene.prefix = 'obstacle_avoidance/'

    def connect_model(self, timeout=30):
        super().connect_model(timeout)
        self.configure_planner()

    def configure_planner(self):
        # Only this named planner is changed, not the default/group planner or
        # the shared pipeline's TOTG parameters used by other RViz sessions.
        setter=self.node.create_client(SetPlannerParams,'/set_planner_params')
        getter=self.node.create_client(GetPlannerParams,'/get_planner_params')
        expected={'type':'geometric::RRTConnect','range':'0.0','longest_valid_segment_fraction':'0.0001'}
        try:
            self.call(setter,SetPlannerParams.Request(pipeline_id='ompl',group=self.settings['group'],
                planner_config=self.planner_id,replace=True,
                params=PlannerParams(keys=list(expected),values=list(expected.values()))),tick=self.tick)
            actual=self.call(getter,GetPlannerParams.Request(pipeline_id='ompl',
                group=self.settings['group'],planner_config=self.planner_id),tick=self.tick).params
            if any(dict(zip(actual.keys,actual.values)).get(k)!=v for k,v in expected.items()):
                raise RuntimeError('无法确认避障专用规划器的碰撞检查精度')
        finally:
            self.node.destroy_client(setter)
            self.node.destroy_client(getter)

    def plan(self, point, start, *, publish=True):
        self.planning_diagnostics=[]
        before = fingerprint(self.scene.get())
        for attempt in range(1,self.max_inspected_attempts+1):
            self.tick()
            if fingerprint(self.scene.get()) != before:
                raise RuntimeError('避障规划或复查期间场景变化，请重新规划')
            try:
                trajectory, endpoint, duration = super().plan(point, start, publish=False)
            except PlanningError as exc:
                self.planning_diagnostics.append(dict(attempt=attempt,moveit_error=exc.details))
                if exc.details['code'] not in (-1,-2,-6,99999) or attempt==self.max_inspected_attempts:
                    raise
            else:
                report = inspect(self, trajectory, stop_on_collision=True)
                self.planning_diagnostics.append(dict(attempt=attempt,inspection=report))
                if fingerprint(self.scene.get()) != before:
                    raise RuntimeError('避障规划或复查期间场景变化，请重新规划')
                if report['valid']:
                    if publish:
                        self.publish_plan(start, [trajectory])
                    return trajectory, endpoint, duration
                if attempt==self.max_inspected_attempts:
                    raise RuntimeError(f'{point["name"]}: {attempt} 次规划后仍有轨迹采样碰撞；请查看本段 inspection 日志')
            print(f'[RETRY] {point["name"]} 路径未通过检查，重新规划 {attempt+1}/{self.max_inspected_attempts}',flush=True)

    def publish_comparison_path(self, reviewed, mode):
        trajectories = [s['trajectory'] for s in reviewed['segments']]
        path = []
        for trajectory in trajectories:
            joint = trajectory.joint_trajectory
            order = [joint.joint_names.index(n) for n in JOINTS]
            selected = list(joint.points[::max(1, len(joint.points)//60)])
            if selected[-1] is not joint.points[-1]: selected.append(joint.points[-1])
            path.extend(self.fk([p.positions[i] for i in order], self.settings['frame_id'],
                                self.settings['tcp_link'])['position_m'] for p in selected)
        endpoints = [self.fk(q, self.settings['frame_id'], self.settings['tcp_link'])['position_m']
                     for q in [reviewed['initial']] + [s['endpoint'] for s in reviewed['segments']]]
        display = SimpleNamespace(start=reviewed['initial'], settings=self.settings,
            publish_plan=self.publish_plan, markers=self.targets,
            baseline_label='BASELINE',
            waypoint_names=['START'] + [p['name'] for p in reviewed['points']])
        PlanningClient.show(display, trajectories, path, endpoints, baseline=(mode == 'baseline'))
