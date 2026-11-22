"""Sequential stop-and-dwell task; the official stack owns trajectory execution."""
from copy import deepcopy
import time
from scripts.task.common.planning_scene import fingerprint
from scripts.task.common.joint_bounds import JointBoundsError, StartStateBoundsError
from scripts.task.common.robot_feedback import FeedbackUnavailableError
from scripts.task.common.schema import validate_snapshot_model
from rosidl_runtime_py.convert import message_to_ordereddict

class MultiPointTask:
    def __init__(self,client,config,scene,log,execute=False):
        self.client,self.config,self.scene,self.log,self.execute=client,config,scene,log,execute
        self.reviewed_plan=None
        self.publish_preview=True

    def run(self,reviewed=None):
        self.reviewed_plan=None
        result={'success':False,'mode':'execute' if self.execute else 'plan-only','segments':[],'physical_arrival_checked':False}
        settings=self.config['settings']
        self.log.write('config.yaml',self.config)
        self.log.write('scene_config.yaml',self.scene)
        try:
            self.client.tick()
            if 'capture' in self.config:
                validate_snapshot_model(self.config,self.client.urdf,self.client.model_sha256)
            if reviewed is not None:
                if not self.execute or reviewed['config']!=self.config:
                    raise RuntimeError('Reviewed plan does not match execution configuration; replan')
                live_model=self.client.model()
                if reviewed['model_sha256']!=self.client.model_sha256 or list(live_model)!=[self.client.urdf,self.client.srdf]:
                    raise RuntimeError('Robot model changed since preview; replan')
                applied=self.client.scene.get()
                current_scene=fingerprint(applied)
                if any(segment['scene']!=current_scene for segment in reviewed['segments']):
                    raise RuntimeError('Planning scene changed since preview; replan')
            else:
                applied=self.client.scene.apply(self.scene)
            self.log.write('scene_initial.json',message_to_ordereddict(applied))
            self.log.write('model.json',dict(sha256=self.client.model_sha256,urdf=self.client.urdf,srdf=self.client.srdf))
            initial=self.client.stable(timeout=5,health=self.execute)
            result['initial_joints_rad']=list(initial.q)
            result['reviewed_execution']=reviewed is not None
            # Preserve the unmodified measurement even when validation aborts before planning.
            self.log.write('initial_feedback.json', dict(joints_rad=list(initial.q),
                stamp=getattr(initial,'stamp',None), received_monotonic=getattr(initial,'received',None),
                limits_rad=getattr(self.client,'limits',{})))
            try:
                self.client.check_state(initial.q)
            except JointBoundsError as exc:
                raise StartStateBoundsError(exc) from exc
            if reviewed is not None:
                if max(abs(a-b) for a,b in zip(initial.q,reviewed['initial']))>settings['start_tolerance_rad']:
                    raise RuntimeError('Real start changed since preview; replan')
                points=deepcopy(reviewed['points'])
            else:
                initial_pose=self.client.fk(initial.q,settings['frame_id'],settings['tcp_link'])
                self.log.write('initial_pose.json',initial_pose)
                points=deepcopy(self.config['points'])
                for point in points:
                    if point['type']=='pose':
                        point['position_m']=[a+b for a,b in zip(point['position_m'],settings['approach_offset_m'])]
                if settings['return_to_initial']:
                    points.append(dict(name='return_to_initial',type='joint',joints_rad=list(initial.q),hold_seconds=0))
            self.log.write('resolved_points.json',points)
            self.client.publish_targets(points)
            predicted=initial.q
            preview=[]
            prepared=[]
            for index,point in enumerate(points,1):
                self.client.tick(self.execute)
                before=self.client.scene.get()
                if reviewed is not None:
                    cached=reviewed['segments'][index-1]
                    if fingerprint(before)!=cached['scene']:
                        raise RuntimeError('Planning scene changed since preview; replan')
                    start,trajectory,endpoint,duration=cached['start'],cached['trajectory'],cached['endpoint'],cached['duration']
                    self.log.event('USING_REVIEWED_PLAN',point=point['name'])
                    self.client.publish_plan(start,[trajectory])
                else:
                    start=self.client.current().q if self.execute else predicted
                    self.log.event('PLANNING',point=point['name'],start_source='real_feedback' if self.execute or index==1 else 'predicted_endpoint')
                    # Persist the attempted goal and scene even when no trajectory is returned.
                    attempt=dict(index=index,point=deepcopy(point),start_joints_rad=list(start))
                    self.log.write(f'{index:03d}_request.json',attempt)
                    self.log.write(f'{index:03d}_scene.json',message_to_ordereddict(before))
                    try:
                        trajectory,endpoint,duration=self.client.plan(point,start,publish=self.execute)
                    except InterruptedError:
                        raise
                    except Exception as exc:
                        failure=dict(attempt,error=f'{type(exc).__name__}: {exc}')
                        if hasattr(exc,'details'):
                            failure['moveit_error']=deepcopy(exc.details)
                        result['planning_failure']=failure
                        self.log.write('planning_failure.json',failure)
                        raise RuntimeError(f'第 {index}/{len(points)} 点 {point["name"]} 规划失败：{exc}') from exc
                    finally:
                        diagnostics=getattr(self.client,'planning_diagnostics',None)
                        if diagnostics is not None:
                            self.log.write(f'{index:03d}_inspection.json',diagnostics)
                prepared.append(dict(start=tuple(start),trajectory=deepcopy(trajectory),endpoint=list(endpoint),
                                     duration=duration,scene=fingerprint(before)))
                segment=dict(index=index,point=point,start_joints_rad=list(start),planned_endpoint_rad=endpoint,
                             duration_seconds=duration,trajectory=message_to_ordereddict(trajectory),execution='not_sent')
                result['segments'].append(segment)
                self.log.write(f'{index:03d}_{point["name"]}.json',segment)
                self.log.write(f'{index:03d}_scene.json',message_to_ordereddict(before))
                if self.execute:
                    if fingerprint(before)!=fingerprint(self.client.scene.get()):
                        raise RuntimeError('Planning scene changed during planning; restart task to replan')
                    self.log.event('EXECUTING',point=point['name'])
                    segment['execution']='submitted'
                    self.client.execute(trajectory,start,duration)
                    self.log.event('WAITING_FOR_REAL_ARRIVAL',point=point['name'])
                    if point['type']=='pose':
                        actual,pose=self.client.stable_pose(point,endpoint)
                        segment['actual_pose']=pose
                    else:
                        actual=self.client.stable(endpoint)
                    # A reached planner endpoint must also satisfy the requested joint goal.
                    if point['type']=='joint' and max(abs(a-b) for a,b in zip(actual.q,point['joints_rad']))>settings['joint_tolerance_rad']:
                        raise RuntimeError('Actual joint position does not satisfy requested point')
                    segment.update(execution='action_and_real_arrival_confirmed',actual_joints_rad=list(actual.q))
                    self.log.event('DWELL',point=point['name'],seconds=point['hold_seconds'])
                    self.client.hold(endpoint,point['hold_seconds'],point)
                else:
                    predicted=endpoint
                    preview.append(trajectory)
                    segment['execution']='planned_only'
                self.log.write(f'{index:03d}_{point["name"]}.json',segment)
            if not self.execute:
                # Retain every segment for RViz subscribers that finish loading later.
                if self.publish_preview: self.client.publish_plan(initial.q,preview)
                self.reviewed_plan=dict(initial=tuple(initial.q),points=deepcopy(points),segments=prepared,
                                        config=deepcopy(self.config),model_sha256=self.client.model_sha256)
            result.update(success=True,physical_arrival_checked=self.execute)
            self.log.event('DONE',mode=result['mode'])
            return result
        except BaseException as exc:
            result['error']=f'{type(exc).__name__}: {exc}'
            if isinstance(exc,(JointBoundsError,FeedbackUnavailableError)): result['error_details']=exc.details
            if self.execute:
                try: result['stop']=self.client.stop()
                except BaseException as stop_error: result['stop']={'error':str(stop_error),'position_stable_observed':False}
            # A full disk/broken log must never prevent cancellation and gate close.
            try: self.log.event('FAILED',error=result['error'])
            except Exception as log_error: print(f'Failure log unavailable: {log_error}',flush=True)
            raise
        finally:
            try:
                # Keep individual segment records consistent even on execution/arrival failure.
                for segment in result['segments']:
                    self.log.write(f'{segment["index"]:03d}_{segment["point"]["name"]}.json',segment)
                self.log.write('result.json',result)
            except Exception as log_error:
                if result['success']: raise
                print(f'Result log unavailable: {log_error}',flush=True)
