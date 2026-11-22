"""Read-only FK and terminal snapshot keys."""
from collections import deque
import select
import sys
import termios
import time
import tty
from pathlib import Path
from xml.etree import ElementTree as ET

from .point_snapshot import Capture, documents, point_text, save_snapshot


class LocalFK:
    def __init__(self, root, frame, tcp):
        from scripts.piperl.deployment import load_driver_config
        from scripts.task.servo_tracking.core import RobotModel, load_config

        driver = load_driver_config()
        if (driver.get('arm_type'), driver.get('effector_type')) != ('piper_l', 'none'):
            raise ValueError('local FK requires the PiPER-L flange model (piper_l, none)')
        root = Path(root)
        description = (root / 'piperl_ws/src/agx_arm_ros/src/agx_arm_description'
                       / 'agx_arm_urdf/piper_l/urdf/piper_l_description.urdf')
        task_config = root / 'scripts/task/servo_tracking/config/config.yaml'
        offset = load_config(task_config)['tcp_offset']

        # Match agx_arm.urdf.xacro for piper_l / none, using the same arm
        # description and TCP configuration as the gamepad launch.
        robot = ET.parse(description).getroot()
        ET.SubElement(robot, 'link', name='tcp_link')
        joint = ET.SubElement(robot, 'joint', name='tcp_joint', type='fixed')
        ET.SubElement(joint, 'parent', link='link6')
        ET.SubElement(joint, 'child', link='tcp_link')
        ET.SubElement(joint, 'origin', xyz=' '.join(map(str, offset[:3])), rpy=' '.join(map(str, offset[3:])))

        try:
            self.model = RobotModel(ET.tostring(robot, encoding='unicode'), base=frame, tip=tcp)
        except (KeyError, ValueError) as exc:
            raise ValueError(f'unsupported frame/TCP for local FK ({frame} -> {tcp})') from exc
        self.frame = frame
        self.kinematics_sha256 = self.model.kinematics_sha256()

    def pose(self, capture):
        pose = self.model.pose(capture.q)
        return dict(frame_id=self.frame, position_m=pose[:3].tolist(), orientation_xyzw=pose[3:].tolist())


class ReadOnlyFK:
    def __init__(self,frame,tcp):
        from scripts.task.common.ros_context import RosContext
        self.context=RosContext('piper_monitor_fk')
        self.frame,self.tcp=frame,tcp
        self.pending=None
        self.model_future=None
        self.model_sha256=None
        self.next_model=0
        self.error='MoveIt FK unavailable'

    def poll(self):
        from rcl_interfaces.srv import GetParameters
        from scripts.task.common.ros_context import model_hash
        now=time.monotonic()
        if self.model_future and self.model_future.done():
            try:
                values=self.model_future.result().values
                if len(values)!=2 or any(v.type!=4 or not v.string_value for v in values): raise ValueError('Live model unavailable')
                self.model_sha256=model_hash(*[v.string_value for v in values])
                import xml.etree.ElementTree as ET
                links={node.get('name') for node in ET.fromstring(values[0].string_value).findall('link')}
                if not {self.frame,self.tcp}<=links: raise ValueError('Configured frame/TCP absent from model')
                self.error=''
            except Exception as exc:
                self.model_sha256=None
                self.error=str(exc)
            self.model_future=None
        if self.model_future and now-self.model_started>2:
            self.context.parameters.remove_pending_request(self.model_future)
            self.model_future=None
            self.model_sha256=None
            self.error='Model service timeout'
        if now>=self.next_model and self.model_future is None:
            self.next_model=now+5
            if self.context.parameters.service_is_ready():
                self.model_started=now
                self.model_future=self.context.parameters.call_async(GetParameters.Request(names=['robot_description','robot_description_semantic']))
            else:
                self.model_sha256=None
                self.error='Start official MoveIt for Cartesian coordinates'

    def submit(self,capture):
        self.poll()
        if self.pending is not None or not self.model_sha256 or not self.context.fk_client.service_is_ready(): return False
        from scripts.task.common.ros_context import fk_request
        self.pending=(self.context.fk_client.call_async(fk_request(capture.q,self.frame,self.tcp)),capture,time.monotonic(),self.model_sha256)
        return True

    def take(self):
        if self.pending is None: return None
        future,capture,started,model=self.pending
        if not future.done():
            if time.monotonic()-started>0.5:
                self.context.fk_client.remove_pending_request(future)
                self.pending=None
                self.error='FK request timed out'
            return None
        self.pending=None
        from scripts.task.common.ros_context import fk_value
        try:
            pose=fk_value(future.result(),self.frame,self.tcp)
            self.error=''
            return capture,pose,model
        except Exception as exc:
            self.error=str(exc)
            return None

    def close(self): self.context.close()

class PointPanel:
    def __init__(self, root, frame='base_link', tcp='tcp_link', snapshot_dir=None, fk_source='local'):
        self.frame,self.tcp=frame,tcp
        self.directory=Path(snapshot_dir or Path(root)/'outputs/points')
        self.local_fk = None
        self.fk = None
        self.fk_source = fk_source
        self.fk_error = ''
        self.note=''
        self.latest=None
        self.frozen=None
        self.pending_capture=None
        self.number=1
        self.history=deque()
        self.last_stamp=None
        self.terminal=None
        if fk_source == 'local':
            try:
                self.local_fk = LocalFK(root, frame, tcp)
            except (ImportError, OSError, ValueError, ET.ParseError) as exc:
                self.fk_error = f'Local FK unavailable: {exc}'
        elif fk_source == 'moveit':
            try:
                self.fk = ReadOnlyFK(frame, tcp)
            except Exception as exc:
                self.fk_error = f'MoveIt FK unavailable ({type(exc).__name__}); use run.sh without --python-only'
        else:
            raise ValueError(f'Unknown FK source: {fk_source}')
        if sys.stdin.isatty():
            self.terminal=termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

    def keys(self):
        if self.terminal is None: return ''
        result=''
        while select.select([sys.stdin],[],[],0)[0]:
            result+=sys.stdin.read(1)
        return result

    def stable(self,capture):
        stamp=min(capture.received)
        if self.last_stamp!=stamp:
            self.last_stamp=stamp
            self.history.append(capture)
        while len(self.history)>2 and self.history[1].monotonic<capture.monotonic-0.4:
            self.history.popleft()
        return (len(self.history)>=3 and capture.monotonic-self.history[0].monotonic>=0.3 and
                all(max(c.q[i] for c in self.history)-min(c.q[i] for c in self.history)<=0.003 for i in range(6)))

    def records(self, capture, pose=None, model=None, *, kinematics=None):
        return documents(capture, pose, frame=self.frame, tcp=self.tcp, model_sha256=model,
                         kinematics_sha256=kinematics, number=self.number)

    def freeze(self,records,save):
        self.frozen=records
        if save:
            path=save_snapshot(self.directory,records)
            self.note=f'Saved: {path}'
            self.number+=1
        else: self.note=''

    def render(self,feedback,now,stale_after):
        pressed=self.keys()
        if 'r' in pressed:
            self.frozen=None
            self.pending_capture=None
        capture=None
        feedback_error=""
        steady=False
        try:
            capture=Capture.from_feedback(feedback,now,stale_after)
            steady=self.stable(capture)
        except ValueError as exc:
            self.latest=None
            self.history.clear()
            self.last_stamp=None
            feedback_error=str(exc)
        if capture and self.local_fk:
            pose = self.local_fk.pose(capture)
            self.latest = (capture, self.records(capture, pose, kinematics=self.local_fk.kinematics_sha256))
        if self.fk:
            self.fk.poll()
            answer=self.fk.take()
            if capture and answer:
                sample,pose,model=answer
                if now-min(sample.received)<=stale_after:
                    records=self.records(sample,pose,model)
                    self.latest=(sample,records)
                    if self.pending_capture and sample is self.pending_capture[0]:
                        self.freeze(records,self.pending_capture[1])
                        self.pending_capture=None
        if self.pending_capture and now-min(self.pending_capture[0].received)>stale_after:
            self.pending_capture=None
            self.note='Capture expired before FK completed; retry'
        if 's' in pressed and self.frozen:
            self.freeze(self.frozen,True)
        elif ('p' in pressed or 's' in pressed) and capture:
            if not steady:
                self.note='Keep arm still for 0.3 s before recording'
            else:
                if self.fk and self.fk.model_sha256 and self.fk.context.fk_client.service_is_ready():
                    # Drop a live-display request, then compute the exact keypress capture.
                    if self.fk.pending:
                        self.fk.context.fk_client.remove_pending_request(self.fk.pending[0])
                        self.fk.pending=None
                    if self.fk.submit(capture): self.pending_capture=(capture,'s' in pressed)
                else:
                    self.freeze(self.latest[1] if self.local_fk else self.records(capture), 's' in pressed)
        if capture and not self.pending_capture and not self.local_fk:
            if self.fk: self.fk.submit(capture)
            # Joint-only display is available even without ROS; never pair it with unrelated FK.
            if (self.latest is None or not self.fk or not self.fk.model_sha256
                    or now-min(self.latest[0].received)>stale_after):
                self.latest=(capture,self.records(capture))
        records=self.frozen
        if not records and self.latest and now-min(self.latest[0].received)<=stale_after:
            records=self.latest[1]
        lines=['','POINTS  p: freeze  s: save  r: live | Ctrl+C: exit']
        if records:
            meta=records['joint']['capture']
            source = f' | FK {self.fk_source}' if 'pose' in records else ''
            lines.append(('FROZEN ' if self.frozen else 'LIVE ')+meta['captured_at']+f' | {self.frame} -> {self.tcp}'+source)
            lines.append(point_text(records['joint']))
            if 'pose' in records:
                lines.append(point_text(records['pose']))
            else:
                lines.append('# Cartesian unavailable; joint point only')
        else: lines.append('# No fresh complete position sample')
        # Feedback warnings describe this render only; recovery clears them.
        if feedback_error: lines.append(feedback_error)
        elif self.note: lines.append(self.note)
        elif self.fk_error: lines.append(self.fk_error)
        elif self.fk and self.fk.error: lines.append(self.fk.error)
        return '\n'.join(lines)

    def close(self):
        if self.terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,self.terminal)
        if self.fk: self.fk.close()
