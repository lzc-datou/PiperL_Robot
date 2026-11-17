"""Nonblocking FK and terminal snapshot keys. Never sends motion/scene commands."""
from collections import deque
import select
import sys
import termios
import time
import tty
from pathlib import Path
from .point_snapshot import Capture,documents,save_snapshot,point_text

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
    def __init__(self,root,frame='base_link',tcp='tcp_link',snapshot_dir=None):
        self.frame,self.tcp=frame,tcp
        self.directory=Path(snapshot_dir or Path(root)/'outputs/points')
        self.fk=None
        self.note=''
        self.latest=None
        self.frozen=None
        self.pending_capture=None
        self.number=1
        self.history=deque()
        self.last_stamp=None
        self.terminal=None
        try:
            self.fk=ReadOnlyFK(frame,tcp)
        except Exception as exc:
            self.note=f'Cartesian unavailable: {type(exc).__name__}; use run.sh without --python-only'
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

    def records(self,capture,pose=None,model=None):
        return documents(capture,pose,frame=self.frame,tcp=self.tcp,model_sha256=model,number=self.number)

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
            feedback_error=str(exc)
        if self.fk:
            self.fk.poll()
            answer=self.fk.take()
            if answer:
                sample,pose,model=answer
                if now-min(sample.received)<=stale_after:
                    self.latest=self.records(sample,pose,model)
                    if self.pending_capture and sample is self.pending_capture[0]:
                        self.freeze(self.latest,self.pending_capture[1])
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
                    self.freeze(self.records(capture), 's' in pressed)
        if capture and not self.pending_capture:
            if self.fk: self.fk.submit(capture)
            # Joint-only display is available even without ROS; never pair it with unrelated FK.
            if self.latest is None or not self.fk or not self.fk.model_sha256:
                self.latest=self.records(capture)
        records=self.frozen or self.latest
        if records and not self.frozen:
            # Latest FK may be older than this CAN update; reject it if its source expired.
            stamp=records['joint']['capture']['captured_at']
            from datetime import datetime
            if (datetime.now().astimezone()-datetime.fromisoformat(stamp)).total_seconds()>stale_after:
                records=None
        lines=['','POINTS  p: freeze  s: save  r: live | Ctrl+C: exit']
        if records:
            meta=records['joint']['capture']
            lines.append(('FROZEN ' if self.frozen else 'LIVE ')+meta['captured_at']+f' | {self.frame} -> {self.tcp}')
            lines.append(point_text(records['joint']))
            if 'pose' in records:
                lines.append(point_text(records['pose']))
            else:
                lines.append('# Cartesian unavailable; joint point only')
        else: lines.append('# No fresh complete position sample')
        # Feedback warnings describe this render only; recovery clears them.
        if feedback_error: lines.append(feedback_error)
        elif self.note: lines.append(self.note)
        elif self.fk and self.fk.error: lines.append(self.fk.error)
        return '\n'.join(lines)

    def close(self):
        if self.terminal is not None:
            termios.tcsetattr(sys.stdin.fileno(),termios.TCSADRAIN,self.terminal)
        if self.fk: self.fk.close()
