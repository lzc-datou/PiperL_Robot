"""Shared-format point records from one frozen set of measured joint angles."""
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import yaml
from scripts.task.common.schema import JOINTS, validate_config

@dataclass(frozen=True)
class Capture:
    q: tuple
    received: tuple
    captured_at: str
    monotonic: float

    @classmethod
    def from_feedback(cls,feedback,now,stale_after=0.5,max_skew=0.05):
        if any(v is None for v in feedback.degrees+feedback.received_at):
            raise ValueError('Position feedback incomplete')
        if any(not math.isfinite(v) for v in feedback.degrees+feedback.received_at):
            raise ValueError('Invalid position feedback')
        ages=[now-stamp for stamp in feedback.received_at]
        if min(ages)<0 or max(ages)>stale_after:
            raise ValueError('Position feedback stale')
        if max(feedback.received_at)-min(feedback.received_at)>max_skew:
            raise ValueError('Joint-pair feedback skew too large')
        return cls(tuple(math.radians(v) for v in feedback.degrees),tuple(feedback.received_at),
                   datetime.now().astimezone().isoformat(timespec='milliseconds'),now)


def documents(capture,pose=None,*,frame='base_link',tcp='tcp_link',model_sha256=None,number=1):
    metadata=dict(valid_at_capture=True,captured_at=capture.captured_at,joint_names=JOINTS,
                  joint_positions_rad=list(capture.q),position_age_ms=[(capture.monotonic-t)*1000 for t in capture.received],
                  pair_skew_ms=(max(capture.received)-min(capture.received))*1000,
                  frame_id=frame,tcp_link=tcp,model_sha256=model_sha256,source='passive_socketcan')
    joint=dict(name=f'point_{number:03d}_joint',type='joint',joints_rad=[round(v,8) for v in capture.q],hold_seconds=0.5)
    settings=dict(frame_id=frame,tcp_link=tcp)
    result={'joint':dict(version=1,settings=settings,capture=metadata,points=[joint])}
    if pose is not None:
        if pose['frame_id']!=frame: raise ValueError('FK frame mismatch')
        point=dict(name=f'point_{number:03d}_pose',type='pose',frame_id=frame,
                   position_m=[round(v,8) for v in pose['position_m']],
                   orientation_xyzw=[round(v,8) for v in pose['orientation_xyzw']],hold_seconds=0.5)
        result['pose']=dict(version=1,settings=deepcopy(settings),capture=deepcopy(metadata),points=[point])
    for document in result.values(): validate_config(document)
    return result


def save_snapshot(directory,records):
    # Validate everything before creating files; each capture is a new directory.
    for record in records.values(): validate_config(record)
    path=Path(directory)/datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    path.mkdir(parents=True,exist_ok=False)
    for kind,record in records.items():
        (path/(kind+'.yaml')).write_text(yaml.safe_dump(record,allow_unicode=True,sort_keys=False))
    return path


def point_text(record):
    point=record['points'][0]
    # Compact YAML list item; valid when pasted under task `points:` (indent by two spaces).
    title=f'- {{name: {point["name"]}, type: {point["type"]}, hold_seconds: 0.5,'
    if point['type']=='joint':
        return title+'\n   joints_rad: ['+', '.join(f'{v:.6f}' for v in point['joints_rad'])+']}'
    return (title+f' frame_id: {point["frame_id"]},\n'
            +'   position_m: ['+', '.join(f'{v:.6f}' for v in point['position_m'])+'],\n'
            +'   orientation_xyzw: ['+', '.join(f'{v:.6f}' for v in point['orientation_xyzw'])+']}')
