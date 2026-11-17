"""Linux process-group teardown for children launched with start_new_session=True."""
import os
from pathlib import Path
import signal
import time


def live_group_members(pgid):
    """Zombies have already stopped; their parent/init is responsible for reaping."""
    members=[]
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            # comm can contain spaces or parentheses, so split after its final ')'.
            fields=path.read_text().rsplit(')',1)[1].split()
            if int(fields[2])==pgid and int(fields[3])==pgid and fields[0] not in ('Z','X'):
                members.append(int(path.parent.name))
        except (FileNotFoundError,ProcessLookupError):
            continue
    return sorted(members)


def stop_process_group(process, timeouts=(6.0,3.0,2.0)):
    """Wait for the whole owned group, even after the launch process has exited."""
    pgid=process.pid  # Runtime.spawn always creates a new session/group.
    if pgid==os.getpgrp():
        raise RuntimeError('Refusing to stop the task process group itself')
    report=dict(pid=pgid,signals=[],remaining_pids=[],errors=[])
    for sig,timeout in zip((signal.SIGINT,signal.SIGTERM,signal.SIGKILL),timeouts):
        process.poll()  # Reap the direct child, without using its exit as completion.
        if not live_group_members(pgid):
            break
        try:
            os.killpg(pgid,sig)
            report['signals'].append(sig.name)
        except ProcessLookupError:
            pass  # Group exited between observation and signal delivery.
        except OSError as exc:
            report['errors'].append(f'{sig.name}: {exc}')
        deadline=time.monotonic()+timeout
        while True:
            process.poll()
            if not live_group_members(pgid) or time.monotonic()>=deadline:
                break
            time.sleep(min(0.05,max(0,deadline-time.monotonic())))
    report['returncode']=process.poll()
    report['remaining_pids']=live_group_members(pgid)
    return report
