"""Own a ROS launch group and reclaim it when the Qt launcher disappears."""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time

from scripts.task.common.process_cleanup import stop_process_group


def start_stack(command, logfile, logs):
    """Return a supervisor and a pipe whose lifetime belongs only to the UI."""
    read_fd, write_fd = os.pipe()
    try:
        process = subprocess.Popen(
            [sys.executable, '-m', __name__, '--lifeline-fd', str(read_fd),
             '--logs', str(logs), '--', *command],
            stdin=subprocess.DEVNULL, stdout=logfile, stderr=subprocess.STDOUT,
            pass_fds=(read_fd,), start_new_session=True,
            cwd=Path(__file__).resolve().parents[3])
    except BaseException:
        os.close(write_fd)
        raise
    finally:
        os.close(read_fd)
    return process, write_fd


def stop_stack(process, lifeline, logs):
    """Allow the supervisor to finish its own bounded child-group teardown."""
    try:
        os.write(lifeline, b'Q')
    except BrokenPipeError:
        pass
    finally:
        os.close(lifeline)
    # The pipe also works before the supervisor has installed signal handlers.
    # Sending SIGINT immediately after Popen can kill it during Python startup.
    deadline = time.monotonic() + 13.
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(.05)
    supervisor = stop_process_group(process, timeouts=(1., 1., 1.))
    report_path = logs/'stack_cleanup.json'
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding='utf-8'))
    else:
        report = {'remaining_pids': [], 'errors': ['未收到后台栈清理报告']}
    report['supervisor_cleanup'] = supervisor
    (logs/'cleanup.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    if report['remaining_pids'] or report['errors'] or supervisor['remaining_pids'] or supervisor['errors']:
        raise RuntimeError(f'后台栈清理不完整，请检查 {logs}/cleanup.json')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lifeline-fd', type=int, required=True)
    parser.add_argument('--logs', type=Path, required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('missing stack command')
    reason = None

    def request_stop(signum, frame):
        nonlocal reason
        if reason is None:
            reason = signal.Signals(signum).name

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    process = None
    try:
        # The supervisor alone owns this child. The UI never has to discover
        # or guess which ROS processes belong to its run.
        process = subprocess.Popen(command, start_new_session=True)
        (args.logs/'stack_process.json').write_text(json.dumps({
            'supervisor_pid': os.getpid(), 'pid': process.pid, 'command': command,
        }, ensure_ascii=False, indent=2), encoding='utf-8')
        while reason is None:
            if process.poll() is not None:
                reason = 'stack_exited'
                break
            readable, _, _ = select.select([args.lifeline_fd], [], [], .1)
            if readable:
                data = os.read(args.lifeline_fd, 1)
                if data == b'Q':
                    reason = 'ui_shutdown'
                elif not data:
                    reason = 'ui_process_exited'
    finally:
        try:
            if process is not None:
                report = stop_process_group(process)
            else:
                report = {'pid': None, 'remaining_pids': [], 'errors': ['后台进程未能启动'], 'returncode': None}
            report['reason'] = reason or 'supervisor_error'
            for name in ('stack_cleanup.json', 'cleanup.json'):
                (args.logs/name).write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        finally:
            os.close(args.lifeline_fd)
    if report['remaining_pids'] or report['errors']:
        return 1
    return (process.returncode or 0) if reason == 'stack_exited' else 0


if __name__ == '__main__':
    raise SystemExit(main())
