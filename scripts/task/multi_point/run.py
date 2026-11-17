#!/usr/bin/env python3
"""Run sequential targets through official MoveIt; default is planning only."""
import argparse
from pathlib import Path
import signal
import threading
from copy import deepcopy
import sys
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from scripts.task.common.schema import load_config,load_scene,DEFAULTS

def main(argv=None, *, client_type=None, session_factory=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,help='Optional startup task for --interactive; otherwise defaults to config/config.yaml')
    parser.add_argument('--scene',type=Path,help='Optional scene.yaml; omission preserves existing scene')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--execute',action='store_true',help='Execute using official controller; never enables the arm')
    mode.add_argument('--interactive',action='store_true',help='Persistent terminal + RViz task session; load, plan, execute and repeat')
    parser.add_argument('--stack',choices=['auto','attach','start'],default='auto',help='Reuse existing stack or start official stack when absent')
    parser.add_argument('--rviz',action=argparse.BooleanOptionalAction,default=False,help='Reuse/open RViz and keep preview available after success; Enter replays, q or Ctrl+C exits')
    parser.add_argument('--check-config',action='store_true',help='Validate files only; no ROS or CAN access')
    parser.add_argument('--output-dir',type=Path,default=ROOT/'outputs/tasks/multi_point')
    args=parser.parse_args(argv)
    if args.interactive and not args.rviz:
        parser.error('--interactive requires --rviz')
    if args.interactive and not args.check_config:
        # A session may start with no task; the panel/terminal can load one later.
        config=dict(settings=deepcopy(DEFAULTS))
        scene=None
        try:
            from piper_multi_point_rviz.srv import TaskCommand
        except ImportError:
            parser.error('Build the task panel first: bash scripts/piperl/build.sh')
    else:
        args.config=args.config or Path(__file__).parent/'config/config.yaml'
        try:
            config=load_config(args.config)
            scene=load_scene(args.scene,config['settings']['frame_id'])
        except (OSError,ValueError) as exc:
            parser.error(str(exc))
    if args.check_config:
        print(f'Configuration OK: {len(config["points"])} points; obstacles={len(scene["objects"])} enabled={scene["enabled"]}')
        return 0
    from scripts.task.common.moveit_client import MoveItClient
    from scripts.task.common.runtime import Runtime
    from scripts.task.common.run_logger import RunLog
    from scripts.task.multi_point.task import MultiPointTask
    log=RunLog(args.output_dir)
    print(f'Output: {log.directory}',flush=True)
    client=None
    runtime=None
    shutdown=threading.Event()
    old_handlers={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    def terminate(*_):
        shutdown.set()
        if client is not None:
            client.cancel_requested.set()
    try:
        for sig in old_handlers: signal.signal(sig,terminate)
        client=(client_type or MoveItClient)(config['settings'])
        for sig in old_handlers: signal.signal(sig,terminate)
        if shutdown.is_set(): raise InterruptedError('Startup cancelled')
        runtime=Runtime(client,ROOT,log.directory)
        runtime.session_panel=args.interactive
        runtime.prepare(args.stack,args.rviz)
        if args.interactive:
            from scripts.task.multi_point.console import run_session
            if session_factory is None:
                run_session(client,runtime,log,args,shutdown)
            else:
                run_session(client,runtime,log,args,shutdown,session_factory=session_factory)
            return 0
        task=MultiPointTask(client,config,scene,log,args.execute)
        task.run()
        if args.rviz:
            runtime.wait_for_rviz()
        return 0
    except (Exception,KeyboardInterrupt) as exc:
        print(f'Task stopped: {exc}\nSee {log.directory}',file=sys.stderr)
        if not (log.directory/'result.json').exists():
            log.write('result.json',dict(success=False,phase='startup',error=f'{type(exc).__name__}: {exc}',motion_submitted=False))
        return 130 if isinstance(exc,(KeyboardInterrupt,InterruptedError)) else 1
    finally:
        # Repeated Ctrl+C must not interrupt cancellation, group teardown or ROS close.
        for sig in old_handlers: signal.signal(sig,terminate)
        try:
            try:
                if runtime: runtime.close()
            finally:
                if client: client.close()
        finally:
            for sig,handler in old_handlers.items(): signal.signal(sig,handler)

if __name__=='__main__': raise SystemExit(main())
