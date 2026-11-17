#!/usr/bin/env python3
"""Compare virtual paths, or use the same-window interactive control and scene panel."""
import argparse
import math
import os
from pathlib import Path
import select
import signal
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent


def main(argv=None, *, client_type=None, runtime_type=None, task_type=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=HERE/'config/config.yaml')
    parser.add_argument('--scene', type=Path, default=HERE/'config/scene.yaml')
    parser.add_argument('--domain', type=int, default=87, help='Isolated preview domain (default 87)')
    parser.add_argument('--interactive', action='store_true', help='Same-window real-feedback comparison/control panel')
    parser.add_argument('--control-domain', type=int, default=43, help='Interactive control domain (default 43)')
    parser.add_argument('--control-stack', choices=['auto', 'attach', 'start'], default='auto',
                        help='Interactive multi_point stack policy')
    parser.add_argument('--rviz', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--once', action='store_true', help='Run comparison then exit')
    parser.add_argument('--self-test', action='store_true', help='Also verify rejection of a colliding goal')
    parser.add_argument('--clearance', type=float, default=.025, help='Planning obstacle margin in metres')
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=ROOT/'outputs/tasks/obstacle_avoidance')
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232: parser.error('--domain must be in [0,232]')
    if not 0 <= args.control_domain <= 232: parser.error('--control-domain must be in [0,232]')
    if args.control_domain == args.domain: parser.error('预览域和实机控制域必须不同')
    if not math.isfinite(args.clearance) or not 0 <= args.clearance <= .2:
        parser.error('--clearance must be in [0,0.2] metres')
    if args.interactive:
        if args.once or args.self_test or not args.rviz or any((client_type, runtime_type, task_type)):
            parser.error('--interactive 不能与纯预览专用选项或自定义任务混用')
        from scripts.task.obstacle_avoidance.control import main as control
        return control(['--config', str(args.config), '--scene', str(args.scene),
                        '--domain', str(args.control_domain), '--stack', args.control_stack,
                        '--output-dir', str(args.output_dir/'control')]
                       + (['--check-config'] if args.check_config else []))
    # Keep config checking available without importing ROS.
    from scripts.task.obstacle_avoidance.inputs import load_inputs
    try:
        config, scene = load_inputs(args.config, args.scene)
    except (ValueError, OSError) as exc: parser.error(str(exc))
    if args.check_config:
        print(f'配置通过：{len(config["points"])} 个点，{len(config["points"])-1} 段，{len(scene["objects"])} 个障碍物；尚未验证可达性/碰撞。')
        return 0
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    from scripts.task.common.run_logger import RunLog
    from scripts.task.obstacle_avoidance.client import PlanningClient
    from scripts.task.obstacle_avoidance.runtime import PreviewRuntime
    from scripts.task.obstacle_avoidance.task import ObstacleTask
    log = RunLog(args.output_dir)
    client = runtime = task = None
    handlers = {sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    shutdown = False
    def stop(*_):
        nonlocal shutdown
        shutdown = True
        if client: client.cancel_requested.set()
    try:
        for sig in handlers: signal.signal(sig, stop)
        client = (client_type or PlanningClient)(config['settings'], config['points'][0]['joints_rad'])
        for sig in handlers: signal.signal(sig, stop)
        if shutdown: raise InterruptedError('Startup cancelled')
        runtime = (runtime_type or PreviewRuntime)(client, ROOT, log.directory)
        runtime.prepare(args.rviz)
        task = (task_type or ObstacleTask)(client, args.config, args.scene, log.directory, args.clearance)
        print(f'纯规划演示 | ROS 域 {args.domain} | 不连接驱动、不执行实机运动\n日志：{log.directory}',flush=True)
        def compare(): return task.compare(invalid_goal_test=args.self_test)
        try: compare()
        except Exception as exc:
            if args.once or isinstance(exc,InterruptedError): raise
            print(f'规划未完成：{exc}',flush=True)
        if args.once: return 0
        print('r=重读配置并对比规划 | b=红色碰撞基准 | v/Enter=绿色绕障 | s=更新场景 | c=清除场景 | q=退出',flush=True)
        stdin_open = True
        while not shutdown:
            if runtime.rviz_process is not None and runtime.rviz_process.poll() is not None:
                if runtime.rviz_process.returncode:
                    raise RuntimeError('RViz 异常退出，请查看 rviz.log')
                break
            if not stdin_open:
                client.cancel_requested.wait(.2)
                continue
            if not select.select([sys.stdin],[],[],.2)[0]: continue
            line = sys.stdin.readline()
            if not line:
                stdin_open = False
                if not args.rviz: break
                continue
            command = line.strip().lower()
            try:
                if command == 'q': break
                elif command == 'r': compare()
                elif command == 'b': task.show(True)
                elif command in ('v',''): task.show(False)
                elif command == 's': task.load_scene()
                elif command == 'c': task.clear_scene()
                else: print('命令：r / b / v / s / c / q',flush=True)
            except Exception as exc:
                if isinstance(exc,InterruptedError): raise
                print(f'操作未完成：{exc}',flush=True)
    except (Exception,KeyboardInterrupt) as exc:
        print(f'演示退出：{exc}\n日志：{log.directory}',file=sys.stderr)
        return 130 if isinstance(exc,(InterruptedError,KeyboardInterrupt)) else 1
    finally:
        for sig in handlers: signal.signal(sig, stop)
        try:
            try:
                if runtime: runtime.close()
            finally:
                if client: client.close()
        finally:
            for sig,handler in handlers.items(): signal.signal(sig, handler)
    return 130 if shutdown else 0


if __name__ == '__main__': raise SystemExit(main())
