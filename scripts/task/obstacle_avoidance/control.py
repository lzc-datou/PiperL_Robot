#!/usr/bin/env python3
"""Open the existing multi-point panel for obstacle-aware real-arm control."""
import argparse
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
HERE = Path(__file__).resolve().parent


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument('--domain', type=int, default=43)
    args, remaining = parser.parse_known_args(argv)
    if not 0 <= args.domain <= 232:
        parser.error('--domain must be in [0,232]')
    if '--execute' in remaining:
        parser.error('请在面板中重新预览后确认执行；此入口不支持自动执行')
    from scripts.task.multi_point.run import main as multi_point
    defaults = ['--config', str(HERE/'config/config.yaml'),
                '--scene', str(HERE/'config/scene.yaml'), '--rviz', '--interactive',
                '--output-dir', str(ROOT/'outputs/tasks/obstacle_avoidance/control')]
    # Pure file checks and help must not initialize a ROS context or driver.
    if any(flag in remaining for flag in ('--check-config', '--help', '-h')):
        return multi_point(defaults+remaining)
    from scripts.task.obstacle_avoidance.control_client import ObstacleControlClient
    from scripts.task.obstacle_avoidance.control_session import ObstacleControlSession
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['PIPERL_ROS_DOMAIN_ID'] = str(args.domain)
    print(f'实机控制面板 | ROS 域 {args.domain} | 重新读取真实反馈并规划；不自动使能或执行。', flush=True)
    return multi_point(defaults+remaining, client_type=ObstacleControlClient, session_factory=ObstacleControlSession)


if __name__ == '__main__':
    raise SystemExit(main())
