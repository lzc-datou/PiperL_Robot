"""Launch the isolated simulation, the camera-only preview or the official robot stack."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from scripts.task.visual_servo.geometry import DEFAULT, calibration, settings


def main():
    parser = argparse.ArgumentParser(description='PiPER-L 棋盘格视觉伺服')
    parser.add_argument('--mode', choices=('sim', 'observe', 'real'), default='sim')
    parser.add_argument('--config', type=Path, default=DEFAULT)
    parser.add_argument('--handeye-dir', type=Path)
    parser.add_argument('--sim-input', choices=('pose', 'image'), default=None)
    parser.add_argument('--no-rviz', action='store_true', help='用于模拟验收')
    args = parser.parse_args()
    try:
        cfg = settings(args.config)
        if args.mode != 'sim' and args.sim_input:
            parser.error('--sim-input 只用于仿真')
        if args.mode == 'real' and args.no_rviz:
            parser.error('实机跟踪要求 RViz 面板在线')
        if args.mode == 'observe':
            from scripts.task.visual_servo.observe import run
            return run(cfg, args.handeye_dir)
        calibration(cfg, args.handeye_dir, sim=args.mode == 'sim')
    except (ValueError, OSError, KeyError) as exc:
        parser.error(str(exc))
    from scripts.task.servo_tracking.run import main as launch_stack
    launch_args = ['--mode', args.mode, '--config', cfg['servo_config']]
    if args.no_rviz:
        launch_args.append('--no-rviz')
    visual = dict(visual_config=str(args.config.resolve()), sim_input=args.sim_input or 'pose')
    if args.handeye_dir is not None and args.mode == 'real':
        visual['handeye_dir'] = str(args.handeye_dir.resolve())
    return launch_stack(launch_args, visual=visual)


if __name__ == '__main__':
    raise SystemExit(main())
