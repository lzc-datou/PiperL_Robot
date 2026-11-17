"""Persistent console and RViz API use the same command queue."""
import select
import shlex
import sys
import time
from .scene_session import SceneSession as TaskSession
from .session_ros import SessionAPI

MENU='命令：l 路径=加载配置 | p=重新规划 | e=确认实机执行 | Enter=重播 | s=停止任务 | m=单点操作 | r=恢复控制 | q=退出'


def run_session(client, runtime, log, args, shutdown, *, session_factory=None):
    session=(session_factory or TaskSession)(client,log.directory,args.scene)
    api=None
    try:
        api=SessionAPI(client,session)
        print(MENU,flush=True)
        if getattr(session, 'comparison_enabled', False):
            print('双路径：p=规划两套 | b=预览无避障 | v=预览有避障 | e=执行当前已预览路径',flush=True)
        print('RViz: Panels → Add New Panel → piper_multi_point_rviz/TaskPanel（外部窗口只需添加一次）。',flush=True)
        if args.config:
            session.submit('load_plan',str(args.config))
        while not shutdown.is_set():
            try:
                session.poll()
                if not sys.stdin.isatty() or not select.select([sys.stdin],[],[],0.1)[0]:
                    time.sleep(0.05)
                    continue
                line=sys.stdin.readline()
                if not line: break
                parts=shlex.split(line)
                key=parts[0].lower() if parts else ''
                if key=='q': break
                if key in ('h','help','?'):
                    print(MENU,flush=True);continue
                if key=='l':
                    if len(parts)!=2:
                        print('用法：l /路径/config.yaml（含空格的路径加引号）',flush=True);continue
                    command,path='load',parts[1]
                else:
                    command={'p':'plan','e':'execute','':'replay','s':'stop','m':'manual','r':'recover',
                             'b':'preview_baseline','v':'preview_avoidance'}.get(key,'')
                    path=''
                accepted,message=session.submit(command,path)
                if not accepted: print(message,flush=True)
            except KeyboardInterrupt:
                shutdown.set()
                client.cancel_requested.set()
                break
            except ValueError as exc:
                print(f'命令格式错误：{exc}',flush=True)
    finally:
        print('正在退出：取消当前任务并等待清理完成。',flush=True)
        try:
            session.close()
        finally:
            if api is not None: api.close()
