# PiperL_Robot

基于 GitHub 全新获取的 AgileX 官方 ROS 2、SDK 和模型建立的 Piper-L 基线。

- [多点运动任务](scripts/task/multi_point/README.md)
- [RViz 六轴伺服（纯仿真 / 实机）](scripts/task/servo_tracking/README.md)
- [项目结构与控制接口：模块职责、算法输入输出、参数优先级](docs/项目结构与控制接口.md)
- [独立笛卡尔控制参数：P / PI / PD / FF+P / FF+PD](scripts/task/servo_tracking/config/control.yaml)
- [多点、多障碍物路径对比](scripts/task/obstacle_avoidance/README.md)
- [实用工具目录](scripts/tools/README.md)
- [飞智冰原狼 2 手柄工具](scripts/tools/gamepad_control/README.md)（离线界面、只读 evdev、MoveIt Servo/mock 仿真；实体手柄与实机仍待验收）
- [手动命令：急停/使能、监控、范围记录、CAN、驱动与 MoveIt](docs/手动命令.md)
- [官方 ROS README](piperl_ws/src/agx_arm_ros/README.md)
- [官方 CAN 配置](piperl_ws/src/agx_arm_ros/docs/CAN_USER.md)
- [伺服参数与关节限速](scripts/task/servo_tracking/README.md#文件与参数)
- [源码提交记录](config/source_versions.json)

环境：Ubuntu 24.04 / ROS 2 Jazzy / 系统 Python 3.12；共享系统 ROS/APT，项目独立 `.venv-piperl`（system-site-packages）与 `piperl_ws`。不使用 Conda，不修改 .bashrc 或全局 pip。默认 Domain 43、仅本机发现；每个终端都通过 run.sh 进入。

官方源码位于 `piperl_ws/src/agx_arm_ros/`，项目环境入口位于 `scripts/piperl/`，任务与工具分别位于 `scripts/task/`、`scripts/tools/`。Servo 的 [核心控制器](scripts/task/servo_tracking/cartesian_control.py) 无 ROS 依赖，输入已验证的目标/实测 TCP 位姿与单调时间，输出基坐标系 TCP 速度；算法参数独立于硬件和任务参数。完整目录及接口约定见上方架构文档，现有多点和避障任务保留原官方执行链路。

首次联网准备或在新目录重新创建：

```bash
# 先克隆自己的项目仓库，递归获取锁定版本的官方依赖
git clone --recurse-submodules https://github.com/lzc-datou/PiperL_Robot.git
cd PiperL_Robot
bash scripts/piperl/bootstrap.sh --online
bash scripts/piperl/run.sh python scripts/piperl/doctor.py
```

本项目通过两个 Git 子模块引用官方 SDK（`external/pyAgxArm`）和 ROS 仓库（`piperl_ws/src/agx_arm_ros`）；模型 `agx_arm_urdf` 是 ROS 仓库内部的子模块。主仓库记录具体提交版本，官方源码保留各自的来源和许可证。若克隆时没有递归获取，进入项目后执行 `git submodule update --init --recursive`。从 GitHub 下载主仓库 ZIP 不包含子模块源码，应使用上述克隆方式。

拉取主项目更新后执行 `git submodule update --init --recursive`，使依赖与主项目锁定版本一致。主动升级官方依赖时，需同步更新 `config/source_versions.json`，运行 `python3 scripts/piperl/verify_sources.py`，再一起提交版本记录和子模块指针。子模块内的修改需要在对应仓库提交并推送到可访问的远程仓库；若使用自己的 fork，应同时更新 `.gitmodules` 中的 URL。

主仓库忽略虚拟环境、构建与运行输出、本机工具配置、wheel 缓存和 `.deps` 依赖缓存；这些内容按环境准备说明重新生成。

已有源码、wheel 缓存和系统依赖时可用 `bootstrap.sh --offline`。移机/改路径不要复制虚拟环境和 build/install/log，应重新创建与构建。系统 ROS、NumPy、SciPy、PyYAML、colcon 等来自 APT；当前版本另存 config/system_packages.txt，不把共享系统依赖宣称为完全隔离。

正常使用：

```bash
bash scripts/piperl/activate_can.sh
bash scripts/piperl/start_moveit.sh
```

也可使用统一操作入口，持续保留官方单点控制和多点任务：

```bash
bash scripts/piperl/build.sh
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py --rviz --interactive
```

已有驱动/MoveIt/RViz 会复用。可加 `--config 路径` 启动时加载任务；终端菜单和独立 RViz“多点任务”面板共用状态。见[操作说明](scripts/task/multi_point/README.md)。

默认 Piper-L、无末端、普通 move_j、SDK 默认速度、关闭自动使能，使用者按手动命令使能。CAN 配置会改变系统接口，ROS Domain 不能隔离同一实体 CAN；旧项目必须先退出。

仅迁移旧项目的被动监控与范围记录工具，保留其不发送 CAN 的回归测试；未迁移旧轨迹执行器、模拟运行入口、SDK/URDF 补丁或业务任务。官方自带 GenericSystem 是其控制链的一部分，保持原样。

```bash
bash scripts/piperl/run.sh --python-only python -m unittest discover -s tests -v
bash scripts/piperl/run.sh python scripts/piperl/verify_sources.py
```

源码重建：按 source_versions.json 中 URL 克隆，检出对应 commit；ROS 仓库执行 git submodule update --init --recursive。不要用 submodule update --remote 改变锁定模型。
尚未实机验收：Piper-L 模型与现场零位/控制器限位的一致性、实际到位和停止行为。

验证结果：[软件基线报告](outputs/validation/report.json)；[Servo 验证记录](scripts/task/servo_tracking/VALIDATION.md)。
