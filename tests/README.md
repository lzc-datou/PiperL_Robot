# 验证入口

本目录区分纯数学回归、假服务和界面回归、隔离仿真集成、性能对照。选择与改动相关的入口；本页不表示这些验证已在当前版本运行。

## 不启动机械臂控制栈的回归

| 类别 | 代表入口 | 运行条件与范围 |
| --- | --- | --- |
| 控制数学和边界 | `test_cartesian_control.py`、`test_servo_damped_velocity.py`、`test_joint_bounds.py`、`test_gamepad_core.py`、`test_gamepad_modes.py`、`test_gamepad_motion.py` | 纯数值或文件输入，无 ROS 节点、SDK 和 CAN |
| 假服务和状态机 | `test_servo_guard_recovery.py`、`test_servo_joint_motion.py`、`test_gamepad_backend.py`、`test_task_ros_messages.py` | 使用 ROS 消息类型和假服务，需要项目 ROS 环境，实际请求由测试夹具接管；手柄用例不初始化 ROS 节点 |
| Python 界面和生命周期 | `test_control_panel_ui.py`、`test_gamepad_input_lifecycle.py`、`test_gamepad_stack_cleanup.py`、`test_gamepad_ui_modes.py` | Qt 离屏窗口、假设备或自有测试子进程，不控制机械臂 |
| Qt C++ 界面 | `test_rviz_layout.py`、`test_scene_editor_ui.py`、`test_motion_controls_ui.py`、`test_scene_marker_display.py` | 编译并运行 `qt/` 中的对应用例，需要 Qt 构建工具；部分用例还需要 ROS/RViz 开发环境 |
| ROS 桥与状态队列 | `test_gamepad_ros_bridge.py` | 独立且无控制栈的 Domain 下构造和销毁 ROS 节点，验证 20 Hz 刷新处理 50 Hz 输入和各 10 Hz 状态；只发布测试状态话题，不发送运动命令或控制请求；需要本机 DDS 通信条件 |

在项目根目录按模块运行，例如：

```bash
bash scripts/piperl/run.sh --python-only python -m unittest tests.test_gamepad_core -v
bash scripts/piperl/run.sh python -m unittest tests.test_servo_guard_recovery -v
```

`test_*.py` 中的导入依赖不同，缺少 ROS 或 Qt 时可能跳过部分用例。查看实际结果中的通过、失败和跳过数量，不能把跳过当作通过。

## 隔离仿真集成

下列入口会创建或操作 ROS 仿真节点，不能对运行中的实机任务执行。按各脚本头部和所属工具的验证记录准备独立 ROS Domain。

| 入口 | 前置条件与范围 |
| --- | --- |
| `gamepad_sim_integration.py` | 自建纯模拟 Servo/Guard/JTC 栈，审计 GenericSystem 且无实机驱动；验证单轴与笛卡尔共 24 方向的 Jog、Servo 输出和模拟反馈/FK、切换残留、选轴边沿、LB/B/失联锁定，以及独立近边界场景的向外受限与反向退回；当前结果见[手柄验证矩阵](../scripts/tools/gamepad_control/VALIDATION.md) |
| `servo_integration.py`、`servo_goal_preview_integration.py` | 接入已经独立启动的 `--mode sim` 栈，验证目标、预览和反馈流程 |
| `servo_rotation_translation_integration.py` | 自建 Domain 91 仿真，验证旋转和平移顺序、边界恢复与动作交接 |
| `servo_live_drag_integration.py` | 自建 Domain 94 仿真，对实际 MoveIt IK 注入测试延迟 |
| `servo_zeroedge_mock_real.py` | 固定 Domain 89，从测试启动清单移除硬件驱动并审计允许启动的进程，使用测试反馈运行生产 real 分支；不调用 SDK/CAN |
| `servo_zeroedge_ik_probe.py` | 固定 Domain 89，在同一无驱动模拟栈中比较 IK 种子，不调用运动启动、规划执行接口 |

`servo_singularity_integration.py` 和 `servo_modes_integration.py` 保留历史场景及参数分支。其中奇异起点拒绝、自动准备位的旧断言与当前 DLS 控制行为不同，不能作为当前版本的通用验收入口。历史脚本没有因目录整理而删除或修改；选用前先核对[Servo 验证记录](../scripts/task/servo_tracking/VALIDATION.md)。

模拟反馈、虚拟关节移动和软件停止验证都不能证明实体手柄映射、无线失联行为或实机动态性能。

手柄集成需要选择未占用的 Domain，例如：

```bash
PIPERL_ROS_DOMAIN_ID=76 bash scripts/piperl/run.sh python tests/gamepad_sim_integration.py
```

脚本固定使用 `mode:=sim`、`gamepad_input_source:=fixture`、`rviz:=false`；发现已有 `/piper_servo` 栈便退出。每轮在 `outputs/gamepad_control/two_modes_<时间>_d<Domain>_<进程号>/` 保存 `summary.json`、两次启动日志和进程清理报告。边界场景只修改该输出目录里的仿真配置副本。

## 纯数学性能对照

`servo_control_benchmark.py` 比较控制算法；`servo_release_braking_benchmark.py` 比较释放后的制动策略。它们使用理想 TCP 速度对象，不包含关节几何、碰撞、CAN、固件或电机动力学。输出位置及模型条件见[Servo 控制接口](../docs/Servo控制接口.md)。

## 文件与记录

`qt/` 保存 C++ 界面用例，`test_*.py` 是回归入口，其余集成和 benchmark 脚本需主动选择。共用测试夹具由相关测试模块导入。

生成报告和截图保存在 `outputs/`，由对应运行记录注明版本和限制。用户点位、场景和历史验证记录不作为测试缓存清理。
