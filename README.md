# PiperL_Robot

PiPER-L 机械臂控制项目，基于 AgileX 官方 ROS 2、MoveIt 2 和 pyAgxArm。日常流程是：先用手柄操作虚拟机械臂，再用控制面板准备硬件，最后进入实机控制。

## 用手柄控制机械臂

1. 首次使用或换电脑，完成 [环境准备](docs/环境准备.md)。
2. 启动虚拟机械臂。在同一手柄窗口检查“单轴控制”“笛卡尔控制”和“笛卡尔单轴”的方向、LB/B、模式切换和断线停止。见 [手柄连接与仿真指南](scripts/tools/gamepad_control/README.md)。
3. 结束旧控制程序，使用 [CAN 控制面板](scripts/tools/control_panel/README.md) 连接 CAN，点击“机械臂使能”，等待反馈确认。
4. 再按 [首次实机操作](scripts/tools/gamepad_control/REAL_MACHINE.md) 启动手柄控制。从低速单轴开始，核对实物运动与停止，再使用笛卡尔控制。

默认单轴模式选中 J1。按住 LB 允许运动，松开 LB 停止；B 停止并锁定，恢复需明确操作。两种控制模式的映射、速度和切换条件见 [手柄参数参考](scripts/tools/gamepad_control/REFERENCE.md)。

面板使能在实机控制栈启动前完成，使官方驱动启动时读取到已使能状态。急停恢复先结束旧控制会话和驱动，再由面板重置、使能，随后重启控制。面板可以保留用于阻尼急停，运动会话中不要用它回零或重置。回零不是启动必需步骤。

## 其他操作入口

| 目标 | 指南 | 硬件范围 |
| --- | --- | --- |
| 检查手柄、操作虚拟机械臂 | [手柄控制](scripts/tools/gamepad_control/README.md) | `input`、`sim` 不连接机械臂 |
| 手柄操作实机 | [首次实机操作](scripts/tools/gamepad_control/REAL_MACHINE.md) | `real` 连接实机 |
| CAN 连接、使能、重置、阻尼急停 | [控制面板](scripts/tools/control_panel/README.md) | 正常模式操作 CAN；`--demo` 离线 |
| 鼠标拖动末端、单点伺服 | [RViz 六轴伺服](scripts/task/servo_tracking/README.md) | 按 `sim`、`real` 区分 |
| 官方单点 Plan/Execute | [单点规划](scripts/task/single_point/README.md) | 实机入口连接；另有离线入口 |
| 多点连续规划与执行 | [多点任务](scripts/task/multi_point/README.md) | 实机反馈与官方执行链 |
| 路径对比、虚拟障碍物编辑 | [避障任务](scripts/task/obstacle_avoidance/README.md) | 纯预览不连接；交互执行连接 |
| 关节监控、范围记录、点位保存 | [监控工具](scripts/tools/monitoring/README.md) | 被动读取硬件反馈 |
| D405 检测预览与眼在手上标定 | [手眼标定，含 `preview` 统一启动](scripts/tools/handeye_calibration/README.md) | 预览只启动相机与检测；实机采样不自动运动 |

同一机械臂同时只运行一套运动控制栈。不同 ROS Domain 仍共享物理 CAN。软件停止和面板阻尼急停均不代替现场物理急停。

## 环境与文档

项目使用 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、独立 `.venv-piperl` 和 `piperl_ws`。各终端通过 `scripts/piperl/run.sh` 使用项目环境。默认 ROS Domain 43、仅本机发现，手柄示例使用 Domain 64。

- [完整文档目录](docs/README.md)
- [安装、构建、更新与移机](docs/环境准备.md)
- [目录职责、控制链与配置归属](docs/项目结构与控制接口.md)
- [鼠标 Servo 算法与接口](docs/Servo控制接口.md)
- [官方源码版本](config/source_versions.json)与[官方 ROS 说明](piperl_ws/src/agx_arm_ros/README.md)

## 验证范围

当前版本的检查结果见 [手柄验证矩阵](scripts/tools/gamepad_control/VALIDATION.md)与 [Servo 验证矩阵](scripts/task/servo_tracking/VALIDATION.md)。历史结果保留原版本和检查范围。

模型动画、手柄读数或软件检查不能代替实机验收。映射、所用连接方式的断线行为、实机平顺性、停止响应及模型与现场零位的一致性，需结合当前版本与实物确认。本机日志和验证产物不随源码克隆分发。
