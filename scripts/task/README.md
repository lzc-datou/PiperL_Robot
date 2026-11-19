# 任务目录

每个任务使用独立子目录和 README，包含入口、任务流程、配置与使用说明；多点/避障复用的接口放在 common。全项目职责见 [项目结构与控制接口](../../docs/项目结构与控制接口.md)。

目录格式约定：

```text
scripts/task/<任务名>/
├── README.md
├── run.py
├── task.py                 # 任务流程（按需要提供）
├── config/
│   ├── config.yaml         # 默认点位/参数
│   ├── scene.yaml          # 障碍物场景
│   ├── control.yaml        # 独立算法参数（Servo）
│   └── <配置变体>.yaml
└── assets/                 # 模型、图片等资源（按需要提供）
```

默认路径由入口脚本的位置解析，不依赖调用目录；用户通过 `--config`、`--scene`、Servo 的 `--control-config` 传入的路径按调用目录解释。变体配置也放在各任务 `config/`。多点/避障日志和快照位于 `outputs/tasks/<任务名>/`，Servo 位于 `outputs/servo_tracking/`。

| 目录 | 用途 |
| --- | --- |
| [single_point](single_point/README.md) | 官方单点 Plan/Execute 操作说明与离线模型入口 |
| [multi_point](multi_point/README.md) | 持续操作入口：终端 + 独立 RViz 面板、任务切换、逐点到位、虚拟障碍物 |
| [obstacle_avoidance](obstacle_avoidance/README.md) | 多点、多障碍物双路径规划/执行、同窗口单点控制与场景编辑 |
| [common](common/README.md) | 点位格式、官方 MoveIt 接口、真实反馈、场景、日志和启动管理 |
| [servo_tracking](servo_tracking/README.md) | XYZ + 姿态连续伺服、单点/恢复/回零与绿色工作空间；纯仿真/实机独立启动 |

任务使用官方轨迹执行链路，不自行发送 CAN 指令或做轨迹插值。实机任务启动前，先用 [控制面板](../tools/control_panel/README.md) 完成 CAN 连接和使能确认；任务不自动使能。手柄操作从 [工具入口](../tools/gamepad_control/README.md) 开始，专用后端位于 Servo 包中。

Servo 按三层分工：`cartesian_control.py` 负责无 ROS 的控制数学，`node.py` 负责目标验证和算法调用，`guard.py` / `joint_motion.py` 负责控制许可与官方执行。算法配置位于 [control.yaml](servo_tracking/config/control.yaml)，硬件、反馈、规划和关节约束位于 [config.yaml](servo_tracking/config/config.yaml)；输入输出、五种算法和同增益对照见 [Servo 控制接口](../../docs/Servo控制接口.md#核心算法输入与输出)。

启动：`bash scripts/piperl/run.sh python scripts/task/multi_point/run.py --rviz --interactive`。每轮结束可重新规划，驱动与 RViz 不重启。
