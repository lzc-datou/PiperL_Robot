# 实用工具目录

工具按用途放在独立子目录，每类 README 提供运行条件、入口、参数与输出。环境与构建入口保留在 `scripts/piperl/`。

| 工具 | 用途 |
| --- | --- |
| [gamepad_control](gamepad_control/README.md) | 单关节点动、笛卡尔组合与单轴点动，支持实体输入、可见仿真与实机操作 |
| [control_panel](control_panel/README.md) | 硬件准备：CAN 连接、使能、重置、回零与阻尼急停 |
| [monitoring](monitoring/README.md) | CAN 关节和 TCP 位姿监控、范围记录、点位保存及只读手柄链路采样 |
| [handeye_calibration](handeye_calibration/README.md) | `preview` 一条命令启动 D405、棋盘格或 ArUco 检测及结果窗口；眼在手上标定采样 |
| [maintenance](maintenance/README.md) | 第一方文档链接、生成物与过期操作入口的只读审计 |
| [ui](ui/README.md) | 独立 Python Qt 窗口共用的字体、色彩与控件样式 |

日常硬件操作统一使用控制面板，先连接并确认使能，再启动实机运动栈。手柄仿真不需要 CAN 或使能。完整入口见 [项目文档索引](../../docs/README.md)。
