# 实用工具分类

工具按用途放入子目录，不直接堆放在 `scripts/tools/` 根目录。新增类别时按实际用途创建，并在此登记。每个工具目录提供独立 README.md，说明功能、运行条件、使用方法、参数及输出。

| 分类 | 内容 |
| --- | --- |
| [monitoring：监控工具](monitoring/README.md) | 六轴位置、电机速度、反馈 age 监控，关节范围记录，以及任务格式点位快照 |
| [control_panel：CAN 控制面板](control_panel/README.md) | 五按钮底层 CAN 工具：连接、使能、回零、阻尼急停、重置；支持端口选择、自动接入和离线演示 |
| [gamepad_control：手柄控制工具](gamepad_control/README.md) | 飞智冰原狼 2、PyQt5 单页窗口、只读 evdev、MoveIt Servo/mock 仿真；实体手柄映射与实机未验收 |

环境准备、构建和官方驱动启动入口仍位于 `scripts/piperl/`。
完整操作命令见 [手动命令](../../docs/手动命令.md)。
