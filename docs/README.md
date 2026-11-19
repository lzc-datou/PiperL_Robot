# 文档目录

按要完成的操作选择指南。CAN 连接、使能、重置和阻尼急停统一使用控制面板，运动会话的启动与结束按对应指南操作。

## 操作指南

| 要做的事 | 文档 |
| --- | --- |
| 连接手柄，在三维窗口验证单轴与笛卡尔控制 | [手柄连接与仿真](../scripts/tools/gamepad_control/README.md) |
| 仿真确认后操作实机 | [手柄首次控制实机](../scripts/tools/gamepad_control/REAL_MACHINE.md) |
| 准备 CAN 与机械臂使能 | [控制面板](../scripts/tools/control_panel/README.md) |
| 鼠标拖动末端或单点伺服 | [RViz 六轴伺服](../scripts/task/servo_tracking/README.md) |
| 使用官方单点规划界面 | [单点 Plan/Execute](../scripts/task/single_point/README.md) |
| 逐点规划、预览和执行 | [多点任务](../scripts/task/multi_point/README.md) |
| 比较路径与编辑障碍物 | [避障任务](../scripts/task/obstacle_avoidance/README.md) |
| 查看关节、保存点位和观察范围 | [监控工具](../scripts/tools/monitoring/README.md) |
| D405 检测与手眼标定采样 | [眼在手上标定](../scripts/tools/handeye_calibration/README.md) |

实机栈启动前先用面板连接、使能并等待确认。急停恢复时先结束旧会话和驱动，再面板重置、使能，最后重新启动控制。回零是单独的运动操作。

## 环境、参考与维护

- [安装、构建、更新与移机](环境准备.md)
- [手柄两种控制模式、参数与状态](../scripts/tools/gamepad_control/REFERENCE.md)
- [项目结构、控制链与进程所有权](项目结构与控制接口.md)
- [鼠标 Servo 算法、I/O 与调参](Servo控制接口.md)
- [任务目录](../scripts/task/README.md)、[工具目录](../scripts/tools/README.md)与[共用模块](../scripts/task/common/README.md)
- [RViz 场景编辑与面板](../scripts/task/multi_point/rviz_panel/README.md)
- [常用操作速查](手动命令.md)
- [测试与集成检查入口](../tests/README.md)
- [只读文档审计](../scripts/tools/maintenance/README.md)

## 当前验证与历史

- [手柄当前验证矩阵](../scripts/tools/gamepad_control/VALIDATION.md)
- [Servo 当前验证矩阵](../scripts/task/servo_tracking/VALIDATION.md)
- [手柄原开发方案，2026-11-14](history/gamepad_control/2026-11-14-development-plan.md)
- [手柄历史验证，归档于 2026-11-17](history/gamepad_control/2026-11-17-validation-history.md)
- [Servo 历史验证，归档于 2026-11-17](history/servo_tracking/2026-11-17-validation-history.md)

历史方案和旧参数不作为当前操作要求。本机生成日志、截图和 JSON 不随 Git 克隆分发；检查范围和复现入口以对应记录为准。

## 资料

- [PiPER-L 公开资料与型号适用关系](PiPER-L公开资料.md)
- [RealSense D405 公开资料](RealSense-D405公开资料.md)
- [官方源码版本记录](../config/source_versions.json)与[官方 ROS 说明](../piperl_ws/src/agx_arm_ros/README.md)
