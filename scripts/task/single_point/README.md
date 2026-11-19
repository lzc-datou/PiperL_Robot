# 官方单点规划与控制

本目录提供 AgileX 官方 MoveIt 2 + RViz 操作说明，不新增规划执行程序。底层使用官方 ROS 驱动和 pyAgxArm，SDK 本身不是图形规划界面。

## 打开实机规划界面

先完成 [环境准备](../../../docs/环境准备.md)，结束其他实机控制程序。按 [控制面板说明](../../tools/control_panel/README.md) 连接 CAN、点击“机械臂使能”并等待六轴确认，再启动：

```bash
bash scripts/piperl/start_moveit.sh
```

入口同时启动官方驱动、MoveIt 和 RViz，不再单独运行驱动。参数来自 [baseline.json](../../../config/baseline.json)，默认 PiPER-L、无末端、关闭自动使能。启动会初始化 SDK，不是离线预览。

## 规划与执行

1. 等待 MotionPlanning 面板和模型加载，核对新鲜实机反馈与模型姿态。
2. 在 **MotionPlanning → Planning** 选择 `arm`，起点设为 `<current>`。
3. 拖动交互标记，点击 **Plan**，检查轨迹与现场空间。
4. 确认状态正常后点击 **Execute**。目标、起点、模型或场景变化后重新规划。

Plan 只预览，Execute 会控制实机。Executed 不等于实测到位，需对照反馈与实物。保持面板只用于必要的阻尼急停，不在运动中回零或重置。急停恢复时先结束旧控制会话和驱动，经面板恢复使能后再重启规划入口。

## 离线模型与规划

退出该 Domain 的所有实机控制程序，再运行：

```bash
bash scripts/piperl/run.sh ros2 launch agx_arm_moveit demo.launch.py arm_type:=piper_l effector_type:=none follow:=false
```

此方式不启动实机驱动，不需要面板连接或使能。同一 Domain 的已有驱动仍可能消费控制话题，不能仅凭 `follow:=false` 判断与实机隔离。

更多参数见 [官方 MoveIt 说明](../../../piperl_ws/src/agx_arm_ros/src/agx_arm_moveit/README.md)，日常入口见 [项目文档](../../../docs/README.md)。
