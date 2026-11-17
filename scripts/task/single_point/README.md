# 官方单点运动规划与控制

本目录仅提供操作说明，不新增程序。这里的“官方单点规划控制 SDK”指 **AgileX 官方 MoveIt 2 + RViz 规划界面**；底层通过官方 ROS 驱动和 pyAgxArm SDK 控制 PiPER-L。SDK 本身不是独立的图形规划窗口。

## 打开实机规划界面

运行条件：项目环境和工作区已完成构建，机械臂及 CAN 已正确连接。先退出其他实机控制程序；本入口会同时启动官方驱动，不要再单独运行 `start_driver.sh`，也不要同时运行多点、伺服或手柄控制。

在终端执行：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot

# 仅在 CAN 尚未激活时执行，会配置系统 CAN 接口
bash scripts/piperl/activate_can.sh

# 启动官方驱动、MoveIt 2 和 RViz
bash scripts/piperl/start_moveit.sh
```

保持终端运行，等待 RViz 模型和 **MotionPlanning** 面板加载完成。启动参数来自 [`config/baseline.json`](../../../config/baseline.json)：默认 `can0`、`piper_l`、无末端执行器、关闭启动时自动使能。该入口连接实机并初始化 SDK，不是离线预览工具。

## 单点规划与执行

1. 在 RViz 的 **MotionPlanning → Planning** 中选择规划组 `arm`。
2. 将起点设为当前状态（`<current>`），确认模型与新鲜的实机关节反馈一致。
3. 拖动末端交互标记设置一个目标位姿，点击 **Plan**，检查规划轨迹。
4. 需要执行时，由现场操作者确认运动空间、停止方式与反馈正常后，另开终端手动使能：

   ```bash
   cd /home/lzc/ROS2_Workspace/PiperL_Robot
   bash scripts/piperl/run.sh ros2 service call /enable_agx_arm std_srvs/srv/SetBool '{data: true}'
   ```
5. 确认服务成功、六轴状态及实物状态正常，点击 **Execute** 执行已检查的轨迹。状态或目标改变后应重新规划。

`Plan` 用于规划预览；`Execute` 会控制实机运动。不要把轨迹动画或界面执行成功当作实物到位证明，需对照真实反馈。Ctrl+C 退出程序不等同于急停；停止与恢复方法见[手动命令](../../../docs/手动命令.md)。

## 只打开离线模型与规划界面

不连接机械臂时，先退出所有实机驱动及其他控制程序，再执行官方模型演示：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh ros2 launch agx_arm_moveit demo.launch.py \
  arm_type:=piper_l effector_type:=none follow:=false
```

此方式不启动实机驱动，不需要激活 CAN 或使能机械臂。**不要与同一 ROS Domain 中的实机驱动同时运行**：演示模式仍会发布控制话题，不能仅凭 `follow:=false` 判断与实机隔离。

官方完整说明：[agx_arm_moveit README](../../../piperl_ws/src/agx_arm_ros/src/agx_arm_moveit/README.md)。
