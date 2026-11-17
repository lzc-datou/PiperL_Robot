# 手柄工具验证记录

日期：2026-11-14（Asia/Shanghai）

## 当前状态

专用手柄链路、PyQt5 界面、Linux evdev 输入、配置 profile、DLS → MoveIt Servo JointJog、共享 guard 和纯 mock 执行均已实现。代码构建及隔离仿真通过。没有接入实体冰原狼 2；设备轴/按键映射、有线/无线断连检测、实机使能和实机运动均未验证。

## 自动化测试

- `bash scripts/piperl/build.sh`：6 个 ROS 包构建完成；colcon 提示一个不存在的可选 `.deps/servo/opt/ros/jazzy` 前缀，但不影响构建。
- 手柄纯逻辑与 profile 测试：`PIPERL_ROS_DOMAIN_ID=64 bash scripts/piperl/run.sh --python-only python -m unittest tests.test_gamepad_core -v`，9 项通过。
- 共享 Servo 回归：`tests.test_servo_damped_velocity`、`tests.test_servo_controller_sync`、`tests.test_servo_hold_guard`、`tests.test_servo_guard_recovery`、`tests.test_servo_feedback_bounds`，共 55 项通过。

## 纯仿真集成

命令：

```bash
PIPERL_ROS_DOMAIN_ID=64 bash scripts/piperl/run.sh python tests/gamepad_sim_integration.py
```

结果：使用 domain 64 的官方 mock/FakeSystem、MoveIt Servo、guard 和 JTC；输入源为专用 `fixture`，没有启动官方实机驱动或发 CAN。

- 六个末端自由度的正负 12 个方向均产生非零 DLS JointJog。
- 通过 Servo 输出话题观察到 47 条轨迹消息。
- 松开 LB 后，guard 以新鲜关节反馈确认停稳；B 停止并锁定通过；松开 B 后显式恢复至等待新 LB 的状态通过。
- 停止发布 fixture 输入后，输入租约超时触发锁定并确认停稳；输入恢复后显式恢复通过。
- 测试清理只向本次启动的 ROS 进程组发送 SIGINT；`cleanup.json` 记录退出码 0、`remaining_pids: []`。ROS 节点日志中的退出信号/清理期 `KeyboardInterrupt` 出现在仿真检查完成后的 teardown。
- 集成脚本在 [gamepad_sim_integration.py](../../../tests/gamepad_sim_integration.py)；隔离栈日志位于 `outputs/gamepad_control/sim_integration/stack.log`。

这证明 mock 执行链可工作，不代表实体手柄兼容或实机安全。测试探针只订阅 guard 之前的 Servo 输出；JTC 命令话题保持 guard 独占订阅。

## 界面与设备输入

- 已使用 PyQt5 创建真实窗口并检查布局；[离线界面截图](/home/lzc/.codex/visualizations/2026/11/14/01a1d8b2-063b-7e42-a8df-228ee11f774b/gamepad-window.png)明确标记为示例输入，不显示伪造的机械臂反馈。
- 设备读取使用 Linux evdev ioctl，不依赖 `python-evdev`。本环境没有发现可读取的 `/dev/input/event*` 手柄设备，无法验证实体轴值、回中噪声、按键代码、拔线、无线关机、接收器保留时断链或重连行为。
- profile 映射可配置 `left_x/left_y/right_x/right_y` 对应 evdev 轴代码与反相、LB/B 按键代码和 D-pad 方向；当前 `flydigi_direwolf2.yaml` 只是起始模板，未校准。

## 实机边界

- `real` 模式需要 `evdev`，并要求操作者显式确认链路失联与映射已验证；guard 拒绝 fixture 和未验证标记。标记本身不证明硬件已验证。
- 本次没有启动 `--mode real`，没有使能、回零、标定、主动 CAN、实机运动或停止用户已有任务。
- 需在以后安排实体手柄验证，分别观察 USB/无线模式、接收器保持连接时手柄关闭、拔线/重连，并记录检测时延。只有确认可靠失联检测与映射后，才可由用户另行安排实机验收。

