# 飞智冰原狼 2 手柄工具

手柄输入通过专用 ROS 后端进入 DLS、MoveIt Servo、共享 guard 和官方 mock/JTC 执行链。`sim` 使用隔离的 FakeSystem；它不会启动官方实机驱动或发 CAN。代码和 mock 仿真已验证，实体手柄映射、无线断连检测和实机行为仍未验收。

## 运行模式

进入项目 Python 3.12 / ROS 2 Jazzy 环境：

```bash
bash scripts/piperl/run.sh python scripts/tools/gamepad_control/run.py --mode demo
bash scripts/piperl/run.sh python scripts/tools/gamepad_control/run.py --mode input
PIPERL_ROS_DOMAIN_ID=64 bash scripts/piperl/run.sh python scripts/tools/gamepad_control/run.py --mode sim
```

- `demo`：离线界面，不连接 ROS、输入设备或机械臂。
- `input`：只读显示 Linux `/dev/input/event*` 的轴、按键和设备代数；不启动运动栈。
- `sim`：启动官方 mock、MoveIt Servo、独立 guard 和 JTC。默认 `fixture` 输入为中立状态，可用以下选项把实体手柄输入送入 mock 机械臂进行进一步验证：

```bash
PIPERL_ROS_DOMAIN_ID=64 bash scripts/piperl/run.sh python scripts/tools/gamepad_control/run.py --mode sim --input-source evdev --device /dev/input/eventN
```

`input` 和 `sim --input-source evdev` 需要设备读取权限。输入由 Linux evdev ioctl 只读轮询，无需安装 `python-evdev`。设备轴和按键起始映射见 `config/flydigi_direwolf2.yaml`；需按实际设备校准，当前 profile 不是实测结果。

`sim` 使用独立 ROS domain，启动器会拒绝复用该 domain 中已有的 `/piper_servo` 栈；domain 不能隔离同一实体 CAN。仿真不会调用 CAN/SDK/官方实机驱动。

## 控制方式

- 左摇杆：末端沿 `base_link` 的 X/Y 平移；右摇杆上下：沿 Z 平移；右摇杆左右：绕 Z 旋转。
- 十字键左右：绕 X 旋转；上下：绕 Y 旋转。
- 持续按住 LB 才有运动许可；开始/恢复会话本身不产生非零运动。松开 LB 立即请求停止并等待 guard 停稳确认。
- B 停止并锁定；释放 B、反馈和输入恢复后仍需显式点击“恢复控制”，随后再次等待 LB 许可。
- 速度滑条仅在非运动状态调整。配置软件上限为平移 `0.05 m/s`、角速度 `0.25 rad/s`，默认比例 `20%`；这些数值不代表已验证的硬件安全速度。

工具通过独立 Servo 后端互斥，不会与 RViz 鼠标目标后端并行发布 JointJog。界面关闭时先请求停止并等待真实/模拟关节反馈停稳；服务 ACK 或 guard 的 `idle` 状态本身不等于停稳证据。软件停止不代替物理急停。

## 实机门禁

实机入口要求 `evdev` 输入，并要求操作者显式传入 `--verified-link --verified-mapping`；guard 也会拒绝 fixture、未经标记的输入链路或映射。标记只是明确确认，不会替代设备实测。目前没有实体手柄可供校准，USB/无线接收器和无线手柄断连行为都未验证，因此本次验证不执行实机模式，不设置这两个标记，也不声称可以实机运行。

真实手柄映射/连接验证通过后，启动命令的结构为：

```bash
bash scripts/piperl/run.sh python scripts/tools/gamepad_control/run.py --mode real --device /dev/input/eventN --verified-link --verified-mapping
```

该命令只用于以后另行安排的实机验收。实机模式保留手动使能，不自动使能、不回零、不标定。

## 文件

- `run.py`：离线、只读输入、mock 仿真和显式实机入口。
- `app.py`、`gamepad_widget.py`、`ros_bridge.py`：PyQt5 窗口、手柄示意和非阻塞 ROS 交互。
- `input_adapter.py`、`ros_input.py`：profile 驱动的 Linux evdev 读取和 ROS 输入节点。
- `core.py`：六轴速度映射、死区/向量限幅、连接代数与输入租约检查、LB/B 状态逻辑。
- `scripts/task/servo_tracking/gamepad_backend.py` 与共享 `guard.py`：DLS/JointJog 后端和独立执行/停稳防护。
- `config/`：软件速度/时序配置与未校准的设备 profile。
- `DEVELOPMENT_PLAN.md`：用户确认的目标和安全边界；`VALIDATION.md`：本次验证结果及未完成的硬件验收。

