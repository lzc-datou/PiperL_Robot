> 历史记录，保留当时的设计和验证证据。它不是当前操作指南；现行入口见 [项目文档索引](../../README.md)。

# 手柄工具验证记录

初次仿真验证：2026-11-14；输入与退出修复：2026-11-17（Asia/Shanghai）。

## 当前状态

专用手柄链路、PyQt5 界面、Linux evdev 输入、配置 profile、DLS → MoveIt Servo JointJog、共享 guard 和纯 mock 执行均已实现。代码构建及隔离仿真已于 2026-11-14 通过；2026-11-17 已读取到实体飞智手柄的持续输入。设备各轴方向/LB/B 的人工对应校准、USB/无线断连行为、实机使能和实机运动仍未验证。

## 2026-11-17 关节限位行为调整与界面简化

后续按用户反馈放大字体与窗口：正文和按钮为 20 px、状态为 32 px、辅助信息与诊断为 18 px；控制窗口初始 900 × 720，只读连接窗口 900 × 520。按钮、滑条和手柄示意同步放大，详情改为可滚动区域，收起后不会缩回原来的小窗口。

已重新运行 `preview.py` 并查看实际 Qt 截图：默认和方向受限界面均为 900 × 720，只读连接为 900 × 520，展开详情为 900 × 978。预览仍为明确标记的示例数据，不启动 ROS、手柄或机械臂。

- 用户运行日志 `20261117_205915_sim_d64_1965359/stack.log` 中，原生 Servo 先报告 `joint5` 到达边界，随后 guard 暂停，手柄后端进入 `LOCKED`。这证实整套锁停来自共享暂停策略，不能据此扩大关节限位。
- 按用户确认，手柄模式设置原生 `halt_all_joints_in_joint_mode=false`；Servo 继续限制触边关节向外运动，guard 对手柄模式的 code 6 仅报告方向受限，不改变 running。反向输入仍通过同一 Servo/guard/JTC 链路。鼠标任务原有边界暂停流程保留。当前依赖为 MoveIt Servo 2.12.4，原生实现按速度方向判断触边；没有增加自定义 CAN 或插值路径。
- 硬限位、0.015 rad 的限位余量、轨迹合法性检查、反馈与权限检查保留；B、LB 释放、断链、异常 Servo 状态和碰撞停止仍执行原规则。仅有限位警告不能从已经锁定/停止的状态自动恢复。
- 主窗口改为紧凑的连接卡片、状态、提示、速度和当前操作；设备代数、原始输入、TCP 和诊断收进详情。只读输入模式隐藏运动控件。界面收到的反馈快照过期时，不继续显示反馈正常。
- 已执行 `bash scripts/piperl/build.sh`，6 个包构建通过；通过 `preview.py` 生成并查看默认、详情、方向受限及只读输入的真实 Qt 布局截图。所有预览明确标为示例数据，仅用于界面审阅，没有 ROS 或设备连接。
- 本轮没有新增或执行测试用例，也没有运行限位运动集成或实机验收。前文已有仿真结果不能作为本次限位策略的通过证据；应退出旧窗口并重启后，在纯仿真中验收持续越界受限、反向退回和停止行为。

## 2026-11-17 三维仿真窗口补全

- 用户反馈先前只看到了手柄界面，没有虚拟机械臂。原因是 `gamepad_control/run.py` 原先固定传入 `rviz:=false`；现改为 `sim` 默认显示 RViz，`--no-rviz` 可关闭，`real` 仍默认关闭。
- 新增 `servo_tracking/config/gamepad.rviz`，通过当前模型话题与 TF 显示 PiPER-L、网格和 TCP 坐标轴，保留显示时间兼容插件。手柄模式使用这份配置；鼠标任务继续使用原有配置。
- 实际启动又发现没有显式设备路径时，空的 `gamepad_device:=` 会被 ROS 2 launch 拒绝。现只在指定设备时传入此参数，未指定时由 launch 默认值处理。
- 修改后已构建 6 个 ROS 包；25 项已有离线/进程清理回归通过。另在独立 Domain 65 启动真实 Qt 窗口、RViz、FakeSystem 和 Servo，使用 **fixture 模拟输入** 完成可见模型检查。最终一轮模拟 LB + 摇杆输入产生最大约 **0.0460 rad** 的关节变化，松开 LB 后收到本次停稳确认。
- 实际 RViz 窗口截图：[动作前](../../../outputs/gamepad_control/gamepad_rviz_before.png)、[动作后](../../../outputs/gamepad_control/gamepad_rviz_after.png)。结果见 `outputs/gamepad_control/rviz_visual_20261117.json`；对应窗口 PID `1963503` 的日志目录含完整后台命令及清理报告。退出后本次进程组 `remaining_pids: []`。
- 这些结果确认三维显示和模拟执行链可用，**不代表实体手柄方向、LB/B 或 USB 断线已经完成验证**。用户应按 README 在可见虚拟机械臂中补齐这些检查，再按 [首次实机操作](../../../scripts/tools/gamepad_control/REAL_MACHINE.md) 进入现场验收。开发侧没有运行实机入口或发送实机使能/运动命令。

## 2026-11-17 仿真遗留进程与真实界面启动修复

- 用户再次启动 Domain 64 时，现场确认存在早先 `mode:=sim`、`gamepad:=true` 的独立 ROS launch 进程组，主窗口已经退出，后台仍有 8 个进程；上一次只读窗口清理没有覆盖这一组。已核对所有权及仿真模式后仅回收该组，记录见 `outputs/gamepad_control/20261117_195245_sim_d64_1926731/cleanup_recovery.json`，`remaining_pids: []`。
- 真实 ROS 环境复现了界面启动错误：`GamepadBridge.clients` 与 `rclpy.node.Node.clients` 的只读属性重名。已改为 `request_clients`，增加实际 ROS 节点构造/销毁回归，避免只用模拟 bridge 掩盖此问题。
- 启动顺序调整为先完成 Qt、ROS bridge 和窗口初始化，再创建后台栈；使用 `ExitStack` 在事件循环异常、bridge 清理异常和正常关闭时释放各项资源。前置 ROS 检查失败也会关闭其上下文。
- 新增独立后台监护和生命周期管道。窗口进程即使被 `SIGKILL` 结束，管道断开也会触发本次 ROS 进程组的有限时长回收；仅根据本次启动时持有的进程身份清理。增加显式手柄路径的启动前检查，拒绝原样传入的 `/dev/input/eventN`。
- 相关测试共 **26 项通过**：`tests.test_gamepad_core`、`tests.test_gamepad_input_lifecycle`、`tests.test_gamepad_stack_cleanup`、`tests.test_task_process_cleanup`、`tests.test_gamepad_ros_bridge`。最后一项也在宿主机的真实 ROS 环境单独通过；无控制命令、驱动或 CAN 操作。
- 实际使用实体手柄 evdev、Domain 64、FakeSystem、MoveIt Servo 和真实 Qt/ROS bridge 连续启动两次（Qt 使用 offscreen）。均观察到 Servo 模式切换完成、新鲜仿真关节反馈及实体手柄名称；没有点击开始控制或启动运动会话。第一次向窗口发送 `SIGINT`，第二次强制 `SIGKILL` 窗口；两次后台 launch 和监护进程组均无存活成员，后台清理报告退出码为 0。
- 持续运行的观察节点在退出后暂留旧的 ROS 发现记录；重新创建的独立 ROS 上下文中，实际 `_preflight_ros('sim', 'can0')` 检查通过。报告见 `outputs/gamepad_control/lifecycle_20261117.json`，两次日志目录分别为 `20261117_203326_sim_d64_1946589`、`20261117_203338_sim_d64_1946942`。
- 回收旧栈时曾出现 `move_group` 析构退出码 `-11`；两次新栈退出日志中仍有部分 ROS 节点收到 SIGINT 后的 `-2`/`KeyboardInterrupt`。这些均发生在关闭阶段，进程回收已确认完成；本次没有修复 ROS/MoveIt 内部析构日志问题，也没有进行实机验收。

## 2026-11-17 输入识别与退出修复

- 用户截图中先 `Ctrl+C` 在 PyQt 回调内产生 `KeyboardInterrupt`，随后 `Ctrl+Z` 将窗口挂起；现场确认只读窗口 PID 的状态是 `T (stopped)`。已仅恢复并结束该 `--mode input` 进程。
- `Ctrl+C`/`SIGTERM` 改由 Qt 定时器触发正常关闭，仿真/实机窗口继续使用原有 guard 停止确认流程；只读窗口清理读取线程并退出。不会把终端信号直接当作退出完成证据。
- 修复仅按 `flydigi` 名称匹配造成的漏识别。兼容设备只有一个时允许 Xbox 名称；设备名无法唯一匹配且有多个候选时要求选择，显式 `--device` 在重连后不改为其他设备。
- 增加 `--list-devices`，区分未发现兼容手柄和读取权限不足；只读模式禁用运动按钮。设备文件由读取线程负责关闭，避免 UI 关闭时的描述符竞争。
- 通过项目 Python 环境执行 `python -m unittest tests.test_gamepad_core tests.test_gamepad_input_lifecycle -v`，共 **17 项通过**。包括无设备时关闭窗口、`Ctrl+C`/`SIGTERM`、读取线程退出，以及带模拟 bridge 的信号关闭必须先收到本次 guard 停稳确认。测试未启动 ROS 或 CAN。
- 宿主机已出现 `Microsoft X-Box 360 pad`，输入 ID `045e:028e`，当时设备路径 `/dev/input/event17`。当前用户可读取；使用本工具默认 profile 自动选择后，四轴和 LB/B 字段持续更新，读取线程正常结束。没有向手柄发送输出，也没有启动运动栈。
- USB 的 `by-id` 描述符包含 `Flydigi_Direwolf_3`；不据此确认物理型号或宣布冰原狼 2 profile 已完成校准。设备名称、USB 标识和实际轴方向仍应分别核对。

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

- 已使用 PyQt5 创建真实窗口并检查布局；离线界面截图，原本机路径 `/home/lzc/.codex/visualizations/2026/11/14/01a1d8b2-063b-7e42-a8df-228ee11f774b/gamepad-window.png`，不随源码分发。截图明确标记为示例输入，不显示伪造的机械臂反馈。
- 设备读取使用 Linux evdev ioctl，不依赖 `python-evdev`。2026-11-14 没有可读取的手柄；2026-11-17 已确认四轴与 LB/B 字段可读取。实际轴向、回中噪声、按键对应、拔线、无线关机、接收器保留时断链和重连行为仍未验证。
- profile 映射可配置 `left_x/left_y/right_x/right_y` 对应 evdev 轴代码与反相、LB/B 按键代码和 D-pad 方向；当前 `flydigi_direwolf2.yaml` 只是起始模板，未校准。

## 实机边界

- `real` 模式需要 `evdev`，并要求操作者显式确认链路失联与映射已验证；guard 拒绝 fixture 和未验证标记。标记本身不证明硬件已验证。
- 本次没有启动 `--mode real`，没有使能、回零、标定、主动 CAN、实机运动或停止用户已有任务。
- 需在以后安排实体手柄验证，分别观察 USB/无线模式、接收器保持连接时手柄关闭、拔线/重连，并记录检测时延。只有确认可靠失联检测与映射后，才可由用户另行安排实机验收。

