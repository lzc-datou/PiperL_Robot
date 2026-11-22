# 机械臂监控工具

## 功能与文件

| 文件                        | 功能                                                                                    |
| --------------------------- | --------------------------------------------------------------------------------------- |
| `read_joint_angles.py`    | 显示六轴角度、电机原始速度、反馈年龄和 TCP 位姿 |
| `point_panel.py`          | 本地 FK、可选 MoveIt FK、实时点位显示与冻结保存 |
| `point_snapshot.py`       | 同一组六轴数据的校验、任务格式转换和快照文件保存                                        |
| `joint_range_recorder.py` | 配套模块：记录手动活动时观察到的关节极值，保存 JSON/CSV；通过监控入口启用，无需单独运行 |

工具被动接收 Linux SocketCAN 反馈。从同一组六轴关节角实时显示关节位置和 TCP 位姿。笛卡尔位姿默认由本地 Python 计算，可在仅有 CAN 反馈的情况下显示。

## 运行条件

- Linux 系统，已配置本项目 Python 环境。
- CAN 接口已激活且有机械臂反馈；默认读取 `config/baseline.json` 中的 `can_port`（can0）。
- 命令从项目根目录运行。接好 CAN 并确认接口开启、有反馈即可。CAN 设置见 [控制面板](../control_panel/README.md)。

## 实时监控

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh --python-only python scripts/tools/monitoring/read_joint_angles.py
```

默认 10 Hz 刷新，在同一终端屏幕更新。Ctrl+C 退出并恢复终端。窗口太小时按提示放大，或加 `--no-points` 仅显示原表格。
`Pos age(ms)` 和 `Vel age(ms)` 分别表示距位置、速度反馈接收的时间；`*` 表示数据过期，数值为最后一次读数；`--` 表示尚未收到。
这里显示的是原始电机速度，未做平滑滤波；终端刷新可能错过短暂峰值。

## 记录观察范围

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh --python-only python scripts/tools/monitoring/read_joint_angles.py \
  --hz 5 --record-dir "outputs/joint_ranges/manual_$(date +%Y%m%d_%H%M%S_%N)"
```

按现场允许的方式手动活动机械臂，工具累计收到的关节最小值和最大值。Ctrl+C 保存并退出；也可加 `--seconds 10` 限时记录。
目标目录必须尚不存在，避免覆盖历史记录。运行中周期保存，结束时再保存一次。

| 输出文件         | 内容                                                           |
| ---------------- | -------------------------------------------------------------- |
| `summary.json` | 各轴观察极值、当前角度、极值时间、对应原始帧、样本数和反馈状态 |
| `ranges.csv`   | 适合表格查看的范围、当前角度、样本数、极值时间及反馈状态       |

观察范围仅代表本次收到的数据，不代表已验证的机械限位，不会自动写入控制器或规划配置。

## 常用参数

| 参数                  | 用法                                                      |
| --------------------- | --------------------------------------------------------- |
| `--hz 5`            | 显示刷新率，默认 10 Hz，最大 100 Hz；不改变机械臂反馈频率 |
| `--stale-after 0.5` | 反馈过期阈值，单位秒，默认 0.5                            |
| `--seconds 10`      | 运行指定秒数后退出；默认持续运行                          |
| `--fk-source local` | 默认。用本地 PiPER-L 模型从 CAN 关节反馈计算位姿 |
| `--fk-source moveit` | 使用同一 ROS Domain 内运行中的官方 MoveIt FK 服务，适用于核对在线模型或使用在线自定义参考系、TCP |
| `--record-dir 路径` | 启用范围记录并创建输出目录                                |
| `--can can0`        | 核对接口名是否与 baseline.json 一致；不能覆盖配置         |
| `--help`            | 查看完整参数，不连接 CAN                                  |

## 常见情况

- 全部为 `--`：尚无反馈，检查配置中的接口名、CAN 激活状态及连接。
- age 持续增加或带 `*`：反馈停止或延迟，显示的是旧数据。
- 提示记录目录已存在：换一个新的目录，不覆盖旧数据。

共享配置由 `scripts/piperl/deployment.py` 读取 `config/baseline.json`。配套测试位于 `tests/test_read_joint_angles.py` 和 `tests/test_joint_range_recorder.py`，使用离线数据，不连接真实 CAN。

## 笛卡尔与关节点位

表格下方显示来自同一组 CAN 关节反馈的关节点位和一条 TCP 位姿。两个条目都可以复制到多点或多点避障任务的 `points` 列表。

| YAML 字段 | 单位和顺序 |
| --- | --- |
| `joints_rad` | 弧度，joint1 至 joint6 |
| `position_m` | 米，`[X, Y, Z]` |
| `orientation_xyzw` | 无单位，`[qx, qy, qz, qw]` |
| `rpy` | 度，`[Roll, Pitch, Yaw]` |

`rpy` 使用固定轴 Roll、Pitch、Yaw，旋转矩阵为 `Rz(Yaw) · Ry(Pitch) · Rx(Roll)`。四元数和 RPY 表示同一个 TCP 朝向。编辑任务点位时可以留其中一个。两者都填时，任务比较实际旋转，角差在 0.001° 以内时采用四元数。

本地 FK 与手柄控制使用同一份 PiPER-L 法兰模型和 `RobotModel.pose()`。默认计算 `base_link → tcp_link`，TCP 偏移来自 [手柄与 Servo 共用配置](../../task/servo_tracking/config/config.yaml) 的 `tcp_offset`。默认零偏移对应法兰。更新工具偏移后重启监控即可读取新配置。将快照用于规划时，应让 MoveIt 使用同一参考系和 TCP 偏移。

如果要以正在运行的 MoveIt 模型计算，可用带 ROS 环境的命令。

```bash
bash scripts/piperl/run.sh python scripts/tools/monitoring/read_joint_angles.py --fk-source moveit
```

监控以 YAML 紧凑写法显示点位。以下示例可直接放在任务配置的 `points` 列表。

```yaml
points:
  - {name: point_a, type: joint, joints_rad: [0.0, 0.2, -0.5, 0.0, 0.3, 0.0]}
  - {name: point_001_pose, type: pose, hold_seconds: 0.5, frame_id: base_link,
     position_m: [0.263916, 0.205623, 0.448602],
     orientation_xyzw: [0.302392, 0.920589, 0.028120, 0.245529],
     rpy: [167.1489, 25.7889, 140.6780]}
```

这里的数值仅说明格式。复制实际点位时保留完整的 `- {...}`：`{}` 内字段之间需要逗号，`[]` 内数字之间也需要逗号；每个点位另起一行，行末 `}` 后不加逗号。放入已有 `points:` 下时缩进两个空格，不要重复添加 `points:`，并为各点设置不同名称。

文档中的多行写法则用换行和缩进组织字段，不需要外层 `{}` 或字段分隔逗号；数值列表仍可使用 `[数字, 数字, ...]`。两种格式含义相同，可直接使用监控的紧凑条目，无需转换。缩进使用空格，不使用 Tab。完整对照见 [多行写法与监控中的紧凑写法](../../task/multi_point/README.md#多行写法与监控中的紧凑写法)。

机械臂静止后操作键盘（需交互终端）：

| 按键   | 操作                                           |
| ------ | ---------------------------------------------- |
| p      | 固定当前点位区，方便复制；上方反馈表格继续更新 |
| s      | 保存当前固定点位；未固定时先记录并保存当前点位 |
| r      | 解除固定，继续实时显示点位                     |
| Ctrl+C | 退出                                           |

记录要求六轴反馈完整、新鲜、各对反馈时间差不超过 50 ms，且最近 0.3 s 位置变化不超过 0.003 rad。
每次保存创建 `outputs/points/<时间>/`，里面有 `joint.yaml`。FK 可用时同时保存 `pose.yaml`，其中包含米制位置、四元数和 RPY。两份文件记录同一时刻的反馈、参考系和 TCP。整份本地 FK 快照还带有运动学链指纹，供任务核对模型和 TCP。冻结后保存的是按 `p` 时的样本。

旧 `pose_xyzrpy.yaml` 快照可将六项 `xyzrpy` 的前三项写入 `position_m`，后三项写入 `rpy`，沿用原来的米和度。

实时位姿要求关节反馈新鲜、完整。FK 不可用时仍可查看及保存有效关节点位。

更多选项：`--frame base_link`、`--tcp tcp_link`、`--snapshot-dir outputs/points`、`--no-points`。
点位 YAML 由 PyYAML 保存，来自项目共享的系统依赖。

使用保存的 TCP 点位规划。

```bash
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py \
  --config "outputs/points/<实际时间>/pose.yaml" --stack attach --rviz
```

在持续操作入口中，可用 `l /完整路径/pose.yaml` 或 RViz“加载配置”按钮载入快照。要按顺序访问多个目标，可以把各次采集的条目组合到同一个 `points` 列表，再加载任务规划。

多点整理、运动选项及障碍物配置见 [多点任务说明](../../task/multi_point/README.md)。

反馈不完整、过期或各轴反馈时间差过大时，底部显示当前异常；数据恢复有效后自动清除。保存路径等操作结果不会被这些临时异常覆盖。

## 记录手柄控制链路

`gamepad_trace.py` 只读订阅已运行的手柄输入、JointJog、Servo 轨迹、三层关节状态与 backend、guard 状态。它不发布控制命令、不调用服务、不启动控制栈，不生成或触发运动。

先按[手柄说明](../gamepad_control/README.md)正常启动与操作，再在另一终端使用相同 ROS domain 采样：

```bash
PIPERL_ROS_DOMAIN_ID=64 bash scripts/piperl/run.sh python scripts/tools/monitoring/gamepad_trace.py --duration 10
```

`--duration` 默认 10 秒，最大 60 秒；Ctrl+C 提前结束也保存摘要。`--output-dir` 可指定尚不存在的目录，默认写入 `outputs/gamepad_control/traces/<上海时间>/`，包含逐条消息 `trace.jsonl` 和汇总 `summary.json`。`--help` 不初始化 ROS 节点。

摘要包含每个话题的消息数、接收到达间隔均值与最大值、JointJog 逐帧关节速度最大变化、实测关节位置逐帧最大变化，以及 backend 的 ACTIVE 状态样本数。逐帧变化与到达间隔用于定位输入、计算或传输波动，不等同于加速度或硬件采样周期。backend JSON 保留请求速度与滤波速度等状态字段，非有限数字存为 `null`。没有 ACTIVE 样本时，不能据此判断抖动是否改善；有 ACTIVE 样本也需要与相同操作条件的记录比较。
