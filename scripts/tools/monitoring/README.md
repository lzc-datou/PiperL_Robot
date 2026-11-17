# 机械臂监控工具

## 功能与文件

| 文件                        | 功能                                                                                    |
| --------------------------- | --------------------------------------------------------------------------------------- |
| `read_joint_angles.py`    | 监控入口：显示六轴角度（rad、deg）、电机原始速度（rad/s）及位置、速度各自的反馈 age     |
| `point_panel.py`          | 实时点位显示、异步 FK、冻结与保存按键                                                   |
| `point_snapshot.py`       | 同一组六轴数据的校验、任务格式转换和快照文件保存                                        |
| `joint_range_recorder.py` | 配套模块：记录手动活动时观察到的关节极值，保存 JSON/CSV；通过监控入口启用，无需单独运行 |

工具直接被动接收 Linux SocketCAN，不发送控制帧、不使能机械臂、不修改零点或限位。角度/速度监控不依赖 ROS 节点、MoveIt 或官方 SDK；新增笛卡尔区通过运行中的 MoveIt FK 服务计算，不初始化 SDK。

## 运行条件

- Linux 系统，已配置本项目 Python 环境。
- CAN 接口已激活且有机械臂反馈；默认读取 `config/baseline.json` 中的 `can_port`（can0）。
- 命令从项目根目录运行。CAN 激活步骤见 [手动命令](../../../docs/手动命令.md) 第 4 项；仅关节监控无需先启动驱动；笛卡尔坐标需要已启动官方 MoveIt。

## 实时监控

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh python scripts/tools/monitoring/read_joint_angles.py
```

默认 10 Hz 刷新，在同一终端屏幕更新。Ctrl+C 退出并恢复终端。窗口太小时按提示放大，或加 `--no-points` 仅显示原表格。
`Pos age(ms)` 和 `Vel age(ms)` 分别表示距位置、速度反馈接收的时间；`*` 表示数据过期，数值为最后一次读数；`--` 表示尚未收到。
这里显示的是原始电机速度，未做平滑滤波；终端刷新可能错过短暂峰值。

## 记录观察范围

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh python scripts/tools/monitoring/read_joint_angles.py \
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
| `--record-dir 路径` | 启用范围记录并创建输出目录                                |
| `--can can0`        | 核对接口名是否与 baseline.json 一致；不能覆盖配置         |
| `--help`            | 查看完整参数，不连接 CAN                                  |

## 常见情况

- 全部为 `--`：尚无反馈，检查配置中的接口名、CAN 激活状态及连接。
- age 持续增加或带 `*`：反馈停止或延迟，显示的是旧数据。
- 提示记录目录已存在：换一个新的目录，不覆盖旧数据。

共享配置由 `scripts/piperl/deployment.py` 读取 `config/baseline.json`。配套测试位于 `tests/test_read_joint_angles.py` 和 `tests/test_joint_range_recorder.py`，使用离线数据，不连接真实 CAN。

## 笛卡尔与关节点位

表格下面显示可直接放入多点任务 points 列表的 YAML 条目：关节使用 joints_rad（joint1..joint6），笛卡尔使用 position_m、orientation_xyzw 和 frame_id。
两种表达来自同一组原始关节位置。末端位姿是运行中模型的 FK 结果，不是外部测量；默认 base_link → tcp_link。
不运行 MoveIt 时仍显示关节点位，笛卡尔区标记不可用。使用 `--python-only` 也能保留关节监控功能，但不能使用 ROS FK。

监控显示的是 YAML 紧凑写法。例如下面的条目可以直接放在任务配置的 `points:` 下：

```yaml
points:
  - {name: point_a, type: joint, joints_rad: [0.0, 0.2, -0.5, 0.0, 0.3, 0.0]}
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
保存到 `outputs/points/<时间>/`：joint.yaml 可直接作为任务配置；FK 可用时同时输出 pose.yaml，两份二选一。
文件包含时间、采样 age、固定关节顺序、参考系/TCP 与可用的模型指纹。已固定的是历史有效点位；此后即使机械臂移动，保存的仍是该固定点位，不是新的当前位置。
反馈过期、不完整或 FK 请求超时，不会将其导出为有效笛卡尔目标。没有 FK 时可保存关节点位。

更多选项：`--frame base_link`、`--tcp tcp_link`、`--snapshot-dir outputs/points`、`--no-points`。
点位 YAML 由 PyYAML 保存，来自项目共享的系统依赖。

使用保存点位先规划：

```bash
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py \
  --config outputs/points/<实际时间>/joint.yaml --stack attach --rviz
```

在持续操作入口中，可用 `l /完整路径/joint.yaml` 或 RViz“加载配置”按钮载入快照，再规划、确认执行。

多点整理、运动选项及障碍物配置见 [多点任务说明](../../task/multi_point/README.md)。

反馈不完整、过期或各轴反馈时间差过大时，底部显示当前异常；数据恢复有效后自动清除。保存路径等操作结果不会被这些临时异常覆盖。
