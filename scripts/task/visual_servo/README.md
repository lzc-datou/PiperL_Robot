# PiPER-L 棋盘格视觉伺服

手持棋盘移动时，机械臂跟随图案中心，使棋盘中心位于 **TCP +Z 正前方**，目标距离由 `distance_m` 设置。跟踪中心位于 8 × 11 个内角点的中央，默认每格 15 mm。控制器用手眼外参分别计算相机光轴和 TCP +Z。

| 棋盘动作或状态 | 机械臂和界面的响应 |
| --- | --- |
| TCP +Z 偏离板面垂线 0°～5°，含 5° | 保持当前工具朝向，只跟位置。板面内滚转不产生新的腕部朝向目标 |
| 垂直偏差超过 5° | 沿最短方向纠倾。预测棋盘角点接近画面边缘时减小旋转，必要时先平移恢复画面余量。反馈回到 5° 以内时锁存朝向 |
| 目标在任务工作空间或腕部可达范围外 | 朝目标检查并跟随可行接近点，按剩余制动距离减速。接近实际边界后无法再推进时，Guard 保持，面板显示受限原因 |
| 实际关节或 Servo 已触及保护边界 | 全轴请求保持，面板显示实际受限轴或 Servo 边界。Guard 通过反馈确认停稳后等待棋盘退回 |
| 棋盘退回 | 连续 3 帧新观测通过目标检查，并确认从当前反馈起步可行后，在同一会话恢复。实测关节已到软边时要求向内起步 |
| 相机和检测器均未送来新的有效结果，上一帧过期 | 冻结 PI，沿已批准的运动减速，最迟在曝光年龄 250 ms 时输出零速。新的有效观测通过验证后平滑恢复 |
| 首次开门前持续收到逆解无解 | 保持零速，面板提示移回棋盘。回到可达位置并通过验证后开始 |
| 点击开启时棋盘正在移动，或目标较远 | 新鲜目标通过碰撞感知 IK 验证、首步方向可行且 Guard 就绪后直接起步。后续观测连续更新同一个伺服目标 |
| 运行中缺少新鲜目标超过 1.5 s，仍有递增的迟帧到达 | Guard 关闭运动许可并确认停稳。连续 3 帧新鲜位姿通过验证、首段通过碰撞检查后自动恢复。棋盘可保持静止 |
| 空画面时点击开启 | 跟踪会话启动，机械臂原地等待，运动许可关闭。首次看见棋盘并验证目标后自动起步 |
| 相机仍正常出帧，棋盘移出画面或被遮挡 | 按最后有效曝光算出的基座系 TCP 目标继续闭环运动。途中重见时转跟通过验证的新目标。到位并停稳后等待重现，保持期间连续三帧通过验证后自动续跟 |
| 棋盘重现，完整目标超出工作空间 | 面板显示棋盘可见以及受限的目标范围。用新观测从实测位置验证接近点，首步可行时在同一会话恢复 |
| 前往最后目标时关节受限，实测 TCP 误差持续缺少进展 | 请求 Guard 停稳并保留会话。限位和缺板的并发保持共用最新停止编号。棋盘重现后按停稳、目标复核和首步检查顺序恢复 |
| 可见棋盘的完整目标在逆解阶段触及关节保护区或无可用解 | 沿目标方向检查可行接近点，面板可显示“分段纠倾”“分段接近”或“先平移”。每个点使用 MoveIt 验证逆解、碰撞和关节余量。持续无法安全推进时，Guard 保持并等待可达目标 |
| 执行最后目标时，缺板结果短暂迟到或断流 | 从最后一次有效缺板观测过期起，在 100 ms 内减速。期间收到更新的缺板观测可续行，超时后由 Guard 保持 |
| 主动停止、检测器掉线或换源、面板失联、明确碰撞或设备故障 | 撤销本次运动许可。条件恢复后由操作者点击“开启跟踪” |

## 启动图像闭环仿真

在仓库根目录运行。首次安装或更新本任务时先构建工作空间，生成视觉 Guard 的服务接口和 RViz 面板。

```bash
bash scripts/piperl/build.sh
bash scripts/piperl/run.sh python scripts/task/visual_servo/run.py --mode sim --sim-input image
```

1. RViz 和控制栈就绪后可直接点击 **开启跟踪**。空画面时保持原位，棋盘进入画面后自动跟随。蓝箭头为实测 TCP +Z，绿箭头从棋盘中心指向有向板法线。详情显示 TCP 系横向偏移、图像年龄和 Guard 状态。
2. 仿真以脚本移动棋盘。独立场景节点读取模拟关节反馈，做正运动学得到当前镜头位置，再以 640 × 480 分辨率生成棋盘图像和内参。项目已有的棋盘检测器发布 PnP 位姿。机械臂每移动一步，下一幅图像都会变化。
3. 观察位置跟踪、主动纠倾和回到容差三个阶段。点击 **停止** 可随时撤销跟踪并继续观察。省略 `--sim-input image` 时直接生成相机系棋盘位姿，便于单独调试控制。两种仿真均使用模拟关节控制器与测试外参。

仿真运行时可从第二个终端移动棋盘。平移和旋转均相对**初始 TCP 坐标轴**，平移单位为米。第一条命令接管脚本轨迹，横移 20 mm 并绕初始 X 轴倾斜 8°。第二条模拟遮挡，改为 `true` 可重新看到棋盘。

```bash
bash scripts/piperl/run.sh ros2 topic pub --once /piper_servo/sim/board_offset geometry_msgs/msg/Pose '{position: {x: 0.020}, orientation: {x: 0.0697565, w: 0.9975641}}'
bash scripts/piperl/run.sh ros2 topic pub --once /piper_servo/sim/visible std_msgs/msg/Bool '{data: false}'
```

## 使用已有 D405 和棋盘

实机跟踪可在第一个终端运行 `bash scripts/tools/handeye_calibration/run.sh preview --tracking`，在第二个终端运行下方的 `--mode real` 命令。预览会启动彩色流、棋盘识别和图像窗口。

相机、识别器和任务使用同一 ROS Domain。开两个终端分别启动相机和检测器，已有进程可直接复用。在第三个终端运行只读预览。下面的路径对应本机已有的联合手眼标定，实机应选择与当前安装、零 TCP 偏移匹配的结果。

```bash
bash scripts/tools/handeye_calibration/run.sh camera --color-only --tracking
bash scripts/tools/handeye_calibration/run.sh marker --tracking
bash scripts/piperl/run.sh python scripts/task/visual_servo/run.py --mode observe --handeye-dir outputs/handeye/20261118_124304_059892_combined
```

预览在终端显示棋盘中心、垂直偏差和容差分支。已有唯一的官方 TCP 位姿反馈时，预览也显示基座系候选位姿。省略外参时可以查看相机系中心和深度。运行 `bash scripts/tools/handeye_calibration/run.sh view` 可以看到角点叠图和重投影误差。[手眼标定说明](../../tools/handeye_calibration/README.md)

`view` 的 `POSE OK` 表示该幅图像通过了位姿检测。`exposure age` 是生成叠图时距曝光的时间。跟踪模式逐帧验证当前棋盘角点，位姿有结果就发布，预览独立刷新，最高 10 Hz。超过 150 ms 的帧不能授权新的追踪指令。若位姿持续迟到，面板显示保持和重新验证进度；检测器在有效图像中找不到整板时，在位姿话题的 `/not_visible` 子话题上发布此次曝光的 Header。连续新鲜的缺板结果允许伺服完成最后目标。旧检测器没有这路输出时，任务会按观测中断处理。会话的 `tracking.jsonl` 记录保持、恢复和停止事件。

跟踪预览给 D405 设置 `depth_module.global_time_enabled: false`。检测器把图像、内参与同帧 metadata 精确配对，使用设备读出时刻、曝光偏移和主机内核帧时间估计曝光时间。滚动窗口采用最小传输偏移，仍含 USB 传输误差。可以在该次预览的 `camera-options.yaml` 核对设置，在 `camera.log` 查找 `HARDWARE_CLOCK` 确认生效。在 marker.log 可对照原图时间与校正后的采集年龄；等待元数据或时钟初始化时没有新的跟踪位姿。

检测器自动匹配彩色图发布者的 ROS 传输策略。相机支持 `RELIABLE` 时可重传缺失的图像分片，外部相机仅提供 `BEST_EFFORT` 时仍可接入。`marker.log` 会打印实际选用的策略。图像订阅只保留最近两帧，检测线程只取最新配对。

实机沿用现有 Servo 的现场初始化、互斥锁、面板心跳和独立 Guard。确认拍摄整板、轴向和反馈时间正确，并核对任务工作空间后，实机入口如下。启动后先进入预览，点击 **开启跟踪** 后等待有效目标，通过验证后申请运动许可。关节软余量沿用共享配置，初始速度滑块为 30%。

```bash
bash scripts/piperl/run.sh python scripts/task/visual_servo/run.py --mode real --handeye-dir outputs/handeye/20261118_124304_059892_combined
```

实机六轴为零时，J2 在下限，J3 在上限。连续伺服在每个周期根据实测关节和已验证的目标求关节速度。常规起步方向受限时，求解器同时约束六轴，让边缘关节向内运动。若没有能推进目标且符合边界的首步，面板显示受限方向并等待退回。

J2 下限和 J3 上限处允许配置内的微小零位反馈偏差。Guard 把这两个轴的容差内越界读数折合到模型边界，供 MoveIt Servo 初始化和控制器对齐使用。编码器原值仍用于 TCP 正运动学和独立保护检查，每个待执行轨迹点仍须通过模型硬限检查。

视觉速度逆解根据实测关节的位置和速度，以及 Guard 放行的轨迹尾段，估计每轴的制动余量。请求减速或反向后，只要实测关节或队列仍朝外运动，程序就继续按这个向外速度检查。预计制动会用完软限余量时，先发零速让 Servo 减速。若按配置立即制动也可能越过硬限，则请求 Guard 保持。无约束解触及边界时，用有界最小二乘求可行速度。棋盘可见并在纠倾时，允许优先平移，前提是平移有更多进展、旋转仍向纠倾方向推进且预测视野可行。发送前会再次校验速度与边界。

Servo 报告关节限位时会保持全轴，跟踪会话继续观察棋盘。面板会提示已知的受限方向，例如“J4 上限”。无法定位具体轴时显示“Servo 关节边界”。将棋盘沿刚才的来向移回，使目标和已验证的首步都指向退回方向。Guard 确认停稳、连续三帧通过验证且首步通过碰撞检查后自动续跟。

视觉 Guard 在控制门关闭时对齐实测与控制器反馈并重置 Servo。对齐完成后，视觉后端用仍有效的观测检查首步，再提交当前准备编号、停稳编号和有效期。校验过期时在门关闭的状态下等待新观测。

前馈使用曝光时刻的基座系棋盘中心估计平移速度，滤波后叠加给已验证目标的笛卡尔 PI。位置误差接近 10 mm 时逐步淡出。丢板、观测失效或进入保持时清零，重见后从新帧重新估速。纠倾保视野使用跟踪检测器随位姿提供的 `/handeye/chessboard/pose/camera_info`。更新后重新启动原来运行的检测进程。

默认滑块 30% 时，TCP 指令上限为 90 mm/s 和 25.8°/s。滑块 50% 时为 150 mm/s 和 43.0°/s。实际速度由位姿误差、加速度、关节速度和 Servo 碰撞减速共同决定。

若提示“观测已过期”，查看同次相机预览的 `marker.log`。日志区分缺少完整棋盘 `grid_missing`、角点存在但位姿无效 `pose_invalid`、等待整板重检 `search_pending`、输入断流 `input_interrupted` 和结果断流 `result_interrupted`。光流 `flow` 和近期棋盘区域 `roi` 优先处理最新帧。¾、原分辨率整图搜索 `sb75`、`sb100` 由独立线程处理。异步结果保留原曝光时间并单独记录过期次数。

## 更换识别对象

识别端提交相机光学坐标系中的三维中心（米）、图像采集时间和可选的接近方向。程序用曝光时的实测 TCP、手眼外参和目标距离计算基座系目标 TCP，再送给同一套伺服。当前实机入口接收棋盘位姿，后续物体可以在识别入口接入同一观测接口。

| 对象 | 提交给跟踪器的观测 | TCP 姿态 |
| --- | --- | --- |
| 棋盘格 | 由角点原点换算的板中心、板面法线 | 垂直偏差 ≤5° 时保持，大于 5° 时纠偏 |
| 乒乓球 | 球心三维位置，方向留空 | 保持工具朝向，只跟位置和距离 |
| 茉莉花苞 | 花苞三维中心，可选可信的接近方向 | 只有中心时跟位置，有方向时可使用配置的角度容差 |

新识别器需提供真实尺度和正确时间戳，接入时指定其唯一发布者和坐标系。当前回归用合成中心与方向验证这三种输入。实物球心、花苞中心和朝向仍需各自验证识别精度、时延与失检行为。

## 配置和运行记录

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `distance_m` | 见配置 | TCP 原点到板中心的目标距离，沿 TCP +Z。增加距离会沿工具轴把目标 TCP 向后推移，可能改变关节可达性 |
| `normal_tolerance_deg`、`correction_target_deg` | 5°、4° | 带外计算指向带内的纠偏目标，实际入带时结束纠偏 |
| `linear_speed_mps`、`angular_speed_deg_s` | 0.30 m/s、86°/s | 取与共享 Servo 上限的较小值，实际满量程为 300 mm/s、1.50 rad/s |
| `default_speed_scale` | 0.30 | 视觉任务启动时的滑块比例 |
| `linear_acceleration_mps2`、`angular_acceleration_deg_s2` | 0.20 m/s²、40°/s² | 连续伺服的速度变化率上限，随滑块缩放 |
| `center_filter_s` | 0.05 s | 连续跟踪按实际曝光间隔平滑基座系中心；遮挡后从新观测初始化 |
| `feedforward_gain`、`feedforward_position_band_m` | 0.85、0.010 m | 前馈增益和随位置误差淡出的范围。增益设为 0 可对照 |
| `feedforward_filter_s`、`feedforward_deadband_mps`、`feedforward_max_mps` | 0.06 s、0.006 m/s、0.12 m/s | 平移估速滤波、死区和限幅。合成指令受任务现有速度与加速度限制 |
| `view_margin_px`、`view_prediction_s` | 14 px、0.20 s | 纠倾时的画面边缘余量和预测时长 |
| `max_age_s`、`max_feedback_gap_s` | 0.15 s、0.05 s | 从图像采集计时，使用相邻 TCP 反馈插值 |
| `brake_window_s` | 0.10 s | 已在运动时短时迟帧的指令减速窗口，独立于关节边界和硬件停机 |
| `tracking_loss_timeout_s` | 1.5 s | 超过此时间仍缺少新鲜目标时请求 Guard 原地保持；检测节点在场时可持续等待棋盘 |
| `workspace_min_m`、`workspace_max_m` | 见配置 | 基座系任务包围盒。现场设置应考虑安装位置、可用空间和工具尺寸 |
| `stationary_position_m`、`stationary_angle_deg` | 1 mm、0.35° | 原始棋盘和候选 TCP 均在静止噪声范围内时，保持 RViz 标记和控制目标。相对上次采用目标的累积变化超阈值时更新；每帧仍按实测偏差判定 5° 分支 |
| `retreat_frames` | 3 | 停稳后连续通过检查的新观测数 |
| `frozen_progress_window_s` | 1.5 s | 按实测关节角判断固定目标续行是否有进展的窗口。误差收敛时可继续走到位 |

使用 `--config 路径` 可切换任务参数。配置内的检测参数文件与 Servo 参数文件路径均相对于该配置自身解析。[视觉任务配置](config/config.yaml)

每次运行的 `outputs/tasks/visual_servo/<会话>/` 包含有效控制配置、外参来源和摘要、ROS 日志、跟踪记录及退出清理结果。`tracking.jsonl` 记录实测关节、已验证目标、Guard 反馈、速度前馈 `linear_feedforward_mps`、逆解受限比例 `joint_constraint_scale`、发送前限速 `joint_bound_speed_scale` 和视野余量 `view_margin_px`。接近限位时可对照请求速度、实测估速 `measured_joint_speed_rad_s`、已放行的轨迹末点 `queued_joints_rad`、末段估速 `queued_joint_speed_rad_s` 和制动方向 `joint_brake_reason`。固定目标到位或停滞时分别记录 `frozen_target_reached` 和 `frozen_target_stalled`。

## 重现验收

```bash
bash scripts/piperl/run.sh python -m unittest tests.test_visual_servo
bash scripts/piperl/run.sh python -m tests.visual_servo_kinematics
PIPERL_ROS_DOMAIN_ID=91 bash scripts/piperl/run.sh python -m tests.visual_servo_ros --sim-input image
```

第二条命令运行单进程关节运动学闭环，结果写入 `outputs/tasks/visual_servo/kinematics.json`。第三条命令自行启动并清理 mock 栈，模拟面板开始操作，检查实际模拟关节移动、滚转、纠偏、到界停稳、反向恢复和遮挡。该命令需在支持本机 DDS 通信的 ROS 环境运行，使用专用 Domain 91。使用 `--replay-start <旧会话的 tracking.jsonl>` 可在位姿输入模式重放旧版规划会话的实测起点和前 12 秒板中心、法线，随后保持最后一个目标并检查模拟 TCP 的跟踪误差。

本次运行记录、已经验证的指标及仍需现场验证的项目见[验收记录](../../../outputs/tasks/visual_servo/VALIDATION.md)。设计约定和验收矩阵见[开发文档](DEVELOPMENT.md)。
