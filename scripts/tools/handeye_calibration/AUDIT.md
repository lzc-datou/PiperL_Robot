# PiPER-L + D405 手眼标定代码审计

审计日期：2026-11-18。范围是本目录的相机启动、棋盘格检测、位姿采样、OpenCV 求解和结果验证，并对照 MoveIt Calibration、Industrial Calibration 和 easy_handeye2 的采样、求解和评估源码。

**结论：未发现当前眼在手上公式或变换方向写反，但修复前的采样和验收检查存在缺口。本轮已修复并通过离线与合成 ROS 消息链路测试，可以按更新后的 README 重新采样。** 已有厘米级偏差仍需独立验证和排查，不能仅凭本次代码审计确定根因，也不能宣称实机精度已经改善。

本报告区分“修复前发现的问题”“已经确认正确的实现”和“仍需实物验证的事项”。本轮修复状态见后文。没有启动相机、CAN、机械臂控制或实机运动来证明精度。

## 1. 已有数据说明什么

原标定目录为 [`20261117_192610_060565`](../../../outputs/handeye/20261117_192610_060565/)，包含 21 组训练样本。独立验证目录为 [`20261117_194052_027136`](../../../outputs/handeye/20261117_194052_027136/)，包含 9 组新姿态。验证使用原手眼矩阵，没有使用这 9 组样本重新拟合。

操作者已确认两批采样期间棋盘格、相机支架和 TCP 设置均未改变。该确认是物理条件的依据，配置文件相同本身不能证明实物未变化。

| 数据与指标 | 结果 | 含义 |
| --- | ---: | --- |
| 训练样本固定棋盘格位置 RMS 散布 | 7.056 mm | 相对训练样本位置均值的散布 |
| 训练样本最大位置散布 | 15.382 mm | 相对同一均值 |
| 训练样本最大方向散布 | 0.625° | 相对训练样本平均方向 |
| 独立验证相对原参考位置的 RMS 偏差 | 22.399 mm | 原参考位置也是从训练样本估计的 |
| 独立验证相对原参考位置的最大偏差 | 38.770 mm | 第 7 组 |
| 两批样本平均棋盘格位置的偏移 | 10.370 mm | 两批估计均值之差 |
| 验证样本自身位置 RMS 散布 | 19.854 mm | 相对验证样本位置均值的散布 |
| 验证样本最大两两位置差 | 58.793 mm | 验证样本中最远的两个估计位置 |
| 验证相对原参考方向的最大差 | 1.357° | 相对训练样本平均方向 |

来源是原目录的 [`consistency.json`](../../../outputs/handeye/20261117_192610_060565/consistency.json) 和验证目录的 [`verification_report.md`](../../../outputs/handeye/20261117_194052_027136/verification_report.md)。

这些指标检查的是固定棋盘格在机器人基坐标系中的一致性。**训练残差不是独立精度，独立样本的一致性也不是对外部真值测量的绝对定位精度。** 它们不证明 D405 的深度精度，也不证明机械臂抓取成功。

补充离线诊断有以下结果：

- 仅用原 21 组训练样本分别计算 Tsai、Park、Horaud、Andreff、Daniilidis，9 组验证数据的位置 RMS 散布仍为约 17.0–19.9 mm。未用这些诊断替换原矩阵。详见 [`solver_diagnostics.json`](../../../outputs/handeye/20261117_194052_027136/solver_diagnostics.json)。
- 只在训练集上拟合视觉平移统一比例，得到 1.03648。对应的验证散布仍约 18.485 mm，不能用单一尺寸比例充分解释误差。该比例未写入实际标定。详见 [`scale_diagnostics.json`](../../../outputs/handeye/20261117_194052_027136/scale_diagnostics.json)。
- 由保存的机器人旋转矩阵重新计算，训练样本之间最大相对旋转为 58.653°，验证样本为 163.240°。旧样本并非完全无旋转的集合。较大验证旋转与较大偏差同时出现，是排查线索，不是因果证明。

## 2. 已确认的实现问题与本轮修复

以下描述针对本轮修复前的实现。修复完成后的具体文件和测试结果在第 7 节登记。

### 求解成功条件过弱

旧实现主要依赖最少样本数、OpenCV 返回以及结果为有限数值。只平移、只绕同一轴转动或大量重复姿态，仍可能被收集并送入求解。有限矩阵不代表平移和旋转都被充分约束。

MoveIt Calibration 在手动采样时拒绝与已有样本旋转差小于 5° 的姿态。这有助于避免重复，但也不充分证明旋转轴分布合理。easy_handeye2 的 OpenCV 后端没有相同的多轴检查，甚至保留了最低样本数是否合理的 TODO。三个工具并非都实现了完整的退化拒绝。

**已修复：**增加多轴旋转的可观测性检查，对旋转不足或轴分布退化的数据拒绝求解，并保留诊断量。该检查用于避免后续接受病态数据，**不表示旧 21 组数据已经被证明退化，也不证明旧结果误差由退化引起**。

### 新鲜位姿对不等于停稳或同步

旧 `collector.py` 只接受两路消息年龄不超过 0.5 秒、时间差不超过 100 ms 的单帧位姿对。两路订阅并行接收，比上游依次订阅两路消息更合理，但没有检查采样前后的运动。

在真正静止时，100 ms 的差值不一定导致明显误差。运动中则不同，图像曝光时刻的相机位姿可能与所选机器人消息不一致。操作说明中的“停稳约 2 秒”是人工条件，旧代码不能证明操作者每次都满足该条件。

easy_handeye2 在统一的 `now−0.2 s` 时刻查询两路 TF。Industrial Calibration 的 ROS 2 采集器按图像时间戳查询机器人 TF。MoveIt Calibration 则分别取两路最新 TF，因此它本身也不是严格时间同步的参考实现。

**已修复：**在采样窗口内检查至少 1 秒的停稳状态，并用图像曝光对应的消息时间戳插值机器人位姿。需要检查图像时刻前后的机器人反馈，不能仅依靠“最近一帧”。消息时间戳仍由驱动提供，插值不等同于硬件触发同步，也不能修正驱动本身的未知时间偏差。

### 只保存矩阵，不能追查图像和时间

旧 `samples.json` 只保存 `base_T_end` 和 `camera_T_marker`。`session.json` 保存配置和依赖版本，但没有逐样本时间戳、时间差、原图、CameraInfo、角点或检测 RMS。

因此，对现有 21 组训练和 9 组验证样本，可以重新计算矩阵关系和比较算法，不能事后核验曝光、模糊、角点质量、时间配对或采样时是否运动。不能把新增加的检查结论追溯应用到这些旧样本。

Industrial Calibration 保存原图与对应机器人位姿，可以离线重新检测特征并复现优化。MoveIt Calibration 和 easy_handeye2 的样本导出主要保存变换，不能据此声称三个工具都完整保留原始观测。

**已修复：**随接受的样本保留原始图像、CameraInfo、相关消息时间戳、机器人位姿历史和配对信息，让下一批异常能够追查到实际输入。会话记录还保存本工具源码的 SHA256。

### 采样失败会结束整轮，数据保存依赖退出路径

旧实现已经用 `finally` 保留正常退出和大部分异常退出时接受的矩阵样本，但主要在求解或退出时写文件。进程被直接终止或电脑异常时，内存中尚未写盘的数据仍会丢失。

采样等待超时会抛出异常，上游交互循环捕获后用通用提示结束整轮，操作人员需要重新开始。订阅错误虽有额外堆栈记录，交互行为仍不利于保留一次较长的采样过程。

**已修复：**每接受或删除一组样本后立即原子替换保存文件。可恢复的采样失败返回交互菜单并允许重试，不计作成功样本，不清空已接受数据。不可恢复错误保留明确原因。

## 3. 变换方向、棋盘格和内参检查未发现明显错误

眼在手上且标定板固定时，应满足：

```text
base_T_board = base_T_end × end_T_camera × camera_T_board
```

当前三段变换的方向与成熟工具相同：

| 数据 | 当前来源和方向 |
| --- | --- |
| `base_T_end` | `/feedback/tcp_pose`，机器人基座中的末端或 TCP 位姿 |
| `camera_T_board` | 棋盘格已知三维角点与图像角点经 `solvePnP` 求得，板坐标转换到彩色光学坐标 |
| `end_T_camera` | OpenCV `calibrateHandEye` 的眼在手上输出，将彩色光学坐标转换到末端坐标 |

AgileX 求解器、MoveIt Calibration 的眼在手上分支和 easy_handeye2 都按上述方向把机器人与目标观测传给 OpenCV。没有发现应该额外取逆的地方。仅因为上游变量名出现 `target2cam` 或 `cam2target`，不能按字面改动矩阵，必须核对实际坐标转换。

当前棋盘格检测已具有这些检查：

- 从图像的 `CameraInfo` 读取内参与畸变参数，检查 frame、图像尺寸、模型、参数长度和有限数值。
- 使用同时间戳的原始彩色图与 CameraInfo，配合畸变参数进行 PnP，没有将原图冒充为已去畸变图。
- 生成规则平面三维角点，使用 OpenCV SB 棋盘格角点检测。
- 拒绝无效解、位于相机后方的板角点，以及角点像素重投影 RMS 超过 1 px 的结果。
- 对当前 8×11 内角点的混合奇偶棋盘格，利用黑白单元颜色统一相对原点。两方向同奇偶的普通棋盘格仍需人工确认原点，代码不能从对称图案辨认物理身份。

没有发现当前对象点排列与角点索引明显错位。像素重投影误差小仍不能证明板的实测尺寸、平整度、相机内参或最终三维位姿没有系统偏差。平面位姿对观察角度和图像质量也敏感。

当前配置 `columns: 8`、`rows: 11` 表示内角点，通常对应 9×12 格。旧 README 个别段落仍写 9×12 个内角点，本轮已统一为实际配置。`square_size_mm: 15.0` 仍须以实测单格边长确认，不能把角点数、格子数或标称尺寸混用。

## 4. 机器人位姿的语义仍有边界

本项目官方驱动的 `/feedback/tcp_pose` 没有填写 `header.frame_id`。旧工具把配置中的 `base_frame` 和 `end_frame` 写入元数据，不能凭配置证明消息实际来自该坐标链。

当前操作要求启动机械臂时固定零 TCP 偏移，并保持采样期间的末端定义不变。该约定需要与实际运行的驱动配置一致。若改用其他 pose 话题、非零 TCP、工具定义或模拟数据，即使四元数有效且时间戳新鲜，求出的矩阵也可能被错误命名。

空 frame 不能被当成真实的 `base_link` 证据，也不应随意改写成 `base_link` 来掩盖来源。这里应核对发布源、运行模式和固定 TCP 参数，对消息中明确填写但不符合约定的 frame 拒绝采样，并记录所采用的来源约定。

另一个已核对的边界是 SDK 拼包。当前 [`get_flange_pose()`](../../../external/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py) 分别读取 XY、Z/RX、RY/RZ 缓存，采用最后读取分组的时间戳；ROS 驱动以默认 200 Hz 发布其快照。同一个 RY/RZ 时间戳期间，XY 数据更新可能使位姿微变。调用真实 getter、仅替换普通缓存对象的离线复现中，Y 增加 1 µm 确实会出现“同时间戳、不同位姿”。这是源码允许的行为，未证明本次实机曾出现该情况。

新采样器因此按机器人源时间戳分组，只计一次时间覆盖。同组变化相对首次值仍在停稳阈值内时采用最新值，并保存首末矩阵、变化次数、最大平移与转角；超限或 frame 改变则拒绝。检测器同时间戳的冲突位姿仍严格拒绝。这是对当前反馈接口的适配，不保证六维数据具有硬件同时性。

两棵暂未连通的 TF 树不妨碍直接使用这两路绝对位姿求手眼。标定工具不需要先凭猜测发布 `world → camera` 连接。结果通过验证后，再按正确方向组合官方相机内部 TF。

## 5. 与三个工具的流程差异

| 环节 | 当前项目修复前 | MoveIt Calibration | Industrial Calibration | easy_handeye2 |
| --- | --- | --- | --- | --- |
| 目标检测 | 自有棋盘格 PnP，或外部 ArUco 节点 | 插件检测 ArUco 或 ChArUco | 从原图提取对应点 | 外部目标检测器提供 TF |
| 机器人观测 | 固件反馈 TCP 的 `PoseStamped` | 机器人 TF | 图像时间戳上的机器人 TF | 统一查询时刻的机器人 TF |
| 采样时间 | 两路新鲜单帧，最近消息差不超过 100 ms | 两路各取 latest | 按图像采集时间查 TF | 两路都查 `now−0.2 s` |
| 重复和退化处理 | 无数值化多轴检查 | 排除旋转差小于 5° 的样本 | 报告优化与协方差，并筛图像特征质量 | OpenCV 后端无充分多轴检查 |
| 求解 | OpenCV 默认 Tsai | 五种 OpenCV 手眼算法 | Ceres 优化图像投影或三维对应误差 | 五种 OpenCV 手眼算法 |
| 原始数据 | 只保存位姿矩阵及会话配置 | 变换样本导出 | 原图、位姿与数据索引 | 变换样本导出 |
| 结果检查 | 固定板训练散布，另有独立样本报告 | 运动方程残差和界面显示 | 收敛、代价、协方差、投影与 PnP 分析 | 独立评估界面，部分判断仍有 TODO |

这些工具的实现各有取舍，不能认为它们都包含停稳、多轴可观测性、严格同步和完整数据保存。上游的“求解成功”提示同样不能代替实际应用验收。

Industrial Calibration 的 `2D3D` 模式使用**图像中的二维特征和标定板上已知的三维点**，并非使用 D405 深度流。其 `3D3D` 是另一类残差。比较算法时需要明确具体模式，不能将“用了三维板坐标”写成“联合校准了深度”。

## 6. 本次核对的固定上游版本

源码以本次联网读取的提交固定，避免将后续分支变化混入结论。

| 上游 | 分支和提交 |
| --- | --- |
| MoveIt Calibration | `ros2`，`3f9d48ebe843caf1de060bfafe78160585c7c26f` |
| Industrial Calibration 核心库 | `main`，`c275104eef312ec5a5e07d4e228d6e9e1038f1e4` |
| Industrial Calibration ROS 2 接口 | `master`，`35e9d32e3dd3dca82e05e68ac8ac6e59b7220c04` |
| easy_handeye2 | `master`，`29bd50a7939e853d768d203d2abc61eb6e50facc` |

对应证据如下：

- MoveIt 的 [采样、latest TF 与最小旋转检查](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_control_widget.cpp#L366-L420)，以及 [样本保存](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_gui/handeye_calibration_rviz_plugin/src/handeye_control_widget.cpp#L899-L950)。
- MoveIt 的 [OpenCV 手眼输入方向](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_plugins/handeye_calibration_solver/src/handeye_solver_opencv.cpp#L144-L179) 与 [运动方程残差](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_plugins/handeye_calibration_solver/include/moveit/handeye_calibration_solver/handeye_solver_base.h#L91-L146)。该残差依据位姿关系计算，不是独立真值误差。
- MoveIt 的 [ChArUco 角点与位姿估计](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_plugins/handeye_calibration_target/src/handeye_target_charuco.cpp#L199-L213)。[目标 TF 的时间戳](https://github.com/moveit/moveit_calibration/blob/3f9d48ebe843caf1de060bfafe78160585c7c26f/moveit_calibration_plugins/handeye_calibration_target/include/moveit/handeye_calibration_target/handeye_target_base.h#L167-L177)采用处理时的当前时间，并非直接保留图像时间戳。
- Industrial ROS 2 的 [图像时刻查询 TF](https://github.com/ros-industrial/industrial_calibration_ros2/blob/35e9d32e3dd3dca82e05e68ac8ac6e59b7220c04/scripts/data_collection_node.py#L75-L96) 与 [原图、位姿、索引保存](https://github.com/ros-industrial/industrial_calibration_ros2/blob/35e9d32e3dd3dca82e05e68ac8ac6e59b7220c04/scripts/data_collection_node.py#L115-L134)。
- Industrial 核心库的 [2D3D 优化及协方差](https://github.com/ros-industrial/industrial_calibration/blob/c275104eef312ec5a5e07d4e228d6e9e1038f1e4/optimizations/src/extrinsic_hand_eye.cpp#L46-L125)、[像素残差](https://github.com/ros-industrial/industrial_calibration/blob/c275104eef312ec5a5e07d4e228d6e9e1038f1e4/optimizations/include/industrial_calibration/optimizations/extrinsic_hand_eye_cost.h#L85-L99)，以及 [图像筛选和结果分析](https://github.com/ros-industrial/industrial_calibration/blob/c275104eef312ec5a5e07d4e228d6e9e1038f1e4/gui/src/extrinsic_hand_eye_calibration_widget.cpp#L254-L301)。
- easy_handeye2 的 [统一查询时刻与变换方向](https://github.com/marcoesposito1988/easy_handeye2/blob/29bd50a7939e853d768d203d2abc61eb6e50facc/easy_handeye2/easy_handeye2/handeye_sampler.py#L87-L114)、[样本保存](https://github.com/marcoesposito1988/easy_handeye2/blob/29bd50a7939e853d768d203d2abc61eb6e50facc/easy_handeye2/easy_handeye2/handeye_sampler.py#L154-L173)，以及 [算法与 OpenCV 求解](https://github.com/marcoesposito1988/easy_handeye2/blob/29bd50a7939e853d768d203d2abc61eb6e50facc/easy_handeye2/easy_handeye2/handeye_calibration_backend_opencv.py#L10-L94)。
- easy_handeye2 的 [结果评估采样](https://github.com/marcoesposito1988/easy_handeye2/blob/29bd50a7939e853d768d203d2abc61eb6e50facc/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py#L110-L156)包含静止和重复姿态判断，但[使用的位姿距离度量仍标有 TODO](https://github.com/marcoesposito1988/easy_handeye2/blob/29bd50a7939e853d768d203d2abc61eb6e50facc/easy_handeye2/easy_handeye2/handeye_rqt_evaluator_widget.py#L194-L214)。评估界面的存在不代表其所有质量指标都已完善。

## 7. 本轮修复验证记录

实际执行并通过：

```bash
bash scripts/tools/handeye_calibration/run.sh check
bash scripts/tools/handeye_calibration/run.sh offline-test
bash scripts/tools/handeye_calibration/run.sh integration-test
```

| 检查 | 结果与覆盖 |
| --- | --- |
| 依赖检查 | 官方相机、ArUco、标定包及 Python 依赖可加载；实际 OpenCV 为 4.6.0 |
| `tests_quality.py` | 5 个回归测试通过：重复、纯平移、单轴和微小旋转拒绝；多轴通过；曝光时刻插值与禁止外推；停稳覆盖、丢帧、运动和冲突源时间戳检查 |
| `tests_sampling.py` | 4 组流程检查通过：精确曝光配对、原图/内参归档；SDK 同时间戳修订不扩展覆盖；四路乱序消息、订阅清理；重试、撤销、逐组保存、取消与退化求解拒绝 |
| 棋盘格回归 | 实际 8×11 与原 9×12，共 18 个倾斜/滚转姿态及各自 180° 图像，检查尺度、变换方向、同一物理原点及坏角点拒绝 |
| 官方求解与导出 | 25 组合成多轴姿态恢复已知手眼矩阵至数值误差，实际调用官方求解与项目结果写盘路径；测试 ArUco ID 582 |
| 真实 ROS 消息链路 | 隔离 ROS 域 86、随机话题；200 Hz 合成机器人反馈与合成图像/CameraInfo，经真实棋盘格检测器、默认 1 秒停稳采样器和 Enter/c 保存路径，核验矩阵、PNG、K/D、源时间戳和机器人历史；通过并清理临时输出 |
| 基础检查 | 修改文件的 Python 语法检查及 Git 空白检查通过 |

回归与集成测试代码保留在本目录，可以重复运行。没有修改官方子模块或旧标定数据，没有连接真实相机或 CAN，也没有使能、规划或移动机械臂。合成栅格测试的 2 mm/1° 容差用于容纳像素取整，不代表实机精度；数值合成求解的极小残差同样不代表实机精度。

下一次先确认实物为 8×11 内角点、单格实测尺寸与配置一致，固定棋盘格、相机支架及零 TCP 定义，再采至少 20 组具有多轴旋转的稳定姿态。求解后另开会话，采 5–10 组新姿态并用 `c` 仅保存，使用原矩阵做独立验证，不将验证组加入训练重新拟合。若仍有厘米级散布，应利用新保存的图像、内参、反馈历史和时间配对定位原因，再用任务所需的真实几何测量验收。

## 8. 手柄控制时没有机器人反馈的修复

2026-11-18 后续现场截图显示：棋盘格检测 `POSE OK / RMS 0.05 px`，采样器反复提示“尚未收到机器人反馈”，已保存 0 组。此前的软件测试只验证了显式指定的合成 TCP 话题，未覆盖手柄控制启动方式的命名空间。这是前一次审计遗漏的使用场景。

源码确认了话题不一致：

- [`task.launch.py`](../../task/servo_tracking/task.launch.py) 把手柄/Servo 实机栈与其中的官方驱动放在 `/piper_servo` 命名空间。
- 官方驱动以相对名称 `feedback/tcp_pose` 发布 TCP 位姿，因此手柄实机控制使用 `/piper_servo/feedback/tcp_pose`。
- 标定原配置固定订阅单点控制的 `/feedback/tcp_pose`。手柄控制正常不意味着该根命名空间话题有发布者。

第一版修复将 `robot_pose_topic` 默认值改为 `auto`，新增 `feedback.py`。在上述两个已知话题中检查唯一的 `geometry_msgs/msg/PoseStamped` 发布者，打印选中的话题与发布节点。本轮采样固定该来源，写入 `session.json` 和每组样本。用户仍可在配置中明确填写绝对话题名。第一版只等待 2 秒就报错退出的问题已在第 9 节修复。没有加入 FK 替代路径、第二个 SDK/CAN 连接或新的控制发布者。

采样超时新增 `input_diagnostics.json`，保存四路消息数量、最新源时间戳、年龄和实际订阅话题。错误提示区分没有机器人消息与收到的机器人消息已过期，不再把两者统一描述为未收到反馈。

验证结果：`offline-test` 全部通过，包括新增 8 个反馈选择测试和缺失/过期反馈的超时诊断测试；`integration-test` 在隔离 ROS 域 86 仅发布 `/piper_servo/feedback/tcp_pose` 的 200 Hz 合成机器人反馈，自动选择该源，经过真实棋盘格检测器、默认 1 秒停稳采样和 Enter/c 路径成功保存 1 组，核验实际来源与 PNG/内参/时间戳记录。语法与空白检查通过。该验证不代表实机已经采入；运行中的旧采样进程需要退出后重新启动以加载修复。手柄、相机、检测和控制参数未修改。

## 9. 未发现反馈源时持续等待

随后用户实际启动时，2 秒发现窗口结束仍未找到 TCP 发布者，程序抛出 `ValueError` 并退出。该行为把暂时的图发现状态当作启动失败，未满足可以先启动采样器、后启动控制会话的操作需求。缺少发布者本身也不能证明 Domain 配置不一致。

已将 `resolve_robot_pose_topic()` 改为持续等待，每约 2 秒复查一次，缺失或冲突状态约每 5 秒提示一次，原因变化时及时更新。发现唯一且类型正确的来源后自动进入采样。没有总等待超时；Ctrl+C 或 ROS 关闭可取消等待，入口统一保存已接受样本并清理，不打印正常取消的异常堆栈。

Domain 数字只在启动时作为当前通信分组显示，移除了“没找到发布者”错误中暗示 Domain 已有问题的措辞。项目入口默认将 ROS 域设为 43，正常使用该入口无需重复手动设置。默认情况下，不同 Domain 的节点互相发现不了，这解释了为什么分组是排查项目之一，但不能由本次错误断定它就是实际原因。依据见 [ROS 2 官方 Domain 说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Domain-ID.rst)。

验证：完整 `offline-test` 通过，包含 11 个反馈选择/持续等待测试，以及入口取消后的保存和清理检查；合成时钟测试覆盖 10 秒没有发布者后正常接入、冲突恢复、日志节流与取消。真实 ROS 集成测试在隔离 Domain 92 中将 TCP 发布者延迟 4 秒创建，前 2 秒提示等待，随后自动选择手柄话题并完成 1 组采样，断言原图、内参、矩阵和时间戳均通过，测试正常清理退出。未使用真实硬件。Python 语法与 Git 空白检查通过。
