# PiPER-L + D405 眼在手上标定

适用情况：**D405 刚性安装在 PiPER-L 末端，相机随末端运动，标定标记固定在桌面上不动**。本工具参考 [AgileX 官方手眼标定教程](https://github.com/agilexrobotics/Agilex-College/blob/master/piper/handeye/README.md)，复用其新版 `agx_arm_calibration` 的 OpenCV `calibrateHandEye` 求解，在项目采样器中检查停稳、时间配对并保存原始输入，不引入旧版 `piper_ros` 驱动。与 MoveIt Calibration、Industrial Calibration 和 easy_handeye2 的源码对照见 [审计记录](AUDIT.md)。

本目录提供 D405、棋盘格和 ArUco 检测入口、官方采样器适配与 ArUco 打印文件。`preview` 一条命令启动相机、检测和结果窗口。工具不会使能、回零、规划或移动机械臂；姿态变化由操作者通过现有控制会话完成。

## 先区分相机标定与手眼标定

D405 出厂带有相机校准数据。通常先使用相机驱动发布的 `CameraInfo` 中的内参与畸变参数，无需为了手眼标定重新做棋盘格内参标定。若图像、内参或测量有异常，再按 [RealSense 校准文档](https://dev.realsenseai.com/docs/calibration/)检查，而不是默认重写出厂校准。

本次要测的是**安装后相机相对末端的刚性变换**。可以用现有棋盘格，也可以用官方教程的单个 **Original ArUco** 标记。两种模式都沿用 D405 内参；切换的是标定板位姿的检测方式，手眼采样和求解流程相同。

## 已配置的内容

| 内容               | 配置与说明                                                                                                                                 |
| ------------------ | ------------------------------------------------------------------------------------------------------------------------------------------ |
| 系统               | 本项目 Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 环境                                                                                       |
| 模式               | `eye_in_hand`                                                                                                                            |
| 官方标定程序       | `handeye_calibration_ros` 的 `agx_arm_calibration`，订阅 `PoseStamped`                                                               |
| 机械臂位姿         | 默认 `auto`，在单点控制 `/feedback/tcp_pose` 和手柄控制 `/piper_servo/feedback/tcp_pose` 中选择唯一有发布者的源；单位米，四元数 xyzw |
| 参考末端           | `tcp_link`；启动时明确使用零 TCP 偏移，固定整个采样期间的定义                                                                            |
| 相机名称／命名空间 | `handeye_d405` / `handeye`                                                                                                             |
| 彩色图像           | `/handeye/handeye_d405/color/image_raw`                                                                                                  |
| 相机内参           | `/handeye/handeye_d405/color/camera_info`                                                                                                |
| 相机坐标系         | `handeye_d405_color_optical_frame`，不是外壳 `camera_link` 或深度 optical frame                                                        |
| 标定板位姿         | 棋盘格：`/handeye/chessboard/pose`；ArUco：`/aruco_single/pose`。均将板上坐标转换到彩色 optical frame                                  |
| 检测配置           | 原始彩色图配合同分辨率的 `CameraInfo` 和畸变参数；棋盘格要求图像与内参的时间戳相同                                                       |
| 当前棋盘格         | 8 × 11 个内角点，每格配置 15 mm；采样前核对实物与实测格长                                                                                 |
| 可选 ArUco         | Original ArUco，ID `582`，打印文件黑色方块名义边长 70 mm；`marker_size_m` 必须填写实测值                                               |
| 采样数             | 至少 20 组，可继续采更多                                                                                                                   |

本项目官方驱动发布的 TCP 位姿没有填写 `header.frame_id`，因此这里通过配置与启动参数明确约定 `base_link → tcp_link`，不凭空重写消息 frame。TCP 偏移、夹爪配置、运动学零位或相机固定位置一旦改变，应重新确认这一定义及标定是否仍有效。

与教程原始命令的区别：不使用 `/end_pose`（旧程序订阅 `Pose`），不使用 `stereo_gazebo_left_camera_optical_frame`，不将未去畸变的原始图像当作 rectified 图像。D405 彩色流配置使用 `depth_module.color_profile`。

## 1. 环境准备

此电脑上已准备独立工作区 `.deps/handeye_ws` 并完成构建；不修改官方机械臂子模块、系统 SDK、全局 pip 或 `.bashrc`。克隆项目到新电脑或删除缓存后，可重新准备：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
python3 scripts/tools/handeye_calibration/prepare.py
```

需要网络、现有项目 ROS 构建环境及 **amd64 的 RealSense SDK 2.59.2 运行库**。此准备脚本针对当前电脑，其他架构/SDK 版本不能直接照用：它校验下载包 SHA256，将开发头文件和缺少的坐标变换依赖解包到 `.deps/handeye_prefix`，并按 `config/dependencies.json` 的提交版本构建官方源码。已有依赖源码版本或工作区修改不匹配时会停止，不自动重置。不会安装另一套 SDK 运行库。

依赖检查与纯离线测试：

```bash
bash scripts/tools/handeye_calibration/run.sh check
bash scripts/tools/handeye_calibration/run.sh offline-test
```

这两条命令不启动相机或机械臂节点。首次构建可能看到 CMake 未使用 `realsense2_DIR` 的警告，因为这个参数只供相机驱动使用；其他包的同类提示不表示构建失败。

`offline-test` 包含 `tests_preview.py` 的进程管理回归测试。测试使用替代子进程检查启动、日志和退出清理，不连接相机、不打开真实预览窗口，也不验证设备图像。

如需复查 ROS 消息传递，可单独运行：

```bash
bash scripts/tools/handeye_calibration/run.sh integration-test
```

该测试先在默认 Context 未初始化时，真实调用 `preview` 的启动检查，验证私有 Context 与执行器匹配、连续调用、同名节点冲突和清理后重试。随后启动合成图像和机器人位姿发布器、真实棋盘格检测器与采样器，使用独立 ROS 域和随机测试话题。临时输出位于 `outputs/` 并自动清理。它不启动设备驱动、不打开图像窗口，也不执行机械臂运动；不测试真实相机、CAN 延迟或硬件精度。

## 2. 选择标定板并填写尺寸

**用现有棋盘格（默认）。** 编辑 [`config/config.yaml`](config/config.yaml)：

```yaml
target: chessboard
chessboard:
  columns: 8
  rows: 11
  square_size_mm: 15.0
```

`columns` 是一行的**内角点数**，`rows` 是一列的**内角点数**，不数外边界，也不数黑白格数量。当前 8 × 11 个内角点对应 9 × 12 格。`square_size_mm` 填一格的实测边长（毫米）；可分别测横、竖连续多格的总长度，再除以格数。检查板是否平整，不能只沿用商品名称或默认 15 mm。棋盘格模式不用填写 `marker_size_m`，也不用打印 ArUco。

**用 ArUco。** 把 `target` 改为 `aruco`，保留现有字典和 ID 设置，按下面的方法填写尺寸。切换模式或改尺寸后，重启 `preview` 和 `calibrate`，新开一轮采样，不混用两种标定板的数据。使用分开启动的调试方式时，重启 `marker`、`view` 和 `calibrate`。

打开 [A4 打印文件](assets/original_582_A4.svg)，按 **A4、100%／实际大小**打印，关闭“适应页面”。PNG 仅用于预览，优先打印带物理尺寸的 SVG。

把纸平整固定在硬板上，测量**黑色方块外边长**，不包含周围白边。横、竖尺寸应一致；有明显缩放差异或纸张翘曲时先重新打印、固定。文件中的 50 mm 比例尺可辅助检查打印缩放。

编辑 [`config/config.yaml`](config/config.yaml)：

```yaml
target: aruco
marker_id: 582
marker_size_m: null  # 用实测边长（米）替换 null；不要原样保留
```

例如实测为 69.6 mm，填写 `0.0696`。标记 ID 和边长必须与实物一致；尺寸错误会直接影响平移尺度。空值会阻止检测和采样启动。若要更换 ID，修改配置后执行 `bash scripts/tools/handeye_calibration/run.sh board` 重新生成对应文件。

## 3. 启动 D405 和标定板识别

先关闭 RealSense Viewer 及其他占用同一相机的程序。相机支架应刚性固定，线缆留有运动余量。把选定的板平整固定在桌面、保持照明均匀。棋盘格先从约 30–40 cm 试拍，以所有角点和板边留白完整入镜为准；ArUco 可从约 20–30 cm 试拍，确保整圈黑框和白边可见。

在一个终端启动相机、检测和结果窗口：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh preview
```

`preview` 按 `target` 选择棋盘格或 ArUco 检测，并自动打开对应结果话题。三个程序继承同一次加载的项目环境与 ROS Domain。相机启动期间，检测器和窗口等待图像；暂未发现标记时保持运行，等待识别。

按 `Ctrl+C` 或关闭结果窗口，会结束本次启动的相机、检测器和窗口。任何一个子进程退出时也会清理其余子进程。启动前会检查重复预览会话，以及当前 ROS Domain 中的同名相机或检测节点；发现冲突时停止启动，不接管已有程序。先结束之前的相机和检测会话，再运行 `preview`。

终端日志分别以 `[camera]`、`[marker]`、`[view]` 为前缀。本次日志保存到 `outputs/handeye_preview/<本地时间戳>/`：`camera.log`、`marker.log`、`view.log` 记录各程序输出，`session.json` 记录启动信息，`cleanup.json` 记录退出清理。该目录只保存预览运行记录，标定采样仍保存在 `outputs/handeye/`。

一键 `preview` 默认使用 640×480@30 的彩色流。需要同时查看深度时可用 `preview --with-depth`。单独启动 `camera` 时默认同时开启彩色和深度。多台 RealSense 同时连接时，可在配置中填写这台 D405 的 `camera_serial`。

视觉伺服使用 `bash scripts/tools/handeye_calibration/run.sh preview --tracking`。第一次看见棋盘时，检测器用 OpenCV SB 搜索完整棋盘。连续帧用双向金字塔光流在原图上定位全部内角点，检查每个角点周围的黑白格，再用当前曝光的角点重新解算位姿。快速移板、丢点或位姿检验失败时，当帧最多做一次整板搜索。640×480 图像通常先以 ¾ 分辨率搜索，失败后在下一张图用原分辨率重试。跟踪叠图按需独立刷新，最高 10 Hz；位姿可随每帧有效观测发布。

手眼标定采样使用普通 `preview`，逐帧在原分辨率搜索。已有预览时，先退出旧进程再切换。跟踪预览使用同帧 D405 元数据，将设备读出时刻与主机内核帧时钟对齐，并扣除曝光偏移。分开启动时使用 `camera --color-only --tracking`、`marker --tracking` 和 `view`。

预览不会启动机械臂控制或交互采样。按后面的步骤保留或启动控制会话，再在另一个终端运行 `calibrate`。

需要分别排查相机、检测器或窗口时，可在三个终端各运行一条命令。分开启动与 `preview` 选择一种，不同时使用：

```bash
# 终端 1：相机
bash scripts/tools/handeye_calibration/run.sh camera
# 终端 2：按 target 启动检测
bash scripts/tools/handeye_calibration/run.sh marker
# 终端 3：检测结果窗口
bash scripts/tools/handeye_calibration/run.sh view
```

分开启动时，每个终端都先进入项目根目录，各程序也需分别退出。`view` 会自动选择预览话题；RViz 中也可以添加 Image，棋盘格选 `/handeye/chessboard/result`，ArUco 选 `/aruco_single/result`。ArUco 节点没有结果或位姿订阅者时会跳过检测，所以先打开预览或订阅位姿。

棋盘格预览中的 `POSE OK` 表示这幅图像已产生有效位姿。`exposure age` 是从这一帧曝光到检测端制作叠图的时间，单位为毫秒。位姿保留原始曝光时间戳，视觉伺服会另行检查控制时刻的图像年龄。`RMS` 是角点重投影误差（像素），超过 1 px 的结果不会发布；低 RMS 本身不保证三维位姿准确。采样前确认图上的 `0` 始终对应板上的**同一个实体角点**；可在旁边白边贴一小块胶带作为目视记号，别盖住格子。当前 8 × 11 的黑白排列可区分两个相对原点，程序会统一角点顺序；若换成两方向同奇偶、旋转 180° 后图案相同的板，需要人工核对原点，发现翻转就不要采那组。

在另一个终端检查：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh ros2 topic echo /handeye/handeye_d405/color/camera_info --once
# 棋盘格模式（ArUco 模式改为 /aruco_single/pose）
bash scripts/piperl/run.sh ros2 topic echo /handeye/chessboard/pose --once
```

确认内参不是全零、图像尺寸一致，标记位姿的 `header.frame_id` 为 `handeye_d405_color_optical_frame`，并且位置变化与画面相符。不要将另一台相机、另一个分辨率或旧文件的内参混入本次流程。

## 4. 使用机械臂官方控制会话

**已经使用手柄控制实机时，保留当前会话。** 手柄工具的官方驱动在 `/piper_servo` 命名空间，发布 `/piper_servo/feedback/tcp_pose`。标定只读取该反馈，不启动第二个驱动，不要求切换到单点控制。采样期间保持零 TCP 偏移，与 `scripts/task/servo_tracking/config/config.yaml` 当前配置一致。

**需要使用官方单点控制时，**先结束其他运动会话和驱动，用 [控制面板](../control_panel/README.md) 完成 CAN 连接、机械臂使能并等待六轴确认，再启动：

```bash
bash scripts/piperl/start_moveit.sh 'tcp_offset:=[0.0,0.0,0.0,0.0,0.0,0.0]'
```

一次只使用一套控制会话，不重复启动驱动。面板可保留用于阻尼急停，采样运动会话中不要回零或重置；急停恢复先结束旧会话与驱动，经面板恢复使能后再启动。单点规划与执行见 [单点指南](../../task/single_point/README.md)。

默认配置为 `robot_pose_topic: auto`。采样器启动后持续等待，每约 2 秒检查上述两个已知 TCP 反馈话题，发现唯一的 `PoseStamped` 发布源后打印实际话题和节点，继续进入交互采样；本轮采样保持该选择不变。暂未找到时不会退出，约每 5 秒显示等待原因；也可以在等待期间启动控制会话。发现多个话题、同一话题多个发布者或类型冲突时显示原因并继续等，直到来源唯一可用。按 `Ctrl+C` 可取消等待，已接受的样本会保留。不会改用手柄界面中的 FK 数值或控制器指令状态。

如需明确指定，在 `config/config.yaml` 中填写本次会话的绝对话题名，例如 `robot_pose_topic: /piper_servo/feedback/tcp_pose`。这只改变标定器读取的输入，不改变机械臂控制配置。

相机及支架会扩大末端外形，现有机械臂模型未因本工具而增加它们的碰撞几何，运动时需留足桌面、标定板与机械臂自身净空。

检查真实位姿持续更新：

```bash
# 手柄控制
bash scripts/piperl/run.sh ros2 topic echo /piper_servo/feedback/tcp_pose --once
# 官方单点控制时改为
bash scripts/piperl/run.sh ros2 topic echo /feedback/tcp_pose --once
```

所有终端用项目入口，默认 ROS Domain 43。Domain 是 ROS 2 的通信分组，默认情况下不同组的节点互相发现不了。项目入口统一设置默认分组，正常使用这些入口时无需额外设置。日志中的 Domain 数字只说明标定器当前所在分组，未发现话题本身不能证明存在 Domain 不一致；若长时间等不到，再核对控制会话、实际话题和通信环境。

不要使用模拟机械臂位姿配真实相机，也不要在采样中改变 TCP 偏移、机械臂零位或支架安装。工具不会替你校零，也不会自动发布相机到末端的 TF。

## 5. 手动采样与计算

确认标记固定、检测稳定且真实位姿正常，再开一个交互终端：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh calibrate
```

每个姿态按下面顺序操作：

1. 通过当前控制会话改变机械臂姿态。使用手柄时按现有操作方式移动，到位后松开运动许可；使用单点规划时先 `Plan` 检查轨迹，再由操作者执行。标记始终保持固定，尽量位于画面中心区域并完整可见。
2. 确认实机到位并完全静止，等待约 2 秒，观察检测稳定。
3. 回到采样终端按 **Enter**，记录一组位姿。采样期间不要运动。
4. 重复至少 20 次；使用多个位置与观察角度，包含绕两个或更多不平行轴的适度旋转，不能只平移、只绕一个轴或连续采几乎相同的姿态。保持标记清晰，避免极斜角、运动模糊和大幅翻转。

交互：**Enter** 采一组，输入 **`d` 再回车**删除上一组，达到 20 组后输入 **`q` 再回车**计算并退出；**`c` 再回车**只保存样本、退出而不求解。独立验证可以另开一次 `calibrate`，采至少 5 个未用于求解的新姿态，再用 `c` 保存，避免把验证组混入训练组。

每次按 Enter 后，程序并行接收机械臂反馈、板位姿、彩色原图和内参，默认最多等 10 秒：

- 图像、内参和板位姿必须具有相同源时间戳及正确的彩色 optical frame；输入年龄不超过 0.5 秒。
- 机器人反馈须覆盖图像时刻之前完整 1 秒，窗口内相对起点变化不超过 1 mm、0.2°，反馈最大间隔不超过 100 ms。
- 通过源时间戳包围图像时刻，平移线性插值、旋转球面插值；该处反馈间隔不超过 50 ms，不外推。
- 每接受或撤销一组立即更新 `samples.json`；每组保存原图与内参、原始时间戳、插值和停稳检查记录。超时只拒绝本组，显示原因，可以继续按 Enter 重试，之前的样本保留。

这些阈值位于 `config/config.yaml` 的 `capture`。它们是输入质量检查，不是标定精度保证。官方 SDK 的六维末端反馈来自多个 CAN 帧，源时间戳也不能保证所有分量在同一硬件时刻采集，所以仍须先停稳再采样。同一机器人源时间戳只计一次；若拼包造成微小位姿更新，程序在停稳阈值内保留最新值，保存相对首次值的变化记录，超过阈值则拒绝本次采样，不伪造新时间戳。若持续无法满足阈值，先检查反馈频率、时间戳和实机是否稳定，再判断是否应调整配置。

求解前还会检查旋转激励：最大两两相对旋转至少 15°，旋转向量的第二/第一奇异值比例至少 0.1。20 组重复姿态、纯平移或仅绕一个轴会被拒绝。拒绝后可以补采或用 `d` 撤销；这项几何检查同样不能替代独立验证。

## 6. 结果、方向与验收

终端会打印本次输出目录，位于 `outputs/handeye/<时间戳>/`：

| 文件                       | 内容                                                                                                              |
| -------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| `session.json`           | 配置、frame 约定、所填尺寸、依赖提交版本与本工具源码 SHA256；尺寸是否实测由操作者确认                             |
| `samples.json`           | 每组 `base_T_end`、`camera_T_marker` 及机器人历史、源时间戳、frame、内参、停稳与插值记录；接受/撤销后立即保存 |
| `images/`                | 每组对应的无损彩色 PNG 原图，用于离线复算角点及检查模糊、遮挡与原点                                               |
| `sampling_error.txt`     | 订阅或采样异常时保存真实异常堆栈，终端也会显示具体原因，避免只看到官方程序的通用退出提示                          |
| `input_diagnostics.json` | 采样超时时保存实际订阅话题、ROS 域、四路消息数量、最新源时间戳与年龄，区分未收到机器人消息和消息过期              |
| `*_calibration.json`     | 官方程序输出：position（米）、orientation（xyzw）、rpy（弧度）                                                    |
| `T_end_camera.json`      | 相同结果的 4×4 齐次矩阵，方便后续使用                                                                            |
| `consistency.json`       | 旋转激励指标，以及本轮采样反算固定标记的平移 RMS/最大散布（mm）和旋转最大散布（度）                               |

本配置得到的是 **`tcp_link_T_handeye_d405_color_optical_frame`**：将彩色相机光学坐标系中的点转换到 TCP 坐标系。

```text
p_tcp = R_tcp_camera × p_camera + t_tcp_camera
T_base_camera = T_base_tcp × T_tcp_camera
T_base_marker = T_base_tcp × T_tcp_camera × T_camera_marker
```

不要把它取反后仍命名为同一个变换，也不要直接当作 `tcp_link → camera_link` 发布。相机外壳 link 与彩色 optical frame 之间还有官方相机 TF；需要接入整体 TF 树时应按这段变换组合，避免同一个 child frame 存在两个父节点。

一致性报告只是**参与求解的样本内残差**，不是独立精度证明，也不是自动通过标志。验收时保持标记不动，另外选择至少 5 个未用于求解的位置/角度，记录新的真实 TCP 和标记位姿，用上述公式检查反算标记在基座中是否基本不变，再用已知尺寸/已知位置目标验证实际定位误差。可另采一轮标定比较结果的重复性，但重复求解不能替代独立验证。项目尚未设定应用误差阈值，不直接以网上的毫米数宣称通过。

这里的位置 RMS 散布计算为 `sqrt(sum(||p_i - p_mean||²) / N)`，其中 `p_i` 是每组反算出的固定标记位置，`p_mean` 是这些位置的均值。它衡量换姿态后估计位置有多不一致，单位 mm，是三维距离而非单轴误差，也不同于检测图上的像素重投影 RMS。它包含视觉检测、机器人反馈和外参的综合影响。均值没有外部真值依据，共同的位置偏移不会反映在散布中，因此不能把 RMS 散布直接当成绝对抓取误差或机器人重复定位精度。

可用 `verify.py` 在纯离线环境中复算独立样本，并在同一验证集上比较新旧矩阵。例如本次验证：

```bash
bash scripts/piperl/run.sh python scripts/tools/handeye_calibration/verify.py \
  --calibration outputs/handeye/20261118_115734_138993 \
  --validation outputs/handeye/20261117_194052_027136 \
  --baseline outputs/handeye/20261117_192610_060565
```

报告保存为新标定目录中的 `verification_against_<验证目录名>.md` 和同名 JSON，包含逐组偏差、输入 SHA256、重复样本检查，以及训练组和验证组中可用源反馈历史的停稳/插值重放。不重新拟合，不改原矩阵或原数据。跨会话复用验证数据的前提是相机安装和 TCP 定义未改变；棋盘格可以在会话之间移动，但在验证样本内部必须固定。默认以验证组自身均值为参考，不把不同会话的棋盘格位置差算作标定误差。

若操作者确认棋盘格从本次标定到验证一直未移动，可增加 `--same-board-pose`，同时检查验证位置相对本次训练参考位置的偏差。例如最新 7 组：

```bash
bash scripts/piperl/run.sh python scripts/tools/handeye_calibration/verify.py \
  --calibration outputs/handeye/20261118_115734_138993 \
  --validation outputs/handeye/20261118_122335_633339 \
  --baseline outputs/handeye/20261117_192610_060565 \
  --same-board-pose
```

训练参考位置也是由样本估计的均值，不是外部真值。使用 `--baseline` 比较旧矩阵时，参考板位姿会从本次训练数据分别用两版固定矩阵反算，不使用其他会话可能已经移动过的棋盘格位置。验证组不需要满足求解器的多轴门槛，但姿态覆盖范围会记录在质量检查中。

标记尺寸错误、内参不匹配、TCP 定义错误、机器人零位误差、时间差、支架挠曲或松动都会影响结果。完成验证前，不用新矩阵自动驱动机械臂接触目标。

## 7. 将多轮采样联合求解

相机安装和 TCP 定义未变时，可以让多轮样本共享同一个 `end_T_camera`。棋盘格移动前后的数据需要分组：一个 `--group` 对应同一个固定的板位姿，可以包含多个采样目录。每个组内部采样时板必须固定。不能把移动板后的所有数据直接合并给单板求解器。

当前保存的有效样本为 21 + 9 + 21 + 7，共 58 组。操作者已确认相机与 TCP 未变；旧 21 + 9 组为一个板位置，新 21 + 7 组为另一个板位置。复算命令：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh python scripts/tools/handeye_calibration/combined.py \
  --group outputs/handeye/20261117_192610_060565 outputs/handeye/20261117_194052_027136 \
  --group outputs/handeye/20261118_115734_138993 outputs/handeye/20261118_122335_633339 \
  --context-note '相机安装和 TCP 未变；棋盘格只在两组之间移动，组内始终固定。'
```

工具使用组内相对运动的 Park/SVD 旋转拟合，再对全部原始样本等权求平移和各组板位置，不进行跨组配对。结果保存到新的 `outputs/handeye/<时间戳>_combined/`，包含 `T_end_camera.json`、`REPORT.md`、`consistency.json` 和带来源记录的 `samples.json`、`session.json`。报告分别统计各板位置的训练散布，并记录原输入 SHA256。`--output` 可指定尚不存在的新目录。

全部 58 组都用于求解后，原来的 9 组和 7 组也成为训练数据。需要另外采集新姿态作独立验证，不能再用这两批评价新矩阵的泛化精度。新采样保存后，可将新联合目录传给 `verify.py --calibration`，新采样目录传给 `--validation`。联合训练包含不同板位置，验证时使用默认的自身均值参考，不传 `--same-board-pose`。

## 文件与版本记录

`run.sh` 是操作入口，`session.sh` 只为本工具加载独立依赖；`prepare.py` 准备源码与构建；`cli.py` 组织命令，并管理 `preview` 的子进程与日志，不另设 `preview.py`；`chessboard.py` 使用 [OpenCV 角点检测与 PnP](https://docs.opencv.org/4.6.0/d9/d0c/group__calib3d.html)估计板的位姿，图像处理使用现有 Jazzy 的 `cv_bridge` 和 `message_filters`；`collector.py` 复用官方求解，管理交互采样与原始记录；`feedback.py` 选择已有控制会话的 TCP 话题；`quality.py` 检查停稳、时间插值与旋转激励；`board.py` 生成 ArUco 标记；`offline_test.py`、`tests_feedback.py`、`tests_quality.py`、`tests_sampling.py` 检查话题选择、几何和采样回归，`tests_preview.py` 检查预览子进程的启动与清理，`integration_test.py` 验证隔离测试话题中的真实 ROS 消息链路。第三方源码和许可证保留在 `.deps/handeye_ws/src/`，不复制进机械臂官方子模块。

| 上游                                                                                                                                     | 固定版本                                                       |
| ---------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| [AgileX handeye_calibration_ros](https://github.com/agilexrobotics/handeye_calibration_ros/tree/3f8e29606f91b2e6c46457961af03154f7ad3809) | `humble`，`3f8e29606f91b2e6c46457961af03154f7ad3809`       |
| [PAL Robotics aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/2872a002ce29362896ed55e28e00abb763335235)                         | `humble-devel`，`2872a002ce29362896ed55e28e00abb763335235` |
| [RealSense ROS 驱动](https://github.com/realsenseai/realsense-ros/tree/960306819f45006ca3f903aa046bed32c509679a)                          | `4.59.2`，`960306819f45006ca3f903aa046bed32c509679a`       |

`verify.py` 用于固定矩阵的离线验证；`combined.py` 用于棋盘格按固定位置分组后的共享外参求解；`tests_combined.py` 检查移动板、多轴激励、矩阵方向及 OpenCV Park 旋转对照。

2026-11-18 的代码审计、软件验证及旧实机数据的独立验证结论记录在 [AUDIT.md](AUDIT.md)。软件测试不代表实机验收；旧一轮结果未通过独立一致性检查，下一次应使用修复后的采样器重新采集并单独验证。
