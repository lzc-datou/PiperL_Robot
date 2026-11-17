# PiPER-L + D405 眼在手上标定

适用情况：**D405 刚性安装在 PiPER-L 末端，相机随末端运动，标定标记固定在桌面上不动**。本工具参考 [AgileX 官方手眼标定教程](https://github.com/agilexrobotics/Agilex-College/blob/master/piper/handeye/README.md)，复用其新版 `agx_arm_calibration` 的交互采样流程及 OpenCV `calibrateHandEye` 求解，不引入旧版 `piper_ros` 驱动。

本目录提供环境准备、D405/ArUco 启动配置、官方采样器适配和打印标记。工具不会使能、回零、规划或移动机械臂；姿态变化由操作者通过现有官方单点控制完成。

## 先区分相机标定与手眼标定

D405 出厂带有相机校准数据。通常先使用相机驱动发布的 `CameraInfo` 中的内参与畸变参数，无需为了手眼标定重新做棋盘格内参标定。若图像、内参或测量有异常，再按 [RealSense 校准文档](https://dev.realsenseai.com/docs/calibration/)检查，而不是默认重写出厂校准。

本次要测的是**安装后相机相对末端的刚性变换**，出厂校准不知道你的支架安装关系。现有棋盘格可以用于其他标定流程，但这里沿用官方单个 **Original ArUco** 标记，不直接识别普通棋盘格。

## 已配置的内容

| 内容 | 配置与说明 |
|---|---|
| 系统 | 本项目 Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 环境 |
| 模式 | `eye_in_hand` |
| 官方标定程序 | `handeye_calibration_ros` 的 `agx_arm_calibration`，订阅 `PoseStamped` |
| 机械臂位姿 | `/feedback/tcp_pose`，表示基座到当前 TCP 的位姿，单位米，四元数 xyzw |
| 参考末端 | `tcp_link`；启动时明确使用零 TCP 偏移，固定整个采样期间的定义 |
| 相机名称／命名空间 | `handeye_d405` / `handeye` |
| 彩色图像 | `/handeye/handeye_d405/color/image_raw` |
| 相机内参 | `/handeye/handeye_d405/color/camera_info` |
| 相机坐标系 | `handeye_d405_color_optical_frame`，不是外壳 `camera_link` 或深度 optical frame |
| 标记位姿 | `/aruco_single/pose`，相机彩色 optical frame 到标记的变换 |
| 检测配置 | 原始彩色图，`image_is_rectified:=false`，使用 CameraInfo 中的畸变参数 |
| 标记 | Original ArUco，ID `582`，打印文件黑色方块名义边长 70 mm |
| 实测尺寸 | **尚未填写**；必须打印后测量，填写 `config/config.yaml` 的 `marker_size_m` |
| 采样数 | 至少 20 组，可继续采更多 |

本项目驱动没有在 `/feedback/tcp_pose` 中填写 `header.frame_id`，因此这里通过配置与启动参数明确约定 `base_link → tcp_link`，不凭空重写消息 frame。TCP 偏移、夹爪配置、运动学零位或相机固定位置一旦改变，应重新确认这一定义及标定是否仍有效。

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

## 2. 打印标记并填写实测边长

打开 [A4 打印文件](assets/original_582_A4.svg)，按 **A4、100%／实际大小**打印，关闭“适应页面”。PNG 仅用于预览，优先打印带物理尺寸的 SVG。

把纸平整固定在硬板上，测量**黑色方块外边长**，不包含周围白边。横、竖尺寸应一致；有明显缩放差异或纸张翘曲时先重新打印、固定。文件中的 50 mm 比例尺可辅助检查打印缩放。

编辑 [`config/config.yaml`](config/config.yaml)：

```yaml
marker_id: 582
marker_size_m: null  # 用实测边长（米）替换 null；不要原样保留
```

例如实测为 69.6 mm，填写 `0.0696`。标记 ID 和边长必须与实物一致；尺寸错误会直接影响平移尺度。空值会阻止检测和采样启动。若要更换 ID，修改配置后执行 `bash scripts/tools/handeye_calibration/run.sh board` 重新生成对应文件。

## 3. 启动 D405 和标记识别

先关闭 RealSense Viewer 及其他占用同一相机的程序。相机支架应刚性固定，线缆留有运动余量。将 ArUco 标记固定在桌面，建议先在约 20–30 cm 观察距离下确认能清晰看到整个黑框和周围白边，保持照明均匀。

**终端 1：启动相机。**

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh camera
```

默认启用 640×480@30 的彩色和深度流；求解只用彩色图像与内参，不使用深度数值。多台 RealSense 同时连接时，先在配置中填写这台 D405 的 `camera_serial`，不要依赖设备发现顺序。

**终端 2：启动 ArUco 检测。**

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh marker
```

**终端 3：观察带检测结果的图像。**

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh view
```

也可以在 RViz 添加 Image 显示 `/aruco_single/result`。ArUco 节点没有结果/位姿订阅者时会跳过检测，因此先打开预览或订阅位姿，再判断是否识别成功。

在另一个终端检查：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/piperl/run.sh ros2 topic echo /handeye/handeye_d405/color/camera_info --once
bash scripts/piperl/run.sh ros2 topic echo /aruco_single/pose --once
```

确认内参不是全零、图像尺寸一致，标记位姿的 `header.frame_id` 为 `handeye_d405_color_optical_frame`，并且位置变化与画面相符。不要将另一台相机、另一个分辨率或旧文件的内参混入本次流程。

## 4. 启动机械臂官方控制

若尚未启动机械臂，在确认 CAN 和现场条件后，按项目单点控制流程启动。不要重复启动驱动，不要同时运行伺服、手柄和多点控制：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
# CAN 尚未激活时才执行
bash scripts/piperl/activate_can.sh
bash scripts/piperl/start_moveit.sh 'tcp_offset:=[0.0,0.0,0.0,0.0,0.0,0.0]'
```

它不会在启动时自动使能；运动操作沿用[手动命令](../../../docs/手动命令.md)中的现场使能、规划、检查轨迹与执行步骤。相机及支架会扩大末端实际外形，现有机械臂模型未因本工具而增加它们的碰撞几何，运动时需留足桌面、标记板和机械臂自身的净空。

检查真实位姿持续更新：

```bash
bash scripts/piperl/run.sh ros2 topic echo /feedback/tcp_pose --once
```

所有终端用项目入口，默认 ROS Domain 43。不要使用模拟机械臂位姿配真实相机，也不要在采样中改变 TCP 偏移、机械臂零位或支架安装。工具不会替你校零，也不会自动发布相机到末端的 TF。

## 5. 手动采样与计算

确认标记固定、检测稳定且真实位姿正常，再开一个交互终端：

```bash
cd /home/lzc/ROS2_Workspace/PiperL_Robot
bash scripts/tools/handeye_calibration/run.sh calibrate
```

每个姿态按下面顺序操作：

1. 用 RViz 官方单点规划改变机械臂姿态；先 `Plan` 检查轨迹，再由操作者执行。标记始终保持固定，尽量位于画面中心区域并完整可见。
2. 确认实机到位并完全静止，等待约 2 秒，观察检测稳定。
3. 回到采样终端按 **Enter**，记录一组位姿。采样期间不要运动。
4. 重复至少 20 次；使用多个位置与观察角度，包含绕两个或更多不平行轴的适度旋转，不能只平移、只绕一个轴或连续采几乎相同的姿态。保持标记清晰，避免极斜角、运动模糊和大幅翻转。

沿用官方交互：**Enter** 采一组，输入 **`d` 再回车**删除上一组，达到 20 组后输入 **`q` 再回车**计算并退出；**`c` 再回车**放弃求解。

每次采样并行等待两路 `PoseStamped`，最多 5 秒；仅接受年龄不超过 0.5 秒、时间差不超过 100 ms、四元数有效且相机 frame 匹配的位姿对。**这不是硬件同步，也不证明机械臂已静止**，所以必须停稳再采。等待超时会中止当前会话，已接受的样本仍保存，修复输入后重新开始；不会触发机械臂动作。

## 6. 结果、方向与验收

终端会打印本次输出目录，位于 `outputs/handeye/<时间戳>/`：

| 文件 | 内容 |
|---|---|
| `session.json` | 配置、frame 约定、实测尺寸、依赖提交版本 |
| `samples.json` | 删除操作处理后的原始接受样本：`base_T_end` 与 `camera_T_marker` |
| `*_calibration.json` | 官方程序输出：position（米）、orientation（xyzw）、rpy（弧度） |
| `T_end_camera.json` | 相同结果的 4×4 齐次矩阵，方便后续使用 |
| `consistency.json` | 用本轮采样反算固定标记，报告平移 RMS/最大散布（mm）和旋转最大散布（度） |

本配置得到的是 **`tcp_link_T_handeye_d405_color_optical_frame`**：将彩色相机光学坐标系中的点转换到 TCP 坐标系。

```text
p_tcp = R_tcp_camera × p_camera + t_tcp_camera
T_base_camera = T_base_tcp × T_tcp_camera
T_base_marker = T_base_tcp × T_tcp_camera × T_camera_marker
```

不要把它取反后仍命名为同一个变换，也不要直接当作 `tcp_link → camera_link` 发布。相机外壳 link 与彩色 optical frame 之间还有官方相机 TF；需要接入整体 TF 树时应按这段变换组合，避免同一个 child frame 存在两个父节点。

一致性报告只是**参与求解的样本内残差**，不是独立精度证明，也不是自动通过标志。验收时保持标记不动，另外选择至少 5 个未用于求解的位置/角度，记录新的真实 TCP 和标记位姿，用上述公式检查反算标记在基座中是否基本不变，再用已知尺寸/已知位置目标验证实际定位误差。可另采一轮标定比较结果的重复性，但重复求解不能替代独立验证。项目尚未设定应用误差阈值，不直接以网上的毫米数宣称通过。

标记尺寸错误、内参不匹配、TCP 定义错误、机器人零位误差、时间差、支架挠曲或松动都会影响结果。完成验证前，不用新矩阵自动驱动机械臂接触目标。

## 文件与版本记录

`run.sh` 是操作入口，`session.sh` 只为本工具加载独立依赖；`prepare.py` 准备源码与构建；`cli.py` 组织命令；`collector.py` 复用官方求解与交互并增加输入检查、样本和一致性输出；`board.py` 生成标记；`offline_test.py` 检查几何与输入边界。第三方源码和许可证保留在 `.deps/handeye_ws/src/`，不复制进机械臂官方子模块。

| 上游 | 固定版本 |
|---|---|
| [AgileX handeye_calibration_ros](https://github.com/agilexrobotics/handeye_calibration_ros/tree/3f8e29606f91b2e6c46457961af03154f7ad3809) | `humble`，`3f8e29606f91b2e6c46457961af03154f7ad3809` |
| [PAL Robotics aruco_ros](https://github.com/pal-robotics/aruco_ros/tree/2872a002ce29362896ed55e28e00abb763335235) | `humble-devel`，`2872a002ce29362896ed55e28e00abb763335235` |
| [RealSense ROS 驱动](https://github.com/realsenseai/realsense-ros/tree/960306819f45006ca3f903aa046bed32c509679a) | `4.59.2`，`960306819f45006ca3f903aa046bed32c509679a` |

验证记录（2026-11-17）：独立工作区七个包构建完成，依赖导入与包索引检查通过；合成数据验证官方眼在手上求解、矩阵方向、样本输出及时间/frame/四元数检查。生成的标记同时通过 OpenCV 和构建得到的官方 ArUco C++ 库识别，ID 均为 582。没有启动真实相机或机械臂、没有使能、运动或采样，尚未产生实机标定结果。实测标记边长仍待打印后填写。
