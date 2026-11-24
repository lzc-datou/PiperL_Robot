# D405 与 PnP 的 ArUco 角点精度验证

`d405_depth_accuracy` 比较 ArUco **100** 黑框四个角的三组 XYZ。测针给出机械臂基座系的参考坐标，D405 和纯 RGB PnP 各自估计四角位置。工具读取相机和机械臂反馈，机械臂移动由现有手柄会话负责。

## 准备输入

使用项目的 Ubuntu、ROS 2 Jazzy 和 Python 环境，需要 OpenCV 的 ArUco 模块、NumPy、SciPy、PyYAML、`sensor_msgs`、`geometry_msgs` 和 `tf2_ros`。D405 的驱动准备见 [手眼标定环境](../handeye_calibration/README.md)。

| 文件 | 作用与填写方式 |
| --- | --- |
| [`config/reference_points.yaml`](config/reference_points.yaml) | 保存针尖 TCP 在机械臂基座系下的四个 XYZ，单位 mm。把四组 `null` 替换为实测值。`frame_id` 填实际基座系，项目惯用 `base_link`。 |
| [`config/probe_samples.csv`](config/probe_samples.csv) | 可选的原始触测记录。每行依次填写角点、重复采样序号和 XYZ（mm）。有数据时，`run.yaml` 会记录每个角的均值、触测散布及与所填参考点的差。 |
| [`config/experiment.yaml`](config/experiment.yaml) | 填生成 ID 100 时的 OpenCV 字典 `dictionary`，以及用量具独立测出的黑框外沿边长 `marker_size_mm`。可按需要修改相机话题前缀、手柄反馈话题和采集参数。 |
| [`config/camera_options.yaml`](config/camera_options.yaml) | 启动 D405 时交给现有手眼工具，开启 RGB 和深度帧的同步。已经在同一 ROS 域发布两路原始流和内参时，可直接复用相机进程。 |
| 手眼结果目录中的 `session.json` 和 `T_end_camera.json` | 用 `--handeye-dir` 指定本次相机安装后的标定结果目录。前者记录 `eye_in_hand`、基座、末端、彩色光学系与矩阵方向，后者为米制 4×4 矩阵。 |

先固定机械臂底座和 100 号板，再用标定好的测针触测**黑框外沿**四个交点。`TL`、`TR`、`BR`、`BL` 是码原始朝向的左上、右上、右下、左下，对应 OpenCV 解码后的索引 `0`、`1`、`2`、`3`。相机转动时按角点身份对应，首张编号图可供核对。换装相机和标定期间保持底座与码板的相对位置。每个角建议重新对准并轻触 5 次，参考文件填写三轴均值。

### 手眼矩阵和实时末端位姿

`T_end_camera.json` 将彩色光学系的点变换到**手眼采样时使用的 `tcp_link`**。默认从 `/piper_servo/feedback/tcp_pose` 读取该末端在基座系的位姿。手柄反馈与手眼标定应使用同一个实际末端坐标系。程序将两者直接相乘。

```text
base_T_camera = base_T_end(采图时刻) × end_T_camera
p_base = base_T_camera × p_camera
```

换装测针时取得的针尖 XYZ 已经表达在基座系。工具直接读取这四个参考值。

## 启动和采集

以下命令从项目根目录运行。手柄实机示例使用 ROS Domain 64，所有相关终端要与实际控制会话的 Domain 一致。

1. 填好配置并换装 D405。用 [手眼工具](../handeye_calibration/README.md) 完成当前安装下的眼在手上标定。先用下列命令检查输入和手眼矩阵。

   ```bash
   PIPERL_ROS_DOMAIN_ID=64 bash scripts/tools/d405_depth_accuracy/run.sh \
     --handeye-dir /实际的手眼结果目录 --check
   ```

2. 通过 [实机手柄入口](../gamepad_control/REAL_MACHINE.md) 启动控制。若尚无运行中的 D405，另开终端启动相机。

   ```bash
   PIPERL_ROS_DOMAIN_ID=64 bash scripts/tools/handeye_calibration/run.sh camera \
     --camera-options scripts/tools/d405_depth_accuracy/config/camera_options.yaml
   ```

3. 新开一个交互终端启动工具。`--config` 和 `--reference` 可以单独指定替代 YAML。

   ```bash
   PIPERL_ROS_DOMAIN_ID=64 bash scripts/tools/d405_depth_accuracy/run.sh \
     --handeye-dir /实际的手眼结果目录
   ```

4. 用手柄移动到 100 号板完整入镜的姿态。终端默认以 4 Hz 刷新，每角一行，依次显示测针参考、D405 XYZ、PnP XYZ、两路三维误差。停稳后按 `s`，记录按键后收到的 60 个 RGB 帧，约 2 秒。首轮可采约 20 cm 正视、约 20 cm 斜视 20°、返回首次位置三组。距离按相机光轴方向记录。
5. 每组保存后可移动到下一姿态。输入 `q` 或 `Ctrl+C` 退出。正在采集的组会保存已有帧，标为 `interrupted`。8 秒内不足 60 帧的组标为 `timeout`，供检查或补采。

## 坐标和数据质量

两路使用同一帧 RGB 的四个亚像素角点和当前相机内参。PnP 依据独立测量的黑框边长，用 `SOLVEPNP_IPPE_SQUARE` 求姿态，从正深度候选中选重投影均方根最小的一解，再算四角坐标。D405 用原始深度光学系内参反投影，通过相机自身 TF 变换至彩色光学系，保留码框内缩区域的有效点，以 RANSAC 和正交最小二乘拟合平面。四条 RGB 角点射线与拟合平面相交得到四角。测针坐标只用于计算误差。

默认读取 `<camera_namespace>/color/image_raw`、`color/camera_info`、`depth/image_rect_raw` 和 `depth/camera_info`。深度图的 `16UC1` 默认按毫米、`32FC1` 按米解释。ROS 驱动发布的整数毫米图会把深度量化带进实验。若更换为非标准单位的发布者，按实际约定填写 `depth.scale_m_per_unit`。工具使用测到的有效深度点，板面覆盖过小或离群点过多时标记该帧的深度结果无效。

RGB 和深度默认允许 12 ms 的采图时间差。机器人位姿通过包围 RGB 时间戳的反馈插值得到，默认最大间隔 50 ms。实际测量组请保持静止，便于减少相机移动、RGB 与深度时间差对结果的影响。文件中的时间戳保留整数纳秒，在 Excel 中建议以文本导入时间戳列。

| 常见终端状态 | 可以检查的输入 |
| --- | --- |
| `tag_missing`、`duplicate_tag` | ID 100 的字典、黑框完整性，以及画面内是否有同 ID 的第二块板。 |
| `color_info`、`color_frame_mismatch` | RGB `CameraInfo`、分辨率与手眼记录的同一彩色光学系。若会话中切换了相机配置，重启测量。 |
| `depth_missing`、`depth_info`、`depth_color_tf` | 原始深度是否启用、RGB-D 时间差、对应内参和相机自身的光学 TF。PnP 在这些情况下仍可输出。 |
| `depth_few_points`、`depth_few_inliers`、`depth_coverage` | 码框内是否存在足够的同一平面深度采样。可从约 20 cm 正视重试。 |
| `pose_source_missing`、`pose_source_conflict`、`pose_missing`、`pose_gap` | ROS Domain、官方反馈话题、发布者数量和位姿的采样时间。 |
| `pnp_no_solution`、`pnp_reprojection` | 黑框尺寸、模糊、角点识别和重投影误差。 |

## 输出文件

每次运行创建 `outputs/d405_depth_accuracy/<日期_时间>_id100/`，终端显示绝对路径。

| 文件 | 内容 |
| --- | --- |
| `corners.csv` | 每个接收到的采样帧写四行。包含组号、帧序号、RGB 与深度时间戳、时间差、位姿插值间隔、角点像素、三组基座系 XYZ、两路状态、各轴有符号偏差及三维误差。记录深度拟合点数、内点比例、板面覆盖和残差，以及 PnP 候选的重投影误差。无效坐标留空，失败帧仍计入分母。 |
| `summary.csv` | 按组、角点和方法汇总。`all` 使用本方法所有有效帧，可看各自有效率。`paired` 使用同一帧中两路均有效的样本，可公平比较精度。记录有效帧数、各轴平均偏差、三维误差 RMSE 和 P95、相对该组估计均值的三维 RMS 抖动。 |
| `run.yaml` | 本次配置与参考点快照、手眼矩阵与文件 SHA256、相机内参和分辨率、深度单位、深度到彩色光学系的外参 `color_T_depth`、ROS Domain、各组状态，以及由收到的 RGB 源时间戳计算的帧率。填写了 `probe_samples.csv` 时还含触测重复性。 |
| `corners.png` | 首次成功识别时保存的编号图，核对 `0 TL`、`1 TR`、`2 BR`、`3 BL` 与实测记录。 |

有符号偏差统一为 `估计坐标 − 测针参考`。测针触点重复性、TCP 标定和机器人正运动学会影响基座系参考值。解读时把测针散布与视觉相对测针的误差放在一起看。平面拟合残差和 PnP 重投影误差分别衡量两路内部拟合，三维偏差用测针参考计算。
