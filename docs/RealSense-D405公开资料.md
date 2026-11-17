# RealSense D405 公开资料

核验日期：**2026 年 11 月 17 日**。

按《PiPER-L 公开资料》的“链接—型号对应关系—内容与核验说明”格式整理。覆盖官方硬件、机械安装、SDK、Python、ROS 2、深度处理、标定、排错、应用代码和研究论文；这是公开资料索引与简要说明，不是网页、视频、安装包或 PDF 的完整离线镜像，也不能保证穷尽互联网。重复转载、无原始出处的参数表和未核验的下载站不作为主要依据。

优先使用 `realsenseai.com`、`dev.realsenseai.com` 和 GitHub `realsenseai` 的新入口。官方 ROS 仓库已公告从 `IntelRealSense` 迁移至 `realsenseai`；旧教程中的域名、仓库名、ROS 参数和版本号需重新核对。下文“页面可读”仅代表本次网络读取结果，不表示代码、固件、附件或设备已实际验证。

先列官方产品、规格、机械模型和合规资料。

| 链接 | D405 对应关系 | 内容与核验说明 |
|---|---|---|
| **[D405 官方中文产品页](https://www.realsenseai.com/cn/products/d405/)** | **D405 专属** | 页面可读。集中介绍近距离双目、全局快门、RGB 来源、工作范围及应用；适合中文入门。 |
| **[D405 Series 官方产品页](https://www.realsenseai.com/products/d405-series/)** | **D405 系列** | 页面可读。面向机械臂近距定位、抓取、对位与装配，也涉及双 RGB 和相关软件能力；新功能须结合 SDK 与硬件支持条件使用。 |
| **[官方数据手册总目录](https://www.realsenseai.com/product-datasheets/)** | **明确列出 D405** | 页面可读，D400 数据手册条目明确包含 D405；应从此处复查后续手册版本。 |
| [D400 系列数据手册，2026 年 3 月 PDF](https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf) | 系列共用，含 D405 专用表格和图纸 | 官方目录当前指向此文件。搜索索引可检索到正文；本次直接打开超时，**未完整下载、逐页核验**。型号专属表格优先于其他 D400 型号的参数。 |
| [开发文档中的 Series Datasheet 入口](https://dev.realsenseai.com/docs/realsense-series-datasheet/) | 官方规格导航 | 页面可读，可与产品站的手册目录交叉查找；导航页面不等于已读取所有附件。 |
| **[官方 CAD 文件目录](https://dev.realsenseai.com/docs/cad-files/)** | D400 相机共用下载入口 | 页面可读，列出 D400 Depth Cameras、Depth Modules、D4 Boards 等资源。相机外壳、D401 模组与 D4 板不是同一安装对象。 |
| [D400 相机 CAD 下载](https://dev.realsenseai.com/download/41950) | 官方 CAD 目录指向的相机附件 | 已确认目录链接解析为 ZIP 下载；**未解包确认 D405 文件清单和尺寸**，不能当作已完成支架设计核验。 |
| [官方合规与材料声明目录](https://www.realsenseai.com/realsense-regulatory-information/) | 包含 D405 条目 | 页面可读，可查认证和材料声明；具体附件未逐个下载，适用地区与型号应按证书核对。 |

读规格时，先区分以下几个容易混淆的概念。下表参数取自上述官方中文产品页；它是概览，配置组合和测试条件仍以数据手册及设备实际枚举结果为准。

| 项目 | D405 资料中的表述 | 使用边界 |
|---|---|---|
| 常规理想工作距离 | 7–50 cm | 不是任意表面、光照和分辨率下都有效的硬边界；近距能力也与配置有关。 |
| 深度与视场角 | 立体深度，全局快门；87° × 58° | 不要直接套用 D435/D455 的基线、视场角或投射器参数。 |
| 分辨率与帧率 | 深度最高 1280×720；帧率最高 90 fps | 两个“最高”不证明能同时达到；须核对具体流配置与 USB 带宽。 |
| RGB | 左侧成像器 RGB 经 ISP 处理 | 不是 D435 式的独立 RGB 相机布局；曝光、流参数和内外参须按 D405 处理。 |
| 精度 | 产品页列出 ±2% @ 50 cm，同时介绍近距亚毫米能力 | “可辨认小物体”、深度绝对误差、噪声和机器人定位总误差是不同指标；不能概括为全范围固定 0.1 mm 或小于 1 mm。 |
| 外形与惯性测量 | 42×42×23 mm；无 IMU | 不包含支架、线缆与固定件；不能从其他带 IMU 型号推断存在加速度计或陀螺仪话题。 |

官方 SDK、安装、工具、API 和 Python 资料如下。此处给资料入口，不表示已向本项目安装依赖或添加相机驱动。

| 链接 | D405 对应关系 | 内容与限制 |
|---|---|---|
| **[librealsense 官方 SDK 仓库](https://github.com/realsenseai/librealsense)** | RealSense 系列 SDK | 页面可读，包含源码、工具、样例和多语言封装；使用具体发布版时应锁定 tag/commit。 |
| **[SDK Releases](https://github.com/realsenseai/librealsense/releases)** | SDK、固件及设备支持核对入口 | 发布页可读。区分正式版、预发布版和对应固件，不能将最新测试版默认当作本项目兼容版本。 |
| [SDK 功能支持矩阵](https://github.com/realsenseai/librealsense/blob/master/doc/support-matrix.md) | 系列能力核对 | 页面可读；用于核对接口与平台支持，不能由系列功能名称推断 D405 支持全部功能。 |
| **[Linux/Ubuntu 二进制包安装](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)** | Linux 共用 | 页面可读，涉及软件源、SDK、开发包及工具。安装前核对 Ubuntu、架构与内核版本。 |
| [Linux 源码构建](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md) | Linux 共用 | 页面可读，用于自定义构建与平台适配；不要混装多个来源的 librealsense。 |
| [NVIDIA Jetson 安装](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md) | Jetson 平台共用 | 页面可读；需按 JetPack/L4T、内核与架构选择方式，不能照搬 x86 Ubuntu 步骤。 |
| **[RealSense Viewer 手册](https://github.com/realsenseai/librealsense/wiki/Realsense-Viewer-Manual)** | 系列共用，功能依型号 | 页面可读。用于识别设备、查看流、参数、深度与点云；手册中的校准和高级设置需确认 D405 是否提供。 |
| [官方命令行工具源码目录](https://github.com/realsenseai/librealsense/tree/master/tools) | 系列共用 | 目录可读。设备枚举、录制转换、固件与质量工具从此查找；工具存在不代表全部支持 D405。 |
| [官方 C/C++ 示例目录](https://github.com/realsenseai/librealsense/tree/master/examples) | 系列共用 | 目录可读。可查采集、对齐、点云、录制回放等示例；检查是否假设独立彩色传感器或 IMU。 |
| **[Python / pyrealsense2 封装](https://github.com/realsenseai/librealsense/tree/master/wrappers/python)** | Python 接口 | 页面可读。安装及绑定说明；wheel 是否适配 Python 3.12、CPU 架构和系统，应按实际发布文件判断。 |
| [Python 示例目录](https://github.com/realsenseai/librealsense/tree/master/wrappers/python/examples) | 通用采集与处理示例 | 目录可读；适合学习 pipeline、帧读取、对齐、点云与回放，具体流配置需改成 D405 支持的组合。 |
| [C++ API Reference](https://realsenseai.github.io/librealsense/doxygen/annotated.html) | SDK API 文档 | 页面可读。查询设备、传感器、帧、内外参、深度比例与处理接口；API 能力以设备实际暴露的选项为准。 |
| [Projection in RealSense SDK 2.0](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0) | 系列共用几何基础 | 页面可读。像素与三维点的投影/反投影、内参、畸变与外参；用于将深度转换为相机坐标，尚需手眼变换才能进入机械臂基坐标系。 |

官方固件、校准和深度质量资料如下。相机内参/双目深度校准、RGB 与深度关系、相机到机械臂的手眼标定属于不同问题，不应混称为同一次“标定”。

| 链接 | D405 对应关系 | 内容与核验说明 |
|---|---|---|
| **[D400 固件与软件更新入口](https://dev.realsenseai.com/docs/realsense-d400-firmware-software-updates/)** | D400 系列共用 | 页面可读，查固件与软件资料；下载前核对型号，不将其他设备的固件用于 D405。 |
| [D400 固件发布说明](https://dev.realsenseai.com/docs/firmware-releases-d400/) | 系列共用版本信息 | 页面可读；与 SDK Releases 交叉核对推荐组合，本索引不固定宣称某一版本永久最新。 |
| [rs-fw-update 官方说明](https://dev.realsenseai.com/docs/firmware-update-tool/) | 系列固件工具 | 页面可读，说明升级工具及操作；本次仅收录，未执行固件更新。 |
| **[相机校准工具总入口](https://dev.realsenseai.com/docs/calibration/)** | D400 等型号共用导航 | 页面可读；从此查找用户指南、API 和工具支持条件。 |
| [D400 校准工具用户指南入口](https://dev.realsenseai.com/docs/user-guide-for-realsense-d400-series-calibration-tools/) | 系列共用 | 页面可读；附件与具体型号支持未逐项验证，不能仅因标题包含 D400 就认定所有流程适用 D405。 |
| [D400 校准工具与 API 程序员指南入口](https://dev.realsenseai.com/docs/d400-dynamic-calibration-quick-start-guide/) | 系列共用 | 页面可读。用于查校准工具接口与实现资料；涉及写入校准数据的流程应先确认设备与工具版本。 |
| [D400 自校准技术说明](https://dev.realsenseai.com/docs/intel-realsense-self-calibration-for-d400-series-depth-cameras/) | 系列技术背景 | 页面可读，介绍自校准方法；D405 的照明和工具支持限制另见下文直接针对 D405 的讨论。 |
| **[深度质量测试方法](https://dev.realsenseai.com/docs/camera-depth-testing-methodology/)** | 系列测量方法 | 页面可读，可用于区分准确度、空间噪声、时间噪声与有效点比例；应记录距离、ROI、目标材质和曝光等条件。 |
| [最佳深度性能调参指南](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/) | 系列通用方法 | 页面可读。介绍影响立体匹配的成像与参数因素；投射器相关步骤不应直接套给没有相应组件的相机。 |
| **[D400 深度后处理白皮书](https://dev.realsenseai.com/docs/depth-post-processing-for-intel-realsense-depth-camera-d400-series/)** | 系列算法参考 | 页面可读。抽样、空间/时间处理和补洞等方法；补出的深度不等同于新测量，机器人感知需关注延迟和边缘误差。 |
| [投影、纹理映射与遮挡](https://dev.realsenseai.com/docs/projection-texture-mapping-and-occlusion-with-intel-realsense-depth-cameras/) | 系列几何处理 | 页面可读。理解 RGB-D 映射、遮挡和坐标关系；对齐不能替代机械臂手眼标定。 |
| [重复纹理影响与缓解](https://dev.realsenseai.com/docs/mitigate-repetitive-pattern-effect-stereo-depth-cameras/) | 双目匹配问题参考 | 页面可读；可查条纹、重复结构导致的深度异常，具体改善效果需用目标场景验证。 |
| [多相机配置说明](https://dev.realsenseai.com/docs/multiple-depth-cameras-configuration/) | D400 系列共用 | 页面可读；用于了解多机、带宽和同步问题，不能据此认定 D405 具有其他型号的硬件同步接口。 |
| **[Min-Z：将双目近距深度扩展至约 2 cm](https://www.realsenseai.com/news-insights/extending-stereo-depth-to-2-cm-with-min-z/)** | **正文明确提到 D401/D405** | 官方文章可读。将 D401/D405 的增强近距感知目标描述为约 2 cm，强调材质、几何、纹理和照明条件。**这是增强处理路径，不能改写为标准原生深度流在所有配置下都支持 2 cm；软件获取、运行要求和实机效果未验证。** |

ROS 2 与模型资料中，优先核对 D405 专用参数和模型。本项目使用 Ubuntu 24.04 / ROS 2 Jazzy，下面官方安装页明确列出该环境，但本次没有安装或启动相机节点。

| 链接 | D405 对应关系 | 内容与限制 |
|---|---|---|
| **[realsense-ros 官方 ROS 2 驱动](https://github.com/realsenseai/realsense-ros)** | 系列驱动，`ros2-master` | README 可读。包含 Ubuntu 24.04/Jazzy 安装、话题、流参数、点云、滤波、TF 和光学坐标说明；建议按发布版选择兼容 SDK。 |
| [ROS 驱动 Releases](https://github.com/realsenseai/realsense-ros/releases) | ROS/SDK 版本配套入口 | 页面可读；核对目标发布版的 Supported RealSense SDK，避免直接组合任意最新版。 |
| **[官方启动参数 rs_launch.py](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py)** | **源码明确注明 D405** | 可检索到 `depth_module.color_profile` 和 `depth_module.color_format` 为 D405 彩色流参数；不要只照搬其他型号的 `rgb_camera.*` 配置。 |
| **[D405 专用 Xacro](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d405.urdf.xacro)** | **D405 专属模型源码** | 页面可读，可作为相机外形与内部 frame 的 ROS 集成参考。相机相对 PiPER-L 法兰的安装位姿仍需另行测量或手眼标定。 |
| [官方相机 URDF 目录](https://github.com/realsenseai/realsense-ros/tree/ros2-master/realsense2_description/urdf) | 多型号模型集合 | 目录可读；应选择 D405，不能用 D435 外形、光学中心或 frame 偏移替代。 |

以下为官方支持平台或官方仓库中的实际问题讨论。讨论内容是特定版本、场景下的排错线索，不等于数据手册参数，也不证明本机已经发生或解决同类问题。

| 链接／作者 | D405 对应关系 | 内容与限制 |
|---|---|---|
| **[Calibration for D405](https://support.realsenseai.com/hc/en-us/community/posts/13077536864659-Calibration-for-D405)** | **直接针对 D405** | 页面可读。讨论 Tare/On-Chip 校准；答复指出 D405 没有 IR 投射器，场景外部照明不足时校准可能困难。不能把重新校准当作所有深度问题的默认解决方式。 |
| [D405 在 30 cm 重建与跟踪中的考虑](https://support.realsenseai.com/hc/en-us/community/posts/48925794889235-Advice-D405-D435-Considerations-for-3D-object-reconstruction-and-tracking-at-30-cm-working-distance) | D405/D435 对比 | 检索到支持答复，讨论纹理、近距质量和缺少 IR 投射器的影响；本次未逐项复现实验。 |
| [近距小物体、点云与 Disparity Shift](https://support.realsenseai.com/hc/en-us/community/posts/19659782434323-3D-scanning-of-objects-max-size-2x2x2cm-at-a-distance-5-to-10cm-and-produce-memory-accessible-pointcloud) | 明确讨论 D405 | 检索到近距调参与点云导出建议；修改视差范围会改变可测区间，不能当作不损失其他范围的免费扩展，也不等于上文 Min-Z 方法。 |
| [D405 无法在 Viewer 工作，Issue #12751](https://github.com/realsenseai/librealsense/issues/12751) | D405 专属问题 | 页面可读，涉及设备识别及校准数据排错；恢复出厂校准等写入操作只应在确认问题后按适配流程处理。 |
| [SDK Troubleshooting Q&A](https://github.com/realsenseai/librealsense/wiki/Troubleshooting-Q%26A) | 系列通用排错 | 页面可读，作为连接、权限与环境问题入口；按系统、SDK 与设备版本筛选。 |

以下为项目作者、设备厂商或研究团队发布的一手应用代码与论文。这里只核验公开描述及入口，**没有克隆运行、验证算法结果或进行实机运动测试**；PiPER、PiperX、TRON2 的控制接口与模型不能直接当作 PiPER-L 的实现。

| 链接／作者 | D405 对应关系 | 内容与限制 |
|---|---|---|
| **[tron2_visual_grasp／LimX Dynamics](https://github.com/limxdynamics/tron2_visual_grasp)** | **腕部 D405、手眼标定与抓取** | README 可读，含标定、目标检测和抓取流程，也有 PiperX 相关适配。属于其他机器人平台；其中自动采样流程会移动机器人，不能直接运行于本项目。 |
| [piper-vision-grasp-new／xsb2lyang](https://github.com/xsb2lyang/piper-vision-grasp-new) | 明确 PiPER + D405/D435i | README 可读，纯 Python、YOLO、手眼标定与桌面抓取；未确认 PiPER-L 专用支持或与本项目官方 ROS 链路兼容。 |
| [Eye-to-Hand_Camera_Calibration／ElijahAlmanzor](https://github.com/ElijahAlmanzor/Eye-to-Hand_Camera_Calibration) | 明确包含 D405 眼在手外 | 页面及检索正文可读。采集、AX=XB 求解与变换验证；作者要求适配机器人位姿来源、板尺寸、相机序列号及坐标方向。 |
| [calibrate_Realsense／PhamHuygaco](https://github.com/PhamHuygaco/calibrate_Realsense) | D405 双臂手眼标定 | README 可读，目标为定制 4 自由度双臂；其误差评价阈值只代表作者应用要求，不能作为 D405 官方精度或本项目验收标准。 |
| [rs-d405-depth-toolkit／boheastill](https://github.com/boheastill/rs-d405-depth-toolkit) | D405 近距滤波与融合 | README 可读，含合成场景评测与 `.bag` 数据接入；合成数据结果不能当作实机精度证明。 |
| [Comparing Commercial Depth Sensor Accuracy for Medical Applications](https://arxiv.org/abs/2606.13028) | 研究明确比较 D405 等四类传感器 | arXiv 摘要页可读，2026 年预印本；约 50 cm 场景包含低纹理、镜面反射和次表面散射。仅核验摘要，未重跑数据或将结果泛化到采摘。 |
| [Benchmarking commercial depth sensors for intraoperative markerless registration in neurosurgery applications](https://link.springer.com/article/10.1007/s11548-025-03416-y) | 明确包含 D405 的对比研究 | 出版社开放获取页面可读，2025 年论文，DOI `10.1007/s11548-025-03416-y`；注册误差等任务指标不能直接等同于原始逐像素深度精度。 |
| [Comparative evaluation of low-cost 3D scanning devices for ear acquisition](https://flore.unifi.it/handle/2158/1375716) | D405 与 iPhone TrueDepth 对比 | 大学机构库搜索摘要明确涉及 D405；本次页面返回 **403**，仅收录可定位的论文入口，**未核验全文**。 |

面向本项目，建议先读“中文产品页 → 数据手册 → Viewer → SDK/Python → ROS 2 启动参数 → D405 Xacro → 手眼标定与质量评估”。以下为结合资料提出的集成建议，不表示已经实现：先枚举设备实际支持的流与深度比例，记录 SDK/固件版本并采集静态数据；确认内参、光学坐标与 `base_link`/TCP 的变换后，再进行识别与三维定位。相机点云需明确接入 MoveIt PlanningScene/OctoMap 才能用于环境碰撞检查，仅在 RViz 显示图像或 PointCloud2 不会自动形成避障能力。

尚未核验：实体 D405 的连接与序列号、当前固件、USB 实际速率、Jazzy 驱动运行、各分辨率/帧率组合、手眼标定结果、花朵/叶片等目标上的误差，以及 Min-Z 软件部署与性能。本文不包含这些项目已通过的结论。
