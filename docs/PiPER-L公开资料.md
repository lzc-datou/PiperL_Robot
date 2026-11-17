# PiPER-L 公开资料



核验日期：**2026 年 11 月 7 日**。

先列官方硬件、接线、通信和校准资料。

| 链接 | PiPER-L 对应关系 | 内容与核验说明 |
|---|---|---|
| [官方中文手册目录](https://agilexsupport.yuque.com/staff-hso6mo/dna5gm/gi7odbvgetmuvtzw?singleDoc) | 官方导航 | 浏览器免登录可读，分别列出 PiPER、X、L、H，可用于确认型号与官方文档入口。 |
| **[PiPER-L 中文用户手册](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/fvtsoyfpewxeuy5v?singleDoc)** | **L 专属** | 全文可读，版本表为 V1.1.0、2026/1/25。涵盖规格、供电接线、接口、坐标系、安装、负载曲线、CAN、示教、主从、上位机、错误清除和维护。官方参数包括 **750 mm、1.5 kg 负载、4.5 kg 本体、DC 24 V**。 |
| [PiPER-L 机械模型附件页](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/qqnssyindu0y2vgw) | L 专属 CAD 资源 | 附件清单可见，含 `PIPER-L对外.tar`、夹爪、示教器、不同法兰及相机支架。**未下载解包核验**；机械 CAD 与下文 URDF 仿真模型用途不同。 |
| [PiPER-L CAN 协议入口](https://agilexsupport.yuque.com/staff-hso6mo/alxgtf/gxh6w5tdqkglyoqp) | L 官方入口，协议共用 | 页面可读，正文指向下一项系列通信协议。 |
| [机械臂对外通信协议表](https://agilexsupport.yuque.com/staff-hso6mo/tetraz/qii9wce225nvworv#7305) | 系列共用，L 官方入口确认适用 | 浏览器中的电子表格可读。包含 CAN2.0B 标准帧、1000K、Motorola/MSB，以及控制、反馈、参数查询设置的 ID 和逐字节字段；需结合实际固件版本使用。 |
| **[Piper 系列固件升级与零点设置](https://agilexsupport.yuque.com/staff-hso6mo/ihw47g/svzsuopxhgd4n8w5)** | **明确包含 L** | 页面全文可读，单列 PIPER-L 固件附件入口，**第 3.3 节为 H/L 校零视频**，另含夹爪零点和升级后检查。视频与固件附件未逐个下载验证。 |

官方软件和仿真资料中，新版统一生态已明确提供 `piper_l` 型号选择；旧版资料的限制列在相应条目中。

| 链接 | PiPER-L 对应关系 | 内容与限制 |
|---|---|---|
| [AgileX 官方开源总索引](https://github.com/agilexrobotics/agilexrobotics/blob/master/README.md) | 明确列出 L | 汇总官方 SDK、ROS、URDF、仿真和学院教程，也可用于确认各仓库的官方归属。 |
| **[pyAgxArm](https://github.com/agilexrobotics/pyAgxArm)** | **明确支持 L** | 统一 Python SDK，提供 CAN、运动控制、状态反馈和末端执行器接口；涵盖 Linux、Windows、macOS 的使用说明。 |
| [Piper API 中英文手册](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/piper_api.md) | 参数明确列出 `PIPER_L` | 关节与笛卡尔运动、MIT/CPV、TCP、状态、主从、关节标定和错误清除等 API。 |
| **[固件兼容矩阵](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/piper/firmware_reference.md)** | 明确包含 `piper_l` | 说明 DEFAULT、V183、V188、V189 与固件的对应关系，以及 MIT 帧、力矩换算、IK 反馈等版本差异。 |
| [PiPER-L 专用 SDK 示例](https://github.com/agilexrobotics/pyAgxArm/blob/master/pyAgxArm/demos/piper_l/test1.py) | L 专属示例 | 源码明确选择 `ArmModel.PIPER_L`，包含多种运动、夹爪、TCP、主从和反馈示例。示例的固件选项仍需按设备修改。 |
| [CAN 模块配置手册](https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/can_user.md) | 统一 SDK 共用 | 介绍单/多 CAN 模块识别与激活，以及 Linux SocketCAN、Windows、macOS 配置。文档说明其流程面向随臂提供的官方模块。 |
| **[agx_arm_ros：ROS 2 驱动](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)** | **明确支持 `arm_type:=piper_l`** | 官方 `ros2` 分支，README 标记 Humble、Jazzy 支持；包含控制、反馈话题、服务和模型集成，依赖 pyAgxArm。 |
| **[统一 MoveIt 2 文档](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/src/agx_arm_moveit/README.md)** | **明确支持 L** | 无实机规划、实机连接、夹爪、TCP、轨迹执行和反馈配置。属于新版实机集成入口。 |
| [PiPER-L 官方 URDF/Xacro/网格目录](https://github.com/agilexrobotics/agx_arm_urdf/tree/main/piper_l) | L 专属 | 含基础机械臂及末端组合模型、显示和碰撞网格，可用于 ROS、规划和仿真。 |
| [PiPER-L 基础 URDF](https://github.com/agilexrobotics/agx_arm_urdf/blob/main/piper_l/urdf/piper_l_description.urdf) | L 专属源码 | 已读到 `robot name="piper_l"`、六关节、惯性和碰撞定义。模型中的限位配置不能替代硬件手册的标称规格。 |
| **[agx_arm_sim：官方仿真工具集](https://github.com/agilexrobotics/agx_arm_sim)** | **支持表明确列 L** | 当前支持表列出 L 型的 **Isaac Sim、MoveIt 2、Gazebo**；**L 型 MuJoCo 栏为空**，不能将标准 PiPER 的 MuJoCo 模型直接当作 L。 |
| [仿真模型使用说明](https://github.com/agilexrobotics/agx_arm_sim/blob/master/agx_arm_description/README.md) | 明确包含 L | 型号、夹爪、示教器、相机等组合配置。注意其末端参数名与实机驱动仓库有所不同。 |
| [PiPER-L＋夹爪＋D435 的 USD 目录](https://github.com/agilexrobotics/agx_arm_sim/tree/master/agx_arm_description/urdf/USD/piper_l_gripper_d435) | L 专属 Isaac Sim 资源 | 已核验目录及 USD 文件存在；**未加载二进制模型运行验证**。 |
| [PiPER-L MoveIt 2/Gazebo 配置包](https://github.com/agilexrobotics/agx_arm_sim/tree/master/Moveit2/piper_l_gripper_moveit_config) | L 专属 | 含 SRDF、运动学、限位、控制器和启动文件。其 Gazebo 集成使用 Classic 风格接口，不能据此认定适配全部新版 Gazebo。 |
| [piper_sdk 中文说明](https://github.com/agilexrobotics/piper_sdk/blob/master/README%28ZH%29.MD) | 系列 SDK，当前文档提及 L | 原有 Python SDK 的安装、控制、反馈和示例入口；其 API 与 pyAgxArm 不同，需核对固件及运动学参数。 |
| [V2 API](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/V2/INTERFACE_V2.MD) ／ [V3 API](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/V3/INTERFACE_V3.MD) | 系列共用接口文档 | 均已读取。描述控制、反馈、限位与配置接口；README 要求固件 1.8-8 及以后使用 V3 接口。 |
| [piper_sdk Q&A](https://github.com/agilexrobotics/piper_sdk/blob/master/assets/Q%26A.MD) | 系列参考 | 夹爪无反馈、Jetson 缺少 `gs_usb`、MIT 力矩和串口 CAN 等排错说明。力矩处理尤其需要核对固件。 |
| [piper_ros／Noetic](https://github.com/agilexrobotics/piper_ros/tree/noetic) | 官方声明覆盖 L，模型需核对 | ROS 1 驱动。官方总索引列出 L 兼容，但此分支未发现 L 专属模型选择，不能默认其规划模型就是长臂版。 |
| [piper_ros／Humble](https://github.com/agilexrobotics/piper_ros/tree/humble) | 旧 ROS 2 系列参考 | 适合维护旧项目；README 使用标准 `piper_description`，未提供明确的 L 型选择。 |
| [Agilex-College 官方教程集合](https://github.com/agilexrobotics/Agilex-College) | 系列参考 | 点位与轨迹录放、视觉、手机/手势/手柄遥操作、MoveIt 和 IsaacLab 示例；需逐项核对型号。 |
| [官方手眼标定教程](https://github.com/agilexrobotics/Agilex-College/blob/master/piper/handeye/README.md) | 标准 PiPER 示例，L 需适配 | ROS 2 Humble，包含眼在手上、眼在手外、ArUco 和相机话题配置；没有给出 L 独立示例，需匹配实际模型与末端位姿。 |

以下为社区或独立研究团队的一手资料。即使讨论位于官方仓库内，也不等同于官方性能承诺或已确认的解决方案。

| 链接／作者 | PiPER-L 对应关系 | 内容与限制 |
|---|---|---|
| [piper-ros2-dls／IIT DLSLab](https://github.com/iit-DLSLab/piper-ros2-dls/blob/main/README.md) | 明确使用 `piper_l` | 基于官方新生态的 ROS 2 集成，含 PD＋重力补偿、DLS 接口和 MoveIt；安装资料涉及 Jazzy。 |
| [sim2real-robot-identification／IIT DLSLab](https://github.com/iit-DLSLab/sim2real-robot-identification) | 支持表明确列 L 已辨识 | 电机、质量、惯量、质心等动力学参数辨识，支持 MuJoCo/IsaacLab。这里的标定指系统辨识。 |
| **[PiPER-L 社区 MJCF/USD 模型目录](https://github.com/iit-DLSLab/sim2real-robot-identification/tree/main/robot_model/piper_l)** | **L 专属** | 含 `piper_l.xml`、原始模型和生成的 USD，可作为 L 型 MuJoCo 资源；辨识后的参数不能当作官方出厂数据。 |
| [LocoPilot 仓库](https://github.com/iit-DLSLab/LocoPilot) ／ [原始论文](https://arxiv.org/abs/2610.27998) | 明确使用两台 PiPER-L | Go2 移动操作、主从遥操作、力反馈和数据采集；仓库及论文摘要页均可读。 |
| [piper_control／Reimagine Robotics](https://github.com/Reimagine-Robotics/piper_control) | 源码明确包含 `PIPER_L` | Python 控制封装，包含 L 的限位和力矩系数；默认机型及内置重力补偿模型仍需单独核对。 |
| [piper_joint_offsets／Reimagine Robotics](https://github.com/Reimagine-Robotics/piper_joint_offsets) | 声明支持 `PIPER_L` | 利用机械限位估计关节零偏。属于社区方法，未找到 L 实机验证证明；自动模式会向机械限位施加力矩。 |
| **[MOVE_L、反馈与完成判据讨论 #65](https://github.com/agilexrobotics/pyAgxArm/discussions/65)** | **直接针对 PiPER-L** | 讨论直线运动、逆解连续性、反馈和完成检查，明确给出固件环境。帖中实测误差不属于官方容差规格。 |
| [PiPER-L／gs_usb 故障报告 #72](https://github.com/agilexrobotics/pyAgxArm/issues/72) | 直接针对 PiPER-L | Jetson、SocketCAN 下 `Unexpected unused echo id` 等问题；核验时尚无解决答复。 |
| [PiPER-L 夹爪 Pace 辨识问题 #14](https://github.com/iit-DLSLab/sim2real-robot-identification/issues/14) | 直接针对 L 模型 | 作者记录 Pace 与 MuJoCo 下夹爪拟合差异；核验时仍是开放问题。 |
| [PiPER-L 视觉拼图仿真／lvreng](https://github.com/lvreng/puzzle-vision-simulator/blob/main/mujoco_sim/README.md) | 中文项目，明确 L | MuJoCo、视觉识别、数值逆解、五次轨迹和磁吸抓放全流程；模型来源为作者复制的描述包，官方版本对应关系未确认。 |

