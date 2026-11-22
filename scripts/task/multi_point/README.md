# 多点运动操作入口

一个程序保留官方单点控制，并可反复加载、规划和执行多点任务。任务走官方 MoveIt 控制链，不自动使能。

## 启动

先在 [CAN 控制面板](../../tools/control_panel/README.md)选择机械臂端口，完成“CAN 连接”和“机械臂使能”，等待六轴使能确认，再启动本任务。驱动读取启动时的使能状态，不能先启动任务再靠面板使能来更新旧驱动。回零不是启动前置步骤。

在项目根目录运行：

```bash
# 持续运行；不加载任务，可先使用官方 RViz 单点控制
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py --rviz --interactive

# 启动后加载并预览指定任务，不立即运动
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py \
  --config scripts/task/multi_point/config/config.yaml --rviz --interactive
```

默认 `--stack auto`：已有完整链路就复用；只有官方驱动时，仅补启动官方 MoveIt；都没有则启动官方链路。已有 RViz 就复用。`--stack attach` 要求链路已启动，`--stack start` 要求没有既有链路。不重复启动驱动；其他不完整链路需先检查原启动进程。

CAN 连接、机械臂使能和阻尼急停统一在控制面板操作。本任务不会自动配置 CAN 或使能；任务停止后的控制门由“恢复控制”显式恢复。面板可保留供阻尼急停使用，正常运动中不要点击回零、重置或重复使能。阻尼急停后先结束旧任务，再按面板说明重置、使能并重新启动任务。

## 两种操作方式

| RViz“多点任务”面板 | 终端命令（输入后回车）  | 功能                                       |
| -------------------- | ----------------------- | ------------------------------------------ |
| 选择文件 → 加载配置 | `l /路径/config.yaml` | 切换任务，不运动；路径含空格时加引号       |
| 规划预览             | `p`                   | 重读当前文件，从当前实测位置重新规划       |
| 执行（实机）         | `e`                   | 执行本轮已预览的轨迹                       |
| 重播                 | 直接 Enter              | 仅更新动画，不运动                         |
| 停止任务             | `s`                   | 请求取消本程序当前任务                     |
| 返回单点             | `m`                   | 退出多点预览，恢复官方显示；不打开控制门   |
| 恢复控制             | `r`                   | 恢复本会话因停止关闭的控制门，不继续旧轨迹 |
| —                   | `q`                   | 退出入口，只清理自己启动的进程             |

两种入口共用一个队列。处理中拒绝重复加载、规划或执行；停止请求优先处理。Ctrl+C 或 q 退出整个入口：先取消当前任务并等待停止处理，再关闭本次启动的 RViz 和官方链路。只想停止任务并保留窗口时，用 s 或“停止任务”。退出清理期间重复按 Ctrl+C 不会中断清理。

每轮成功后回到等待，可再次按 p → e，也可加载新配置。配置为 A → B → C 时，第一轮是当前位置 → A → B → C，第二轮从 C 附近的**实测位置**重新规划。中途通过 RViz 移到 D 后，下一轮从 D 出发。

一次预览只允许一次执行尝试；重新规划后获得新的执行确认。加载新配置、手动运动或反馈异常会清除旧预览与执行确认。执行前还会核对真实起点、模型和规划场景；等待执行服务结束后再次检查取消、起点及控制器活动，发生变化时要求重新规划，不自动补接未预览的轨迹。

## RViz 面板

入口新开的 RViz 自动加载“多点任务”面板，官方 MotionPlanning 保留单点 Plan/Execute。多点执行期间不要同时点击官方 Execute；程序不能锁住官方面板的按钮。

安装、布局、实体与预览模型的区别，以及场景编辑统一见 [RViz 面板说明](rviz_panel/README.md)。单点的 Executed 表示官方执行链返回成功，不包含本工具多点任务的实测 TCP 到位复核。

## 点位配置

任务配置集中放在本任务的 `config/` 目录：`config/config.yaml` 为默认点位，`config/config_0.yaml` 为已有点位变体，`config/scene.yaml` 为可选障碍物。一次性模式未指定 `--config` 时读取 `config/config.yaml`；交互模式未指定时仍等待用户加载。外部自定义配置路径继续受支持，点位内容和执行顺序保持原样。

可将监控工具的关节点位或笛卡尔位姿条目复制到 `points`。同一列表可混用关节点和位姿点。见[监控工具](../../tools/monitoring/README.md)。

以下数值仅说明格式：

```yaml
version: 1
settings:
  frame_id: base_link
  tcp_link: tcp_link
  velocity_scaling: 0.1
  acceleration_scaling: 0.1
  hold_seconds: 0.5
  return_to_initial: false
points:
  - name: point_a
    type: joint
    joints_rad: [0.0, 0.2, -0.5, 0.0, 0.3, 0.0]
  - name: point_b
    type: pose
    frame_id: base_link
    position_m: [0.2, 0.1, 0.3]
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
  - name: point_c
    type: pose
    frame_id: base_link
    position_m: [0.2, 0.1, 0.35]
    rpy: [0.0, 30.0, -15.0]
```

- 关节点使用 `joints_rad`，按 joint1～joint6 排列，单位为弧度。
- 位姿点的 `position_m` 按 `[X, Y, Z]` 排列，单位为米。朝向至少填 `orientation_xyzw` 或 `rpy` 中的一项。
- `orientation_xyzw` 是单位四元数 `[qx, qy, qz, qw]`。`rpy` 按 `[Roll, Pitch, Yaw]` 排列，单位为度，采用固定轴旋转 `Rz(Yaw) · Ry(Pitch) · Rx(Roll)`。
- 只填 `rpy` 时，任务加载时将它转换为四元数。两项都填时，实际旋转角差在 0.001° 以内采用给定的四元数，超过时配置加载会报告点名和角差。
- 名称在同一任务内必须唯一；按列表顺序执行，不按名称编号排序。
- 每点可单独设置 `hold_seconds`。`return_to_initial: true` 返回本轮开始时的位置；启用时 `return_to_initial` 为自动返回点保留名称，用户点位不能同名。
- 单独复制条目不会复制快照模型指纹，需保证模型、参考系和 TCP 一致。

旧点位若使用六项 `xyzrpy`，可把前三项填入 `position_m`、后三项填入 `rpy`。曾使用 `orientation: initial` 的自定义点位需要填写明确的目标朝向。

### 多行写法与监控中的紧凑写法

上面的 `points` 等价于：

```yaml
points:
  - {name: point_a, type: joint, joints_rad: [0.0, 0.2, -0.5, 0.0, 0.3, 0.0]}
  - {name: point_b, type: pose, frame_id: base_link, position_m: [0.2, 0.1, 0.3], orientation_xyzw: [0.0, 0.0, 0.0, 1.0]}
  - {name: point_c, type: pose, frame_id: base_link, position_m: [0.2, 0.1, 0.35], rpy: [0.0, 30.0, -15.0]}
```

多行写法用换行和缩进分隔字段；紧凑写法用 `{}` 包住字段，字段间加逗号。两种写法中的数值列表均可用 `[]`，数字间加逗号。

每个点以 `-` 开头，点与点之间不加逗号。复制完整 `- {...}` 到已有 `points:` 下，缩进两个空格；`points:` 只保留一个，使用空格而非 Tab。

## 可调项

写在 `settings` 中。下表列的是字段省略时的解析器缺省值；随附的多点 `config/config.yaml` 和 `config/config_0.yaml` 将速度、加速度比例都设为 **0.30**，避障任务的随附配置都设为 **0.15**。配置中的值优先，缩放比例不等于实测速度。

| 参数                                             | 省略时缺省值       | 说明                            |
| ------------------------------------------------ | ------------------ | ------------------------------- |
| velocity_scaling / acceleration_scaling          | 0.1 / 0.1          | MoveIt 缩放，范围 (0,1]         |
| hold_seconds                                     | 0.5 s              | 各点默认停留时间                |
| feedback_timeout                                 | 0.5 s              | 关节位置反馈年龄上限            |
| diagnostic_timeout                               | 1.5 s              | 官方 2 Hz 电机/整臂诊断年龄上限 |
| arrival_timeout                                  | 15 s               | 等待实物到位时间                |
| settle_seconds / stable_range_rad                | 0.5 s / 0.003 rad  | 停稳观察窗口与位置变化上限      |
| start_tolerance_rad / joint_tolerance_rad        | 0.01 / 0.02 rad    | 起点匹配与关节到位容差          |
| position_tolerance_m / orientation_tolerance_rad | 0.003 m / 0.02 rad | 笛卡尔到位容差                  |
| execution_margin_seconds                         | 10 s               | 规划时长之外的执行等待余量      |
| approach_offset_m                                | [0, 0, 0]          | 仅加到笛卡尔目标的 XYZ 偏移     |

停留时反馈仍在更新，停留时长不受诊断超时限制。停留期间持续检查诊断、规划关节终点及原始目标误差；位姿点还会用新鲜实测反馈核对 TCP 位置和朝向，超过配置门限即停止，不自动修正。

笛卡尔目标规划使用到位容差的一半，为实机跟踪误差留余量：默认规划位置半径 1.5 mm，姿态各轴容差为 `0.02 / (2√3)` rad；最终实测位置门限仍为 3 mm、总姿态角门限仍为 0.02 rad。MoveIt 执行成功后，使用新鲜的实测关节反馈做正运动学，同时检查关节终点误差、末端位置/姿态误差和停稳窗口。短暂超限会重新开始停稳观察；在 `arrival_timeout` 内仍无法稳定到位则停止，错误中记录点名及末端误差。不自动补发修正运动、不跳过失败点。

## 虚拟障碍物

启动时加 `--scene scripts/task/multi_point/config/scene.yaml`。先编辑其中的 `enabled`、物体位置和尺寸；交互会话首次加载时读取场景；之后使用当前已应用的场景。外部修改文件后，从面板重新加载并应用，避免覆盖尚未另存的编辑。

支持 box `[长,宽,高]`、sphere `[半径]`、cylinder `[高度,半径]`，单位米。`enabled: true` 只替换 `multi_point/` 前缀物体；`objects: []` 清除该前缀。省略文件或 `enabled: false` 不改现有场景。

加载、选择物体、写入草稿、应用和另存为的操作见 [场景编辑](rviz_panel/README.md#场景编辑)。该说明同时适用于多点和避障交互入口。

RViz 手工障碍物须同步到 MoveIt PlanningScene，仅显示 Marker 不参与避障。执行期间不要修改场景；动态避障未实现。

## 其他命令与日志

```bash
# 仅检查文件，不连接 ROS/CAN
bash scripts/piperl/run.sh --python-only python scripts/task/multi_point/run.py \
  --config scripts/task/multi_point/config/config.yaml --check-config
```

不加 `--interactive` 保留一次性模式：默认只规划；`--execute` 启动后直接执行。`--interactive` 与 `--execute` 不能同时使用。

持续会话日志：`outputs/tasks/multi_point/<会话>/rounds/<轮次>/`；每轮执行记录在其 `executions/<时间>/` 下。预览成功不代表实机到位，应查看执行目录的 `result.json`。

退出时检查本次启动的整个进程组，启动器先退出也会继续清理驱动子进程。依次发送 SIGINT、SIGTERM、SIGKILL，分别最多等待 6、3、2 秒；日志 `process_exit.json` 记录信号和剩余进程。复用的外部驱动或 RViz 不属于本次进程组，仍由其原启动终端关闭。进程退出不代表机械臂自动失能或硬件急停。

任务失败后保留入口，不自动重试。停止已提交的运动时，取消轨迹并关闭官方控制转发门；切换单点/多点不会开门。

出现 `PLANNING` 后失败表示规划阶段失败，配置可能已经加载成功。错误会指出第几个点及其名称。`99999 (FAILURE)` 是 MoveIt 通用失败码，单凭该码不能断定碰撞或逆解无解；查看会话的 `official_stack.log`（复用外部 MoveIt 时查看其启动终端）。若提示 `Unable to sample any valid states for goal tree`，表示本次未采到满足目标的有效状态，应核对完整位置、朝向、模型/TCP、限位和碰撞场景。

每轮 `initial_pose.json` 保存初始 TCP 位姿，`resolved_points.json` 保存应用偏移后的目标位姿。日志中的位姿使用米制位置和四元数。`NNN_request.json` 和 `NNN_scene.json` 保存每次规划的起点、目标与场景。失败时另存 `planning_failure.json`，包含失败点、起点及 MoveIt 返回的错误码和消息（如有）。失败预览不可执行，不会自动跳过目标或放宽约束。

执行异常时优先取消动作并关闭控制门，再记录失败；日志写入失败不会跳过停止处理。异常退出会尝试更新各段记录和总结果；磁盘不可写时错误输出到终端，原始任务异常仍保留。

**恢复流程：停止 → 恢复控制（r）→ 单点操作，或重新规划 → 执行。**

恢复控制不会自动续跑。再次规划执行时，从当前实测位置重新到第 1 点，再按配置顺序执行全部目标；不会从上次停止点跳过前面的目标。

恢复时检查旧动作已终止、反馈正常且停稳；保持门关闭，经官方控制器把保持目标同步到实测关节位置，核对输出后才开门。开门请求期间收到停止，会再次请求关门。检查失败保持锁定，原因见终端。不会自动使能、继续旧轨迹或打开其他程序关闭的门。此操作只处理本会话的停止；入口重启不自动恢复门。官方自动开门节点与此流程冲突，入口检测到 `control_gate` 时拒绝接入，需使用 `auto_control_gate:=false`。

停止请求不等于硬件急停，位置稳定观察也不是硬件停止确认。只关闭自己启动的进程，不关闭复用的官方窗口或自动失能。

### 实测起点越界提示

面板只显示一行状态，详细错误和文件路径放在悬停提示及终端日志中。若显示“起点越界”，本轮尚未生成可执行轨迹：当前真实反馈不在模型范围内，修改场景无效。未使能、自然下垂时也可能出现；使能本身不保证恢复有效姿态。按正常操作流程恢复有效姿态后重新规划；若反馈与实际姿态不符，核对零位和模型。任务不截断真实反馈，不放宽限位，不自动使能。

即使在起点检查失败时，`initial_feedback.json` 也会保存原始六关节反馈、时间戳及限位，`result.json` 保存全部越界关节及 `source: real_feedback`。

### 选择物体与草稿操作

见 [面板的选择物体与草稿操作](rviz_panel/README.md#选择物体与草稿操作)。

### 预览遇到反馈短暂中断

反馈等待窗口、错误含义与日志字段见 [面板的反馈短暂中断说明](rviz_panel/README.md#预览遇到反馈短暂中断)。
