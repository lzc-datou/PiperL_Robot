# RViz 多点任务面板

独立面板 `piper_multi_point_rviz/TaskPanel`；单点规划执行仍使用官方 MotionPlanning 实现。

```bash
bash scripts/piperl/build.sh
bash scripts/piperl/run.sh python scripts/task/multi_point/run.py --rviz --interactive
```

新窗口自动加载；旧窗口通过 Panels → Add New Panel 添加。首次安装后找不到插件时，重启 RViz 并使用项目环境。

面板仅保留文件、操作按钮和一行状态；详细日志在启动终端。

交互入口生成的启动配置启用 `Default Left Layout`：窗口就绪后，将 MotionPlanning/Planning 页放在左侧上方，多点任务放在左侧下方，并隐藏 Displays、Views、Help 和轨迹滑条。`default_layout.hpp` 使用 Qt 停靠布局，只应用一次；用户另存布局时该开关保存为 false，不反复覆盖手动排版。旧窗口手动添加面板不触发重排。

按钮：选择/加载配置、规划预览、执行（实机）、重播、停止任务、返回单点；停止运动后显示“恢复控制”。与终端菜单共用任务队列，状态断开时禁用任务按钮。面板本身不创建驱动，也不发送机械臂轨迹。

接口：`/piper_multi_point/command`（TaskCommand 服务）、`/piper_multi_point/status`（JSON 状态）。与官方话题/服务分离。关闭面板不停止后台入口；退出入口用终端 q。

`timed_displays.cpp` 为本机 RViz 14.1.23 修正帧时间单位，避免动画瞬间跳完、循环闪烁；其他版本直接传递原值。新窗口自动使用兼容分组；旧窗口添加面板时将现有显示项移入该分组，保留显示设置与交互目标。官方仓库和系统库不修改。

预览使用独立 Trajectory 显示项 `/multi_point/display_planned_path`。实体模型保留显示，跟随官方真实反馈场景，Alpha 为 1；单点与多点规划动画使用 Alpha 为 0.35 的虚影，关闭 `Color Enabled`，保留模型原色。单点恢复官方 MotionPlanning 的黄色目标模型，起终点编辑模型 Alpha 为 0.45；三维交互球与可见的半透明目标模型联动，保留官方拖动、逆解、规划和执行行为。返回单点时隐藏多点预览、清除标记，恢复官方轨迹显示；不重置官方交互标记。更新后重新打开 RViz 生效。

“返回单点”仅切换显示，不开控制门。“恢复控制”检查旧动作、反馈和停稳状态，隔离同步控制器保持位置后开门；不使能、不继续旧轨迹。多点执行需重新规划。

完整命令和限制见[任务说明](../README.md)。

单点轨迹默认关闭 `Loop Animation`，避免回放虚影与真实实体的位置混淆。


## 场景编辑与双路径模式

面板分为“运动”和“场景”两页。场景页加载任务 YAML，提供 box/sphere/cylinder 的新增、删除、米制尺寸/位置、角度制 RPY 和三维六轴手柄。数值先“写入草稿”，拖拽松开时更新草稿，再“应用场景”；“另存为”只写新文件。手柄话题为 `/multi_point/scene_editor/update`，添加面板时自动创建 Scene editor 显示项，使用 RViz Interact 工具操作。

场景操作沿用 TaskCommand 服务，通过 `scene_load/edit/upsert/delete/pose/apply/save` 命令处理，JSON 对象放在原有 `path` 字段，服务定义保持兼容。处理期间禁用编辑；后端验证格式、运动状态和当前预览，并在应用后清除旧确认。

避障交互入口复用同一插件，运动按钮为五排：①规划无避障 / 规划有避障；②预览无避障 / 预览有避障；③执行无避障 / 执行有避障；④重播 / 停止任务；⑤返回单点（整排）。两种规划、预览、执行分别使用 `plan_baseline/avoidance`、`preview_baseline/avoidance`、`execute_baseline/avoidance` 命令。只有对应路径已预览且缓存仍有效时，才允许执行原轨迹；规划不会自动播放。普通 multi_point 保留“规划预览 / 执行（实机）”操作。

任务停止并关闭控制门后，原“停止任务”位置显示“恢复控制”，不增加按钮排数。恢复操作仍检查停稳反馈，不自动使能或恢复旧动作。

状态区保持单行；错误全文、场景文件路径与尺寸解释移入悬停提示。实测起点越界时，运动页显示关节名及角度，执行保持禁用；场景页不重复显示运动错误全文。

### 勾选拖拽编辑却没有手柄

球体/箱体的碰撞显示本身不能直接拖动。勾选“拖拽编辑”，使用 RViz 顶部 Interact 工具拖动障碍物周围的轴箭头，圆环用于旋转；松开后写入草稿，点击“应用场景”才更新碰撞物体，之后可另存为。机械臂末端的圆环属于官方单点控制，不是障碍物手柄。

旧版面板错误使用了 `Update Topic` 属性，导致手柄未连接。当前版会自动配置 `Scene editor` 的 `Interactive Markers Namespace: /multi_point/scene_editor`。更新后重启 RViz；若要修复当前窗口，在 Panels → Add New Panel → Displays 打开显示列表（已有 Displays 时直接显示），展开 Displays (time compatible) → Scene editor，填写上述命名空间，勾选启用和 Show Visual Aids。末尾不要加 `/update`。

### 选择物体与草稿操作

开启“拖拽编辑”后，只在当前选中物体上显示三轴箭头和旋转圆环。可通过左侧物体下拉框选择，也可使用 RViz 顶部 Interact 点击三维物体本体切换选择；表单同步显示所选物体的位置、形状和尺寸。其他物体仍可点击，但不显示操作手柄。单纯选择不会改动草稿、碰撞场景或已有路径。

| 操作 | 更新内容 | 是否用于碰撞检测 | 是否写入文件 |
| --- | --- | --- | --- |
| 编辑表单数值 | 仅当前表单 | 否 | 否 |
| 写入草稿 | 把表单数值写入本次会话草稿 | 否 | 否 |
| 拖动手柄后松开 | 自动把新位姿写入草稿，无须再点写入草稿 | 否 | 否 |
| 应用场景 | 将整份草稿同步至 MoveIt，关闭手柄、作废旧预览 | 是 | 否 |
| 另存为 | 保存已应用的场景为新 YAML | 不改变当前场景 | 是 |

数值编辑后需先“写入草稿”，再“应用场景”；直接点击应用不会读取尚未写入草稿的表单值。想下次继续使用，应在应用后另存为。
