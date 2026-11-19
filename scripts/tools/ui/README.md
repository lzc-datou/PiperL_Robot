# Qt 界面公共资源

此目录保存独立控制面板与手柄窗口共用的视觉资源，不创建 ROS 节点或发送控制命令。

| 文件 | 用途 |
| --- | --- |
| `theme.qss` | 独立窗口的字体、色彩、尺寸、控件与状态样式 |
| `theme.py` | Python Qt 窗口通过 `apply_theme(widget, compact=False)` 加载样式 |
| `theme.qrc` | 此独立窗口主题的 Qt 资源索引；旧版离线预览使用 |
| `chevron-down.svg`、`chevron-up.svg` | 下拉与数值步进箭头，随 QRC 打包 |

独立窗口使用浅蓝白工作台、大尺寸按钮和全宽红色停止：正文 22 px，按钮及分区标题 26 px，页标题和停止 32 px。`density="compact"` 用于模式和选轴按钮，仍保留约 58 px 高度。Python 的 `compact` 可调整基础密度。RViz 侧栏使用 [任务共用目录](../../task/common/README.md) 的原生 Qt 字体样式，跟随官方 MotionPlanning 的背景和控件外观。

共享规则见项目根目录 [DESIGN.md](../../../DESIGN.md)。本轮独立窗口截图使用离线示例数据，保存于 `outputs/ui_redesign/desktop_workstation/`，不能作为设备反馈或运动验收证据。
