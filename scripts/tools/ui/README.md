# Qt 界面公共资源

此目录保存独立控制面板与手柄窗口共用的视觉资源，不创建 ROS 节点或发送控制命令。

| 文件 | 用途 |
| --- | --- |
| `theme.qss` | 独立窗口的字体、色彩、尺寸、控件与状态样式 |
| `theme.py` | 窗口主题、端口指示灯、点阵背景、线稿显示及操作图标 |
| `theme.qrc` | 此独立窗口主题的 Qt 资源索引；旧版离线预览使用 |
| `chevron-down.svg`、`chevron-up.svg` | 下拉与数值步进箭头，随 QRC 打包 |

独立窗口使用灰白画布、石墨色线稿区、靛蓝主操作和底部全宽停止按钮。硬件示意由 QtSvg 缩放绘制，Ubuntu 使用 `python3-pyqt5.qtsvg`。手柄图用于查看当前模式的按键映射，展开输入诊断可观察实际输入。

项目布局规范见 [DESIGN.md](../../../DESIGN.md)。两个独立窗口的离线截图保存在 `outputs/generated_images/`。RViz 样式位于[任务共用目录](../../task/common/README.md)。
