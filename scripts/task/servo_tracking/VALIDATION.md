# Servo 当前验证矩阵

更新日期：2026-11-17，Asia/Shanghai。本页区分共享执行链、鼠标任务与手柄任务，旧版本完整结果保存在历史记录。

| 范围 | 当前记录 |
| --- | --- |
| ROS 构建 | `bash scripts/piperl/build.sh` 返回 0，6 个包完成 |
| 文档静态审计 | 30 份第一方 Markdown 无缺失相对链接，标题锚点检查无异常 |
| 本轮手柄与共享执行链仿真 | Domain 76、fixture、GenericSystem 通过，两轮退出无残留；详见 [手柄矩阵](../../tools/gamepad_control/VALIDATION.md) |
| 本轮两模式边界 | 向外限速至零、会话保持、反向退回与 B 锁定通过；鼠标恢复逻辑另行区分 |
| 本轮共享逻辑、界面与桥接回归 | 430 项通过，0 跳过/失败/错误；覆盖假服务时序、数学、生命周期、Qt 及 Domain 77 状态队列 |
| 鼠标 PI、目标验证、执行反馈与保持 | 既有结果见历史记录，不把旧结果视为本轮全部改动的回归 |
| 实机速度、抖动与停止响应 | 尚未完成本轮现场验收 |

本轮集成命令为 `PIPERL_ROS_DOMAIN_ID=76 bash scripts/piperl/run.sh python tests/gamepad_sim_integration.py`，返回 0。结果显式记录 `fixture_only=true`、`real_hardware=false`，没有启动 SDK/CAN 实机驱动。24 个方向包括模拟 FK 反馈核对，停止和边界验证也仅适用于该 mock 对象。

最终仿真报告为本机 `outputs/gamepad_control/two_modes_20261117_224505_d76_2006900/summary.json`；统一回归命令、模块、SHA256 与逐项日志位于 `outputs/gamepad_control/verification_20261117_225028_d77_2009680/`。这些输出不随源码克隆分发，结果范围与复现入口见 [手柄验证矩阵](../../tools/gamepad_control/VALIDATION.md)。

随附鼠标 [control.yaml](config/control.yaml) 当前选择 `pi`。算法、速度与配置优先级以当前文件和 [Servo 控制接口](../../../docs/Servo控制接口.md) 为准。历史 P、FF+P、准备位和旧速度只描述当时版本。

## 历史与操作

- [Servo 验证历史，归档于 2026-11-17](../../../docs/history/servo_tracking/2026-11-17-validation-history.md)
- [鼠标伺服操作](README.md)
- [手柄操作](../../tools/gamepad_control/README.md)
- [测试与隔离集成入口](../../../tests/README.md)

代码、mock、真实反馈读取和实机运动是不同的检查。官方 Action 成功、仿真到位、窗口截图或清理报告都不能代替实机验收。没有运行的检查不填写“通过”。
