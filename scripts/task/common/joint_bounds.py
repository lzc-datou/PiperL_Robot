"""Structured model-bound failures; never clamp measured joint positions."""
import math


class JointBoundsError(ValueError):
    def __init__(self, violations):
        self.details = dict(kind='joint_bounds', violations=violations)
        super().__init__('；'.join(
            f'{v["joint"]}: {v["value_rad"]:.6f} rad ({math.degrees(v["value_rad"]):.3f}°) '
            f'超出模型 [{math.degrees(v["lower_rad"]):.3f}°, {math.degrees(v["upper_rad"]):.3f}°]'
            for v in violations))


class StartStateBoundsError(JointBoundsError):
    def __init__(self, error):
        super().__init__(error.details['violations'])
        self.details.update(kind='start_state_bounds', source='real_feedback')
        first = self.details['violations'][0]
        self.summary = f'起点越界 · {first["joint"]} {math.degrees(first["value_rad"]):.2f}°'
        detail = str(self)
        self.args = (f'实测起点不在模型范围内：{detail}。未生成可执行预览。'
                     '未使能时的自然下垂也可能造成越界；使能本身不保证恢复有效姿态。'
                     '请按机械臂正常操作流程恢复模型范围内的姿态，再重新规划；'
                     '若反馈与实际姿态不符，需核对零位及模型。修改场景不能解决起点越界。',)
