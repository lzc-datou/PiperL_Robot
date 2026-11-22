"""Preview input validation, available without ROS imports."""
from scripts.task.common.schema import load_config, load_scene


def load_inputs(config_path, scene_path):
    config = load_config(config_path)
    if len(config['points']) < 2:
        raise ValueError('避障演示需要至少两个点，按列表顺序规划')
    if config['settings']['return_to_initial'] or any(config['settings']['approach_offset_m']):
        raise ValueError('避障演示不使用自动返回或接近偏移')
    scene = load_scene(scene_path, config['settings']['frame_id'])
    if not scene['enabled'] or not scene['objects']:
        raise ValueError('scene.yaml 必须启用且至少包含一个障碍物')
    return config, scene
