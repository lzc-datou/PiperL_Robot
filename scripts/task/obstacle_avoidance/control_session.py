"""Both previewed paths are eligible for explicit, reviewed execution."""
from scripts.task.multi_point.scene_session import SceneSession


class ObstacleControlSession(SceneSession):
    comparison_enabled = True
