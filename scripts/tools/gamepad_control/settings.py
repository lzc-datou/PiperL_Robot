"""Load the single gamepad configuration used by the launcher, backend and UI."""
from pathlib import Path
import yaml
from .core import validate_tool_config


TOOL_CONFIG_PATH = Path(__file__).parent/'config/config.yaml'
INPUT_PROFILE_PATH = Path(__file__).parent/'config/flydigi_direwolf2.yaml'


def load_tool_config(path=TOOL_CONFIG_PATH):
    with Path(path).open(encoding='utf-8') as stream:
        return validate_tool_config(yaml.safe_load(stream))


def load_input_profile():
    with INPUT_PROFILE_PATH.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)
