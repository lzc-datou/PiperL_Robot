"""Read baseline settings only; no ROS, SDK, or hardware initialization."""
import json
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
def load_driver_config():
    data = json.loads((ROOT/'config/baseline.json').read_text())
    channel = data.get('can_port')
    if not isinstance(channel, str) or not channel or any(c.isspace() for c in channel):
        raise ValueError('Invalid can_port in config/baseline.json')
    return data
