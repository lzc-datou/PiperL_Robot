import json
from datetime import datetime
from pathlib import Path
import yaml

class RunLog:
    def __init__(self, root):
        self.directory = Path(root)/datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
        self.directory.mkdir(parents=True, exist_ok=False)
        self.events = []

    def write(self, name, value):
        path = self.directory/name
        text = yaml.safe_dump(value, allow_unicode=True, sort_keys=False) if path.suffix in ('.yaml','.yml') else json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(text+'\n')
        temporary.replace(path)

    def event(self, state, **details):
        event = dict(time=datetime.now().astimezone().isoformat(), state=state, **details)
        self.events.append(event)
        with (self.directory/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False)+'\n')
        print(state, details, flush=True)
