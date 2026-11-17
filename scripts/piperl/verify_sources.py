"""Verify pinned commits and tracked upstream files without importing the SDK."""
import json, subprocess
from pathlib import Path
root = Path(__file__).resolve().parents[2]
manifest = json.loads((root/'config/source_versions.json').read_text())
for repo in manifest['repositories']:
    command = ['git', '-C', str(root/repo['path'])]
    actual = subprocess.check_output(command+['rev-parse', 'HEAD'], text=True).strip()
    if actual != repo['commit']: raise SystemExit(f"Commit mismatch: {repo['name']}")
    status = subprocess.check_output(command+['status', '--porcelain', '--untracked-files=no'], text=True)
    if status.strip(): raise SystemExit(f"Modified upstream source: {repo['name']}\n{status}")
    print(f"{repo['name']}: {actual} (tracked source clean)")
