"""Prepare pinned official dependencies locally; never starts ROS nodes or CAN."""
import hashlib
import json
from pathlib import Path
import subprocess

TOOL = Path(__file__).resolve().parent
ROOT = TOOL.parents[2]


def main():
    manifest = json.loads((TOOL / 'config/dependencies.json').read_text())
    downloads = ROOT / '.deps/handeye_downloads'
    prefix = ROOT / '.deps/handeye_prefix'
    source = ROOT / '.deps/handeye_ws/src'
    for directory in (downloads, prefix, source):
        directory.mkdir(parents=True, exist_ok=True)
    for item in manifest['debs']:
        path = downloads / item['file']
        if not path.exists():
            subprocess.run(['apt-get', 'download', f"{item['package']}={item['version']}"], cwd=downloads, check=True)
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise SystemExit(f'Checksum mismatch: {path}')
        subprocess.run(['dpkg-deb', '-x', str(path), str(prefix)], check=True)
    # Development package refers to this exact system runtime. Do not install a second SDK.
    libdir = prefix / 'usr/lib/x86_64-linux-gnu'
    for name in ('librealsense2.so.2.59.2', 'librealsense2.so.2.59'):
        runtime = Path('/usr/lib/x86_64-linux-gnu') / name
        if not runtime.exists():
            raise SystemExit('Requires installed RealSense SDK runtime 2.59.2 (amd64).')
        link = libdir / name
        if not link.exists() and not link.is_symlink():
            link.symlink_to(runtime)
    for item in manifest['repositories']:
        repo = source / item['name']
        if not repo.exists():
            subprocess.run(['git', 'clone', '--branch', item['branch'], '--single-branch', item['url'], str(repo)], check=True)
            subprocess.run(['git', '-C', str(repo), 'checkout', '--detach', item['commit']], check=True)
        actual = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        if actual != item['commit']:
            # Preserve any existing checkout rather than resetting user changes.
            raise SystemExit(f"Version mismatch in {repo}; expected {item['commit']}, got {actual}.")
        if subprocess.check_output(['git', '-C', str(repo), 'status', '--porcelain', '--untracked-files=no'], text=True).strip():
            raise SystemExit(f'Modified dependency: {repo}')
    subprocess.run(['bash', str(ROOT / 'scripts/piperl/run.sh'), '--build', 'bash', str(TOOL / 'session.sh'), '__build'], check=True)


if __name__ == '__main__':
    main()
