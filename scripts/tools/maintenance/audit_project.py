#!/usr/bin/env python3
"""Read-only checks for first-party Markdown and project documentation drift."""
import argparse
import html
import json
import os
from pathlib import Path
import re
from urllib.parse import unquote, urlsplit


EXCLUDED = {"external", ".deps", ".venv", "piperl_ws", "outputs", ".git"}
LINK_START = re.compile(r"!?\[[^\]\n]*\]\(")
REFERENCE = re.compile(r"^ {0,3}\[[^\]\n]+\]:\s*(.+)$")
OUTPUT_PATH = re.compile(r"(?<![\w-])outputs[/\\]")
COMMANDS = (
    ("can-inspection-command", re.compile(r"\bip\s+-(?:details|d)\s+link\s+show\b")),
    ("can-activation-command", re.compile(
        r'''(?:^|[\s`;&|"'])(?:[^\s`"']*/)?activate_can\.sh(?=[\s`;&|"']|$)''')),
    ("arm-enable-command", re.compile(
        r"\bros2\s+service\s+call\s+[^\s`]*enable_agx_arm\b")),
    ("raw-can-command", re.compile(r"\bcansend\s+[^\s`]+\s+[^\s`]+")),
)


def markdown_files(root):
    """Do not enter third-party, generated, build or environment directories."""
    readme = root / "README.md"
    if readme.is_file():
        yield readme
    for name in ("docs", "scripts", "tests"):
        directory = root / name
        if not directory.is_dir():
            continue
        for current, directories, files in os.walk(directory):
            directories[:] = sorted(d for d in directories if d not in EXCLUDED)
            for filename in sorted(files):
                if filename.lower().endswith(".md"):
                    yield Path(current) / filename


def without_code(lines):
    """Preserve line numbers while excluding fenced and inline code examples."""
    marker = None
    for line in lines:
        fence = re.match(r"^ {0,3}(`{3,}|~{3,})", line)
        if marker is not None:
            if fence and fence[1][0] == marker[0] and len(fence[1]) >= len(marker):
                marker = None
            yield ""
        elif fence:
            marker = fence[1]
            yield ""
        else:
            yield re.sub(r"(`+).*?\1", "", line)


def destination(value):
    """Accept angle destinations, spaces, CJK, escaped brackets and link titles."""
    value = value.strip()
    if value.startswith("<"):
        end = value.find(">")
        return value[1:end] if end >= 0 else ""
    value = re.sub(r'''\s+(?:"[^"\n]*"|'[^'\n]*'|\([^()\n]*\))\s*$''', "", value)
    return html.unescape(re.sub(r"\\([\\() <>])", r"\1", value.strip()))


def inline_links(line):
    for match in LINK_START.finditer(line):
        start = match.end()
        if line[start:].lstrip().startswith("<"):
            value = line[start:].lstrip()
            end = value.find(">")
            if end >= 0:
                yield destination(value[:end + 1])
            continue
        depth, end = 1, start
        while end < len(line):
            char = line[end]
            if char == "\\":
                end += 2
                continue
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
                if depth == 0:
                    yield destination(line[start:end])
                    break
            end += 1


def audit_file(path, root):
    relative = path.relative_to(root).as_posix()
    issues = []

    def report(severity, code, line, message, target=None):
        issue = dict(severity=severity, code=code, file=relative, line=line, message=message)
        if target is not None:
            issue["target"] = target
        if issue not in issues:
            issues.append(issue)

    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeError) as error:
        report("warning", "unreadable-document", 1, str(error))
        return issues
    visible = list(without_code(lines))
    headings = []
    for index, line in enumerate(visible):
        number = index + 1
        if re.match(r"^ {0,3}#\s+", line):
            headings.append(number)
        elif (re.match(r"^ {0,3}={3,}\s*$", line) and index and
              visible[index - 1].strip()):
            headings.append(number - 1)
        links = list(inline_links(line))
        reference = REFERENCE.match(line)
        if reference:
            links.append(destination(reference[1]))
        for target in links:
            if not target or target.startswith("#"):
                continue
            if (target.startswith(("/", "~/", "\\")) and not target.startswith("//") or
                    re.match(r"^[A-Za-z]:[/\\]", target) or target.startswith("file:")):
                report("warning", "absolute-local-link", number,
                       "本机绝对链接无法随项目迁移", target)
                continue
            if target.startswith("//") or re.match(r"^[A-Za-z][A-Za-z0-9+.-]*:", target):
                continue
            # Split before unquoting so an encoded # remains part of a filename.
            local = unquote(urlsplit(target).path)
            if local and not (path.parent / local).exists():
                report("error", "missing-link", number, "相对链接目标不存在", target)
    for number in headings[1:]:
        report("warning", "multiple-h1", number, "同一文档存在多个一级标题")
    for number, line in enumerate(lines, 1):
        if OUTPUT_PATH.search(line):
            report("warning", "generated-output-reference", number,
                   "引用 outputs 生成物，请确认新检出项目仍可使用")
    if not relative.startswith("docs/history/"):
        # Join shell continuations for detection, retaining the original line.
        pending, started = "", 1
        for number, line in enumerate(lines, 1):
            if not pending:
                started = number
            pending += line.rstrip().removesuffix("\\") + " "
            if line.rstrip().endswith("\\"):
                continue
            for code, pattern in COMMANDS:
                if pattern.search(pending):
                    report("warning", code, started,
                           "当前文档包含需人工确认的 CAN 或实机操作命令")
            pending = ""
    return issues


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[3],
                        help="项目根目录，默认根据脚本位置确定")
    parser.add_argument("--json", action="store_true", help="输出 JSON，便于后续整理")
    args = parser.parse_args()
    root = args.root.resolve()
    if not root.is_dir():
        parser.error(f"项目目录不存在：{root}")
    files = sorted(markdown_files(root))
    issues = [issue for path in files for issue in audit_file(path, root)]
    issues.sort(key=lambda item: (item["file"], item["line"], item["code"]))
    errors = sum(issue["severity"] == "error" for issue in issues)
    warnings = len(issues) - errors
    report = dict(root=str(root), files_scanned=len(files), errors=errors,
                  warnings=warnings, issues=issues)
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print(f"扫描 {len(files)} 份 Markdown；错误 {errors}，警告 {warnings}")
        for issue in issues:
            target = f" → {issue['target']}" if "target" in issue else ""
            print(f"{issue['severity'].upper()} {issue['file']}:{issue['line']} "
                  f"[{issue['code']}] {issue['message']}{target}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
