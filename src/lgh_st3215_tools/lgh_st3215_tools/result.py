"""Small, deterministic YAML/text result writer used by preflight commands."""
from __future__ import annotations
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml

@dataclass
class CheckResult:
    name: str
    status: str
    message: str
    details: dict[str, Any] = field(default_factory=dict)

@dataclass
class ToolResult:
    tool: str
    mode: str
    status: str
    exit_code: int
    started_utc: str
    completed_utc: str
    checks: list[CheckResult]
    metadata: dict[str, Any] = field(default_factory=dict)

def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()

def make_report_dir(root: Path | None, prefix: str) -> Path:
    base = root.expanduser() if root else Path.home() / '.ros' / 'lgh_reports'
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    path = base / f'{stamp}_{prefix}'
    path.mkdir(parents=True, exist_ok=False)
    return path

def write_result(path: Path, result: ToolResult) -> None:
    payload = asdict(result)
    (path / 'report.yaml').write_text(yaml.safe_dump(payload, sort_keys=False, width=140))
    lines = [
        f'{result.tool.upper()}: {result.status}',
        f'mode: {result.mode}',
        f'exit_code: {result.exit_code}',
    ]
    for check in result.checks:
        lines.append(f'[{check.status}] {check.name}: {check.message}')
    (path / 'summary.txt').write_text('\n'.join(lines) + '\n')
