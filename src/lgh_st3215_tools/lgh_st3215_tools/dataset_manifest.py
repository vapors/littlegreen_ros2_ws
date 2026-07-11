"""Create a common provenance manifest for Track 2 datasets."""
from __future__ import annotations
import argparse, hashlib, os, platform, socket, subprocess, sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
import yaml
from ament_index_python.packages import get_package_share_directory
from lgh_st3215_tools.exit_codes import ExitCode


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def package_version(package_name: str) -> str:
    try:
        share = Path(get_package_share_directory(package_name))
        package_xml = share / 'package.xml'
        if package_xml.exists():
            import xml.etree.ElementTree as ET
            return ET.parse(package_xml).getroot().findtext('version') or 'unknown'
    except Exception:
        pass
    return 'unknown'


def write_manifest(
    output_dir: Path,
    experiment_type: str,
    command: list[str] | None = None,
    config_paths: dict[str, Path] | None = None,
    result_status: str = 'unknown',
    exit_code: int | None = None,
    extra: dict[str, Any] | None = None,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    configs: dict[str, Any] = {}
    for label, path in (config_paths or {}).items():
        resolved = Path(path).expanduser().resolve()
        configs[label] = {
            'path': str(resolved),
            'exists': resolved.exists(),
            'sha256': sha256_file(resolved) if resolved.is_file() else None,
        }
    payload: dict[str, Any] = {
        'schema_version': 1,
        'experiment_type': experiment_type,
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'host': {
            'hostname': socket.gethostname(),
            'platform': platform.platform(),
            'python': sys.version.split()[0],
            'ros_distro': os.environ.get('ROS_DISTRO', 'unknown'),
        },
        'packages': {
            'lgh_st3215_driver': package_version('lgh_st3215_driver'),
            'lgh_st3215_tools': package_version('lgh_st3215_tools'),
        },
        'command': command or sys.argv,
        'config': configs,
        'result': {'status': result_status, 'exit_code': exit_code},
    }
    if extra:
        payload['extra'] = extra
    target = output_dir / 'dataset_manifest.yaml'
    target.write_text(yaml.safe_dump(payload, sort_keys=False, width=140))
    return target


def main() -> int:
    parser = argparse.ArgumentParser(description='Create or refresh a LGH Track 2 dataset manifest.')
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--experiment-type', default='manual')
    parser.add_argument('--servo-map', type=Path)
    args = parser.parse_args()
    configs = {'servo_map': args.servo_map} if args.servo_map else {}
    target = write_manifest(args.output_dir.expanduser(), args.experiment_type, config_paths=configs, result_status='manual')
    print(target)
    return int(ExitCode.PASS)

if __name__ == '__main__':
    raise SystemExit(main())
