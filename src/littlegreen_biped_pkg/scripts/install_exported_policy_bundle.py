#!/usr/bin/env python3
"""Audit and atomically install a Track-1 deployment bundle into Track 2.

The source bundle must contain policy.onnx, policy.yaml, deployment_contract.yaml,
policy.sha256, and bundle_manifest.yaml.  No servo or policy-live state is changed;
this tool only updates the packaged policy artifacts after a successful contract audit.
"""
from __future__ import annotations

import argparse
import importlib.util
import os
import shutil
import sys
import tempfile
from pathlib import Path

PASS = 0
AUDIT_FAIL = 2
CONFIG_ERROR = 5


def load_audit_module(script_dir: Path):
    path = script_dir / 'policy_bundle_audit.py'
    spec = importlib.util.spec_from_file_location('lgh_policy_bundle_audit', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'unable to load audit module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source_defaults(script_path: Path) -> tuple[Path, Path]:
    package = script_path.resolve().parents[1]
    configs = package / 'src' / 'configs'
    checkpoints = package / 'src' / 'checkpoints'
    return configs, checkpoints


def atomic_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        prefix=destination.name + '.', suffix='.tmp', dir=destination.parent, delete=False
    ) as stream:
        temp = Path(stream.name)
    try:
        shutil.copy2(source, temp)
        os.replace(temp, destination)
    finally:
        temp.unlink(missing_ok=True)


def main() -> int:
    script_path = Path(__file__)
    default_configs, default_checkpoints = source_defaults(script_path)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-dir', type=Path, required=True)
    parser.add_argument('--destination-config-dir', type=Path, default=default_configs)
    parser.add_argument('--checkpoint-dir', type=Path, default=default_checkpoints)
    parser.add_argument('--joint-map', type=Path, default=None)
    parser.add_argument('--onnx-shape-probe', type=Path, default=None)
    parser.add_argument('--skip-onnx-shape-check', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()

    try:
        bundle = args.bundle_dir.expanduser().resolve()
        configs = args.destination_config_dir.expanduser().resolve()
        checkpoints = args.checkpoint_dir.expanduser().resolve()
        required = {
            'policy.onnx': bundle / 'policy.onnx',
            'policy.yaml': bundle / 'policy.yaml',
            'deployment_contract.yaml': bundle / 'deployment_contract.yaml',
            'policy.sha256': bundle / 'policy.sha256',
            'bundle_manifest.yaml': bundle / 'bundle_manifest.yaml',
        }
        missing = [name for name, path in required.items() if not path.is_file()]
        if missing:
            raise ValueError(f'bundle is missing required files: {missing}')

        joint_map = (
            args.joint_map.expanduser().resolve()
            if args.joint_map else configs / 'joint_map.yaml'
        )
        if not joint_map.is_file():
            raise ValueError(f'joint map is missing: {joint_map}')

        audit = load_audit_module(script_path.resolve().parent)
        probe = audit.resolve_probe(
            args.onnx_shape_probe.expanduser().resolve() if args.onnx_shape_probe else None
        )
        errors, warnings, _ = audit.audit(
            required['policy.yaml'],
            joint_map,
            required['policy.onnx'],
            probe,
            args.skip_onnx_shape_check,
            required['deployment_contract.yaml'],
            required['policy.sha256'],
            required['bundle_manifest.yaml'],
        )
        for warning in warnings:
            print(f'WARN  {warning}')
        if errors:
            print('POLICY BUNDLE INSTALL: AUDIT FAIL')
            for error in errors:
                print(f'FAIL  {error}')
            return AUDIT_FAIL

        destinations = {
            configs / 'policy.onnx': required['policy.onnx'],
            configs / 'policy_latest.yaml': required['policy.yaml'],
            configs / 'policy.yaml': required['policy.yaml'],
            configs / 'deployment_contract.yaml': required['deployment_contract.yaml'],
            configs / 'policy.sha256': required['policy.sha256'],
            configs / 'bundle_manifest.yaml': required['bundle_manifest.yaml'],
            checkpoints / 'policy.onnx': required['policy.onnx'],
        }
        print('POLICY BUNDLE INSTALL: AUDIT PASS')
        for destination, source in destinations.items():
            print(f'{source.name} -> {destination}')
        if args.dry_run:
            print('DRY RUN: no files changed')
            return PASS
        for destination, source in destinations.items():
            atomic_copy(source, destination)
        print('POLICY BUNDLE INSTALL: COMPLETE')
        print('Rebuild/source the workspace before running shadow mode.')
        return PASS
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'POLICY BUNDLE INSTALL: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR


if __name__ == '__main__':
    raise SystemExit(main())
