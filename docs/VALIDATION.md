# Validation Record

The documentation refresh was checked against the supplied repository.

## Passed

- `scripts/validate_source_tree.py`
- Python AST parsing for all Python source files
- XML parsing for all package manifests
- YAML parsing for all YAML files
- Bash syntax checks for all workspace scripts
- local Markdown-link validation for current operator and package documentation
- confirmation that no generated `__pycache__` or `.pyc` files remain
- confirmation that servo calibration, joint limits, driver timing/configuration, policy YAML, and `policy.onnx` were not modified

## Intentional non-runtime changes

- current documentation was rewritten and re-indexed;
- historical release and migration documents were moved under `docs/archive/`;
- old package implementation notes were moved under `docs/history/`;
- `POLICY_DEBUG.md` is now installed beside `POLICY_SHADOW.md`;
- the environment configuration script now tells the operator to source `~/.bashrc`, while direct environment sourcing remains optional;
- the workspace release marker was advanced to `2.6.5`.

A full Orange Pi build and hardware run remain the final integration checks for any installed copy.
