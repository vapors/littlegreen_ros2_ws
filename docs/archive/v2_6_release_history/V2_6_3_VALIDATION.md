# v2.6.3 validation record

The v2.6.3 shell hotfix was checked with:

- `bash -n` for all active shell scripts;
- the ROS-independent source validator;
- a synthetic ROS setup file that references an unset variable while the calling script has `set -u` enabled;
- verification that nounset is restored after the synthetic setup file is sourced;
- generation and sourcing of `~/.config/littlegreen/ros2_env.sh` both with and without nounset enabled;
- archive integrity and executable-permission checks.

The Orange Pi ROS 2 build and hardware commissioning remain the authoritative integration tests.
