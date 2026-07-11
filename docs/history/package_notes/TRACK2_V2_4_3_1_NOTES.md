# Track 2 v2.4.3.1 — Standing pose contract audit

This revision does not widen any hardware/training joint limits.

It adds an explicit Track 1 action-contract v3 file and requires the standing-load
runner to cross-check `servo_map.yaml` against that contract before capture or
evaluation. The contract includes the canonical 12-joint order, q_default, hardware
clip limits, ±0.20 rad residual action range, 50 Hz policy rate, and nominal measured
base-COM height 0.4899105727672577 m.

Pose capture now prints a complete 12-joint audit table and always saves YAML/CSV
capture-audit artifacts. If any captured joint is outside the guarded contract, the
pose is rejected from the executable pose library, but the full measurement is retained
for diagnosis. This prevents first-joint-only failures from discarding the information
needed to distinguish:

- a manually posed configuration that differs from q_default,
- calibration/center offset drift,
- a training/driver contract mismatch, or
- a genuinely too-narrow physical model limit.

Do not widen limits solely to make a captured pose pass. First compare the full 12-joint
capture against q_default and the Track 1 contract.
