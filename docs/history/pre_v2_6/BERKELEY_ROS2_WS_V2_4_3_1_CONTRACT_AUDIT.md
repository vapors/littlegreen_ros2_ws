# Berkeley ROS 2 Workspace v2.4.3.1 — Track 1 Contract Audit

v2.4.3.1 is a focused correction to the standing-pose capture workflow.

The runner now verifies `servo_map.yaml` against `track1_action_contract_v3.yaml`
before capture/evaluation and preserves complete rejected-pose audit reports rather
than failing on the first out-of-range joint.

The nominal Track 1 contract is:

- action contract version: 3
- policy rate: 50 Hz
- q_default: `[0, 0, -0.1, 0.4, -0.3, 0, 0, 0, -0.1, 0.4, -0.3, 0]`
- symmetric residual half-range: 0.20 rad
- nominal measured base COM height: 0.4899105727672577 m

No joint limits were changed in this revision.
