# Berkeley ROS 2 Workspace v2.4.3 — Standing Load Characterization

This release extends the v2.4.2 Track 1 identification baseline with a separate Track 2 whole-body loaded-pose workflow.

## Preserved baseline

- native Orange Pi ST3215 driver;
- 50 Hz bus and command cadence;
- cycle-synchronous `/st3215_driver/telemetry`;
- fixed `max_envelope_fixed_0_0` motion profile;
- unchanged single-joint identification runner.

## New tooling

`standing_load_characterization_runner.py` supports:

1. torque-off manual pose capture at operator-measured base-COM heights;
2. a reusable YAML pose library;
3. guarded standing/crouch evaluation;
4. independent crouch and stand-return position-reference speed limits;
5. transition and static-load summaries for Track 1.

See `bhl_st3215_driver/TRACK2_V2_4_3_NOTES.md` for details and commands.
