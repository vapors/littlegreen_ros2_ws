# Track 2 v2.4 Notes

## Purpose

v2.4 establishes a cycle-synchronous telemetry foundation for one-joint ST3215
identification and later outer-PD tuning.

## Native-driver changes

- Added `lgh_st3215_driver/msg/ServoTelemetry.msg`.
- Added `/st3215_driver/telemetry` with one coherent snapshot per completed bus cycle.
- Added a bounded producer/consumer telemetry queue so DDS publication does not run
  in the UART worker hot path.
- Added exact steady-clock timestamps for:
  - command receipt;
  - bus cycle start/end;
  - SyncWrite start/end;
  - every joint feedback sample.
- Expanded each servo read from `0x38 length 4` to `0x38 length 15`, covering through
  `0x46` in one request/reply transaction.
- Decodes and publishes position, speed, load, voltage, temperature, status/moving,
  and current.
- Current scaling: `current_a = raw_current * 0.0065`.
- Load scaling: `load_ratio = signed_raw_load * 0.001`.
- Load is not converted into current.

## Runner changes

- Telemetry is the authoritative CSV source.
- Existing `/joint_states`, feedback age, diagnostics, and graph ownership checks remain
  safety inputs.
- Added timing-chain decomposition for step tests.
- Added direct current/load/voltage/temperature columns.
- Added run-level current/load statistics and empirical absolute-load/current correlation.
- Armed tests abort if the telemetry queue drop counter increases during the test.

## Required first validation

Before physical motion, run feedback-only and compare v2.4 timing metrics against the
previous 4-byte feedback baseline. The 15-byte read adds reply serialization bytes but
avoids extra per-servo current transactions.
