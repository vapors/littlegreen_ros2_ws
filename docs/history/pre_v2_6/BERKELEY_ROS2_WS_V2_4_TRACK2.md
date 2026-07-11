# Berkeley ROS 2 Workspace v2.4 — Track 2 Identification Telemetry

This release advances the Orange Pi 5 Max Track 2 path from guarded one-joint motion
into cycle-synchronous ST3215 identification telemetry.

## Main additions

- `bhl_st3215_driver/msg/ServoTelemetry.msg`
- `/st3215_driver/telemetry`, nominally one coherent snapshot per native 50 Hz bus cycle
- command receipt sequence and monotonic timestamp
- SyncWrite start/end timestamps and duration
- per-joint physical sample timestamps and feedback ages
- exact target radians and quantized target steps
- full ST3215 feedback block `0x38..0x46` in one transaction per servo:
  position, speed, load, voltage, temperature, status, moving flag, and direct current
- telemetry-backed identification CSV and timing-chain analysis
- telemetry queue drop guard during armed experiments

Low-rate `/st3215_driver/diagnostics` remains the health/watchdog channel; it is not
used as the high-rate experiment data stream.

## Current and load

The ST3215 load field is kept as a control-effort/duty-cycle proxy. It is not converted
to amperes. Direct current is read from the servo's separate current register and logged
alongside load. The runner reports empirical load/current correlation so a conditioned
proxy model can be evaluated later without assuming a universal conversion.

## First validation sequence

1. Build `bhl_st3215_driver` and source the workspace.
2. Start the driver with `enable_writes:=false`.
3. Verify `/st3215_driver/telemetry` is stable near 50 Hz.
4. Capture `/st3215_driver/diagnostics --once` and compare cycle/sweep/read timing to the previous baseline.
5. Confirm no telemetry drops or new read/checksum/timeouts.
6. Only then enable writes and repeat the supported left-ankle ±0.02 rad test.

See `bhl_st3215_driver/TRACK2_IDENTIFICATION_GUIDE.md` for commands and test progression.
