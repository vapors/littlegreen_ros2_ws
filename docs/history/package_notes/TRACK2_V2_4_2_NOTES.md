# Track 2 v2.4.2 — Fixed Maximum-Envelope Servo Baseline

## Purpose

v2.4.2 establishes a fresh Track 1 actuator-identification baseline by removing the
previous explicit ST3215 motion-profile values (`speed=2000`, `acceleration=100`)
from the deployed test profile.

The shipped servo map now uses the ST3215/SCServo zero-value convention for every
position SyncWrite:

```text
speed = 0
acceleration = 0
```

The intent is to request the servo's maximum/unrestricted speed behavior and immediate
(no programmed acceleration ramp) response so Track 2 measurements capture the fixed
hardware/servo envelope rather than the earlier 2000/100 profile.

The supplied STS3215 memory table confirms that acceleration register `0x29` defaults
to zero and that speed and acceleration are command fields. The table does not itself
spell out the zero-value special-case semantics, so the first v2.4.2 ankle sweep should
also be treated as an empirical validation of the zero-value behavior on this exact
servo/firmware population.

## Driver changes

- `config/servo_map.yaml`: all 12 joints now explicitly use `speed: 0` and
  `acceleration: 0`.
- `config/servo_driver.yaml`: fallback defaults changed to `0/0`.
- C++ `JointConfig` defaults and ROS parameter defaults changed to `0/0`.
- `/st3215_driver/telemetry` now includes:
  - `configured_speed_steps_s[12]`
  - `configured_acceleration_units[12]`
- diagnostics now include:
  - `motion_profile`
  - `configured_speed_steps_s`
  - `configured_acceleration_units`
- the startup log prints the resolved motion profile and all per-joint values.

The driver still writes position, speed, and acceleration fields on every
`SyncWritePositionEx`; this revision changes the fixed values, not the bus protocol.

## Runner changes

The guarded identification runner now:

- writes selected-joint configured speed and acceleration values into every CSV row;
- records the diagnostics motion profile and all configured values in `metadata.yaml`;
- requires `motion_profile=max_envelope_fixed_0_0` before arming by default;
- supports `--allow-nonmax-motion-profile` only for intentional profile-sensitivity
  experiments.

No PD or policy layer dynamically modulates the ST3215 speed or acceleration registers
in v2.4.2. Direct identification still bypasses the outer controller.

## Recommended first validation

Keep the robot securely suspended with feet clear and hardware power disconnect reachable.

1. Build and source the driver package.
2. Start with writes disabled and verify 50 Hz telemetry plus zero telemetry drops.
3. Start with writes enabled but no identification runner and capture diagnostics.
4. Confirm diagnostics show:

```text
motion_profile: max_envelope_fixed_0_0
configured_speed_steps_s: 0,0,0,0,0,0,0,0,0,0,0,0
configured_acceleration_units: 0,0,0,0,0,0,0,0,0,0,0,0
```

5. Before a full 0.20 rad boundary test, run a conservative ankle sanity sweep at
   `0.02,0.05` rad to confirm the zero-value profile behaves as expected on hardware.
6. Then collect the full fresh baseline series under a single unchanged v2.4.2 profile.

## Data provenance

Existing v2.4/v2.4.1 data remains valid as Profile A:

```text
speed = 2000
acceleration = 100
```

v2.4.2 data is Profile B:

```text
speed = 0
acceleration = 0
motion_profile = max_envelope_fixed_0_0
```

Do not pool Profile A and Profile B transient/velocity data without retaining the profile
as an explanatory variable.
