# Berkeley ROS 2 Workspace v2.4.2 — Track 1 Fresh Actuator Baseline

This workspace preserves the v2.4 cycle-synchronous ST3215 telemetry system and the
v2.4.1 fresh-local-baseline step-sweep runner, while changing the ST3215 position-mode
motion profile used by every SyncWrite.

## Baseline profile

All 12 joints ship with:

```yaml
speed: 0
acceleration: 0
```

The profile is fixed in the native driver map. The policy and outer controller do not
change these hardware profile fields in this release.

## Why

Earlier Track 2 ankle data was collected with `speed=2000` and `acceleration=100`.
v2.4.2 creates a new, explicitly identifiable baseline intended to expose the servo and
joint response without the previous programmed speed/acceleration profile clipping the
transient.

## Provenance additions

`/st3215_driver/telemetry` carries the configured speed and acceleration arrays in every
cycle snapshot. The identification CSV carries the selected joint values in every row,
and metadata records the complete diagnostics profile.

The identification runner refuses to arm unless the driver reports:

```text
motion_profile=max_envelope_fixed_0_0
```

unless the operator explicitly passes `--allow-nonmax-motion-profile` for a deliberate
profile study.

## Recommended data sequence

Use the same supported/suspended mechanical setup used for the v2.4.1 ankle work.
First perform a conservative 0.02/0.05 rad sanity sweep to empirically validate zero-value
speed/acceleration behavior on the installed hardware. Then collect the new baseline
ankle series through the policy-boundary amplitude before moving to the remaining joint
pairs.
