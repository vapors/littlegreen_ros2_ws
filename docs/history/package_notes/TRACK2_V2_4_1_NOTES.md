# Track 2 v2.4.1 Runner Update

This revision updates only the guarded identification runner. The v2.4 native driver,
50 Hz telemetry message, and extended ST3215 feedback transaction are unchanged.

## Step-sweep behavior

For `step_sweep`, the default amplitudes remain:

```text
0.02, 0.05, 0.10 rad
```

With `--direction both`, the sequence is:

```text
+0.02
-0.02
+0.05
-0.05
+0.10
-0.10 rad
```

A temporary test center can be supplied with:

```text
--test-center-offset-rad <offset>
```

The offset is measured from the initial measured safe anchor. The runner:

1. captures the original measured safe anchor;
2. validates the nominal test center and every nominal sweep target against the guarded joint range;
3. moves slowly to the temporary center;
4. settles at the center;
5. before every trial, reissues the center command, settles, and measures a fresh local baseline;
6. commands `q_target = q_local + requested_offset`;
7. returns slowly to the temporary center after every trial;
8. after the full sweep, returns slowly to the original measured safe anchor.

Each trial target is revalidated at runtime against the measured local baseline.

## New transient metrics

The summary now distinguishes:

```text
SyncWrite end -> moving flag
SyncWrite end -> first encoder-count change
SyncWrite end -> sustained motion threshold
```

It also keeps command-relative static metrics and adds achieved-response-relative
transient metrics:

```text
achieved-response 10-90% rise time
achieved-response 63.2% time
settling within one encoder count of achieved steady response
```

Per-trial peak absolute load ratio, peak absolute current, median voltage, and maximum
temperature are also included.

## Recommended ankle-pitch sweep

With the robot suspended and feet clear:

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --test-center-offset-rad 0.05 \
  --support-condition "securely suspended, feet unloaded"
```

Repeat for `leg_right_ankle_pitch_joint` using the same support configuration.

The default step-sweep amplitude list is already `0.02,0.05,0.10`, so
`--amplitudes-rad` is optional for this protocol.
