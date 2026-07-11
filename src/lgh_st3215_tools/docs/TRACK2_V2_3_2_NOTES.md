# Track 2 v2.3.2 identification runner fix

This patch fixes a false `driver diagnostics stale` abort that could occur immediately
at motion start.

Root cause: the guarded identification runner is single-threaded. The blocking ARM
prompt and the original countdown used `time.sleep(1.0)`, so ROS subscription callbacks
were not serviced during the countdown. With driver diagnostics published at 1 Hz and
a 2.5 s freshness guard, a 3 s countdown could make healthy diagnostics appear stale.

Changes:

- after the ARM phrase, preflight is run again before any motion command is published;
- the countdown spins ROS callbacks instead of sleeping;
- the continuous driver/feedback and ROS graph guards remain active during countdown;
- one final guard check is performed immediately before `START`.

No servo amplitudes, physical limits, feedback thresholds, command-path rules, or abort
hold behavior were changed.
