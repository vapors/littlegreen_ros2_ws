# Policy observability topics — v2.8

The canonical policy-debug reference is [`POLICY_DEBUG.md`](POLICY_DEBUG.md).

v2.8.0 retains all existing action and target debug topics while allowing either the legacy 45-D observation or the phase-guided 47-D observation. A 47-D policy additionally publishes `/policy_debug/gait_phase` and exposes `/policy/reset_gait_phase` in non-live modes.

All numeric debug topics use best-effort keep-last-1 QoS so a slow echo or logger cannot back-pressure the 50 Hz policy timer.
