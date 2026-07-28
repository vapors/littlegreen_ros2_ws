# Policy Debug v2

v2.9.0 retains all action and target debug topics and adds task-specific phase lifecycle reporting for the shared 47-D contract. The packaged Stand policy publishes `/policy_debug/gait_phase`; its phase/sine/cosine remain static for an episode. `/policy/reset_gait_phase` begins a new episode in shadow/disabled modes and is refused in live mode.

See `POLICY_DEBUG.md` for the current array layout and command examples.
