# LittleGreen Policy Integration v1.2 — Hardware Feedback Freshness

The physical servo firmware publishes `/joint_states` as a 20 Hz cached snapshot while sampling one ST3215 at a time. Therefore the receipt time of `/joint_states` is not the same as the hardware sample time of every joint.

This update adds `/joint_feedback_age_ms` (`std_msgs/UInt32MultiArray`, 12 values) from the servo XIAO. Each value is the age in milliseconds of that joint's last successful physical bus read. `UINT32_MAX` means no valid sample has been received since boot. The topic is computed from timestamps already held in RAM and adds no ST3215 bus transactions.

The host policy gate now requires, for physical deployment:

- fresh `/imu/data`;
- fresh and complete `/joint_states`;
- a fresh `/joint_feedback_age_ms` topic;
- all 12 joints to have at least one successful physical feedback sample;
- every joint's hardware feedback age to be below `joint_feedback_max_age_sec`;
- finite observations and finite ONNX outputs.

The host does not assume a 10 ms feedback sampling period. If `FEEDBACK_SAMPLE_PERIOD_MS` is later changed to 8 ms or 5 ms, the MCU-reported ages naturally decrease and the same gate continues to work. The configured maximum age is an absolute safety limit, not a derived schedule period.
