# Feedback-age QoS compatibility update v1.2.1

The physical host subscription to `/joint_feedback_age_ms` now uses best-effort QoS so it is compatible with ST3215 micro-ROS firmware v6.5.5.

`/joint_states` handling is unchanged. Policy freshness logic and thresholds are unchanged.
