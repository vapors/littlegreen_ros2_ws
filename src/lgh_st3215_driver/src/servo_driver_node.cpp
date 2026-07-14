#include "lgh_st3215_driver/diagnostics.hpp"
#include "lgh_st3215_driver/joint_map.hpp"
#include "lgh_st3215_driver/servo_bus.hpp"
#include "lgh_st3215_driver/state_buffer.hpp"
#include "lgh_st3215_driver/msg/servo_telemetry.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace lgh_st3215_driver {
namespace {

std::string joinAges(const std::array<std::uint32_t, kNumJoints>& ages) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < ages.size(); ++i) {
    if (i != 0) stream << ',';
    if (ages[i] == std::numeric_limits<std::uint32_t>::max()) {
      stream << "UINT32_MAX";
    } else {
      stream << ages[i];
    }
  }
  return stream.str();
}

std::string joinBoolStatus(const JointStateSnapshot& state) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << (state.joints[i].last_read_ok ? 1 : 0);
  }
  return stream.str();
}

std::string joinRawPosition(const JointStateSnapshot& state) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << state.joints[i].raw_position_steps;
  }
  return stream.str();
}

std::string joinRawSpeed(const JointStateSnapshot& state) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << state.joints[i].raw_speed;
  }
  return stream.str();
}

std::string joinConfiguredSpeed(const JointMap& joint_map) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << joint_map.at(i).speed;
  }
  return stream.str();
}

std::string joinConfiguredAcceleration(const JointMap& joint_map) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << static_cast<int>(joint_map.at(i).acceleration);
  }
  return stream.str();
}

std::string motionProfileName(const JointMap& joint_map) {
  const bool all_zero = std::all_of(
      joint_map.joints().begin(), joint_map.joints().end(),
      [](const JointConfig& joint) {
        return joint.speed == 0 && joint.acceleration == 0;
      });
  return all_zero ? "max_envelope_fixed_0_0" : "fixed_custom";
}

std::string joinReadOkCounts(const JointStateSnapshot& state) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << state.joints[i].read_ok_count;
  }
  return stream.str();
}

std::string joinReadFailCounts(const JointStateSnapshot& state) {
  std::ostringstream stream;
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (i != 0) stream << ',';
    stream << state.joints[i].read_fail_count;
  }
  return stream.str();
}

void addKey(
    diagnostic_msgs::msg::DiagnosticStatus& status,
    const std::string& key,
    const std::string& value) {
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = value;
  status.values.push_back(std::move(entry));
}

template <typename T>
std::string asString(const T& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

std::string fixedString(const double value, const int precision = 3) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

}  // namespace

class ServoDriverNode final : public rclcpp::Node {
 public:
  ServoDriverNode() : Node("lgh_st3215_driver") {
    declareParameters();

    const std::string package_share =
        ament_index_cpp::get_package_share_directory("lgh_st3215_driver");

    std::string joint_map_path = get_parameter("joint_map_path").as_string();
    if (joint_map_path.empty()) {
      joint_map_path = package_share + "/config/servo_map.yaml";
    }

    const auto default_speed = static_cast<std::uint16_t>(
        std::clamp<int64_t>(get_parameter("default_speed").as_int(), 0, 32767));
    const auto default_acceleration = static_cast<std::uint8_t>(
        std::clamp<int64_t>(get_parameter("default_acceleration").as_int(), 0, 254));

    joint_map_ = JointMap::loadFromYaml(
        joint_map_path, default_speed, default_acceleration);

    ServoBusConfig config;
    config.port = get_parameter("port").as_string();
    config.baud = static_cast<int>(get_parameter("baud").as_int());
    config.bus_rate_hz = get_parameter("bus_rate_hz").as_double();
    config.command_rate_hz = get_parameter("command_rate_hz").as_double();
    config.read_timeout_ms = static_cast<int>(get_parameter("read_timeout_ms").as_int());
    config.write_timeout_ms = static_cast<int>(get_parameter("write_timeout_ms").as_int());
    config.command_timeout_ms = static_cast<int>(get_parameter("command_timeout_ms").as_int());
    config.command_timeout_behavior = get_parameter("command_timeout_behavior").as_string();
    config.writes_enabled = get_parameter("writes_enabled").as_bool();
    config.telemetry_enabled = get_parameter("publish_telemetry").as_bool();
    config.require_full_feedback_before_writes =
        get_parameter("require_full_feedback_before_writes").as_bool();
    config.startup_hold_current_position =
        get_parameter("startup_hold_current_position").as_bool();
    config.rotate_read_order = get_parameter("rotate_read_order").as_bool();
    config.read_order_stride = static_cast<int>(get_parameter("read_order_stride").as_int());
    config.skip_unchanged_writes = get_parameter("skip_unchanged_writes").as_bool();
    config.write_keepalive_ms = static_cast<int>(get_parameter("write_keepalive_ms").as_int());
    config.velocity_filter_alpha = get_parameter("velocity_filter_alpha").as_double();
    config.velocity_deadband_rad_s = get_parameter("velocity_deadband_rad_s").as_double();
    config.diagnostic_window_cycles = static_cast<std::size_t>(
        std::max<int64_t>(1, get_parameter("diagnostic_window_cycles").as_int()));
    config.worker_cpu = static_cast<int>(get_parameter("worker_cpu").as_int());
    config.realtime_priority = static_cast<int>(get_parameter("realtime_priority").as_int());

    compact_joint_state_ = get_parameter("compact_joint_state").as_bool();
    frame_id_ = get_parameter("frame_id").as_string();
    max_feedback_warn_age_ms_ = static_cast<std::uint32_t>(
        std::max<int64_t>(0, get_parameter("max_feedback_warn_age_ms").as_int()));
    driver_profile_ = get_parameter("driver_profile").as_string();
    publish_joint_states_ = get_parameter("publish_joint_states").as_bool();
    publish_feedback_age_ = get_parameter("publish_feedback_age").as_bool();
    publish_raw_position_ = get_parameter("publish_raw_position").as_bool();
    publish_raw_speed_ = get_parameter("publish_raw_speed").as_bool();
    publish_telemetry_ = get_parameter("publish_telemetry").as_bool();
    publish_diagnostics_ = get_parameter("publish_diagnostics").as_bool();
    publish_legacy_debug_string_ = get_parameter("publish_legacy_debug_string").as_bool();
    publish_target_debug_string_ = get_parameter("publish_target_debug_string").as_bool();
    writes_enabled_ = config.writes_enabled;
    default_pose_move_duration_sec_ =
        std::max(0.1, get_parameter("default_pose_move_duration_sec").as_double());
    default_pose_ramp_rate_hz_ =
        std::max(1.0, get_parameter("default_pose_ramp_rate_hz").as_double());
    default_pose_hold_after_move_ =
        get_parameter("default_pose_hold_after_move").as_bool();

    const auto sensor_qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort();
    // Preserve the micro-ROS v6.5.8 command subscription contract: best effort,
    // keep-last 1. A RELIABLE publisher such as the current PD node can still
    // satisfy this BEST_EFFORT subscription.
    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

    servo_target_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        get_parameter("servo_target_topic").as_string(),
        command_qos,
        std::bind(&ServoDriverNode::servoTargetCallback, this, std::placeholders::_1));

    if (publish_joint_states_) {
      joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
          get_parameter("joint_state_topic").as_string(), sensor_qos);
    }
    if (publish_feedback_age_) {
      feedback_age_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>(
          get_parameter("feedback_age_topic").as_string(), sensor_qos);
    }
    if (publish_raw_position_) {
      raw_position_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
          get_parameter("raw_position_topic").as_string(), sensor_qos);
    }
    if (publish_raw_speed_) {
      raw_speed_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(
          get_parameter("raw_speed_topic").as_string(), sensor_qos);
    }
    if (publish_diagnostics_) {
      diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          get_parameter("diagnostics_topic").as_string(), rclcpp::QoS(10).reliable());
    }
    if (publish_telemetry_) {
      telemetry_pub_ = create_publisher<lgh_st3215_driver::msg::ServoTelemetry>(
          get_parameter("telemetry_topic").as_string(),
          rclcpp::QoS(rclcpp::KeepLast(20)).best_effort());
    }

    if (publish_legacy_debug_string_) {
      legacy_debug_pub_ = create_publisher<std_msgs::msg::String>(
          get_parameter("legacy_debug_topic").as_string(),
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }
    if (publish_target_debug_string_) {
      target_debug_pub_ = create_publisher<std_msgs::msg::String>(
          get_parameter("target_debug_topic").as_string(),
          rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    }

    move_default_pose_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("move_default_pose_service").as_string(),
        std::bind(
            &ServoDriverNode::moveToDefaultPoseCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    release_pose_override_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("release_pose_override_service").as_string(),
        std::bind(
            &ServoDriverNode::releasePoseOverrideCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    abort_pose_move_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("abort_pose_move_service").as_string(),
        std::bind(
            &ServoDriverNode::abortPoseMoveCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    hold_current_pose_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("hold_current_pose_service").as_string(),
        std::bind(
            &ServoDriverNode::holdCurrentPoseCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    disable_torque_all_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("disable_torque_all_service").as_string(),
        std::bind(
            &ServoDriverNode::disableTorqueAllCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
    enable_torque_hold_current_service_ = create_service<std_srvs::srv::Trigger>(
        get_parameter("enable_torque_hold_current_service").as_string(),
        std::bind(
            &ServoDriverNode::enableTorqueHoldCurrentCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    if (publish_telemetry_) {
      telemetry_thread_ = std::thread(&ServoDriverNode::telemetryPublishLoop, this);
    }

    bus_ = std::make_unique<ServoBus>(
        config, joint_map_, command_buffer_, state_buffer_, stats_buffer_, telemetry_queue_);
    bus_->start();

    const double publish_rate_hz = std::max(1.0, get_parameter("joint_state_publish_hz").as_double());
    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    if (publish_joint_states_ || publish_feedback_age_ || publish_raw_position_ || publish_raw_speed_) {
      state_timer_ = create_wall_timer(
          publish_period, std::bind(&ServoDriverNode::publishState, this));
    }

    if (publish_diagnostics_) {
      const double diagnostics_rate_hz = std::max(0.1, get_parameter("diagnostics_rate_hz").as_double());
      const auto diagnostics_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / diagnostics_rate_hz));
      diagnostics_timer_ = create_wall_timer(
          diagnostics_period, std::bind(&ServoDriverNode::publishDiagnostics, this));
    }

    RCLCPP_INFO(
        get_logger(),
        "Native ST3215 driver starting: profile=%s port=%s baud=%d joints=%zu bus=%.1fHz command=%.1fHz writes=%s map=%s",
        driver_profile_.c_str(), config.port.c_str(), config.baud, joint_map_.size(), config.bus_rate_hz,
        config.command_rate_hz, config.writes_enabled ? "ENABLED" : "DISABLED",
        joint_map_path.c_str());
    RCLCPP_INFO(
        get_logger(),
        "ROS interface: subscribe %s; publish %s and %s. Rotating read order: %s stride=%d",
        get_parameter("servo_target_topic").as_string().c_str(),
        get_parameter("joint_state_topic").as_string().c_str(),
        get_parameter("feedback_age_topic").as_string().c_str(),
        config.rotate_read_order ? "enabled" : "disabled", config.read_order_stride);

    RCLCPP_INFO(
        get_logger(),
        "ST3215 motion profile: %s; speed=[%s] acceleration=[%s]",
        motionProfileName(joint_map_).c_str(),
        joinConfiguredSpeed(joint_map_).c_str(),
        joinConfiguredAcceleration(joint_map_).c_str());

    if (!config.writes_enabled) {
      RCLCPP_WARN(
          get_logger(),
          "Servo writes are disabled. Feedback and diagnostics are live, but SyncWrite commands will not be sent.");
    }
  }

  ~ServoDriverNode() override {
    pose_stop_requested_.store(true);
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }
    if (bus_) {
      bus_->stop();
    }
    telemetry_queue_.stop();
    if (telemetry_thread_.joinable()) {
      telemetry_thread_.join();
    }
  }

 private:
  void declareParameters() {
    declare_parameter<std::string>("port", "/dev/ttyS3");
    declare_parameter<int>("baud", 1000000);
    declare_parameter<std::string>("joint_map_path", "");

    declare_parameter<double>("bus_rate_hz", 50.0);
    declare_parameter<double>("command_rate_hz", 50.0);
    declare_parameter<double>("joint_state_publish_hz", 50.0);
    declare_parameter<double>("diagnostics_rate_hz", 1.0);

    declare_parameter<int>("read_timeout_ms", 10);
    declare_parameter<int>("write_timeout_ms", 5);
    declare_parameter<int>("command_timeout_ms", 500);
    declare_parameter<std::string>("command_timeout_behavior", "hold_last");

    declare_parameter<bool>("writes_enabled", false);
    declare_parameter<bool>("require_full_feedback_before_writes", true);
    declare_parameter<bool>("startup_hold_current_position", true);
    declare_parameter<bool>("skip_unchanged_writes", false);
    declare_parameter<int>("write_keepalive_ms", 200);

    declare_parameter<bool>("rotate_read_order", true);
    declare_parameter<int>("read_order_stride", 1);

    declare_parameter<double>("velocity_filter_alpha", 0.30);
    declare_parameter<double>("velocity_deadband_rad_s", 0.001);
    declare_parameter<int>("default_speed", 0);
    declare_parameter<int>("default_acceleration", 0);

    declare_parameter<bool>("compact_joint_state", true);
    declare_parameter<std::string>("frame_id", "st3215_bus");
    declare_parameter<std::string>("driver_profile", "commissioning");
    declare_parameter<bool>("publish_joint_states", true);
    declare_parameter<bool>("publish_feedback_age", true);
    declare_parameter<bool>("publish_raw_position", true);
    declare_parameter<bool>("publish_raw_speed", true);
    declare_parameter<bool>("publish_telemetry", true);
    declare_parameter<bool>("publish_diagnostics", true);
    declare_parameter<int>("max_feedback_warn_age_ms", 250);
    declare_parameter<int>("diagnostic_window_cycles", 500);
    declare_parameter<int>("worker_cpu", -1);
    declare_parameter<int>("realtime_priority", 0);

    declare_parameter<std::string>("servo_target_topic", "/servo_target_radians");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("feedback_age_topic", "/joint_feedback_age_ms");
    // Canonical-order raw hardware state for calibration and commissioning tools.
    declare_parameter<std::string>(
        "raw_position_topic", "/st3215_driver/raw_position_steps");
    declare_parameter<std::string>(
        "raw_speed_topic", "/st3215_driver/raw_speed");
    declare_parameter<std::string>("diagnostics_topic", "/st3215_driver/diagnostics");
    declare_parameter<std::string>("telemetry_topic", "/st3215_driver/telemetry");
    declare_parameter<bool>("publish_legacy_debug_string", false);
    declare_parameter<std::string>("legacy_debug_topic", "/st3215_feedback_debug");
    declare_parameter<bool>("publish_target_debug_string", true);
    declare_parameter<std::string>("target_debug_topic", "/servo_target_steps_debug");

    // Explicit guarded training-default-pose move. The move service ramps from
    // measured feedback and, by default, holds an internal pose override until
    // the separate release service is called.
    declare_parameter<double>("default_pose_move_duration_sec", 4.0);
    declare_parameter<double>("default_pose_ramp_rate_hz", 50.0);
    declare_parameter<bool>("default_pose_hold_after_move", true);
    declare_parameter<std::string>(
        "move_default_pose_service", "/st3215_driver/move_to_default_pose");
    declare_parameter<std::string>(
        "release_pose_override_service", "/st3215_driver/release_pose_override");
    declare_parameter<std::string>(
        "abort_pose_move_service", "/st3215_driver/abort_pose_move");
    declare_parameter<std::string>(
        "hold_current_pose_service", "/st3215_driver/hold_current_pose");
    declare_parameter<std::string>(
        "disable_torque_all_service", "/st3215_driver/disable_torque_all");
    declare_parameter<std::string>(
        "enable_torque_hold_current_service",
        "/st3215_driver/enable_torque_hold_current");
  }

  void servoTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!msg || msg->data.size() < kNumJoints) {
      ++command_reject_count_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejected /servo_target_radians: expected at least 12 values, got %zu.",
          msg ? msg->data.size() : 0U);
      return;
    }

    std::array<double, kNumJoints> target{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (!std::isfinite(msg->data[i])) {
        ++command_reject_count_;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected /servo_target_radians containing non-finite value at index %zu.", i);
        return;
      }
      target[i] = msg->data[i];
    }

    ++command_rx_count_;
    if (pose_override_active_.load()) {
      ++command_ignored_pose_override_count_;
      return;
    }

    command_buffer_.store(target);
  }

  std::array<double, kNumJoints> trainingDefaultPose() const {
    std::array<double, kNumJoints> target{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      target[i] = joint_map_.at(i).training_default_rad;
    }
    return target;
  }

  void moveToDefaultPoseCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!writes_enabled_) {
      response->success = false;
      response->message =
          "Rejected: writes_enabled=false. Relaunch with enable_writes:=true after safety checks.";
      return;
    }
    if (pose_move_running_.load()) {
      response->success = false;
      response->message = "Rejected: a default-pose ramp is already running.";
      return;
    }

    const JointStateSnapshot state = state_buffer_.copy();
    if (!state.full_feedback_ready) {
      response->success = false;
      response->message = "Rejected: complete fresh servo feedback is not ready.";
      return;
    }
    const auto ages = computeFeedbackAges(state);
    const bool feedback_too_old = std::any_of(
        ages.begin(), ages.end(),
        [this](const std::uint32_t age_ms) {
          return age_ms == std::numeric_limits<std::uint32_t>::max() ||
                 age_ms > max_feedback_warn_age_ms_;
        });
    if (feedback_too_old) {
      response->success = false;
      response->message = "Rejected: one or more joint feedback samples are too old for a pose ramp.";
      return;
    }

    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }

    // Seed the bus command with the measured physical pose before enabling the
    // override thread so the first ramp packet cannot jump to an older policy target.
    std::array<double, kNumJoints> measured_hold{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      measured_hold[i] = state.joints[i].position_rad;
    }
    command_buffer_.store(measured_hold);

    pose_stop_requested_.store(false);
    pose_override_active_.store(true);
    pose_move_running_.store(true);
    pose_thread_ = std::thread(&ServoDriverNode::runDefaultPoseRamp, this, state);

    response->success = true;
    response->message =
        "Started smooth ramp to the policy-default stance. External servo targets are ignored while pose override is active.";
  }

  void releasePoseOverrideCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (pose_move_running_.load()) {
      response->success = false;
      response->message =
          "Rejected: pose ramp is still running. Wait for completion before releasing override.";
      return;
    }
    if (!pose_override_active_.load()) {
      response->success = true;
      response->message = "Pose override is already released.";
      return;
    }

    pose_override_active_.store(false);
    response->success = true;
    response->message =
        "Pose override released. Align/reset the PD controller to feedback before release so the next external target is rate-limited from the physical pose.";
    RCLCPP_WARN(
        get_logger(),
        "Policy-default stance override released; external /servo_target_radians commands are active again.");
  }

  void holdCurrentPoseCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!writes_enabled_) {
      response->success = false;
      response->message =
          "Rejected: writes_enabled=false. Current-pose hold requires active servo writes.";
      return;
    }

    // Stop any in-flight guarded ramp before latching the measured pose.
    pose_stop_requested_.store(true);
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }
    pose_move_running_.store(false);

    const JointStateSnapshot state = state_buffer_.copy();
    const auto ages = computeFeedbackAges(state);
    bool fresh_complete_feedback = state.full_feedback_ready;
    std::array<double, kNumJoints> measured_hold{};

    for (std::size_t i = 0; i < kNumJoints && fresh_complete_feedback; ++i) {
      const auto& sample = state.joints[i];
      if (!sample.has_sample || !std::isfinite(sample.position_rad) ||
          ages[i] == std::numeric_limits<std::uint32_t>::max() ||
          ages[i] > max_feedback_warn_age_ms_) {
        fresh_complete_feedback = false;
        break;
      }
      measured_hold[i] = sample.position_rad;
    }

    // Assert the internal override even when fresh feedback is unavailable. In that
    // degraded case the bus worker keeps the last safe command rather than accepting
    // a new external publisher while the operator handles the hardware.
    if (fresh_complete_feedback) {
      command_buffer_.store(measured_hold);
    }
    pose_override_active_.store(true);
    pose_stop_requested_.store(false);
    ++hold_pose_latch_count_;

    response->success = true;
    if (fresh_complete_feedback) {
      response->message =
          "Latched latest measured physical pose and asserted internal pose override. "
          "External servo targets are blocked until release_pose_override is called.";
    } else {
      response->message =
          "Fresh complete feedback was unavailable. Internal pose override was asserted "
          "and the last commanded pose remains held; external servo targets are blocked.";
    }

    RCLCPP_ERROR(
        get_logger(),
        "SOFTWARE CURRENT-POSE HOLD LATCHED: internal override active. "
        "This is not a hardware torque-off E-stop.");
  }

  void disableTorqueAllCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!writes_enabled_) {
      response->success = false;
      response->message =
          "Rejected: writes_enabled=false. Torque control requires enable_writes:=true.";
      return;
    }

    // Stop guarded motion and block all external targets before removing torque.
    pose_stop_requested_.store(true);
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }
    pose_move_running_.store(false);
    pose_override_active_.store(true);

    std::string message;
    const bool ok = bus_ && bus_->requestTorqueEnabled(
        false, std::chrono::milliseconds(1000), message);
    response->success = ok;
    response->message = ok
        ? message + " External position targets remain blocked by pose override."
        : message;

    if (ok) {
      RCLCPP_ERROR(
          get_logger(),
          "ALL-SERVO TORQUE DISABLED by explicit service request. Robot must be mechanically supported.");
    }
  }

  void enableTorqueHoldCurrentCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!writes_enabled_) {
      response->success = false;
      response->message =
          "Rejected: writes_enabled=false. Torque control requires enable_writes:=true.";
      return;
    }

    const JointStateSnapshot state = state_buffer_.copy();
    const auto ages = computeFeedbackAges(state);
    bool fresh_complete_feedback = state.full_feedback_ready;
    std::array<double, kNumJoints> measured_hold{};
    for (std::size_t i = 0; i < kNumJoints && fresh_complete_feedback; ++i) {
      const auto& sample = state.joints[i];
      if (!sample.has_sample || !std::isfinite(sample.position_rad) ||
          ages[i] == std::numeric_limits<std::uint32_t>::max() ||
          ages[i] > max_feedback_warn_age_ms_) {
        fresh_complete_feedback = false;
        break;
      }
      measured_hold[i] = sample.position_rad;
    }

    if (!fresh_complete_feedback) {
      response->success = false;
      response->message =
          "Rejected: fresh complete feedback is required before torque enable.";
      return;
    }

    // Seed measured pose and keep pose override active. Give the bus several cycles
    // to write the measured goal while torque remains disabled, then enable torque.
    pose_stop_requested_.store(true);
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }
    pose_move_running_.store(false);
    command_buffer_.store(measured_hold);
    pose_override_active_.store(true);
    pose_stop_requested_.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    std::string message;
    const bool ok = bus_ && bus_->requestTorqueEnabled(
        true, std::chrono::milliseconds(1000), message);
    response->success = ok;
    response->message = ok
        ? message + " Measured pose is held and pose override remains active until release_pose_override."
        : message;

    if (ok) {
      RCLCPP_WARN(
          get_logger(),
          "ALL-SERVO TORQUE ENABLED at measured current pose; internal pose override remains active.");
    }
  }

  void abortPoseMoveCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    const bool was_running = pose_move_running_.load();
    const bool override_active = pose_override_active_.load();

    if (!was_running && !override_active) {
      response->success = false;
      response->message =
          "No default-pose move or pose override is active. Nothing to abort.";
      return;
    }

    // Request that the ramp thread stop at its next 50 Hz boundary. Joining the
    // thread ensures it can no longer overwrite the hold target after this callback.
    pose_stop_requested_.store(true);
    if (pose_thread_.joinable()) {
      pose_thread_.join();
    }

    // Latch the best available physical feedback as the new software hold target.
    // This is intentionally a software motion abort/hold, not a torque-off E-stop.
    const JointStateSnapshot state = state_buffer_.copy();
    bool latched_feedback_hold = state.full_feedback_ready;

    if (latched_feedback_hold) {
      std::array<double, kNumJoints> measured_hold{};
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        if (!state.joints[i].has_sample ||
            !std::isfinite(state.joints[i].position_rad)) {
          latched_feedback_hold = false;
          break;
        }
        measured_hold[i] = state.joints[i].position_rad;
      }

      if (latched_feedback_hold) {
        command_buffer_.store(measured_hold);
      }
    }

    pose_move_running_.store(false);
    // Keep the internal override asserted so external policy/PD commands cannot
    // immediately take control after an abort.
    pose_override_active_.store(true);
    ++pose_abort_count_;

    response->success = true;
    if (latched_feedback_hold) {
      response->message =
          "Default-pose move aborted. Holding the latest measured physical pose; "
          "pose override remains active and external servo targets are still blocked.";
    } else {
      response->message =
          "Default-pose move aborted. Fresh complete feedback was unavailable, so "
          "the last commanded pose is being held; pose override remains active.";
    }

    RCLCPP_ERROR(
        get_logger(),
        "SOFTWARE POSE ABORT: ramp stopped; pose override remains active. "
        "This is not a hardware torque-off E-stop.");
  }

  void runDefaultPoseRamp(const JointStateSnapshot start_state) {
    std::array<double, kNumJoints> start{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      start[i] = start_state.joints[i].position_rad;
    }
    const auto goal = trainingDefaultPose();

    const std::size_t step_count = static_cast<std::size_t>(std::max(
        1.0, std::ceil(default_pose_move_duration_sec_ * default_pose_ramp_rate_hz_)));
    const auto period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / default_pose_ramp_rate_hz_));
    auto next_step = SteadyClock::now();

    RCLCPP_WARN(
        get_logger(),
        "Starting guarded move to policy-default stance over %.2f s (%zu ramp steps).",
        default_pose_move_duration_sec_, step_count);

    for (std::size_t step = 1; step <= step_count && !pose_stop_requested_.load(); ++step) {
      const double u = static_cast<double>(step) / static_cast<double>(step_count);
      const double smooth = u * u * (3.0 - 2.0 * u);  // smoothstep
      std::array<double, kNumJoints> target{};
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        target[i] = start[i] + smooth * (goal[i] - start[i]);
      }
      command_buffer_.store(target);
      next_step += period;
      std::this_thread::sleep_until(next_step);
    }

    const bool aborted = pose_stop_requested_.load();

    if (!aborted) {
      command_buffer_.store(goal);
      RCLCPP_WARN(
          get_logger(),
          "Policy-default stance reached. Pose override hold=%s.",
          default_pose_hold_after_move_ ? "true" : "false");
    } else {
      // The abort service will latch measured feedback after joining this thread.
      // Keep override active here so external commands remain blocked.
      pose_override_active_.store(true);
      RCLCPP_ERROR(
          get_logger(),
          "Policy-default stance ramp stopped before completion; waiting for abort hold latch.");
    }

    pose_move_running_.store(false);
    if (!aborted && !default_pose_hold_after_move_) {
      pose_override_active_.store(false);
    }
  }

  std::array<std::uint32_t, kNumJoints> computeFeedbackAges(
      const JointStateSnapshot& snapshot) const {
    std::array<std::uint32_t, kNumJoints> ages{};
    const auto now = SteadyClock::now();

    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto& sample = snapshot.joints[i];
      if (!sample.has_sample) {
        ages[i] = std::numeric_limits<std::uint32_t>::max();
        continue;
      }

      const double age_ms = std::chrono::duration<double, std::milli>(
          now - sample.sample_time).count();
      if (age_ms < 0.0) {
        ages[i] = 0;
      } else if (age_ms >= static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        ages[i] = std::numeric_limits<std::uint32_t>::max();
      } else {
        ages[i] = static_cast<std::uint32_t>(std::llround(age_ms));
      }
    }
    return ages;
  }

  void telemetryPublishLoop() {
    while (true) {
      TelemetrySnapshot snapshot;
      if (!telemetry_queue_.waitPop(snapshot, std::chrono::milliseconds(100))) {
        if (telemetry_queue_.stoppedAndEmpty()) {
          break;
        }
        continue;
      }
      publishTelemetry(snapshot);
    }
  }

  void publishTelemetry(const TelemetrySnapshot& snapshot) {
    if (!telemetry_pub_) {
      return;
    }
    lgh_st3215_driver::msg::ServoTelemetry msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;

    msg.cycle_index = snapshot.cycle_index;
    msg.cycle_start_monotonic_ns = snapshot.cycle_start_monotonic_ns;
    msg.cycle_end_monotonic_ns = snapshot.cycle_end_monotonic_ns;
    msg.read_start_index = snapshot.read_start_index;

    msg.command_valid = snapshot.command_valid;
    msg.command_stale = snapshot.command_stale;
    msg.command_sequence = snapshot.command_sequence;
    msg.command_receipt_monotonic_ns = snapshot.command_receipt_monotonic_ns;
    msg.command_age_ms = snapshot.command_age_ms;
    msg.target_valid = snapshot.target_valid;
    msg.write_due = snapshot.write_due;
    msg.write_attempted = snapshot.write_attempted;
    msg.write_ok = snapshot.write_ok;
    msg.written_command_sequence = snapshot.written_command_sequence;
    msg.sync_write_start_monotonic_ns = snapshot.sync_write_start_monotonic_ns;
    msg.sync_write_end_monotonic_ns = snapshot.sync_write_end_monotonic_ns;
    msg.sync_write_us = snapshot.sync_write_us;
    msg.feedback_sweep_us = snapshot.feedback_sweep_us;
    msg.cycle_work_us = snapshot.cycle_work_us;
    msg.telemetry_dropped_count = snapshot.telemetry_dropped_count;
    msg.torque_enabled_state = static_cast<std::int8_t>(snapshot.torque_enabled_state);

    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const auto& sample = snapshot.state.joints[i];
      msg.command_target_rad[i] = snapshot.command_target_rad[i];
      msg.target_rad_from_steps[i] = snapshot.target_rad_from_steps[i];
      msg.target_steps[i] = static_cast<std::int32_t>(snapshot.target_steps[i]);
      msg.configured_speed_steps_s[i] = joint_map_.at(i).speed;
      msg.configured_acceleration_units[i] = joint_map_.at(i).acceleration;
      msg.q_meas_rad[i] = sample.position_rad;
      msg.qdot_meas_rad_s[i] = sample.velocity_rad_s;
      msg.raw_position_steps[i] = static_cast<std::int32_t>(sample.raw_position_steps);
      msg.raw_speed[i] = static_cast<std::int32_t>(sample.raw_speed);
      msg.raw_load[i] = static_cast<std::int32_t>(sample.raw_load);
      msg.load_ratio[i] = sample.load_ratio;
      msg.voltage_v[i] = sample.voltage_v;
      msg.temperature_c[i] = static_cast<std::int32_t>(sample.temperature_c);
      msg.servo_status[i] = sample.servo_status;
      msg.moving[i] = sample.moving;
      msg.raw_current[i] = static_cast<std::int32_t>(sample.raw_current);
      msg.current_a[i] = sample.current_a;
      msg.sample_monotonic_ns[i] = sample.has_sample
          ? std::chrono::duration_cast<std::chrono::nanoseconds>(
                sample.sample_time.time_since_epoch()).count()
          : 0;
      msg.feedback_age_ms_at_cycle_end[i] = snapshot.feedback_age_ms_at_cycle_end[i];
      msg.read_ok[i] = sample.last_read_ok;
      msg.status_error[i] = sample.status_error;
    }

    last_telemetry_dropped_count_.store(snapshot.telemetry_dropped_count);
    telemetry_pub_->publish(msg);
  }

  void publishState() {
    const JointStateSnapshot snapshot = state_buffer_.copy();
    if (snapshot.generation == 0) {
      return;
    }

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.header.frame_id = frame_id_;

    if (!compact_joint_state_) {
      joint_state.name.reserve(kNumJoints);
      for (const auto& joint : joint_map_.joints()) {
        joint_state.name.push_back(joint.name);
      }
    }

    joint_state.position.resize(kNumJoints);
    joint_state.velocity.resize(kNumJoints);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      joint_state.position[i] = snapshot.joints[i].position_rad;
      joint_state.velocity[i] = snapshot.joints[i].velocity_rad_s;
    }
    // effort intentionally remains empty, matching compact micro-ROS v6.5.8.
    if (joint_state_pub_) {
      joint_state_pub_->publish(joint_state);
    }

    const auto ages = computeFeedbackAges(snapshot);
    std_msgs::msg::UInt32MultiArray age_message;
    age_message.data.assign(ages.begin(), ages.end());
    if (feedback_age_pub_) {
      feedback_age_pub_->publish(age_message);
    }

    std_msgs::msg::Int32MultiArray raw_position_message;
    std_msgs::msg::Int32MultiArray raw_speed_message;
    raw_position_message.data.resize(kNumJoints);
    raw_speed_message.data.resize(kNumJoints);
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      raw_position_message.data[i] = snapshot.joints[i].raw_position_steps;
      raw_speed_message.data[i] = snapshot.joints[i].raw_speed;
    }
    if (raw_position_pub_) {
      raw_position_pub_->publish(raw_position_message);
    }
    if (raw_speed_pub_) {
      raw_speed_pub_->publish(raw_speed_message);
    }
  }

  void publishDiagnostics() {
    if (!diagnostics_pub_) {
      return;
    }
    DriverStatsSnapshot stats = stats_buffer_.copy();
    stats.command_rx_count = command_rx_count_.load();
    stats.command_reject_count = command_reject_count_.load();

    const JointStateSnapshot state = state_buffer_.copy();
    const auto ages = computeFeedbackAges(state);

    std::uint32_t max_age_ms = 0;
    bool have_invalid_age = false;
    for (const auto age : ages) {
      if (age == std::numeric_limits<std::uint32_t>::max()) {
        have_invalid_age = true;
      } else {
        max_age_ms = std::max(max_age_ms, age);
      }
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "ST3215 native single bus";
    status.hardware_id = configHardwareId();

    if (!stats.worker_running || !stats.uart_open) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = stats.last_error.empty() ? "bus worker or UART is not running" : stats.last_error;
    } else if (!stats.feedback_ready || have_invalid_age) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "waiting for complete servo feedback";
    } else if (bus_ && bus_->torqueEnabledState() == 0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "bus healthy, all-servo torque disabled for manual pose capture";
    } else if (pose_override_active_.load()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = pose_move_running_.load()
          ? "bus healthy, guarded policy-default stance ramp active"
          : "bus healthy, internal pose override holding";
    } else if (stats.writes_enabled && stats.command_stale) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "command stream is stale; holding last safe target";
    } else if (max_age_ms > max_feedback_warn_age_ms_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "hardware feedback age exceeds warning threshold";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = stats.writes_enabled ? "bus healthy, feedback and writes active" : "bus healthy, feedback-only mode";
    }

    addKey(status, "cycle_rate_hz", fixedString(stats.cycle_rate_hz));
    addKey(status, "cycle_work_us_mean", fixedString(stats.cycle_work_us_mean));
    addKey(status, "cycle_work_us_p99", fixedString(stats.cycle_work_us_p99));
    addKey(status, "cycle_work_us_max", fixedString(stats.cycle_work_us_max));
    addKey(status, "feedback_sweep_us_mean", fixedString(stats.feedback_sweep_us_mean));
    addKey(status, "feedback_sweep_us_p99", fixedString(stats.feedback_sweep_us_p99));
    addKey(status, "feedback_sweep_us_max", fixedString(stats.feedback_sweep_us_max));
    addKey(status, "sync_write_call_us_mean", fixedString(stats.sync_write_call_us_mean));
    addKey(status, "sync_write_call_us_max", fixedString(stats.sync_write_call_us_max));
    addKey(status, "read_rtt_us_mean", fixedString(stats.read_rtt_us_mean));
    addKey(status, "read_rtt_us_p99", fixedString(stats.read_rtt_us_p99));
    addKey(status, "read_rtt_us_max", fixedString(stats.read_rtt_us_max));

    addKey(status, "cycle_count", asString(stats.cycle_count));
    addKey(status, "sync_write_count", asString(stats.sync_write_count));
    addKey(status, "sync_write_error_count", asString(stats.sync_write_error_count));
    addKey(status, "read_success_count", asString(stats.read_success_count));
    addKey(status, "read_timeout_count", asString(stats.read_timeout_count));
    addKey(status, "checksum_error_count", asString(stats.checksum_error_count));
    addKey(status, "malformed_frame_count", asString(stats.malformed_frame_count));
    addKey(status, "wrong_id_count", asString(stats.wrong_id_count));
    addKey(status, "io_error_count", asString(stats.io_error_count));
    addKey(status, "servo_status_error_count", asString(stats.servo_status_error_count));
    addKey(status, "deadline_miss_count", asString(stats.deadline_miss_count));
    addKey(status, "cycles_over_period_count", asString(stats.cycles_over_period_count));
    addKey(status, "command_rx_count", asString(stats.command_rx_count));
    addKey(status, "command_reject_count", asString(stats.command_reject_count));
    addKey(status, "command_age_ms", fixedString(stats.command_age_ms));
    addKey(status, "feedback_ready", stats.feedback_ready ? "true" : "false");
    addKey(status, "writes_enabled", stats.writes_enabled ? "true" : "false");
    addKey(status, "driver_profile", driver_profile_);
    addKey(status, "publish_joint_states", publish_joint_states_ ? "true" : "false");
    addKey(status, "publish_feedback_age", publish_feedback_age_ ? "true" : "false");
    addKey(status, "publish_raw_position", publish_raw_position_ ? "true" : "false");
    addKey(status, "publish_raw_speed", publish_raw_speed_ ? "true" : "false");
    addKey(status, "publish_telemetry", publish_telemetry_ ? "true" : "false");
    addKey(status, "publish_diagnostics", publish_diagnostics_ ? "true" : "false");
    addKey(status, "motion_profile", motionProfileName(joint_map_));
    addKey(status, "configured_speed_steps_s", joinConfiguredSpeed(joint_map_));
    addKey(status, "configured_acceleration_units", joinConfiguredAcceleration(joint_map_));
    addKey(status, "pose_override_active", pose_override_active_.load() ? "true" : "false");
    addKey(status, "pose_move_running", pose_move_running_.load() ? "true" : "false");
    addKey(status, "pose_abort_count", asString(pose_abort_count_.load()));
    addKey(status, "hold_pose_latch_count", asString(hold_pose_latch_count_.load()));
    addKey(status, "command_ignored_pose_override_count", asString(command_ignored_pose_override_count_.load()));
    addKey(status, "telemetry_dropped_count", asString(last_telemetry_dropped_count_.load()));
    addKey(
        status, "torque_enabled_state",
        bus_ ? asString(bus_->torqueEnabledState()) : "-1");
    addKey(status, "max_joint_age_ms", asString(max_age_ms));
    addKey(status, "per_joint_age_ms", joinAges(ages));
    addKey(status, "last_read_ok", joinBoolStatus(state));
    addKey(status, "per_joint_read_ok_count", joinReadOkCounts(state));
    addKey(status, "per_joint_read_fail_count", joinReadFailCounts(state));
    addKey(status, "raw_position_steps", joinRawPosition(state));
    addKey(status, "raw_speed", joinRawSpeed(state));
    if (!stats.last_error.empty()) {
      addKey(status, "last_error", stats.last_error);
    }

    array.status.push_back(status);
    diagnostics_pub_->publish(array);

    if (legacy_debug_pub_) {
      std_msgs::msg::String debug;
      std::ostringstream stream;
      stream << "ok=[" << joinBoolStatus(state) << "] age=[" << joinAges(ages)
             << "] pos=[" << joinRawPosition(state) << "] speed=["
             << joinRawSpeed(state) << "]";
      debug.data = stream.str();
      legacy_debug_pub_->publish(debug);
    }

    if (target_debug_pub_) {
      const CommandSnapshot command = command_buffer_.copy();
      std_msgs::msg::String debug;
      std::ostringstream stream;
      stream << "rx=" << stats.command_rx_count
             << " reject=" << stats.command_reject_count
             << " age_ms=" << fixedString(stats.command_age_ms)
             << " writes=" << (stats.writes_enabled ? 1 : 0)
             << " steps=[";
      if (command.valid) {
        for (std::size_t i = 0; i < kNumJoints; ++i) {
          if (i != 0) stream << ',';
          stream << joint_map_.radiansToSteps(i, command.target_rad[i]);
        }
        stream << "] rad=[";
        for (std::size_t i = 0; i < kNumJoints; ++i) {
          if (i != 0) stream << ',';
          stream << std::fixed << std::setprecision(3) << command.target_rad[i];
        }
      }
      stream << ']';
      debug.data = stream.str();
      target_debug_pub_->publish(debug);
    }
  }

  std::string configHardwareId() const {
    return get_parameter("port").as_string() + "@" + asString(get_parameter("baud").as_int());
  }

  JointMap joint_map_;
  CommandBuffer command_buffer_;
  StateBuffer state_buffer_;
  StatsBuffer stats_buffer_;
  TelemetryQueue telemetry_queue_{256};
  std::unique_ptr<ServoBus> bus_;
  std::thread telemetry_thread_;

  std::atomic<std::uint64_t> command_rx_count_{0};
  std::atomic<std::uint64_t> command_reject_count_{0};
  std::atomic<std::uint64_t> command_ignored_pose_override_count_{0};
  std::atomic<std::uint64_t> pose_abort_count_{0};
  std::atomic<std::uint64_t> hold_pose_latch_count_{0};
  std::atomic<std::uint64_t> last_telemetry_dropped_count_{0};

  std::atomic_bool pose_override_active_{false};
  std::atomic_bool pose_move_running_{false};
  std::atomic_bool pose_stop_requested_{false};
  std::thread pose_thread_;
  bool writes_enabled_{false};
  double default_pose_move_duration_sec_{4.0};
  double default_pose_ramp_rate_hz_{50.0};
  bool default_pose_hold_after_move_{true};

  bool compact_joint_state_{true};
  std::string driver_profile_{"commissioning"};
  bool publish_joint_states_{true};
  bool publish_feedback_age_{true};
  bool publish_raw_position_{true};
  bool publish_raw_speed_{true};
  bool publish_telemetry_{true};
  bool publish_diagnostics_{true};
  bool publish_legacy_debug_string_{false};
  bool publish_target_debug_string_{true};
  std::string frame_id_{"st3215_bus"};
  std::uint32_t max_feedback_warn_age_ms_{250};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr servo_target_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr feedback_age_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr raw_speed_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<lgh_st3215_driver::msg::ServoTelemetry>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr target_debug_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_default_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_pose_override_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_pose_move_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_current_pose_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_torque_all_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_torque_hold_current_service_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace lgh_st3215_driver

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<lgh_st3215_driver::ServoDriverNode>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("lgh_st3215_driver"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
