#pragma once

#include "lgh_st3215_driver/diagnostics.hpp"
#include "lgh_st3215_driver/joint_map.hpp"
#include "lgh_st3215_driver/serial_port.hpp"
#include "lgh_st3215_driver/state_buffer.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace lgh_st3215_driver {

struct ServoBusConfig {
  std::string port{"/dev/ttyS3"};
  int baud{1000000};
  double bus_rate_hz{50.0};
  double command_rate_hz{50.0};
  int read_timeout_ms{10};
  int write_timeout_ms{5};
  int command_timeout_ms{500};
  std::string command_timeout_behavior{"hold_last"};
  bool writes_enabled{false};
  bool telemetry_enabled{true};
  bool require_full_feedback_before_writes{true};
  bool startup_hold_current_position{true};
  bool rotate_read_order{true};
  int read_order_stride{1};
  bool skip_unchanged_writes{false};
  int write_keepalive_ms{200};
  double velocity_filter_alpha{0.30};
  double velocity_deadband_rad_s{0.001};
  std::size_t diagnostic_window_cycles{500};
  int worker_cpu{-1};
  int realtime_priority{0};
};

struct SyncWriteTiming {
  SteadyClock::time_point start{};
  SteadyClock::time_point end{};
  double duration_us{0.0};
  bool attempted{false};
  bool ok{false};
};

class ServoBus {
 public:
  ServoBus(
      ServoBusConfig config,
      JointMap joint_map,
      CommandBuffer& command_buffer,
      StateBuffer& state_buffer,
      StatsBuffer& stats_buffer,
      TelemetryQueue& telemetry_queue);
  ~ServoBus();

  ServoBus(const ServoBus&) = delete;
  ServoBus& operator=(const ServoBus&) = delete;

  void start();
  void stop();
  bool running() const noexcept;

  // Queue a bus-thread-owned broadcast torque command and wait for the UART
  // write result. The serial port remains single-owner: service callbacks never
  // touch the UART directly. State is -1 unknown, 0 disabled, 1 enabled.
  bool requestTorqueEnabled(
      bool enabled,
      std::chrono::milliseconds timeout,
      std::string& message);
  int torqueEnabledState() const noexcept;

 private:
  void workerLoop();
  void configureWorkerScheduling();
  void processTorqueRequest(DriverStatsSnapshot& counters);

  SyncWriteTiming performSyncWrite(
      const std::array<int, kNumJoints>& target_steps,
      RollingWindow& sync_write_us,
      DriverStatsSnapshot& counters);

  void performFeedbackSweep(
      JointStateSnapshot& state,
      RollingWindow& read_rtt_us,
      DriverStatsSnapshot& counters);

  void updateVelocity(
      std::size_t joint_index,
      double position_rad,
      SteadyClock::time_point sample_time,
      JointSample& sample);

  bool allFeedbackReady(const JointStateSnapshot& state) const;
  std::array<int, kNumJoints> commandToSteps(const CommandSnapshot& command) const;
  std::array<int, kNumJoints> currentRawSteps(const JointStateSnapshot& state) const;
  std::array<double, kNumJoints> stepsToRadians(
      const std::array<int, kNumJoints>& steps) const;

  void publishStats(
      const DriverStatsSnapshot& counters,
      const RollingWindow& cycle_period_ms,
      const RollingWindow& cycle_work_us,
      const RollingWindow& feedback_sweep_us,
      const RollingWindow& sync_write_us,
      const RollingWindow& read_rtt_us,
      bool feedback_ready,
      const CommandSnapshot& command,
      bool command_stale,
      const std::string& last_error);

  ServoBusConfig config_;
  JointMap joint_map_;
  CommandBuffer& command_buffer_;
  StateBuffer& state_buffer_;
  StatsBuffer& stats_buffer_;
  TelemetryQueue& telemetry_queue_;

  SerialPort serial_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool running_{false};
  std::thread worker_;

  std::size_t read_start_index_{0};
  std::array<double, kNumJoints> last_position_rad_{};
  std::array<double, kNumJoints> filtered_velocity_rad_s_{};
  std::array<SteadyClock::time_point, kNumJoints> last_sample_time_{};
  std::array<bool, kNumJoints> have_velocity_history_{};

  mutable std::mutex torque_mutex_;
  std::condition_variable torque_condition_;
  bool torque_requested_enabled_{true};
  std::uint64_t torque_request_sequence_{0};
  std::uint64_t torque_applied_sequence_{0};
  bool torque_last_request_ok_{false};
  std::string torque_last_message_;
  std::atomic<int> torque_enabled_state_{-1};
};

}  // namespace lgh_st3215_driver
