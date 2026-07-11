#pragma once

#include "lgh_st3215_driver/joint_map.hpp"

#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <utility>

namespace lgh_st3215_driver {

using SteadyClock = std::chrono::steady_clock;

struct CommandSnapshot {
  std::array<double, kNumJoints> target_rad{};
  SteadyClock::time_point receipt_time{};
  std::uint64_t sequence{0};
  bool valid{false};
};

class CommandBuffer {
 public:
  void store(const std::array<double, kNumJoints>& target_rad) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.target_rad = target_rad;
    snapshot_.receipt_time = SteadyClock::now();
    snapshot_.valid = true;
    ++snapshot_.sequence;
  }

  CommandSnapshot copy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  CommandSnapshot snapshot_;
};

struct JointSample {
  double position_rad{0.0};
  double velocity_rad_s{0.0};
  int raw_position_steps{0};
  int raw_speed{0};
  int raw_load{0};
  double load_ratio{0.0};
  double voltage_v{0.0};
  int temperature_c{0};
  std::uint8_t servo_status{0};
  bool moving{false};
  int raw_current{0};
  double current_a{0.0};
  std::uint8_t status_error{0};
  SteadyClock::time_point sample_time{};
  bool has_sample{false};
  bool last_read_ok{false};
  std::uint64_t read_ok_count{0};
  std::uint64_t read_fail_count{0};
};

struct JointStateSnapshot {
  std::array<JointSample, kNumJoints> joints{};
  SteadyClock::time_point sweep_complete_time{};
  std::uint64_t generation{0};
  bool full_feedback_ready{false};
};

class StateBuffer {
 public:
  void store(const JointStateSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = snapshot;
  }

  JointStateSnapshot copy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  JointStateSnapshot snapshot_;
};

struct TelemetrySnapshot {
  std::uint64_t cycle_index{0};
  std::int64_t cycle_start_monotonic_ns{0};
  std::int64_t cycle_end_monotonic_ns{0};
  std::uint32_t read_start_index{0};

  bool command_valid{false};
  bool command_stale{true};
  std::uint64_t command_sequence{0};
  std::int64_t command_receipt_monotonic_ns{0};
  double command_age_ms{-1.0};
  std::array<double, kNumJoints> command_target_rad{};

  bool target_valid{false};
  std::array<double, kNumJoints> target_rad_from_steps{};
  std::array<int, kNumJoints> target_steps{};

  bool write_due{false};
  bool write_attempted{false};
  bool write_ok{false};
  std::uint64_t written_command_sequence{0};
  std::int64_t sync_write_start_monotonic_ns{0};
  std::int64_t sync_write_end_monotonic_ns{0};
  double sync_write_us{0.0};

  double feedback_sweep_us{0.0};
  double cycle_work_us{0.0};
  std::array<double, kNumJoints> feedback_age_ms_at_cycle_end{};
  JointStateSnapshot state{};
  std::uint64_t telemetry_dropped_count{0};
  int torque_enabled_state{-1};
};

class TelemetryQueue {
 public:
  explicit TelemetryQueue(std::size_t capacity = 256) : capacity_(capacity > 0 ? capacity : 1) {}

  void push(const TelemetrySnapshot& snapshot) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (queue_.size() >= capacity_) {
        queue_.pop_front();
        ++dropped_count_;
      }
      TelemetrySnapshot copy = snapshot;
      copy.telemetry_dropped_count = dropped_count_;
      queue_.push_back(std::move(copy));
    }
    condition_.notify_one();
  }

  bool waitPop(TelemetrySnapshot& snapshot, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    condition_.wait_for(lock, timeout, [this] { return stopped_ || !queue_.empty(); });
    if (queue_.empty()) {
      return false;
    }
    snapshot = std::move(queue_.front());
    queue_.pop_front();
    return true;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    condition_.notify_all();
  }

  bool stoppedAndEmpty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopped_ && queue_.empty();
  }

 private:
  std::size_t capacity_{256};
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<TelemetrySnapshot> queue_;
  std::uint64_t dropped_count_{0};
  bool stopped_{false};
};

}  // namespace lgh_st3215_driver
