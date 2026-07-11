#include "lgh_st3215_driver/servo_bus.hpp"

#include "lgh_st3215_driver/protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <stdexcept>
#include <utility>

namespace lgh_st3215_driver {
namespace {

double durationUs(const SteadyClock::duration duration) {
  return std::chrono::duration<double, std::micro>(duration).count();
}

double durationMs(const SteadyClock::duration duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

std::int64_t steadyNs(const SteadyClock::time_point time_point) {
  if (time_point == SteadyClock::time_point{}) {
    return 0;
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      time_point.time_since_epoch()).count();
}

bool arraysEqual(
    const std::array<int, kNumJoints>& a,
    const std::array<int, kNumJoints>& b) {
  return std::equal(a.begin(), a.end(), b.begin());
}

}  // namespace

ServoBus::ServoBus(
    ServoBusConfig config,
    JointMap joint_map,
    CommandBuffer& command_buffer,
    StateBuffer& state_buffer,
    StatsBuffer& stats_buffer,
    TelemetryQueue& telemetry_queue)
    : config_(std::move(config)),
      joint_map_(std::move(joint_map)),
      command_buffer_(command_buffer),
      state_buffer_(state_buffer),
      stats_buffer_(stats_buffer),
      telemetry_queue_(telemetry_queue) {
  if (config_.bus_rate_hz <= 0.0 || config_.command_rate_hz <= 0.0) {
    throw std::invalid_argument("bus_rate_hz and command_rate_hz must be positive");
  }
  if (config_.read_timeout_ms <= 0 || config_.write_timeout_ms <= 0) {
    throw std::invalid_argument("UART timeouts must be positive");
  }
  if (config_.command_timeout_behavior != "hold_last" &&
      config_.command_timeout_behavior != "stop_writes") {
    throw std::invalid_argument(
        "command_timeout_behavior must be 'hold_last' or 'stop_writes'");
  }
  config_.velocity_filter_alpha = std::clamp(config_.velocity_filter_alpha, 0.0, 1.0);
  if (config_.read_order_stride <= 0) {
    config_.read_order_stride = 1;
  }
}

ServoBus::~ServoBus() { stop(); }

void ServoBus::start() {
  if (running_.load()) return;
  stop_requested_.store(false);
  worker_ = std::thread(&ServoBus::workerLoop, this);
}

void ServoBus::stop() {
  stop_requested_.store(true);
  if (worker_.joinable()) {
    worker_.join();
  }
  serial_.closePort();
  running_.store(false);
}

bool ServoBus::running() const noexcept { return running_.load(); }

int ServoBus::torqueEnabledState() const noexcept {
  return torque_enabled_state_.load();
}

bool ServoBus::requestTorqueEnabled(
    const bool enabled,
    const std::chrono::milliseconds timeout,
    std::string& message) {
  std::unique_lock<std::mutex> lock(torque_mutex_);
  torque_requested_enabled_ = enabled;
  const std::uint64_t request_sequence = ++torque_request_sequence_;
  lock.unlock();

  torque_condition_.notify_all();

  lock.lock();
  const bool completed = torque_condition_.wait_for(
      lock, timeout, [this, request_sequence] {
        return torque_applied_sequence_ >= request_sequence || !running_.load();
      });

  if (!completed) {
    message = "Timed out waiting for bus-thread torque command.";
    return false;
  }
  if (torque_applied_sequence_ < request_sequence) {
    message = "Servo bus stopped before torque command completed.";
    return false;
  }

  message = torque_last_message_;
  return torque_last_request_ok_;
}

void ServoBus::processTorqueRequest(DriverStatsSnapshot& counters) {
  bool enabled = true;
  std::uint64_t request_sequence = 0;
  {
    std::lock_guard<std::mutex> lock(torque_mutex_);
    if (torque_request_sequence_ <= torque_applied_sequence_) {
      return;
    }
    enabled = torque_requested_enabled_;
    request_sequence = torque_request_sequence_;
  }

  const auto packet = buildBroadcastTorqueEnable(enabled);
  std::string error;
  const IoStatus status = serial_.writeAll(
      packet,
      std::chrono::milliseconds(config_.write_timeout_ms),
      error);

  bool ok = status == IoStatus::kOk;
  std::string result_message;
  if (ok) {
    torque_enabled_state_.store(enabled ? 1 : 0);
    result_message = enabled
        ? "Broadcast torque enable command written successfully."
        : "Broadcast torque disable command written successfully.";
  } else {
    ++counters.io_error_count;
    counters.last_error =
        std::string("Torque broadcast write failed: ") + error;
    result_message = counters.last_error;
  }

  {
    std::lock_guard<std::mutex> lock(torque_mutex_);
    torque_applied_sequence_ = request_sequence;
    torque_last_request_ok_ = ok;
    torque_last_message_ = result_message;
  }
  torque_condition_.notify_all();
}

void ServoBus::configureWorkerScheduling() {
#ifdef __linux__
  if (config_.worker_cpu >= 0) {
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(config_.worker_cpu, &cpu_set);
    (void)::pthread_setaffinity_np(
        ::pthread_self(), sizeof(cpu_set_t), &cpu_set);
  }

  if (config_.realtime_priority > 0) {
    sched_param param{};
    param.sched_priority = config_.realtime_priority;
    (void)::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param);
  }
#endif
}

std::array<int, kNumJoints> ServoBus::commandToSteps(
    const CommandSnapshot& command) const {
  std::array<int, kNumJoints> steps{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    steps[i] = joint_map_.radiansToSteps(i, command.target_rad[i]);
  }
  return steps;
}

std::array<int, kNumJoints> ServoBus::currentRawSteps(
    const JointStateSnapshot& state) const {
  std::array<int, kNumJoints> steps{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    steps[i] = state.joints[i].raw_position_steps;
  }
  return steps;
}

std::array<double, kNumJoints> ServoBus::stepsToRadians(
    const std::array<int, kNumJoints>& steps) const {
  std::array<double, kNumJoints> radians{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    radians[i] = joint_map_.stepsToRadians(i, steps[i]);
  }
  return radians;
}

bool ServoBus::allFeedbackReady(const JointStateSnapshot& state) const {
  return std::all_of(
      state.joints.begin(), state.joints.end(),
      [](const JointSample& sample) { return sample.has_sample; });
}

SyncWriteTiming ServoBus::performSyncWrite(
    const std::array<int, kNumJoints>& target_steps,
    RollingWindow& sync_write_us,
    DriverStatsSnapshot& counters) {
  std::vector<SyncWriteTarget> targets;
  targets.reserve(kNumJoints);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto& joint = joint_map_.at(i);
    targets.push_back(SyncWriteTarget{
        joint.servo_id,
        static_cast<std::int16_t>(target_steps[i]),
        joint.speed,
        joint.acceleration});
  }

  const auto packet = buildSyncWritePositionEx(targets);
  std::string error;
  SyncWriteTiming timing;
  timing.attempted = true;
  timing.start = SteadyClock::now();
  const IoStatus status = serial_.writeAll(
      packet,
      std::chrono::milliseconds(config_.write_timeout_ms),
      error);
  timing.end = SteadyClock::now();
  timing.duration_us = durationUs(timing.end - timing.start);
  sync_write_us.add(timing.duration_us);

  if (status == IoStatus::kOk) {
    ++counters.sync_write_count;
    timing.ok = true;
    return timing;
  }

  ++counters.sync_write_error_count;
  counters.last_error = "SyncWrite failed: " + error;
  return timing;
}

void ServoBus::updateVelocity(
    const std::size_t joint_index,
    const double position_rad,
    const SteadyClock::time_point sample_time,
    JointSample& sample) {
  double measured_velocity = 0.0;

  if (have_velocity_history_[joint_index]) {
    const double dt = std::chrono::duration<double>(
        sample_time - last_sample_time_[joint_index]).count();
    if (dt > 0.0 && dt < 1.0) {
      measured_velocity =
          (position_rad - last_position_rad_[joint_index]) / dt;
    }

    const double alpha = config_.velocity_filter_alpha;
    filtered_velocity_rad_s_[joint_index] =
        (1.0 - alpha) * filtered_velocity_rad_s_[joint_index] +
        alpha * measured_velocity;
  } else {
    filtered_velocity_rad_s_[joint_index] = 0.0;
    have_velocity_history_[joint_index] = true;
  }

  if (!std::isfinite(filtered_velocity_rad_s_[joint_index]) ||
      std::abs(filtered_velocity_rad_s_[joint_index]) <
          config_.velocity_deadband_rad_s) {
    filtered_velocity_rad_s_[joint_index] = 0.0;
  }

  sample.velocity_rad_s = filtered_velocity_rad_s_[joint_index];
  last_position_rad_[joint_index] = position_rad;
  last_sample_time_[joint_index] = sample_time;
}

void ServoBus::performFeedbackSweep(
    JointStateSnapshot& state,
    RollingWindow& read_rtt_us,
    DriverStatsSnapshot& counters) {
  for (std::size_t offset = 0; offset < kNumJoints; ++offset) {
    const std::size_t index = (read_start_index_ + offset) % kNumJoints;
    const auto& joint = joint_map_.at(index);
    const auto request = buildReadPresentFeedbackRequest(joint.servo_id);

    const auto transaction_start = SteadyClock::now();
    std::string write_error;
    const IoStatus write_status = serial_.writeAll(
        request,
        std::chrono::milliseconds(config_.write_timeout_ms),
        write_error);

    if (write_status != IoStatus::kOk) {
      auto& sample = state.joints[index];
      sample.last_read_ok = false;
      sample.velocity_rad_s = 0.0;
      ++sample.read_fail_count;
      ++counters.io_error_count;
      counters.last_error =
          "Read request write failed for servo " + std::to_string(joint.servo_id) +
          ": " + write_error;
      serial_.flushInput();
      continue;
    }

    const FrameReadResult frame_result = serial_.readFrame(
        joint.servo_id,
        std::chrono::milliseconds(config_.read_timeout_ms));
    const auto transaction_end = SteadyClock::now();
    read_rtt_us.add(durationUs(transaction_end - transaction_start));

    auto& sample = state.joints[index];

    if (frame_result.status != IoStatus::kOk) {
      sample.last_read_ok = false;
      sample.velocity_rad_s = 0.0;
      ++sample.read_fail_count;

      switch (frame_result.status) {
        case IoStatus::kTimeout:
          ++counters.read_timeout_count;
          break;
        case IoStatus::kChecksumError:
          ++counters.checksum_error_count;
          break;
        case IoStatus::kWrongServoId:
          ++counters.wrong_id_count;
          break;
        case IoStatus::kMalformedFrame:
          ++counters.malformed_frame_count;
          break;
        case IoStatus::kIoError:
          ++counters.io_error_count;
          break;
        case IoStatus::kOk:
          break;
      }

      counters.last_error =
          "Feedback read failed for servo " + std::to_string(joint.servo_id) +
          ": " + frame_result.message;
      serial_.flushInput();
      continue;
    }

    PresentFeedbackReply reply;
    std::string parse_error;
    if (!parsePresentFeedbackReply(
            frame_result.frame, joint.servo_id, reply, parse_error)) {
      sample.last_read_ok = false;
      sample.velocity_rad_s = 0.0;
      ++sample.read_fail_count;
      ++counters.malformed_frame_count;
      counters.last_error =
          "Malformed feedback reply for servo " + std::to_string(joint.servo_id) +
          ": " + parse_error;
      serial_.flushInput();
      continue;
    }

    const auto sample_time = SteadyClock::now();
    const double position_rad = joint_map_.stepsToRadians(index, reply.position_steps);

    sample.position_rad = position_rad;
    sample.raw_position_steps = reply.position_steps;
    sample.raw_speed = reply.speed_raw;
    sample.raw_load = reply.load_raw;
    sample.load_ratio = static_cast<double>(reply.load_raw) * 0.001;
    sample.voltage_v = static_cast<double>(reply.voltage_raw) * 0.1;
    sample.temperature_c = static_cast<int>(reply.temperature_c);
    sample.servo_status = reply.servo_status;
    sample.moving = reply.moving;
    sample.raw_current = reply.current_raw;
    sample.current_a = static_cast<double>(reply.current_raw) * 0.0065;
    sample.status_error = reply.status_error;
    sample.sample_time = sample_time;
    sample.has_sample = true;
    sample.last_read_ok = true;
    ++sample.read_ok_count;
    ++counters.read_success_count;

    if (reply.status_error != 0U || reply.servo_status != 0U) {
      ++counters.servo_status_error_count;
    }

    updateVelocity(index, position_rad, sample_time, sample);
  }

  if (config_.rotate_read_order) {
    read_start_index_ =
        (read_start_index_ + static_cast<std::size_t>(config_.read_order_stride)) %
        kNumJoints;
  }
}

void ServoBus::publishStats(
    const DriverStatsSnapshot& counters,
    const RollingWindow& cycle_period_ms,
    const RollingWindow& cycle_work_us,
    const RollingWindow& feedback_sweep_us,
    const RollingWindow& sync_write_us,
    const RollingWindow& read_rtt_us,
    const bool feedback_ready,
    const CommandSnapshot& command,
    const bool command_stale,
    const std::string& last_error) {
  DriverStatsSnapshot snapshot = counters;
  snapshot.worker_running = running_.load();
  snapshot.uart_open = serial_.isOpen();
  snapshot.feedback_ready = feedback_ready;
  snapshot.command_seen = command.valid;
  snapshot.command_stale = command_stale;
  snapshot.writes_enabled = config_.writes_enabled;
  snapshot.last_error = last_error.empty() ? counters.last_error : last_error;

  const double period_mean_ms = cycle_period_ms.mean();
  snapshot.cycle_rate_hz = period_mean_ms > 0.0 ? 1000.0 / period_mean_ms : 0.0;
  snapshot.cycle_work_us_mean = cycle_work_us.mean();
  snapshot.cycle_work_us_max = cycle_work_us.max();
  snapshot.cycle_work_us_p99 = cycle_work_us.percentile(99.0);
  snapshot.feedback_sweep_us_mean = feedback_sweep_us.mean();
  snapshot.feedback_sweep_us_max = feedback_sweep_us.max();
  snapshot.feedback_sweep_us_p99 = feedback_sweep_us.percentile(99.0);
  snapshot.sync_write_call_us_mean = sync_write_us.mean();
  snapshot.sync_write_call_us_max = sync_write_us.max();
  snapshot.read_rtt_us_mean = read_rtt_us.mean();
  snapshot.read_rtt_us_max = read_rtt_us.max();
  snapshot.read_rtt_us_p99 = read_rtt_us.percentile(99.0);

  if (command.valid) {
    snapshot.command_age_ms = durationMs(SteadyClock::now() - command.receipt_time);
  } else {
    snapshot.command_age_ms = -1.0;
  }

  stats_buffer_.store(snapshot);
}

void ServoBus::workerLoop() {
  configureWorkerScheduling();
  running_.store(true);

  DriverStatsSnapshot counters;
  counters.worker_running = true;
  counters.writes_enabled = config_.writes_enabled;

  RollingWindow cycle_period_ms(config_.diagnostic_window_cycles);
  RollingWindow cycle_work_us(config_.diagnostic_window_cycles);
  RollingWindow feedback_sweep_us(config_.diagnostic_window_cycles);
  RollingWindow sync_write_us(config_.diagnostic_window_cycles);
  RollingWindow read_rtt_us(config_.diagnostic_window_cycles * kNumJoints);

  JointStateSnapshot state;
  std::array<int, kNumJoints> last_safe_steps{};
  std::array<int, kNumJoints> last_written_steps{};
  std::uint64_t last_written_command_sequence{0};
  bool have_last_safe_steps = false;
  bool have_last_written_steps = false;
  SteadyClock::time_point last_write_time{};
  std::string last_error;

  try {
    serial_.openPort(config_.port, config_.baud);
    counters.uart_open = true;
  } catch (const std::exception& error) {
    counters.last_error = error.what();
    stats_buffer_.store(counters);
    running_.store(false);
    return;
  }

  const auto cycle_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / config_.bus_rate_hz));
  const auto command_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / config_.command_rate_hz));

  auto next_cycle = SteadyClock::now();
  auto next_write = next_cycle;
  SteadyClock::time_point previous_cycle_start{};
  bool have_previous_cycle = false;

  while (!stop_requested_.load()) {
    const auto before_wait = SteadyClock::now();
    if (before_wait < next_cycle) {
      std::this_thread::sleep_until(next_cycle);
    } else if (counters.cycle_count > 0) {
      ++counters.deadline_miss_count;
    }

    const auto cycle_start = SteadyClock::now();

    // Torque writes are serialized through the same worker that owns the UART.
    // This prevents ROS service callbacks from racing feedback reads or SyncWrite.
    processTorqueRequest(counters);

    const std::size_t cycle_read_start_index = read_start_index_;
    if (have_previous_cycle) {
      cycle_period_ms.add(durationMs(cycle_start - previous_cycle_start));
    }
    previous_cycle_start = cycle_start;
    have_previous_cycle = true;

    const CommandSnapshot command = command_buffer_.copy();
    const bool command_stale =
        !command.valid ||
        durationMs(cycle_start - command.receipt_time) >
            static_cast<double>(config_.command_timeout_ms);

    const bool write_due = cycle_start >= next_write;
    if (write_due) {
      do {
        next_write += command_period;
      } while (next_write <= cycle_start);
    }

    TelemetrySnapshot telemetry;
    telemetry.cycle_index = counters.cycle_count + 1U;
    telemetry.cycle_start_monotonic_ns = steadyNs(cycle_start);
    telemetry.read_start_index = static_cast<std::uint32_t>(cycle_read_start_index);
    telemetry.command_valid = command.valid;
    telemetry.command_stale = command_stale;
    telemetry.command_sequence = command.sequence;
    telemetry.command_receipt_monotonic_ns = steadyNs(command.receipt_time);
    telemetry.command_age_ms = command.valid
        ? durationMs(cycle_start - command.receipt_time)
        : -1.0;
    telemetry.command_target_rad = command.target_rad;
    telemetry.write_due = write_due;

    std::array<int, kNumJoints> cycle_target_steps{};
    bool cycle_target_valid = false;

    if (write_due && config_.writes_enabled &&
        (!config_.require_full_feedback_before_writes || state.full_feedback_ready)) {
      bool have_target = false;
      std::array<int, kNumJoints> target_steps{};
      std::uint64_t target_command_sequence = last_written_command_sequence;

      if (command.valid && !command_stale) {
        target_steps = commandToSteps(command);
        last_safe_steps = target_steps;
        have_last_safe_steps = true;
        have_target = true;
        target_command_sequence = command.sequence;
      } else if (command.valid) {
        if (config_.command_timeout_behavior == "hold_last" && have_last_safe_steps) {
          target_steps = last_safe_steps;
          have_target = true;
        }
      } else if (config_.startup_hold_current_position && state.full_feedback_ready) {
        target_steps = currentRawSteps(state);
        last_safe_steps = target_steps;
        have_last_safe_steps = true;
        have_target = true;
        target_command_sequence = 0;
      } else if (config_.command_timeout_behavior == "hold_last" && have_last_safe_steps) {
        target_steps = last_safe_steps;
        have_target = true;
      }

      if (have_target) {
        cycle_target_steps = target_steps;
        cycle_target_valid = true;
        const bool changed =
            !have_last_written_steps || !arraysEqual(target_steps, last_written_steps);
        const bool keepalive_due =
            config_.write_keepalive_ms > 0 &&
            (!have_last_written_steps ||
             durationMs(cycle_start - last_write_time) >=
                 static_cast<double>(config_.write_keepalive_ms));
        const bool should_write =
            !config_.skip_unchanged_writes || changed || keepalive_due;

        if (should_write) {
          const SyncWriteTiming timing =
              performSyncWrite(target_steps, sync_write_us, counters);
          telemetry.write_attempted = timing.attempted;
          telemetry.write_ok = timing.ok;
          telemetry.sync_write_start_monotonic_ns = steadyNs(timing.start);
          telemetry.sync_write_end_monotonic_ns = steadyNs(timing.end);
          telemetry.sync_write_us = timing.duration_us;

          if (timing.ok) {
            last_written_steps = target_steps;
            have_last_written_steps = true;
            last_write_time = timing.end;
            last_written_command_sequence = target_command_sequence;
          }
        }
      }
    }

    if (!cycle_target_valid && have_last_written_steps) {
      cycle_target_steps = last_written_steps;
      cycle_target_valid = true;
    }
    telemetry.target_valid = cycle_target_valid;
    telemetry.target_steps = cycle_target_steps;
    if (cycle_target_valid) {
      telemetry.target_rad_from_steps = stepsToRadians(cycle_target_steps);
    }
    telemetry.written_command_sequence = last_written_command_sequence;

    const auto sweep_start = SteadyClock::now();
    performFeedbackSweep(state, read_rtt_us, counters);
    const auto sweep_end = SteadyClock::now();
    const double sweep_us = durationUs(sweep_end - sweep_start);
    feedback_sweep_us.add(sweep_us);

    state.full_feedback_ready = allFeedbackReady(state);
    state.sweep_complete_time = sweep_end;
    ++state.generation;

    if (state.full_feedback_ready && !have_last_safe_steps &&
        config_.startup_hold_current_position) {
      last_safe_steps = currentRawSteps(state);
      have_last_safe_steps = true;
    }

    state_buffer_.store(state);

    const auto cycle_end = SteadyClock::now();
    const double work_us = durationUs(cycle_end - cycle_start);
    cycle_work_us.add(work_us);
    ++counters.cycle_count;

    if (durationMs(cycle_end - cycle_start) > 1000.0 / config_.bus_rate_hz) {
      ++counters.cycles_over_period_count;
    }

    telemetry.cycle_end_monotonic_ns = steadyNs(cycle_end);
    telemetry.feedback_sweep_us = sweep_us;
    telemetry.cycle_work_us = work_us;
    telemetry.state = state;
    telemetry.torque_enabled_state = torque_enabled_state_.load();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (state.joints[i].has_sample) {
        telemetry.feedback_age_ms_at_cycle_end[i] =
            durationMs(cycle_end - state.joints[i].sample_time);
      } else {
        telemetry.feedback_age_ms_at_cycle_end[i] =
            std::numeric_limits<double>::infinity();
      }
    }
    if (config_.telemetry_enabled) {
      telemetry_queue_.push(telemetry);
    }

    publishStats(
        counters,
        cycle_period_ms,
        cycle_work_us,
        feedback_sweep_us,
        sync_write_us,
        read_rtt_us,
        state.full_feedback_ready,
        command,
        command_stale,
        last_error);

    next_cycle += cycle_period;
    const auto now = SteadyClock::now();
    if (now - next_cycle > cycle_period) {
      const auto periods_behind =
          static_cast<std::uint64_t>((now - next_cycle) / cycle_period) + 1U;
      next_cycle += cycle_period * periods_behind;
    }
  }

  counters.worker_running = false;
  counters.uart_open = false;
  stats_buffer_.store(counters);
  torque_condition_.notify_all();
  serial_.closePort();
  running_.store(false);
}

}  // namespace lgh_st3215_driver
