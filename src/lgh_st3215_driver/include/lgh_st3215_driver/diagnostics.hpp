#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace lgh_st3215_driver {

class RollingWindow {
 public:
  explicit RollingWindow(std::size_t capacity = 500) : capacity_(std::max<std::size_t>(1, capacity)) {}

  void add(double value) {
    values_.push_back(value);
    if (values_.size() > capacity_) {
      values_.pop_front();
    }
  }

  std::size_t size() const noexcept { return values_.size(); }

  double mean() const {
    if (values_.empty()) return 0.0;
    double sum = 0.0;
    for (const double value : values_) sum += value;
    return sum / static_cast<double>(values_.size());
  }

  double max() const {
    if (values_.empty()) return 0.0;
    return *std::max_element(values_.begin(), values_.end());
  }

  double percentile(double p) const {
    if (values_.empty()) return 0.0;
    std::vector<double> ordered(values_.begin(), values_.end());
    std::sort(ordered.begin(), ordered.end());
    const double rank = (std::clamp(p, 0.0, 100.0) / 100.0) *
                        static_cast<double>(ordered.size() - 1);
    const std::size_t lo = static_cast<std::size_t>(std::floor(rank));
    const std::size_t hi = static_cast<std::size_t>(std::ceil(rank));
    if (lo == hi) return ordered[lo];
    const double weight = rank - static_cast<double>(lo);
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight;
  }

 private:
  std::size_t capacity_;
  std::deque<double> values_;
};

struct DriverStatsSnapshot {
  bool worker_running{false};
  bool uart_open{false};
  bool feedback_ready{false};
  bool command_seen{false};
  bool command_stale{true};
  bool writes_enabled{false};
  std::string last_error;

  double cycle_rate_hz{0.0};
  double cycle_work_us_mean{0.0};
  double cycle_work_us_max{0.0};
  double cycle_work_us_p99{0.0};
  double feedback_sweep_us_mean{0.0};
  double feedback_sweep_us_max{0.0};
  double feedback_sweep_us_p99{0.0};
  double sync_write_call_us_mean{0.0};
  double sync_write_call_us_max{0.0};
  double read_rtt_us_mean{0.0};
  double read_rtt_us_max{0.0};
  double read_rtt_us_p99{0.0};

  std::uint64_t cycle_count{0};
  std::uint64_t sync_write_count{0};
  std::uint64_t sync_write_error_count{0};
  std::uint64_t read_success_count{0};
  std::uint64_t read_timeout_count{0};
  std::uint64_t checksum_error_count{0};
  std::uint64_t malformed_frame_count{0};
  std::uint64_t wrong_id_count{0};
  std::uint64_t io_error_count{0};
  std::uint64_t servo_status_error_count{0};
  std::uint64_t deadline_miss_count{0};
  std::uint64_t cycles_over_period_count{0};
  std::uint64_t command_rx_count{0};
  std::uint64_t command_reject_count{0};
  double command_age_ms{0.0};
};

class StatsBuffer {
 public:
  void store(const DriverStatsSnapshot& snapshot) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = snapshot;
  }

  DriverStatsSnapshot copy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

 private:
  mutable std::mutex mutex_;
  DriverStatsSnapshot snapshot_;
};

}  // namespace lgh_st3215_driver
