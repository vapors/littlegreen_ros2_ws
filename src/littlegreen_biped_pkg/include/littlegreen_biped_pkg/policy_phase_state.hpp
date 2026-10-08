#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

#include "littlegreen_biped_pkg/policy_observation_contract.hpp"

namespace littlegreen_biped
{

enum class PolicyPhaseMode : std::uint8_t
{
  disabled = 0U,
  randomized_static_per_episode = 1U,
  command_synchronized_continuous_nonblocking = 2U,
  legacy_successful_tick_clock = 3U,
  neutral_static = 4U,
};

inline std::string phase_mode_name(PolicyPhaseMode mode)
{
  switch (mode) {
    case PolicyPhaseMode::disabled:
      return "disabled";
    case PolicyPhaseMode::randomized_static_per_episode:
      return "randomized_static_per_episode";
    case PolicyPhaseMode::command_synchronized_continuous_nonblocking:
      return "command_synchronized_continuous_nonblocking";
    case PolicyPhaseMode::legacy_successful_tick_clock:
      return "legacy_successful_tick_clock";
    case PolicyPhaseMode::neutral_static:
      return "neutral_static";
  }
  return "unknown";
}

struct CommandVelocity
{
  double vx = 0.0;
  double vy = 0.0;
  double yaw_rate = 0.0;
};

struct PolicyPhaseConfig
{
  PolicyPhaseMode mode = PolicyPhaseMode::disabled;
  double policy_dt_s = 0.02;
  double period_s = 0.84;
  double linear_command_threshold = 0.20;
  double yaw_command_threshold = 0.08;
  bool initial_first_swing_left = true;
};

struct PolicyPhaseSample
{
  PhasePair pair{};
  std::uint64_t episode = 0U;
  std::uint64_t successful_tick = 0U;
  bool moving = false;
  bool first_swing_left = false;
  PolicyPhaseMode mode = PolicyPhaseMode::disabled;
};

class PolicyPhaseState
{
public:
  void configure(const PolicyPhaseConfig & config)
  {
    if (!std::isfinite(config.policy_dt_s) || config.policy_dt_s <= 0.0) {
      throw std::invalid_argument("policy phase policy_dt must be finite and positive");
    }
    if (!std::isfinite(config.period_s) || config.period_s <= 0.0) {
      throw std::invalid_argument("policy phase period must be finite and positive");
    }
    if (!std::isfinite(config.linear_command_threshold) ||
      config.linear_command_threshold < 0.0 ||
      !std::isfinite(config.yaw_command_threshold) ||
      config.yaw_command_threshold < 0.0)
    {
      throw std::invalid_argument("policy phase movement thresholds must be finite and non-negative");
    }
    config_ = config;
    configured_ = true;
    episode_active_ = false;
    episode_ = 0U;
    successful_tick_ = 0U;
  }

  void begin_episode(
    std::optional<std::uint64_t> deterministic_seed = std::nullopt,
    std::optional<double> injected_phase = std::nullopt)
  {
    require_configured();
    ++episode_;
    successful_tick_ = 0U;
    was_moving_ = false;
    moving_ = false;
    next_first_swing_left_ = config_.initial_first_swing_left;
    active_first_swing_left_ = false;

    if (config_.mode == PolicyPhaseMode::randomized_static_per_episode) {
      if (injected_phase.has_value()) {
        if (!std::isfinite(*injected_phase) || *injected_phase < 0.0 ||
          *injected_phase >= 1.0)
        {
          throw std::invalid_argument("injected Stand phase must be in [0,1)");
        }
        phase_ = *injected_phase;
      } else {
        std::mt19937_64 generator;
        if (deterministic_seed.has_value()) {
          generator.seed(*deterministic_seed);
        } else {
          std::random_device device;
          std::seed_seq seed{
            device(), device(), device(), device(),
            static_cast<unsigned int>(episode_ & 0xffffffffU)};
          generator.seed(seed);
        }
        phase_ = std::generate_canonical<double, 53>(generator);
      }
    } else {
      phase_ = 0.0;
    }
    episode_active_ = true;
  }

  void end_episode()
  {
    episode_active_ = false;
    was_moving_ = false;
    moving_ = false;
    phase_ = 0.0;
    successful_tick_ = 0U;
  }

  [[nodiscard]] bool configured() const {return configured_;}
  [[nodiscard]] bool episode_active() const {return episode_active_;}
  [[nodiscard]] std::uint64_t episode() const {return episode_;}
  [[nodiscard]] PolicyPhaseMode mode() const {return config_.mode;}

  [[nodiscard]] PolicyPhaseSample sample(const CommandVelocity & command) const
  {
    require_active();
    bool sample_moving = moving_;
    bool sample_first_swing_left = active_first_swing_left_;
    double sample_phase = phase_;
    if (config_.mode == PolicyPhaseMode::command_synchronized_continuous_nonblocking) {
      sample_moving = is_moving(command);
      if (!sample_moving) {
        sample_phase = 0.0;
        sample_first_swing_left = false;
      } else if (!was_moving_) {
        sample_first_swing_left = next_first_swing_left_;
        sample_phase = sample_first_swing_left ? 0.5 : 0.0;
      }
    }
    PolicyPhaseSample result;
    result.pair = phase_pair_from_fraction(sample_phase);
    result.episode = episode_;
    result.successful_tick = successful_tick_;
    result.moving = sample_moving;
    result.first_swing_left = sample_first_swing_left;
    result.mode = config_.mode;
    return result;
  }

  void on_successful_policy_tick(const CommandVelocity & command)
  {
    require_active();
    if (successful_tick_ != std::numeric_limits<std::uint64_t>::max()) {
      ++successful_tick_;
    }

    switch (config_.mode) {
      case PolicyPhaseMode::disabled:
      case PolicyPhaseMode::randomized_static_per_episode:
      case PolicyPhaseMode::neutral_static:
        return;
      case PolicyPhaseMode::legacy_successful_tick_clock:
        phase_ = std::fmod(phase_ + config_.policy_dt_s / config_.period_s, 1.0);
        return;
      case PolicyPhaseMode::command_synchronized_continuous_nonblocking:
        update_walk(command);
        return;
    }
  }

private:
  [[nodiscard]] bool is_moving(const CommandVelocity & command) const
  {
    const double linear_norm = std::hypot(command.vx, command.vy);
    return linear_norm > config_.linear_command_threshold ||
           std::fabs(command.yaw_rate) > config_.yaw_command_threshold;
  }

  void update_walk(const CommandVelocity & command)
  {
    moving_ = is_moving(command);
    if (!moving_) {
      phase_ = 0.0;
      was_moving_ = false;
      active_first_swing_left_ = false;
      return;
    }

    const bool onset = !was_moving_;
    if (onset) {
      active_first_swing_left_ = next_first_swing_left_;
      next_first_swing_left_ = !next_first_swing_left_;
      phase_ = active_first_swing_left_ ? 0.5 : 0.0;
    } else {
      phase_ = std::fmod(phase_ + config_.policy_dt_s / config_.period_s, 1.0);
    }
    was_moving_ = true;
  }

  void require_configured() const
  {
    if (!configured_) {
      throw std::logic_error("policy phase state is not configured");
    }
  }

  void require_active() const
  {
    require_configured();
    if (!episode_active_) {
      throw std::logic_error("policy phase episode is not active");
    }
  }

  PolicyPhaseConfig config_{};
  bool configured_ = false;
  bool episode_active_ = false;
  std::uint64_t episode_ = 0U;
  std::uint64_t successful_tick_ = 0U;
  double phase_ = 0.0;
  bool was_moving_ = false;
  bool moving_ = false;
  bool next_first_swing_left_ = true;
  bool active_first_swing_left_ = false;
};

}  // namespace littlegreen_biped
