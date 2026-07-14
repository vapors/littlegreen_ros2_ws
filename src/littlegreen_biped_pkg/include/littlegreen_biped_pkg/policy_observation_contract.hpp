#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace littlegreen_biped
{

inline constexpr std::size_t kNumPolicyActions = 12U;
inline constexpr std::size_t kLegacyObservationCount = 45U;
inline constexpr std::size_t kPhaseGuidedObservationCount = 47U;
inline constexpr double kTwoPi = 6.283185307179586476925286766559;

inline bool is_supported_observation_count(std::size_t count)
{
    return count == kLegacyObservationCount || count == kPhaseGuidedObservationCount;
}

inline std::string observation_contract_label(std::size_t count)
{
    if (count == kLegacyObservationCount) {
        return "legacy_hardware_45";
    }
    if (count == kPhaseGuidedObservationCount) {
        return "phase_guided_hardware_47";
    }
    return "unsupported_" + std::to_string(count);
}

struct GaitPhaseSample
{
    std::uint64_t tick = 0U;
    std::size_t period_ticks = 0U;
    float phase = 0.0F;
    float sine = 0.0F;
    float cosine = 1.0F;
    std::uint8_t expected_half_cycle = 0U;
};

class GaitPhaseClock
{
public:
    void configure(double period_s, double policy_dt_s)
    {
        if (!std::isfinite(period_s) || !std::isfinite(policy_dt_s) ||
            period_s <= 0.0 || policy_dt_s <= 0.0) {
            throw std::invalid_argument("gait phase period and policy_dt must be finite and positive");
        }

        const double exact_ticks = period_s / policy_dt_s;
        const auto rounded_ticks = static_cast<std::size_t>(std::llround(exact_ticks));
        if (rounded_ticks < 2U) {
            throw std::invalid_argument("gait phase period must contain at least two policy ticks");
        }

        const double reconstructed_period = static_cast<double>(rounded_ticks) * policy_dt_s;
        const double tolerance = std::max(1.0e-9, std::fabs(period_s) * 1.0e-6);
        if (std::fabs(reconstructed_period - period_s) > tolerance) {
            throw std::invalid_argument(
                "gait phase period must be an integer multiple of policy_dt for deterministic deployment");
        }

        period_s_ = period_s;
        policy_dt_s_ = policy_dt_s;
        period_ticks_ = rounded_ticks;
        reset();
    }

    void reset()
    {
        tick_ = 0U;
    }

    [[nodiscard]] bool configured() const
    {
        return period_ticks_ > 0U;
    }

    [[nodiscard]] std::uint64_t tick() const
    {
        return tick_;
    }

    [[nodiscard]] std::size_t period_ticks() const
    {
        return period_ticks_;
    }

    [[nodiscard]] double period_s() const
    {
        return period_s_;
    }

    [[nodiscard]] double policy_dt_s() const
    {
        return policy_dt_s_;
    }

    [[nodiscard]] GaitPhaseSample sample() const
    {
        if (!configured()) {
            throw std::logic_error("gait phase clock is not configured");
        }

        const std::size_t wrapped_tick = static_cast<std::size_t>(tick_ % period_ticks_);
        const double phase = static_cast<double>(wrapped_tick) /
            static_cast<double>(period_ticks_);
        const double angle = kTwoPi * phase;

        GaitPhaseSample result;
        result.tick = tick_;
        result.period_ticks = period_ticks_;
        result.phase = static_cast<float>(phase);
        result.sine = static_cast<float>(std::sin(angle));
        result.cosine = static_cast<float>(std::cos(angle));
        result.expected_half_cycle = phase < 0.5 ? 0U : 1U;
        return result;
    }

    void advance()
    {
        if (!configured()) {
            throw std::logic_error("gait phase clock is not configured");
        }
        if (tick_ == std::numeric_limits<std::uint64_t>::max()) {
            tick_ %= static_cast<std::uint64_t>(period_ticks_);
        }
        ++tick_;
    }

private:
    double period_s_ = 0.0;
    double policy_dt_s_ = 0.0;
    std::size_t period_ticks_ = 0U;
    std::uint64_t tick_ = 0U;
};

inline std::vector<float> build_policy_observation(
    const std::vector<float>& command_velocity,
    const std::vector<float>& base_angular_velocity,
    const std::array<float, 3>& projected_gravity,
    const std::vector<float>& relative_joint_positions,
    const std::vector<float>& joint_velocities,
    const std::vector<float>& previous_bounded_actions,
    const GaitPhaseSample* gait_phase)
{
    if (command_velocity.size() != 3U || base_angular_velocity.size() != 3U ||
        relative_joint_positions.size() != kNumPolicyActions ||
        joint_velocities.size() != kNumPolicyActions ||
        previous_bounded_actions.size() != kNumPolicyActions) {
        throw std::invalid_argument("policy observation component size mismatch");
    }

    std::vector<float> observation;
    observation.reserve(gait_phase == nullptr
        ? kLegacyObservationCount
        : kPhaseGuidedObservationCount);

    observation.insert(observation.end(), command_velocity.begin(), command_velocity.end());
    observation.insert(
        observation.end(), base_angular_velocity.begin(), base_angular_velocity.end());
    observation.insert(observation.end(), projected_gravity.begin(), projected_gravity.end());
    observation.insert(
        observation.end(), relative_joint_positions.begin(), relative_joint_positions.end());
    observation.insert(observation.end(), joint_velocities.begin(), joint_velocities.end());
    observation.insert(
        observation.end(), previous_bounded_actions.begin(), previous_bounded_actions.end());

    if (gait_phase != nullptr) {
        observation.push_back(gait_phase->sine);
        observation.push_back(gait_phase->cosine);
    }

    return observation;
}

}  // namespace littlegreen_biped
