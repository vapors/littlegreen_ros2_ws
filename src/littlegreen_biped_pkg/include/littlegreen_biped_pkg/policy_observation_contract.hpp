#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

namespace littlegreen_biped
{

inline constexpr std::size_t kNumPolicyActions = 12U;
inline constexpr std::size_t kLegacyObservationCount = 45U;
inline constexpr std::size_t kSharedPhaseObservationCount = 47U;
inline constexpr double kTwoPi = 6.283185307179586476925286766559;

inline constexpr const char* kV231ObservationContractName =
    "littlegreen_velocity_47d_phase_v1";
inline constexpr const char* kV280LegacyPhaseContractName =
    "littlegreen_hardware_phase_guided_47_v1";
inline constexpr const char* kV231CompactObservationLayout =
    "command3,base_ang_vel3,projected_gravity3,joint_pos_rel12,joint_vel12,"
    "previous_bounded_action12,phase_sin1,phase_cos1";

inline bool is_supported_observation_count(std::size_t count)
{
    return count == kLegacyObservationCount || count == kSharedPhaseObservationCount;
}

inline std::string observation_contract_label(std::size_t count)
{
    if (count == kLegacyObservationCount) {
        return "legacy_hardware_45";
    }
    if (count == kSharedPhaseObservationCount) {
        return "shared_stand_walk_phase_47";
    }
    return "unsupported_" + std::to_string(count);
}

struct PhasePair
{
    double phase = 0.0;
    float sine = 0.0F;
    float cosine = 1.0F;
};

inline PhasePair phase_pair_from_fraction(double phase)
{
    if (!std::isfinite(phase)) {
        throw std::invalid_argument("phase must be finite");
    }
    phase = phase - std::floor(phase);
    const double angle = kTwoPi * phase;
    return PhasePair{
        phase,
        static_cast<float>(std::sin(angle)),
        static_cast<float>(std::cos(angle))};
}

inline std::vector<float> build_policy_observation(
    const std::vector<float>& command_velocity,
    const std::vector<float>& base_angular_velocity,
    const std::array<float, 3>& projected_gravity,
    const std::vector<float>& relative_joint_positions,
    const std::vector<float>& joint_velocities,
    const std::vector<float>& previous_bounded_actions,
    const PhasePair* phase_pair)
{
    if (command_velocity.size() != 3U || base_angular_velocity.size() != 3U ||
        relative_joint_positions.size() != kNumPolicyActions ||
        joint_velocities.size() != kNumPolicyActions ||
        previous_bounded_actions.size() != kNumPolicyActions) {
        throw std::invalid_argument("policy observation component size mismatch");
    }

    std::vector<float> observation;
    observation.reserve(phase_pair == nullptr
        ? kLegacyObservationCount
        : kSharedPhaseObservationCount);

    observation.insert(observation.end(), command_velocity.begin(), command_velocity.end());
    observation.insert(
        observation.end(), base_angular_velocity.begin(), base_angular_velocity.end());
    observation.insert(observation.end(), projected_gravity.begin(), projected_gravity.end());
    observation.insert(
        observation.end(), relative_joint_positions.begin(), relative_joint_positions.end());
    observation.insert(observation.end(), joint_velocities.begin(), joint_velocities.end());
    observation.insert(
        observation.end(), previous_bounded_actions.begin(), previous_bounded_actions.end());

    if (phase_pair != nullptr) {
        observation.push_back(phase_pair->sine);
        observation.push_back(phase_pair->cosine);
    }

    return observation;
}

}  // namespace littlegreen_biped
