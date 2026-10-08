#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "littlegreen_biped_pkg/policy_observation_contract.hpp"
#include "littlegreen_biped_pkg/policy_phase_state.hpp"

namespace
{
using littlegreen_biped::CommandVelocity;
using littlegreen_biped::PolicyPhaseConfig;
using littlegreen_biped::PolicyPhaseMode;
using littlegreen_biped::PolicyPhaseState;
using littlegreen_biped::build_policy_observation;
using littlegreen_biped::phase_pair_from_fraction;

std::vector<float> build_at_phase(double phase)
{
  const auto pair = phase_pair_from_fraction(phase);
  return build_policy_observation(
    std::vector<float>{1.0F, 2.0F, 3.0F},
    std::vector<float>{4.0F, 5.0F, 6.0F},
    std::array<float, 3>{7.0F, 8.0F, 9.0F},
    std::vector<float>(12U, 0.1F),
    std::vector<float>(12U, 0.2F),
    std::vector<float>(12U, 0.3F),
    &pair);
}

TEST(ObservationBuilder, PreservesExactShared47DOrdering)
{
  const std::vector<float> command{1.0F, 2.0F, 3.0F};
  const std::vector<float> angular{4.0F, 5.0F, 6.0F};
  const std::array<float, 3> gravity{7.0F, 8.0F, 9.0F};
  std::vector<float> relative(12U);
  std::vector<float> velocity(12U);
  std::vector<float> previous(12U);
  for (std::size_t i = 0; i < 12U; ++i) {
    relative[i] = 10.0F + static_cast<float>(i);
    velocity[i] = 30.0F + static_cast<float>(i);
    previous[i] = 50.0F + static_cast<float>(i);
  }
  const auto pair = phase_pair_from_fraction(0.25);
  const auto observation = build_policy_observation(
    command, angular, gravity, relative, velocity, previous, &pair);

  ASSERT_EQ(observation.size(), 47U);
  EXPECT_FLOAT_EQ(observation[0], 1.0F);
  EXPECT_FLOAT_EQ(observation[2], 3.0F);
  EXPECT_FLOAT_EQ(observation[3], 4.0F);
  EXPECT_FLOAT_EQ(observation[8], 9.0F);
  EXPECT_FLOAT_EQ(observation[9], 10.0F);
  EXPECT_FLOAT_EQ(observation[20], 21.0F);
  EXPECT_FLOAT_EQ(observation[21], 30.0F);
  EXPECT_FLOAT_EQ(observation[32], 41.0F);
  EXPECT_FLOAT_EQ(observation[33], 50.0F);
  EXPECT_FLOAT_EQ(observation[44], 61.0F);
  EXPECT_NEAR(observation[45], 1.0F, 1.0e-6F);
  EXPECT_NEAR(observation[46], 0.0F, 1.0e-6F);
}

TEST(ObservationBuilder, Legacy45DRemainsAvailable)
{
  const auto observation = build_policy_observation(
    std::vector<float>(3U, 0.0F),
    std::vector<float>(3U, 0.0F),
    std::array<float, 3>{0.0F, 0.0F, -1.0F},
    std::vector<float>(12U, 0.0F),
    std::vector<float>(12U, 0.0F),
    std::vector<float>(12U, 0.0F),
    nullptr);
  EXPECT_EQ(observation.size(), 45U);
}

TEST(StandPhase, CardinalInjectedPhasesMatchSinCosConvention)
{
  for (const auto & expected : std::vector<std::array<double, 3>>{
      {0.00, 0.0, 1.0},
      {0.25, 1.0, 0.0},
      {0.50, 0.0, -1.0},
      {0.75, -1.0, 0.0}})
  {
    PolicyPhaseState state;
    PolicyPhaseConfig config;
    config.mode = PolicyPhaseMode::randomized_static_per_episode;
    state.configure(config);
    state.begin_episode(std::nullopt, expected[0]);
    const auto sample = state.sample(CommandVelocity{});
    EXPECT_NEAR(sample.pair.sine, expected[1], 1.0e-6);
    EXPECT_NEAR(sample.pair.cosine, expected[2], 1.0e-6);
    EXPECT_NEAR(
      std::hypot(sample.pair.sine, sample.pair.cosine), 1.0, 1.0e-6);
  }
}

TEST(StandPhase, PairRemainsStaticAcrossTenThousandSuccessfulSamples)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::randomized_static_per_episode;
  state.configure(config);
  state.begin_episode(std::nullopt, 0.37125);
  const auto first = state.sample(CommandVelocity{});
  for (int i = 0; i < 10000; ++i) {
    state.on_successful_policy_tick(CommandVelocity{0.8, -0.4, 0.7});
    const auto sample = state.sample(CommandVelocity{-0.8, 0.4, -0.7});
    EXPECT_FLOAT_EQ(sample.pair.sine, first.pair.sine);
    EXPECT_FLOAT_EQ(sample.pair.cosine, first.pair.cosine);
    EXPECT_DOUBLE_EQ(sample.pair.phase, first.pair.phase);
  }
}

TEST(StandPhase, ReadinessLossDoesNotResampleAndNewEpisodeChangesOnlyTail)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::randomized_static_per_episode;
  state.configure(config);
  state.begin_episode(std::nullopt, 0.125);
  const auto first = state.sample(CommandVelocity{});
  // A readiness outage performs no phase call other than observation sampling.
  const auto after_outage = state.sample(CommandVelocity{1.0, 1.0, 1.0});
  EXPECT_FLOAT_EQ(after_outage.pair.sine, first.pair.sine);
  EXPECT_FLOAT_EQ(after_outage.pair.cosine, first.pair.cosine);

  const auto observation_a = build_at_phase(first.pair.phase);
  state.begin_episode(std::nullopt, 0.625);
  const auto second = state.sample(CommandVelocity{});
  const auto observation_b = build_at_phase(second.pair.phase);
  ASSERT_EQ(observation_a.size(), observation_b.size());
  for (std::size_t i = 0; i < 45U; ++i) {
    EXPECT_FLOAT_EQ(observation_a[i], observation_b[i]);
  }
  EXPECT_NE(observation_a[45], observation_b[45]);
  EXPECT_NE(observation_a[46], observation_b[46]);
}

TEST(WalkPhase, ThresholdsHoldStandingPhaseZero)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::command_synchronized_continuous_nonblocking;
  config.period_s = 0.84;
  state.configure(config);
  state.begin_episode();
  const auto sample = state.sample(CommandVelocity{0.20, 0.0, 0.08});
  EXPECT_FALSE(sample.moving);
  EXPECT_NEAR(sample.pair.phase, 0.0, 1.0e-12);
  EXPECT_NEAR(sample.pair.sine, 0.0, 1.0e-6);
  EXPECT_NEAR(sample.pair.cosine, 1.0, 1.0e-6);
}

TEST(WalkPhase, OnsetAlternatesFirstSwingAndAdvancesContinuously)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::command_synchronized_continuous_nonblocking;
  config.policy_dt_s = 0.02;
  config.period_s = 0.84;
  config.initial_first_swing_left = true;
  state.configure(config);
  state.begin_episode();

  const CommandVelocity moving{0.21, 0.0, 0.0};
  auto onset = state.sample(moving);
  EXPECT_TRUE(onset.moving);
  EXPECT_TRUE(onset.first_swing_left);
  EXPECT_NEAR(onset.pair.phase, 0.5, 1.0e-12);
  state.on_successful_policy_tick(moving);    // Commit onset; no advance.

  auto same_boundary = state.sample(moving);
  EXPECT_NEAR(same_boundary.pair.phase, 0.5, 1.0e-12);
  state.on_successful_policy_tick(moving);
  auto advanced = state.sample(moving);
  EXPECT_NEAR(advanced.pair.phase, 0.5 + 0.02 / 0.84, 1.0e-12);

  // Standing clears the active half-cycle but retains the alternating preference.
  state.on_successful_policy_tick(CommandVelocity{});
  const auto standing = state.sample(CommandVelocity{});
  EXPECT_NEAR(standing.pair.phase, 0.0, 1.0e-12);

  const auto second_onset = state.sample(moving);
  EXPECT_FALSE(second_onset.first_swing_left);
  EXPECT_NEAR(second_onset.pair.phase, 0.0, 1.0e-12);
}

TEST(WalkPhase, WrapsAndNewEpisodeDoesNotResumeMidGait)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::command_synchronized_continuous_nonblocking;
  config.policy_dt_s = 0.02;
  config.period_s = 0.80;
  state.configure(config);
  state.begin_episode();
  const CommandVelocity moving{0.3, 0.0, 0.0};
  state.on_successful_policy_tick(moving);    // onset 0.5
  for (int i = 0; i < 40; ++i) {
    state.on_successful_policy_tick(moving);
  }
  const auto wrapped = state.sample(moving);
  EXPECT_NEAR(wrapped.pair.phase, 0.5, 1.0e-9);

  state.begin_episode();
  const auto rearmed = state.sample(CommandVelocity{});
  EXPECT_NEAR(rearmed.pair.phase, 0.0, 1.0e-12);
  EXPECT_EQ(rearmed.successful_tick, 0U);
}

TEST(LocomotionPhase, NeutralStaticIsAlwaysSinZeroCosOne)
{
  PolicyPhaseState state;
  PolicyPhaseConfig config;
  config.mode = PolicyPhaseMode::neutral_static;
  config.policy_dt_s = 0.02;
  state.configure(config);
  state.begin_episode();

  for (int i = 0; i < 10000; ++i) {
    const CommandVelocity command{0.65, -0.30, 0.50};
    const auto before = state.sample(command);
    EXPECT_NEAR(before.pair.phase, 0.0, 1.0e-12);
    EXPECT_NEAR(before.pair.sine, 0.0, 1.0e-7);
    EXPECT_NEAR(before.pair.cosine, 1.0, 1.0e-7);
    state.on_successful_policy_tick(command);
  }
  const auto after = state.sample(CommandVelocity{-0.45, 0.30, -0.50});
  EXPECT_NEAR(after.pair.phase, 0.0, 1.0e-12);
  EXPECT_NEAR(after.pair.sine, 0.0, 1.0e-7);
  EXPECT_NEAR(after.pair.cosine, 1.0, 1.0e-7);
}

}  // namespace
