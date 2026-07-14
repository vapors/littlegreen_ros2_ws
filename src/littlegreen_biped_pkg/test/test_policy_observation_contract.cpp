#include <array>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "littlegreen_biped_pkg/policy_observation_contract.hpp"

namespace
{
using littlegreen_biped::GaitPhaseClock;
using littlegreen_biped::build_policy_observation;

TEST(GaitPhaseClock, StartsAtPhaseZeroAndWrapsAfterThirtySixTicks)
{
    GaitPhaseClock clock;
    clock.configure(0.72, 0.02);

    const auto first = clock.sample();
    EXPECT_EQ(first.tick, 0U);
    EXPECT_EQ(first.period_ticks, 36U);
    EXPECT_FLOAT_EQ(first.phase, 0.0F);
    EXPECT_NEAR(first.sine, 0.0F, 1.0e-6F);
    EXPECT_NEAR(first.cosine, 1.0F, 1.0e-6F);
    EXPECT_EQ(first.expected_half_cycle, 0U);

    for (int i = 0; i < 18; ++i) {
        clock.advance();
    }
    const auto half = clock.sample();
    EXPECT_NEAR(half.phase, 0.5F, 1.0e-6F);
    EXPECT_NEAR(half.sine, 0.0F, 1.0e-6F);
    EXPECT_NEAR(half.cosine, -1.0F, 1.0e-6F);
    EXPECT_EQ(half.expected_half_cycle, 1U);

    for (int i = 0; i < 18; ++i) {
        clock.advance();
    }
    const auto wrapped = clock.sample();
    EXPECT_EQ(wrapped.tick, 36U);
    EXPECT_FLOAT_EQ(wrapped.phase, 0.0F);
    EXPECT_NEAR(wrapped.sine, 0.0F, 1.0e-6F);
    EXPECT_NEAR(wrapped.cosine, 1.0F, 1.0e-6F);
}

TEST(GaitPhaseClock, ExplicitResetReturnsToPhaseZero)
{
    GaitPhaseClock clock;
    clock.configure(0.72, 0.02);
    for (int i = 0; i < 11; ++i) {
        clock.advance();
    }
    EXPECT_NE(clock.sample().tick, 0U);
    clock.reset();
    const auto reset = clock.sample();
    EXPECT_EQ(reset.tick, 0U);
    EXPECT_FLOAT_EQ(reset.phase, 0.0F);
    EXPECT_NEAR(reset.sine, 0.0F, 1.0e-6F);
    EXPECT_NEAR(reset.cosine, 1.0F, 1.0e-6F);
}

TEST(GaitPhaseClock, RejectsNonIntegralPolicyTickPeriod)
{
    GaitPhaseClock clock;
    EXPECT_THROW(clock.configure(0.73, 0.02), std::invalid_argument);
}

TEST(ObservationBuilder, PreservesLegacyOrdering)
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

    const auto observation = build_policy_observation(
        command, angular, gravity, relative, velocity, previous, nullptr);
    ASSERT_EQ(observation.size(), 45U);
    EXPECT_FLOAT_EQ(observation[0], 1.0F);
    EXPECT_FLOAT_EQ(observation[8], 9.0F);
    EXPECT_FLOAT_EQ(observation[9], 10.0F);
    EXPECT_FLOAT_EQ(observation[21], 30.0F);
    EXPECT_FLOAT_EQ(observation[33], 50.0F);
    EXPECT_FLOAT_EQ(observation[44], 61.0F);
}

TEST(ObservationBuilder, AppendsPhaseAfterPreviousAction)
{
    GaitPhaseClock clock;
    clock.configure(0.72, 0.02);
    const auto phase = clock.sample();

    const auto observation = build_policy_observation(
        std::vector<float>(3U, 0.0F),
        std::vector<float>(3U, 0.0F),
        std::array<float, 3>{0.0F, 0.0F, -1.0F},
        std::vector<float>(12U, 0.0F),
        std::vector<float>(12U, 0.0F),
        std::vector<float>(12U, 0.0F),
        &phase);

    ASSERT_EQ(observation.size(), 47U);
    EXPECT_NEAR(observation[45], 0.0F, 1.0e-6F);
    EXPECT_NEAR(observation[46], 1.0F, 1.0e-6F);
}
}  // namespace
