#include "safety/RiskEngine.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

RiskInput baseInput()
{
    RiskInput input;
    input.timeToCollision = 20.0;
    input.relativeVelocity = 0.0;
    input.brakingDistance = 0.0;
    input.safetyMargin = 100.0;
    input.conflictType = conflict::ConflictType::Junction;
    input.trainMass = 45000.0;
    input.sensorConfidence = 1.0;
    input.communicationConfidence = 1.0;
    return input;
}

} // namespace

TEST(RiskEngineTest, LowTtcProducesHighRisk)
{
    auto input = baseInput();
    input.timeToCollision = 1.0;
    input.brakingDistance = 100.0;
    input.safetyMargin = 1.0;

    const auto assessment = RiskEngine{}.assess(input);

    EXPECT_GT(assessment.score, 60.0);
    EXPECT_EQ(assessment.level, RiskLevel::High);
}

TEST(RiskEngineTest, CriticalRiskAboveEighty)
{
    auto input = baseInput();
    input.timeToCollision = 0.0;
    input.relativeVelocity = 25.0;
    input.brakingDistance = 500.0;
    input.safetyMargin = -1.0;
    input.conflictType = conflict::ConflictType::HeadOn;
    input.trainMass = 150000.0;

    const auto assessment = RiskEngine{}.assess(input);

    EXPECT_GT(assessment.score, 80.0);
    EXPECT_EQ(assessment.level, RiskLevel::Critical);
}

TEST(RiskEngineTest, RiskScoreIsClampedToZeroHundred)
{
    auto input = baseInput();
    input.timeToCollision = 0.0;
    input.relativeVelocity = 100.0;
    input.brakingDistance = 1000.0;
    input.safetyMargin = -100.0;
    input.conflictType = conflict::ConflictType::HeadOn;
    input.trainMass = 500000.0;
    input.sensorConfidence = 0.0;
    input.communicationConfidence = 0.0;

    const auto assessment = RiskEngine{}.assess(input);

    EXPECT_GE(assessment.score, 0.0);
    EXPECT_LE(assessment.score, 100.0);
    EXPECT_EQ(assessment.score, 100.0);
}

TEST(RiskEngineTest, HeadOnConflictHasHigherRisk)
{
    auto junctionInput = baseInput();
    auto headOnInput = junctionInput;
    headOnInput.conflictType = conflict::ConflictType::HeadOn;

    const RiskEngine engine;
    EXPECT_GT(engine.assess(headOnInput).score, engine.assess(junctionInput).score);
}

TEST(RiskEngineTest, DegradedSensorIncreasesRisk)
{
    auto input = baseInput();
    const RiskEngine engine;
    const auto healthy = engine.assess(input);
    input.sensorConfidence = 0.0;

    EXPECT_GT(engine.assess(input).score, healthy.score);
}

TEST(RiskEngineTest, DegradedCommunicationIncreasesRisk)
{
    auto input = baseInput();
    const RiskEngine engine;
    const auto healthy = engine.assess(input);
    input.communicationConfidence = 0.0;

    EXPECT_GT(engine.assess(input).score, healthy.score);
}

TEST(RiskEngineTest, InfiniteAndNegativeTtcProduceZeroTtcRisk)
{
    auto input = baseInput();
    input.timeToCollision = 100.0; // > 60s has 0 TTC risk
    const RiskEngine engine;
    const auto baseline = engine.assess(input);

    input.timeToCollision = std::numeric_limits<double>::infinity();
    const auto infAssessment = engine.assess(input);
    EXPECT_DOUBLE_EQ(infAssessment.score, baseline.score);
    EXPECT_LE(infAssessment.score, 30.0);
    EXPECT_EQ(infAssessment.level, RiskLevel::Low);

    input.timeToCollision = -5.0;
    const auto negAssessment = engine.assess(input);
    EXPECT_DOUBLE_EQ(negAssessment.score, baseline.score);
}

} // namespace tcas::safety
