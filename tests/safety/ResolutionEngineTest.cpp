#include "safety/ResolutionEngine.hpp"

#include "train/FreightTrain.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

train::FreightTrain makeTrain()
{
    train::FreightTrain train(
        3,
        120000.0,
        22.2,
        0.5,
        0.8);

    train.setVelocity(20.0);

    return train;
}

conflict::Conflict makeConflict()
{
    return {
        1,
        3,
        conflict::ConflictType::Junction,
        0,
        2,
        5.0,
        7.0,
        0.0
    };
}

RiskAssessment makeRisk(
    RiskLevel level,
    double score,
    double safetyMargin = 0.0)
{
    RiskAssessment risk;

    risk.level = level;
    risk.score = score;
    risk.safetyMargin = safetyMargin;

    return risk;
}

SafetyCommand resolve(
    const RiskAssessment& risk,
    bool priorityGranted = false,
    bool brakingFeasible = true,
    double availableDistance = 1000.0,
    double requiredBrakingDistance = 100.0)
{
    const auto train = makeTrain();

    return ResolutionEngine{}.resolve({
        train,
        makeConflict(),
        risk,
        priorityGranted,
        brakingFeasible,
        availableDistance,
        requiredBrakingDistance
    });
}

} // namespace

TEST(
    ResolutionEngineTest,
    CriticalRiskProducesEmergencyBrake)
{
    const auto command =
        resolve(makeRisk(RiskLevel::Critical, 90.0));

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);

    EXPECT_TRUE(command.isEmergency());
}

TEST(
    ResolutionEngineTest,
    ExplicitlyInfeasibleBrakingProducesEmergencyBrake)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::High, 70.0),
            false,
            false,
            1000.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);
}

TEST(
    ResolutionEngineTest,
    CalculatedInfeasibleBrakingProducesEmergencyBrake)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::High, 70.0),
            false,
            true,
            50.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);
}

TEST(
    ResolutionEngineTest,
    SafetyMarginIsIncludedInBrakingFeasibility)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::High, 70.0, 50.0),
            false,
            true,
            120.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);
}

TEST(
    ResolutionEngineTest,
    HighRiskNonPriorityTrainHolds)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::High, 70.0),
            false,
            true,
            1000.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::HoldAtSignal);

    EXPECT_DOUBLE_EQ(
        command.targetSpeed,
        0.0);
}

TEST(
    ResolutionEngineTest,
    MediumRiskProducesSpeedReduction)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::Medium, 50.0),
            true,
            true,
            1000.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::ReduceSpeed);

    EXPECT_DOUBLE_EQ(
        command.targetSpeed,
        10.0);
}

TEST(
    ResolutionEngineTest,
    LowRiskProducesNoAction)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::Low, 10.0),
            false,
            true,
            1000.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::NoAction);

    EXPECT_DOUBLE_EQ(
        command.targetSpeed,
        20.0);
}

TEST(
    ResolutionEngineTest,
    PriorityCannotOverrideUnsafeBraking)
{
    const auto command =
        resolve(
            makeRisk(RiskLevel::High, 70.0),
            true,
            true,
            50.0,
            100.0);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);

    EXPECT_TRUE(command.isEmergency());
}

TEST(
    ResolutionEngineTest,
    SlowMovingTrainMaintainsCautionFloorWhenDistanceIsAmple)
{
    train::FreightTrain slowTrain(
        4, 120000.0, 22.2, 0.5, 0.8);
    slowTrain.setVelocity(1.0);

    const auto command = ResolutionEngine{}.resolve({
        slowTrain,
        makeConflict(),
        makeRisk(RiskLevel::Medium, 50.0),
        false,
        true,
        500.0,
        10.0
    });

    EXPECT_EQ(command.type, SafetyCommandType::ReduceSpeed);
    EXPECT_GE(command.targetSpeed, 3.0);
}

} // namespace tcas::safety