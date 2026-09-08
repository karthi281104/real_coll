#include "conflict/ConflictDetector.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"

#include <gtest/gtest.h>

namespace tcas::safety
{
namespace
{

infrastructure::RailwayNetwork makeNetwork()
{
    infrastructure::RailwayNetwork network;

    network.addNode({
        1,
        "Express Origin",
        infrastructure::NodeType::Generic
    });

    network.addNode({
        2,
        "Alpha Junction",
        infrastructure::NodeType::Junction
    });

    network.addNode({
        3,
        "Express Destination",
        infrastructure::NodeType::Generic
    });

    network.addNode({
        4,
        "Freight Origin",
        infrastructure::NodeType::Generic
    });

    network.addNode({
        5,
        "Freight Destination",
        infrastructure::NodeType::Generic
    });

    network.addTrack({
        101,
        1,
        2,
        1000.0,
        30.0,
        0.0
    });

    network.addTrack({
        102,
        2,
        3,
        1000.0,
        30.0,
        0.0
    });

    network.addTrack({
        103,
        4,
        2,
        1000.0,
        30.0,
        0.0
    });

    network.addTrack({
        104,
        2,
        5,
        1000.0,
        30.0,
        0.0
    });

    return network;
}

prediction::FutureState state(
    TimeSeconds time,
    TrackId track)
{
    return {
        time,
        track,
        0.0,
        20.0,
        0.0,
        1.0
    };
}

} // namespace

TEST(
    SafetyIntegrationTest,
    JunctionConflictProducesPriorityAndFreightResolution)
{
    const auto network = makeNetwork();

    const auto expressTrajectory =
        std::vector<prediction::FutureState>{
            state(0.0, 101),
            state(20.0, 102)
        };

    const auto freightTrajectory =
        std::vector<prediction::FutureState>{
            state(0.0, 103),
            state(21.0, 104)
        };

    const auto conflicts =
        conflict::ConflictDetector{}.detect(
            1,
            expressTrajectory,
            3,
            freightTrajectory,
            network);

    ASSERT_EQ(conflicts.size(), 1U);

    ASSERT_EQ(
        conflicts.front().type,
        conflict::ConflictType::Junction);

    train::ExpressTrain express(
        1,
        45000.0,
        45.0,
        0.9,
        1.4);

    train::FreightTrain freight(
        3,
        120000.0,
        22.2,
        0.5,
        0.8);

    express.setVelocity(20.0);
    freight.setVelocity(20.0);

    RiskInput riskInput;

    riskInput.timeToCollision =
        conflicts.front().firstConflictTime;

    // Two trains converging at a junction from perpendicular directions each at
    // 20 m/s; their combined closing speed component is ~10 m/s (BUG-7 fix:
    // relativeVelocity must be non-zero here to push the risk score above 30.0
    // into Medium, matching the physical reality of a junction conflict).
    riskInput.relativeVelocity = 10.0;

    riskInput.brakingDistance = 100.0;

    riskInput.safetyMargin = 50.0;

    riskInput.conflictType =
        conflicts.front().type;

    // Risk is evaluated from the Freight train's
    // operational perspective.
    riskInput.trainMass = freight.mass();

    riskInput.sensorConfidence = 1.0;
    riskInput.communicationConfidence = 1.0;

    const RiskAssessment risk =
        RiskEngine{}.assess(riskInput);

    ConflictPriorityQueue queue;

    queue.push(
        conflicts.front(),
        risk);

    ASSERT_FALSE(queue.empty());

    const auto prioritized =
        queue.top();

    const PriorityEngine priorityEngine;

    const auto expressPriority =
        priorityEngine.assess(express);

    const auto freightPriority =
        priorityEngine.assess(freight);

    ASSERT_TRUE(
        expressPriority.higherThan(
            freightPriority));

    // Freight is the yielding train.
    const ResolutionInput resolutionInput{
        freight,
        prioritized.conflict,
        prioritized.risk,
        false,
        true,
        1000.0,
        100.0
    };

    const auto command =
        ResolutionEngine{}.resolve(
            resolutionInput);

    EXPECT_EQ(
        command.trainId,
        freight.id());

    EXPECT_EQ(
        command.type,
        SafetyCommandType::ReduceSpeed);

    EXPECT_DOUBLE_EQ(
        command.targetSpeed,
        10.0);
}

TEST(
    SafetyIntegrationTest,
    PriorityDoesNotOverrideUnsafeStoppingDistance)
{
    const auto network = makeNetwork();

    const auto expressTrajectory =
        std::vector<prediction::FutureState>{
            state(0.0, 101),
            state(20.0, 102)
        };

    const auto freightTrajectory =
        std::vector<prediction::FutureState>{
            state(0.0, 103),
            state(21.0, 104)
        };

    const auto conflicts =
        conflict::ConflictDetector{}.detect(
            1,
            expressTrajectory,
            3,
            freightTrajectory,
            network);

    ASSERT_EQ(conflicts.size(), 1U);

    train::FreightTrain freight(
        3,
        120000.0,
        22.2,
        0.5,
        0.8);

    freight.setVelocity(20.0);

    RiskAssessment risk;
    risk.score = 70.0;
    risk.level = RiskLevel::High;
    risk.safetyMargin = 50.0;

    const ResolutionInput input{
        freight,
        conflicts.front(),
        risk,

        // Freight does NOT have priority.
        false,

        // Caller claims braking is feasible,
        // but distances prove otherwise.
        true,

        100.0,
        200.0
    };

    const auto command =
        ResolutionEngine{}.resolve(input);

    EXPECT_EQ(
        command.type,
        SafetyCommandType::EmergencyBrake);
}

} // namespace tcas::safety