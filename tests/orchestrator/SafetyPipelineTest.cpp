#include "conflict/ConflictDetector.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteNavigator.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "orchestrator/WorldState.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/TrainManager.hpp"

#include <gtest/gtest.h>

namespace tcas::orchestrator
{
namespace
{

infrastructure::RailwayNetwork makeJunctionNetwork()
{
    infrastructure::RailwayNetwork net;
    net.addNode({1, "Origin A",   infrastructure::NodeType::Generic});
    net.addNode({2, "Junction",   infrastructure::NodeType::Junction});
    net.addNode({3, "Dest A",     infrastructure::NodeType::Generic});
    net.addNode({4, "Origin B",   infrastructure::NodeType::Generic});
    net.addNode({5, "Dest B",     infrastructure::NodeType::Generic});
    net.addTrack({101, 1, 2, 2000.0, 30.0, 0.0});
    net.addTrack({102, 2, 3, 1500.0, 30.0, 0.0});
    net.addTrack({103, 4, 2, 2000.0, 25.0, 0.0});
    net.addTrack({104, 2, 5, 1800.0, 25.0, 0.0});
    return net;
}

} // namespace

TEST(SafetyPipelineTest, EmptyRoutesProducesEmptyResult)
{
    train::TrainManager manager;
    const auto network = makeJunctionNetwork();
    SafetyPipeline pipeline(network, manager, {});
    const auto step = pipeline.makeStep();

    WorldState state;
    const auto result = step(state);
    EXPECT_TRUE(result.predictions.empty());
    EXPECT_TRUE(result.activeConflicts.empty());
    EXPECT_TRUE(result.commands.empty());
    EXPECT_TRUE(result.decisions.empty());
}

TEST(SafetyPipelineTest, SingleTrainProducesNoPairConflict)
{
    const auto network = makeJunctionNetwork();
    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setPosition(500.0);
    manager.getTrain(1)->setVelocity(20.0);

    const auto route = navigation::RouteNavigator::findRoute(network, 1, 3);
    ASSERT_TRUE(route.success);

    SafetyPipeline pipeline(network, manager, {{ 1, 101, route }});
    const auto step = pipeline.makeStep();

    WorldState state;
    state.trains.push_back({1, TrainType::Express, 101,
        45000.0, 45.0, 0.9, 1.4,
        TrainState::Running, 500.0, 20.0, 0.0});

    const auto result = step(state);
    // Single train: no conflict pairs
    EXPECT_TRUE(result.activeConflicts.empty());
    EXPECT_TRUE(result.commands.empty());
    // Should still produce predictions
    EXPECT_FALSE(result.predictions.empty());
}

TEST(SafetyPipelineTest, JunctionConflictProducesCommand)
{
    const auto network = makeJunctionNetwork();
    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));
    manager.getTrain(1)->setPosition(1900.0);
    manager.getTrain(1)->setVelocity(20.0);
    manager.getTrain(3)->setPosition(1900.0);
    manager.getTrain(3)->setVelocity(15.0);

    const auto routeExpress = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto routeFreight = navigation::RouteNavigator::findRoute(network, 4, 5);
    ASSERT_TRUE(routeExpress.success);
    ASSERT_TRUE(routeFreight.success);

    SafetyPipeline pipeline(network, manager, {
        { 1, 101, routeExpress },
        { 3, 103, routeFreight }
    });
    const auto step = pipeline.makeStep();

    WorldState state;
    state.trains.push_back({1, TrainType::Express, 101,
        45000.0, 45.0, 0.9, 1.4,
        TrainState::Running, 1900.0, 20.0, 0.0});
    state.trains.push_back({3, TrainType::Freight, 103,
        120000.0, 22.2, 0.5, 0.8,
        TrainState::Running, 1900.0, 15.0, 0.0});

    const auto result = step(state);
    EXPECT_FALSE(result.activeConflicts.empty());
    EXPECT_FALSE(result.commands.empty());
    EXPECT_FALSE(result.decisions.empty());

    // Express has higher priority — decision should yield Freight (#3)
    EXPECT_EQ(result.decisions.front().priorityTrain, 1U);
    EXPECT_EQ(result.decisions.front().yieldingTrain, 3U);
}

TEST(SafetyPipelineTest, SensorFailureIncreasesRisk)
{
    const auto network = makeJunctionNetwork();
    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));
    manager.getTrain(1)->setPosition(1900.0);
    manager.getTrain(1)->setVelocity(20.0);
    manager.getTrain(3)->setPosition(1900.0);
    manager.getTrain(3)->setVelocity(15.0);

    const auto routeExpress = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto routeFreight = navigation::RouteNavigator::findRoute(network, 4, 5);

    SafetyPipeline pipeline(network, manager, {
        { 1, 101, routeExpress },
        { 3, 103, routeFreight }
    });
    const auto step = pipeline.makeStep();

    WorldState normal;
    normal.trains.push_back({1, TrainType::Express, 101,
        45000.0, 45.0, 0.9, 1.4,
        TrainState::Running, 1900.0, 20.0, 0.0});
    normal.trains.push_back({3, TrainType::Freight, 103,
        120000.0, 22.2, 0.5, 0.8,
        TrainState::Running, 1900.0, 15.0, 0.0});
    normal.sensorFailure = false;

    WorldState degraded = normal;
    degraded.sensorFailure = true;

    const auto resultNormal  = step(normal);
    const auto resultDegraded = step(degraded);

    if (!resultNormal.decisions.empty() && !resultDegraded.decisions.empty())
    {
        EXPECT_GE(resultDegraded.decisions.front().riskScore,
                  resultNormal.decisions.front().riskScore);
    }
}

TEST(SafetyPipelineTest, SetRoutesUpdatesContext)
{
    const auto network = makeJunctionNetwork();
    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setPosition(500.0);
    manager.getTrain(1)->setVelocity(20.0);

    SafetyPipeline pipeline(network, manager, {});
    EXPECT_NO_THROW(pipeline.setRoutes(
        {{ 1, 101, navigation::RouteNavigator::findRoute(network, 1, 3) }}));
}

TEST(SafetyPipelineTest, DistantRearEndConvoyReceivesNoBrakingCommand)
{
    const auto network = makeJunctionNetwork();
    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setPosition(1200.0);
    manager.getTrain(1)->setVelocity(25.0);

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        2, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(2)->setPosition(500.0);
    manager.getTrain(2)->setVelocity(25.0);

    const auto route = navigation::RouteNavigator::findRoute(network, 1, 3);
    ASSERT_TRUE(route.success);

    SafetyPipeline pipeline(network, manager, {
        { 1, 101, route },
        { 2, 101, route }
    });
    const auto step = pipeline.makeStep();

    WorldState state;
    state.trains.push_back({1, TrainType::Express, 101,
        45000.0, 45.0, 0.9, 1.4,
        TrainState::Running, 1200.0, 25.0, 0.0});
    state.trains.push_back({2, TrainType::Express, 101,
        45000.0, 45.0, 0.9, 1.4,
        TrainState::Running, 500.0, 25.0, 0.0});

    const auto result = step(state);
    for (const auto& cmd : result.commands)
    {
        if (cmd.trainId == 2)
        {
            EXPECT_NE(cmd.type, safety::SafetyCommandType::EmergencyBrake);
            EXPECT_NE(cmd.type, safety::SafetyCommandType::HoldAtSignal);
        }
    }
}

} // namespace tcas::orchestrator
