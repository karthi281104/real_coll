#include "conflict/ConflictDetector.hpp"
#include "conflict/ResourceReservationManager.hpp"
#include "communication/CommunicationChannel.hpp"
#include "infrastructure/Node.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "infrastructure/Track.hpp"
#include "navigation/RouteNavigator.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "prediction/PredictionEngine.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <atomic>
#include <thread>
#include <vector>

namespace tcas::orchestrator
{
namespace
{

std::unique_ptr<ThreadOrchestrator> makeOrchestrator(
    train::TrainManager& manager,
    communication::CommunicationChannel& channel)
{
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setVelocity(10.0);

    static const infrastructure::RailwayNetwork network;
    const OrchestratorConfig config{};
    const SafetyStep safetyStep =
        [](const WorldState&)
        {
            SafetyCycleResult result;
            result.commands.push_back({
                safety::SafetyCommandType::ReduceSpeed,
                1,
                5.0,
                0.0,
                1.0});
            return result;
        };
    const std::vector<TrainId> trainIds{ 1 };
    return std::make_unique<ThreadOrchestrator>(
        network,
        manager,
        channel,
        trainIds,
        config,
        safetyStep);
}

infrastructure::RailwayNetwork makeJunctionNetwork()
{
    infrastructure::RailwayNetwork network;
    network.addNode({1, "Express Origin", infrastructure::NodeType::Generic});
    network.addNode({2, "Alpha Junction", infrastructure::NodeType::Junction});
    network.addNode({3, "Express Destination", infrastructure::NodeType::Generic});
    network.addNode({4, "Freight Origin", infrastructure::NodeType::Generic});
    network.addNode({5, "Freight Destination", infrastructure::NodeType::Generic});
    network.addTrack({101, 1, 2, 2000.0, 30.0, 0.0});
    network.addTrack({102, 2, 3, 1500.0, 30.0, 0.0});
    network.addTrack({103, 4, 2, 2000.0, 25.0, 0.0});
    network.addTrack({104, 2, 5, 1800.0, 25.0, 0.0});
    return network;
}

} // namespace

TEST(ThreadOrchestratorTest, StartStopAndRestart)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    EXPECT_TRUE(orchestrator->isRunning());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    orchestrator->stop();
    EXPECT_FALSE(orchestrator->isRunning());

    const auto firstPhysicsCycles = orchestrator->physicsCycles();
    EXPECT_GT(firstPhysicsCycles, 0U);
    EXPECT_GT(orchestrator->safetyCycles(), 0U);
    EXPECT_GT(orchestrator->communicationCycles(), 0U);
    EXPECT_GT(orchestrator->hmiCycles(), 0U);

    orchestrator->start();
    EXPECT_TRUE(orchestrator->isRunning());
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    orchestrator->stop();
    EXPECT_FALSE(orchestrator->isRunning());
    EXPECT_GT(orchestrator->physicsCycles(), firstPhysicsCycles);
}

TEST(ThreadOrchestratorTest, SafetyCommandReachesPhysics)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
    orchestrator->stop();

    ASSERT_NE(manager.getTrain(1), nullptr);
    EXPECT_LE(manager.getTrain(1)->velocity(), 5.0);
}

TEST(ThreadOrchestratorTest, SnapshotContainsTrainState)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    const auto state = orchestrator->snapshot();
    ASSERT_EQ(state.trains.size(), 1U);
    EXPECT_EQ(state.trains.front().id, 1U);
}

TEST(ThreadOrchestratorTest, SafetyResultPopulatesWorldState)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    static const infrastructure::RailwayNetwork network;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        OrchestratorConfig{},
        [](const WorldState&)
        {
            SafetyCycleResult result;
            result.activeConflicts.push_back({
                1, 2, conflict::ConflictType::Junction,
                0, 9, 1.0, 2.0, 0.0});
            return result;
        });

    orchestrator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    orchestrator.stop();

    EXPECT_FALSE(orchestrator.snapshot().activeConflicts.empty());
}

TEST(ThreadOrchestratorTest, PeriodsProduceExpectedCycleRanges)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    orchestrator->stop();

    EXPECT_GE(orchestrator->physicsCycles(), 40U);
    EXPECT_LE(orchestrator->physicsCycles(), 60U);
    EXPECT_GE(orchestrator->safetyCycles(), 8U);
    EXPECT_LE(orchestrator->safetyCycles(), 12U);
    EXPECT_GE(orchestrator->communicationCycles(), 8U);
    EXPECT_LE(orchestrator->communicationCycles(), 12U);
    EXPECT_GE(orchestrator->hmiCycles(), 4U);
    EXPECT_LE(orchestrator->hmiCycles(), 7U);
}

TEST(ThreadOrchestratorTest, ConcurrentSnapshotsRemainConsistent)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    auto orchestrator = makeOrchestrator(manager, channel);

    orchestrator->start();
    for (int iteration = 0; iteration < 500; ++iteration)
    {
        const auto state = orchestrator->snapshot();
        EXPECT_LE(state.trains.size(), 1U);
        if (!state.trains.empty())
        {
            EXPECT_TRUE(std::isfinite(state.trains.front().position));
            EXPECT_TRUE(std::isfinite(state.trains.front().velocity));
        }
        std::this_thread::yield();
    }
    orchestrator->stop();

    EXPECT_FALSE(orchestrator->isRunning());
}

TEST(ThreadOrchestratorTest, RealSafetyPipelinePublishesEmergencyCommand)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.addTrain(std::make_unique<train::FreightTrain>(
        3, 120000.0, 22.2, 0.5, 0.8));
    manager.getTrain(1)->setPosition(1900.0);
    manager.getTrain(1)->setVelocity(20.0);
    manager.getTrain(3)->setPosition(1900.0);
    manager.getTrain(3)->setVelocity(20.0);

    const auto expressRoute = navigation::RouteNavigator::findRoute(network, 1, 3);
    const auto freightRoute = navigation::RouteNavigator::findRoute(network, 4, 5);
    ASSERT_TRUE(expressRoute.success);
    ASSERT_TRUE(freightRoute.success);

    auto sawConflict = std::make_shared<std::atomic<bool>>(false);
    auto sawEmergency = std::make_shared<std::atomic<bool>>(false);

    const SafetyStep safetyStep =
        [network, expressRoute, freightRoute, sawConflict, sawEmergency](
            const WorldState& state)
        {
            SafetyCycleResult result;
            const auto findTrain = [&state](TrainId id) -> const TrainSnapshot*
            {
                for (const auto& train : state.trains)
                {
                    if (train.id == id)
                    {
                        return &train;
                    }
                }
                return nullptr;
            };

            const auto* expressState = findTrain(1);
            const auto* freightState = findTrain(3);
            if (expressState == nullptr || freightState == nullptr)
            {
                return result;
            }

            train::ExpressTrain express(1, 45000.0, 45.0, 0.9, 1.4);
            train::FreightTrain freight(3, 120000.0, 22.2, 0.5, 0.8);
            express.setPosition(expressState->position);
            express.setVelocity(expressState->velocity);
            express.setAcceleration(expressState->acceleration);
            freight.setPosition(freightState->position);
            freight.setVelocity(freightState->velocity);
            freight.setAcceleration(freightState->acceleration);

            const auto expressPrediction = prediction::PredictionEngine::predictStandardHorizon(
                express, network, expressRoute, 101, 1.0);
            const auto freightPrediction = prediction::PredictionEngine::predictStandardHorizon(
                freight, network, freightRoute, 103, 1.0);
            result.predictions = expressPrediction;

            const auto conflicts = conflict::ConflictDetector{}.detect(
                1, expressPrediction, 3, freightPrediction, network);
            result.activeConflicts = conflicts;
            if (conflicts.empty())
            {
                return result;
            }
            sawConflict->store(true);

            safety::ConflictPriorityQueue queue;
            const auto& detected = conflicts.front();
            safety::RiskInput riskInput;
            riskInput.timeToCollision = detected.firstConflictTime;
            riskInput.brakingDistance = 250.0;
            riskInput.safetyMargin = -150.0;
            riskInput.conflictType = detected.type;
            riskInput.trainMass = freight.mass();
            queue.push(detected, safety::RiskEngine{}.assess(riskInput));

            const auto prioritized = queue.top();
            const conflict::ConflictZone zone{
                conflict::ConflictZoneType::Junction,
                prioritized.conflict.resourceNodeId,
                0};
            conflict::ResourceReservationManager reservations;
            const bool reserved = reservations.request(
                1,
                zone,
                prioritized.conflict.firstConflictTime,
                prioritized.conflict.lastConflictTime);
            if (!reserved)
            {
                return result;
            }
            result.reservations = reservations.reservations();

            const safety::ResolutionInput resolutionInput{
                freight,
                prioritized.conflict,
                prioritized.risk,
                false,
                true,
                100.0,
                250.0};
            result.commands.push_back(safety::ResolutionEngine{}.resolve(resolutionInput));
            sawEmergency->store(result.commands.back().isEmergency());
            return result;
        };

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1, 3 },
        OrchestratorConfig{},
        safetyStep);
    orchestrator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(160));
    orchestrator.stop();

    const auto state = orchestrator.snapshot();
    EXPECT_TRUE(sawConflict->load());
    EXPECT_TRUE(sawEmergency->load());
}

TEST(ThreadOrchestratorTest, DynamicTrainAndFaultManagement)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setVelocity(10.0);

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        OrchestratorConfig{});

    orchestrator.start();

    // Verify initial state
    auto state = orchestrator.snapshot();
    EXPECT_EQ(state.trains.size(), 1U);
    EXPECT_FALSE(state.sensorFailure);
    EXPECT_FALSE(state.communicationFailure);

    // Live fault injection
    orchestrator.setSensorFault(true);
    orchestrator.setCommFault(true);
    state = orchestrator.snapshot();
    EXPECT_TRUE(state.sensorFailure);
    EXPECT_TRUE(state.communicationFailure);

    // Live fault recovery
    orchestrator.setSensorFault(false);
    orchestrator.setCommFault(false);
    state = orchestrator.snapshot();
    EXPECT_FALSE(state.sensorFailure);
    EXPECT_FALSE(state.communicationFailure);

    // Live train addition
    manager.addTrain(std::make_unique<train::FreightTrain>(
        2, 120000.0, 22.2, 0.5, 0.8));
    manager.getTrain(2)->setVelocity(8.0);
    orchestrator.addTrain(2);

    state = orchestrator.snapshot();
    EXPECT_EQ(state.trains.size(), 2U);

    // Live train removal
    orchestrator.removeTrain(1);
    state = orchestrator.snapshot();
    EXPECT_EQ(state.trains.size(), 1U);
    EXPECT_EQ(state.trains.front().id, 2U);

    orchestrator.stop();
}

TEST(ThreadOrchestratorTest, TruePauseAndResume)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setVelocity(20.0);

    OrchestratorConfig cfg;
    cfg.physicsPeriod = std::chrono::milliseconds(10);
    cfg.safetyPeriod = std::chrono::milliseconds(20);
    cfg.hmiPeriod = std::chrono::milliseconds(20);

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        cfg);

    orchestrator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    EXPECT_TRUE(orchestrator.isRunning());
    EXPECT_FALSE(orchestrator.isPaused());

    // Pause the simulation
    orchestrator.pause();
    EXPECT_TRUE(orchestrator.isPaused());

    const auto statePaused1 = orchestrator.snapshot();
    EXPECT_EQ(statePaused1.systemStatus, SystemStatus::Paused);
    const double pausedSimTime = statePaused1.simulationTime;
    const double pausedPos = statePaused1.trains.front().position;
    const std::size_t pausedPhysCycles = orchestrator.physicsCycles();
    const std::size_t pausedHmiCycles = orchestrator.hmiCycles();

    // Sleep while paused
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    const auto statePaused2 = orchestrator.snapshot();
    // Sim time and train position must NOT advance while paused
    EXPECT_DOUBLE_EQ(statePaused2.simulationTime, pausedSimTime);
    EXPECT_DOUBLE_EQ(statePaused2.trains.front().position, pausedPos);
    EXPECT_EQ(orchestrator.physicsCycles(), pausedPhysCycles);

    // HMI continues running
    EXPECT_GT(orchestrator.hmiCycles(), pausedHmiCycles);

    // Resume the simulation
    orchestrator.resume();
    EXPECT_FALSE(orchestrator.isPaused());
    std::this_thread::sleep_for(std::chrono::milliseconds(80));

    const auto stateResumed = orchestrator.snapshot();
    EXPECT_GT(stateResumed.simulationTime, pausedSimTime);
    EXPECT_GT(stateResumed.trains.front().position, pausedPos);

    orchestrator.stop();
}

TEST(ThreadOrchestratorTest, UserCommandQueueExecution)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.getTrain(1)->setVelocity(10.0);

    OrchestratorConfig cfg;
    cfg.physicsPeriod = std::chrono::milliseconds(10);

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        cfg);

    orchestrator.start();

    // Post SetSpeed command via queue
    orchestrator.postCommand({UserCommandType::SetSpeed, 1, 35.0});
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    auto state = orchestrator.snapshot();
    EXPECT_DOUBLE_EQ(state.trains.front().velocity, 35.0);

    // Post Hold command via queue
    orchestrator.postCommand({UserCommandType::HoldTrain, 1});
    std::this_thread::sleep_for(std::chrono::milliseconds(60));

    state = orchestrator.snapshot();
    EXPECT_DOUBLE_EQ(state.trains.front().velocity, 0.0);
    EXPECT_EQ(state.trains.front().state, TrainState::Stopped);

    orchestrator.stop();
}

TEST(ThreadOrchestratorTest, TrackBoundaryProgression)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));

    // Track 101 has length 2000m. Put train at 1990m, moving at 40 m/s
    manager.getTrain(1)->setPosition(1990.0);
    manager.getTrain(1)->setVelocity(40.0);

    const auto route = navigation::RouteNavigator::findRoute(network, 1, 3);
    ASSERT_TRUE(route.success);
    ASSERT_GE(route.tracks.size(), 2U);

    OrchestratorConfig cfg;
    cfg.physicsPeriod = std::chrono::milliseconds(10);

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1 },
        cfg);

    orchestrator.setTrainRoute(1, 101, route);
    orchestrator.start();

    // Let train cross boundary from Track 101 to 102
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    orchestrator.stop();

    const auto state = orchestrator.snapshot();
    ASSERT_FALSE(state.trains.empty());
    // Train should have transitioned to Track 102!
    EXPECT_EQ(state.trains.front().trackId, 102U);
    EXPECT_LT(state.trains.front().position, 2000.0);
}

TEST(ThreadOrchestratorTest, PerTrainSensorFault)
{
    train::TrainManager manager;
    communication::CommunicationChannel channel;
    const auto network = makeJunctionNetwork();

    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    manager.addTrain(std::make_unique<train::FreightTrain>(
        2, 120000.0, 22.2, 0.5, 0.8));

    ThreadOrchestrator orchestrator(
        network,
        manager,
        channel,
        std::vector<TrainId>{ 1, 2 },
        OrchestratorConfig{});

    orchestrator.start();

    // Inject sensor fault specifically on Train 1
    orchestrator.setSensorFault(1, true);

    auto state = orchestrator.snapshot();
    EXPECT_TRUE(state.sensorFailure);

    ASSERT_EQ(state.trains.size(), 2U);
    for (const auto& t : state.trains)
    {
        if (t.id == 1)
        {
            EXPECT_TRUE(t.sensorFailure);
        }
        else if (t.id == 2)
        {
            EXPECT_FALSE(t.sensorFailure);
        }
    }

    // Clear sensor fault on Train 1
    orchestrator.setSensorFault(1, false);
    state = orchestrator.snapshot();
    EXPECT_FALSE(state.sensorFailure);
    for (const auto& t : state.trains)
    {
        EXPECT_FALSE(t.sensorFailure);
    }

    orchestrator.stop();
}

TEST(ThreadOrchestratorTest, CompletedTrainIsRemovedAfterDwellPeriod)
{
    infrastructure::RailwayNetwork network;
    network.addNode({ 1, "Station A", infrastructure::NodeType::Generic });
    network.addNode({ 2, "Station B", infrastructure::NodeType::Generic });
    network.addTrack({ 101, 1, 2, 100.0, 30.0, 0.0 });

    train::TrainManager manager;
    manager.addTrain(std::make_unique<train::ExpressTrain>(
        1, 45000.0, 45.0, 0.9, 1.4));
    auto* t = manager.getTrain(1);
    t->setPosition(99.0);
    t->setVelocity(30.0);
    t->setState(TrainState::Running);

    navigation::RouteResult route;
    route.success = true;
    route.tracks = { 101 };

    communication::CommunicationChannel channel;
    OrchestratorConfig config;
    config.physicsPeriod = std::chrono::milliseconds(10);
    config.completedTrainDwellSeconds = 0.1; // 100ms dwell for fast test

    ThreadOrchestrator orchestrator(
        network, manager, channel, { 1 }, config);
    orchestrator.setTrainRoute(1, 101, route);

    orchestrator.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    orchestrator.stop();

    auto state = orchestrator.snapshot();
    // Train should now be removed from active service
    EXPECT_TRUE(state.trains.empty());
}

} // namespace tcas::orchestrator
