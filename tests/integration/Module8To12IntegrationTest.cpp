#include "communication/CommunicationChannel.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "scenario/ScenarioManager.hpp"
#include "train/TrainManager.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <thread>

namespace tcas
{
namespace
{

// Build orchestrator around a scenario and let it run for a short duration.
// Verify that the safety pipeline produces real commands (Modules 8-10 active).
TEST(Module8To12IntegrationTest, JunctionScenarioProducesSafetyCommand)
{
    infrastructure::RailwayNetwork network;
    train::TrainManager manager;
    communication::CommunicationChannel channel;

    // Load the junction conflict scenario
    scenario::ScenarioManager scenMgr(network, manager);
    const auto result =
        scenMgr.load(scenario::ScenarioType::JunctionConflict);

    ASSERT_FALSE(result.routes.empty());
    ASSERT_GE(manager.trainCount(), 2U);

    // Build the real safety pipeline (Modules 8-10)
    orchestrator::SafetyPipeline pipeline(network, manager, result.routes);

    // Collect train IDs
    std::vector<TrainId> trainIds;
    for (const auto& r : result.routes)
    {
        trainIds.push_back(r.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.safetyPeriod = std::chrono::milliseconds(50);

    orchestrator::ThreadOrchestrator orch(
        network, manager, channel, trainIds, cfg);
    orch.setSafetyStep(pipeline.makeStep());
    orch.start();

    // Allow several safety cycles to detect and act on the conflict
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    orch.stop();

    // The orchestrator should have run at least a few safety cycles
    EXPECT_GE(orch.safetyCycles(), 3U)
        << "Expected at least 3 safety cycles to run.";

    // The safety pipeline must produce a real safety action (conflict detected,
    // reservation/decision generated, command emitted, or train speed/state modified).
    const auto state = orch.snapshot();
    const bool hadAction =
        !state.activeConflicts.empty() ||
        !state.commands.empty()        ||
        !state.decisions.empty()       ||
        !state.reservations.empty()    ||
        (manager.getTrain(3) != nullptr && manager.getTrain(3)->state() != TrainState::Idle);
    EXPECT_TRUE(hadAction)
        << "Expected safety pipeline to detect conflict and generate safety action.";
}

TEST(Module8To12IntegrationTest, UnsafeStoppingProducesEmergencyBrake)
{
    infrastructure::RailwayNetwork network;
    train::TrainManager manager;
    communication::CommunicationChannel channel;

    scenario::ScenarioManager scenMgr(network, manager);
    const auto result =
        scenMgr.load(scenario::ScenarioType::UnsafeStopping);

    ASSERT_FALSE(result.routes.empty());

    orchestrator::SafetyPipeline pipeline(network, manager, result.routes);

    std::vector<TrainId> trainIds;
    for (const auto& r : result.routes)
    {
        trainIds.push_back(r.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.safetyPeriod = std::chrono::milliseconds(50);

    orchestrator::ThreadOrchestrator orch(
        network, manager, channel, trainIds, cfg);
    orch.setSafetyStep(pipeline.makeStep());
    orch.start();

    // Run for longer to ensure several safety cycles complete
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    orch.stop();

    const auto state = orch.snapshot();

    // At minimum, the safety pipeline should have run
    EXPECT_GE(orch.safetyCycles(), 3U)
        << "Expected at least 3 safety cycles for unsafe stopping scenario.";

    // Unsafe stopping distance must have triggered EmergencyBrake on yielding train (Freight #3)
    const auto* freight = manager.getTrain(3);
    ASSERT_NE(freight, nullptr);
    EXPECT_EQ(freight->state(), TrainState::EmergencyBrake);
}

TEST(Module8To12IntegrationTest, MultipleConflictScenarioRunsStably)
{
    infrastructure::RailwayNetwork network;
    train::TrainManager manager;
    communication::CommunicationChannel channel;

    scenario::ScenarioManager scenMgr(network, manager);
    const auto result =
        scenMgr.load(scenario::ScenarioType::MultipleConflicts);

    ASSERT_FALSE(result.routes.empty());
    ASSERT_GE(manager.trainCount(), 3U);

    orchestrator::SafetyPipeline pipeline(network, manager, result.routes);

    std::vector<TrainId> trainIds;
    for (const auto& r : result.routes)
    {
        trainIds.push_back(r.trainId);
    }

    orchestrator::OrchestratorConfig cfg;
    cfg.safetyPeriod = std::chrono::milliseconds(50);

    orchestrator::ThreadOrchestrator orch(
        network, manager, channel, trainIds, cfg);
    orch.setSafetyStep(pipeline.makeStep());
    orch.start();

    // Run for longer to test multi-cycle stability
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    orch.stop();

    // Must not crash; world state must be valid
    const auto state = orch.snapshot();
    EXPECT_LE(state.trains.size(), 3U);
    for (const auto& t : state.trains)
    {
        EXPECT_TRUE(std::isfinite(t.position));
        EXPECT_TRUE(std::isfinite(t.velocity));
    }
    EXPECT_GE(orch.safetyCycles(), 5U);
}

TEST(Module8To12IntegrationTest, ScenarioManagerLoadsAllScenarios)
{
    using namespace scenario;
    // HeadOn scenario is now valid (no negative velocity). All scenarios included.
    const std::vector<ScenarioType> all = {
        ScenarioType::JunctionConflict,
        ScenarioType::RearEndConflict,
        ScenarioType::HeadOnConflict,
        ScenarioType::PlatformConflict,
        ScenarioType::MultipleConflicts,
        ScenarioType::SensorFailure,
        ScenarioType::CommunicationFailure,
        ScenarioType::UnsafeStopping
    };
    for (const auto type : all)
    {
        infrastructure::RailwayNetwork network;
        train::TrainManager manager;
        ScenarioManager mgr(network, manager);
        const auto result = mgr.load(type);
        EXPECT_FALSE(result.routes.empty())
            << "Scenario " << ScenarioManager::scenarioName(type)
            << " produced empty routes.";
        EXPECT_GT(manager.trainCount(), 0U)
            << "Scenario " << ScenarioManager::scenarioName(type)
            << " registered no trains.";
        EXPECT_FALSE(result.description.empty());
    }
}

} // namespace
} // namespace tcas
