#pragma once

#include "conflict/ResourceReservationManager.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteResult.hpp"
#include "orchestrator/ThreadOrchestrator.hpp"
#include "train/TrainManager.hpp"

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace tcas::orchestrator
{

// Encapsulates the full Modules 8-10 safety pipeline as a SafetyStep callable.
//
// Wire-up order per cycle:
//   PredictionEngine (8) -> ConflictDetector (9) -> RiskEngine (10)
//   -> PriorityEngine (10) -> ConflictPriorityQueue (10)
//   -> ResourceReservationManager (9) -> ResolutionEngine (10)
//
// Owns a persistent ResourceReservationManager so reservation state
// is maintained across consecutive safety cycles.
class SafetyPipeline
{
public:
    struct TrainRoute
    {
        TrainId trainId;
        TrackId currentTrackId;
        navigation::RouteResult route;
    };

    // network and trainManager must outlive this object.
    SafetyPipeline(
        const infrastructure::RailwayNetwork& network,
        const train::TrainManager& trainManager,
        std::vector<TrainRoute> routes
    );

    // Update the route table (call when a train's route changes, or a new
    // train is added/removed dynamically).
    void setRoutes(std::vector<TrainRoute> routes);
    void addOrUpdateRoute(TrainRoute route);
    void removeRoute(TrainId trainId);

    // Returns a SafetyStep callable suitable for
    // ThreadOrchestrator::setSafetyStep().
    SafetyStep makeStep();

private:
    SafetyCycleResult run(const WorldState& state);

    const infrastructure::RailwayNetwork& network_;
    const train::TrainManager& trainManager_;
    mutable std::mutex routesMutex_;
    std::vector<TrainRoute> routes_;
    conflict::ResourceReservationManager reservations_;
    std::unordered_map<std::uint64_t, TrainId> stickyPriorities_;
};

} // namespace tcas::orchestrator
