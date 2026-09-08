#pragma once

#include "common/Types.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteResult.hpp"
#include "orchestrator/SafetyPipeline.hpp"
#include "train/Train.hpp"
#include "train/TrainManager.hpp"

#include <string>
#include <vector>

namespace tcas::scenario
{

struct CatalogRouteInfo
{
    int catalogId{ 0 };
    std::string code;
    std::string name;
    TrainType defaultType{ TrainType::Passenger };
    NodeId sourceNode{ 0 };
    NodeId destinationNode{ 0 };
    TrackId initialTrackId{ 0 };
    DistanceMeters initialPosition{ 0.0 };
    SpeedMetersPerSecond initialSpeed{ 20.0 };
    std::string conflictNotice;
};

struct DispatchedTrainResult
{
    TrainId trainId{ 0 };
    orchestrator::SafetyPipeline::TrainRoute trainRoute;
    std::string routeName;
    bool success{ false };
    std::string message;
};

class RouteCatalog
{
public:
    // Returns the static list of 10 pre-defined operational catalog routes.
    [[nodiscard]]
    static const std::vector<CatalogRouteInfo>& allRoutes() noexcept;

    // Looks up a catalog route by catalog ID (1..10). Returns nullptr if invalid.
    [[nodiscard]]
    static const CatalogRouteInfo* findRoute(int catalogId) noexcept;

    // Dispatches a train based on a catalog route into the train manager and network.
    // If suggestedTrainId is 0, an unused ID is allocated automatically.
    [[nodiscard]]
    static DispatchedTrainResult dispatchCatalogRoute(
        int catalogId,
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        TrainId suggestedTrainId = 0
    );

    // Dispatches a custom route by running Dijkstra between sourceNode and destinationNode.
    [[nodiscard]]
    static DispatchedTrainResult dispatchCustomRoute(
        NodeId sourceNode,
        NodeId destinationNode,
        TrainType type,
        SpeedMetersPerSecond speed,
        const infrastructure::RailwayNetwork& network,
        train::TrainManager& trainManager,
        TrainId suggestedTrainId = 0
    );
};

} // namespace tcas::scenario

