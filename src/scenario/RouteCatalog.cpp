#include "scenario/RouteCatalog.hpp"

#include "navigation/RouteNavigator.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <algorithm>
#include <memory>

namespace tcas::scenario
{

namespace
{

const std::vector<CatalogRouteInfo> kCatalogRoutes = {
    {
        1,
        "R-01",
        "South Shore Express (Central -> South Harbor)",
        TrainType::Express,
        1, 5,
        101, 100.0, 30.0,
        "Mainline to South Harbor via Alpha Jct. Disjoint path with R-02 [NO COLLISION]."
    },
    {
        2,
        "R-02",
        "North Corridor Commuter (Beta Jct -> North)",
        TrainType::Passenger,
        3, 4,
        103, 100.0, 25.0,
        "Beta to North Terminal commuter run. Disjoint path with R-01 [NO COLLISION]."
    },
    {
        3,
        "R-03",
        "Eastbound Mainline Express (Central -> Alpha Jct)",
        TrainType::Express,
        1, 2,
        101, 200.0, 30.0,
        "Eastbound on Track 101 towards Alpha Jct [HEAD-ON COLLISION with R-04]."
    },
    {
        4,
        "R-04",
        "Westbound Counter-Flow (Alpha Jct -> Central)",
        TrainType::Passenger,
        2, 1,
        201, 300.0, 30.0,
        "Westbound opposing run on single corridor Track 201 [HEAD-ON COLLISION with R-03]."
    },
    {
        5,
        "R-05",
        "Slow Heavy Freight Lead (Central -> North)",
        TrainType::Freight,
        1, 4,
        101, 700.0, 15.0,
        "Slow-moving freight ahead on Track 101 [REAR-END COLLISION with R-06]."
    },
    {
        6,
        "R-06",
        "High-Speed Overtaker (Central -> North)",
        TrainType::Express,
        1, 4,
        101, 100.0, 35.0,
        "Dispatched behind R-05 at 35 m/s on Track 101 [REAR-END COLLISION with R-05]."
    },
    {
        7,
        "R-07",
        "Approach Yard Freight (Freight Approach -> Yard)",
        TrainType::Freight,
        6, 7,
        105, 1500.0, 20.0,
        "Crosses Alpha Junction at t=25s [JUNCTION COLLISION with R-08]."
    },
    {
        8,
        "R-08",
        "Alpha Converging Passenger (Central -> North)",
        TrainType::Passenger,
        1, 4,
        101, 1375.0, 25.0,
        "Converges at Alpha Junction at t=25s [JUNCTION COLLISION with R-07]."
    },
    {
        9,
        "R-09",
        "South Harbor Platform Feeder (South Harbor -> Platform A)",
        TrainType::Passenger,
        5, 8,
        107, 100.0, 16.0,
        "Arrives at Platform A at t=25s [PLATFORM CONFLICT with R-10]."
    },
    {
        10,
        "R-10",
        "Beta Platform Feeder (Beta Jct -> Platform A)",
        TrainType::Express,
        3, 8,
        108, 200.0, 16.0,
        "Arrives at Platform A at t=25s [PLATFORM CONFLICT with R-09]."
    }
};

std::unique_ptr<train::Train> createTrainByType(
    TrainId id,
    TrainType type,
    SpeedMetersPerSecond speed,
    DistanceMeters initialPosition)
{
    std::unique_ptr<train::Train> t;
    switch (type)
    {
    case TrainType::Express:
        t = std::make_unique<train::ExpressTrain>(id, 45000.0, 45.0, 0.9, 1.4);
        break;
    case TrainType::Freight:
        t = std::make_unique<train::FreightTrain>(id, 120000.0, 22.2, 0.5, 0.8);
        break;
    case TrainType::Passenger:
    default:
        t = std::make_unique<train::PassengerTrain>(id, 60000.0, 33.3, 0.8, 1.2);
        break;
    }
    t->setPosition(initialPosition);
    t->setVelocity(speed);
    t->setAcceleration(0.0);
    t->setState(TrainState::Running);
    return t;
}

TrainId allocateNextTrainId(const train::TrainManager& manager, TrainId suggested)
{
    if (suggested != 0 && manager.getTrain(suggested) == nullptr)
    if (suggested != 0 && !manager.hasEverUsedId(suggested) && manager.getTrain(suggested) == nullptr)
    {
        return suggested;
    }
    TrainId candidate = 101;
    while (manager.getTrain(candidate) != nullptr)
    while (manager.hasEverUsedId(candidate) || manager.getTrain(candidate) != nullptr)
    {
        ++candidate;
    }
    return candidate;
}

} // namespace

const std::vector<CatalogRouteInfo>& RouteCatalog::allRoutes() noexcept
{
    return kCatalogRoutes;
}

const CatalogRouteInfo* RouteCatalog::findRoute(int catalogId) noexcept
{
    for (const auto& r : kCatalogRoutes)
    {
        if (r.catalogId == catalogId)
        {
            return &r;
        }
    }
    return nullptr;
}

DispatchedTrainResult RouteCatalog::dispatchCatalogRoute(
    int catalogId,
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager,
    TrainId suggestedTrainId)
{
    DispatchedTrainResult result;
    const auto* info = findRoute(catalogId);
    if (info == nullptr)
    {
        result.message = "Invalid Catalog ID " + std::to_string(catalogId);
        return result;
    }

    const auto route = navigation::RouteNavigator::findRoute(
        network, info->sourceNode, info->destinationNode);

    if (!route.success || route.tracks.empty())
    {
        result.message = "Dijkstra routing failed between node " +
            std::to_string(info->sourceNode) + " and " +
            std::to_string(info->destinationNode);
        return result;
    }

    const TrainId trainId = allocateNextTrainId(trainManager, suggestedTrainId);
    auto train = createTrainByType(
        trainId, info->defaultType, info->initialSpeed, info->initialPosition);

    if (!trainManager.addTrain(std::move(train)))
    {
        result.message = "Failed to register Train #" + std::to_string(trainId);
        return result;
    }

    const TrackId startTrack = (!route.tracks.empty()) ? route.tracks.front() : info->initialTrackId;
    result.trainId = trainId;
    result.trainRoute = { trainId, startTrack, route };
    result.routeName = info->name;
    result.success = true;
    result.message = "Dispatched " + info->code + " [" + info->name + "] as Train #" +
        std::to_string(trainId) + " (initial speed " +
        std::to_string(static_cast<int>(info->initialSpeed)) + " m/s).";
    return result;
}

DispatchedTrainResult RouteCatalog::dispatchCustomRoute(
    NodeId sourceNode,
    NodeId destinationNode,
    TrainType type,
    SpeedMetersPerSecond speed,
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager,
    TrainId suggestedTrainId)
{
    DispatchedTrainResult result;
    const auto route = navigation::RouteNavigator::findRoute(
        network, sourceNode, destinationNode);

    if (!route.success || route.tracks.empty())
    {
        result.message = "No topological path exists between Node " +
            std::to_string(sourceNode) + " and Node " +
            std::to_string(destinationNode);
        return result;
    }

    const TrainId trainId = allocateNextTrainId(trainManager, suggestedTrainId);
    auto train = createTrainByType(trainId, type, speed, 0.0);

    if (!trainManager.addTrain(std::move(train)))
    {
        result.message = "Failed to register Train #" + std::to_string(trainId);
        return result;
    }

    const TrackId startTrack = route.tracks.front();
    result.trainId = trainId;
    result.trainRoute = { trainId, startTrack, route };
    result.routeName = "Custom Route (Node " + std::to_string(sourceNode) +
        " -> Node " + std::to_string(destinationNode) + ")";
    result.success = true;
    result.message = "Dispatched custom route as Train #" + std::to_string(trainId) +
        " (speed: " + std::to_string(static_cast<int>(speed)) + " m/s).";
    return result;
}

} // namespace tcas::scenario

