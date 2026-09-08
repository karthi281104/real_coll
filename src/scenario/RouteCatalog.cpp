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
        "InterCity North Express (Central -> North)",
        TrainType::Express,
        1, 4,
        101, 100.0, 32.0,
        "Mainline fast service. Converges with R-02 at Alpha Jct (Node 2)."
    },
    {
        2,
        "R-02",
        "Heavy Yard Freight (Approach -> Freight Yard)",
        TrainType::Freight,
        6, 7,
        105, 200.0, 18.0,
        105, 750.0, 20.0,
        "Heavy cargo. Converges across Alpha Junction (Node 2) with R-01 [JUNCTION CONFLICT]."
    },
    {
        3,
        "R-03",
        "Southbound Local (Central -> South Harbor)",
        TrainType::Passenger,
        1, 5,
        101, 400.0, 20.0,
        "Shares Track 101 with R-01/R-04 [SAME-TRACK SPACING / REAR-END CONFLICT]."
    },
    {
        4,
        "R-04",
        "High-Speed Overtaker (Central -> North)",
        TrainType::Express,
        1, 4,
        101, 50.0, 38.0,
        "Dispatched behind preceding trains at high speed [RAPID REAR-END CLOSING]."
    },
    {
        5,
        "R-05",
        "Island Shuttle Alpha (South Harbor -> Platform A)",
        TrainType::Passenger,
        5, 8,
        107, 50.0, 16.0,
        "Approaches Platform A from South. Converges with R-06 [PLATFORM CONFLICT]."
    },
    {
        6,
        "R-06",
        "Island Shuttle Beta (Beta Jct -> Platform A)",
        TrainType::Passenger,
        3, 8,
        108, 50.0, 16.0,
        "Approaches Platform A from North. Converges with R-05 [PLATFORM CONFLICT]."
    },
    {
        7,
        "R-07",
        "Cross-Country Freight (Approach -> South Harbor)",
        TrainType::Freight,
        6, 5,
        105, 100.0, 17.0,
        "Industrial connection crossing Alpha Junction towards South Harbor."
    },
    {
        8,
        "R-08",
        "North Corridor Commuter (Alpha Jct -> North)",
        TrainType::Passenger,
        2, 4,
        102, 100.0, 26.0,
        "Intermediate mainline commuter run via Beta Junction."
    },
    {
        9,
        "R-09",
        "Yard Shunting Transfer (Alpha Jct -> Freight Yard)",
        TrainType::Freight,
        2, 7,
        106, 50.0, 14.0,
        "Low-speed yard shunting run on dedicated industrial spur."
    },
    {
        10,
        "R-10",
        "South Coastal Express (Alpha Jct -> South Harbor)",
        TrainType::Express,
        2, 5,
        104, 100.0, 28.0,
        "Regional branch express to the coastal passenger terminal."
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
    {
        return suggested;
    }
    TrainId candidate = 101;
    while (manager.getTrain(candidate) != nullptr)
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

