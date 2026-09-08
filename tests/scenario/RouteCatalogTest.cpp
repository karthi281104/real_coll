#include "scenario/RouteCatalog.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "train/TrainManager.hpp"

#include <gtest/gtest.h>

namespace tcas::scenario
{
namespace
{

infrastructure::RailwayNetwork buildTestNetwork()
{
    using namespace tcas::infrastructure;
    RailwayNetwork net;
    net.addNode(Node(1, "Central Station",   NodeType::Station));
    net.addNode(Node(2, "Alpha Junction",    NodeType::Junction));
    net.addNode(Node(3, "Beta Junction",     NodeType::Junction));
    net.addNode(Node(4, "North Terminal",    NodeType::Station));
    net.addNode(Node(5, "South Harbor",      NodeType::Station));
    net.addNode(Node(6, "Freight Approach",  NodeType::Generic));
    net.addNode(Node(7, "Freight Yard",      NodeType::Generic));
    net.addNode(Node(8, "Platform A",        NodeType::Platform));

    net.addTrack(Track(101, 1, 2, 2000.0, 35.0, 0.000));
    net.addTrack(Track(102, 2, 3, 1500.0, 30.0, 0.020));
    net.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));
    net.addTrack(Track(104, 2, 5, 3000.0, 25.0, 0.010));
    net.addTrack(Track(105, 6, 2, 2000.0, 25.0, 0.000));
    net.addTrack(Track(106, 2, 7, 1800.0, 25.0, 0.000));
    net.addTrack(Track(107, 5, 8,  500.0, 20.0, 0.000));
    net.addTrack(Track(108, 3, 8,  600.0, 20.0, 0.000));
    return net;
}

TEST(RouteCatalogTest, ReturnsTenCatalogRoutes)
{
    const auto& routes = RouteCatalog::allRoutes();
    EXPECT_EQ(routes.size(), 10U);
    for (std::size_t i = 0; i < routes.size(); ++i)
    {
        EXPECT_EQ(routes[i].catalogId, static_cast<int>(i + 1));
        EXPECT_FALSE(routes[i].code.empty());
        EXPECT_FALSE(routes[i].name.empty());
        EXPECT_GT(routes[i].initialSpeed, 0.0);
    }
}

TEST(RouteCatalogTest, FindRouteLookup)
{
    const auto* r1 = RouteCatalog::findRoute(1);
    ASSERT_NE(r1, nullptr);
    EXPECT_EQ(r1->code, "R-01");

    const auto* rInvalid = RouteCatalog::findRoute(99);
    EXPECT_EQ(rInvalid, nullptr);
}

TEST(RouteCatalogTest, DispatchCatalogRoutesSuccessfully)
{
    const auto net = buildTestNetwork();
    train::TrainManager mgr;

    for (int id = 1; id <= 10; ++id)
    {
        const auto result = RouteCatalog::dispatchCatalogRoute(id, net, mgr);
        EXPECT_TRUE(result.success) << "Failed to dispatch route " << id << ": " << result.message;
        EXPECT_GT(result.trainId, 0U);
        EXPECT_FALSE(result.trainRoute.route.tracks.empty());
        EXPECT_GT(result.trainRoute.route.totalDistance, 0.0);
    }
    EXPECT_EQ(mgr.trainCount(), 10U);
}

TEST(RouteCatalogTest, CustomRouteDispatch)
{
    const auto net = buildTestNetwork();
    train::TrainManager mgr;

    // Valid path from 1 to 4
    const auto resValid = RouteCatalog::dispatchCustomRoute(
        1, 4, TrainType::Express, 30.0, net, mgr);
    EXPECT_TRUE(resValid.success);
    EXPECT_EQ(mgr.trainCount(), 1U);

    // Invalid path where no directed track connects (e.g. 4 to 1)
    const auto resInvalid = RouteCatalog::dispatchCustomRoute(
        4, 1, TrainType::Passenger, 20.0, net, mgr);
    EXPECT_FALSE(resInvalid.success);
}

} // namespace
} // namespace tcas::scenario

