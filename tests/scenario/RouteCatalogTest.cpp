#include "scenario/RouteCatalog.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "navigation/RouteNavigator.hpp"
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

    net.addTrack(Track(101, 1, 2, 2000.0, 35.0,  0.000));
    net.addTrack(Track(201, 2, 1, 2000.0, 35.0,  0.000));
    net.addTrack(Track(102, 2, 3, 1500.0, 30.0,  0.020));
    net.addTrack(Track(202, 3, 2, 1500.0, 30.0, -0.020));
    net.addTrack(Track(103, 3, 4, 2500.0, 40.0, -0.015));
    net.addTrack(Track(203, 4, 3, 2500.0, 40.0,  0.015));
    net.addTrack(Track(104, 2, 5, 3000.0, 25.0,  0.010));
    net.addTrack(Track(204, 5, 2, 3000.0, 25.0, -0.010));
    net.addTrack(Track(105, 6, 2, 2000.0, 25.0,  0.000));
    net.addTrack(Track(205, 2, 6, 2000.0, 25.0,  0.000));
    net.addTrack(Track(106, 2, 7, 1800.0, 25.0,  0.000));
    net.addTrack(Track(206, 7, 2, 1800.0, 25.0,  0.000));
    net.addTrack(Track(107, 5, 8,  500.0, 20.0,  0.000));
    net.addTrack(Track(207, 8, 5, 1000.0, 20.0,  0.000));
    net.addTrack(Track(108, 3, 8,  600.0, 20.0,  0.000));
    net.addTrack(Track(208, 8, 3, 1100.0, 20.0,  0.000));
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

    // Valid reverse path from 4 to 1
    const auto resReverse = RouteCatalog::dispatchCustomRoute(
        4, 1, TrainType::Passenger, 20.0, net, mgr);
    EXPECT_TRUE(resReverse.success);
    EXPECT_EQ(mgr.trainCount(), 2U);

    // Invalid non-existent node
    const auto resInvalid = RouteCatalog::dispatchCustomRoute(
        99, 1, TrainType::Passenger, 20.0, net, mgr);
    EXPECT_FALSE(resInvalid.success);

    // Invalid same node
    const auto resSame = RouteCatalog::dispatchCustomRoute(
        1, 1, TrainType::Passenger, 20.0, net, mgr);
    EXPECT_FALSE(resSame.success);
}

TEST(RouteCatalogTest, AllFiftySixSourceDestinationPairsAreRoutable)
{
    const auto net = buildTestNetwork();
    std::size_t successCount = 0;

    for (NodeId src = 1; src <= 8; ++src)
    {
        for (NodeId dst = 1; dst <= 8; ++dst)
        {
            if (src == dst) { continue; }
            const auto route = navigation::RouteNavigator::findRoute(net, src, dst);
            EXPECT_TRUE(route.success)
                << "Expected reachable path from Node " << src << " to Node " << dst;
            EXPECT_FALSE(route.tracks.empty());
            EXPECT_GT(route.totalDistance, 0.0);
            if (route.success && !route.tracks.empty())
            {
                ++successCount;
            }
        }
    }
    EXPECT_EQ(successCount, 56U);
}

TEST(RouteCatalogTest, DoesNotReuseTrainIdOfArrivedOrRemovedTrains)
{
    const auto net = buildTestNetwork();
    train::TrainManager mgr;

    // Dispatch Train 1 and 2
    const auto r1 = RouteCatalog::dispatchCatalogRoute(1, net, mgr);
    EXPECT_TRUE(r1.success);
    EXPECT_EQ(r1.trainId, 101U);

    const auto r2 = RouteCatalog::dispatchCatalogRoute(2, net, mgr);
    EXPECT_TRUE(r2.success);
    EXPECT_EQ(r2.trainId, 102U);

    // Train 101 arrives at destination and is removed from active fleet
    EXPECT_TRUE(mgr.removeTrain(101));
    EXPECT_EQ(mgr.getTrain(101), nullptr);

    // Dispatch another train (R-01 again)
    // It must NOT reuse Train ID 101!
    const auto r3 = RouteCatalog::dispatchCatalogRoute(1, net, mgr);
    EXPECT_TRUE(r3.success);
    EXPECT_EQ(r3.trainId, 103U);

    // Train 102 arrives at destination and is removed
    EXPECT_TRUE(mgr.removeTrain(102));
    EXPECT_EQ(mgr.getTrain(102), nullptr);

    // Dispatch custom route
    // It must NOT reuse 101 or 102!
    const auto r4 = RouteCatalog::dispatchCustomRoute(
        1, 4, TrainType::Passenger, 25.0, net, mgr);
    EXPECT_TRUE(r4.success);
    EXPECT_EQ(r4.trainId, 104U);

    // Even if someone explicitly suggests 101 (an arrived train's ID), it must not reuse it!
    const auto r5 = RouteCatalog::dispatchCatalogRoute(1, net, mgr, 101);
    EXPECT_TRUE(r5.success);
    EXPECT_NE(r5.trainId, 101U);
    EXPECT_EQ(r5.trainId, 105U);
}

} // namespace
} // namespace tcas::scenario

