#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"
#include "train/TrainManager.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace tcas;
using namespace tcas::train;

TEST(TrainManagerTest, StartsEmpty)
{
    TrainManager manager;

    EXPECT_EQ(manager.trainCount(), 0);
}

TEST(TrainManagerTest, AddsTrain)
{
    TrainManager manager;

    auto train = std::make_unique<ExpressTrain>(
        1,
        50000.0,
        45.0,
        0.8,
        1.2
    );

    EXPECT_TRUE(manager.addTrain(std::move(train)));
    EXPECT_EQ(manager.trainCount(), 1);
}

TEST(TrainManagerTest, FindsTrain)
{
    TrainManager manager;

    manager.addTrain(
        std::make_unique<ExpressTrain>(
            1,
            50000.0,
            45.0,
            0.8,
            1.2
        )
    );

    Train* train = manager.getTrain(1);

    ASSERT_NE(train, nullptr);
    EXPECT_EQ(train->id(), 1);
    EXPECT_EQ(train->type(), TrainType::Express);
}

TEST(TrainManagerTest, MissingTrainReturnsNull)
{
    TrainManager manager;

    EXPECT_EQ(
        manager.getTrain(999),
        nullptr
    );
}

TEST(TrainManagerTest, RejectsDuplicateTrainId)
{
    TrainManager manager;

    EXPECT_TRUE(
        manager.addTrain(
            std::make_unique<ExpressTrain>(
                1,
                50000.0,
                45.0,
                0.8,
                1.2
            )
        )
    );

    EXPECT_FALSE(
        manager.addTrain(
            std::make_unique<FreightTrain>(
                1,
                500000.0,
                25.0,
                0.5,
                0.8
            )
        )
    );

    EXPECT_EQ(manager.trainCount(), 1);
}

TEST(TrainManagerTest, RejectsNullTrain)
{
    TrainManager manager;

    std::unique_ptr<Train> train;

    EXPECT_FALSE(
        manager.addTrain(std::move(train))
    );

    EXPECT_EQ(manager.trainCount(), 0);
}

TEST(TrainManagerTest, SupportsMultipleTrainTypes)
{
    TrainManager manager;

    EXPECT_TRUE(
        manager.addTrain(
            std::make_unique<ExpressTrain>(
                1,
                50000.0,
                45.0,
                0.8,
                1.2
            )
        )
    );

    EXPECT_TRUE(
        manager.addTrain(
            std::make_unique<PassengerTrain>(
                2,
                60000.0,
                35.0,
                0.7,
                1.1
            )
        )
    );

    EXPECT_TRUE(
        manager.addTrain(
            std::make_unique<FreightTrain>(
                3,
                500000.0,
                25.0,
                0.5,
                0.8
            )
        )
    );

    EXPECT_EQ(manager.trainCount(), 3);

    EXPECT_EQ(
        manager.getTrain(1)->type(),
        TrainType::Express
    );

    EXPECT_EQ(
        manager.getTrain(2)->type(),
        TrainType::Passenger
    );

    EXPECT_EQ(
        manager.getTrain(3)->type(),
        TrainType::Freight
    );
}

TEST(TrainManagerTest, RemovesTrain)
{
    TrainManager manager;

    manager.addTrain(
        std::make_unique<ExpressTrain>(
            1,
            50000.0,
            45.0,
            0.8,
            1.2
        )
    );

    EXPECT_TRUE(manager.removeTrain(1));
    EXPECT_EQ(manager.trainCount(), 0);
    EXPECT_EQ(manager.getTrain(1), nullptr);
}

TEST(TrainManagerTest, RemovingMissingTrainReturnsFalse)
{
    TrainManager manager;

    EXPECT_FALSE(
        manager.removeTrain(999)
    );
}

TEST(TrainManagerTest, ContainsTrain)
{
    TrainManager manager;

    manager.addTrain(
        std::make_unique<PassengerTrain>(
            2,
            60000.0,
            35.0,
            0.7,
            1.1
        )
    );

    EXPECT_TRUE(manager.contains(2));
    EXPECT_FALSE(manager.contains(999));
}

TEST(TrainManagerTest, ClearRemovesAllTrains)
{
    TrainManager manager;

    manager.addTrain(
        std::make_unique<ExpressTrain>(
            1,
            50000.0,
            45.0,
            0.8,
            1.2
        )
    );

    manager.addTrain(
        std::make_unique<FreightTrain>(
            2,
            500000.0,
            25.0,
            0.5,
            0.8
        )
    );

    manager.clear();

    EXPECT_EQ(manager.trainCount(), 0);
}

TEST(TrainManagerTest, TracksUsedIdsEvenAfterRemoval)
{
    TrainManager manager;

    EXPECT_FALSE(manager.hasEverUsedId(101));

    manager.addTrain(
        std::make_unique<ExpressTrain>(
            101,
            50000.0,
            45.0,
            0.8,
            1.2
        )
    );

    EXPECT_TRUE(manager.hasEverUsedId(101));
    EXPECT_TRUE(manager.contains(101));

    // Remove train (simulating arrival / completion)
    EXPECT_TRUE(manager.removeTrain(101));
    EXPECT_FALSE(manager.contains(101));

    // Must still remember that 101 was used!
    EXPECT_TRUE(manager.hasEverUsedId(101));

    // Clear resets history for new session
    manager.clear();
    EXPECT_FALSE(manager.hasEverUsedId(101));
}