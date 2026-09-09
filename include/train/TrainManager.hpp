#pragma once

#include "train/Train.hpp"

#include <cstddef>
#include <memory>
#include <shared_mutex>
#include <unordered_map>
#include <unordered_set>

namespace tcas::train
{

class TrainManager
{
public:
    TrainManager() = default;

    bool addTrain(std::unique_ptr<Train> train);

    bool removeTrain(TrainId id);

    [[nodiscard]]
    Train* getTrain(TrainId id) noexcept;

    [[nodiscard]]
    const Train* getTrain(TrainId id) const noexcept;

    [[nodiscard]]
    bool contains(TrainId id) const noexcept;

    [[nodiscard]]
    bool hasEverUsedId(TrainId id) const noexcept;

    [[nodiscard]]
    std::size_t trainCount() const noexcept;

    void clear() noexcept;

private:
    mutable std::shared_mutex mutex_;
    std::unordered_map<
        TrainId,
        std::unique_ptr<Train>
    > trains_;
    std::unordered_set<TrainId> usedIds_;
};

} // namespace tcas::train