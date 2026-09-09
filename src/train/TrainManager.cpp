#include "train/TrainManager.hpp"
#include <mutex>

namespace tcas::train
{

bool TrainManager::addTrain(
    std::unique_ptr<Train> train
)
{
    if (!train)
    {
        return false;
    }

    const TrainId id = train->id();
    std::unique_lock lock(mutex_);

    const auto [iterator, inserted] =
        trains_.emplace(id, std::move(train));

    if (inserted)
    {
        usedIds_.insert(id);
    }

    return inserted;
}

bool TrainManager::removeTrain(TrainId id)
{
    std::unique_lock lock(mutex_);
    return trains_.erase(id) > 0;
}

Train* TrainManager::getTrain(TrainId id) noexcept
{
    std::shared_lock lock(mutex_);
    const auto iterator = trains_.find(id);

    if (iterator == trains_.end())
    {
        return nullptr;
    }

    return iterator->second.get();
}

const Train* TrainManager::getTrain(
    TrainId id
) const noexcept
{
    std::shared_lock lock(mutex_);
    const auto iterator = trains_.find(id);

    if (iterator == trains_.end())
    {
        return nullptr;
    }

    return iterator->second.get();
}

bool TrainManager::contains(TrainId id) const noexcept
{
    std::shared_lock lock(mutex_);
    return trains_.contains(id);
}

bool TrainManager::hasEverUsedId(TrainId id) const noexcept
{
    std::shared_lock lock(mutex_);
    return usedIds_.contains(id);
}

std::size_t TrainManager::trainCount() const noexcept
{
    std::shared_lock lock(mutex_);
    return trains_.size();
}

void TrainManager::clear() noexcept
{
    std::unique_lock lock(mutex_);
    trains_.clear();
    usedIds_.clear();
}

} // namespace tcas::train