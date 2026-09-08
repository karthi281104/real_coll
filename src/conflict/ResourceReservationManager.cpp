#include "conflict/ResourceReservationManager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace tcas::conflict
{

bool ResourceReservationManager::request(
    TrainId trainId,
    const ConflictZone& zone,
    TimeSeconds startTime,
    TimeSeconds endTime
)
{
    if (trainId == 0 || startTime < 0.0 || endTime <= startTime ||
        !std::isfinite(startTime) || !std::isfinite(endTime))
    {
        throw std::invalid_argument("Invalid reservation interval");
    }

    if (isReserved(zone, startTime, endTime, trainId))
    {
        return false;
    }

    reservations_.push_back({
        trainId,
        zone,
        startTime,
        endTime,
        ResourceState::Reserved
    });
    return true;
}

bool ResourceReservationManager::release(
    TrainId trainId,
    const ConflictZone& zone
)
{
    const auto iterator = std::find_if(
        reservations_.begin(), reservations_.end(),
        [&](const ResourceReservation& reservation)
        {
            return reservation.trainId == trainId &&
                   sameZone(reservation.zone, zone) &&
                   reservation.state == ResourceState::Reserved;
        });

    if (iterator == reservations_.end())
    {
        return false;
    }

    iterator->state = ResourceState::Released;
    return true;
}

void ResourceReservationManager::clearReleased()
{
    reservations_.erase(
        std::remove_if(
            reservations_.begin(), reservations_.end(),
            [](const ResourceReservation& reservation)
            {
                return reservation.state == ResourceState::Released;
            }),
        reservations_.end());
}

void ResourceReservationManager::clearExpired(TimeSeconds currentTime)
{
    reservations_.erase(
        std::remove_if(
            reservations_.begin(), reservations_.end(),
            [currentTime](const ResourceReservation& reservation)
            {
                // A reservation expires when the simulation clock passes its end window.
                return reservation.endTime <= currentTime;
            }),
        reservations_.end());
}

bool ResourceReservationManager::isReserved(
    const ConflictZone& zone,
    TimeSeconds startTime,
    TimeSeconds endTime,
    TrainId requestingTrain
) const
{
    return std::any_of(
        reservations_.begin(), reservations_.end(),
        [&](const ResourceReservation& reservation)
        {
            return reservation.state == ResourceState::Reserved &&
                   reservation.trainId != requestingTrain &&
                   sameZone(reservation.zone, zone) &&
                   intervalsOverlap(
                       startTime,
                       endTime,
                       reservation.startTime,
                       reservation.endTime);
        });
}

const std::vector<ResourceReservation>&
ResourceReservationManager::reservations() const noexcept
{
    return reservations_;
}

bool ResourceReservationManager::sameZone(
    const ConflictZone& first,
    const ConflictZone& second
) noexcept
{
    return first.type == second.type &&
           first.nodeId == second.nodeId &&
           first.trackId == second.trackId;
}

bool ResourceReservationManager::intervalsOverlap(
    TimeSeconds firstStart,
    TimeSeconds firstEnd,
    TimeSeconds secondStart,
    TimeSeconds secondEnd
) noexcept
{
    return firstStart < secondEnd && secondStart < firstEnd;
}

} // namespace tcas::conflict
