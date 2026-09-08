#pragma once

#include "conflict/ResourceReservation.hpp"

#include <vector>

namespace tcas::conflict
{

class ResourceReservationManager
{
public:
    [[nodiscard]]
    bool request(
        TrainId trainId,
        const ConflictZone& zone,
        TimeSeconds startTime,
        TimeSeconds endTime
    );

    [[nodiscard]]
    bool release(TrainId trainId, const ConflictZone& zone);

    void clearReleased();

    // Remove reservations whose endTime is before currentTime (train already cleared the zone).
    void clearExpired(TimeSeconds currentTime);

    [[nodiscard]]
    bool isReserved(
        const ConflictZone& zone,
        TimeSeconds startTime,
        TimeSeconds endTime,
        TrainId requestingTrain
    ) const;

    [[nodiscard]]
    const std::vector<ResourceReservation>& reservations() const noexcept;

private:
    std::vector<ResourceReservation> reservations_;

    [[nodiscard]]
    static bool sameZone(
        const ConflictZone& first,
        const ConflictZone& second
    ) noexcept;

    [[nodiscard]]
    static bool intervalsOverlap(
        TimeSeconds firstStart,
        TimeSeconds firstEnd,
        TimeSeconds secondStart,
        TimeSeconds secondEnd
    ) noexcept;
};

} // namespace tcas::conflict
