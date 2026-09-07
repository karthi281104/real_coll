#pragma once

#include "common/Types.hpp"
#include "conflict/Conflict.hpp"
#include "infrastructure/RailwayNetwork.hpp"
#include "prediction/FutureState.hpp"

#include <vector>

namespace tcas::conflict
{

class ConflictDetector
{
public:
    explicit ConflictDetector(ConflictDetectionConfig config = {});

    // Detects same-track, junction, and platform conflicts from predicted
    // trajectories and the existing railway topology.
    [[nodiscard]]
    std::vector<Conflict> detect(
        TrainId trainA,
        const std::vector<prediction::FutureState>& trajectoryA,
        TrainId trainB,
        const std::vector<prediction::FutureState>& trajectoryB,
        const infrastructure::RailwayNetwork& network
    ) const;

private:
    ConflictDetectionConfig config_;

    [[nodiscard]]
    ConflictType classifySameTrack(
        const prediction::FutureState& a0,
        const prediction::FutureState& a1,
        const prediction::FutureState& b0,
        const prediction::FutureState& b1
    ) const noexcept;

    [[nodiscard]]
    bool hasTemporalConflict(
        const prediction::FutureState& a0,
        const prediction::FutureState& a1,
        const prediction::FutureState& b0,
        const prediction::FutureState& b1,
        DistanceMeters& minimumSeparation,
        TimeSeconds& firstTime,
        TimeSeconds& lastTime
    ) const noexcept;
};

} // namespace tcas::conflict
