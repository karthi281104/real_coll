#pragma once

#include <cstdint>

namespace tcas
{

// ============================================================
// Infrastructure identifiers
// ============================================================

using NodeId = std::uint32_t;
using TrackId = std::uint32_t;
using StationId = std::uint32_t;
using JunctionId = std::uint32_t;
using PlatformId = std::uint32_t;

// ============================================================
// Train identifier
// ============================================================

using TrainId = std::uint32_t;

// ============================================================
// Physical quantities
// ============================================================

using TimeSeconds = double;
using DistanceMeters = double;
using SpeedMetersPerSecond = double;
using AccelerationMetersPerSecondSquared = double;
using SimTimeTick = std::uint64_t;

// ============================================================
// Train classification
// ============================================================

enum class TrainType
{
    Express,
    Passenger,
    Freight
};

// ============================================================
// Train operational state
// ============================================================

enum class TrainState
{
    Idle,
    Running,
    Slowing,        ///< ReduceSpeed safety command — service braking to target speed
    Braking,        ///< Hard service braking — approaching HoldAtSignal
    Stopped,        ///< HoldAtSignal — stationary at signal
    EmergencyBrake, ///< True emergency stop — requires operator reset
    Completed       ///< Train reached end of route
};

} // namespace tcas