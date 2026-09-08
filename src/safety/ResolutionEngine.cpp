#include "safety/ResolutionEngine.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::safety
{

SafetyCommand ResolutionEngine::resolve(
    const ResolutionInput& input
) const noexcept
{
    // Safety has absolute precedence over operational priority.

    if (input.risk.isCritical())
    {
        return emergencyBrake(input);
    }

    if (!input.brakingFeasible)
    {
        return emergencyBrake(input);
    }

    if (!calculateBrakingFeasibility(input))
    {
        return emergencyBrake(input);
    }

    // Non-priority train must yield to the priority train.
    if (!input.priorityGranted)
    {
        switch (input.risk.level)
        {
        case RiskLevel::Critical:
            return emergencyBrake(input);

        case RiskLevel::High:
            return holdAtSignal(input);

        case RiskLevel::Medium:
            return reduceSpeed(input);

        case RiskLevel::Low:
            return noAction(input);
        }
    }

    // Priority does not mean unrestricted operation.
    // High and medium risk still require speed reduction.
    switch (input.risk.level)
    {
    case RiskLevel::Critical:
        return emergencyBrake(input);

    case RiskLevel::High:
        return reduceSpeed(input);

    case RiskLevel::Medium:
        return reduceSpeed(input);

    case RiskLevel::Low:
        return noAction(input);
    }

    return emergencyBrake(input);
}

bool ResolutionEngine::calculateBrakingFeasibility(
    const ResolutionInput& input
) noexcept
{
    if (!std::isfinite(input.availableDistance)
        || !std::isfinite(input.requiredBrakingDistance)
        || !std::isfinite(input.risk.safetyMargin))
    {
        return false;
    }

    if (input.availableDistance < 0.0
        || input.requiredBrakingDistance < 0.0)
    {
        return false;
    }

    // A negative margin means the train has already consumed
    // the available safety clearance. The safety margin used
    // for feasibility is therefore never allowed to reduce
    // the stopping-distance requirement.
    const DistanceMeters requiredDistance =
        input.requiredBrakingDistance
        + std::max(0.0, input.risk.safetyMargin);

    return input.availableDistance >= requiredDistance;
}

SafetyCommand ResolutionEngine::emergencyBrake(
    const ResolutionInput& input
) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::EmergencyBrake;
    command.trainId = input.train.id();
    command.targetSpeed = 0.0;
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

SafetyCommand ResolutionEngine::reduceSpeed(
    const ResolutionInput& input
) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::ReduceSpeed;
    command.trainId = input.train.id();
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    const SpeedMetersPerSecond currentSpeed =
        std::max(0.0, input.train.velocity());

    // Halve the current speed as the target reduction.
    // Apply a minimum floor: if the halved target would drop below
    // kMinReduceSpeedFloor m/s, escalate to HoldAtSignal instead so the
    // train fully stops rather than crawling asymptotically near zero.
    constexpr SpeedMetersPerSecond kMinReduceSpeedFloor = 3.0;
    const SpeedMetersPerSecond halved = currentSpeed * 0.5;

    if (halved < kMinReduceSpeedFloor)
    {
        // Escalate: train is already too slow to meaningfully reduce speed,
        // issue a hold so it completes the braking cleanly.
        command.type = SafetyCommandType::HoldAtSignal;
        command.targetSpeed = 0.0;
        return command;
    }

    command.targetSpeed = halved;
    return command;
}


SafetyCommand ResolutionEngine::holdAtSignal(
    const ResolutionInput& input
) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::HoldAtSignal;
    command.trainId = input.train.id();
    command.targetSpeed = 0.0;
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

SafetyCommand ResolutionEngine::noAction(
    const ResolutionInput& input
) noexcept
{
    SafetyCommand command;

    command.type = SafetyCommandType::NoAction;
    command.trainId = input.train.id();
    command.targetSpeed =
        std::max(0.0, input.train.velocity());
    command.issuedAt = input.conflict.firstConflictTime;
    command.riskScore = input.risk.score;

    return command;
}

} // namespace tcas::safety