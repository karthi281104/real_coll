#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{

void requireFinite(const double value, const char* const name)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(std::string{name} + " must be finite");
    }
}

} // namespace

namespace tcas::physics
{

double KinematicsEngine::updatePosition(
    double position,
    double velocity,
    double acceleration,
    double dt
)
{
    requireFinite(position, "position");
    requireFinite(velocity, "velocity");
    requireFinite(acceleration, "acceleration");
    requireFinite(dt, "dt");
    if (dt <= 0.0)
    {
        return position;
    }

    if (velocity < 0.0)
    {
        velocity = 0.0;
    }

    // No motion if zero velocity and no positive acceleration.
    if (velocity == 0.0 && acceleration <= 0.0)
    {
        return position;
    }

    // If the train is decelerating, check whether it stops
    // during this timestep.
    if (acceleration < 0.0)
    {
        const double stoppingTime = -velocity / acceleration;

        if (stoppingTime <= dt)
        {
            // Train reaches zero velocity before the timestep ends.
            // Move only until it stops.
            return position
                + velocity * stoppingTime
                + 0.5 * acceleration * stoppingTime * stoppingTime;
        }
    }

    // Normal kinematic update.
    return position
        + velocity * dt
        + 0.5 * acceleration * dt * dt;
}

SpeedMetersPerSecond KinematicsEngine::updateVelocity(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared acceleration,
    const TimeSeconds dt,
    const SpeedMetersPerSecond maximumSpeed
)
{
    requireFinite(velocity, "velocity");
    requireFinite(acceleration, "acceleration");
    requireFinite(dt, "dt");
    requireFinite(maximumSpeed, "maximumSpeed");

    if (maximumSpeed < 0.0)
    {
        throw std::invalid_argument("maximumSpeed must not be negative");
    }
    if (dt <= 0.0)
    {
        return velocity;
    }

    const double updated = velocity + acceleration * dt;
    return std::clamp(updated, 0.0, maximumSpeed);
}

AccelerationMetersPerSecondSquared KinematicsEngine::effectiveDeceleration(
    const AccelerationMetersPerSecondSquared nominalDeceleration,
    const double gradient
)
{
    requireFinite(nominalDeceleration, "nominalDeceleration");
    requireFinite(gradient, "gradient");
    // Positive gradient (uphill): gravity component assists braking.
    // Negative gradient (downhill): gravity component opposes braking.
    const double gradientCorrection = kGravity * gradient;
    const double effective = nominalDeceleration + gradientCorrection;

    // Clamp to minimum to prevent runaway (zero or negative deceleration).
    return std::max(effective, kMinDeceleration);
}

DistanceMeters KinematicsEngine::brakingDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared effectiveDecel
)
{
    requireFinite(velocity, "velocity");
    requireFinite(effectiveDecel, "effectiveDecel");
    if (velocity <= 0.0)
    {
        return 0.0;
    }

    const double decel = std::max(effectiveDecel, kMinDeceleration);
    return (velocity * velocity) / (2.0 * decel);
}

double KinematicsEngine::brakingDistance(
    double velocity,
    double deceleration,
    double gradient
)
{
    requireFinite(velocity, "velocity");
    requireFinite(deceleration, "nominalDeceleration");
    requireFinite(gradient, "gradient");
    if (velocity <= 0.0)
    {
        return 0.0;
    }

    const double effective =
        effectiveDeceleration(deceleration, gradient);

    if (effective <= 0.0)
    {
        return 0.0;
    }

    return (velocity * velocity) / (2.0 * effective);
}

DistanceMeters KinematicsEngine::reactionDistance(
    const SpeedMetersPerSecond velocity,
    const TimeSeconds reactionTime
)
{
    requireFinite(velocity, "velocity");
    requireFinite(reactionTime, "reactionTime");
    if (velocity <= 0.0 || reactionTime <= 0.0)
    {
        return 0.0;
    }

    return velocity * reactionTime;
}

DistanceMeters KinematicsEngine::safeDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared nominalDeceleration,
    const double gradient,
    const TimeSeconds reactionTime,
    const DistanceMeters safetyMargin
)
{
    requireFinite(velocity, "velocity");
    requireFinite(nominalDeceleration, "nominalDeceleration");
    requireFinite(gradient, "gradient");
    requireFinite(reactionTime, "reactionTime");
    requireFinite(safetyMargin, "safetyMargin");
    const double dReaction = reactionDistance(velocity, reactionTime);
    const double dBraking  = brakingDistance(velocity, nominalDeceleration, gradient);
    const double margin    = std::max(safetyMargin, 0.0);

    return dReaction + dBraking + margin;
}

DistanceMeters KinematicsEngine::emergencyStoppingDistance(
    const SpeedMetersPerSecond velocity,
    const AccelerationMetersPerSecondSquared emergencyDeceleration,
    const double gradient
)
{
    requireFinite(velocity, "velocity");
    requireFinite(emergencyDeceleration, "emergencyDeceleration");
    requireFinite(gradient, "gradient");
    // Emergency braking: no reaction time component.
    const double aEff = effectiveDeceleration(emergencyDeceleration, gradient);
    return brakingDistance(velocity, aEff);
}

double KinematicsEngine::speedAfterDistance(
    double initialVelocity,
    double acceleration,
    double distance
)
{
    requireFinite(initialVelocity, "initialVelocity");
    requireFinite(acceleration, "acceleration");
    requireFinite(distance, "distance");
    // Negative initial velocity is invalid.
    if (initialVelocity < 0.0)
    {
        return 0.0;
    }

    // No distance travelled -> velocity unchanged.
    if (distance <= 0.0)
    {
        return initialVelocity;
    }

    // v² = u² + 2as
    const double velocitySquared =
        initialVelocity * initialVelocity
        + 2.0 * acceleration * distance;

    // The train has decelerated to zero before
    // travelling the requested distance.
    if (velocitySquared <= 0.0)
    {
        return 0.0;
    }

    return std::sqrt(velocitySquared);
}

} // namespace tcas::physics
