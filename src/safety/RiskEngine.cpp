#include "safety/RiskEngine.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::safety
{
namespace
{

double clampScore(double value) noexcept
{
    if (!std::isfinite(value))
    {
        return 100.0;
    }

    return std::clamp(value, 0.0, 100.0);
}

double clampConfidence(double value) noexcept
{
    if (!std::isfinite(value))
    {
        return 0.0;
    }

    return std::clamp(value, 0.0, 1.0);
}

} // namespace

bool RiskAssessment::isActionRequired() const noexcept
{
    return score > 30.0;
}

bool RiskAssessment::isCritical() const noexcept
{
    return level == RiskLevel::Critical;
}

RiskAssessment RiskEngine::assess(
    const RiskInput& input
) const noexcept
{
    const double score =
        calculateTtcRisk(input.timeToCollision)
        + calculateRelativeVelocityRisk(input.relativeVelocity)
        + calculateBrakingRisk(
            input.brakingDistance,
            input.safetyMargin)
        + calculateConflictTypeRisk(input.conflictType)
        + calculateMassRisk(input.trainMass)
        + calculateSensorRisk(input.sensorConfidence)
        + calculateCommunicationRisk(
            input.communicationConfidence);

    RiskAssessment assessment;

    assessment.score = clampScore(score);
    assessment.level = classify(assessment.score);
    assessment.timeToCollision = input.timeToCollision;
    assessment.brakingDistance = input.brakingDistance;
    assessment.safetyMargin = input.safetyMargin;

    return assessment;
}

RiskLevel RiskEngine::classify(double score) noexcept
{
    const double clamped = clampScore(score);

    if (clamped <= 30.0)
    {
        return RiskLevel::Low;
    }

    if (clamped <= 60.0)
    {
        return RiskLevel::Medium;
    }

    if (clamped <= 80.0)
    {
        return RiskLevel::High;
    }

    return RiskLevel::Critical;
}

double RiskEngine::calculateTtcRisk(
    TimeSeconds ttc
) noexcept
{
    if (!std::isfinite(ttc))
    {
        return 35.0;
    }

    if (ttc <= 0.0)
    {
        return 35.0;  // Collision already occurring
    }

    if (ttc <= 3.0)
    {
        return 30.0;  // Imminent — less than 3 s
    }

    if (ttc <= 8.0)
    {
        return 22.0;  // Very close — under 8 s
    }

    if (ttc <= 15.0)
    {
        return 15.0;  // Close — under 15 s
    }

    if (ttc <= 30.0)
    {
        return 8.0;   // Moderate — under 30 s
    }

    if (ttc <= 60.0)
    {
        return 3.0;   // Early warning — under 60 s
    }

    return 0.0;  // Well ahead — no TTC contribution
}

double RiskEngine::calculateRelativeVelocityRisk(
    SpeedMetersPerSecond relativeVelocity
) noexcept
{
    if (!std::isfinite(relativeVelocity))
    {
        return 20.0;
    }

    const double closingSpeed =
        std::max(0.0, relativeVelocity);

    if (closingSpeed <= 1.0)
    {
        return 0.0;
    }

    if (closingSpeed <= 5.0)
    {
        return 5.0;
    }

    if (closingSpeed <= 10.0)
    {
        return 10.0;
    }

    if (closingSpeed <= 20.0)
    {
        return 15.0;
    }

    return 20.0;
}

double RiskEngine::calculateBrakingRisk(
    DistanceMeters brakingDistance,
    DistanceMeters safetyMargin
) noexcept
{
    if (!std::isfinite(brakingDistance)
        || !std::isfinite(safetyMargin))
    {
        return 25.0;
    }

    if (brakingDistance < 0.0)
    {
        return 25.0;
    }

    if (safetyMargin <= 0.0)
    {
        return 25.0;
    }

    if (brakingDistance == 0.0)
    {
        return 0.0;
    }

    const double ratio =
        safetyMargin / brakingDistance;

    if (ratio < 0.10)
    {
        return 25.0;
    }

    if (ratio < 0.25)
    {
        return 20.0;
    }

    if (ratio < 0.50)
    {
        return 10.0;
    }

    if (ratio < 1.0)
    {
        return 5.0;
    }

    return 0.0;
}

double RiskEngine::calculateConflictTypeRisk(
    conflict::ConflictType type
) noexcept
{
    switch (type)
    {
    case conflict::ConflictType::HeadOn:
        return 15.0;

    case conflict::ConflictType::Junction:
        return 10.0;

    case conflict::ConflictType::Platform:
        return 8.0;

    case conflict::ConflictType::RearEnd:
        return 10.0;
    }

    return 15.0;
}

double RiskEngine::calculateMassRisk(
    double mass
) noexcept
{
    if (!std::isfinite(mass) || mass <= 0.0)
    {
        return 10.0;
    }

    if (mass >= 150000.0)
    {
        return 10.0;
    }

    if (mass >= 100000.0)
    {
        return 7.0;
    }

    if (mass >= 50000.0)
    {
        return 4.0;
    }

    return 1.0;
}

double RiskEngine::calculateSensorRisk(
    double sensorConfidence
) noexcept
{
    const double confidence =
        clampConfidence(sensorConfidence);

    return (1.0 - confidence) * 15.0;
}

double RiskEngine::calculateCommunicationRisk(
    double communicationConfidence
) noexcept
{
    const double confidence =
        clampConfidence(communicationConfidence);

    return (1.0 - confidence) * 10.0;
}

} // namespace tcas::safety