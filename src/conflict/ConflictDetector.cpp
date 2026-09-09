#include "conflict/ConflictDetector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace tcas::conflict
{
namespace
{

struct Sample
{
    TimeSeconds time{ 0.0 };
    DistanceMeters position{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};

Sample interpolate(
    const prediction::FutureState& first,
    const prediction::FutureState& second,
    TimeSeconds time)
{
    const TimeSeconds duration = second.timestamp - first.timestamp;
    if (duration <= 0.0)
    {
        return { time, first.position, first.uncertainty };
    }

    const double fraction =
        std::clamp((time - first.timestamp) / duration, 0.0, 1.0);

    return {
        time,
        first.position + fraction * (second.position - first.position),
        first.uncertainty + fraction * (second.uncertainty - first.uncertainty)
    };
}

bool validTrajectory(const std::vector<prediction::FutureState>& trajectory)
{
    if (trajectory.empty())
    {
        return false;
    }

    for (std::size_t index = 0; index < trajectory.size(); ++index)
    {
        const auto& state = trajectory[index];
        if (!std::isfinite(state.timestamp) ||
            !std::isfinite(state.position) ||
            !std::isfinite(state.velocity) ||
            !std::isfinite(state.uncertainty) ||
            state.uncertainty < 0.0 ||
            (index > 0 && state.timestamp < trajectory[index - 1].timestamp))
        {
            return false;
        }
    }
    return true;
}

struct NodeEvent
{
    NodeId nodeId{ 0 };
    TimeSeconds time{ 0.0 };
    DistanceMeters uncertainty{ 0.0 };
};

void addResourceEvent(
    std::vector<NodeEvent>& events,
    const prediction::FutureState& state,
    const infrastructure::Track* track,
    const infrastructure::RailwayNetwork& network)
{
    if (track == nullptr)
    {
        return;
    }

    // A train only occupies the track's source node if it is physically located
    // at the beginning of the track (within 25m clearance zone). Once farther
    // down the track, it has cleared the source junction and is not occupying it.
    if (state.position > 25.0)
    {
        return;
    }

    const auto* node = network.getNode(track->source());
    if (node != nullptr &&
        (node->type() == infrastructure::NodeType::Junction ||
         node->type() == infrastructure::NodeType::Platform))
    {
        events.push_back({track->source(), state.timestamp, state.uncertainty});
    }
}

std::vector<NodeEvent> extractNodeEvents(
    const std::vector<prediction::FutureState>& trajectory,
    const infrastructure::RailwayNetwork& network)
{
    std::vector<NodeEvent> events;
    if (trajectory.empty())
    {
        return events;
    }

    addResourceEvent(
        events,
        trajectory.front(),
        network.getTrack(trajectory.front().trackId),
        network);

    for (std::size_t index = 1; index < trajectory.size(); ++index)
    {
        const auto& previous = trajectory[index - 1];
        const auto& current = trajectory[index];
        if (previous.trackId == current.trackId)
        {
            continue;
        }

        const auto* previousTrack = network.getTrack(previous.trackId);
        const auto* currentTrack = network.getTrack(current.trackId);
        if (previousTrack == nullptr || currentTrack == nullptr ||
            previousTrack->destination() != currentTrack->source())
        {
            continue;
        }

        const auto* node = network.getNode(previousTrack->destination());
        if (node != nullptr &&
            (node->type() == infrastructure::NodeType::Junction ||
             node->type() == infrastructure::NodeType::Platform))
        {
            const double remDist = std::max(0.0, previousTrack->length() - previous.position);
            const double speed = std::max(1.0, previous.velocity);
            const double dt = current.timestamp - previous.timestamp;
            const double timeOffset = (dt > 0.0)
                ? std::clamp(remDist / speed, 0.0, dt)
                : 0.0;
            const double exactTime = previous.timestamp + timeOffset;

            events.push_back({
                previousTrack->destination(),
                exactTime,
                current.uncertainty
            });
        }
    }

    return events;
}

} // namespace

ConflictDetector::ConflictDetector(ConflictDetectionConfig config)
    : config_(config)
{
    if (!std::isfinite(config_.minimumTrackSeparation) ||
        config_.minimumTrackSeparation < 0.0 ||
        !std::isfinite(config_.resourceTimeSeparation) ||
        config_.resourceTimeSeparation < 0.0 ||
        !std::isfinite(config_.resourceClearanceTime) ||
        config_.resourceClearanceTime < 0.0)
    {
        throw std::invalid_argument("Invalid conflict detection configuration");
    }
}

std::vector<Conflict> ConflictDetector::detect(
    TrainId trainA,
    const std::vector<prediction::FutureState>& trajectoryA,
    TrainId trainB,
    const std::vector<prediction::FutureState>& trajectoryB,
    const infrastructure::RailwayNetwork& network) const
{
    if (trainA == trainB)
    {
        throw std::invalid_argument("Conflict detection requires two different trains");
    }
    if (!validTrajectory(trajectoryA) || !validTrajectory(trajectoryB))
    {
        throw std::invalid_argument("Trajectories must be non-empty and chronological");
    }

    std::vector<Conflict> conflicts;

    for (std::size_t i = 0; i + 1 < trajectoryA.size(); ++i)
    {
        for (std::size_t j = 0; j + 1 < trajectoryB.size(); ++j)
        {
            const auto& a0 = trajectoryA[i];
            const auto& a1 = trajectoryA[i + 1];
            const auto& b0 = trajectoryB[j];
            const auto& b1 = trajectoryB[j + 1];

            if (a0.trackId != a1.trackId || b0.trackId != b1.trackId)
            {
                continue;
            }

            const auto* trackA = network.getTrack(a0.trackId);
            const auto* trackB = network.getTrack(b0.trackId);
            if (trackA == nullptr || trackB == nullptr)
            {
                continue;
            }

            const bool sameTrack = (a0.trackId == b0.trackId);
            const bool opposingTrack = (trackA->source() == trackB->destination() &&
                                        trackA->destination() == trackB->source());

            if (!sameTrack && !opposingTrack)
            {
                continue;
            }

            prediction::FutureState b0_mapped = b0;
            prediction::FutureState b1_mapped = b1;
            if (opposingTrack)
            {
                b0_mapped.position = trackA->length() - b0.position;
                b0_mapped.velocity = -b0.velocity;
                b1_mapped.position = trackA->length() - b1.position;
                b1_mapped.velocity = -b1.velocity;
            }

            DistanceMeters minimumSeparation =
                std::numeric_limits<DistanceMeters>::infinity();
            TimeSeconds firstTime = 0.0;
            TimeSeconds lastTime = 0.0;
            if (!hasTemporalConflict(
                    a0, a1, b0_mapped, b1_mapped, minimumSeparation, firstTime, lastTime))
            {
                continue;
            }

            const Conflict candidate{
                trainA,
                trainB,
                opposingTrack ? ConflictType::HeadOn : classifySameTrack(a0, b0),
                a0.trackId,
                0,
                firstTime,
                lastTime,
                minimumSeparation
            };

            auto it = std::find_if(
                conflicts.begin(),
                conflicts.end(),
                [&](const Conflict& existing)
                {
                    return existing.trackId == candidate.trackId &&
                           existing.type == candidate.type;
                });
            if (it != conflicts.end())
            {
                it->firstConflictTime = std::min(it->firstConflictTime, candidate.firstConflictTime);
                it->lastConflictTime = std::max(it->lastConflictTime, candidate.lastConflictTime);
                it->minimumSeparation = std::min(it->minimumSeparation, candidate.minimumSeparation);
            }
            else
            {
                conflicts.push_back(candidate);
            }
        }
    }

    const auto eventsA = extractNodeEvents(trajectoryA, network);
    const auto eventsB = extractNodeEvents(trajectoryB, network);

    for (const auto& eventA : eventsA)
    {
        const auto* node = network.getNode(eventA.nodeId);
        if (node == nullptr)
        {
            continue;
        }

        for (const auto& eventB : eventsB)
        {
            if (eventA.nodeId != eventB.nodeId ||
                std::abs(eventA.time - eventB.time) > config_.resourceTimeSeparation)
            {
                continue;
            }

            const ConflictType type =
                node->type() == infrastructure::NodeType::Junction
                    ? ConflictType::Junction
                    : ConflictType::Platform;

            const Conflict candidate{
                trainA,
                trainB,
                type,
                0,
                eventA.nodeId,
                std::min(eventA.time, eventB.time),
                std::max(eventA.time, eventB.time) + config_.resourceClearanceTime,
                0.0
            };

            auto it = std::find_if(
                conflicts.begin(),
                conflicts.end(),
                [&](const Conflict& existing)
                {
                    return existing.type == candidate.type &&
                           existing.resourceNodeId == candidate.resourceNodeId;
                });
            if (it != conflicts.end())
            {
                it->firstConflictTime = std::min(it->firstConflictTime, candidate.firstConflictTime);
                it->lastConflictTime = std::max(it->lastConflictTime, candidate.lastConflictTime);
            }
            else
            {
                conflicts.push_back(candidate);
            }
        }
    }

    return conflicts;
}

ConflictType ConflictDetector::classifySameTrack(
    const prediction::FutureState& a,
    const prediction::FutureState& b) const noexcept
{
    return (a.velocity * b.velocity < 0.0)
        ? ConflictType::HeadOn
        : ConflictType::RearEnd;
}

bool ConflictDetector::hasTemporalConflict(
    const prediction::FutureState& a0,
    const prediction::FutureState& a1,
    const prediction::FutureState& b0,
    const prediction::FutureState& b1,
    DistanceMeters& minimumSeparation,
    TimeSeconds& firstTime,
    TimeSeconds& lastTime) const noexcept
{
    const TimeSeconds start = std::max(a0.timestamp, b0.timestamp);
    const TimeSeconds end = std::min(a1.timestamp, b1.timestamp);
    if (end < start)
    {
        return false;
    }

    const TimeSeconds interval = end - start;
    if (interval <= 0.0)
    {
        return false;
    }

    const Sample aStart = interpolate(a0, a1, start);
    const Sample aEnd = interpolate(a0, a1, end);
    const Sample bStart = interpolate(b0, b1, start);
    const Sample bEnd = interpolate(b0, b1, end);

    const double relativeStart = aStart.position - bStart.position;
    const double relativeSlope =
        ((aEnd.position - aStart.position) - (bEnd.position - bStart.position)) /
        interval;
    const double marginStart =
        config_.minimumTrackSeparation + aStart.uncertainty + bStart.uncertainty;
    const double marginSlope =
        ((aEnd.uncertainty - aStart.uncertainty) +
         (bEnd.uncertainty - bStart.uncertainty)) /
        interval;

    const double qa = relativeSlope * relativeSlope - marginSlope * marginSlope;
    const double qb = 2.0 * (relativeStart * relativeSlope - marginStart * marginSlope);
    const double qc = relativeStart * relativeStart - marginStart * marginStart;

    auto f = [&](double timeFromStart)
    {
        return (qa * timeFromStart + qb) * timeFromStart + qc;
    };

    std::vector<double> candidates{0.0, interval};
    constexpr double kEpsilon = 1e-12;

    if (std::abs(qa) > kEpsilon)
    {
        const double discriminant = qb * qb - 4.0 * qa * qc;
        if (discriminant >= 0.0)
        {
            const double root = std::sqrt(discriminant);
            candidates.push_back((-qb - root) / (2.0 * qa));
            candidates.push_back((-qb + root) / (2.0 * qa));
        }
        candidates.push_back(-qb / (2.0 * qa));
    }
    else if (std::abs(qb) > kEpsilon)
    {
        candidates.push_back(-qc / qb);
    }

    std::sort(candidates.begin(), candidates.end());
    candidates.erase(
        std::unique(
            candidates.begin(),
            candidates.end(),
            [](double first, double second)
            {
                return std::abs(first - second) < kEpsilon;
            }),
        candidates.end());

    minimumSeparation = std::numeric_limits<DistanceMeters>::infinity();
    bool conflict = false;
    firstTime = 0.0;
    lastTime = 0.0;

    for (double candidate : candidates)
    {
        if (candidate < 0.0 || candidate > interval)
        {
            continue;
        }
        const Sample a = interpolate(a0, a1, start + candidate);
        const Sample b = interpolate(b0, b1, start + candidate);
        const double separation = std::abs(a.position - b.position);
        minimumSeparation = std::min(minimumSeparation, separation);

        const double margin =
            config_.minimumTrackSeparation + a.uncertainty + b.uncertainty;
        if (separation <= margin)
        {
            if (!conflict)
            {
                firstTime = start + candidate;
                conflict = true;
            }
            lastTime = std::max(lastTime, start + candidate);
        }
    }

    for (std::size_t index = 0; index + 1 < candidates.size(); ++index)
    {
        const double left = std::max(0.0, candidates[index]);
        const double right = std::min(interval, candidates[index + 1]);
        if (right < left)
        {
            continue;
        }

        const double midpoint = (left + right) * 0.5;
        if (f(midpoint) <= 0.0)
        {
            if (!conflict)
            {
                firstTime = start + left;
                conflict = true;
            }
            lastTime = start + right;
            lastTime = std::max(lastTime, start + right);
        }
    }

    return conflict;
}

} // namespace tcas::conflict
