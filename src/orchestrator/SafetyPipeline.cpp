#include "orchestrator/SafetyPipeline.hpp"

#include "conflict/ConflictDetector.hpp"
#include "conflict/ConflictZone.hpp"
#include "physics/KinematicsEngine.hpp"
#include "prediction/PredictionEngine.hpp"
#include "safety/ConflictPriorityQueue.hpp"
#include "safety/PriorityEngine.hpp"
#include "safety/ResolutionEngine.hpp"
#include "safety/RiskEngine.hpp"
#include "train/ExpressTrain.hpp"
#include "train/FreightTrain.hpp"
#include "train/PassengerTrain.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::orchestrator
{

SafetyPipeline::SafetyPipeline(
    const infrastructure::RailwayNetwork& network,
    const train::TrainManager& trainManager,
    std::vector<TrainRoute> routes)
    : network_(network),
      trainManager_(trainManager),
      routes_(std::move(routes))
{
}

void SafetyPipeline::setRoutes(std::vector<TrainRoute> routes)
{
    std::lock_guard lock(routesMutex_);
    routes_ = std::move(routes);
}

void SafetyPipeline::addOrUpdateRoute(TrainRoute route)
{
    std::lock_guard lock(routesMutex_);
    for (auto& r : routes_)
    {
        if (r.trainId == route.trainId)
        {
            r = std::move(route);
            return;
        }
    }
    routes_.push_back(std::move(route));
}

void SafetyPipeline::removeRoute(TrainId trainId)
{
    std::lock_guard lock(routesMutex_);
    std::erase_if(routes_, [trainId](const TrainRoute& r) {
        return r.trainId == trainId;
    });
}

SafetyStep SafetyPipeline::makeStep()
{
    return [this](const WorldState& state) -> SafetyCycleResult
    {
        return run(state);
    };
}

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

namespace
{

// Build a lightweight Train proxy from a WorldState snapshot for use with
// physics / safety engines that require a Train&.
// We use the real TrainManager to obtain the physics constants (mass, braking
// deceleration etc.) and override the kinematic state from the snapshot.
std::unique_ptr<train::Train> buildTrainProxy(
    const TrainSnapshot& snap)
{
    std::unique_ptr<train::Train> proxy;
    switch (snap.type)
    {
    case TrainType::Express:
        proxy = std::make_unique<train::ExpressTrain>(
            snap.id, snap.mass, snap.maximumSpeed,
            snap.serviceBraking, snap.emergencyBraking);
        break;
    case TrainType::Passenger:
        proxy = std::make_unique<train::PassengerTrain>(
            snap.id, snap.mass, snap.maximumSpeed,
            snap.serviceBraking, snap.emergencyBraking);
        break;
    case TrainType::Freight:
        proxy = std::make_unique<train::FreightTrain>(
            snap.id, snap.mass, snap.maximumSpeed,
            snap.serviceBraking, snap.emergencyBraking);
        break;
    }

    if (proxy)
    {
        proxy->setPosition(snap.position);
        proxy->setVelocity(snap.velocity);
        proxy->setAcceleration(snap.acceleration);
        proxy->setState(snap.state);
    }
    return proxy;
}

// Find the distance from the train's current position to a target node along
// its assigned route. Returns 0.0 if not found.
DistanceMeters distanceToNode(
    const infrastructure::RailwayNetwork& network,
    const navigation::RouteResult& route,
    TrackId currentTrackId,
    DistanceMeters currentPosition,
    NodeId targetNodeId)
{
    const auto it = std::find(
        route.tracks.begin(), route.tracks.end(), currentTrackId);
    if (it == route.tracks.end())
    {
        return 0.0;
    }

    const std::size_t startIdx = static_cast<std::size_t>(
        std::distance(route.tracks.begin(), it));
    DistanceMeters distance = 0.0;

    for (std::size_t i = startIdx; i < route.tracks.size(); ++i)
    {
        const auto* track = network.getTrack(route.tracks[i]);
        if (track == nullptr)
        {
            return 0.0;
        }
        if (track->source() == targetNodeId)
        {
            return distance;
        }
        const DistanceMeters pos =
            (i == startIdx)
                ? std::clamp(currentPosition, 0.0, track->length())
                : 0.0;
        distance += track->length() - pos;
        if (track->destination() == targetNodeId)
        {
            return distance;
        }
    }
    return 0.0;
}

[[maybe_unused]] const char* commandTypeName(safety::SafetyCommandType t) noexcept
{
    switch (t)
    {
    case safety::SafetyCommandType::NoAction:       return "NO_ACTION";
    case safety::SafetyCommandType::ReduceSpeed:    return "REDUCE_SPEED";
    case safety::SafetyCommandType::HoldAtSignal:   return "HOLD_AT_SIGNAL";
    case safety::SafetyCommandType::EmergencyBrake: return "EMERGENCY_BRAKE";
    }
    return "UNKNOWN";
}

} // namespace

// -----------------------------------------------------------------------
// Main pipeline
// -----------------------------------------------------------------------

SafetyCycleResult SafetyPipeline::run(const WorldState& state)
{
    SafetyCycleResult result;

    std::vector<TrainRoute> activeRoutes;
    {
        std::lock_guard lock(routesMutex_);
        activeRoutes = routes_;
    }

    if (activeRoutes.empty() || state.trains.empty())
    {
        return result;
    }

    // ------------------------------------------------------------------
    // Step 1 — build proxy Train objects and predict trajectories
    // ------------------------------------------------------------------
    struct TrainContext
    {
        std::unique_ptr<train::Train> proxy;
        std::vector<prediction::FutureState> trajectory;
        TrackId currentTrackId{ 0 };
        const navigation::RouteResult* route{ nullptr };
        bool hasSensorFault{ false };
    };

    std::vector<TrainContext> contexts;
    contexts.reserve(activeRoutes.size());

    for (const auto& trainRoute : activeRoutes)
    {
        // Find snapshot
        const TrainSnapshot* snap = nullptr;
        for (const auto& t : state.trains)
        {
            if (t.id == trainRoute.trainId)
            {
                snap = &t;
                break;
            }
        }
        if (snap == nullptr)
        {
            continue;
        }

        // If the train has finished its route and docked at the terminal station,
        // it has cleared the mainline corridor and should not block following trains.
        if (snap->state == TrainState::Stopped && !trainRoute.route.tracks.empty())
        {
            if (snap->trackId == trainRoute.route.tracks.back() || trainRoute.currentTrackId == trainRoute.route.tracks.back())
            {
                const auto* finalTrack = network_.getTrack(trainRoute.route.tracks.back());
                if (finalTrack != nullptr && snap->position >= finalTrack->length() - 5.0)
                {
                    continue;
                }
            }
        }

        auto proxy = buildTrainProxy(*snap);
        if (proxy == nullptr)
        {
            continue;
        }

        const TrackId effectiveTrackId = (snap->trackId != 0)
            ? snap->trackId
            : trainRoute.currentTrackId;

        const auto* track = network_.getTrack(effectiveTrackId);
        if (track != nullptr)
        {
            // Clamp proxy position to track length so PredictionEngine doesn't throw
            proxy->setPosition(std::clamp(proxy->position(), 0.0, track->length()));
        }

        // Step 2 — Module 8: Prediction
        std::vector<prediction::FutureState> trajectory;
        try
        {
            trajectory =
                prediction::PredictionEngine::predictStandardHorizon(
                    *proxy,
                    network_,
                    trainRoute.route,
                    effectiveTrackId,
                    1.0 /* m uncertainty */);
        }
        catch (const std::exception& /*e*/)
        {
            // Boundary overrun or prediction error handled safely
            trajectory.clear();
        }

        contexts.push_back({
            std::move(proxy),
            std::move(trajectory),
            effectiveTrackId,
            &trainRoute.route,
            snap->sensorFailure
        });
    }

    // Collect all predictions for WorldState
    for (const auto& ctx : contexts)
    {
        for (const auto& fs : ctx.trajectory)
        {
            result.predictions.push_back(fs);
        }
    }

    if (contexts.size() < 2)
    {
        return result;
    }

    // ------------------------------------------------------------------
    // Step 3 — Module 9: Conflict detection (all pairs)
    // ------------------------------------------------------------------
    conflict::ConflictDetector detector;
    std::vector<conflict::Conflict> allConflicts;

    for (std::size_t i = 0; i < contexts.size(); ++i)
    {
        for (std::size_t j = i + 1; j < contexts.size(); ++j)
        {
            const auto conflicts = detector.detect(
                contexts[i].proxy->id(), contexts[i].trajectory,
                contexts[j].proxy->id(), contexts[j].trajectory,
                network_);
            for (const auto& c : conflicts)
            {
                allConflicts.push_back(c);
            }
        }
    }

    result.activeConflicts = allConflicts;

    if (allConflicts.empty())
    {
        return result;
    }

    // ------------------------------------------------------------------
    // Step 4 — Module 10: Risk assessment + Priority queue
    // ------------------------------------------------------------------
    safety::RiskEngine riskEngine;
    safety::PriorityEngine priorityEngine;
    safety::ConflictPriorityQueue conflictQueue;

    const TimeSeconds currentTime = state.simulationTime;

    for (const auto& detected : allConflicts)
    {
        // Find the two train contexts
        const TrainContext* ctxA = nullptr;
        const TrainContext* ctxB = nullptr;
        for (const auto& ctx : contexts)
        {
            if (ctx.proxy->id() == detected.trainA) { ctxA = &ctx; }
            if (ctx.proxy->id() == detected.trainB) { ctxB = &ctx; }
        }
        if (ctxA == nullptr || ctxB == nullptr) { continue; }

        // detected.firstConflictTime is already the relative TTC from prediction horizons
        const TimeSeconds ttc = std::max(0.0, detected.firstConflictTime);
        const double relVel = std::abs(
            ctxA->proxy->velocity() - ctxB->proxy->velocity());

        // Determine the yielding train (lower priority)
        const auto prioA = priorityEngine.assess(*ctxA->proxy);
        const auto prioB = priorityEngine.assess(*ctxB->proxy);
        const bool aHasPriority =
            prioA.higherThan(prioB) ||
            (prioA.priority == prioB.priority &&
             ctxA->proxy->id() < ctxB->proxy->id());

        const TrainContext* yieldCtx = aHasPriority ? ctxB : ctxA;

        // Braking geometry
        const auto* yieldTrack = network_.getTrack(yieldCtx->currentTrackId);
        const double gradient =
            yieldTrack != nullptr ? yieldTrack->gradient() : 0.0;
        const double speed = std::max(0.0, yieldCtx->proxy->velocity());
        const double brakingDist =
            physics::KinematicsEngine::emergencyStoppingDistance(
                speed, yieldCtx->proxy->emergencyBraking(), gradient);
        const double availDist = distanceToNode(
            network_,
            *yieldCtx->route,
            yieldCtx->currentTrackId,
            yieldCtx->proxy->position(),
            detected.resourceNodeId);
        const double safetyMargin = availDist - brakingDist;

        safety::RiskInput riskInput;
        riskInput.timeToCollision           = ttc;
        riskInput.relativeVelocity          = relVel;
        riskInput.brakingDistance           = brakingDist;
        riskInput.safetyMargin              = safetyMargin;
        riskInput.conflictType              = detected.type;
        riskInput.trainMass                 = yieldCtx->proxy->mass();
        riskInput.sensorConfidence          = (yieldCtx->hasSensorFault || state.sensorFailure) ? 0.5 : 1.0;
        riskInput.communicationConfidence   = state.communicationFailure ? 0.4 : 1.0;

        const auto risk = riskEngine.assess(riskInput);
        conflictQueue.push(detected, risk);
    }

    // ------------------------------------------------------------------
    // Step 5 — Module 9+10: Process queue, reserve, resolve
    // ------------------------------------------------------------------
    safety::ResolutionEngine resolutionEngine;

    // Release stale reservations from previous cycles
    reservations_.clearReleased();
    reservations_.clearExpired(currentTime); // Expire zones the clock has already passed

    while (!conflictQueue.empty())
    {
        const auto top = conflictQueue.top();
        conflictQueue.pop();

        const auto& detected = top.conflict;

        // Locate contexts
        const TrainContext* ctxA = nullptr;
        const TrainContext* ctxB = nullptr;
        for (const auto& ctx : contexts)
        {
            if (ctx.proxy->id() == detected.trainA) { ctxA = &ctx; }
            if (ctx.proxy->id() == detected.trainB) { ctxB = &ctx; }
        }
        if (ctxA == nullptr || ctxB == nullptr) { continue; }

        const auto prioA = priorityEngine.assess(*ctxA->proxy);
        const auto prioB = priorityEngine.assess(*ctxB->proxy);
        const bool aHasPriority =
            prioA.higherThan(prioB) ||
            (prioA.priority == prioB.priority &&
             ctxA->proxy->id() < ctxB->proxy->id());

        const TrainContext* prioCtx  = aHasPriority ? ctxA : ctxB;
        const TrainContext* yieldCtx = aHasPriority ? ctxB : ctxA;

        // Resource reservation for the priority train
        conflict::ConflictZoneType zoneType = conflict::ConflictZoneType::TrackSection;
        if (detected.type == conflict::ConflictType::Junction)
        {
            zoneType = conflict::ConflictZoneType::Junction;
        }
        else if (detected.type == conflict::ConflictType::Platform)
        {
            zoneType = conflict::ConflictZoneType::Platform;
        }

        const conflict::ConflictZone zone{
            zoneType,
            detected.resourceNodeId,
            detected.trackId
        };

        // Reservations manage absolute simulation time intervals
        const bool reserved = reservations_.request(
            prioCtx->proxy->id(),
            zone,
            currentTime + detected.firstConflictTime,
            currentTime + detected.lastConflictTime);

        if (!reserved)
        {
            // Already reserved by someone else — skip this conflict
            continue;
        }
        result.reservations = reservations_.reservations();

        // Braking feasibility for yielding train
        const auto* yieldTrack = network_.getTrack(yieldCtx->currentTrackId);
        const double gradient =
            yieldTrack != nullptr ? yieldTrack->gradient() : 0.0;
        const double speed = std::max(0.0, yieldCtx->proxy->velocity());
        const double brakingDist =
            physics::KinematicsEngine::emergencyStoppingDistance(
                speed, yieldCtx->proxy->emergencyBraking(), gradient);
        const double availDist = distanceToNode(
            network_,
            *yieldCtx->route,
            yieldCtx->currentTrackId,
            yieldCtx->proxy->position(),
            detected.resourceNodeId);
        const bool brakingFeasible =
            std::isfinite(brakingDist) &&
            brakingDist >= 0.0 &&
            availDist >= brakingDist + std::max(0.0, top.risk.safetyMargin);

        const safety::ResolutionInput resInput{
            *yieldCtx->proxy,
            detected,
            top.risk,
            false,          // yielding train has no priority
            brakingFeasible,
            availDist,
            brakingDist
        };

        const auto command = resolutionEngine.resolve(resInput);
        result.commands.push_back(command);

        // Safety decision record
        SafetyDecision decision;
        decision.priorityTrain = prioCtx->proxy->id();
        decision.yieldingTrain = yieldCtx->proxy->id();
        decision.riskScore     = top.risk.score;
        decision.commandType   = command.type;
        result.decisions.push_back(decision);
    }

    return result;
}

} // namespace tcas::orchestrator
