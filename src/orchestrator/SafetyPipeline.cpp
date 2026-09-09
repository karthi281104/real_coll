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
// its assigned route.
// Returns -1.0 (sentinel) when currentTrackId is not present in the route,
// so callers can distinguish "not found" from "distance is zero / node reached".
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
        return -1.0; // BUG-5 fix: sentinel — track not in route
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
            if (i == startIdx && currentPosition > 25.0)
            {
                // The source node is already behind the train's current position on this track
                return 0.0;
            }
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
        if (snap->state == TrainState::Completed)
        {
            continue;
        }

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
            constexpr std::array<TimeSeconds, 7> kPipelineHorizons{
                5.0, 10.0, 20.0, 30.0, 60.0, 90.0, 120.0
            };
            trajectory =
                prediction::PredictionEngine::predict(
                    *proxy,
                    network_,
                    trainRoute.route,
                    effectiveTrackId,
                    std::vector<TimeSeconds>(kPipelineHorizons.begin(), kPipelineHorizons.end()),
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

    const TimeSeconds currentTime = state.simulationTime;

    // Release reservations whose reserving train has already passed through the zone
    for (const auto& r : reservations_.reservations())
    {
        for (const auto& ctx : contexts)
        {
            if (ctx.proxy->id() == r.trainId)
            {
                if (r.zone.nodeId != 0)
                {
                    const auto* curTrack = network_.getTrack(ctx.currentTrackId);
                    if (curTrack != nullptr && curTrack->source() == r.zone.nodeId && ctx.proxy->position() > 25.0)
                    {
                        static_cast<void>(reservations_.release(r.trainId, r.zone));
                    }
                }
            }
        }
    }
    reservations_.clearReleased();
    reservations_.clearExpired(currentTime);

    // If any train is approaching a node currently reserved by ANOTHER train,
    // maintain the reservation conflict so yielding trains hold safely until cleared
    for (const auto& r : reservations_.reservations())
    {
        if (r.state != conflict::ResourceState::Reserved)
        {
            continue;
        }
        for (const auto& ctx : contexts)
        {
            if (ctx.proxy->id() != r.trainId && ctx.route != nullptr && r.zone.nodeId != 0)
            {
                const double dist = distanceToNode(
                    network_, *ctx.route, ctx.currentTrackId, ctx.proxy->position(), r.zone.nodeId);
                if (dist >= 0.0 && dist <= 500.0)
                {
                    conflict::Conflict resConflict{
                        r.trainId,
                        ctx.proxy->id(),
                        (r.zone.type == conflict::ConflictZoneType::Junction)
                            ? conflict::ConflictType::Junction
                            : conflict::ConflictType::Platform,
                        0,
                        r.zone.nodeId,
                        std::max(0.0, r.startTime - currentTime),
                        std::max(0.0, r.endTime - currentTime),
                        dist
                    };
                    const bool dup = std::any_of(
                        allConflicts.begin(), allConflicts.end(),
                        [&](const conflict::Conflict& ex) {
                            return ex.resourceNodeId == r.zone.nodeId &&
                                   ((ex.trainA == r.trainId && ex.trainB == ctx.proxy->id()) ||
                                    (ex.trainB == r.trainId && ex.trainA == ctx.proxy->id()));
                        });
                    if (!dup)
                    {
                        allConflicts.push_back(resConflict);
                    }
                }
            }
        }
    }

    result.activeConflicts = allConflicts;
    result.reservations = reservations_.reservations();

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
        // LOGIC-1 fix: when types are equal, the train with higher approach
        // velocity is harder to stop and therefore gets priority.
        // Tiebreak on lower ID only when velocities are also equal.
        const bool aHasPriority =
            prioA.higherThan(prioB) ||
            (prioA.priority == prioB.priority &&
             (ctxA->proxy->velocity() > ctxB->proxy->velocity() ||
              (ctxA->proxy->velocity() == ctxB->proxy->velocity() &&
               ctxA->proxy->id() < ctxB->proxy->id())));

        const TrainContext* yieldCtx = aHasPriority ? ctxB : ctxA;

        // Braking geometry
        const auto* yieldTrack = network_.getTrack(yieldCtx->currentTrackId);
        const double gradient =
            yieldTrack != nullptr ? yieldTrack->gradient() : 0.0;
        const double speed = std::max(0.0, yieldCtx->proxy->velocity());
        const double brakingDist =
            physics::KinematicsEngine::emergencyStoppingDistance(
                speed, yieldCtx->proxy->emergencyBraking(), gradient);

        double availDist = 0.0;
        if (detected.resourceNodeId != 0)
        {
            availDist = distanceToNode(
                network_,
                *yieldCtx->route,
                yieldCtx->currentTrackId,
                yieldCtx->proxy->position(),
                detected.resourceNodeId);
            if (availDist < 0.0)
            {
                // BUG-5 fix: -1.0 sentinel means track not in route — skip
                continue;
            }
        if (availDist == 0.0)
            {
                // Conflict node is exactly at current position — already passed
                continue;
            }
        }
        else
        {
            // Same-track (Rear-End) or opposing-track (Head-On) conflict
            const auto* otherCtx = (yieldCtx == ctxA) ? ctxB : ctxA;
            if (detected.type == conflict::ConflictType::HeadOn)
            {
                const auto* trk = network_.getTrack(yieldCtx->currentTrackId);
                const double trkLen = trk != nullptr ? trk->length() : 2000.0;
                availDist = std::max(15.0, trkLen - yieldCtx->proxy->position() - otherCtx->proxy->position());
            }
            else
            {
                // Rear-End: yieldCtx is trailing, otherCtx is lead
                availDist = std::max(15.0, otherCtx->proxy->position() - yieldCtx->proxy->position());
            }
        }
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
        // LOGIC-1 fix: same tiebreaker as Step 4
        const bool aHasPriority =
            prioA.higherThan(prioB) ||
            (prioA.priority == prioB.priority &&
             (ctxA->proxy->velocity() > ctxB->proxy->velocity() ||
              (ctxA->proxy->velocity() == ctxB->proxy->velocity() &&
               ctxA->proxy->id() < ctxB->proxy->id())));

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

        if (detected.resourceNodeId != 0)
        {
            // LOGIC-6 fix: cap reservation window to 30 s from now to avoid
            // locking a junction for up to 125 s based on distant horizons.
            constexpr double kMaxReservationLookahead = 30.0;
            const double reserveStart = currentTime + std::min(detected.firstConflictTime, kMaxReservationLookahead);
            const double reserveEnd   = currentTime + std::min(detected.lastConflictTime,  kMaxReservationLookahead);

            const bool reserved = reservations_.request(
                prioCtx->proxy->id(),
                zone,
                reserveStart,
                reserveEnd);

            if (!reserved)
            {
                // Resource already reserved by another priority movement.
                // The yielding train must definitively hold at approach signal!
                safety::SafetyCommand holdCmd;
                holdCmd.type = safety::SafetyCommandType::HoldAtSignal;
                holdCmd.trainId = yieldCtx->proxy->id();
                holdCmd.targetSpeed = 0.0;
                holdCmd.issuedAt = currentTime;
                holdCmd.riskScore = top.risk.score;
                result.commands.push_back(holdCmd);
                continue;
            }
            result.reservations = reservations_.reservations();
        }

        // Braking feasibility for yielding train
        const auto* yieldTrack = network_.getTrack(yieldCtx->currentTrackId);
        const double gradient =
            yieldTrack != nullptr ? yieldTrack->gradient() : 0.0;
        const double speed = std::max(0.0, yieldCtx->proxy->velocity());
        const double brakingDist =
            physics::KinematicsEngine::emergencyStoppingDistance(
                speed, yieldCtx->proxy->emergencyBraking(), gradient);

        double availDist = 0.0;
        if (detected.resourceNodeId != 0)
        {
            availDist = distanceToNode(
                network_,
                *yieldCtx->route,
                yieldCtx->currentTrackId,
                yieldCtx->proxy->position(),
                detected.resourceNodeId);
            if (availDist < 0.0)
            {
                // BUG-5 fix: -1.0 sentinel — track not in route, skip
                continue;
            }
            if (availDist == 0.0)
            {
                // Conflict node exactly at current position — already passed
                continue;
            }
        }
        else
        {
            const auto* otherCtx = (yieldCtx == ctxA) ? ctxB : ctxA;
            if (detected.type == conflict::ConflictType::HeadOn)
            {
                const auto* trk = network_.getTrack(yieldCtx->currentTrackId);
                const double trkLen = trk != nullptr ? trk->length() : 2000.0;
                availDist = std::max(15.0, trkLen - yieldCtx->proxy->position() - otherCtx->proxy->position());
            }
            else
            {
                // LOGIC-5 fix: position subtraction is only meaningful if both
                // trains are on the same track.  When they are on different
                // tracks estimate the gap as (yield remaining on its track) +
                // (lead position on its track).
                if (yieldCtx->currentTrackId == otherCtx->currentTrackId)
                {
                    availDist = std::max(15.0,
                        otherCtx->proxy->position() - yieldCtx->proxy->position());
                }
                else
                {
                    const auto* yTrk = network_.getTrack(yieldCtx->currentTrackId);
                    const double yRemaining = (yTrk != nullptr)
                        ? std::max(0.0, yTrk->length() - yieldCtx->proxy->position())
                        : 0.0;
                    availDist = std::max(15.0,
                        yRemaining + otherCtx->proxy->position());
                }
            }
        }

        // Use a 15m stopping clearance buffer before the fouling point
        constexpr double kSafetyClearanceBuffer = 15.0;
        const bool brakingFeasible =
            std::isfinite(brakingDist) &&
            brakingDist >= 0.0 &&
            availDist >= (brakingDist + kSafetyClearanceBuffer);

        auto riskForResolution = top.risk;
        // Pass the actual safety margin (availDist - brakingDist) to resolution.
        // The top.risk was computed in Step 4 with the real margin already;
        // just ensure it's consistent with what we computed here for feasibility.
        riskForResolution.safetyMargin = availDist - brakingDist;

        const safety::ResolutionInput resInput{
            *yieldCtx->proxy,
            detected,
            riskForResolution,
            false,          // yielding train has no priority
            brakingFeasible,
            availDist,
            brakingDist
        };

        auto command = resolutionEngine.resolve(resInput);
        if (detected.resourceNodeId != 0 &&
            (yieldCtx->proxy->state() == TrainState::Stopped ||
             yieldCtx->proxy->state() == TrainState::EmergencyBrake ||
             yieldCtx->proxy->velocity() < 0.1) &&
            command.type != safety::SafetyCommandType::EmergencyBrake)
        {
            command.type = safety::SafetyCommandType::HoldAtSignal;
            command.targetSpeed = 0.0;
        }
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
