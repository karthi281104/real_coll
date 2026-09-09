#include "orchestrator/ThreadOrchestrator.hpp"

#include "physics/KinematicsEngine.hpp"

#include <algorithm>
#include <iostream>

namespace tcas::orchestrator
{

ThreadOrchestrator::ThreadOrchestrator(
    const infrastructure::RailwayNetwork& network,
    train::TrainManager& trainManager,
    communication::CommunicationChannel& communicationChannel,
    std::vector<TrainId> trainIds,
    OrchestratorConfig config,
    SafetyStep safetyStep)
    : network_(network),
      trainManager_(trainManager),
      communicationChannel_(communicationChannel),
      trainIds_(std::move(trainIds)),
      config_(config),
    telemetryLogger_(config_.telemetryDirectory),
      safetyStep_(std::move(safetyStep))
{
    updateWorldSnapshotLocked();
}

ThreadOrchestrator::~ThreadOrchestrator()
{
    stop();
}

void ThreadOrchestrator::start()
{
    if (running_.exchange(true))
    {
        return;
    }
    paused_.store(false);

    {
        std::unique_lock lock(worldMutex_);
        worldState_.systemStatus = SystemStatus::Running;
    }

    physicsThread_ = std::thread(&ThreadOrchestrator::physicsLoop, this);
    safetyThread_ = std::thread(&ThreadOrchestrator::safetyLoop, this);
    communicationThread_ = std::thread(&ThreadOrchestrator::communicationLoop, this);
    hmiThread_ = std::thread(&ThreadOrchestrator::hmiLoop, this);
}

void ThreadOrchestrator::stop()
{
    if (!running_.exchange(false))
    {
        return;
    }
    paused_.store(false);

    {
        std::unique_lock lock(worldMutex_);
        worldState_.systemStatus = SystemStatus::Shutdown;
    }

    shutdownCondition_.notify_all();

    if (physicsThread_.joinable())
    {
        physicsThread_.join();
    }
    if (safetyThread_.joinable())
    {
        safetyThread_.join();
    }
    if (communicationThread_.joinable())
    {
        communicationThread_.join();
    }
    if (hmiThread_.joinable())
    {
        hmiThread_.join();
    }
}

void ThreadOrchestrator::pause()
{
    paused_.store(true);
    std::unique_lock lock(worldMutex_);
    worldState_.systemStatus = SystemStatus::Paused;
}

void ThreadOrchestrator::resume()
{
    paused_.store(false);
    std::unique_lock lock(worldMutex_);
    worldState_.systemStatus = SystemStatus::Running;
}

bool ThreadOrchestrator::isRunning() const noexcept
{
    return running_.load();
}

bool ThreadOrchestrator::isPaused() const noexcept
{
    return paused_.load();
}

WorldState ThreadOrchestrator::snapshot() const
{
    std::shared_lock lock(worldMutex_);
    return worldState_;
}

std::size_t ThreadOrchestrator::physicsCycles() const noexcept
{
    return physicsCycles_.load();
}

std::size_t ThreadOrchestrator::safetyCycles() const noexcept
{
    return safetyCycles_.load();
}

std::size_t ThreadOrchestrator::communicationCycles() const noexcept
{
    return communicationCycles_.load();
}

std::size_t ThreadOrchestrator::hmiCycles() const noexcept
{
    return hmiCycles_.load();
}

void ThreadOrchestrator::setSafetyStep(SafetyStep safetyStep)
{
    std::lock_guard lock(safetyStepMutex_);
    safetyStep_ = std::move(safetyStep);
}

void ThreadOrchestrator::postCommand(UserCommand command)
{
    std::lock_guard lock(userCommandMutex_);
    userCommandQueue_.push(std::move(command));
}

void ThreadOrchestrator::setSensorFault(TrainId trainId, bool fault)
{
    std::unique_lock lock(worldMutex_);
    if (fault)
    {
        failedSensors_.insert(trainId);
    }
    else
    {
        failedSensors_.erase(trainId);
    }
    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setSensorFault(bool fault)
{
    std::unique_lock lock(worldMutex_);
    if (fault)
    {
        for (const auto tid : trainIds_)
        {
            failedSensors_.insert(tid);
        }
    }
    else
    {
        failedSensors_.clear();
    }
    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setCommFault(bool fault)
{
    userCommFault_.store(fault);
    std::unique_lock lock(worldMutex_);
    worldState_.communicationFailure = userCommFault_.load() || commChannelDegraded_.load();
}

void ThreadOrchestrator::addTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    if (std::find(trainIds_.begin(), trainIds_.end(), trainId) == trainIds_.end())
    {
        trainIds_.push_back(trainId);
        updateWorldSnapshotLocked();
    }
}

void ThreadOrchestrator::removeTrain(TrainId trainId)
{
    std::unique_lock lock(worldMutex_);
    const auto it = std::find(trainIds_.begin(), trainIds_.end(), trainId);
    if (it != trainIds_.end())
    {
        trainIds_.erase(it);
        navStates_.erase(trainId);
        failedSensors_.erase(trainId);
        operatorSpeedLimits_.erase(trainId);
        safetySpeedLimits_.erase(trainId);
        dispatchSpeeds_.erase(trainId);
        trainsHeldBySafety_.erase(trainId);
        emergencyBrakeSet_.erase(trainId);
        trainManager_.removeTrain(trainId);
        updateWorldSnapshotLocked();
    }
}

void ThreadOrchestrator::setTrainRoute(
    TrainId trainId,
    TrackId startTrackId,
    navigation::RouteResult route)
{
    std::unique_lock lock(worldMutex_);
    TrainNavigationState nav;
    nav.trainId = trainId;
    nav.currentTrackId = startTrackId;
    nav.routeTrackIndex = 0;
    nav.route = std::move(route);

    for (std::size_t i = 0; i < nav.route.tracks.size(); ++i)
    {
        if (nav.route.tracks[i] == startTrackId)
        {
            nav.routeTrackIndex = i;
            break;
        }
    }
    navStates_[trainId] = std::move(nav);

    // BUG-1 fix: record the train's current velocity as its dispatch speed so
    // auto-resume always restores the correct catalog-assigned speed.
    if (!dispatchSpeeds_.contains(trainId))
    {
        const auto* train = trainManager_.getTrain(trainId);
        if (train != nullptr && train->velocity() > 0.0)
        {
            dispatchSpeeds_[trainId] = train->velocity();
        }
    }

    updateWorldSnapshotLocked();
}

void ThreadOrchestrator::setOperatorMessage(std::string message)
{
    std::unique_lock lock(worldMutex_);
    worldState_.operatorMessage = std::move(message);
}

void ThreadOrchestrator::waitUntil(
    const std::chrono::steady_clock::time_point next)
{
    std::unique_lock lock(shutdownMutex_);
    shutdownCondition_.wait_until(
        lock,
        next,
        [this]
        {
            return !running_.load();
        });
}

void ThreadOrchestrator::updateWorldSnapshotLocked()
{
    worldState_.trains.clear();
    worldState_.trains.reserve(trainIds_.size());

    for (const TrainId trainId : trainIds_)
    {
        const auto* train = trainManager_.getTrain(trainId);
        if (train == nullptr)
        {
            continue;
        }

        TrackId trackId = 0;
        const auto it = navStates_.find(trainId);
        if (it != navStates_.end())
        {
            trackId = it->second.currentTrackId;
        }

        const bool hasSensorFault = failedSensors_.contains(trainId);

        worldState_.trains.push_back({
            train->id(),
            train->type(),
            trackId,
            train->mass(),
            train->maximumSpeed(),
            train->serviceBraking(),
            train->emergencyBraking(),
            train->state(),
            train->position(),
            train->velocity(),
            train->acceleration(),
            hasSensorFault
        });
    }

    worldState_.sensorFailure = !failedSensors_.empty();
    worldState_.communicationFailure = userCommFault_.load() || commChannelDegraded_.load();
    worldState_.systemStatus = paused_.load()
        ? SystemStatus::Paused
        : ((safetyFailure_.load() || worldState_.sensorFailure ||
            worldState_.communicationFailure)
            ? SystemStatus::Degraded
            : (running_.load() ? SystemStatus::Running : SystemStatus::Shutdown));

    worldState_.timing.physicsCycles = physicsCycles_.load();
    worldState_.timing.safetyCycles = safetyCycles_.load();
    worldState_.timing.commCycles = communicationCycles_.load();
    worldState_.timing.hmiCycles = hmiCycles_.load();
    worldState_.timing.physicsPeriodTargetMs = static_cast<double>(config_.physicsPeriod.count());
    worldState_.timing.safetyPeriodTargetMs = static_cast<double>(config_.safetyPeriod.count());
    worldState_.timing.commPeriodTargetMs = static_cast<double>(config_.communicationPeriod.count());
    worldState_.timing.hmiPeriodTargetMs = static_cast<double>(config_.hmiPeriod.count());
    worldState_.timing.packetsSent = communicationChannel_.totalSent();
    worldState_.timing.packetsDelivered = communicationChannel_.totalDelivered();
    worldState_.timing.packetsDropped = communicationChannel_.totalDropped();
    if (worldState_.timing.packetsSent > 0)
    {
        worldState_.timing.packetDropRatePct = (100.0 * static_cast<double>(worldState_.timing.packetsDropped)) / static_cast<double>(worldState_.timing.packetsSent);
    }
}

void ThreadOrchestrator::processUserCommandsLocked()
{
    std::queue<UserCommand> commands;
    {
        std::lock_guard lock(userCommandMutex_);
        std::swap(commands, userCommandQueue_);
    }

    while (!commands.empty())
    {
        const auto cmd = std::move(commands.front());
        commands.pop();

        switch (cmd.type)
        {
        case UserCommandType::Start:
            paused_.store(false);
            worldState_.systemStatus = SystemStatus::Running;
            worldState_.operatorMessage = "[OK] Simulation RUNNING.";
            break;

        case UserCommandType::Pause:
            paused_.store(true);
            worldState_.systemStatus = SystemStatus::Paused;
            worldState_.operatorMessage = "[OK] Simulation PAUSED.";
            break;

        case UserCommandType::Resume:
            paused_.store(false);
            worldState_.systemStatus = SystemStatus::Running;
            worldState_.operatorMessage = "[OK] Simulation RESUMED.";
            break;

        case UserCommandType::SetSpeed:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                const double targetSpd = std::clamp(cmd.numericValue, 0.0, train->maximumSpeed());
                operatorSpeedLimits_[cmd.trainId] = targetSpd;
                dispatchSpeeds_[cmd.trainId] = targetSpd; // BUG-1: track dispatch speed
                safetySpeedLimits_.erase(cmd.trainId);
                trainsHeldBySafety_.erase(cmd.trainId);
                emergencyBrakeSet_.erase(cmd.trainId);    // operator reset clears E-brake
                train->setState(TrainState::Running);
                train->setVelocity(targetSpd);
                train->setAcceleration(0.5);
                worldState_.operatorMessage = "[OK] Speed for Train #" + std::to_string(cmd.trainId) +
                    " set to " + std::to_string(static_cast<int>(targetSpd)) + " m/s (Emergency released).";
            }
            break;

        case UserCommandType::HoldTrain:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
                train->setState(TrainState::Stopped);
                worldState_.operatorMessage = "[OK] Train #" + std::to_string(cmd.trainId) + " HELD at signal.";
            }
            break;

        case UserCommandType::ResumeTrain:
            if (auto* train = trainManager_.getTrain(cmd.trainId))
            {
                const double targetSpd = (cmd.numericValue > 0.0)
                    ? std::min(cmd.numericValue, train->maximumSpeed())
                    : std::min(20.0, train->maximumSpeed());
                operatorSpeedLimits_[cmd.trainId] = targetSpd;
                dispatchSpeeds_[cmd.trainId] = targetSpd; // BUG-1: track dispatch speed
                safetySpeedLimits_.erase(cmd.trainId);
                trainsHeldBySafety_.erase(cmd.trainId);
                emergencyBrakeSet_.erase(cmd.trainId);    // operator reset clears E-brake
                train->setState(TrainState::Running);
                train->setVelocity(targetSpd);
                train->setAcceleration(0.5);
                worldState_.operatorMessage = "[OK] Train #" + std::to_string(cmd.trainId) + " RESUMED.";
            }
            break;

        case UserCommandType::InjectSensorFailure:
            if (cmd.trainId != 0)
            {
                failedSensors_.insert(cmd.trainId);
            }
            else
            {
                for (auto tid : trainIds_) { failedSensors_.insert(tid); }
            }
            worldState_.sensorFailure = !failedSensors_.empty();
            worldState_.operatorMessage = "[FAULT] Sensor uncertainty expanded (+-50m) -> Earlier predictive safety margin.";
            break;

        case UserCommandType::RecoverSensor:
            if (cmd.trainId != 0)
            {
                failedSensors_.erase(cmd.trainId);
            }
            else
            {
                failedSensors_.clear();
            }
            worldState_.sensorFailure = !failedSensors_.empty();
            worldState_.operatorMessage = "[OK] Sensor faults cleared -> Precision tracking restored (1.0m).";
            break;

        case UserCommandType::InjectCommFailure:
            userCommFault_.store(true);
            worldState_.communicationFailure = true;
            worldState_.operatorMessage = "[FAULT] Wireless blackout active -> Fail-safe restricted speed (10 m/s) enforced.";
            break;

        case UserCommandType::RecoverComm:
            userCommFault_.store(false);
            worldState_.communicationFailure = commChannelDegraded_.load();
            for (const auto tid : trainIds_)
            {
                if (auto* train = trainManager_.getTrain(tid))
                {
                    if (train->state() == TrainState::Running)
                    {
                        train->setAcceleration(0.5);
                    }
                }
            }
            worldState_.operatorMessage = "[OK] Wireless comm link RESTORED -> Line speed restored, trains re-accelerating.";
            break;

        case UserCommandType::AddTrain:
            if (std::find(trainIds_.begin(), trainIds_.end(), cmd.trainId) == trainIds_.end())
            {
                trainIds_.push_back(cmd.trainId);
            }
            break;

        case UserCommandType::RemoveTrain:
            std::erase(trainIds_, cmd.trainId);
            navStates_.erase(cmd.trainId);
            failedSensors_.erase(cmd.trainId);
            operatorSpeedLimits_.erase(cmd.trainId);
            safetySpeedLimits_.erase(cmd.trainId);
            dispatchSpeeds_.erase(cmd.trainId);
            trainsHeldBySafety_.erase(cmd.trainId);
            emergencyBrakeSet_.erase(cmd.trainId);
            trainManager_.removeTrain(cmd.trainId); // Safe: always done under worldMutex_
            break;

        case UserCommandType::ChangeRoute:
            if (cmd.payload.has_value())
            {
                try
                {
                    const auto spec = std::any_cast<UserRouteSpec>(cmd.payload);
                    TrainNavigationState nav;
                    nav.trainId = spec.trainId;
                    nav.currentTrackId = spec.startTrackId;
                    nav.routeTrackIndex = 0;
                    nav.route = spec.route;
                    for (std::size_t i = 0; i < nav.route.tracks.size(); ++i)
                    {
                        if (nav.route.tracks[i] == spec.startTrackId)
                        {
                            nav.routeTrackIndex = i;
                            break;
                        }
                    }
                    navStates_[spec.trainId] = std::move(nav);
                }
                catch (...) {}
            }
            break;

        default:
            break;
        }
    }
}

void ThreadOrchestrator::physicsLoop()
{
    auto next = std::chrono::steady_clock::now();
    const double dt = static_cast<double>(config_.physicsPeriod.count()) / 1000.0;

    while (running_.load())
    {
        next += config_.physicsPeriod;

        std::unique_lock lock(worldMutex_);

        // 1. Drain user commands synchronously inside physics tick
        processUserCommandsLocked();

        // 2. If paused, do not advance kinematics or simulation time
        if (paused_.load())
        {
            updateWorldSnapshotLocked();
            lock.unlock();
            waitUntil(next);
            continue;
        }

        // 3. Process safety commands from SafetyPipeline
        safety::SafetyCommand command;
        while (commandQueue_.tryPop(&command))
        {
            auto* train = trainManager_.getTrain(command.trainId);
            if (train == nullptr)
            {
                continue;
            }

            switch (command.type)
            {
            case safety::SafetyCommandType::ReduceSpeed:
                // Graduate: target speed 50% current — set Slowing (service brake applied)
                safetySpeedLimits_[command.trainId] = std::min(
                    safetySpeedLimits_.contains(command.trainId)
                        ? safetySpeedLimits_[command.trainId]
                        : train->maximumSpeed(),
                    std::max(0.0, command.targetSpeed));
                if (train->state() != TrainState::EmergencyBrake &&
                    train->state() != TrainState::Stopped &&
                    train->state() != TrainState::Completed)
                {
                    train->setAcceleration(-train->serviceBraking());
                    train->setState(TrainState::Slowing);
                }
                break;
            case safety::SafetyCommandType::HoldAtSignal:
                safetySpeedLimits_[command.trainId] = 0.0;
                if (train->state() != TrainState::EmergencyBrake &&
                    train->state() != TrainState::Completed)
                {
                    train->setAcceleration(-train->serviceBraking());
                    train->setState(TrainState::Braking);
                    // Velocity will be zeroed in kinematics once it reaches 0
                }
                break;
            case safety::SafetyCommandType::EmergencyBrake:
                // LOGIC-4 fix: EmergencyBrake is sticky — requires explicit
                // operator reset (SetSpeed or ResumeTrain command) before the
                // train may move again.  It is tracked in emergencyBrakeSet_
                // and is NOT auto-resumed by the safety loop.
                safetySpeedLimits_[command.trainId] = 0.0;
                if (train->state() != TrainState::Completed)
                {
                    train->setVelocity(0.0);
                    train->setAcceleration(0.0);
                    train->setState(TrainState::EmergencyBrake);
                    emergencyBrakeSet_.insert(command.trainId);
                }
                break;
            case safety::SafetyCommandType::NoAction:
                break;
            }
        }

        // 4. Update kinematics & track transitions
        for (const TrainId trainId : trainIds_)
        {
            auto* train = trainManager_.getTrain(trainId);
            if (train == nullptr)
            {
                continue;
            }

            const auto newPosition = physics::KinematicsEngine::updatePosition(
                train->position(), train->velocity(), train->acceleration(), dt);
            const auto newVelocity = physics::KinematicsEngine::updateVelocity(
                train->velocity(), train->acceleration(), dt, train->maximumSpeed());

            // Track boundary checking
            auto navIt = navStates_.find(trainId);
            if (navIt != navStates_.end() && navIt->second.currentTrackId != 0)
            {
                auto& nav = navIt->second;
                const auto* curTrack = network_.getTrack(nav.currentTrackId);
                if (curTrack != nullptr)
                {
                    if (newPosition >= curTrack->length())
                    {
                        if (nav.routeTrackIndex + 1 < nav.route.tracks.size())
                        {
                            const double excess = newPosition - curTrack->length();
                            ++nav.routeTrackIndex;
                            nav.currentTrackId = nav.route.tracks[nav.routeTrackIndex];
                            train->setPosition(excess);
                        }
                        else
                        {
                            // BUG-3 fix: remove the redundant setState(Stopped)
                            // that was immediately overwritten by setState(Completed).
                            train->setPosition(curTrack->length());
                            train->setVelocity(0.0);
                            train->setAcceleration(0.0);
                            train->setState(TrainState::Completed);
                        }
                    }
                    else
                    {
                        train->setPosition(newPosition);
                    }
                }
                else
                {
                    train->setPosition(newPosition);
                }
            }
            else
            {
                train->setPosition(newPosition);
            }

            const double operatorLimit = operatorSpeedLimits_.contains(trainId)
                ? operatorSpeedLimits_[trainId]
                : train->maximumSpeed();
            double safetyLimit = safetySpeedLimits_.contains(trainId)
                ? safetySpeedLimits_[trainId]
                : train->maximumSpeed();
            if (worldState_.communicationFailure)
            {
                // Fail-safe restricted speed (10 m/s) under radio blackout
                safetyLimit = std::min(safetyLimit, 10.0);
            }
            if (train->state() == TrainState::EmergencyBrake ||
                train->state() == TrainState::Completed)
            {
                // Terminal states: hold at zero
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
            }
            else if (train->state() == TrainState::Stopped ||
                     (safetySpeedLimits_.contains(trainId) && safetySpeedLimits_[trainId] <= 0.0 &&
                      train->state() != TrainState::Braking &&
                      train->state() != TrainState::Slowing))
            {
                // Held at signal: stay zero
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
            }
            else
            {
                const double targetMaxSpeed = std::min({
                    operatorLimit,
                    safetyLimit,
                    train->maximumSpeed()});

                // SLOWING: service braking to target — transition to Running once at limit
                if (train->state() == TrainState::Slowing)
                {
                    const double effectiveSpeed = std::min(newVelocity, targetMaxSpeed);
                    train->setVelocity(effectiveSpeed);
                    if (train->velocity() <= safetyLimit + 0.1)
                    {
                        train->setAcceleration(0.0);
                        train->setState(TrainState::Running);
                    }
                }
                // BRAKING: approaching HoldAtSignal — transition to Stopped when velocity reaches 0
                else if (train->state() == TrainState::Braking)
                {
                    const double effectiveSpeed = std::max(0.0, newVelocity);
                    train->setVelocity(effectiveSpeed);
                    if (train->velocity() <= 0.1)
                    {
                        train->setVelocity(0.0);
                        train->setAcceleration(0.0);
                        train->setState(TrainState::Stopped);
                    }
                }
                // RUNNING: When there is no collision / safety intervention, accelerate and travel at max speed
                else if (train->state() == TrainState::Running)
                {
                    if (train->velocity() < targetMaxSpeed - 0.05)
                    {
                        train->setAcceleration(0.8);
                        train->setVelocity(std::min(newVelocity, targetMaxSpeed));
                    }
                    else
                    {
                        train->setVelocity(targetMaxSpeed);
                        train->setAcceleration(0.0);
                    }
                }
                else
                {
                    const double effectiveSpeed = std::min(newVelocity, targetMaxSpeed);
                    train->setVelocity(effectiveSpeed);
                }
            }
        }

        worldState_.simulationTime += dt;
        updateWorldSnapshotLocked();
        ++physicsCycles_;
        lock.unlock();
        waitUntil(next);
    }
}

void ThreadOrchestrator::safetyLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.safetyPeriod;

        if (paused_.load())
        {
            waitUntil(next);
            continue;
        }

        const WorldState state = snapshot();
        SafetyStep step;
        {
            std::lock_guard lock(safetyStepMutex_);
            step = safetyStep_;
        }
        if (step)
        {
            try
            {
                const SafetyCycleResult result = step(state);
                std::unique_lock lock(worldMutex_);
                worldState_.predictions    = result.predictions;
                worldState_.activeConflicts = result.activeConflicts;
                worldState_.reservations   = result.reservations;
                worldState_.commands       = result.commands;
                worldState_.decisions      = result.decisions;

                std::unordered_set<TrainId> commandedThisCycle;
                for (const auto& command : result.commands)
                {
                    commandQueue_.push(command);
                    commandedThisCycle.insert(command.trainId);
                    // Track ALL safety-restricted trains including EmergencyBrake
                    if (command.type == safety::SafetyCommandType::HoldAtSignal ||
                        command.type == safety::SafetyCommandType::ReduceSpeed ||
                        command.type == safety::SafetyCommandType::EmergencyBrake)
                    {
                        trainsHeldBySafety_.insert(command.trainId);
                    }
                }

                // Auto-Resume: If a train was held/slowing but no longer
                // has active conflicts or commands that cycle, step it back
                // toward Running.
                //
                // EmergencyBrake trains are in emergencyBrakeSet_ and are
                // NEVER auto-resumed here — they require an operator command.
                //
                // BUG-4 fix: only resume if the train is truly stopped
                // (Stopped or Slowing).  A train still in Braking has not yet
                // physically halted; resuming it creates a race where
                // safetySpeedLimits_ are cleared while the physics loop is
                // still decelerating the train.
                std::vector<TrainId> toResume;
                for (const auto tid : trainsHeldBySafety_)
                {
                    if (!commandedThisCycle.contains(tid))
                    {
                        bool inConflict = false;
                        for (const auto& c : result.activeConflicts)
                        {
                            if (c.trainA == tid || c.trainB == tid)
                            {
                                inConflict = true;
                                break;
                            }
                        }

                        // If the only active conflicts involving this train are with
                        // other stopped trains (deadlock resolution), allow resume
                        // so both sides can clear the junction.
                        if (inConflict)
                        {
                            bool onlyStoppedPartners = true;
                            for (const auto& c : result.activeConflicts)
                            {
                                if (c.trainA == tid || c.trainB == tid)
                                {
                                    if (c.type != conflict::ConflictType::Junction)
                                    {
                                        onlyStoppedPartners = false;
                                        break;
                                    }
                                    const TrainId partner = (c.trainA == tid) ? c.trainB : c.trainA;
                                    const auto* partnerTrain = trainManager_.getTrain(partner);
                                    if (partnerTrain == nullptr ||
                                        (partnerTrain->state() != TrainState::Stopped &&
                                         partnerTrain->state() != TrainState::EmergencyBrake))
                                    {
                                        onlyStoppedPartners = false;
                                        break;
                                    }
                                }
                            }
                            if (onlyStoppedPartners)
                            {
                                inConflict = false; // treat as clear (deadlock break)
                            }
                        }

                        if (!inConflict)
                        {
                            // Only resume once the train has fully stopped (not while decelerating in Braking)
                            auto* t = trainManager_.getTrain(tid);
                            if (t != nullptr &&
                                t->state() != TrainState::Braking)
                            {
                                toResume.push_back(tid);
                            }
                        }
                    }
                }

                for (const auto tid : toResume)
                {
                    auto* train = trainManager_.getTrain(tid);
                    if (train == nullptr)
                    {
                        trainsHeldBySafety_.erase(tid);
                        emergencyBrakeSet_.erase(tid);
                        continue;
                    }

                    // Full resume: clear safety hold, release emergency brake, and re-accelerate
                    trainsHeldBySafety_.erase(tid);
                    emergencyBrakeSet_.erase(tid);
                    safetySpeedLimits_.erase(tid);
                    train->setState(TrainState::Running);
                    train->setAcceleration(0.8);
                    if (train->velocity() < 1.0)
                    {
                        train->setVelocity(1.0);
                    }
                    worldState_.operatorMessage = "[AUTO-RESUME] Conflict cleared for Train #" +
                        std::to_string(tid) + " -> Signal cleared, re-accelerating to line speed.";
                }
            }
            catch (const std::exception&)
            {
                safetyFailure_.store(true);
                std::unique_lock lock(worldMutex_);
                worldState_.systemStatus = SystemStatus::Degraded;
            }
        }
        ++safetyCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::communicationLoop()
{
    auto next = std::chrono::steady_clock::now();
    while (running_.load())
    {
        next += config_.communicationPeriod;

        if (paused_.load())
        {
            waitUntil(next);
            continue;
        }

        const auto state = snapshot();
        const auto tick = static_cast<SimTimeTick>(
            std::max(0.0, state.simulationTime) * 1000.0);
        const auto sentBefore = communicationChannel_.totalSent();
        const auto deliveredBefore = communicationChannel_.totalDelivered();
        const auto droppedBefore = communicationChannel_.totalDropped();
        communicationChannel_.step(tick);
        {
            std::unique_lock lock(worldMutex_);
            const auto sent = communicationChannel_.totalSent() - sentBefore;
            const auto delivered = communicationChannel_.totalDelivered() - deliveredBefore;
            const auto dropped = communicationChannel_.totalDropped() - droppedBefore;

            // LOGIC-7 fix: use hysteresis so that a single zero-packet cycle
            // does not clear the degradation flag.
            // Degraded = more dropped than delivered in this cycle.
            // Only clear once a non-trivial good cycle is observed.
            bool degradedThisCycle = (sent > 0U && dropped > delivered);
            bool degraded = degradedThisCycle || (commDegradedPrev_ && sent == 0U);
            commDegradedPrev_ = degraded;

            commChannelDegraded_.store(degraded);
            worldState_.communicationFailure = userCommFault_.load() || degraded;
        }
        ++communicationCycles_;
        waitUntil(next);
    }
}

void ThreadOrchestrator::hmiLoop()
{
    auto next = std::chrono::steady_clock::now();

    while (running_.load())
    {
        next += config_.hmiPeriod;
        const auto started = std::chrono::steady_clock::now();
        const WorldState state = snapshot();
        telemetryLogger_.logSnapshot(state);
        telemetryLogger_.logConflicts(state);
        performanceMetrics_.observe(
            state,
            std::chrono::steady_clock::now() - started);
        if (config_.printHmi)
        {
            hmi::HmiDisplay::render(state, std::cout);
        }
        ++hmiCycles_;
        waitUntil(next);
    }
}

} // namespace tcas::orchestrator
