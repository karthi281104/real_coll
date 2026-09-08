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
                safetySpeedLimits_.erase(cmd.trainId);
                trainsHeldBySafety_.erase(cmd.trainId);
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
                safetySpeedLimits_.erase(cmd.trainId);
                trainsHeldBySafety_.erase(cmd.trainId);
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
            break;

        case UserCommandType::InjectCommFailure:
            userCommFault_.store(true);
            worldState_.communicationFailure = true;
            break;

        case UserCommandType::RecoverComm:
            userCommFault_.store(false);
            worldState_.communicationFailure = commChannelDegraded_.load();
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
                safetySpeedLimits_[command.trainId] = std::min(
                    safetySpeedLimits_.contains(command.trainId)
                        ? safetySpeedLimits_[command.trainId]
                        : train->maximumSpeed(),
                    std::max(0.0, command.targetSpeed));
                train->setVelocity(std::min(
                    train->velocity(), safetySpeedLimits_[command.trainId]));
                train->setAcceleration(std::min(train->acceleration(), 0.0));
                break;
            case safety::SafetyCommandType::HoldAtSignal:
            case safety::SafetyCommandType::EmergencyBrake:
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
                train->setState(command.isEmergency()
                    ? TrainState::EmergencyBrake
                    : TrainState::Braking);
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
                            train->setPosition(curTrack->length());
                            train->setVelocity(0.0);
                            train->setAcceleration(0.0);
                            train->setState(TrainState::Stopped);
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
            if (train->state() == TrainState::Stopped)
            {
                train->setVelocity(0.0);
                train->setAcceleration(0.0);
            }
            else
            {
                train->setVelocity(std::min({
                    newVelocity,
                    operatorLimit,
                    safetyLimit,
                    train->maximumSpeed()}));
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
                {
                    std::unique_lock lock(worldMutex_);
                    worldState_.predictions = result.predictions;
                    worldState_.activeConflicts = result.activeConflicts;
                    worldState_.reservations = result.reservations;
                    worldState_.commands = result.commands;
                    worldState_.decisions = result.decisions;
                }
                std::unordered_set<TrainId> commandedThisCycle;
                for (const auto& command : result.commands)
                {
                    commandQueue_.push(command);
                    commandedThisCycle.insert(command.trainId);
                    if (command.type == safety::SafetyCommandType::HoldAtSignal ||
                        command.type == safety::SafetyCommandType::ReduceSpeed)
                    {
                        trainsHeldBySafety_.insert(command.trainId);
                    }
                }

                // Auto-Resume: If a train was held at a signal / speed reduction but no longer has active conflicts/commands, resume it
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
                        if (!inConflict)
                        {
                            toResume.push_back(tid);
                        }
                    }
                }

                for (const auto tid : toResume)
                {
                    trainsHeldBySafety_.erase(tid);
                    safetySpeedLimits_.erase(tid);
                    if (auto* train = trainManager_.getTrain(tid))
                    {
                        if (train->state() == TrainState::Braking ||
                            train->state() == TrainState::Stopped)
                        {
                            train->setState(TrainState::Running);
                            const double targetSpd = operatorSpeedLimits_.contains(tid)
                                ? operatorSpeedLimits_[tid]
                                : std::min(20.0, train->maximumSpeed());
                            train->setVelocity(std::max(train->velocity(), std::min(15.0, targetSpd)));
                            train->setAcceleration(0.5);
                        }
                    }
                    std::unique_lock lock(worldMutex_);
                    worldState_.operatorMessage = "[AUTO-RESUME] Conflict cleared for Train #" +
                        std::to_string(tid) + " -> Signal cleared, re-accelerating.";
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
            const bool degraded = (sent > 0U && dropped > delivered);
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
