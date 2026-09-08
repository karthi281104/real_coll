#include "hmi/HmiDisplay.hpp"

#include "conflict/ConflictZone.hpp"

#include <iomanip>
#include <ostream>
#include <sstream>

namespace tcas::hmi
{
namespace
{

const char* trainTypeName(const TrainType type)
{
    switch (type)
    {
    case TrainType::Express: return "Express";
    case TrainType::Passenger: return "Passenger";
    case TrainType::Freight: return "Freight";
    }
    return "Unknown";
}

const char* trainStateName(const TrainState state)
{
    switch (state)
    {
    case TrainState::Idle: return "IDLE";
    case TrainState::Running: return "RUNNING";
    case TrainState::Slowing: return "SLOWING";
    case TrainState::Braking: return "BRAKING";
    case TrainState::Stopped: return "HOLD (SIG)";
    case TrainState::EmergencyBrake: return "EMERGENCY";
    case TrainState::Completed: return "ARRIVED";
    }
    return "UNKNOWN";
}

const char* conflictTypeName(const conflict::ConflictType type)
{
    switch (type)
    {
    case conflict::ConflictType::RearEnd: return "REAR-END";
    case conflict::ConflictType::HeadOn: return "HEAD-ON";
    case conflict::ConflictType::Junction: return "JUNCTION";
    case conflict::ConflictType::Platform: return "PLATFORM";
    }
    return "UNKNOWN";
}

const char* systemStatusName(const orchestrator::SystemStatus s)
{
    switch (s)
    {
    case orchestrator::SystemStatus::Ready:    return "READY";
    case orchestrator::SystemStatus::Running:  return "RUNNING";
    case orchestrator::SystemStatus::Paused:   return "PAUSED";
    case orchestrator::SystemStatus::Degraded: return "DEGRADED";
    case orchestrator::SystemStatus::Shutdown: return "SHUTDOWN";
    }
    return "UNKNOWN";
}

const char* commandName(const safety::SafetyCommandType type)
{
    switch (type)
    {
    case safety::SafetyCommandType::NoAction: return "NO ACTION";
    case safety::SafetyCommandType::ReduceSpeed: return "REDUCE SPEED";
    case safety::SafetyCommandType::HoldAtSignal: return "HOLD AT SIGNAL";
    case safety::SafetyCommandType::EmergencyBrake: return "EMERGENCY BRAKE";
    }
    return "UNKNOWN";
}

} // namespace

std::string HmiDisplay::format(const orchestrator::WorldState& state)
{
    std::ostringstream output;
    output << std::fixed << std::setprecision(2);
    output << "==============================================================\n";
    output << "                    TCAS CONTROL CENTER\n";
    output << "==============================================================\n\n";
    output << "SYSTEM STATUS : " << systemStatusName(state.systemStatus) << '\n';
    output << "SAFETY STATUS : " << (state.activeConflicts.empty() ? "SAFE" : "CONFLICT ACTIVE") << '\n';
    output << "SIMULATION    : " << state.simulationTime << " s\n";
    output << "COMMUNICATION : " << (state.communicationFailure ? "FAILED" : "OK") << '\n';
    output << "SENSORS       : " << (state.sensorFailure ? "FAILED" : "OK") << "\n\n";

    output << "--------------------------------------------------------------\n";
    output << "THREADS\n";
    output << "Physics : " << state.timing.physicsCycles
           << " | Safety : " << state.timing.safetyCycles
           << " | Comm : " << state.timing.commCycles
           << " | HMI : " << state.timing.hmiCycles << "\n";

    output << "--------------------------------------------------------------\n";
    output << "TRAIN STATUS\n";
    output << "--------------------------------------------------------------\n";
    output << "ID       TYPE       TRACK       POSITION    SPEED     STATE\n";
    for (const auto& train : state.trains)
    {
        output << std::setw(8) << train.id << ' '
               << std::setw(10) << trainTypeName(train.type) << ' '
               << std::setw(10) << train.trackId << ' '
               << std::setw(10) << train.position << " m  "
               << std::setw(8) << train.velocity << " m/s  "
               << trainStateName(train.state) << '\n';
    }

    output << "\n--------------------------------------------------------------\n";
    output << "ACTIVE CONFLICTS\n";
    output << "--------------------------------------------------------------\n";
    if (state.activeConflicts.empty())
    {
        output << "None\n";
    }
    for (const auto& conflict : state.activeConflicts)
    {
        output << "Type: " << conflictTypeName(conflict.type)
               << "  Node: " << conflict.resourceNodeId
               << "  Trains: " << conflict.trainA << " <-> " << conflict.trainB
               << "  TTC: " << conflict.firstConflictTime << " s"
               << "  MinSep: " << conflict.minimumSeparation << " m\n";
    }

    if (!state.decisions.empty())
    {
        output << "\n--------------------------------------------------------------\n";
        output << "SAFETY DECISIONS\n";
        output << "--------------------------------------------------------------\n";
        for (const auto& decision : state.decisions)
        {
            output << "Priority Train #" << decision.priorityTrain
                   << "  Yielding Train #" << decision.yieldingTrain
                   << "  Risk: " << decision.riskScore
                   << "  Command: " << commandName(decision.commandType) << '\n';
        }
    }

    output << "\n--------------------------------------------------------------\n";
    output << "JUNCTION RESERVATIONS\n";
    output << "--------------------------------------------------------------\n";
    if (state.reservations.empty())
    {
        output << "None\n";
    }
    for (const auto& reservation : state.reservations)
    {
        output << "Node " << reservation.zone.nodeId
               << " : owner Train #" << reservation.trainId
               << "  state=" << static_cast<int>(reservation.state) << '\n';
    }

    output << "\n--------------------------------------------------------------\n";
    output << "COMMANDS\n";
    output << "--------------------------------------------------------------\n";
    if (state.commands.empty())
    {
        output << "None\n";
    }
    for (const auto& command : state.commands)
    {
        output << "Train #" << command.trainId << " : "
               << commandName(command.type)
               << " -> " << command.targetSpeed << " m/s\n";
    }

    output << "\n--------------------------------------------------------------\n";
    output << "ACTION:\n";
    if (state.commands.empty())
    {
        output << "  Nominal operation (all trains clear)\n";
    }
    else
    {
        for (const auto& command : state.commands)
        {
            output << "Train #" << command.trainId << " -> "
                   << commandName(command.type)
                   << " -> " << command.targetSpeed << " m/s\n";
        }
    }

    output << "\nSYSTEM STATE : "
           << (state.activeConflicts.empty() ? "SAFE" : "CONFLICT ACTIVE") << '\n';

    if (!state.operatorMessage.empty())
    {
        output << "OPERATOR MSG  : " << state.operatorMessage << '\n';
    }
    if (!state.safetyError.empty())
    {
        output << "SAFETY ERROR  : " << state.safetyError << '\n';
    }

    output << "\n==============================================================\n"
           << "OPERATOR ACTIONS:\n"
           << "  [1] Start      [2] Pause      [3] Resume     [4] Add Train   [5] Remove Train\n"
           << "  [6] Set Speed  [7] Chg Route  [8] Hold Train [9] Resume Train\n"
           << " [10] Fault Sens [11] Recv Sens [12] Fault Comm [13] Recv Comm\n"
           << " [14] Conflicts  [15] Reserv    [16] Telemetry  [17] Metrics   [18] Reset   [19] Shutdown\n\n"
           << "DEMO SCENARIOS:\n"
           << " [24] Junction Conflict    [25] Rear-End Conflict   [26] Head-On Conflict\n"
           << " [27] Platform Conflict    [28] Multiple Conflicts  [29] Sensor Failure\n"
           << " [30] Comm Failure         [31] Unsafe Stopping Distance\n"
           << "Shortcuts: P=Pause, R=Resume, F=Sensor, C=Comm, S <id> <v>=Speed, H <id>=Hold, Q=Quit\n"
           << "==============================================================\n"
           << "Command > ";
    return output.str();
}

void HmiDisplay::render(
    const orchestrator::WorldState& state,
    std::ostream& output)
{
    output << "\033[H" << format(state) << "\033[J";
    output.flush();
}

} // namespace tcas::hmi
