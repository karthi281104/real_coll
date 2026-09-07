#pragma once

#include "common/Types.hpp"
#include "navigation/RouteResult.hpp"

#include <any>
#include <memory>
#include <string>

namespace tcas::orchestrator
{

enum class UserCommandType
{
    Start,
    Pause,
    Resume,
    Reset,
    Shutdown,

    AddTrain,
    RemoveTrain,
    SetSpeed,
    ChangeRoute,
    HoldTrain,
    ResumeTrain,

    InjectSensorFailure,
    RecoverSensor,
    InjectCommFailure,
    RecoverComm,

    LoadScenario
};

struct UserTrainSpec
{
    TrainId id{ 0 };
    TrainType type{ TrainType::Express };
    double initialPosition{ 0.0 };
    double initialVelocity{ 0.0 };
    TrackId startTrackId{ 0 };
    navigation::RouteResult route;
};

struct UserRouteSpec
{
    TrainId trainId{ 0 };
    TrackId startTrackId{ 0 };
    navigation::RouteResult route;
};

struct UserCommand
{
    UserCommandType type{ UserCommandType::Start };
    TrainId trainId{ 0 };
    double numericValue{ 0.0 };
    std::string textValue;
    std::any payload;

    UserCommand() = default;

    UserCommand(UserCommandType t)
        : type(t) {}

    UserCommand(UserCommandType t, TrainId tid)
        : type(t), trainId(tid) {}

    UserCommand(UserCommandType t, TrainId tid, double num)
        : type(t), trainId(tid), numericValue(num) {}

    UserCommand(UserCommandType t, TrainId tid, double num, std::string text, std::any p = {})
        : type(t), trainId(tid), numericValue(num), textValue(std::move(text)), payload(std::move(p)) {}
};

} // namespace tcas::orchestrator
