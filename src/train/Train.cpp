#include "train/Train.hpp"

#include <cmath>
#include <stdexcept>

namespace tcas::train
{

Train::Train(
    TrainId id,
    double mass,
    double maximumSpeed,
    double serviceBraking,
    double emergencyBraking
)
    : id_(id),
      mass_(mass),
      maximumSpeed_(maximumSpeed),
      serviceBraking_(serviceBraking),
      emergencyBraking_(emergencyBraking),
      position_(0.0),
      velocity_(0.0),
      acceleration_(0.0),
      state_(TrainState::Idle)
{
    if (!std::isfinite(mass) || mass <= 0.0)
    {
        throw std::invalid_argument(
            "Train mass must be finite and greater than zero"
        );
    }

    if (!std::isfinite(maximumSpeed) || maximumSpeed < 0.0)
    {
        throw std::invalid_argument(
            "Train maximum speed must be finite and non-negative"
        );
    }

    if (!std::isfinite(serviceBraking) || serviceBraking < 0.0)
    {
        throw std::invalid_argument(
            "Service braking must be finite and non-negative"
        );
    }

    if (!std::isfinite(emergencyBraking) || emergencyBraking < 0.0)
    {
        throw std::invalid_argument(
            "Emergency braking must be finite and non-negative"
        );
    }

    if (emergencyBraking < serviceBraking)
    {
        throw std::invalid_argument(
            "Emergency braking must be greater than or equal to service braking"
        );
    }
}

TrainId Train::id() const noexcept
{
    return id_;
}

double Train::mass() const noexcept
{
    return mass_;
}

double Train::maximumSpeed() const noexcept
{
    return maximumSpeed_;
}

double Train::serviceBraking() const noexcept
{
    return serviceBraking_;
}

double Train::emergencyBraking() const noexcept
{
    return emergencyBraking_;
}

double Train::position() const noexcept
{
    return position_;
}

double Train::velocity() const noexcept
{
    return velocity_;
}

double Train::acceleration() const noexcept
{
    return acceleration_;
}

TrainState Train::state() const noexcept
{
    return state_;
}

void Train::setPosition(double position)
{
    if (!std::isfinite(position) || position < 0.0)
    {
        throw std::invalid_argument(
            "Train position must be finite and non-negative"
        );
    }

    position_ = position;
}

void Train::setVelocity(double velocity)
{
    if (!std::isfinite(velocity) || velocity < 0.0)
    {
        throw std::invalid_argument(
            "Train velocity must be finite and non-negative"
        );
    }

    if (velocity > maximumSpeed_)
    {
        throw std::invalid_argument(
            "Train velocity cannot exceed maximum speed"
        );
    }

    velocity_ = velocity;
}

void Train::setAcceleration(double acceleration)
{
    if (!std::isfinite(acceleration))
    {
        throw std::invalid_argument(
            "Train acceleration must be finite"
        );
    }
    acceleration_ = acceleration;
}

void Train::setState(TrainState state) noexcept
{
    state_ = state;
}

} // namespace tcas::train