#include "communication/CommunicationChannel.hpp"

#include <algorithm>
#include <cmath>

namespace tcas::communication
{

CommunicationChannel::CommunicationChannel(const ChannelConfig& config)
    : config_(config)
{
}

void CommunicationChannel::registerEntity(const std::uint32_t entityId)
{
    registeredEntities_.insert(entityId);
    mailboxes_.try_emplace(entityId);
}

void CommunicationChannel::unregisterEntity(const std::uint32_t entityId)
{
    registeredEntities_.erase(entityId);
    mailboxes_.erase(entityId);
}

bool CommunicationChannel::sendMessage(
    Message message,
    const DistanceMeters senderPosition,
    const DistanceMeters recipientPosition
)
{
    ++totalSent_;

    // 1. Recipient registration check for unicast messages
    if (message.header.recipientId != kBroadcastRecipientId &&
        !registeredEntities_.contains(message.header.recipientId))
    {
        ++totalDropped_;
        return false;
    }

    // 2. Range verification check
    const double distance = std::abs(senderPosition - recipientPosition);
    if (config_.maxRangeMeters > 0.0 && distance > config_.maxRangeMeters)
    {
        ++totalDropped_;
        return false;
    }

    // 3. Deterministic drop modulo check (if enabled)
    if (config_.deterministicDropModulo > 0 &&
        (totalSent_ % config_.deterministicDropModulo == 0))
    {
        ++totalDropped_;
        return false;
    }

    // 4. Probabilistic packet loss check
    if (config_.packetLossRate > 0.0)
    {
        if (config_.packetLossRate >= 1.0)
        {
            ++totalDropped_;
            return false;
        }
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        if (dist(rng_) < config_.packetLossRate)
        {
            ++totalDropped_;
            return false;
        }
    }

    // Schedule arrival tick
    message.header.deliveryTick = message.header.sentTick + config_.latencyTicks;
    inFlight_.push_back(std::move(message));

    return true;
}

void CommunicationChannel::step(const SimTimeTick currentTick)
{
    if (inFlight_.empty())
    {
        return;
    }

    std::vector<Message> remainingInFlight;
    remainingInFlight.reserve(inFlight_.size());

    for (auto& msg : inFlight_)
    {
        if (msg.header.deliveryTick <= currentTick)
        {
            // Deliver message to recipient mailbox or broadcast
            if (msg.header.recipientId == kBroadcastRecipientId)
            {
                // Deliver copy to every registered entity except sender
                for (const std::uint32_t entityId : registeredEntities_)
                {
                    if (entityId != msg.header.senderId)
                    {
                        mailboxes_[entityId].push_back(msg);
                    }
                }
            }
            else
            {
                mailboxes_[msg.header.recipientId].push_back(msg);
            }

            ++totalDelivered_;
        }
        else
        {
            remainingInFlight.push_back(std::move(msg));
        }
    }

    inFlight_ = std::move(remainingInFlight);
}

bool CommunicationChannel::hasMessages(const std::uint32_t recipientId) const noexcept
{
    const auto it = mailboxes_.find(recipientId);
    if (it == mailboxes_.end())
    {
        return false;
    }

    return !it->second.empty();
}

std::vector<Message> CommunicationChannel::receiveMessages(const std::uint32_t recipientId)
{
    const auto it = mailboxes_.find(recipientId);
    if (it == mailboxes_.end() || it->second.empty())
    {
        return {};
    }

    std::vector<Message> delivered = std::move(it->second);
    it->second.clear();

    return delivered;
}

std::size_t CommunicationChannel::totalSent() const noexcept
{
    return totalSent_;
}

std::size_t CommunicationChannel::totalDelivered() const noexcept
{
    return totalDelivered_;
}

std::size_t CommunicationChannel::totalDropped() const noexcept
{
    return totalDropped_;
}

std::size_t CommunicationChannel::inFlightCount() const noexcept
{
    return inFlight_.size();
}

double CommunicationChannel::deliveryRate() const noexcept
{
    if (totalSent_ == 0)
    {
        return 1.0;
    }

    return static_cast<double>(totalDelivered_) / static_cast<double>(totalSent_);
}

void CommunicationChannel::setPacketLossRate(const double rate) noexcept
{
    config_.packetLossRate = std::clamp(rate, 0.0, 1.0);
}

double CommunicationChannel::packetLossRate() const noexcept
{
    return config_.packetLossRate;
}

void CommunicationChannel::clear() noexcept
{
    inFlight_.clear();
    mailboxes_.clear();
    registeredEntities_.clear();
    totalSent_ = 0;
    totalDelivered_ = 0;
    totalDropped_ = 0;
}

void CommunicationChannel::clearMailboxes() noexcept
{
    inFlight_.clear();
    // Re-create empty mailboxes for all still-registered entities
    for (const std::uint32_t entityId : registeredEntities_)
    {
        mailboxes_[entityId].clear();
    }
    totalSent_ = 0;
    totalDelivered_ = 0;
    totalDropped_ = 0;
}

} // namespace tcas::communication
