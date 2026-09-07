#pragma once

#include "common/Types.hpp"
#include "communication/Message.hpp"

#include <cstdint>
#include <deque>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tcas::communication
{

struct ChannelConfig
{
    SimTimeTick    latencyTicks{ 1 };          // Propagation delay in simulation ticks
    double         packetLossRate{ 0.0 };       // Probabilistic drop rate [0.0, 1.0]
    DistanceMeters maxRangeMeters{ 10000.0 };   // Maximum wireless transmission range
    std::uint32_t  deterministicDropModulo{ 0 }; // If > 0, drops every N-th packet deterministically
};

// Realistic Wireless V2V / V2I Communication Channel Simulator.
//
// Models transmission latency, packet drops, range cut-offs,
// and broadcast / unicast mailbox distribution.
class CommunicationChannel
{
public:
    explicit CommunicationChannel(const ChannelConfig& config = {});

    // Register active entity (TrainId or Station/NodeId) to receive broadcasts
    void registerEntity(std::uint32_t entityId);

    void unregisterEntity(std::uint32_t entityId);

    // Transmit a message across the wireless channel
    // Returns true if message was accepted into transit, false if dropped (loss or range)
    bool sendMessage(
        Message message,
        DistanceMeters senderPosition = 0.0,
        DistanceMeters recipientPosition = 0.0
    );

    // Advance channel simulation time; delivers ready messages to mailboxes
    void step(SimTimeTick currentTick);

    // Check if recipient has pending delivered messages
    [[nodiscard]]
    bool hasMessages(std::uint32_t recipientId) const noexcept;

    // Retrieve and consume all pending delivered messages for recipient
    [[nodiscard]]
    std::vector<Message> receiveMessages(std::uint32_t recipientId);

    // Diagnostics & Statistics
    [[nodiscard]]
    std::size_t totalSent() const noexcept;

    [[nodiscard]]
    std::size_t totalDelivered() const noexcept;

    [[nodiscard]]
    std::size_t totalDropped() const noexcept;

    [[nodiscard]]
    std::size_t inFlightCount() const noexcept;

    // Delivery rate ratio (totalDelivered / totalSent). Returns 1.0 when no messages have been sent.
    [[nodiscard]]
    double deliveryRate() const noexcept;

    // Dynamically adjust packet loss rate [0.0, 1.0]
    void setPacketLossRate(double rate) noexcept;

    [[nodiscard]]
    double packetLossRate() const noexcept;

    // Reset all queues and counters (full channel reset including entity registrations)
    void clear() noexcept;

    // Soft reset: clear in-flight messages and mailboxes but retain entity registrations
    void clearMailboxes() noexcept;

private:
    ChannelConfig config_;

    std::unordered_set<std::uint32_t> registeredEntities_;
    std::vector<Message>              inFlight_;
    std::unordered_map<std::uint32_t, std::vector<Message>> mailboxes_;

    std::size_t totalSent_{ 0 };
    std::size_t totalDelivered_{ 0 };
    std::size_t totalDropped_{ 0 };
    std::mt19937 rng_{ 42 };
};

} // namespace tcas::communication
