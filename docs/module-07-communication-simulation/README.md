# Module 7 — Wireless Communication Simulation

## 1. Module Overview

Module 7 is the wireless vehicle-to-vehicle (V2V) and vehicle-to-infrastructure (V2I) communication simulation engine of the Train Collision Avoidance System (TCAS).

In modern railway networks (e.g., LTE-R, GSM-R, or IEEE 802.11p/DSRC), trains exchange real-time status data with neighboring trains and central dispatch controllers. Real radio signals, however, are subject to physical phenomena:
- Wireless propagation delay (latency)
- Signal attenuation over long distances (range limits)
- Packet corruption and drops due to electromagnetic interference, tunnels, or foliage
- Mailbox queues where incoming messages wait until the target system processes them

Module 7 simulates these realistic network conditions deterministically, enabling the TCAS safety pipeline to be thoroughly tested against communication lag, packet loss, and link degradation.

---

## 2. Objectives

1. **V2V and V2I Message Exchange**: Enable structured, type-safe messaging between trains and trackside stations.
2. **Deterministic & Probabilistic Latency**: Simulate propagation delays as a configurable number of simulation ticks.
3. **Range Filtering**: Enforce physical wireless propagation limits; messages transmitted beyond `maxRangeMeters` are automatically dropped.
4. **Packet Loss & Drop Modeling**: Support both random probabilistic drop rates ($0.0$ to $1.0$) and deterministic modulo drops (e.g., drop every $N$-th message) for repeatable automated testing.
5. **Mailbox Delivery System**: Provide distinct mailboxes for each train and infrastructure entity with support for both targeted Unicast and fleet-wide Broadcast.
6. **Diagnostics & Degradation Tracking**: Continuously track sent, delivered, and dropped counts, computing live delivery rates to inform safety supervisors of degraded channels.

---

## 3. Architectural Role & System Seams

```text
  Train Telemetry / Movement Authorities / Emergency Brakes
                             │
                             ▼
                tcas::communication::Message
       ├── Header (ID, Sender, Recipient, Tick, Priority)
       └── Payload (std::variant<Telemetry, MA, Brake, ...>)
                             │
                             ▼
                CommunicationChannel Simulator
        ├── Range Check (senderPos <-> recipientPos <= maxRange)
        ├── Packet Drop Check (random loss or modulo drop)
        ├── Delay Queue (deliveryTick = sentTick + latencyTicks)
        └── Mailbox Distribution (Unicast or Broadcast)
                             │
       ┌─────────────────────┴─────────────────────┐
       ▼                                           ▼
[ThreadOrchestrator Comms Loop]           [SafetyPipeline]
 (Steps channel every 100ms,              (Exchanges telemetry and
  delivers arrived messages)               reservation requests)
```

---

## 4. Implementation Details

### 4.1 Message Types and Payloads
Messages are strongly typed and encapsulated using `std::variant`:

| Message Type | Payload Structure | Priority | Description |
|---|---|---|---|
| `Heartbeat` | `HeartbeatPayload` | `Low` (0) | Periodic liveness beacon indicating subsystem health |
| `Telemetry` | `TelemetryPayload` | `Normal` (1) | Train position, velocity, active track, and operational state |
| `MovementAuthority` | `MovementAuthorityPayload` | `High` (2) | Target speed and distance limits granted by dispatch |
| `EmergencyBrake` | `EmergencyBrakePayload` | `Emergency` (3) | Immediate stop order with danger zone bounds |
| `ReservationRequest` | `ReservationPayload` | `Normal` (1) | Request to lock a junction or track segment |
| `ReservationResponse`| `ReservationPayload` | `High` (2) | Grant or denial of requested railway resource |

### 4.2 Transmission & Delivery Mechanics
1. **`sendMessage(msg, senderPos, recipientPos)`**:
   - If recipient is not broadcast (`0xFFFFFFFF`), calculates physical distance: $| \text{senderPos} - \text{recipientPos} |$. If distance $> \text{maxRangeMeters}$, the message is dropped.
   - Evaluates packet loss: checks deterministic drop modulo first; if zero, samples uniform random distribution against `packetLossRate`.
   - If accepted, stamps `deliveryTick = currentTick + latencyTicks` and places the message into the `inFlight_` queue.
2. **`step(currentTick)`**:
   - Iterates through `inFlight_` messages.
   - For all messages where `currentTick >= deliveryTick`, routes them into the recipient's mailbox `mailboxes_[recipientId]`.
   - If recipient is `kBroadcastRecipientId`, duplicates the message into every registered entity's mailbox.
3. **`receiveMessages(recipientId)`**:
   - Moves and clears all arrived messages for that entity.

### 4.3 Channel Degradation & Hysteresis (LOGIC-7 Implementation)
When the delivery rate falls below acceptable thresholds (or when fault injection is enabled), the channel is flagged as `commChannelDegraded_`. To prevent high-frequency toggling (chattering), hysteresis is maintained:
- Enters degraded state when delivery rate drops below $70\%$ (or manual fault injected).
- Remains degraded until delivery rate reliably recovers above $85\%$.

---

## 5. Public API Reference

```cpp
namespace tcas::communication {

inline constexpr std::uint32_t kBroadcastRecipientId = 0xFFFFFFFF;

struct ChannelConfig {
    SimTimeTick latencyTicks{ 1 };
    double packetLossRate{ 0.0 };
    DistanceMeters maxRangeMeters{ 10000.0 };
    std::uint32_t deterministicDropModulo{ 0 };
};

class CommunicationChannel {
public:
    explicit CommunicationChannel(const ChannelConfig& config = {});

    void registerEntity(std::uint32_t entityId);
    void unregisterEntity(std::uint32_t entityId);

    bool sendMessage(
        Message message,
        DistanceMeters senderPosition = 0.0,
        DistanceMeters recipientPosition = 0.0
    );

    void step(SimTimeTick currentTick);

    [[nodiscard]] bool hasMessages(std::uint32_t recipientId) const noexcept;
    [[nodiscard]] std::vector<Message> receiveMessages(std::uint32_t recipientId);

    [[nodiscard]] std::size_t totalSent() const noexcept;
    [[nodiscard]] std::size_t totalDelivered() const noexcept;
    [[nodiscard]] std::size_t totalDropped() const noexcept;
    [[nodiscard]] std::size_t inFlightCount() const noexcept;
    [[nodiscard]] double deliveryRate() const noexcept;

    void clear() noexcept;
    void clearMailboxes() noexcept;
};

} // namespace tcas::communication
```

---

## 6. Execution Flow

```text
[Train 1 produces Telemetry]
              │
              ▼
   sendMessage(msg, pos1, pos2)
              │
      [Range <= 10km?]
      ├── NO  ──► totalDropped++, return false
      └── YES
              │
      [Packet Loss Check]
      ├── DROPPED ──► totalDropped++, return false
      └── ACCEPTED ──► In-Flight Queue (deliveryTick = tick + 1)
                             │
                 [100ms Communication Loop]
                             │
                             ▼
                  Channel::step(tick)
                             │
              [deliveryTick <= tick?]
              ├── NO  ──► Wait in flight
              └── YES ──► Route to Train 2 Mailbox
                                   │
                                   ▼
                 Train 2::receiveMessages()
```

---

## 7. Edge Cases Handled

1. **Out-of-Range Transmission**: Radio signals drop off when trains are separated by $>10\text{ km}$, preventing stale remote data from clogging local collision processing.
2. **Broadcast to Unregistered Entities**: Broadcast messages are only routed to active, registered trains, preventing memory leaks from abandoned entities.
3. **Empty Mailbox Retrieval**: `receiveMessages()` on an empty mailbox returns an empty `std::vector` without allocation overhead.
4. **Division by Zero Protection**: `deliveryRate()` returns `1.0` when zero messages have been sent.

---

## 8. Automated Test Verification

Validated through `tests/communication/MessageTest.cpp` and `tests/communication/CommunicationChannelTest.cpp`:

- Message factory construction and payload variant unpacking
- Exact tick latency delay across discrete `step()` invocations
- Range boundary cut-off enforcement at $10,000\text{ m}$
- Deterministic modulo packet dropping ($1$-in-$3$ drop assertions)
- Multi-subscriber broadcast fan-out

All tests pass with 100% success rate.

---

## 9. Layman's Terms Description (What Does This Module Do?)

Imagine trains talking to each other and to the station control tower over walkie-talkies:

If you are shouting into a radio in real life:
- **Lag (Latency)**: Your voice takes a fraction of a second to travel through the air.
- **Range Limit**: If another train is 20 miles away on the other side of a mountain, they can't hear you at all.
- **Static & Dropped Words (Packet Loss)**: If you go through a tunnel or lightning strikes nearby, part of your message gets crackled and lost.
- **Mailboxes**: If the dispatcher is busy looking at a screen, your message waits in their inbox until they check it.

**Module 7 is the "Radio & Cellular Network Simulator" for the trains.**

It simulates real-world wireless radios. When Train 1 says: *"I'm braking at Mile Marker 42!"*, Module 7 makes sure the message travels with realistic radio delay, checks if the trains are within antenna range, and even introduces simulated static or packet loss. 

This guarantees that the safety system works reliably even when the cellular or radio signal is spotty or lagging.
