# Module 2 — Train Entity Manager

## 1. Module Overview

The Train Entity Manager is responsible for representing, managing, and controlling train entities within the Real-Time Train Collision Avoidance System (TCAS).

This module establishes the object-oriented model for different train types and provides a centralized manager for creating, storing, retrieving, and removing trains.

The module is designed to support the later predictive safety modules, where train properties such as velocity, braking capability, mass, and operational state are required for collision prediction and conflict resolution.

---

## 2. Objectives

The primary objectives of Module 2 are:

- Represent trains using an object-oriented design.
- Provide a common abstract `Train` base class.
- Support different train types through inheritance.
- Implement polymorphic train behavior.
- Store important physical and operational properties.
- Validate train parameters during construction.
- Maintain train operational state.
- Provide centralized train lifecycle management.
- Prevent duplicate train IDs.
- Provide safe train lookup and removal.
- Establish a reliable foundation for the physics and safety modules.

---

## 3. Supported Train Types

The POC currently supports three train categories:

### Express Train

Represents high-speed passenger/express services.

Expected characteristics include:

- Higher operating speed.
- Train-specific braking characteristics.
- Express classification.

### Passenger Train

Represents normal passenger services.

Expected characteristics include:

- Moderate operating speed.
- Passenger-service braking characteristics.
- Passenger classification.

### Freight Train

Represents heavy freight services.

Expected characteristics include:

- High mass.
- Lower operating speed compared with express services.
- Different braking characteristics.
- Freight classification.

The implementation uses inheritance so that all train types can be handled through the common `Train` interface.

---

## 4. Object-Oriented Design

The module follows the following hierarchy:

```text
                    Train
                 (Abstract)
                     |
          +----------+----------+
          |          |          |
      Express    Passenger    Freight
       Train       Train        Train
```

`Train` provides the common properties and behavior.

Derived classes identify the specific train type through the virtual `type()` method.

---

## 5. Main Components

### Train

The base class represents common train characteristics.

Important properties include:

- Train ID
- Mass
- Maximum speed
- Service braking capability
- Emergency braking capability
- Position
- Velocity
- Acceleration
- Operational state

The class is abstract because the train type is provided by derived classes.

---

### ExpressTrain

Derived from `Train`.

Responsibilities:

- Represent an express train.
- Provide `TrainType::Express`.
- Preserve the common train interface.

---

### PassengerTrain

Derived from `Train`.

Responsibilities:

- Represent a passenger train.
- Provide `TrainType::Passenger`.
- Preserve the common train interface.

---

### FreightTrain

Derived from `Train`.

Responsibilities:

- Represent a freight train.
- Provide `TrainType::Freight`.
- Preserve the common train interface.

---

### TrainManager

The `TrainManager` provides centralized lifecycle management.

Responsibilities:

- Add trains.
- Find trains by ID.
- Check whether a train exists.
- Remove trains.
- Clear all trains.
- Reject duplicate IDs.
- Reject null train objects.
- Support multiple train types.

Internally, trains are managed using smart pointers to ensure RAII-based lifetime management.

---

## 6. Common Types

Shared types are defined centrally in:

```text
include/common/Types.hpp
```

Examples include:

```text
TrainId
TrainType
TrainState
```

Infrastructure-related types are also maintained in the common type layer.

This prevents individual modules from defining duplicate domain types.

---

## 7. Train State Model

The train state represents the current operational condition of a train.

The POC supports states such as:

```text
Idle
Running
Braking
Stopped
EmergencyBrake
```

The state will become particularly important in later modules.

For example:

```text
Conflict detected
       |
       v
Safety Engine
       |
       v
Train state = Braking
       |
       v
Physics Engine
```

For a critical conflict:

```text
Critical Conflict
       |
       v
EmergencyBrake
```

---

## 8. Input Validation

The module validates important physical parameters.

Invalid configurations are rejected.

Examples:

- Zero mass.
- Negative mass.
- Negative maximum speed.
- Negative service braking.
- Negative emergency braking.
- Emergency braking lower than service braking.
- Negative position.
- Velocity greater than maximum permitted speed.

This prevents invalid train states from entering the simulation.

---

## 9. Memory Management

The module uses modern C++ ownership semantics.

`TrainManager` stores trains using:

```text
std::unique_ptr<Train>
```

This provides:

- Automatic memory management.
- Clear ownership.
- RAII.
- No manual `delete`.
- Polymorphic storage of derived train classes.

---

## 10. Integration With Other Modules

Module 2 provides train information to future modules.

```text
Module 1
Railway Infrastructure
        |
        v
Module 2
Train Entity Manager
        |
        +----------------+
        |                |
        v                v
Module 3             Module 4
Simulation Clock     Kinematics
        |                |
        +--------+-------+
                 |
                 v
             Safety Engine
```

Future modules will use:

- Train position.
- Train velocity.
- Train acceleration.
- Train type.
- Train mass.
- Braking capability.
- Train state.

---

## 11. Files Implemented

### Headers

```text
include/train/Train.hpp
include/train/ExpressTrain.hpp
include/train/PassengerTrain.hpp
include/train/FreightTrain.hpp
include/train/TrainManager.hpp
```

### Source files

```text
src/train/Train.cpp
src/train/ExpressTrain.cpp
src/train/PassengerTrain.cpp
src/train/FreightTrain.cpp
src/train/TrainManager.cpp
```

### Tests

```text
tests/train/TrainTest.cpp
tests/train/ExpressTrainTest.cpp
tests/train/PassengerTrainTest.cpp
tests/train/FreightTrainTest.cpp
tests/train/TrainManagerTest.cpp
```

---

## 12. Completion Criteria

Module 2 is considered complete when:

- [x] Base Train class implemented.
- [x] Train inheritance implemented.
- [x] Express train implemented.
- [x] Passenger train implemented.
- [x] Freight train implemented.
- [x] Polymorphism verified.
- [x] Train state management implemented.
- [x] Physical parameter validation implemented.
- [x] TrainManager implemented.
- [x] Duplicate ID protection implemented.
- [x] Null train protection implemented.
- [x] Train lookup implemented.
- [x] Train removal implemented.
- [x] Multiple train types supported.
- [x] GoogleTest coverage implemented.
- [x] Module 1 regression tests continue to pass.

---

## 13. Verification Result

Module 2 was compiled using:

```text
C++23
MSVC 19.42
CMake
GoogleTest
```

Final verification:

```text
48 tests passed
0 tests failed
```

Module 1:

```text
17/17 passed
```

Module 2:

```text
31/31 passed
```

Overall:

```text
100% tests passed
```

Therefore, Module 2 is considered successfully implemented and verified.

---

## 14. Layman's Terms Description (What Does This Module Do?)

If Module 1 was the static map and railway tracks, **Module 2 is the train depot and fleet registry**.

Imagine you are the chief railway dispatcher. You need to know:
- What trains exist on your network right now? (Train ID 1, Train ID 2, etc.)
- What kind of trains are they?
  - An **Express Train** is sleek, carries passengers at high speed (up to 45 m/s or 162 km/h), and has high-performance brakes.
  - A **Passenger Train** runs standard commuter services at normal speeds (up to 33 m/s or 120 km/h) with comfortable deceleration.
  - A **Freight Train** is a massive, heavy cargo hauler (1,500,000 kg). It moves slower (max 22 m/s or 80 km/h) and takes a very long distance to slow down due to pure momentum.
- What is their current health and status? (Are they Idle in the yard, Running along a route, Braking to avoid a hazard, or Stopped at a red signal?)

**TrainManager** acts as the central registry office. It guarantees that no two trains have the same number, lets any part of the system look up a train instantly, and safely adds or retires trains from the network.