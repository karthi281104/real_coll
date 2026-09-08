cmake_minimum_required(VERSION 3.20)

project(TCAS
    VERSION 0.1.0
    DESCRIPTION "Real-Time Train Collision Avoidance System with Predictive Conflict Resolution"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(MSVC)
    add_compile_options(/W4 /permissive-)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

add_library(tcas_core
    src/infrastructure/Node.cpp
    src/infrastructure/Track.cpp
    src/infrastructure/Station.cpp
    src/infrastructure/Junction.cpp
    src/infrastructure/Platform.cpp
    src/infrastructure/RailwayNetwork.cpp
    src/train/Train.cpp
    src/train/ExpressTrain.cpp
    src/train/PassengerTrain.cpp
    src/train/FreightTrain.cpp
    src/train/TrainManager.cpp
    src/simulation/SimClock.cpp
    src/simulation/SimulationTimer.cpp
    src/physics/KinematicsEngine.cpp
    src/navigation/RouteNavigator.cpp
    src/sensor/Odometer.cpp
    src/sensor/StateEstimator.cpp
    src/communication/Message.cpp
    src/communication/CommunicationChannel.cpp
    src/prediction/PredictionEngine.cpp
    src/conflict/ConflictDetector.cpp
    src/conflict/ResourceReservationManager.cpp
    src/safety/RiskEngine.cpp
    src/safety/PriorityEngine.cpp
    src/safety/ConflictPriorityQueue.cpp
    src/safety/ResolutionEngine.cpp
    src/orchestrator/SafetyPipeline.cpp
    src/scenario/ScenarioManager.cpp
    src/scenario/RouteCatalog.cpp
    src/orchestrator/ThreadOrchestrator.cpp
    src/hmi/HmiDisplay.cpp
    src/hmi/TelemetryLogger.cpp
    src/hmi/PerformanceMetrics.cpp
    src/application/TcasApplication.cpp
)

find_package(Threads REQUIRED)

target_include_directories(tcas_core PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(tcas_core PUBLIC ${PROJECT_SOURCE_DIR}/include/scenario)
target_link_libraries(tcas_core PUBLIC Threads::Threads)

add_executable(tcas
    src/main.cpp
    src/demo/FoundationDemo.cpp
    src/demo/SimulationDemo.cpp
    src/demo/IntegratedDemo.cpp
    src/demo/Module9Demo.cpp
    src/demo/Module10Demo.cpp
)

target_link_libraries(tcas PRIVATE tcas_core Threads::Threads)
target_include_directories(tcas PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_include_directories(tcas PRIVATE ${PROJECT_SOURCE_DIR}/include/scenario)

include(FetchContent)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(tcas_tests
    tests/infrastructure/NodeTest.cpp
    tests/infrastructure/TrackTest.cpp
    tests/infrastructure/RailwayNetworkTest.cpp
    tests/train/TrainTest.cpp
    tests/train/ExpressTrainTest.cpp
    tests/train/PassengerTrainTest.cpp
    tests/train/FreightTrainTest.cpp
    tests/train/TrainManagerTest.cpp
    tests/simulation/SimClockTest.cpp
    tests/simulation/SimulationConfigTest.cpp
    tests/simulation/SimulationTimerTest.cpp
    tests/physics/KinematicsEngineTest.cpp
    tests/navigation/RouteNavigatorTest.cpp
    tests/integration/PhysicsNavigationIntegrationTest.cpp
    tests/integration/Module9IntegrationTest.cpp
    tests/sensor/OdometerTest.cpp
    tests/sensor/StateEstimatorTest.cpp
    tests/communication/MessageTest.cpp
    tests/communication/CommunicationChannelTest.cpp
    tests/prediction/PredictionEngineTest.cpp
    tests/conflict/ConflictDetectorTest.cpp
    tests/safety/RiskEngineTest.cpp
    tests/safety/PriorityEngineTest.cpp
    tests/safety/ConflictPriorityQueueTest.cpp
    tests/safety/ResolutionEngineTest.cpp
    tests/safety/SafetyIntegrationTest.cpp
    tests/orchestrator/SafetyPipelineTest.cpp
    tests/scenario/RouteCatalogTest.cpp
    tests/integration/Module8To12IntegrationTest.cpp
    tests/orchestrator/CommandQueueTest.cpp
    tests/orchestrator/ThreadOrchestratorTest.cpp
    tests/hmi/HmiDisplayTest.cpp
    tests/hmi/TelemetryLoggerTest.cpp
    tests/hmi/PerformanceMetricsTest.cpp
)

target_include_directories(tcas_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(tcas_tests PRIVATE tcas_core GTest::gtest GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(tcas_tests)
