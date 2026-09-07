.PHONY: all configure build run test test-physics test-route test-sensor test-comm test-integration test-verbose check clean help

BUILD_DIR = build
CMAKE    = cmake
CTEST    = ctest
CPPCHECK = cppcheck

# ============================================================
# Cross-platform OS detection
# ============================================================
ifeq ($(OS),Windows_NT)
    CMAKE_GENERATOR = -G "MinGW Makefiles"
    RUN_EXE   = .\$(BUILD_DIR)\tcas.exe
    TEST_EXE  = .\$(BUILD_DIR)\tcas_tests.exe
    CLEAN_CMD = powershell -Command "if (Test-Path $(BUILD_DIR)) { Remove-Item -Recurse -Force $(BUILD_DIR) }; Write-Host 'Build directory cleaned.'"
else
    CMAKE_GENERATOR =
    RUN_EXE   = ./$(BUILD_DIR)/tcas
    TEST_EXE  = ./$(BUILD_DIR)/tcas_tests
    CLEAN_CMD = rm -rf $(BUILD_DIR) && echo "Build directory cleaned."
endif

# ============================================================
# Build targets
# ============================================================

all: build

configure:
	@$(CMAKE) -S . -B $(BUILD_DIR) $(CMAKE_GENERATOR) -DCMAKE_BUILD_TYPE=RelWithDebInfo

build: configure
	@$(CMAKE) --build $(BUILD_DIR) --parallel

run: build
	@$(RUN_EXE)

# ============================================================
# Test targets
# ============================================================

test: build
	@$(TEST_EXE)

test-verbose: build
	@$(CTEST) --test-dir $(BUILD_DIR) --output-on-failure -V

test-physics: build
	@$(TEST_EXE) --gtest_filter="KinematicsEngineTest.*"

test-route: build
	@$(TEST_EXE) --gtest_filter="RouteNavigatorTest.*"

test-sensor: build
	@$(TEST_EXE) --gtest_filter="OdometerTest.*:StateEstimatorTest.*"

test-comm: build
	@$(TEST_EXE) --gtest_filter="MessageTest.*:CommunicationChannelTest.*"

test-integration: build
	@$(TEST_EXE) --gtest_filter="PhysicsNavigationIntegrationTest.*"

test-infra: build
	@$(TEST_EXE) --gtest_filter="NodeTest.*:TrackTest.*:RailwayNetworkTest.*"

test-train: build
	@$(TEST_EXE) --gtest_filter="TrainTest.*:ExpressTrainTest.*:PassengerTrainTest.*:FreightTrainTest.*:TrainManagerTest.*"

test-sim: build
	@$(TEST_EXE) --gtest_filter="SimClockTest.*:SimulationTimerTest.*:SimulationConfigTest.*"

# ============================================================
# Static analysis
# ============================================================

check:
	@$(CPPCHECK) --enable=warning,style,performance,portability \
	    --suppress=missingIncludeSystem \
	    --suppress=unusedFunction \
	    --suppress=syntaxError \
	    --std=c++23 \
	    -I include src tests

# ============================================================
# Clean
# ============================================================

clean:
	@$(CLEAN_CMD)

# ============================================================
# Help
# ============================================================

help:
	@echo "============================================================"
	@echo "              TCAS BUILD SHORTCUTS"
	@echo "============================================================"
	@echo "  make build             - Configure and compile the project"
	@echo "  make run               - Build and launch the integrated demo"
	@echo "  make test              - Run ALL unit and integration tests"
	@echo "  make test-infra        - Run Module 1 Infrastructure tests"
	@echo "  make test-train        - Run Module 2 Train Management tests"
	@echo "  make test-sim          - Run Module 3 Simulation tests"
	@echo "  make test-physics      - Run Module 4 Physics tests"
	@echo "  make test-route        - Run Module 5 Navigation tests"
	@echo "  make test-sensor       - Run Module 6 Sensor and Estimation tests"
	@echo "  make test-comm         - Run Module 7 Communication tests"
	@echo "  make test-integration  - Run cross-module integration tests"
	@echo "  make test-verbose      - Run all tests with full CTest verbosity"
	@echo "  make check             - Run Cppcheck static analysis"
	@echo "  make clean             - Remove build directory"
	@echo "============================================================"
