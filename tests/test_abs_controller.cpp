/**
 * test_abs_controller.cpp
 * =======================
 * Unit tests for the ABS ECU FaultGuard simulation.
 *
 * Coverage:
 *   1. SlipCalculation   – slip ratio arithmetic and edge cases
 *   2. StateTransition   – APPLY / HOLD / RELEASE threshold logic
 *   3. FaultInjector     – all four fault type transforms
 *   4. ReferenceSpeed    – peak-hold / deceleration-limited estimator
 *
 * Testing approach (Option B):
 *   Tests instantiate ABSController directly and observe get_actuators()
 *   after one control_cycle() call.  The constructor attempts to open
 *   "logs/abs_log.csv" relative to ctest's working directory (the build
 *   directory).  That directory doesn't exist during testing, so the file
 *   silently fails to open — the log_file.is_open() guard inside
 *   control_cycle() ensures no writes happen and no crash occurs.
 *   All control logic (slip, estimator, actuator commands) still runs
 *   normally.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

#include "ABSController.hpp"   // also pulls in SLIP_THRESHOLD, LOW_SLIP_THRESHOLD, MAX_REF_DECEL
#include "FaultInjector.hpp"
#include "DiagnosticsManager.hpp"
#include "CANBus.hpp"
#include "Sensor.hpp"
#include "Wheel.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>

// ─────────────────────────────────────────────────────────────────────────────
// Helper: build a Vehicle + ABSController pair initialised at `init_speed`,
// then force all four wheel sensors to `wheel_speed` so the next
// control_cycle() sees a known, deterministic slip ratio.
//
// Sensor::read() adds Gaussian noise, which would make slip-ratio tests
// non-deterministic.  We work around this by overriding the true_speed used
// internally via Sensor::update() (which sets true_speed directly) and noting
// that the noise term is proportional to true_speed — at exactly 0.0 m/s
// the noise contribution is also 0.  So wherever we need a perfectly
// predictable reading we set the sensor to 0.0.
//
// For non-zero wheel speeds the noise is small (~0.01 * speed) and all
// assertions use EXPECT_NEAR with a tolerance that comfortably covers it.
// ─────────────────────────────────────────────────────────────────────────────
struct TestRig {
    Vehicle       vehicle;
    ABSController ecu;

    explicit TestRig(double init_speed = 30.0)
        : vehicle(init_speed), ecu(vehicle, init_speed) {}

    // Set all four wheel sensors to the same speed.
    void set_all_wheel_speeds(double speed) {
        for (int i = 0; i < 4; i++)
            ecu.get_sensors()[i].update(speed);
    }

    // Run one control cycle with a 20 ms timestep (matches the real sim).
    void step() { ecu.control_cycle(0.0, 0.020); }
};


// =============================================================================
// Suite 1: Slip Calculation
// =============================================================================

// When every wheel matches the reference speed exactly the ECU should see
// zero slip and put all actuators into APPLY (grip is plentiful).
// Using 60 m/s ensures Gaussian sensor noise (stddev 0.3) causes <0.02 slip,
// well below LOW_SLIP_THRESHOLD (0.05).
TEST(SlipCalculation, ZeroSlipWhenWheelMatchesReference) {
    TestRig rig(60.0);
    rig.set_all_wheel_speeds(60.0);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators()) {
        EXPECT_EQ(act.get_state(), BrakeState::APPLY)
            << "Expect APPLY (zero slip) when wheel speed equals reference";
    }
}

// A wheel reading of 0 m/s against a 30 m/s reference is full lockup
// (slip = 1.0) — the ECU must command RELEASE.
TEST(SlipCalculation, FullLockupSlipTriggerRelease) {
    // Set the wheel we care about to exactly 0 so noise is also 0.
    TestRig rig(30.0);
    // Put all wheels at 0 — slip = (ref - 0) / ref = 1.0 for every wheel.
    rig.set_all_wheel_speeds(0.0);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators()) {
        EXPECT_EQ(act.get_state(), BrakeState::RELEASE)
            << "Expect RELEASE (full lockup slip = 1.0)";
    }
}

// Partial slip: ref=30, wheel=20 => slip = (29.82 - 20) / 29.82 ≈ 0.329 > SLIP_THRESHOLD (0.20).
// Margin is over 12 standard deviations from SLIP_THRESHOLD.
TEST(SlipCalculation, PartialSlipAboveThresholdTriggerRelease) {
    TestRig rig(30.0);
    rig.set_all_wheel_speeds(20.0);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators()) {
        EXPECT_EQ(act.get_state(), BrakeState::RELEASE)
            << "Expect RELEASE when partial slip > SLIP_THRESHOLD";
    }
}

// When ref_speed_ drops below 1.0 m/s the ECU's early-return guard fires
// and skips actuator commands.  Verify no crash / undefined state occurs.
// (The guard exists to prevent divide-by-zero in slip = (ref - wheel) / ref.)
TEST(SlipCalculation, NoCrashWhenReferenceSpeedBelowGuard) {
    // Initialise very slow — ref starts at 0.5 m/s which is already < 1.0.
    TestRig rig(0.5);
    rig.set_all_wheel_speeds(0.0);

    // Should not throw, assert, or crash.  Actuators stay at their default
    // (constructed) state since control_cycle() returned early.
    EXPECT_NO_THROW(rig.step());
}

// All 4 wheels simultaneously locked (original bug scenario):
// When all wheels drop to 0, peak-hold reference must hold above wheel speeds
// (~29.82 m/s), resulting in slip ≈ 1.0 > SLIP_THRESHOLD across all 4 wheels.
// All four actuators must command RELEASE.
TEST(SlipCalculation, FourWheelSimultaneousLockupCommandsRelease) {
    TestRig rig(30.0);
    rig.set_all_wheel_speeds(0.0);
    rig.step();

    EXPECT_GE(rig.ecu.estimate_vehicle_speed(), 29.0)
        << "Peak-hold estimator must maintain reference near 29.82 m/s during full lockup";

    for (const auto& act : rig.ecu.get_actuators()) {
        EXPECT_EQ(act.get_state(), BrakeState::RELEASE)
            << "All 4 actuators must command RELEASE when all 4 wheels lock simultaneously";
    }
}


// =============================================================================
// Suite 2: State Transition
// Tests use the named constants (SLIP_THRESHOLD, LOW_SLIP_THRESHOLD) from the
// header so they remain valid if the thresholds are ever changed.
// =============================================================================

// Slip well below LOW_SLIP_THRESHOLD → plenty of grip → APPLY.
// ref ≈ 60, wheel = 60 → slip ≈ 0 < LOW_SLIP_THRESHOLD.
TEST(StateTransition, ApplyWhenSlipBelowLowThreshold) {
    TestRig rig(60.0);
    rig.set_all_wheel_speeds(60.0);   // slip ≈ 0.0
    rig.step();

    for (const auto& act : rig.ecu.get_actuators())
        EXPECT_EQ(act.get_state(), BrakeState::APPLY);
}

// Slip in the HOLD band: LOW_SLIP_THRESHOLD < slip < SLIP_THRESHOLD (0.05 to 0.20).
// Center target slip at midpoint 0.125:
//   ref after step = max(26.1, 30.0 - 9.0*0.02) = max(26.1, 29.82) = 29.82
//   slip = (29.82 - 26.1) / 29.82 ≈ 0.1247   ← dead center of (0.05, 0.20)
//   Distance to both boundaries is > 2.2 m/s (> 7 standard deviations of noise).
TEST(StateTransition, HoldWhenSlipInOptimalWindow) {
    TestRig rig(30.0);
    rig.set_all_wheel_speeds(26.1);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators())
        EXPECT_EQ(act.get_state(), BrakeState::HOLD)
            << "Slip ≈ 0.125 should produce HOLD";
}

// Slip above SLIP_THRESHOLD → wheel approaching lockup → RELEASE.
// ref=30, wheel=20 → slip ≈ 0.329 > SLIP_THRESHOLD (0.20).
TEST(StateTransition, ReleaseWhenSlipAboveHighThreshold) {
    TestRig rig(30.0);
    rig.set_all_wheel_speeds(20.0);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators())
        EXPECT_EQ(act.get_state(), BrakeState::RELEASE);
}

// Boundary at SLIP_THRESHOLD: slip clearly above → RELEASE; clearly below → HOLD.
// Computed backwards from ref after one step (29.82 with wheels below 29.82):
//   slip = (ref - wheel) / ref = SLIP_THRESHOLD
//   => wheel = ref * (1 - SLIP_THRESHOLD)
//   => wheel = 29.82 * 0.80 = 23.856
//
// At 23.856, slip = exactly 0.20. Using a 1.5 m/s margin (5 standard deviations
// of sensor noise) guarantees deterministic results without test flakiness.
TEST(StateTransition, ThresholdBoundary_JustAbove_IsRelease) {
    const double ref_after = 30.0 - MAX_REF_DECEL * 0.020;   // 29.82
    const double w_at_boundary = ref_after * (1.0 - SLIP_THRESHOLD);
    const double w_just_above_threshold = w_at_boundary - 1.5; // higher slip (slip ≈ 0.25)

    TestRig rig(30.0);
    rig.set_all_wheel_speeds(w_just_above_threshold);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators())
        EXPECT_EQ(act.get_state(), BrakeState::RELEASE)
            << "Slip above SLIP_THRESHOLD should be RELEASE";
}

TEST(StateTransition, ThresholdBoundary_JustBelow_IsHold) {
    const double ref_after = 30.0 - MAX_REF_DECEL * 0.020;
    const double w_at_boundary = ref_after * (1.0 - SLIP_THRESHOLD);
    const double w_just_below_threshold = w_at_boundary + 1.5; // lower slip (slip ≈ 0.15)

    TestRig rig(30.0);
    rig.set_all_wheel_speeds(w_just_below_threshold);
    rig.step();

    for (const auto& act : rig.ecu.get_actuators())
        EXPECT_EQ(act.get_state(), BrakeState::HOLD)
            << "Slip below SLIP_THRESHOLD should be HOLD";
}


// =============================================================================
// Suite 3: FaultInjector
// FaultInjector is a pure transform — no Vehicle or ABSController needed.
// =============================================================================

TEST(FaultInjector, NonePassesThroughUnchanged) {
    FaultInjector fi;
    fi.set_fault(0, FaultType::NONE);
    EXPECT_DOUBLE_EQ(fi.apply(0, 20.0), 20.0);
}

TEST(FaultInjector, BiasAddsFixedOffset) {
    FaultInjector fi;
    fi.set_fault(0, FaultType::BIAS, 5.0);
    EXPECT_DOUBLE_EQ(fi.apply(0, 20.0), 25.0);
}

TEST(FaultInjector, BiasWorksWithNegativeOffset) {
    FaultInjector fi;
    fi.set_fault(1, FaultType::BIAS, -3.5);
    EXPECT_DOUBLE_EQ(fi.apply(1, 20.0), 16.5);
}

TEST(FaultInjector, LockupReturnsForcedNearZeroValue) {
    FaultInjector fi;
    fi.set_fault(2, FaultType::LOCKUP);
    // The implementation returns 0.1 (not exactly 0) to avoid ECU divide-by-zero.
    EXPECT_DOUBLE_EQ(fi.apply(2, 20.0), 0.1);
}

TEST(FaultInjector, DisconnectedReturnsExactlyZero) {
    FaultInjector fi;
    fi.set_fault(3, FaultType::DISCONNECTED);
    EXPECT_DOUBLE_EQ(fi.apply(3, 20.0), 0.0);
}

TEST(FaultInjector, OutOfRangeWheelIndexThrows) {
    FaultInjector fi;
    EXPECT_THROW(fi.set_fault(-1, FaultType::BIAS, 1.0), std::out_of_range);
    EXPECT_THROW(fi.set_fault(4,  FaultType::BIAS, 1.0), std::out_of_range);
}

TEST(FaultInjector, FaultsArePerWheelIndependent) {
    FaultInjector fi;
    fi.set_fault(0, FaultType::LOCKUP);
    fi.set_fault(1, FaultType::NONE);
    // Wheel 0 is faulted; wheel 1 should still pass through.
    EXPECT_DOUBLE_EQ(fi.apply(0, 20.0), 0.1);
    EXPECT_DOUBLE_EQ(fi.apply(1, 20.0), 20.0);
}

// Clearing a fault (setting FaultType::NONE on a wheel that previously had a fault)
// correctly reverts behavior to pass-through.
TEST(FaultInjector, ClearFaultRevertsToPassThrough) {
    FaultInjector fi;
    fi.set_fault(0, FaultType::LOCKUP);
    EXPECT_DOUBLE_EQ(fi.apply(0, 25.0), 0.1);
    EXPECT_EQ(fi.get_fault_type(0), FaultType::LOCKUP);

    // Revert to NONE
    fi.set_fault(0, FaultType::NONE);
    EXPECT_DOUBLE_EQ(fi.apply(0, 25.0), 25.0);
    EXPECT_EQ(fi.get_fault_type(0), FaultType::NONE);
}


// =============================================================================
// Suite 4: Reference Speed Estimator (peak-hold / decel-limited)
// =============================================================================

// After one cycle with wheels slower than init_speed, the reference should
// still be ABOVE all wheel readings (decel-limited carry-over dominates).
//
// init=30, wheels=25, dt=0.02:
//   max_wheel = 25.0
//   decel_limited = 30.0 - 9.0*0.02 = 29.82
//   ref = max(25.0, 29.82) = 29.82  — which is > 25.0  ✓
//
// Note: ref > 1.0, so the early-return guard does NOT fire.
TEST(ReferenceSpeed, RefHoldsAboveWheelReadings) {
    TestRig rig(30.0);
    rig.set_all_wheel_speeds(25.0);
    rig.step();

    const double ref = rig.ecu.estimate_vehicle_speed();
    EXPECT_GE(ref, 25.0)
        << "Reference speed must be >= max wheel reading after one step";
}

// The reference should drop by AT MOST MAX_REF_DECEL * dt per cycle, even
// when all wheels read 0 (worst case: all wheels fully locked).
//
// init=30, wheels=0, dt=0.02:
//   decel_limited = 30.0 - 0.18 = 29.82
//   max_wheel = 0
//   ref = max(0, 29.82) = 29.82
//   drop = 30.0 - 29.82 = 0.18 = MAX_REF_DECEL * 0.02  ✓
//
// Note: ref=29.82 >> 1.0, early-return does NOT fire.
TEST(ReferenceSpeed, RefDecaysAtMostMaxRefDecelPerStep) {
    const double init_speed = 30.0;
    const double dt = 0.020;

    TestRig rig(init_speed);
    rig.set_all_wheel_speeds(0.0);   // worst case: all wheels locked
    rig.step();

    const double ref_after = rig.ecu.estimate_vehicle_speed();
    const double max_allowed_drop = MAX_REF_DECEL * dt;

    EXPECT_GE(ref_after, init_speed - max_allowed_drop - 1e-9)
        << "Reference must not drop faster than MAX_REF_DECEL per step";
}

// With wheels at 0 and a small initial speed, ref decays over many cycles and
// will eventually fall below 1.0 (triggering the early-return guard inside
// control_cycle).  The floor clamp in update_reference_speed must keep ref
// non-negative throughout.
//
// init=2.0, wheels=0, dt=0.02:
//   Each cycle: ref -= 0.18.  ref goes: 2.0 → 1.82 → 1.64 → 1.46 → 1.28
//               → 1.10 → 0.92 (< 1.0: early-return now fires in step 3 below)
//               → clamp to max(0, 0.74) → ... → clamp to 0.
//
// This specifically tests that update_reference_speed's "if (ref_speed_ < 0)
// ref_speed_ = 0" guard is exercised, not the actuator-command path.
TEST(ReferenceSpeed, RefDoesNotGoNegative) {
    TestRig rig(2.0);
    rig.set_all_wheel_speeds(0.0);

    // Run enough cycles to drive ref below zero without the clamp.
    // 2.0 / 0.18 ≈ 12 cycles.  Run 20 to be safe.
    for (int i = 0; i < 20; i++) {
        // Re-set sensors each cycle because the physics plant (not under test
        // here) is not being driven — sensors retain their last update().
        rig.set_all_wheel_speeds(0.0);
        rig.ecu.control_cycle(static_cast<double>(i) * 0.020, 0.020);
    }

    EXPECT_GE(rig.ecu.estimate_vehicle_speed(), 0.0)
        << "Reference speed must never go negative";
}

// Single wheel positive bias pulls up estimator:
// When one wheel reads far above true speed, the peak-hold estimator anchors
// to that maximum reading, pulling the global reference up with it.
TEST(ReferenceSpeed, SingleWheelPositiveBiasPullsUpEstimator) {
    Vehicle vehicle(30.0);
    ABSController ecu(vehicle, 30.0);
    FaultInjector fi;
    fi.set_fault(0, FaultType::BIAS, 20.0); // +20 m/s bias on wheel 0
    ecu.set_fault_injector(&fi);

    for (int i = 0; i < 4; i++) {
        ecu.get_sensors()[i].update(30.0);
    }
    ecu.control_cycle(0.0, 0.020);

    // Wheel 0 reads ~30 + 20 = 50 m/s. Estimator should be pulled up to ~50 m/s.
    EXPECT_GE(ecu.estimate_vehicle_speed(), 45.0)
        << "Single wheel with high positive bias should pull up peak-hold reference speed";
}


// =============================================================================
// Suite 5: Regression Tests
// =============================================================================

// Regression test for CSV logging bug:
// Asserts that when a fault is active, the value written to the CSV for that
// wheel matches the post-fault readings[i] value (from FaultInjector::apply),
// NOT the sensor's raw/true physical speed.
TEST(Regression, CSVLoggingLogsPostFaultValueNotRawSpeed) {
    const std::string test_csv = "test_abs_log_regression.csv";
    std::remove(test_csv.c_str());

    Vehicle vehicle(30.0);
    {
        ABSController ecu(vehicle, 30.0, test_csv);
        FaultInjector fi;
        fi.set_fault(0, FaultType::LOCKUP); // Forces 0.1 m/s
        ecu.set_fault_injector(&fi);

        // Physical wheels are spinning at 30.0 m/s
        for (int i = 0; i < 4; i++) {
            ecu.get_sensors()[i].update(30.0);
        }

        ecu.control_cycle(0.020, 0.020);
    } // destructor flushes and closes log_file

    std::ifstream file(test_csv);
    ASSERT_TRUE(file.is_open()) << "Failed to open test CSV log";

    std::string header_line, data_line;
    std::getline(file, header_line);
    ASSERT_TRUE(std::getline(file, data_line));
    file.close();
    std::remove(test_csv.c_str());

    // Header: Time(s),Veh_Speed,Est_Veh_Speed,W0_Speed,W1_Speed,...
    std::stringstream ss(data_line);
    std::string token;
    std::vector<std::string> tokens;
    while (std::getline(ss, token, ',')) {
        tokens.push_back(token);
    }

    ASSERT_GE(tokens.size(), 4u);
    double w0_logged = std::stod(tokens[3]);

    // The post-fault value for LOCKUP is 0.100 m/s, whereas raw true speed is ~30.0 m/s.
    EXPECT_NEAR(w0_logged, 0.100, 0.05)
        << "W0_Speed in CSV must be the post-fault reading (~0.1 m/s), not true speed (~30 m/s)";
    EXPECT_LT(w0_logged, 5.0);
}


// =============================================================================
// Suite 6: DiagnosticsManager
// =============================================================================

// Circuit fault DTC (C0031-C0034) is raised after CIRCUIT_FAULT_THRESHOLD_CYCLES
// consecutive cycles of near-zero reading while vehicle is in motion (ref > 5.0).
TEST(DiagnosticsManager, CircuitFaultDTCRaisedAfterConsecutiveZeroReadings) {
    DiagnosticsManager diag;
    std::vector<double> readings = {0.0, 30.0, 30.0, 30.0};
    std::vector<BrakeState> states(4, BrakeState::APPLY);
    double ref = 30.0;

    // Run 9 cycles: below threshold (10), DTC should not be active yet
    for (int i = 0; i < 9; i++) {
        diag.evaluate_cycle(i * 0.020, readings, ref, states);
        EXPECT_FALSE(diag.has_dtc(DTC::C0031_W0_SENSOR_CIRCUIT));
    }

    // 10th cycle: threshold reached, C0031 must be raised
    diag.evaluate_cycle(0.180, readings, ref, states);
    EXPECT_TRUE(diag.has_dtc(DTC::C0031_W0_SENSOR_CIRCUIT));
    EXPECT_EQ(diag.get_active_dtcs_string(), "C0031");
}

// Circuit fault DTC clears when wheel sensor reading recovers above 1.0 m/s.
TEST(DiagnosticsManager, CircuitFaultDTCClearsWhenSensorRecovers) {
    DiagnosticsManager diag;
    std::vector<double> faulted_readings = {0.0, 30.0, 30.0, 30.0};
    std::vector<BrakeState> states(4, BrakeState::APPLY);
    double ref = 30.0;

    for (int i = 0; i < 10; i++) {
        diag.evaluate_cycle(i * 0.020, faulted_readings, ref, states);
    }
    ASSERT_TRUE(diag.has_dtc(DTC::C0031_W0_SENSOR_CIRCUIT));

    // Sensor recovers to normal speed
    std::vector<double> recovered_readings = {30.0, 30.0, 30.0, 30.0};
    diag.evaluate_cycle(0.200, recovered_readings, ref, states);

    EXPECT_FALSE(diag.has_dtc(DTC::C0031_W0_SENSOR_CIRCUIT));
    EXPECT_EQ(diag.get_active_dtcs_string(), "NONE");
}

// Implausible reference DTC (C0040) is raised when a wheel reading is far above ref.
TEST(DiagnosticsManager, ImplausibleReferenceDTCRaisedWhenSensorFarAboveRef) {
    DiagnosticsManager diag;
    // Wheel 1 reads 45 m/s when ref is 30 m/s (delta = 15 m/s > PLAUSIBLE_SPEED_DELTA = 10.0)
    std::vector<double> readings = {30.0, 45.0, 30.0, 30.0};
    std::vector<BrakeState> states(4, BrakeState::APPLY);
    double ref = 30.0;

    for (int i = 0; i < 5; i++) {
        diag.evaluate_cycle(i * 0.020, readings, ref, states);
    }
    EXPECT_TRUE(diag.has_dtc(DTC::C0040_REF_SPEED_IMPLAUSIBLE));
}

// Degraded control loop DTC (C0050) is raised when a wheel is stuck in RELEASE for 25 cycles.
TEST(DiagnosticsManager, DegradedControlLoopDTCRaisedWhenWheelStuckInRelease) {
    DiagnosticsManager diag;
    std::vector<double> readings(4, 20.0);
    std::vector<BrakeState> states = {BrakeState::RELEASE, BrakeState::APPLY, BrakeState::APPLY, BrakeState::APPLY};
    double ref = 30.0;

    for (int i = 0; i < 24; i++) {
        diag.evaluate_cycle(i * 0.020, readings, ref, states);
        EXPECT_FALSE(diag.has_dtc(DTC::C0050_CONTROL_LOOP_DEGRADED));
    }

    diag.evaluate_cycle(0.480, readings, ref, states);
    EXPECT_TRUE(diag.has_dtc(DTC::C0050_CONTROL_LOOP_DEGRADED));
}


// =============================================================================
// Suite 7: CANBus
// =============================================================================

TEST(CANBus, WheelSpeedFramePackUnpackRoundtrip) {
    std::vector<double> original = {29.85, 29.83, 29.86, 29.90};
    CANFrame frame = pack_wheel_speeds(0.040, original);

    EXPECT_EQ(frame.id, CAN_ID_WHEEL_SPEEDS);
    EXPECT_EQ(frame.dlc, 8);
    EXPECT_DOUBLE_EQ(frame.timestamp, 0.040);

    auto unpacked = unpack_wheel_speeds(frame);
    ASSERT_EQ(unpacked.size(), 4u);
    for (int i = 0; i < 4; i++) {
        EXPECT_NEAR(unpacked[i], original[i], 0.015);
    }
}

TEST(CANBus, ABSStatusFramePackUnpackRoundtrip) {
    std::vector<BrakeState> original_states = {
        BrakeState::RELEASE, BrakeState::HOLD, BrakeState::APPLY, BrakeState::RELEASE
    };
    CANFrame frame = pack_abs_status(0.100, original_states, true);

    EXPECT_EQ(frame.id, CAN_ID_ABS_STATUS);
    std::vector<BrakeState> unpacked_states;
    bool unpacked_active = false;
    unpack_abs_status(frame, unpacked_states, unpacked_active);

    EXPECT_TRUE(unpacked_active);
    ASSERT_EQ(unpacked_states.size(), 4u);
    for (int i = 0; i < 4; i++) {
        EXPECT_EQ(unpacked_states[i], original_states[i]);
    }
}

TEST(CANBus, DiagnosticsFramePackUnpackRoundtrip) {
    std::vector<DTC> dtcs = {DTC::C0031_W0_SENSOR_CIRCUIT, DTC::C0050_CONTROL_LOOP_DEGRADED};
    CANFrame frame = pack_diagnostics(0.200, dtcs);

    EXPECT_EQ(frame.id, CAN_ID_DIAGNOSTICS);
    uint8_t count = 0, mask = 0;
    unpack_diagnostics(frame, count, mask);

    EXPECT_EQ(count, 2);
    EXPECT_TRUE(mask & (1 << 0)); // C0031
    EXPECT_TRUE(mask & (1 << 5)); // C0050
    EXPECT_FALSE(mask & (1 << 4)); // C0040 not set
}

TEST(CANBus, TransmitAppendsToHistoryAndDumpsCSV) {
    const std::string test_csv = "test_can_bus_export.csv";
    std::remove(test_csv.c_str());

    CANBus bus;
    CANFrame f1 = pack_wheel_speeds(0.020, {30.0, 30.0, 30.0, 30.0});
    CANFrame f2 = pack_abs_status(0.020, {BrakeState::APPLY, BrakeState::APPLY, BrakeState::APPLY, BrakeState::APPLY}, false);
    bus.transmit(f1);
    bus.transmit(f2);

    EXPECT_EQ(bus.get_history().size(), 2u);

    bus.dump_to_csv(test_csv);

    std::ifstream file(test_csv);
    ASSERT_TRUE(file.is_open());
    std::string header;
    std::getline(file, header);
    EXPECT_NE(header.find("CAN_ID"), std::string::npos);

    std::string line1, line2;
    EXPECT_TRUE(std::getline(file, line1));
    EXPECT_TRUE(std::getline(file, line2));
    file.close();
    std::remove(test_csv.c_str());
}
