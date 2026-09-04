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
#include "Sensor.hpp"
#include "Wheel.hpp"

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
