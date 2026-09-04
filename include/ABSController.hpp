#pragma once

#include <vector>
#include <string>
#include <fstream>
#include "Sensor.hpp"
#include "Wheel.hpp"
#include "FaultInjector.hpp"
#include "DiagnosticsManager.hpp"
#include "CANBus.hpp"

// ── ECU slip-control thresholds ────────────────────────────────────────────
// Declared in the header (not ABSController.cpp) so that unit tests can
// reference these constants by name rather than duplicating magic numbers.

// Slip above this triggers RELEASE (wheel is on the edge of lockup).
inline constexpr double SLIP_THRESHOLD     = 0.20;
// Slip below this triggers APPLY (plenty of grip left, safe to rebuild pressure).
inline constexpr double LOW_SLIP_THRESHOLD = 0.05;
// Maximum rate (m/s²) at which the ECU's reference speed is allowed to
// decrease per cycle.
inline constexpr double MAX_REF_DECEL      = 9.0;

// Simple one-dimensional vehicle physics model.
// Speed decreases proportionally to average brake pressure applied.
class Vehicle {

private:
    double speed;

public:
    explicit Vehicle(double initial);

    double get_speed() const;

    // Reduce vehicle speed based on average brake pressure over one timestep.
    // avg_pressure : mean brake pressure across all wheels (0–100 %)
    // dt           : timestep length in seconds
    void update(double avg_pressure, double dt);
};

// ABS Electronic Control Unit.
//
// Each control cycle the ECU:
//   1. Reads all wheel speed sensors (once, reusing the values to avoid
//      inconsistent noise between estimation and slip calculation).
//   2. Updates the reference vehicle speed using a peak-hold /
//      deceleration-limited estimator (so the reference does not collapse
//      when all wheels lock simultaneously).
//   3. Computes per-wheel slip ratios and commands APPLY / HOLD / RELEASE.
//   4. Evaluates diagnostics (DTCs) and broadcasts CAN frames.
//   5. Logs the full state to a CSV file.
class ABSController {

private:
    Vehicle& vehicle;

    // Peak-hold, deceleration-limited reference speed (m/s).
    // This is the ECU's best estimate of true vehicle speed.
    double ref_speed_;

    // Optional fault injector — nullptr means no faults active.
    FaultInjector* fault_injector_;

    std::vector<WheelSpeedSensor> sensors;
    std::vector<BrakeActuator>   actuators;

    DiagnosticsManager diagnostics_;
    CANBus             can_bus_;

    std::ofstream log_file;

    // Internal helper: update ref_speed_ from latest sensor readings.
    // Uses peak-hold capped by a maximum deceleration rate so the
    // reference cannot drop faster than a physically realistic vehicle.
    void update_reference_speed(const std::vector<double>& readings, double dt);

public:
    ABSController(Vehicle& v, double init_speed, const std::string& log_path = "logs/abs_log.csv");

    ~ABSController();

    // Attach a FaultInjector (call before the simulation loop).
    // Passing nullptr disables fault injection.
    void set_fault_injector(FaultInjector* fi);

    // Return the current reference speed estimate (m/s).
    // Safe to call from main() for console output between cycles.
    double estimate_vehicle_speed() const;

    // Run one ECU control cycle.
    // sim_time : elapsed simulation time in seconds (for CSV logging)
    // dt       : timestep length in seconds (used for decel-limit calculation)
    void control_cycle(double sim_time, double dt);

    const std::vector<BrakeActuator>& get_actuators() const;
    std::vector<WheelSpeedSensor>&    get_sensors();

    DiagnosticsManager&       get_diagnostics();
    const DiagnosticsManager& get_diagnostics() const;

    CANBus&       get_can_bus();
    const CANBus& get_can_bus() const;
};