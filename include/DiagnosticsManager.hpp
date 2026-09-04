#pragma once

#include <vector>
#include <string>
#include "Wheel.hpp"

// Diagnostic Trouble Codes (DTCs) defined for the ABS ECU FaultGuard.
// C0031-C0034 : Wheel speed sensor circuit malfunction (per-wheel)
// C0040       : ABS reference speed implausible (sensor reading far above physical reference)
// C0050       : ABS control loop degraded (wheel actuator stuck in RELEASE)
enum class DTC {
    C0031_W0_SENSOR_CIRCUIT,
    C0032_W1_SENSOR_CIRCUIT,
    C0033_W2_SENSOR_CIRCUIT,
    C0034_W3_SENSOR_CIRCUIT,
    C0040_REF_SPEED_IMPLAUSIBLE,
    C0050_CONTROL_LOOP_DEGRADED
};

// Convert DTC enum to standard alphanumeric trouble code string (e.g., "C0031").
std::string dtc_to_string(DTC code);

// DiagnosticsManager monitors ECU inputs, reference estimates, and actuator commands
// each cycle to identify anomalies and maintain active Diagnostic Trouble Codes (DTCs).
class DiagnosticsManager {
public:
    static constexpr int CIRCUIT_FAULT_THRESHOLD_CYCLES  = 10; // 10 cycles (~200ms)
    static constexpr int RELEASE_STUCK_THRESHOLD_CYCLES  = 25; // 25 cycles (~500ms)
    static constexpr int REF_IMPLAUSIBLE_THRESHOLD_CYCLES= 5;  // 5 cycles (~100ms)
    static constexpr double PLAUSIBLE_SPEED_DELTA        = 10.0; // m/s above reference

    DiagnosticsManager();

    // Evaluate telemetry for current cycle and update DTC active states.
    // sim_time  : elapsed simulation time
    // readings  : post-fault sensor speed readings per wheel (4)
    // ref_speed : ECU peak-hold vehicle reference speed estimate
    // states    : current BrakeState commanded for each wheel (4)
    void evaluate_cycle(double sim_time,
                        const std::vector<double>& readings,
                        double ref_speed,
                        const std::vector<BrakeState>& states);

    // Query active DTCs
    std::vector<DTC> get_active_dtcs() const;
    bool has_dtc(DTC code) const;

    // Changes detected in the most recent evaluate_cycle() call
    const std::vector<DTC>& get_newly_raised() const;
    const std::vector<DTC>& get_newly_cleared() const;

    // Return a semicolon-separated string of active codes, or "NONE" if empty
    std::string get_active_dtcs_string() const;

    // Reset all internal counters and active DTCs
    void reset();

private:
    std::vector<DTC> active_dtcs_;
    std::vector<DTC> newly_raised_;
    std::vector<DTC> newly_cleared_;

    int zero_reading_counter_[4]{};
    int release_counter_[4]{};
    int ref_implausible_counter_{};

    void raise_dtc(DTC code);
    void clear_dtc(DTC code);
};
