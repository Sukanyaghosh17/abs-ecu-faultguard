#pragma once

#include <array>

// Fault types supported per wheel sensor channel.
// NONE        = normal operation, reading passes through unchanged.
// BIAS        = a fixed offset is added to every reading (stuck-high / electromagnetic interference).
// LOCKUP      = reading is forced near zero, simulating a wheel that has locked up
//               independently of brake pressure (e.g., seized caliper).
// DISCONNECTED= reading is always 0.0, simulating an open-circuit sensor wire.
enum class FaultType {
    NONE,
    BIAS,
    LOCKUP,
    DISCONNECTED
};

// Per-wheel fault configuration.
struct WheelFault {
    FaultType type      = FaultType::NONE;
    double    bias_value = 0.0;   // Only used when type == BIAS
};

// FaultInjector sits between the raw sensor reading and the ECU control logic.
// Faults are configured once at startup (via CLI) and then queried every cycle.
// The ABSController calls apply() after each sensor.read() to obtain the
// (potentially corrupted) speed value the ECU will actually use.
class FaultInjector {
public:
    static constexpr int NUM_WHEELS = 4;

    // Configure a fault for a specific wheel.
    // wheel  : 0-3
    // type   : fault type (see FaultType enum)
    // bias   : fixed offset applied when type == BIAS (m/s)
    void set_fault(int wheel, FaultType type, double bias = 0.0);

    // Apply the configured fault to a raw sensor reading.
    // Returns the (possibly corrupted) speed value the ECU will see.
    double apply(int wheel, double raw_reading) const;

    // Query what fault type is active for a given wheel (used for CSV logging).
    FaultType get_fault_type(int wheel) const;

private:
    std::array<WheelFault, NUM_WHEELS> faults_{};
};
