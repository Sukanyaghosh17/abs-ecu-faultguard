#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <string>
#include "Wheel.hpp"
#include "DiagnosticsManager.hpp"

// Standard CAN Arbitration IDs used by the ABS ECU
constexpr uint32_t CAN_ID_WHEEL_SPEEDS = 0x0C0;
constexpr uint32_t CAN_ID_ABS_STATUS   = 0x0C1;
constexpr uint32_t CAN_ID_DIAGNOSTICS  = 0x0C2;

// Fixed 8-byte payload CAN Frame model
struct CANFrame {
    uint32_t               id        = 0;
    std::array<uint8_t, 8> data{};
    uint8_t                dlc       = 8;
    double                 timestamp = 0.0;
};

// Signal packing and unpacking helpers for ECU frames
CANFrame pack_wheel_speeds(double timestamp, const std::vector<double>& speeds);
std::vector<double> unpack_wheel_speeds(const CANFrame& frame);

CANFrame pack_abs_status(double timestamp, const std::vector<BrakeState>& states, bool abs_active);
void unpack_abs_status(const CANFrame& frame, std::vector<BrakeState>& states_out, bool& abs_active_out);

CANFrame pack_diagnostics(double timestamp, const std::vector<DTC>& active_dtcs);
void unpack_diagnostics(const CANFrame& frame, uint8_t& dtc_count_out, uint8_t& dtc_mask_out);

// Decode raw CANFrame payload into a human-readable string summary
std::string decode_frame_summary(const CANFrame& frame);

// In-memory simulation of a CAN bus network
class CANBus {
public:
    CANBus() = default;

    // Transmit a frame on the simulated bus (appends to in-memory history)
    void transmit(const CANFrame& frame);

    // Retrieve full bus traffic history
    const std::vector<CANFrame>& get_history() const;

    // Export bus log to CSV (includes hex payload and human-readable decoded summary).
    // Guarded by file.is_open() so unopened destinations (such as in unit test environments)
    // cleanly no-op.
    void dump_to_csv(const std::string& filepath = "logs/can_bus.csv") const;

    // Clear history
    void clear();

private:
    std::vector<CANFrame> history_;
};
