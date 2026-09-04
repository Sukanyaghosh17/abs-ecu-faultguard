#include "CANBus.hpp"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

static const char* state_to_string(BrakeState s) {
    switch (s) {
        case BrakeState::APPLY:   return "APPLY";
        case BrakeState::HOLD:    return "HOLD";
        case BrakeState::RELEASE: return "RELEASE";
    }
    return "UNKNOWN";
}

CANFrame pack_wheel_speeds(double timestamp, const std::vector<double>& speeds) {
    CANFrame frame;
    frame.id        = CAN_ID_WHEEL_SPEEDS;
    frame.dlc       = 8;
    frame.timestamp = timestamp;

    for (int i = 0; i < 4; i++) {
        double sp = (i < static_cast<int>(speeds.size())) ? speeds[i] : 0.0;
        uint16_t raw = static_cast<uint16_t>(std::clamp(sp * 100.0, 0.0, 65535.0));
        frame.data[i * 2]     = static_cast<uint8_t>((raw >> 8) & 0xFF);
        frame.data[i * 2 + 1] = static_cast<uint8_t>(raw & 0xFF);
    }
    return frame;
}

std::vector<double> unpack_wheel_speeds(const CANFrame& frame) {
    std::vector<double> speeds(4, 0.0);
    for (int i = 0; i < 4; i++) {
        uint16_t raw = (static_cast<uint16_t>(frame.data[i * 2]) << 8) |
                        static_cast<uint16_t>(frame.data[i * 2 + 1]);
        speeds[i] = static_cast<double>(raw) / 100.0;
    }
    return speeds;
}

CANFrame pack_abs_status(double timestamp, const std::vector<BrakeState>& states, bool abs_active) {
    CANFrame frame;
    frame.id        = CAN_ID_ABS_STATUS;
    frame.dlc       = 8;
    frame.timestamp = timestamp;

    uint8_t s0 = (states.size() > 0) ? static_cast<uint8_t>(states[0]) : 0;
    uint8_t s1 = (states.size() > 1) ? static_cast<uint8_t>(states[1]) : 0;
    uint8_t s2 = (states.size() > 2) ? static_cast<uint8_t>(states[2]) : 0;
    uint8_t s3 = (states.size() > 3) ? static_cast<uint8_t>(states[3]) : 0;

    frame.data[0] = static_cast<uint8_t>((s0 & 0x03) | ((s1 & 0x03) << 2));
    frame.data[1] = static_cast<uint8_t>((s2 & 0x03) | ((s3 & 0x03) << 2));
    frame.data[2] = abs_active ? 1 : 0;
    return frame;
}

void unpack_abs_status(const CANFrame& frame, std::vector<BrakeState>& states_out, bool& abs_active_out) {
    states_out.resize(4);

    // Defensive check: 2-bit extraction (data & 0x03) can produce value 3, which has no
    // corresponding BrakeState (only 0=APPLY, 1=HOLD, 2=RELEASE are valid).
    // Clamp to BrakeState::HOLD as a safe default against corrupted/malformed CAN payloads
    // rather than casting an invalid value into the enum (not something that occurs in normal operation).
    auto safe_brake_state = [](uint8_t raw) -> BrakeState {
        if (raw > 2) {
            return BrakeState::HOLD;
        }
        return static_cast<BrakeState>(raw);
    };

    states_out[0] = safe_brake_state(frame.data[0] & 0x03);
    states_out[1] = safe_brake_state((frame.data[0] >> 2) & 0x03);
    states_out[2] = safe_brake_state(frame.data[1] & 0x03);
    states_out[3] = safe_brake_state((frame.data[1] >> 2) & 0x03);
    abs_active_out = (frame.data[2] != 0);
}

CANFrame pack_diagnostics(double timestamp, const std::vector<DTC>& active_dtcs) {
    CANFrame frame;
    frame.id        = CAN_ID_DIAGNOSTICS;
    frame.dlc       = 8;
    frame.timestamp = timestamp;

    frame.data[0] = static_cast<uint8_t>(std::min<size_t>(active_dtcs.size(), 255));
    uint8_t mask = 0;
    for (DTC code : active_dtcs) {
        switch (code) {
            case DTC::C0031_W0_SENSOR_CIRCUIT:     mask |= (1 << 0); break;
            case DTC::C0032_W1_SENSOR_CIRCUIT:     mask |= (1 << 1); break;
            case DTC::C0033_W2_SENSOR_CIRCUIT:     mask |= (1 << 2); break;
            case DTC::C0034_W3_SENSOR_CIRCUIT:     mask |= (1 << 3); break;
            case DTC::C0040_REF_SPEED_IMPLAUSIBLE: mask |= (1 << 4); break;
            case DTC::C0050_CONTROL_LOOP_DEGRADED:  mask |= (1 << 5); break;
        }
    }
    frame.data[1] = mask;
    return frame;
}

void unpack_diagnostics(const CANFrame& frame, uint8_t& dtc_count_out, uint8_t& dtc_mask_out) {
    dtc_count_out = frame.data[0];
    dtc_mask_out  = frame.data[1];
}

std::string decode_frame_summary(const CANFrame& frame) {
    std::ostringstream oss;
    switch (frame.id) {
        case CAN_ID_WHEEL_SPEEDS: {
            auto sp = unpack_wheel_speeds(frame);
            oss << std::fixed << std::setprecision(2)
                << "W0=" << sp[0] << " W1=" << sp[1]
                << " W2=" << sp[2] << " W3=" << sp[3] << " m/s";
            break;
        }
        case CAN_ID_ABS_STATUS: {
            std::vector<BrakeState> states;
            bool abs_active = false;
            unpack_abs_status(frame, states, abs_active);
            oss << "W0=" << state_to_string(states[0])
                << " W1=" << state_to_string(states[1])
                << " W2=" << state_to_string(states[2])
                << " W3=" << state_to_string(states[3])
                << " | ABS=" << (abs_active ? "ACTIVE" : "INACTIVE");
            break;
        }
        case CAN_ID_DIAGNOSTICS: {
            uint8_t count = 0, mask = 0;
            unpack_diagnostics(frame, count, mask);
            oss << "Active_DTC_Count=" << static_cast<int>(count);
            if (mask == 0) {
                oss << " (NONE)";
            } else {
                oss << " (";
                bool first = true;
                if (mask & (1 << 0)) { oss << (first ? "" : ";") << "C0031"; first = false; }
                if (mask & (1 << 1)) { oss << (first ? "" : ";") << "C0032"; first = false; }
                if (mask & (1 << 2)) { oss << (first ? "" : ";") << "C0033"; first = false; }
                if (mask & (1 << 3)) { oss << (first ? "" : ";") << "C0034"; first = false; }
                if (mask & (1 << 4)) { oss << (first ? "" : ";") << "C0040"; first = false; }
                if (mask & (1 << 5)) { oss << (first ? "" : ";") << "C0050"; first = false; }
                oss << ")";
            }
            break;
        }
        default:
            oss << "Unknown arbitration ID";
            break;
    }
    return oss.str();
}

void CANBus::transmit(const CANFrame& frame) {
    history_.push_back(frame);
}

const std::vector<CANFrame>& CANBus::get_history() const {
    return history_;
}

void CANBus::clear() {
    history_.clear();
}

void CANBus::dump_to_csv(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        // Explicit guard: if directory doesn't exist or file fails to open,
        // silently return (consistent with ABSController's CSV logging behavior).
        return;
    }

    file << "Time(s),CAN_ID,DLC,Data_Hex,Decoded_Summary\n";

    for (const auto& frame : history_) {
        std::ostringstream hex_stream;
        hex_stream << std::hex << std::uppercase << std::setfill('0');
        for (int i = 0; i < frame.dlc; i++) {
            hex_stream << std::setw(2) << static_cast<int>(frame.data[i]);
            if (i + 1 < frame.dlc) hex_stream << " ";
        }

        std::ostringstream id_stream;
        id_stream << "0x" << std::hex << std::uppercase << std::setfill('0')
                  << std::setw(3) << frame.id;

        file << std::fixed << std::setprecision(3)
             << frame.timestamp << ","
             << id_stream.str() << ","
             << static_cast<int>(frame.dlc) << ","
             << "\"" << hex_stream.str() << "\","
             << "\"" << decode_frame_summary(frame) << "\"\n";
    }
}
