#include "DiagnosticsManager.hpp"
#include <algorithm>

std::string dtc_to_string(DTC code) {
    switch (code) {
        case DTC::C0031_W0_SENSOR_CIRCUIT:     return "C0031";
        case DTC::C0032_W1_SENSOR_CIRCUIT:     return "C0032";
        case DTC::C0033_W2_SENSOR_CIRCUIT:     return "C0033";
        case DTC::C0034_W3_SENSOR_CIRCUIT:     return "C0034";
        case DTC::C0040_REF_SPEED_IMPLAUSIBLE: return "C0040";
        case DTC::C0050_CONTROL_LOOP_DEGRADED:  return "C0050";
    }
    return "UNKNOWN";
}

DiagnosticsManager::DiagnosticsManager() {
    reset();
}

void DiagnosticsManager::reset() {
    active_dtcs_.clear();
    newly_raised_.clear();
    newly_cleared_.clear();
    for (int i = 0; i < 4; i++) {
        zero_reading_counter_[i] = 0;
        release_counter_[i]      = 0;
    }
    ref_implausible_counter_ = 0;
}

void DiagnosticsManager::raise_dtc(DTC code) {
    if (!has_dtc(code)) {
        active_dtcs_.push_back(code);
        newly_raised_.push_back(code);
    }
}

void DiagnosticsManager::clear_dtc(DTC code) {
    auto it = std::find(active_dtcs_.begin(), active_dtcs_.end(), code);
    if (it != active_dtcs_.end()) {
        active_dtcs_.erase(it);
        newly_cleared_.push_back(code);
    }
}

bool DiagnosticsManager::has_dtc(DTC code) const {
    return std::find(active_dtcs_.begin(), active_dtcs_.end(), code) != active_dtcs_.end();
}

std::vector<DTC> DiagnosticsManager::get_active_dtcs() const {
    return active_dtcs_;
}

const std::vector<DTC>& DiagnosticsManager::get_newly_raised() const {
    return newly_raised_;
}

const std::vector<DTC>& DiagnosticsManager::get_newly_cleared() const {
    return newly_cleared_;
}

std::string DiagnosticsManager::get_active_dtcs_string() const {
    if (active_dtcs_.empty()) {
        return "NONE";
    }
    std::string result;
    for (size_t i = 0; i < active_dtcs_.size(); i++) {
        result += dtc_to_string(active_dtcs_[i]);
        if (i + 1 < active_dtcs_.size()) {
            result += ";";
        }
    }
    return result;
}

void DiagnosticsManager::evaluate_cycle(double /*sim_time*/,
                                        const std::vector<double>& readings,
                                        double ref_speed,
                                        const std::vector<BrakeState>& states) {
    newly_raised_.clear();
    newly_cleared_.clear();

    const DTC circuit_dtcs[4] = {
        DTC::C0031_W0_SENSOR_CIRCUIT,
        DTC::C0032_W1_SENSOR_CIRCUIT,
        DTC::C0033_W2_SENSOR_CIRCUIT,
        DTC::C0034_W3_SENSOR_CIRCUIT
    };

    // 1. Wheel speed sensor circuit malfunction (C0031-C0034)
    // Raised when vehicle is in motion (ref_speed > 5.0 m/s) and a wheel reading
    // is stuck near zero (<= 0.1 m/s) for CIRCUIT_FAULT_THRESHOLD_CYCLES consecutive cycles.
    for (int i = 0; i < 4 && i < static_cast<int>(readings.size()); i++) {
        if (ref_speed > 5.0 && readings[i] <= 0.1) {
            zero_reading_counter_[i]++;
            if (zero_reading_counter_[i] >= CIRCUIT_FAULT_THRESHOLD_CYCLES) {
                raise_dtc(circuit_dtcs[i]);
            }
        } else if (readings[i] > 1.0) {
            zero_reading_counter_[i] = 0;
            clear_dtc(circuit_dtcs[i]);
        }
    }

    // 2. ABS reference speed implausible (C0040)
    // Raised when a wheel sensor reading exceeds ref_speed by PLAUSIBLE_SPEED_DELTA
    // for REF_IMPLAUSIBLE_THRESHOLD_CYCLES consecutive cycles.
    bool any_sensor_implausibly_high = false;
    for (int i = 0; i < 4 && i < static_cast<int>(readings.size()); i++) {
        if (readings[i] > ref_speed + PLAUSIBLE_SPEED_DELTA) {
            any_sensor_implausibly_high = true;
            break;
        }
    }
    if (any_sensor_implausibly_high) {
        ref_implausible_counter_++;
        if (ref_implausible_counter_ >= REF_IMPLAUSIBLE_THRESHOLD_CYCLES) {
            raise_dtc(DTC::C0040_REF_SPEED_IMPLAUSIBLE);
        }
    } else {
        ref_implausible_counter_ = 0;
        clear_dtc(DTC::C0040_REF_SPEED_IMPLAUSIBLE);
    }

    // 3. ABS control loop degraded (C0050)
    // Raised when a wheel has been continuously held in RELEASE for
    // RELEASE_STUCK_THRESHOLD_CYCLES consecutive cycles (~500 ms).
    bool any_release_stuck = false;
    for (int i = 0; i < 4 && i < static_cast<int>(states.size()); i++) {
        if (states[i] == BrakeState::RELEASE) {
            release_counter_[i]++;
            if (release_counter_[i] >= RELEASE_STUCK_THRESHOLD_CYCLES) {
                any_release_stuck = true;
            }
        } else {
            release_counter_[i] = 0;
        }
    }
    if (any_release_stuck) {
        raise_dtc(DTC::C0050_CONTROL_LOOP_DEGRADED);
    } else {
        bool any_above_threshold = false;
        for (int i = 0; i < 4; i++) {
            if (release_counter_[i] >= RELEASE_STUCK_THRESHOLD_CYCLES) {
                any_above_threshold = true;
                break;
            }
        }
        if (!any_above_threshold) {
            clear_dtc(DTC::C0050_CONTROL_LOOP_DEGRADED);
        }
    }
}
