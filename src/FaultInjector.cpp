#include "FaultInjector.hpp"
#include <stdexcept>

void FaultInjector::set_fault(int wheel, FaultType type, double bias) {
    if (wheel < 0 || wheel >= NUM_WHEELS) {
        throw std::out_of_range("Wheel index out of range in FaultInjector::set_fault");
    }
    faults_[wheel].type       = type;
    faults_[wheel].bias_value = bias;
}

double FaultInjector::apply(int wheel, double raw_reading) const {
    const WheelFault& fault = faults_[wheel];

    switch (fault.type) {
        case FaultType::NONE:
            // Normal sensor — pass through unchanged.
            return raw_reading;

        case FaultType::BIAS:
            // Fixed electromagnetic / calibration offset.
            return raw_reading + fault.bias_value;

        case FaultType::LOCKUP:
            // Wheel appears to be fully locked; return near-zero speed.
            // Not exactly 0.0 to avoid a divide-by-zero inside the ECU
            // (which guards against est_veh < 1.0, not against wheel_sp == 0).
            return 0.1;

        case FaultType::DISCONNECTED:
            // Open-circuit wire — sensor always reads zero.
            return 0.0;
    }

    // Unreachable, but keeps compilers happy.
    return raw_reading;
}

FaultType FaultInjector::get_fault_type(int wheel) const {
    return faults_[wheel].type;
}
