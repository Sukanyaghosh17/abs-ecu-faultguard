#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <filesystem>

#include "ABSController.hpp"
#include "FaultInjector.hpp"

using namespace std;

// ── Simulation parameters ──────────────────────────────────────────────────
constexpr double DT            = 0.020;   // Control cycle: 20 ms
constexpr double INITIAL_SPEED = 30.0;    // Starting speed: ~108 km/h

// Wheel physics (used in the plant model below, separate from ECU constants).
constexpr double MAX_PRESSURE    = 100.0;
constexpr double MAX_WHEEL_DECEL = 25.0;  // Max wheel deceleration (m/s²)
constexpr double RECOVERY_RATE   = 15.0;  // Wheel recovery when pressure releases (m/s²)

// ── Helper: map FaultType to a short label for console output ─────────────
static const char* fault_label(FaultType ft) {
    switch (ft) {
        case FaultType::NONE:         return "none";
        case FaultType::BIAS:         return "bias";
        case FaultType::LOCKUP:       return "lockup";
        case FaultType::DISCONNECTED: return "disconnected";
    }
    return "unknown";
}

// ── CLI argument parser ───────────────────────────────────────────────────
//
// Supported syntax (one --fault flag per wheel, repeatable):
//   --fault wheel=N type=bias        bias=V
//   --fault wheel=N type=lockup
//   --fault wheel=N type=disconnected
//
// Example:
//   ./abs_ecu_sim --fault wheel=2 type=lockup --fault wheel=0 type=bias bias=5.0
//
static void parse_faults(int argc, char* argv[], FaultInjector& injector) {
    for (int i = 1; i < argc; ) {
        string arg = argv[i++];

        if (arg != "--fault")
            continue;

        int       wheel       = -1;
        FaultType ftype       = FaultType::NONE;
        double    bias_val    = 0.0;
        bool      has_bias    = false;
        bool      parse_error = false;

        // Consume key=value tokens until we hit the next flag or end of args.
        while (i < argc && string(argv[i]).find('=') != string::npos) {
            string kv  = argv[i++];
            auto   eq  = kv.find('=');
            string key = kv.substr(0, eq);
            string val = kv.substr(eq + 1);

            try {
                if (key == "wheel") {
                    wheel = stoi(val);
                } else if (key == "type") {
                    if      (val == "bias")         ftype = FaultType::BIAS;
                    else if (val == "lockup")       ftype = FaultType::LOCKUP;
                    else if (val == "disconnected") ftype = FaultType::DISCONNECTED;
                    else {
                        cerr << "Warning: invalid --fault argument '" << kv
                             << "', ignoring.\n";
                        parse_error = true;
                    }
                } else if (key == "bias") {
                    bias_val = stod(val);
                    has_bias = true;
                }
            } catch (const std::exception&) {
                cerr << "Warning: invalid --fault argument '" << kv
                     << "', ignoring.\n";
                parse_error = true;
            }
        }

        if (parse_error)
            continue;

        // Validate and register the fault.
        if (wheel < 0 || wheel >= FaultInjector::NUM_WHEELS) {
            cerr << "Warning: invalid wheel index " << wheel
                 << " — must be 0-3. Skipping.\n";
            continue;
        }
        if (ftype == FaultType::NONE) {
            cerr << "Warning: no valid type specified for --fault wheel=" << wheel
                 << " — skipping.\n";
            continue;
        }
        if (ftype == FaultType::BIAS && !has_bias) {
            cerr << "Warning: type=bias specified for wheel " << wheel
                 << " without a bias= value — skipping.\n";
            continue;
        }

        injector.set_fault(wheel, ftype, bias_val);
    }
}

// ── Main ──────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    std::filesystem::create_directories("logs");

    cout << "ABS ECU Simulation Started\n";
    cout << "  Initial speed : " << INITIAL_SPEED << " m/s ("
         << fixed << setprecision(1) << INITIAL_SPEED * 3.6 << " km/h)\n";
    cout << "  Cycle time    : " << DT * 1000.0 << " ms\n\n";

    // ── Set up fault injection ───────────────────────────────────────────
    FaultInjector fault_injector;
    parse_faults(argc, argv, fault_injector);

    // Print any active faults so the user can confirm what was configured.
    bool any_fault = false;
    for (int w = 0; w < FaultInjector::NUM_WHEELS; w++) {
        if (fault_injector.get_fault_type(w) != FaultType::NONE) {
            if (!any_fault) {
                cout << "Active faults:\n";
                any_fault = true;
            }
            cout << "  Wheel " << w << " : "
                 << fault_label(fault_injector.get_fault_type(w)) << "\n";
        }
    }
    if (!any_fault)
        cout << "No faults injected — nominal run.\n";
    cout << "\n";

    // ── Initialise simulation objects ────────────────────────────────────
    Vehicle       vehicle(INITIAL_SPEED);
    ABSController ecu(vehicle, INITIAL_SPEED);

    // Always attach the injector; if no faults were configured all wheels
    // will pass through unchanged (FaultType::NONE is the default).
    ecu.set_fault_injector(&fault_injector);

    double sim_time = 0.0;

    // ── Main simulation loop ─────────────────────────────────────────────
    while (vehicle.get_speed() > 0.5) {

        // ── ECU step ────────────────────────────────────────────────────
        ecu.control_cycle(sim_time, DT);

        // ── Plant (physics) update ───────────────────────────────────────
        // Update wheel speeds and vehicle speed based on actuator commands.
        double avg_pressure = 0.0;

        auto& actuators = ecu.get_actuators();
        auto& sensors   = ecu.get_sensors();

        for (int i = 0; i < 4; i++) {
            double p = actuators[i].get_pressure();
            avg_pressure += p;

            // Decelerate the wheel proportionally to brake pressure.
            double wheel_decel = (p / MAX_PRESSURE) * MAX_WHEEL_DECEL;
            double new_speed   = sensors[i].get_true_speed() - wheel_decel * DT;

            // If ABS released pressure, the wheel begins to recover.
            if (actuators[i].get_state() == BrakeState::RELEASE) {
                new_speed += RECOVERY_RATE * DT;
            }

            // Wheel speed cannot exceed vehicle speed (no wheel spin scenario)
            // nor go below zero.
            new_speed = clamp(new_speed, 0.0, vehicle.get_speed() * 1.02);

            sensors[i].update(new_speed);
        }

        avg_pressure /= 4.0;
        vehicle.update(avg_pressure, DT);

        sim_time += DT;

        // Pace the output at the real-time cycle rate.
        this_thread::sleep_for(chrono::milliseconds(20));

        // ── Console output ───────────────────────────────────────────────
        cout << fixed << setprecision(2);
        cout << "t = "       << sim_time
             << " s | Vehicle: " << vehicle.get_speed()
             << " m/s | Ref: "   << ecu.estimate_vehicle_speed()
             << " m/s\n";

        for (int i = 0; i < 4; i++) {
            string state_str;
            switch (actuators[i].get_state()) {
                case BrakeState::APPLY:   state_str = "APPLY";   break;
                case BrakeState::HOLD:    state_str = "HOLD";    break;
                case BrakeState::RELEASE: state_str = "RELEASE"; break;
            }

            // Append a fault indicator to the wheel line if one is active.
            string fault_str = "";
            if (fault_injector.get_fault_type(i) != FaultType::NONE)
                fault_str = string(" [FAULT:") + fault_label(fault_injector.get_fault_type(i)) + "]";

            cout << "  Wheel " << i
                 << ": " << sensors[i].read() << " m/s"
                 << " | " << actuators[i].get_pressure() << "%"
                 << " | " << state_str
                 << fault_str << "\n";
        }

        cout << "------------------------------\n";
    }

    cout << "\nSimulation finished. Logs saved to logs/abs_log.csv and logs/can_bus.csv\n";
    return 0;
}