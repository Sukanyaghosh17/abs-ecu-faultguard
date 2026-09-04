#include "ABSController.hpp"
#include <iomanip>
#include <algorithm>
#include <numeric>

using namespace std;

// ── ECU slip-control thresholds & reference speed rate ───────────────────
// These constants are defined as inline constexpr in ABSController.hpp
// so that unit tests can reference them by name.
// (SLIP_THRESHOLD, LOW_SLIP_THRESHOLD, MAX_REF_DECEL)

// ── Vehicle physics constants ──────────────────────────────────────────────
constexpr double MAX_PRESSURE  = 100.0;
constexpr double MAX_VEH_DECEL = 8.0;


// ════════════════════════════════════════════════════════════════════════════
// Vehicle
// ════════════════════════════════════════════════════════════════════════════

Vehicle::Vehicle(double initial)
    : speed(initial) {}

double Vehicle::get_speed() const {
    return speed;
}

void Vehicle::update(double avg_pressure, double dt) {
    double decel = (avg_pressure / MAX_PRESSURE) * MAX_VEH_DECEL;
    speed -= decel * dt;
    if (speed < 0.0)
        speed = 0.0;
}


// ════════════════════════════════════════════════════════════════════════════
// ABSController
// ════════════════════════════════════════════════════════════════════════════

ABSController::ABSController(Vehicle& v, double init_speed, const string& log_path)
    : vehicle(v),
      ref_speed_(init_speed),
      fault_injector_(nullptr) {

    for (int i = 0; i < 4; i++) {
        sensors.emplace_back(init_speed);
        actuators.emplace_back();
    }

    log_file.open(log_path);

    if (log_file.is_open()) {
        // S0-S3 : per-wheel ABS active flag (1 = this wheel is releasing pressure)
        // F0-F3 : fault type index (0=none, 1=bias, 2=lockup, 3=disconnected)
        // Active_DTCs : active diagnostic trouble codes
        log_file << "Time(s),Veh_Speed,Est_Veh_Speed,"
                 << "W0_Speed,W1_Speed,W2_Speed,W3_Speed,"
                 << "P0,P1,P2,P3,"
                 << "S0,S1,S2,S3,"
                 << "F0,F1,F2,F3,"
                 << "ABS_Active,Active_DTCs\n";
    }
}

ABSController::~ABSController() {
    if (log_file.is_open())
        log_file.close();
    can_bus_.dump_to_csv("logs/can_bus.csv");
}

// ── FaultInjector attachment ──────────────────────────────────────────────

void ABSController::set_fault_injector(FaultInjector* fi) {
    fault_injector_ = fi;
}

// ── Reference speed estimator ─────────────────────────────────────────────
//
// Strategy: peak-hold + deceleration limit.
//
//   ref_speed_ = max(
//       highest wheel reading this cycle,          // peak-hold anchor
//       ref_speed_ - MAX_REF_DECEL * dt            // max allowed drop
//   )
//
// Why this fixes the bug:
//   With the old average, when ALL four wheels slow together (simultaneous
//   lockup), est_veh drops with them, slip = (est_veh - wheel_sp)/est_veh
//   stays near zero, and the ECU never triggers RELEASE.
//   With peak-hold, ref_speed_ is anchored by the highest sensor and can
//   only decrease at 9 m/s² even if every wheel is reading zero — so slip
//   correctly shoots up and the ECU releases brake pressure.
//
void ABSController::update_reference_speed(const vector<double>& readings, double dt) {
    // Find the fastest-spinning wheel this cycle.
    double max_wheel = *max_element(readings.begin(), readings.end());

    // How far the reference is physically allowed to drop in one timestep.
    double decel_limited = ref_speed_ - MAX_REF_DECEL * dt;

    // Take whichever is higher: the peak wheel reading, or the decel-limited
    // carry-over from the previous cycle.
    ref_speed_ = max(max_wheel, decel_limited);

    if (ref_speed_ < 0.0)
        ref_speed_ = 0.0;
}

double ABSController::estimate_vehicle_speed() const {
    return ref_speed_;
}

// ── Main control cycle ────────────────────────────────────────────────────

void ABSController::control_cycle(double sim_time, double dt) {

    // ── Step 1: Sample every sensor exactly once ──────────────────────────
    // Reusing these values for both reference estimation and slip calculation
    // ensures both computations see a consistent snapshot of wheel speeds.
    // (The old code called sensor.read() twice, getting different noise each
    // time, which could produce nonsense slip values.)
    vector<double> readings(4);
    for (int i = 0; i < 4; i++) {
        readings[i] = sensors[i].read();

        // Apply fault injection if a FaultInjector is attached.
        // The ECU is completely unaware of whether a reading is faulted;
        // it just sees the (potentially corrupted) value.
        if (fault_injector_) {
            readings[i] = fault_injector_->apply(i, readings[i]);
        }
    }

    // ── Step 2: Update reference speed ───────────────────────────────────
    update_reference_speed(readings, dt);
    double ref_veh = ref_speed_;

    // Below 1 m/s the vehicle is essentially stopped; skip control to avoid
    // divide-by-zero in slip calculation.
    if (ref_veh < 1.0)
        return;

    // ── Step 3: Compute slip and command actuators ────────────────────────
    bool               abs_active = false;
    vector<double>     pressures(4);
    vector<int>        wheel_abs(4, 0);   // per-wheel ABS active flag
    vector<BrakeState> states(4);

    for (int i = 0; i < 4; i++) {
        double wheel_sp = readings[i];

        // Slip ratio: 0 = no slip, 1 = full lockup.
        double slip = (ref_veh - wheel_sp) / ref_veh;

        BrakeState cmd;

        if (slip > SLIP_THRESHOLD) {
            // Wheel is locking up — release pressure immediately.
            cmd          = BrakeState::RELEASE;
            abs_active   = true;
            wheel_abs[i] = 1;
        } else if (slip < LOW_SLIP_THRESHOLD) {
            // Plenty of grip — safe to increase braking force.
            cmd = BrakeState::APPLY;
        } else {
            // In the optimal slip window — hold current pressure.
            cmd = BrakeState::HOLD;
        }

        actuators[i].command(cmd);
        states[i]    = cmd;
        pressures[i] = actuators[i].get_pressure();
    }

    // ── Step 4: Diagnostics evaluation & CAN frame broadcast ──────────────
    diagnostics_.evaluate_cycle(sim_time, readings, ref_speed_, states);

    for (DTC code : diagnostics_.get_newly_raised()) {
        cout << ">>> [DIAGNOSTIC EVENT] DTC RAISED: " << dtc_to_string(code)
             << " at t = " << fixed << setprecision(2) << sim_time << " s\n";
    }
    for (DTC code : diagnostics_.get_newly_cleared()) {
        cout << ">>> [DIAGNOSTIC EVENT] DTC CLEARED: " << dtc_to_string(code)
             << " at t = " << fixed << setprecision(2) << sim_time << " s\n";
    }

    can_bus_.transmit(pack_wheel_speeds(sim_time, readings));
    can_bus_.transmit(pack_abs_status(sim_time, states, abs_active));
    can_bus_.transmit(pack_diagnostics(sim_time, diagnostics_.get_active_dtcs()));

    // ── Step 5: CSV logging ───────────────────────────────────────────────
    if (log_file.is_open()) {
        log_file << fixed << setprecision(3)
                 << sim_time         << ","
                 << vehicle.get_speed() << ","
                 << ref_speed_       << ",";

        // Post-fault sensor readings — the same values used by slip
        // calculation and the reference speed estimator.  Logging these
        // (instead of the raw true speed) makes fault effects (lockup, bias,
        // disconnected) visible in the CSV and the visualization dashboard.
        for (int i = 0; i < 4; i++)
            log_file << readings[i] << ",";

        // Brake pressures per wheel.
        for (int i = 0; i < 4; i++)
            log_file << pressures[i] << ",";

        // Per-wheel ABS active flag.
        for (int i = 0; i < 4; i++)
            log_file << wheel_abs[i] << ",";

        // Per-wheel fault type index (matches FaultType enum order).
        for (int i = 0; i < 4; i++) {
            int ft = 0;
            if (fault_injector_)
                ft = static_cast<int>(fault_injector_->get_fault_type(i));
            log_file << ft;
            if (i < 3) log_file << ",";
        }

        log_file << "," << (abs_active ? "1" : "0")
                 << "," << diagnostics_.get_active_dtcs_string() << "\n";
    }
}

// ── Accessors ─────────────────────────────────────────────────────────────

const vector<BrakeActuator>& ABSController::get_actuators() const {
    return actuators;
}

vector<WheelSpeedSensor>& ABSController::get_sensors() {
    return sensors;
}

DiagnosticsManager& ABSController::get_diagnostics() {
    return diagnostics_;
}

const DiagnosticsManager& ABSController::get_diagnostics() const {
    return diagnostics_;
}

CANBus& ABSController::get_can_bus() {
    return can_bus_;
}

const CANBus& ABSController::get_can_bus() const {
    return can_bus_;
}