// Implementation of the TelemetryMonitor class.
// Each subscription callback locks the mutex, updates the relevant
// fields in _current_frame, and unlocks.
//

#include "telemetry_monitor.h"
#include <chrono>
#include <iostream>

using namespace mavsdk;

// ----Constructor----
TelemetryMonitor::TelemetryMonitor(std::shared_ptr<System> system)
    : _telemetry{system}
{
}

// ----now_seconds()----
// Returns the current time as fractional seconds since the Unix epoch?

double TelemetryMonitor::now_seconds()
{
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}

// ----start()-----
// Registers all telemetry subscriptions. Each callback folows the same pattern:
//  1. Lock the mutex (using lock_guard for automatic unlock)
//  2. Update the relevant fields in _current_frame 
//  3. Mutex automatically unlocks when lock_gaurd goes out of scope
//
void TelemetryMonitor::start()
{
    // Set update rates (Hz) for telemetry streams.
    // Higher rates = more data but more CPU and bandwidth.
    // 2 Hz is a good balance for logging and display.
    _telemetry.set_rate_position(2.0);
    _telemetry.set_rate_attitude_euler(2.0);
    _telemetry.set_rate_battery(0.5);
    _telemetry.set_rate_gps_info(1.0);

    // Poistion subscription
    _pos_handle = _telemetry.subscribe_position(
        [this](Telemetry::Position pos) {
            // "this" is needed so we can access _mutex and _current_frame
            // from inside the lambda. The lambda runs on MAVSDK's thread
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.timestamp_s          = now_seconds();
            _current_frame.latitude_deg         = pos.latitude_deg;
            _current_frame.longitude_deg        = pos.longitude_deg;
            _current_frame.absolute_altitude_m  = pos.absolute_altitude_m;
            _current_frame.relative_altitude_m  = pos.relative_altitude_m;
        }
    );

    // ----Attitude subscription (Euler angles)
    _att_handle = _telemetry.subscribe_attitude_euler(
        [this](Telemetry::EulerAngle euler) {
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.roll_deg  = euler.roll_deg;
            _current_frame.pitch_deg = euler.pitch_deg;
            _current_frame.yaw_deg   = euler.yaw_deg;
        }
    );

    // ----Battery subscription----
    _bat_handle = _telemetry.subscribe_battery(
        [this](Telemetry::Battery battery) {
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.battery_voltage_v  = battery.voltage_v;
            _current_frame.battery_remaining  = battery.remaining_percent / 100.0f;
        }
    );

    // ----GPS info subscription----
    _gps_handle = _telemetry.subscribe_gps_info(
        [this](Telemetry::GpsInfo gps) {
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.gps_num_satellites = gps.num_satellites;
            _current_frame.gps_fix_type       = static_cast<int>(gps.fix_type);
        }
    );

    // ----FLight Mode subscription----
    _mode_handle = _telemetry.subscribe_flight_mode(
        [this](Telemetry::FlightMode mode) {
            std::string mode_str;
            switch (mode) {
                case Telemetry::FlightMode::Ready:       mode_str = "READY"; break;
                case Telemetry::FlightMode::Takeoff:     mode_str = "TAKEOFF"; break;
                case Telemetry::FlightMode::Hold:        mode_str = "HOLD"; break;
                case Telemetry::FlightMode::Mission:     mode_str = "MISSION"; break;
                case Telemetry::FlightMode::ReturnToLaunch: mode_str = "RTL"; break;
                case Telemetry::FlightMode::Land:        mode_str = "LAND"; break;
                case Telemetry::FlightMode::Offboard:    mode_str = "OFFBOARD"; break;
                case Telemetry::FlightMode::Manual:      mode_str = "MANUAL"; break;
                case Telemetry::FlightMode::Posctl:      mode_str = "POSCTL"; break;
                case Telemetry::FlightMode::Altctl:      mode_str = "ALTCTL"; break;
                case Telemetry::FlightMode::Stabilized:  mode_str = "STABILIZED"; break;
                default:                                 mode_str = "UNKNOWN"; break;
            }
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.flight_mode = mode_str;
        }
    );

    // ----Armed subscription----
    _armed_handle = _telemetry.subscribe_armed(
        [this](bool armed) {
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.armed = armed;
        }
    );

    // ---- In-air subscription----
    _in_air_handle = _telemetry.subscribe_in_air(
        [this](bool in_air) {
            std::lock_guard<std::mutex> lock(_mutex);
            _current_frame.in_air = in_air;
        }
    );

    std::cout << "[TelemetryMonitor] Subscriptions active." << std::endl;
}

// ---- stop() ----
// Unsubscribe all callbacks. Important for clean shutdown —
// if the TelemetryMonitor is destroyed while callbacks are
// still registered, they'll try to access freed memory
// (a "use-after-free" bug, one of the nastiest in C++).
//
void TelemetryMonitor::stop()
{
    _telemetry.unsubscribe_position(_pos_handle);
    _telemetry.unsubscribe_attitude_euler(_att_handle);
    _telemetry.unsubscribe_battery(_bat_handle);
    _telemetry.unsubscribe_gps_info(_gps_handle);
    _telemetry.unsubscribe_flight_mode(_mode_handle);
    _telemetry.unsubscribe_armed(_armed_handle);
    _telemetry.unsubscribe_in_air(_in_air_handle);

    std::cout << "[TelemetryMonitor] Subscriptions stopped." << std::endl;
}

// ── get_frame() ─────────────────────────────────────────────
// Returns a COPY of the current telemetry frame.
// The lock_guard ensures we don't read while a callback is
// writing. The copy means the caller owns its own data and
// can use it without holding the lock.
//
TelemetryFrame TelemetryMonitor::get_frame() const
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _current_frame;
}
