// Subscribes to all relevant PX4 telemetry streams via MAVSDK and stores
// the latest values. Access is thread-safe because MAVSDK delivers
// callbacks on its own internal threads, but we read from main().
//
// - Header guards (#pragma once)
// - std::mutex thread safety
// - std::atomic lock-free shared state

#pragma once
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mutex>
#include <atomic>
#include <string>

// ---Snapshot of all telemetry at one instant---
struct TelemetryFrame {
    // Timestamp (seconds since epoch, from system clock)
    double timestamp_s = 0.0;

    // Position
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    float absolute_altitude_m = 0.0f;  // MSL (above sea level)
    float relative_altitude_m = 0.0f;  // AGL (above takeoff)

    // Attitude (Euler angles in degrees)
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;

    // Battery
    float battery_voltage_v = 0.0f;
    float battery_remaining = 0.0f;

    // GPS
    int gps_num_satellites = 0;
    int gps_fix_type = 0;

    // Flight status
    std::string flight_mode = "UNKNOWN";
    bool armed              = false;
    bool in_air             = false;
};

class TelemetryMonitor {
public:
    // ---Constructor---
    // Takes a shared_ptr to a System (the connected vehicle).
    // Store it and create the Telemetry plugin from it.
    //
    explicit TelemetryMonitor(std::shared_ptr<mavsdk::System> system);

    // ---Start and Stop subscriptions---
    // start() registers all the telemetry callbacks with MAVSDK.
    // stop() unsubscribe them
    void start();
    void stop();

    TelemetryFrame get_frame() const;

private:
    // The MAVSDK telemetry plugin instance
    mavsdk::Telemetry _telemetry;

    // Latest telemetry data, updated callbacks
    TelemetryFrame _current_frame;

    // ---Mutex for thread safety---
    mutable std::mutex _mutex;

    //---Subscription handles---
    ///MAVSDK v3 returns handles when subscribed
    mavsdk::Telemetry::PositionHandle           _pos_handle;
    mavsdk::Telemetry::AttitudeEulerHandle       _att_handle;
    mavsdk::Telemetry::BatteryHandle            _bat_handle;
    mavsdk::Telemetry::GpsInfoHandle            _gps_handle;
    mavsdk::Telemetry::FlightModeHandle         _mode_handle;
    mavsdk::Telemetry::ArmedHandle              _armed_handle;
    mavsdk::Telemetry::InAirHandle              _in_air_handle;

    // Helper:current time fractional seconds
    static double now_seconds();
};