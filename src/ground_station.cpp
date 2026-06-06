//
// ground_station.cpp
//
// Merged ground station: telemetry monitoring, CSV logging,
// and orbital photogrammetry mission execution in one program.
//
// Usage:
//   ./ground_station [connection_url]
//
// Default connection: udpin://0.0.0.0:14540
//
// Currently uses a hardcoded test mission (a simulated rock at
// the PX4 SITL default location). In later milestones this will
// accept mission parameters from a config file or command line.
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "telemetry_monitor.h"
#include "csv_logger.h"
#include "orbital_planner.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <future>
#include <csignal>
#include <memory>
#include <cstdint>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// ── Clean shutdown flag ─────────────────────────────────────
std::atomic<bool> g_should_quit{false};

void signal_handler(int /*signum*/)
{
    g_should_quit.store(true);
}

// ── Wait for system discovery ───────────────────────────────
std::shared_ptr<System> wait_for_system(Mavsdk& mavsdk, int timeout_s = 10)
{
    auto promise = std::make_shared<std::promise<std::shared_ptr<System>>>();
    auto future = promise->get_future();

    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system(
        [&mavsdk, promise, &handle]() {
            auto system = mavsdk.systems().back();
            if (system->has_autopilot()) {
                mavsdk.unsubscribe_on_new_system(handle);
                promise->set_value(system);
            }
        }
    );

    if (future.wait_for(seconds(timeout_s)) == std::future_status::timeout) {
        return nullptr;
    }
    return future.get();
}

// ── Convert our Mission to MAVSDK MissionItems ──────────────
//
// MAVSDK's Mission plugin takes a vector of MissionItem objects.
// Each MissionItem is a waypoint with lat/lon/alt plus optional
// camera and gimbal actions.
//
// This function flattens our Mission (passes -> orbits -> waypoints)
// into a single linear sequence that MAVSDK can upload to PX4.
//
std::vector<Mission::MissionItem> mission_to_mavsdk_items(const OrbitalMission& orbital_mission)
{
    std::vector<Mission::MissionItem> items;

    for (const auto& pass : orbital_mission.passes) {
        for (const auto& orbit : pass.orbits) {
            for (const auto& wp : orbit.waypoints) {
                Mission::MissionItem item;

                item.latitude_deg  = wp.coordinate.latitude_deg;
                item.longitude_deg = wp.coordinate.longitude_deg;
                item.relative_altitude_m = wp.altitude_m
                    - orbital_mission.config.ground_elevation_msl_m;

                // Speed: slower for detail, faster for overview
                item.speed_m_s = (pass.type == PassType::OVERVIEW)
                                 ? 3.0f : 2.0f;

                // Gimbal pitch and yaw
                item.gimbal_pitch_deg = wp.gimbal_pitch;
                item.gimbal_yaw_deg   = wp.heading_deg;

                // Camera trigger at every waypoint
                item.camera_action = wp.trigger_camera
                    ? Mission::MissionItem::CameraAction::TakePhoto
                    : Mission::MissionItem::CameraAction::None;

                items.push_back(item);
            }
        }
    }

    return items;
}

// ── Print dashboard (same as telemetry_dashboard but with mission info)
void print_dashboard(const TelemetryFrame& f, const std::string& log_file,
                     const std::string& mission_status, int wp_current, int wp_total)
{
    std::cout << "\033[H";
    std::cout << "\033[1m"
              << "═══════════════════════════════════════════════════\n"
              << "  UAV GROUND STATION — ORBITAL PHOTOGRAMMETRY     \n"
              << "═══════════════════════════════════════════════════\n"
              << "\033[0m";

    std::cout << std::fixed;

    std::cout << "  Mode: \033[1m" << f.flight_mode << "\033[0m"
              << "    Armed: " << (f.armed ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "    In Air: " << (f.in_air ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "                    \n";

    std::cout << "\n";

    std::cout << "  \033[1mPosition\033[0m\n"
              << "    Lat:  " << std::setprecision(7) << f.latitude_deg << "°"
              << "                    \n"
              << "    Lon:  " << std::setprecision(7) << f.longitude_deg << "°"
              << "                    \n"
              << "    AGL:  " << std::setprecision(2) << f.relative_altitude_m << " m"
              << "                    \n";

    std::cout << "\n";

    std::cout << "  \033[1mAttitude\033[0m\n"
              << "    Roll:  " << std::setprecision(1) << std::setw(7) << f.roll_deg << "°"
              << "    Pitch: " << std::setw(7) << f.pitch_deg << "°"
              << "    Yaw: " << std::setw(7) << f.yaw_deg << "°"
              << "                    \n";

    std::cout << "\n";

    std::cout << "  \033[1mBattery\033[0m  "
              << std::setprecision(2) << f.battery_voltage_v << " V  ("
              << std::setprecision(0) << (f.battery_remaining * 100.0f) << "%)"
              << "                    \n";

    std::cout << "  \033[1mGPS\033[0m      "
              << f.gps_num_satellites << " sats, fix " << f.gps_fix_type
              << "                    \n";

    std::cout << "\n";

    std::cout << "  \033[1mMission\033[0m  " << mission_status
              << "                    \n";
    if (wp_total > 0) {
        std::cout << "  \033[1mProgress\033[0m "
                  << wp_current << " / " << wp_total << " waypoints"
                  << "                    \n";
    }

    std::cout << "\n";

    std::cout << "  \033[1mLog:\033[0m " << log_file
              << "                    \n";
    std::cout << "  Press Ctrl+C to abort mission."
              << "                    \n";

    std::cout << std::flush;
}

// ── Create a test mission for SITL ──────────────────────────
//
// PX4 SITL default home position is near Zurich, Switzerland:
//   Lat: 47.397742, Lon: 8.545594, Alt: 488m MSL
//
// We place a simulated 10m x 8m x 7m rock formation 50m north
// of the home position for testing.
//
MissionConfig create_test_mission_config()
{
    MissionConfig config;

    // Simulated rock center: 50m north of SITL home
    config.center.latitude_deg  = 47.397742 + (50.0 / 111320.0);
    config.center.longitude_deg = 8.545594;

    // Elliptical footprint: 5m x-radius, 4m z-radius
    // (These are GPS points at the edge of the rock)
    config.x_extent_coord.latitude_deg  = config.center.latitude_deg;
    config.x_extent_coord.longitude_deg = config.center.longitude_deg
        + (5.0 / (111320.0 * std::cos(config.center.latitude_deg * 3.14159265 / 180.0)));

    config.z_extent_coord.latitude_deg  = config.center.latitude_deg
        + (4.0 / 111320.0);
    config.z_extent_coord.longitude_deg = config.center.longitude_deg;

    config.object_height_m = 7.0f;

    // Quality
    config.detail_gsd_mm   = 1.5f;
    config.overview_gsd_mm = 4.0f;

    // Camera: Sony A7II with 28mm
    config.camera.focal_length_mm     = 28.0f;
    config.camera.sensor_width_mm     = 35.8f;
    config.camera.sensor_height_mm    = 23.9f;
    config.camera.image_width_pixels  = 6000;
    config.camera.image_height_pixels = 4000;
    config.camera.overlap             = 0.75f;

    // Safety
    config.min_altitude_agl_m = 2.0f;
    config.ground_elevation_msl_m = 488.0f;  // SITL default
    config.extra_top_orbits = 2;

    return config;
}

// ══════════════════════════════════════════════════════════════
// Main
// ══════════════════════════════════════════════════════════════

int main(int argc, char* argv[])
{
    const std::string connection_url =
        (argc > 1) ? argv[1] : "udpin://0.0.0.0:14540";

    std::signal(SIGINT, signal_handler);

    // ── Plan the mission first (no connection needed) ───────
    std::cout << "Planning orbital photogrammetry mission...\n" << std::endl;

    MissionConfig mission_config = create_test_mission_config();
    OrbitalMission orbital_mission = plan_mission(mission_config);

    print_mission_summary(orbital_mission);

    std::cout << "Press Enter to connect and fly, or Ctrl+C to abort." << std::endl;
    std::cin.get();

    if (g_should_quit.load()) return 0;

    // ── Connect to MAVSDK ───────────────────────────────────
    std::cout << "Connecting to: " << connection_url << std::endl;

    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    const ConnectionResult conn_result = mavsdk.add_any_connection(connection_url);

    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << conn_result << std::endl;
        return 1;
    }

    auto system = wait_for_system(mavsdk);
    if (!system) {
        std::cerr << "Timed out waiting for system." << std::endl;
        return 1;
    }
    std::cout << "System discovered." << std::endl;

    // ── Set up plugins ──────────────────────────────────────
    auto action = Action{system};
    auto mission_plugin = mavsdk::Mission{system};

    // ── Start telemetry and logging ─────────────────────────
    TelemetryMonitor monitor{system};
    CSVLogger logger;
    monitor.start();

    if (!logger.is_open()) {
        std::cerr << "Failed to open log file." << std::endl;
        return 1;
    }

    sleep_for(seconds(2));  // Let telemetry populate

    // ── Convert and upload mission ──────────────────────────
    std::cout << "Converting mission to MAVSDK format..." << std::endl;

    auto mavsdk_items = mission_to_mavsdk_items(orbital_mission);
    int total_wps = static_cast<int>(mavsdk_items.size());

    std::cout << "Uploading " << total_wps << " waypoints..." << std::endl;

    mavsdk::Mission::MissionPlan plan;
    plan.mission_items = mavsdk_items;

    auto upload_result = mission_plugin.upload_mission(plan);
    if (upload_result != mavsdk::Mission::Result::Success) {
        std::cerr << "Mission upload failed: " << upload_result << std::endl;
        monitor.stop();
        return 1;
    }
    std::cout << "Mission uploaded." << std::endl;

    // ── Wait for vehicle ready ──────────────────────────────
    auto telemetry = Telemetry{system};
    std::cout << "Waiting for vehicle to be ready..." << std::endl;
    while (!telemetry.health_all_ok() && !g_should_quit.load()) {
        sleep_for(seconds(1));
    }
    if (g_should_quit.load()) { monitor.stop(); return 0; }

    // ── Arm and start mission ───────────────────────────────
    std::cout << "Arming..." << std::endl;
    auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        monitor.stop();
        return 1;
    }

    std::cout << "Starting mission..." << std::endl;
    auto start_result = mission_plugin.start_mission();
    if (start_result != mavsdk::Mission::Result::Success) {
        std::cerr << "Mission start failed: " << start_result << std::endl;
        monitor.stop();
        return 1;
    }

    // ── Clear screen for dashboard ──────────────────────────
    std::cout << "\033[2J";

    std::string mission_status = "EXECUTING";
    int current_wp = 0;

    // ── Main loop: monitor mission progress ─────────────────
    while (!g_should_quit.load()) {
        TelemetryFrame frame = monitor.get_frame();
        logger.log(frame);

        // Check mission progress
        auto progress = mission_plugin.mission_progress();
        current_wp = progress.current;

        if (progress.current >= progress.total) {
            mission_status = "COMPLETE";
            print_dashboard(frame, logger.filename(),
                           mission_status, current_wp, total_wps);
            break;
        }

        print_dashboard(frame, logger.filename(),
                       mission_status, current_wp, total_wps);

        sleep_for(milliseconds(500));
    }

    // ── Land after mission ──────────────────────────────────
    if (!g_should_quit.load()) {
        std::cout << "\n\nMission complete. Landing..." << std::endl;
        auto land_result = action.land();
        if (land_result != Action::Result::Success) {
            std::cerr << "Land command failed: " << land_result << std::endl;
        }

        // Wait for landing
        while (telemetry.in_air() && !g_should_quit.load()) {
            TelemetryFrame frame = monitor.get_frame();
            logger.log(frame);
            mission_status = "LANDING";
            print_dashboard(frame, logger.filename(),
                           mission_status, total_wps, total_wps);
            sleep_for(milliseconds(500));
        }
    }

    // ── Clean shutdown ──────────────────────────────────────
    std::cout << "\n\nShutting down..." << std::endl;
    monitor.stop();
    std::cout << "Telemetry log: " << logger.filename() << std::endl;
    std::cout << "Done." << std::endl;

    return 0;
}