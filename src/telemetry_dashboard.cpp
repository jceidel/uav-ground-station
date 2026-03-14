// Full telemetry monitoring with CSV logging.
//
// This program connects to PX4 SITL, subscribes to all telemetry streams, displays
// a formatted dashboard in the terminal, and logs everything to a CSV file for
// post-flight analysis
//
// Usage: ./telemetry_dashboard [connection_url]
// Default connetion: udpin://0.0.0.0:14540

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include "telemetry_monitor.h"
#include "csv_logger.h"

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <future>
#include <csignal>
#include <cstdint>
#include <memory>

using namespace mavsdk;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

// -----Global flag for clean shutdown----
// Use std::atomic for shared state between threads and signal handlers
// 
std::atomic<bool> g_should_quit{false};

void signal_handler(int /*signum*/)
{
    g_should_quit.store(true);
}

// -----Wait for system discovery-----
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

// -----Print a formatted telemetry dashboard-----
//
// "\033[2J"  = clear screen
// "\033[H"   = move cursor to top-left
// "\033[1m"  = bold text
// "\033[0m"  = reset formatting
//
void print_dashboard(const TelemetryFrame& f, const std::string& log_file)
{
    // Move cursor to top left
    std::cout << "\033[H"
              << "====================================================\n"
              << "   UAV GROUND STATION -- TELEMETRY DASHBOARD        \n"
              << "====================================================\n"
              << "\033[0m";

    std::cout << std::fixed;

    // Status line
    std::cout << "  Mode: \033[1m" << f.flight_mode << "\033[0m"
              << "    Armed: " << (f.armed ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "    In Air: " << (f.in_air ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "                    \n";

    std::cout << "\n";

    // Position
    std::cout << "  \033[1mPosition\033[0m\n"
              << "    Lat:  " << std::setprecision(7) << f.latitude_deg << "°"
              << "                    \n"
              << "    Lon:  " << std::setprecision(7) << f.longitude_deg << "°"
              << "                    \n"
              << "    MSL:  " << std::setprecision(2) << f.absolute_altitude_m << " m"
              << "                    \n"
              << "    AGL:  " << std::setprecision(2) << f.relative_altitude_m << " m"
              << "                    \n";

    std::cout << "\n";

    // Attitude
    std::cout << "  \033[1mAttitude\033[0m\n"
              << "    Roll:  " << std::setprecision(1) << std::setw(7) << f.roll_deg << "°"
              << "    Pitch: " << std::setw(7) << f.pitch_deg << "°"
              << "    Yaw: " << std::setw(7) << f.yaw_deg << "°"
              << "                    \n";

    std::cout << "\n";

    // Battery
    std::cout << "  \033[1mBattery\033[0m\n"
              << "    Voltage:   " << std::setprecision(2) << f.battery_voltage_v << " V"
              << "                    \n"
              << "    Remaining: " << std::setprecision(0) << (f.battery_remaining * 100.0f) << "%"
              << "                    \n";

    std::cout << "\n";

    // GPS
    std::cout << "  \033[1mGPS\033[0m\n"
              << "    Satellites: " << f.gps_num_satellites
              << "                    \n"
              << "    Fix type:   " << f.gps_fix_type
              << "                    \n";

    std::cout << "\n";

    // Log file
    std::cout << "  \033[1mLogging:\033[0m " << log_file
              << "                    \n";

    std::cout << "\n"
              << "  Press Ctrl+C to stop."
              << "                    \n";

    std::cout << std::flush;
}

// -----Main-----
int main(int argc, char* argv[])
{
    const std::string connection_url = 
        (argc > 1) ? argv[1] : "udpin://0.0.0.0:14540";

    // Install signal handler for Ctrl+C
    std::signal(SIGINT, signal_handler);

    std::cout << "=== UAV Ground Station -- Telemetry Dashboard ===" << std::endl;
    std::cout << "Connecting to: " << connection_url << std::endl;

    // -----Connect------
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

    // -----Set up telemetry and logging------
    TelemetryMonitor monitor{system};
    CSVLogger logger;     // Auto-generates timestamped filename

    if (!logger.is_open()) {
        std::cerr << "Failed to open log file." << std::endl;
        return 1;
    }

    monitor.start();

    // Give subscriptions a moment to receive first data
    sleep_for(seconds(2));

    // Clear screen for dashboard
    std::cout << "\033[2J";

    // ----- Main Loop------
    // Read telemetry snapshot, log it, display it, repeat.
    // Runs at ~2 Hz (every 500ms).
    // 
    while (!g_should_quit.load()) {
        TelemetryFrame frame = monitor.get_frame();

        // Log to CSV
        logger.log(frame);

        // Display dashboard
        print_dashboard(frame, logger.filename());

        // Sleep 500ms between updates
        sleep_for(milliseconds(500));
    }

    // -----Clean Shutdown-----
    std::cout << "\n\nShutting down..." << std::endl;
    monitor.stop();
    // CSVLogger destructor flushes and closes the files automatically

    std::cout << "Telemetry log saved to: " << logger.filename() << std::endl;
    std::cout << "Done." << std::endl;

    return 0;
}