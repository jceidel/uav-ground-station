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
              << "____________________________________________________\n"
              << "   UAV GROUND STATION -- TELEMETRY DASHBOARD        \n"
              << "____________________________________________________\n"
              << "\033[0m";

    std::cout << std::fixed;

    // Status line
    std::cout << "  Mode: \033[1m" << f.flight_mode << "\033[0m"
              << "    Armed: " << (f.armed ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "    In Air: " << (f.in_air ? "\033[32mYES\033[0m" : "\033[31mNO\033[0m")
              << "                    \n";

    std::cout << "\n";
}