// Arm the vehicle, take off, and land
// Usage: ./takeoff_land [connection_url]
// Default connection: udpin://0.0.0.0:14540
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/action/action.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cstdint>
#include <memory>

using namespace mavsdk;
using::std::chrono::seconds;
using::std::this_thread::sleep_for;

// Helper: Wait for system 
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
        return nullptr; // No system found
    }

    return future.get();
}

int main(int argc, char* argv[])
{
    const std::string connection_url = 
        (argc > 1) ? argv[1] : "udpin://0.0.0.0:14540";

    std::cout << "=== UAV Ground Station - Takeoff and Land ===" << std::endl;
    std::cout << "Connecting to: " << connection_url << std::endl;

    // Connect to Mavsdk
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
    const ConnectionResult conn_result = mavsdk.add_any_connection(connection_url);

    if (conn_result != ConnectionResult::Success) {
        std::cerr << "Connection Failed" << conn_result << std::endl;
        return 1;
    }

    // Discover the vehicle
    auto system = wait_for_system(mavsdk);
    if (!system) {
        std::cerr << "Timed out waiting for system." << std::endl;
        return 1;
    }
    std::cout << "System discovered" << std::endl;

    // Instantiate Plugins
    // Mavsdk plugins take a shared_ptr<System> because the system might be shared 
    // across multiple plugins and threads.
    //
    auto telemetry = Telemetry{system};
    auto action = Action{system};

    // Subscribe to telemetry (position at 1Hz)
    // MAVSDK calls [](Telemetry::Position pos) { ... } onn its own internal thread
    // everytime new position data arrives.
    //
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting telemetry rate failed:" << set_rate_result << std::endl;
        return 1;
    }

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "  Alt: " << position.relative_altitude_m << " m  "
                  << "Lat: " << position.latitude_deg << "  "
                  << "Lon: " << position.longitude_deg
                  << std::endl;
    });

    // Wait until the vehicle is ready to arm
    // PX4 runs pre_flight checks (GPS lock, IMU calibrated, etc.).
    // health_all_ok() returns true when all checks pass.
    std::cout << "Waiting for vehicle to be ready..." << std::endl;
    while (!telemetry.health_all_ok()) {
        sleep_for(seconds(1));
    }
    std::cout << "Vehicle is ready." << std::endl;

    // -----Arm-----
    // "Arming" enables the motors.
    std::cout << "Arming..." << std::endl;
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << std::endl;
        return 1;
    }
    std::cout << "Armed." << std::endl;

    // -----Take off-----
    // Commands PX4 to switch to Takeoff mode and climb to default takeoff altitude
    std::cout << "Taking off..." << std::endl;
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << std::endl;
        return 1;
    }

    // Wait for drone to reach altitude
    std::cout << "Ascending... (waiting 10 sesonds)" << std::endl;
    sleep_for(seconds(10));

    // -----Land-----
    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Land failed: " << land_result << std::endl;
        return 1;
    }

    // Wait for land to complete
    // Check in air "in_air" telemetry to know when its done.
    while (telemetry.in_air()) {
        sleep_for(seconds(1));
    }

    std::cout << "Landed. Disarming..." << std::endl;
    sleep_for(seconds(2)); // Give PX4 a moment to settle

    std::cout << std::endl;
    std::cout << "=== MISSION COMPLETE ===" << std::endl;

    return 0;
}