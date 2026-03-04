//
// connection_test.cpp
// 
// Connect to PX4 SITL and print system info.
// This verifies that MAVSDK is installed correctly, PX4 SITL is running,
// and UDP communication is working.
//
// Usage: ./connection_test [connection_url]
// Default connection: udpin://0.0.0.0:14540
//

#include <mavsdk/mavsdk.h>
#include <mavsdk/component_type.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cstdint>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main(int argc, char* argv[])
{
    // --- Parse connection URL from command line ---
    // Default is the standard PX4 SITL UDP port.
    // "udpin" means we're listening for incoming UDP packets
    // (PX4 broadcasts heartbeat messages; we receive them).
    const std::string connection_url = 
        (argc >1) ? argv[1] : "udpin://0.0.0.0:14540";

    std::cout << "=== UAV Ground Station -- Connection Test ===" << std::endl;
    std::cout << "Connecting to:" << connection_url << std::endl;

    // --- Create the MAVSDK instance ---
    // The Configuration tells MAVSDK what role we play in the
    // MAVLink network. We are a GroundStation (GCS), not an
    // autopilot or a camera.
    Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};

    //---Add UDP connection---
    //This tells the MAVSDK to listen on the specified 
    // UDP port for MAVLink heartbeat messages from PX4.
    const ConnectionResult conn_result = mavsdk.add_any_connection(connection_url);

    if (conn_result != ConnectionResult::Success) {
        std::cerr << "ERROR: Connection failed: " << conn_result << std::endl;
        return 1;
    }
    std::cout << "Connection Initiated. Waiting for system.." << std::endl;

    //---Wait for a system (vehicle) to appear---
    // px4 sends heartbeat message at 1 Hz. MAVSDK uses these
    // to discover systems on the network. We use a promise/future
    // pattern to wait asynchonously.
    auto promise = std::make_shared<std::promise<std::shared_ptr<System>>>();
    auto future = promise->get_future();

    // subscribe_on_a_new_system is a callback: MAVSDK will call this
    // function every time it discovers a new system on the network.
    Mavsdk::NewSystemHandle handle = mavsdk.subscribe_on_new_system(
        [&mavsdk, promise, &handle]() {
            // Get the most recent discovered system
            auto system = mavsdk.systems().back();

            //We only care about the systems with an autopilot
            // (as opposed to cameras, gimbals, etc.)
            if (system->has_autopilot()) {
                std::cout << "Discovered autopilot" << std::endl;

                // Unsubscribe so this callback doesnt fire again
                mavsdk.unsubscribe_on_new_system(handle);

                //Fulfill the promise (unblocks future.get())
                promise->set_value(system);
            }
        }
    );

    // Block here for up to 10 secondswaiting for a system.
    // If PX4 SITL is running, this should resolve in 1-2 seconds.
    if  (future.wait_for(seconds(10)) == std::future_status::timeout) {
        std::cerr << "Error: No autopilot found within 10 seconds." << std::endl;
        std::cerr << "Is PX4 SITL running? Try cd ~/PX4-Autopilot && make px4_sitl gz_x500"
                  << std::endl;
        return 1;
    }

    // System discovered -- print info
    auto system = future.get();

    std::cout << std::endl;
    std::cout << "  System ID:       " << static_cast<int>(system->get_system_id()) << std::endl;
    std::cout << "  Has autopilot:   " << (system->has_autopilot() ? "YES" : "NO") << std::endl;
    std::cout << "  Is connected:    " << (system->is_connected() ? "YES" : "NO") << std::endl;
    std::cout << std::endl;

    std::cout << "SUCCESS: Ground station connected to PX4 SITL!" << std::endl;
    std::cout << "You are ready for Milestone 2 (takeoff & land)." << std::endl;

    return 0;
}