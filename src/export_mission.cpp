//
// export_mission.cpp
//
// Standalone tool: plans an orbital photogrammetry mission and
// exports it as JSON for Unreal Engine visualization.
//
// No MAVSDK, no PX4, no drone connection needed.
//
// Usage: ./export_mission [output.json]
//

#include "orbital_planner.h"
#include "mission_export.h"
#include <iostream>
#include <cmath>

MissionConfig create_test_mission_config()
{
    MissionConfig config;

    // Simulated rock — 10m x 8m footprint, 7m tall
    config.center.latitude_deg  = 47.397742 + (50.0 / 111320.0);
    config.center.longitude_deg = 8.545594;

    config.x_extent_coord.latitude_deg  = config.center.latitude_deg;
    config.x_extent_coord.longitude_deg = config.center.longitude_deg
        + (5.0 / (111320.0 * std::cos(config.center.latitude_deg * 3.14159265 / 180.0)));

    config.z_extent_coord.latitude_deg  = config.center.latitude_deg
        + (4.0 / 111320.0);
    config.z_extent_coord.longitude_deg = config.center.longitude_deg;

    config.object_height_m = 7.0f;
    config.detail_gsd_mm   = 1.5f;
    config.overview_gsd_mm = 4.0f;

    config.camera.focal_length_mm     = 28.0f;
    config.camera.sensor_width_mm     = 35.8f;
    config.camera.sensor_height_mm    = 23.9f;
    config.camera.image_width_pixels  = 6000;
    config.camera.image_height_pixels = 4000;
    config.camera.overlap             = 0.75f;

    config.min_altitude_agl_m = 2.0f;
    config.ground_elevation_msl_m = 488.0f;
    config.extra_top_orbits = 2;

    return config;
}

int main(int argc, char* argv[])
{
    std::string output_file = (argc > 1) ? argv[1] : "mission_waypoints.json";

    std::cout << "Planning orbital photogrammetry mission..." << std::endl;

    MissionConfig config = create_test_mission_config();
    OrbitalMission mission = plan_mission(config);

    print_mission_summary(mission);

    export_mission_json(mission, output_file);

    return 0;
}