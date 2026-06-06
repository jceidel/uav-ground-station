//
// mission_export.cpp
//
// Converts GPS waypoints to local XYZ coordinates and exports
// as JSON for Unreal Engine visualization.
//

#include "mission_export.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>

static constexpr double PI = 3.14159265358979323846;
static constexpr double DEG_TO_RAD = PI / 180.0;
static constexpr double METERS_PER_DEG_LAT = 111320.0;

bool export_mission_json(const OrbitalMission& mission,
                         const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "[Export] Could not open " << filename << std::endl;
        return false;
    }

    const auto& center = mission.config.center;
    double center_lat_rad = center.latitude_deg * DEG_TO_RAD;

    file << std::fixed << std::setprecision(3);
    file << "{\n";

    // Object info
    file << "  \"object\": {\n";
    file << "    \"x_radius_m\": " << mission.x_radius_m << ",\n";
    file << "    \"z_radius_m\": " << mission.z_radius_m << ",\n";
    file << "    \"height_m\": " << mission.config.object_height_m << "\n";
    file << "  },\n";

    // Passes
    file << "  \"passes\": [\n";

    for (size_t p = 0; p < mission.passes.size(); ++p) {
        const auto& pass = mission.passes[p];
        file << "    {\n";
        file << "      \"type\": \"" << pass_type_to_string(pass.type) << "\",\n";
        file << "      \"distance_to_surface_m\": " << pass.distance_to_surface_m << ",\n";
        file << "      \"orbits\": [\n";

        for (size_t o = 0; o < pass.orbits.size(); ++o) {
            const auto& orbit = pass.orbits[o];
            file << "        {\n";
            file << "          \"altitude_agl_m\": " << orbit.altitude_agl_m << ",\n";
            file << "          \"gimbal_pitch_deg\": " << orbit.gimbal_pitch_deg << ",\n";
            file << "          \"waypoints\": [\n";

            for (size_t w = 0; w < orbit.waypoints.size(); ++w) {
                const auto& wp = orbit.waypoints[w];

                // Convert GPS to local meters relative to center
                // X = East (positive), Y = North (positive), Z = Up
                double delta_lat = wp.coordinate.latitude_deg - center.latitude_deg;
                double delta_lon = wp.coordinate.longitude_deg - center.longitude_deg;

                double y_meters = delta_lat * METERS_PER_DEG_LAT;
                double x_meters = delta_lon * METERS_PER_DEG_LAT * std::cos(center_lat_rad);
                double z_meters = wp.altitude_m - mission.config.ground_elevation_msl_m;

                file << "            {";
                file << "\"x\": " << x_meters << ", ";
                file << "\"y\": " << y_meters << ", ";
                file << "\"z\": " << z_meters << ", ";
                file << "\"heading\": " << std::setprecision(1) << wp.heading_deg;
                file << "}" << std::setprecision(3);

                if (w < orbit.waypoints.size() - 1) file << ",";
                file << "\n";
            }

            file << "          ]\n";
            file << "        }";
            if (o < pass.orbits.size() - 1) file << ",";
            file << "\n";
        }

        file << "      ]\n";
        file << "    }";
        if (p < mission.passes.size() - 1) file << ",";
        file << "\n";
    }

    file << "  ]\n";
    file << "}\n";

    file.close();
    std::cout << "[Export] Mission exported to: " << filename << std::endl;
    std::cout << "[Export] Total waypoints: " << mission.total_waypoints() << std::endl;

    return true;
}