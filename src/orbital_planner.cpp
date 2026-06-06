//
// orbital_planner.cpp
//
// Implementation of the orbital photogrammetry mission planner.
//

#include "orbital_planner.h"
#include <iostream>
#include <iomanip>
#include <algorithm>

// ── Constants ───────────────────────────────────────────────
static constexpr double PI = 3.14159265358979323846;
static constexpr double DEG_TO_RAD = PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / PI;
static constexpr double METERS_PER_DEG_LAT = 111320.0;

// ══════════════════════════════════════════════════════════════
// Utility / geometry functions
// ══════════════════════════════════════════════════════════════

std::string pass_type_to_string(PassType type)
{
    switch (type) {
        case PassType::OVERVIEW:           return "Overview";
        case PassType::DETAIL_HORIZONTAL:  return "Detail Horizontal";
        case PassType::DETAIL_PITCH_UP:    return "Detail Pitch Up (+45)";
        case PassType::DETAIL_PITCH_DOWN:  return "Detail Pitch Down (-45)";
        default:                           return "Unknown";
    }
}

int OrbitalMission::total_waypoints() const
{
    int count = 0;
    for (const auto& pass : passes) {
        for (const auto& orbit : pass.orbits) {
            count += static_cast<int>(orbit.waypoints.size());
        }
    }
    return count;
}

int OrbitalMission::total_orbits() const
{
    int count = 0;
    for (const auto& pass : passes) {
        count += static_cast<int>(pass.orbits.size());
    }
    return count;
}

// ── GSD -> distance to surface ──────────────────────────────
//
// From Joe's pseudocode: calculateDistancetoSurface
//
// GSD = (distance * sensor_width) / (focal_length * image_width_pixels)
// Solving for distance:
// distance = (GSD * focal_length * image_width_pixels) / sensor_width
//
// Note: GSD input is in mm, sensor_width and focal_length in mm,
// so distance comes out in mm — we convert to meters.
//
float calculate_distance_to_surface(float gsd_mm, const CameraParams& cam)
{
    float distance_mm = (gsd_mm * cam.focal_length_mm * cam.image_width_pixels)
                        / cam.sensor_width_mm;
    return distance_mm / 1000.0f;  // Convert mm to meters
}

// ── GPS coords -> meters ────────────────────────────────────
//
// From Joe's pseudocode: calculateEllipseRadius
//
// Converts the GPS coordinates of the object's extent points
// into meter distances from center, giving us the semi-major
// and semi-minor axes of the elliptical footprint.
//
void calculate_ellipse_radii(const GPSCoord& center,
                             const GPSCoord& x_coord,
                             const GPSCoord& z_coord,
                             float& x_radius_m,
                             float& z_radius_m)
{
    // X-axis radius
    double x_lat_rad = x_coord.latitude_deg * DEG_TO_RAD;
    double x_dx = (x_coord.longitude_deg - center.longitude_deg)
                  * METERS_PER_DEG_LAT * std::cos(x_lat_rad);
    double x_dy = (x_coord.latitude_deg - center.latitude_deg)
                  * METERS_PER_DEG_LAT;
    x_radius_m = static_cast<float>(std::sqrt(x_dx * x_dx + x_dy * x_dy));

    // Z-axis radius
    double z_lat_rad = z_coord.latitude_deg * DEG_TO_RAD;
    double z_dx = (z_coord.longitude_deg - center.longitude_deg)
                  * METERS_PER_DEG_LAT * std::cos(z_lat_rad);
    double z_dy = (z_coord.latitude_deg - center.latitude_deg)
                  * METERS_PER_DEG_LAT;
    z_radius_m = static_cast<float>(std::sqrt(z_dx * z_dx + z_dy * z_dy));
}

// ── Ramanujan's ellipse circumference approximation ─────────
//
// From Joe's pseudocode: calculateOrbitCircumference
//
// C ≈ π * (3(a+b) - sqrt((3a+b)(a+3b)))
//
// This is accurate to within 0.04% for most eccentricities.
//
float ellipse_circumference(float a, float b)
{
    float sum = a + b;
    return static_cast<float>(PI) * (3.0f * sum
           - std::sqrt((3.0f * a + b) * (a + 3.0f * b)));
}

// ── Ellipse radius at angle theta (polar form) ─────────────
//
// From Joe's pseudocode: calculateWaypointRadius
//
// r(θ) = (a * b) / sqrt((b*cos(θ))² + (a*sin(θ))²)
//
// This gives the distance from center to the ellipse edge
// at any angle, which we need for placing waypoints on
// non-circular orbits.
//
float ellipse_radius_at_angle(float a, float b, float theta_rad)
{
    float cos_t = std::cos(theta_rad);
    float sin_t = std::sin(theta_rad);
    return (a * b) / std::sqrt((b * cos_t) * (b * cos_t)
                              + (a * sin_t) * (a * sin_t));
}

// ── Heading from one GPS point toward another ───────────────
//
// From Joe's pseudocode: calculateHeading
//
// Returns bearing in degrees (0=North, 90=East, 180=South, 270=West).
//
// Joe's original had the subtraction order reversed — fixed here
// so the heading points FROM 'from' TOWARD 'to'.
//
float calculate_heading(const GPSCoord& from, const GPSCoord& to)
{
    double from_lat_rad = from.latitude_deg * DEG_TO_RAD;

    double dx = (to.longitude_deg - from.longitude_deg) * std::cos(from_lat_rad);
    double dy = to.latitude_deg - from.latitude_deg;

    double bearing_rad = std::atan2(dx, dy);
    double bearing_deg = bearing_rad * RAD_TO_DEG;

    // Normalize to 0-360
    return static_cast<float>(std::fmod(bearing_deg + 360.0, 360.0));
}

// ══════════════════════════════════════════════════════════════
// Waypoint generation
// ══════════════════════════════════════════════════════════════

// ── Waypoints per orbit ─────────────────────────────────────
//
// From Joe's pseudocode: totalWaypoints
//
// Calculates how many evenly-spaced photos are needed around
// one orbit to achieve the target overlap percentage.
//
// image_width_on_surface = (sensor_width * distance) / focal_length
// step_distance = image_width * (1 - overlap)
// num_waypoints = ceil(orbit_circumference / step_distance)
//
int calculate_waypoints_per_orbit(float x_radius_m, float z_radius_m,
                                  float distance_to_surface_m,
                                  const CameraParams& cam)
{
    // Orbit radii = rock radii + distance to surface
    float orbit_a = x_radius_m + distance_to_surface_m;
    float orbit_b = z_radius_m + distance_to_surface_m;

    float circumference = ellipse_circumference(orbit_a, orbit_b);

    // Image width on the rock surface (using distance_to_surface, not orbit radius)
    float image_width_m = (cam.sensor_width_mm * distance_to_surface_m)
                          / cam.focal_length_mm
                          / 1000.0f;  // sensor_width is mm, distance is m

    // Wait — sensor_width is mm and distance is meters, so:
    // image_width = (sensor_width_mm / focal_length_mm) * distance_m
    // The mm units cancel in the ratio, leaving meters.
    image_width_m = (cam.sensor_width_mm / cam.focal_length_mm)
                    * distance_to_surface_m;

    float step = image_width_m * (1.0f - cam.overlap);

    int num_waypoints = static_cast<int>(std::ceil(circumference / step));

    // Minimum 8 waypoints for any orbit (ensures reasonable angular coverage)
    return std::max(num_waypoints, 8);
}

// ── Generate waypoints for one orbit ────────────────────────
//
// From Joe's pseudocode: generate_orbit_waypoints
//
// Places N waypoints on an elliptical orbit, computing GPS
// coordinates and headings (pointing toward object center).
//
std::vector<Waypoint> generate_orbit_waypoints(const GPSCoord& center,
                                                float x_orbit_radius_m,
                                                float z_orbit_radius_m,
                                                float altitude_msl_m,
                                                float gimbal_pitch_deg,
                                                int num_waypoints)
{
    std::vector<Waypoint> waypoints;
    waypoints.reserve(num_waypoints);

    float delta_theta = 2.0f * static_cast<float>(PI) / num_waypoints;

    for (int i = 0; i < num_waypoints; ++i) {
        float theta = i * delta_theta;

        // Radius at this angle on the elliptical orbit
        float radius = ellipse_radius_at_angle(x_orbit_radius_m,
                                                z_orbit_radius_m,
                                                theta);

        // Offset in meters from center
        // Note: x = East/West (longitude), z = North/South (latitude)
        float delta_east  = radius * std::sin(theta);   // East-West offset
        float delta_north = radius * std::cos(theta);   // North-South offset

        // Convert meters to GPS degrees
        double center_lat_rad = center.latitude_deg * DEG_TO_RAD;
        double delta_lat = delta_north / METERS_PER_DEG_LAT;
        double delta_lon = delta_east / (METERS_PER_DEG_LAT
                           * std::cos(center_lat_rad));

        Waypoint wp;
        wp.coordinate.latitude_deg  = center.latitude_deg + delta_lat;
        wp.coordinate.longitude_deg = center.longitude_deg + delta_lon;
        wp.altitude_m = altitude_msl_m;
        wp.gimbal_pitch = gimbal_pitch_deg;
        wp.trigger_camera = true;

        // Heading: point the drone TOWARD the center
        wp.heading_deg = calculate_heading(wp.coordinate, center);

        waypoints.push_back(wp);
    }

    return waypoints;
}

// ══════════════════════════════════════════════════════════════
// Orbit / pass calculations
// ══════════════════════════════════════════════════════════════

// ── Number of orbits for a pass ─────────────────────────────
//
// Joe's pseudocode: totalOrbits (was empty — filled in here)
//
// Uses vertical FOV to determine altitude spacing between orbits,
// then divides the object height by the step to get the count.
//
// vertical_fov_on_surface = (sensor_height / focal_length) * distance
// altitude_step = vertical_fov * (1 - overlap)
// num_orbits = ceil(object_height / altitude_step) + 1
//
int calculate_orbits_for_pass(float object_height_m,
                              float distance_to_surface_m,
                              const CameraParams& cam)
{
    float vertical_fov_m = (cam.sensor_height_mm / cam.focal_length_mm)
                           * distance_to_surface_m;

    float altitude_step = vertical_fov_m * (1.0f - cam.overlap);

    int num_orbits = static_cast<int>(std::ceil(object_height_m / altitude_step)) + 1;

    return std::max(num_orbits, 2);  // At least 2 orbits
}

// ── Overview pitch per orbit ────────────────────────────────
//
// Joe's pseudocode: overviewPitch (was empty — filled in here)
//
// Instead of always aiming at rock center, each orbit aims at
// a different vertical slice:
//   - Lowest orbit  → aims at bottom 1/3
//   - Mid orbit     → aims at middle
//   - Highest orbit → aims at top 1/3
//
// target_height = rock_height * (orbit_index / (total_orbits - 1))
// pitch = atan2(target_height - orbit_altitude, radius)
//
float calculate_overview_pitch(int orbit_index, int total_orbits,
                               float orbit_altitude_agl,
                               float object_height_m,
                               float overview_radius_m)
{
    // Distribute aim point from base to top across orbits
    float t = (total_orbits > 1)
              ? static_cast<float>(orbit_index) / (total_orbits - 1)
              : 0.5f;

    float target_height = object_height_m * t;

    // Pitch angle: positive = up, negative = down
    float pitch_rad = std::atan2(target_height - orbit_altitude_agl,
                                 overview_radius_m);
    return static_cast<float>(pitch_rad * RAD_TO_DEG);
}

// ══════════════════════════════════════════════════════════════
// Mission planning — the main algorithm
// ══════════════════════════════════════════════════════════════

OrbitalMission plan_mission(const MissionConfig& config)
{
    OrbitalMission mission;
    mission.config = config;

    // ── Step 1: Compute object dimensions in meters ─────────
    calculate_ellipse_radii(config.center,
                            config.x_extent_coord,
                            config.z_extent_coord,
                            mission.x_radius_m,
                            mission.z_radius_m);

    // ── Step 2: Compute distances to surface from GSD ───────
    float detail_dist = calculate_distance_to_surface(
                            config.detail_gsd_mm, config.camera);
    float overview_dist = calculate_distance_to_surface(
                            config.overview_gsd_mm, config.camera);

    // ── Step 3: Compute orbit counts ────────────────────────
    int detail_orbits = calculate_orbits_for_pass(
                            config.object_height_m,
                            detail_dist, config.camera);
    int overview_orbits = calculate_orbits_for_pass(
                            config.object_height_m,
                            overview_dist, config.camera);

    // ── Step 4: Compute waypoints per orbit ─────────────────
    int detail_wps = calculate_waypoints_per_orbit(
                         mission.x_radius_m, mission.z_radius_m,
                         detail_dist, config.camera);
    int overview_wps = calculate_waypoints_per_orbit(
                           mission.x_radius_m, mission.z_radius_m,
                           overview_dist, config.camera);

    // Orbit radii (object radius + distance to surface)
    float detail_orbit_a = mission.x_radius_m + detail_dist;
    float detail_orbit_b = mission.z_radius_m + detail_dist;
    float overview_orbit_a = mission.x_radius_m + overview_dist;
    float overview_orbit_b = mission.z_radius_m + overview_dist;

    // Altitude range for orbits
    float min_alt = config.min_altitude_agl_m;
    float max_alt = config.object_height_m;
    float ground_msl = config.ground_elevation_msl_m;

    // ═══════════════════════════════════════════════════════════
    // Pass 1: OVERVIEW — dynamic pitch, full altitude stack
    // ═══════════════════════════════════════════════════════════
    {
        Pass pass;
        pass.type = PassType::OVERVIEW;
        pass.distance_to_surface_m = overview_dist;

        float alt_step = (overview_orbits > 1)
                         ? (max_alt - min_alt) / (overview_orbits - 1)
                         : 0.0f;

        for (int i = 0; i < overview_orbits; ++i) {
            Orbit orbit;
            orbit.altitude_agl_m = min_alt + i * alt_step;

            // Dynamic pitch: each orbit aims at a different vertical slice
            orbit.gimbal_pitch_deg = calculate_overview_pitch(
                i, overview_orbits, orbit.altitude_agl_m,
                config.object_height_m, overview_dist);

            float alt_msl = ground_msl + orbit.altitude_agl_m;

            orbit.waypoints = generate_orbit_waypoints(
                config.center, overview_orbit_a, overview_orbit_b,
                alt_msl, orbit.gimbal_pitch_deg, overview_wps);

            pass.orbits.push_back(orbit);
        }
        mission.passes.push_back(pass);
    }

    // ═══════════════════════════════════════════════════════════
    // Pass 2: DETAIL HORIZONTAL — pitch 0°, full altitude stack
    // ═══════════════════════════════════════════════════════════
    {
        Pass pass;
        pass.type = PassType::DETAIL_HORIZONTAL;
        pass.distance_to_surface_m = detail_dist;
        pass.base_pitch_deg = 0.0f;

        float alt_step = (detail_orbits > 1)
                         ? (max_alt - min_alt) / (detail_orbits - 1)
                         : 0.0f;

        for (int i = 0; i < detail_orbits; ++i) {
            Orbit orbit;
            orbit.altitude_agl_m = min_alt + i * alt_step;
            orbit.gimbal_pitch_deg = 0.0f;

            float alt_msl = ground_msl + orbit.altitude_agl_m;

            orbit.waypoints = generate_orbit_waypoints(
                config.center, detail_orbit_a, detail_orbit_b,
                alt_msl, orbit.gimbal_pitch_deg, detail_wps);

            pass.orbits.push_back(orbit);
        }
        mission.passes.push_back(pass);
    }

    // ═══════════════════════════════════════════════════════════
    // Pass 3: DETAIL PITCH UP (+45°) — low orbits only
    // ═══════════════════════════════════════════════════════════
    //
    // Cutoff: stop when the camera at +45° would see above the
    // rock top. That happens when:
    //   orbit_alt + detail_dist * tan(45°) > object_height
    //   orbit_alt + detail_dist > object_height
    //   orbit_alt > object_height - detail_dist
    //
    {
        Pass pass;
        pass.type = PassType::DETAIL_PITCH_UP;
        pass.distance_to_surface_m = detail_dist;
        pass.base_pitch_deg = 45.0f;

        float cutoff_alt = config.object_height_m - detail_dist;
        if (cutoff_alt < min_alt) cutoff_alt = min_alt;

        float alt_step = (detail_orbits > 1)
                         ? (max_alt - min_alt) / (detail_orbits - 1)
                         : 0.0f;

        for (int i = 0; i < detail_orbits; ++i) {
            float alt_agl = min_alt + i * alt_step;

            if (alt_agl > cutoff_alt) break;  // Above cutoff, stop

            Orbit orbit;
            orbit.altitude_agl_m = alt_agl;
            orbit.gimbal_pitch_deg = 45.0f;

            float alt_msl = ground_msl + orbit.altitude_agl_m;

            orbit.waypoints = generate_orbit_waypoints(
                config.center, detail_orbit_a, detail_orbit_b,
                alt_msl, orbit.gimbal_pitch_deg, detail_wps);

            pass.orbits.push_back(orbit);
        }
        mission.passes.push_back(pass);
    }

    // ═══════════════════════════════════════════════════════════
    // Pass 4: DETAIL PITCH DOWN (-45°) — high orbits + extra above
    // ═══════════════════════════════════════════════════════════
    //
    // Cutoff: stop going DOWN when the camera at -45° would see
    // below ground:
    //   orbit_alt - detail_dist * tan(45°) < 0
    //   orbit_alt < detail_dist
    //
    // Extra orbits ABOVE the rock top to capture the crown.
    //
    {
        Pass pass;
        pass.type = PassType::DETAIL_PITCH_DOWN;
        pass.distance_to_surface_m = detail_dist;
        pass.base_pitch_deg = -45.0f;

        float cutoff_alt = detail_dist;  // Below this, camera sees underground
        if (cutoff_alt < min_alt) cutoff_alt = min_alt;

        float alt_step = (detail_orbits > 1)
                         ? (max_alt - min_alt) / (detail_orbits - 1)
                         : 0.0f;

        // Regular orbits above the cutoff
        for (int i = 0; i < detail_orbits; ++i) {
            float alt_agl = min_alt + i * alt_step;

            if (alt_agl < cutoff_alt) continue;  // Below cutoff, skip

            Orbit orbit;
            orbit.altitude_agl_m = alt_agl;
            orbit.gimbal_pitch_deg = -45.0f;

            float alt_msl = ground_msl + orbit.altitude_agl_m;

            orbit.waypoints = generate_orbit_waypoints(
                config.center, detail_orbit_a, detail_orbit_b,
                alt_msl, orbit.gimbal_pitch_deg, detail_wps);

            pass.orbits.push_back(orbit);
        }

        // Extra orbits above rock top for crown capture
        for (int j = 1; j <= config.extra_top_orbits; ++j) {
            float alt_agl = max_alt + j * alt_step;

            Orbit orbit;
            orbit.altitude_agl_m = alt_agl;
            orbit.gimbal_pitch_deg = -45.0f;

            float alt_msl = ground_msl + orbit.altitude_agl_m;

            orbit.waypoints = generate_orbit_waypoints(
                config.center, detail_orbit_a, detail_orbit_b,
                alt_msl, orbit.gimbal_pitch_deg, detail_wps);

            pass.orbits.push_back(orbit);
        }
        mission.passes.push_back(pass);
    }

    return mission;
}

// ══════════════════════════════════════════════════════════════
// Debug output
// ══════════════════════════════════════════════════════════════

void print_mission_summary(const OrbitalMission& mission)
{
    std::cout << "\n";
    std::cout << "╔══════════════════════════════════════════════════╗\n";
    std::cout << "║        ORBITAL PHOTOGRAMMETRY MISSION PLAN      ║\n";
    std::cout << "╠══════════════════════════════════════════════════╣\n";

    std::cout << std::fixed;
    std::cout << "║  Object center:  " << std::setprecision(7)
              << mission.config.center.latitude_deg << ", "
              << mission.config.center.longitude_deg << "\n";
    std::cout << "║  Object size:    " << std::setprecision(1)
              << mission.x_radius_m * 2.0f << "m x "
              << mission.z_radius_m * 2.0f << "m x "
              << mission.config.object_height_m << "m (W x D x H)\n";
    std::cout << "║  Detail GSD:     " << mission.config.detail_gsd_mm << " mm/pixel\n";
    std::cout << "║  Overview GSD:   " << mission.config.overview_gsd_mm << " mm/pixel\n";
    std::cout << "║  Total orbits:   " << mission.total_orbits() << "\n";
    std::cout << "║  Total waypoints:" << mission.total_waypoints() << "\n";
    std::cout << "╠══════════════════════════════════════════════════╣\n";

    for (const auto& pass : mission.passes) {
        std::cout << "║\n";
        std::cout << "║  ── " << pass_type_to_string(pass.type) << " ──\n";
        std::cout << "║  Distance to surface: " << std::setprecision(1)
                  << pass.distance_to_surface_m << " m\n";
        std::cout << "║  Orbits: " << pass.orbits.size() << "\n";

        for (size_t i = 0; i < pass.orbits.size(); ++i) {
            const auto& orbit = pass.orbits[i];
            std::cout << "║    Orbit " << (i + 1) << ": "
                      << std::setprecision(1) << orbit.altitude_agl_m << "m AGL, "
                      << "pitch " << std::setprecision(0) << orbit.gimbal_pitch_deg << "°, "
                      << orbit.waypoints.size() << " waypoints\n";
        }
    }

    std::cout << "╚══════════════════════════════════════════════════╝\n";
    std::cout << std::endl;
}