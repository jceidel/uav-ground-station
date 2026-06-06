//
// orbital_planner.h
//
// Generates orbital photogrammetry waypoints for capturing large
// 3D objects (rock formations, structures) from multiple altitudes
// and gimbal angles.
//
// Algorithm designed by Joe Eidel, March 2026.
//
// The orbit geometry is derived from target GSD (ground sampling
// distance), which determines distance-to-surface, which determines
// orbit radius, which determines waypoint count and spacing.
//
// Supports elliptical orbits for non-circular objects using the
// polar form of an ellipse and Ramanujan's circumference approximation.
//

#pragma once

#include <vector>
#include <string>
#include <cmath>

// ── GPS coordinate pair ─────────────────────────────────────
struct GPSCoord {
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
};

// ── Single waypoint: where to be and how to point ───────────
struct Waypoint {
    GPSCoord coordinate;
    float heading_deg   = 0.0f;   // Yaw: direction drone faces (0=N, 90=E, 180=S, 270=W)
    float altitude_m    = 0.0f;   // MSL altitude for this waypoint
    float gimbal_pitch  = 0.0f;   // Camera pitch: +up, -down, 0=horizontal
    bool  trigger_camera = true;
};

// ── One circular/elliptical orbit at a single altitude ──────
struct Orbit {
    float altitude_agl_m    = 0.0f;   // Height above ground level
    float gimbal_pitch_deg  = 0.0f;   // Camera pitch for this orbit
    std::vector<Waypoint> waypoints;
};

// ── A pass: collection of orbits with a capture strategy ────
//
// Instead of four separate structs (PassDetailUp, PassDetailDown,
// etc.), we use one struct with a pass_type field. This is cleaner
// in C++ — the behavior differences are handled in the planner
// logic, not in the type system.
//
enum class PassType {
    OVERVIEW,
    DETAIL_HORIZONTAL,
    DETAIL_PITCH_UP,
    DETAIL_PITCH_DOWN
};

std::string pass_type_to_string(PassType type);

struct Pass {
    PassType type;
    float distance_to_surface_m = 0.0f;
    float base_pitch_deg        = 0.0f;  // Fixed pitch for detail passes
    std::vector<Orbit> orbits;
};

// ── Camera parameters ───────────────────────────────────────
// These determine GSD, FOV, and therefore all the orbit geometry.
//
struct CameraParams {
    float focal_length_mm     = 28.0f;     // Sony FE 28mm f/2
    float sensor_width_mm     = 35.8f;     // Sony A7II full-frame
    float sensor_height_mm    = 23.9f;     // Sony A7II
    int   image_width_pixels  = 6000;      // A7II horizontal resolution
    int   image_height_pixels = 4000;      // A7II vertical resolution
    float overlap             = 0.75f;     // 75% overlap between adjacent photos
};

// ── Mission configuration ───────────────────────────────────
// Everything the planner needs to generate the full flight plan.
//
struct MissionConfig {
    // Object location and geometry
    GPSCoord center;                       // GPS center of the object
    GPSCoord x_extent_coord;               // GPS point on the object's wide axis edge
    GPSCoord z_extent_coord;               // GPS point on the object's narrow axis edge
    float    object_height_m = 10.0f;      // Approximate height of the object

    // Quality targets
    float detail_gsd_mm   = 1.5f;         // Target GSD for detail passes (mm/pixel)
    float overview_gsd_mm = 4.0f;         // Target GSD for overview pass (mm/pixel)

    // Camera
    CameraParams camera;

    // Safety
    float min_altitude_agl_m = 1.0f;      // Lowest orbit altitude
    float ground_elevation_msl_m = 0.0f;  // Ground elevation at object base (MSL)

    // Pitch-down extension
    int   extra_top_orbits = 2;            // Extra orbits above rock for crown capture
};

// ── The full mission: all passes combined ───────────────────
struct OrbitalMission {
    MissionConfig config;
    std::vector<Pass> passes;

    // Computed object dimensions in meters
    float x_radius_m = 0.0f;
    float z_radius_m = 0.0f;

    int total_waypoints() const;
    int total_orbits() const;
};

// ══════════════════════════════════════════════════════════════
// Planner functions
// ══════════════════════════════════════════════════════════════

// ── Top-level: generate the entire mission ──────────────────
OrbitalMission plan_mission(const MissionConfig& config);

// ── Distance / geometry calculations ────────────────────────

// GSD -> distance to surface
float calculate_distance_to_surface(float gsd_mm, const CameraParams& cam);

// GPS coords -> meters (object radii)
void calculate_ellipse_radii(const GPSCoord& center,
                             const GPSCoord& x_coord,
                             const GPSCoord& z_coord,
                             float& x_radius_m,
                             float& z_radius_m);

// Ramanujan's approximation for ellipse circumference
float ellipse_circumference(float a, float b);

// Radius at angle theta for an ellipse (polar form)
float ellipse_radius_at_angle(float x_radius, float z_radius, float theta_rad);

// ── Waypoint calculations ───────────────────────────────────

// How many waypoints needed for one orbit given overlap requirements
int calculate_waypoints_per_orbit(float x_radius_m, float z_radius_m,
                                  float distance_to_surface_m,
                                  const CameraParams& cam);

// Generate waypoints around one orbit
std::vector<Waypoint> generate_orbit_waypoints(const GPSCoord& center,
                                                float x_orbit_radius_m,
                                                float z_orbit_radius_m,
                                                float altitude_msl_m,
                                                float gimbal_pitch_deg,
                                                int num_waypoints);

// ── Orbit / pass calculations ───────────────────────────────

// How many orbits for a pass (based on vertical FOV coverage)
int calculate_orbits_for_pass(float object_height_m,
                              float distance_to_surface_m,
                              const CameraParams& cam);

// Compute overview gimbal pitch for a specific orbit
// (aims at different vertical slices of the rock)
float calculate_overview_pitch(int orbit_index, int total_orbits,
                               float orbit_altitude_agl,
                               float object_height_m,
                               float overview_radius_m);

// Heading from one GPS point toward another (degrees, 0=N)
float calculate_heading(const GPSCoord& from, const GPSCoord& to);

// ── Debug / display ─────────────────────────────────────────
void print_mission_summary(const OrbitalMission& mission);