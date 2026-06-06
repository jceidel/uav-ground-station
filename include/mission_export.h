//
// mission_export.h
//
// Exports an OrbitalMission to a JSON file with local XYZ coordinates
// (meters relative to object center) for visualization in Unreal Engine.
//
// UE5 coordinate system: X = forward, Y = right, Z = up
// Our coordinate system: North = +Y, East = +X, Up = +Z
// So we map: East->X, North->Y, Altitude->Z (UE5 uses cm, we convert)
//

#pragma once

#include "orbital_planner.h"
#include <string>

// Export the full mission as JSON with local coordinates (meters)
bool export_mission_json(const OrbitalMission& mission,
                         const std::string& filename = "mission_waypoints.json");