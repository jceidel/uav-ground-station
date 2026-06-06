cmake_minimum_required(VERSION 3.20)
project(uav_ground_station
    VERSION 0.2.0
    LANGUAGES CXX
)

# ── C++17 (MAVSDK requires it) ──────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── Compiler warnings ───────────────────────────────────────
add_compile_options(-Wall -Wextra -Wpedantic)

# ── Find MAVSDK ─────────────────────────────────────────────
find_package(MAVSDK REQUIRED)

# ── Include directory for our headers ────────────────────────
include_directories(${CMAKE_SOURCE_DIR}/include)

# ── Milestone 1: Connection test ─────────────────────────────
add_executable(connection_test
    src/connection_test.cpp
)
target_link_libraries(connection_test
    MAVSDK::mavsdk
)

# ── Milestone 2: Takeoff & Land ──────────────────────────────
add_executable(takeoff_land
    src/takeoff_land.cpp
)
target_link_libraries(takeoff_land
    MAVSDK::mavsdk
)

# ── Milestone 3: Telemetry Dashboard ─────────────────────────
add_executable(telemetry_dashboard
    src/telemetry_dashboard.cpp
    src/telemetry_monitor.cpp
    src/csv_logger.cpp
)
target_link_libraries(telemetry_dashboard
    MAVSDK::mavsdk
)

# ── Milestone 4: Ground Station (merged) ─────────────────────
add_executable(ground_station
    src/ground_station.cpp
    src/telemetry_monitor.cpp
    src/csv_logger.cpp
    src/orbital_planner.cpp
)
target_link_libraries(ground_station
    MAVSDK::mavsdk
)

# ── Mission Export (no MAVSDK needed) ────────────────────────
add_executable(export_mission
    src/export_mission.cpp
    src/orbital_planner.cpp
    src/mission_export.cpp
)