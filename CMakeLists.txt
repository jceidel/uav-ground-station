cmake_minimum_required(VERSION 3.20)
project(uav_ground_station
    VERSION 0.1.0
    LANGUAGES CXX
)

#---C++17 (MAVSDK requires)---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#---Compiler Warnings---
add_compile_options(-Wall -Wextra -Wpedantic)

#---Find MAVSDK---
find_package(MAVSDK REQUIRED)

#---Connection Test---
add_executable(connection_test
    src/connection_test.cpp
)
target_link_libraries(connection_test
    MAVSDK::mavsdk
)

#---Takeoff and land---
add_executable(takeoff_land
    src/takeoff_land.cpp
)
target_link_libraries(takeoff_land
    MAVSDK::mavsdk
)