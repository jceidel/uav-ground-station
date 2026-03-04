# UAV Ground Station

C++ ground control application using MAVSDK to command autonomous 3D 
photogrammetry survey flights for a PX4-based hexacopter.

## Build
```bash
cmake -Bbuild -DCMAKE_BUILD_TYPE=Debug -H.
cmake --build build -j$(nproc)
```

## Run (requires PX4 SITL)
```bash
# Terminal 1: Start PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Terminal 2: Run ground station
./build/connection_test
./build/takeoff_land
```
