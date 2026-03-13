// Implementation of the CSVLogger class.
//

#include "csv_logger.h"
#include <iomanip>
#include <iostream>
#include <chrono>
#include <sstream>
#include <ctime>

// ---generate_filename()---
// Creates a filename using the current time
//
std::string CSVLogger::generate_filename()
{
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    auto* tm = std::localtime(&time_t_now);

    std::ostringstream oss;
    oss << "flight_"
        << std::put_time(tm, "%Y%m%d_%H%M%S")
        << ".csv";
    return oss.str();
}

// ----Constructor---
//
CSVLogger::CSVLogger(const std::string& filename)
    : _filename(filename.empty() ? generate_filename() : filename)
{
    _file.open(_filename, std::ios::out | std::ios::trunc);

    if (_file.is_open()) {
        write_header();
        std::cout << "[CSVLogger] Logging to: " << _filename << std::endl;
    } else {
        std::cerr << "[CSVLogger] ERROR: Could not open " << _filename << std::endl;
    }
}

// ----Destructor----
//
CSVLogger::~CSVLogger()
{
    if (_file.is_open()) {
        _file.flush();
        _file.close();
        std::cout << "[CSVLogger] File closed: " << _filename << std::endl;
    }
}

//----write_header()----
// CSV header defines the columns. This must match the order of 
// values written in log().
//
void CSVLogger::write_header()
{
    _file << "timestamp_s,"
          << "latitude_deg,"
          << "longitude_deg,"
          << "alt_msl_m,"
          << "alt_agl_m,"
          << "roll_deg,"
          << "pitch_deg,"
          << "yaw_deg,"
          << "battery_v,"
          << "battery_pct,"
          << "gps_sats,"
          << "gps_fix,"
          << "flight_mode,"
          << "armed,"
          << "in_air"
          << "\n";
}

// ----log()----
// Writes one row of telemetry data to the CSV filke
// Fof GPS coord, we need fixed-point with 7 decimals for aprox 1cm accuracy for lat/lon
void CSVLogger::log(const TelemetryFrame& frame)
{
    std::lock_guard<std::mutex> lock(_mutex);

    if (!_file.is_open()) return;

    _file << std::fixed
          << std::setprecision(3) << frame.timestamp_s << ","
          << std::setprecision(7) << frame.latitude_deg << ','
          << std::setprecision(7) << frame.longitude_deg << ','
          << std::setprecision(2) << frame.absolute_altitude_m << ','
          << std::setprecision(2) << frame.relative_altitude_m << ','
          << std::setprecision(1) << frame.roll_deg << ','
          << std::setprecision(1) << frame.pitch_deg << ','
          << std::setprecision(1) << frame.yaw_deg << ','
          << std::setprecision(2) << frame.battery_voltage_v << ','
          << std::setprecision(2) << frame.battery_remaining << ','
          << frame.gps_num_satellites << ","
          << frame.gps_fix_type << ","
          << frame.flight_mode << ","
          << (frame.armed ? 1 : 0) << ","
          << (frame.in_air ? 1 : 0) << ","
          << "/n";

    // Flush periodically to avoid data loss if the program crashes.
    _file.flush();

}

// ----is_open----
bool CSVLogger::is_open() const
{
    return _file.is_open();
}
