// Writes TelemetryFrame data to a timestamped CSV file.
// Thread-safe: can be called from any thread.

#pragma oncoe

#include "telemetry_monitor.h"
#include <fstream>
#include <string>
#include <mutex>

class CSVLogger {
public:
    // ---Constructor---
    // Opens or creates a CSV file and writed the header row.
    // IF the file cannot be opened is_open() returns false.
    //
    explicit CSVLogger(const std::string& filename = "");

    // ---Destructor---
    // FLushes and closes the file. RAII - the resource
    // (open file) is released when the object is destryed.
    //
    ~CSVLogger();

    // ---Log one frame--
    // Writes a single row to the CSV file.
    void log(const TelemetryFrame& frame);

    // Check if the file is open
    bool is_open() const;

    // Get the filename
    std::string filename() const { return _filename; }

private:
    std::ofstream _file;
    std::string  _filename;
    mutable std::mutex _mutex;

    // Generate a filename
    static std::string generate_filename();

    // Write CSV header row
    void write_header();
};