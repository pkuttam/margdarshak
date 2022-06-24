
#ifndef MARGDARSHAK_TIMEMEASUREMENT_H
#define MARGDARSHAK_TIMEMEASUREMENT_H

#include <string>
#include <fstream>
#include <chrono>
#include <map>
#include <memory>


namespace margdarshak
{
// Saves mean, maximum, and variance.
class MeasurementLog
{
public:
    MeasurementLog() = default;

    void addMeasurement(double time);

    void writeLogLine(std::ostream& stream) const;

    int getNum() const;
    double getMax() const;
    double getMean() const;
    double getVariance() const;
private:
    double sum{0};
    double max{0};
    int num{0};

    double first{-1};
    // For computing stddev we shift by the first sample.
    double sumShifted{0};
    double sumSquared{0};

};

// Used to measure and log wall time for different code parts.
// Note that this class is only partially thread-safe, meaning there should not be measurements with the same name
// in different threads, otherwise there can be an endless loop / segfault in the first call!
class TimeMeasurement final
{
public:
    TimeMeasurement(std::string name);
    TimeMeasurement(const TimeMeasurement&) = delete;
    ~TimeMeasurement();

    // End the measurement interval. Optional, if not called the destructor will call it.
    double end();

    // Cancel the time measurement.
    void cancel();

    static void saveResults(std::string filename);

private:
    static bool saveFileOpen;
    static std::ofstream saveFile;
    static std::map<std::string, MeasurementLog> logs;

    std::string name;
    std::chrono::high_resolution_clock::time_point begin;
    bool ended{false};
};
}

std::ostream& operator<<(std::ostream& os, const margdarshak::MeasurementLog& obj);


#endif //MARGDARSHAK_TIMEMEASUREMENT_H
