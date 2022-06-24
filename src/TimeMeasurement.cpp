
#include "TimeMeasurement.h"
#include <iostream>

using namespace margdarshak;
using namespace std::chrono;


std::map<std::string, margdarshak::MeasurementLog> margdarshak::TimeMeasurement::logs = std::map<std::string, MeasurementLog>();
bool margdarshak::TimeMeasurement::saveFileOpen = false;

margdarshak::TimeMeasurement::TimeMeasurement(std::string name)
        : name(name)
{
    begin = high_resolution_clock::now();
}

margdarshak::TimeMeasurement::~TimeMeasurement()
{
    end();
}

double margdarshak::TimeMeasurement::end()
{
    if(ended)
    {
        return -1;
    }

    auto end = high_resolution_clock::now();
    double duration = duration_cast<std::chrono::duration<double>>(end - begin).count();

    logs[name].addMeasurement(duration);

    ended = true;

    return duration;
}

void margdarshak::TimeMeasurement::saveResults(std::string filename)
{
    std::ofstream saveFile;
    saveFile.open(filename);

    for(const auto& pair : logs)
    {
        saveFile << pair.first << ' ' << pair.second << '\n';
    }
    saveFile.close();
}

void margdarshak::TimeMeasurement::cancel()
{
    ended = true;
}

void margdarshak::MeasurementLog::addMeasurement(double time)
{
    if(num == 0)
    {
        first = time;
    }
    sum += time;
    num++;

    double shifted = time - first;
    sumShifted += shifted;
    sumSquared += shifted * shifted;

    if(time > max)
    {
        max = time;
    }
}

void margdarshak::MeasurementLog::writeLogLine(std::ostream& stream) const
{
    double mean = getMean();
    double variance = getVariance();

    stream << mean << ' ' << variance << ' ' << max << ' ' << num;

}

double margdarshak::MeasurementLog::getVariance() const
{
    double variance = (sumSquared - (sumShifted * sumShifted) / num) / (num - 1);
    return variance;
}

double margdarshak::MeasurementLog::getMean() const
{
    double mean = sum / (double) num;
    return mean;
}

double margdarshak::MeasurementLog::getMax() const
{
    return max;
}

int margdarshak::MeasurementLog::getNum() const
{
    return num;
}

std::ostream& operator<<(std::ostream& os, const margdarshak::MeasurementLog& obj)
{
    obj.writeLogLine(os);
    return os;
}
