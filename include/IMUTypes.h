#ifndef MARGDARSHAK_IMUTYPES_H
#define MARGDARSHAK_IMUTYPES_H

#include <Eigen/Core>
#include <vector>

namespace margdarshak
{
class IMUMeasurement
{
public:
    IMUMeasurement(const Eigen::Vector3d& accData, const Eigen::Vector3d& gyrData, double integrationTime);

    const Eigen::Vector3d& getAccData() const;

    const Eigen::Vector3d& getGyrData() const;

    double getIntegrationTime() const;

private:
    Eigen::Vector3d accData{};
    Eigen::Vector3d gyrData{};
    double integrationTime; // time between this and previous IMU measurement.
};

typedef std::vector<IMUMeasurement> IMUData;
}

#endif //MARGDARSHAK_IMUTYPES_H
