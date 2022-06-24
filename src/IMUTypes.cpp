#include "IMUTypes.h"

using namespace margdarshak;

IMUMeasurement::IMUMeasurement(const Eigen::Vector3d& accData, const Eigen::Vector3d& gyrData, double integrationTime)
        : accData(accData), gyrData(gyrData), integrationTime(integrationTime)
{}

const Eigen::Vector3d& IMUMeasurement::getAccData() const
{
    return accData;
}

const Eigen::Vector3d& IMUMeasurement::getGyrData() const
{
    return gyrData;
}

double IMUMeasurement::getIntegrationTime() const
{
    return integrationTime;
}
