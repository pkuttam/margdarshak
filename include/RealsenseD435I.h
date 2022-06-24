
#ifndef MARGDARSHAK_REALSENSED435I_H
#define MARGDARSHAK_REALSENSED435I_H

#include <librealsense2/rs.hpp>
//#include "IMU/IMUSettings.h"
#include "IMUInterpolator.h"
#include "FrameContainer.h"
//#include "util/Undistort.h"
#include "DatasetSaver.h"

namespace margdarshak
{
// Class for interacting with the RealsenseT265 camera.
    class Realsense435I
    {
    public:
        // Images and IMU data will be passed into frameContainer which can be used to get synchronized image and IMU data.
        // Factory calibration will be saved to cameraCalibSavePath.
        // If datasetSaver is set, the IMU data and images will also be saved to file.
        Realsense435I(FrameContainer& frameContainer, std::string cameraCalibSavePath, DatasetSaver* datasetSaver);

        // Start receiving data.
        void start();

        // Set the undistorter to use. Until this is set, no images are passed forward to the frameContainer.
        //void setUndistorter(margdarshak::Undistort* undistort);

        //std::unique_ptr<IMUCalibration> imuCalibration;
    private:
        void readCalibration();
        std::string cameraCalibSavePath;

        rs2::context context;
        rs2::config config;
        rs2::pipeline pipe;

        rs2::pipeline_profile profile;

        // Use left camera.
        int useCam = 0;

        rs2_motion_device_intrinsic gyroIntrinsics, accelIntrinsics;

        std::atomic<bool> calibrationRead{false};

        // IMU interpolator will take care of creating "fake measurements" to synchronize the sensors by interpolating IMU data.
        IMUInterpolator imuInt;
        //dso::Undistort* undistorter = nullptr;
        double lastImgTimestamp = -1.0;

        DatasetSaver* saver;
    };

}

#endif //MARGDARSHAK_REALSENSED435I_H
