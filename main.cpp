#include "DatasetSaver.h"
#include "FrameContainer.h"
#include <memory>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>

// If mainSettings.calib is set we use this instead of the factory calibration.
std::string calibSavePath = "./factoryCalibrationT265Camera.txt"; // Factory calibration will be saved here.

margdarshak::FrameContainer frameContainer;
std::unique_ptr<margdarshak::DatasetSaver> datasetSaver;
std::string saveDatasetPath = "./img1";

int main() {

    rs2::context context;
    rs2::config config;
    rs2::pipeline pipe;

    rs2::pipeline_profile profile;
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    pipe = rs2::pipeline(context);

    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

    //datasetSaver = std::make_unique<margdarshak::DatasetSaver>("./img1");

    auto callback = [&](const rs2::frame &frame) {
        if (auto fp = frame.as<rs2::motion_frame>()) {
            auto motion = frame.as<rs2::motion_frame>();

            if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
                motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                auto motionData = motion.get_motion_data();
                std::vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;
                std::cout << "gyro timestamp: "<< motion.get_timestamp() / 1000.0 << std::endl;

                // Multiply by factory calibration scale and subtract bias.
                //for(int i = 0; i < 3; ++i)
                //{
                //    data[i] = data[i] * gyroIntrinsics.data[i][i] - gyroIntrinsics.data[i][3];
                //}

                // timestamp is in milliseconds, but shall be in seconds
                //imuInt.addGyrData(data, motion.get_timestamp() / 1000.0);

            } else if (motion &&
                       motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                       motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                auto motionData = motion.get_motion_data();
                std::vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;
                std::cout << "accel timestamp: "<< motion.get_timestamp() / 1000.0 << std::endl;

                // Multiply by factory calibration scale and subtract bias.
                //for(int i = 0; i < 3; ++i)
                //{
                //    data[i] = data[i] * accelIntrinsics.data[i][i] - accelIntrinsics.data[i][3];
                //}

                //imuInt.addAccData(data, motion.get_timestamp() / 1000.0);
            }
        } else if (auto fs = frame.as<rs2::frameset>()) {
            auto f = fs[0]; // We only use left camera
            if (!f.as<rs2::video_frame>()) {
                std::cout << "Weird Frame, skipping" << std::endl;
                return;
            }
            auto vf = f.as<rs2::video_frame>();

            double timestamp = vf.get_timestamp();
            std::cout << "camera timestamp: "<< timestamp / 1000.0 << std::endl;
            // We somehow seem to get each image twice.
            //if(undistorter && std::abs(timestamp - lastImgTimestamp) > 0.001)
            //{
            //    cv::Mat mat = frame_to_mat(f);
            //    assert(mat.type() == CV_8U);

            //    // Multiply exposure by 1000, as we want milliseconds.
            //    double exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-3;

            //    if(saver)
            //    {
            //        saver->addImage(mat, timestamp / 1000.0, exposure);
            //    }

            //    auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
            //    memcpy(img->data, mat.data, mat.rows * mat.cols);

            // timestamp is in milliseconds, but shall be in seconds
            //    double finalTimestamp = timestamp / 1000.0;
            // gets float exposure and double timestamp
            //    std::unique_ptr<dso::ImageAndExposure> finalImage(undistorter->undistort<unsigned char>(
            //            img.get(),
            //            static_cast<float>(exposure),
            //            finalTimestamp));
            //    img.reset();

            // Add image to the IMU interpolator, which will forward it to the FrameContainer, once the
            // corresponding IMU data is available.
            //   imuInt.addImage(std::move(finalImage), finalTimestamp);

            //   lastImgTimestamp = timestamp;
        }
    };
    profile = pipe.start(config, callback);


    std::cout << "started\n";
    int cnt = 0;
    while (true) {
        //auto pair = frameContainer.getImageAndIMUData();
        cnt = cnt + 1;
    }

}