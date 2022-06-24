
#include "RealsenseD435I.h"
#include <iostream>
#include <iomanip>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
//#include "IMU/IMUTypes.h"
//#include "sophus/se3.hpp"
//#include "util/MinimalImage.h"

using namespace margdarshak;
using std::vector;

static cv::Mat frame_to_mat(const rs2::frame& f);

margdarshak::Realsense435I::Realsense435I(FrameContainer& frameContainer, std::string cameraCalibSavePath, DatasetSaver*
datasetSaver)
        : imuInt(frameContainer, datasetSaver), cameraCalibSavePath(cameraCalibSavePath), saver(datasetSaver)
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    pipe = rs2::pipeline(context);

    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    config.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

    if(context.query_devices().size() == 0)
    {
        std::cout << "Waiting for device to be connected" << std::endl;
        rs2::device_hub hub(context);
        hub.wait_for_device();
    }

    for(auto& s : context.query_devices()[0].query_sensors())
    {
        std::cout << "Sensor " << s.get_info(RS2_CAMERA_INFO_NAME)
                  << ". Supported options:" << std::endl;

        for(const auto& o : s.get_supported_options())
        {
            std::cout << "\t" << rs2_option_to_string(o) << std::endl;
        }
    }

    auto device = context.query_devices()[0];
    device.hardware_reset();

    std::cout << "Device " << device.get_info(RS2_CAMERA_INFO_NAME)
              << " connected" << std::endl;
}

void margdarshak::Realsense435I::start()
{
    std::cout << "Starting realsense feeds\n";
    auto callback = [&](const rs2::frame& frame)
    {
        if(!calibrationRead)
        {
            std::cout << " no calibration data till now" << std::endl;
            return;
        }
        if(auto fp = frame.as<rs2::motion_frame>())
        {
            auto motion = frame.as<rs2::motion_frame>();

            if(motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
               motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                auto motionData = motion.get_motion_data();
                vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;
                //std::cout << data[0] << std::endl;

                // Multiply by factory calibration scale and subtract bias.
                for(int i = 0; i < 3; ++i)
                {
                    data[i] = data[i] * gyroIntrinsics.data[i][i] - gyroIntrinsics.data[i][3];
                }

                // timestamp is in milliseconds, but shall be in seconds
                imuInt.addGyrData(data, motion.get_timestamp() / 1000.0);

            }else if(motion &&
                     motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
                     motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                auto motionData = motion.get_motion_data();
                vector<float> data(3);
                data[0] = motionData.x;
                data[1] = motionData.y;
                data[2] = motionData.z;

                // Multiply by factory calibration scale and subtract bias.
                for(int i = 0; i < 3; ++i)
                {
                    data[i] = data[i] * accelIntrinsics.data[i][i] - accelIntrinsics.data[i][3];
                }

                imuInt.addAccData(data, motion.get_timestamp() / 1000.0);
            }
        }else if(auto fs = frame.as<rs2::frameset>())
        {
            auto f = fs[useCam]; // We only use left camera
            if(!f.as<rs2::video_frame>())
            {
                std::cout << "Weird Frame, skipping" << std::endl;
                return;
            }
            auto vf = f.as<rs2::video_frame>();

            double timestamp = vf.get_timestamp();

            // We somehow seem to get each image twice.
            if(std::abs(timestamp - lastImgTimestamp) > 0.001)
            {
                cv::Mat mat = frame_to_mat(f);
                assert(mat.type() == CV_8U);

                // Multiply exposure by 1000, as we want milliseconds.
                double exposure = vf.get_frame_metadata(RS2_FRAME_METADATA_ACTUAL_EXPOSURE) * 1e-3;

                if(saver)
                {
                    saver->addImage(mat, timestamp / 1000.0, exposure);
                }

                //auto img = std::make_unique<dso::MinimalImageB>(mat.cols, mat.rows);
                //memcpy(img->data, mat.data, mat.rows * mat.cols);

                // timestamp is in milliseconds, but shall be in seconds
                double finalTimestamp = timestamp / 1000.0;
                // gets float exposure and double timestamp
                //std::unique_ptr<dso::ImageAndExposure> finalImage(undistorter->undistort<unsigned char>(
                //        img.get(),
                //        static_cast<float>(exposure),
                //       finalTimestamp));
                //img.reset();

                // Add image to the IMU interpolator, which will forward it to the FrameContainer, once the
                // corresponding IMU data is available.
                //imuInt.addImage(std::move(finalImage), finalTimestamp);

                lastImgTimestamp = timestamp;
            }
        }
    };

    //std::cout << "setting profile\n";
    profile = pipe.start(config, callback);
    //std::cout << "reading now" << std::endl;
    readCalibration();
}

void dmvio::Realsense435I::readCalibration()
{
    if(calibrationRead) return;
    auto accel_stream = profile.get_stream(RS2_STREAM_ACCEL);
    auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
    auto cam0_stream = profile.get_stream(RS2_STREAM_COLOR);
    //auto cam1_stream = profile.get_stream(RS2_STREAM_FISHEYE, 2);

    // get gyro extrinsics
    if(auto gyro = gyro_stream.as<rs2::motion_stream_profile>())
    {
        gyroIntrinsics = gyro.get_motion_intrinsics();

        Eigen::Matrix<double, 3, 4> gyroMatrix;
        for(int x = 0; x < 3; ++x)
        {
            for(int y = 0; y < 4; ++y)
            {
                gyroMatrix(x, y) = gyroIntrinsics.data[x][y];
            }
        }

        // When receiving gyro measurements we need to multiply by the scale and subtract the bias!
        std::cout << "Gyro Matrix\n" << gyroMatrix << std::endl;

        Eigen::Vector3d gyro_noise_std = Eigen::Vector3d(gyroIntrinsics.noise_variances[0],
                                                         gyroIntrinsics.noise_variances[1],
                                                         gyroIntrinsics.noise_variances[2]).cwiseSqrt();

        Eigen::Vector3d gyro_bias_std = Eigen::Vector3d(gyroIntrinsics.bias_variances[0],
                                                        gyroIntrinsics.bias_variances[1],
                                                        gyroIntrinsics.bias_variances[2]).cwiseSqrt();

        std::cout << "Gyro noise var: " << gyro_noise_std
                  << " bias var: " << gyro_bias_std << std::endl;
    }else
    {
        std::abort();
    }

    // get accel extrinsics
    if(auto accel = accel_stream.as<rs2::motion_stream_profile>())
    {
        accelIntrinsics = accel.get_motion_intrinsics();
        Eigen::Matrix<double, 3, 4> accelMatrix;
        for(int x = 0; x < 3; ++x)
        {
            for(int y = 0; y < 4; ++y)
            {
                accelMatrix(x, y) = accelIntrinsics.data[x][y];
            }
        }

        Eigen::Vector3d accel_noise_std = Eigen::Vector3d(accelIntrinsics.noise_variances[0],
                                                          accelIntrinsics.noise_variances[1],
                                                          accelIntrinsics.noise_variances[2]).cwiseSqrt();

        Eigen::Vector3d accel_bias_std = Eigen::Vector3d(accelIntrinsics.bias_variances[0],
                                                         accelIntrinsics.bias_variances[1],
                                                         accelIntrinsics.bias_variances[2]).cwiseSqrt();

        std::cout << "Accel noise var: " << accel_noise_std
                  << " bias var: " << accel_bias_std << std::endl;
    }else
    {
        std::abort();
    }

    // get camera ex-/intrinsics
    for(const auto& cam_stream: {cam0_stream} )
    {
        if(auto cam = cam_stream.as<rs2::video_stream_profile>())
        {
            // extrinsics
            rs2_extrinsics ex = cam.get_extrinsics_to(gyro_stream);
            Eigen::Matrix3f rot = Eigen::Map<Eigen::Matrix3f>(ex.rotation);
            Eigen::Vector3f trans = Eigen::Map<Eigen::Vector3f>(ex.translation);

            Sophus::SE3d T_imu_cam(rot.cast<double>(), trans.cast<double>());

            std::cout << "D_imu_cam: " << T_imu_cam.matrix() << std::endl;
            std::cout << "D_cam_imu: " << T_imu_cam.inverse().matrix() << std::endl;

            imuCalibration = std::make_unique<dmvio::IMUCalibration>(T_imu_cam.inverse());

            // intrinsics
            rs2_intrinsics intrinsics = cam.get_intrinsics();

            // We assume Kanala Brandt model.
            std::cout << "distortion model type: " << intrinsics.model << std::endl;
            std::cout << intrinsics.fx << " " << intrinsics.fy << " " << intrinsics.ppx << " "
                         << intrinsics.ppy << " " << intrinsics.coeffs[0] << " " << intrinsics.coeffs[1] << " " <<
                         intrinsics.coeffs[2] << " " << intrinsics.coeffs[3] <<  " " << intrinsics.coeffs[4] <<  " " << intrinsics.coeffs[5]<< "\n";
            //assert(intrinsics.model == RS2_DISTORTION_KANNALA_BRANDT4); // all distortion are zero
            //std::cout << intrinsics.
            // Write camera calibration to file.
            std::ofstream calibStream(cameraCalibSavePath);

            calibStream << "KannalaBrandt " << intrinsics.fx << " " << intrinsics.fy << " " << intrinsics.ppx << " "
                        << intrinsics.ppy << " " << intrinsics.coeffs[0] << " " << intrinsics.coeffs[1] << " " <<
                        intrinsics.coeffs[2] << " " << intrinsics.coeffs[3] << "\n";
            calibStream << cam.width() << " " << cam.height() << "\n";
            // We rectify to a focal length of 0.2 instead of using the full size as otherwise too much of the
            // rectified image will be focus on a small outer part of the original fisheye image.
            calibStream << "0.2 0.2 0.499 0.499 0\n";
            calibStream << cam.width() << " " << cam.height() << "\n";
            calibStream.flush();

        }else
        {
            std::abort();
        }
    }

    calibrationRead = true;

}

void Realsense435I::setUndistorter(dso::Undistort* undistort)
{
    this->undistorter = undistort;
}


// This Method was copied from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/cv-helpers.hpp
// License: Apache 2.0. See http://www.apache.org/licenses/LICENSE-2.0 or below.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.
// Convert rs2::frame to cv::Mat
static cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if(f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void*) f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }else if(f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }else if(f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*) f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}
