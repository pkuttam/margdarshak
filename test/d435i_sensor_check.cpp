// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout

// Hello RealSense example demonstrates the basics of connecting to a RealSense device
// and taking advantage of depth data
int main(int argc, char *argv[]) try {
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    rs2::config cfg;

    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F,200);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F,200);
    double start_ts = 0;
    int cnt = 0;
    // Configure and start the pipeline
    auto profile = p.start(cfg, [&](rs2::frame frame) {
        // Cast the frame that arrived to motion frame
        auto motion = frame.as<rs2::motion_frame>();
        auto fs = frame.as<rs2::frameset>();
        // If casting succeeded and the arrived frame is from gyro stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get the timestamp of the current frame
            double ts = motion.get_timestamp();
            if(cnt==0) start_ts = ts;
            // Get gyro measures
            rs2_vector gyro_sample = motion.get_motion_data();
            cnt = cnt + 1;
            std::cout<< "timestamp:" <<ts-start_ts << " Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << std::endl;
            // Call function that computes the angle of motion based on the retrieved measures
        }
        // If casting succeeded and the arrived frame is from accelerometer stream
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // Get accelerometer measures
            double ts = motion.get_timestamp();
            if(cnt==0) start_ts = ts;
            rs2_vector accel_sample = motion.get_motion_data();
            cnt = cnt + 1;
            std::cout << "timestamp:" <<ts-start_ts << " Accel:" << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z <<  std::endl;
            // Call function that computes the angle of motion based on the retrieved measures
        }

        if (fs)
        {
            auto f = fs[0]; // one camera
            if(!f.as<rs2::video_frame>())
            {
                std::cout << "Weird Frame, skipping" << std::endl;
                return;
            }
            auto vf = f.as<rs2::video_frame>();

            double ts = vf.get_timestamp();
            std::cout << "timestamp:" <<ts-start_ts << " Video_frame:" << std::endl;
        }


    });
    while(true){

    }
    p.stop();
    return EXIT_SUCCESS;
}
catch (const rs2::error &e) {
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
