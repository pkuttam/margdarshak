
#include <iomanip>
#include "settings.h"
#include "FrameContainer.h"
#include "IMUInterpolator.h"
#include "IMUTypes.h"

std::pair<std::unique_ptr<margdarshak::ImageAndExposure>, margdarshak::IMUData>
margdarshak::FrameContainer::getImageAndIMUData(int maxSkipFrames)
{
    std::unique_lock<std::mutex> lock(framesMutex);
    while(frames.size() == 0 && !stopSystem) // Wait for new image.
    {
        frameArrivedCond.wait(lock);
    }

    if(stopSystem) return std::make_pair(nullptr, margdarshak::IMUData{});

    IMUData imuData;

    // Skip frames if necessary.
    // Now frames.size() must be greater than 0.
    size_t useFrame = frames.size() - 1;
    int numFramesAfter = 0;
    if(frames.size() > 1)
    {
        int framesToSkip = frames.size() - 1; // also the index of the frame that will be used.
        if(maxSkipFrames >= 0 && maxSkipFrames < framesToSkip)
        {
            framesToSkip = maxSkipFrames;
        }
        useFrame = framesToSkip;
        numFramesAfter = frames.size() - useFrame - 1;
        if(!margdarshak::setting_debugout_runquiet)
        {
            std::cout << "SKIPPING " << framesToSkip << " FRAMES!" << " frames remaining in queue: "
                      << numFramesAfter << std::endl;
        }
    }

    auto returnImg = std::move(frames[useFrame].img);

    // Fill IMU data to return, also consider IMU data for skipped frames.
    for(int j = 0; j <= useFrame; ++j)
    {
        std::vector<margdarshak::IMUDataDuringInterpolation>& data = frames[j].imuData;
        for(int i = 0; i < data.size(); ++i)
        {
            Eigen::Vector3d acc, gyr;
            for(int x = 0; x < 3; ++x)
            {
                acc(x) = data[i].accData[x];
                gyr(x) = data[i].gyrData[x];
            }

            double integrationTime = 0.0;

            integrationTime = data[i].timestamp - prevTimestamp;
            assert(integrationTime >= 0);

            imuData.emplace_back(acc, gyr, integrationTime);

            prevTimestamp = data[i].timestamp;
        }
    }
    if(prevTimestamp < 0.0)
    {
        prevTimestamp = frames[useFrame].imgTimestamp;
    }
    assert(std::abs(frames[useFrame].imgTimestamp - prevTimestamp) < 0.0001);
    frames.erase(frames.begin(), frames.begin() + useFrame + 1);
    assert(frames.size() == numFramesAfter);

    return std::make_pair(std::move(returnImg), imuData);
}

void margdarshak::FrameContainer::addFrame(Frame frame)
{
    {
        std::unique_lock<std::mutex> lock(framesMutex);
        frames.push_back(std::move(frame));
    }
    frameArrivedCond.notify_all();
}

int margdarshak::FrameContainer::getQueueSize()
{
    std::unique_lock<std::mutex> lock(framesMutex);
    return frames.size();
}

void margdarshak::FrameContainer::stop()
{
    stopSystem = true;
    frameArrivedCond.notify_all();
}

margdarshak::Frame::Frame(std::unique_ptr<margdarshak::ImageAndExposure>&& img, double imgTimestamp) : img(std::move(img)),
                                                                                         imgTimestamp(imgTimestamp)
{}
