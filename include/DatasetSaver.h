#ifndef MARGDARSHAK_DATASETSAVER_H
#define MARGDARSHAK_DATASETSAVER_H

#include <string>
#include <opencv2/core/mat.hpp>
#include <mutex>
#include <deque>
#include <fstream>
#include <thread>
#include <condition_variable>

namespace margdarshak
{

// Helper for recording live data to file.
class DatasetSaver
{
public:
    DatasetSaver(std::string saveFolder);

    // timestamp in seconds, exposure in milliseconds.
    void addImage(cv::Mat mat, double timestamp, double exposure);

    void addIMUData(double timestamp, std::vector<float> accData, std::vector<float> gyrData);

    void saveImagesWorker();

    void end();

private:
    std::string imgSaveFolder;

    std::ofstream timesFile, imuFile;

    std::thread imageSaveThread;

    // protects image queue.
    std::mutex mutex;
    std::condition_variable frameArrivedCond;
    std::deque<std::tuple<cv::Mat, double, double>> imageQueue;

    bool running = true;

};


}
#endif //MARGDARSHAK_DATASETSAVER_H
