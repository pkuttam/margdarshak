#include "camera.h"
#include <memory>



int main(){
    std::string calibSavePath = "./calib_data.txt";
    margdarshak::FrameContainer frameContainer;
    std::unique_ptr<margdarshak::DatasetSaver> dataSaver;
    dataSaver = std::make_unique<margdarshak::DatasetSaver>("./img1");

    margdarshak::camera camera(frameContainer, calibSavePath, dataSaver.get());
    camera.start();


    while(true) {
       // Skip the first few frames if the start variable is set.
        auto pair = frameContainer.getImageAndIMUData();
        auto a = pair.first.get();
        auto b = pair.second.size();
        std::cout << b<< std::endl;

    }

    return(0);
}