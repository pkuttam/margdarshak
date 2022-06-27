cmake_minimum_required(VERSION 3.1.0)

project(margdarshak)

find_package(realsense2 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Boost COMPONENTS system thread filesystem chrono serialization date_time timer)
if(Boost_FOUND)
    message("Boost Found!")
endif()


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


include_directories(${realsense2_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Subdirectory Test
add_subdirectory(test)

set(SRC src/IMUTypes.cpp
        src/DatasetSaver.cpp
        src/FrameContainer.cpp
        src/TimeMeasurement.cpp
        src/DatasetSaver.cpp
        src/settings.cpp
        src/IMUInterpolator.cpp
        src/RealsenseD435I.cpp
        src/camera.cpp)



include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/third_party/Sophus)

add_executable(main check_camera_class.cpp ${SRC})
target_link_libraries(main ${realsense2_LIBRARY} ${OpenCV_LIBS} ${Boost_LIBRARIES} )