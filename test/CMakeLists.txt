find_package(realsense2 REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(${realsense2_INCLUDE_DIRS})
add_executable(d345i_test d435i.cpp)
target_link_libraries(d345i_test ${realsense2_LIBRARY})

add_executable(d345i_info d435i_info.cpp)
target_link_libraries(d345i_info ${realsense2_LIBRARY})

add_executable(d345i_check d435i_sensor_check.cpp)
target_link_libraries(d345i_check ${realsense2_LIBRARY})