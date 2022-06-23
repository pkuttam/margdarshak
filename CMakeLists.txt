# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(margdarshak)

find_package(realsense2 REQUIRED)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

include_directories(${realsense2_INCLUDE_DIRS})

# Subdirectory Test
add_subdirectory(test)



set(DSO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/dso)

# DSO Source files
set(dso_SOURCE_FILES
        ${DSO_SOURCE_DIR}/FullSystem/FullSystem.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptimize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemOptPoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemDebugStuff.cpp
        ${DSO_SOURCE_DIR}/FullSystem/FullSystemMarginalize.cpp
        ${DSO_SOURCE_DIR}/FullSystem/Residuals.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseTracker.cpp
        ${DSO_SOURCE_DIR}/FullSystem/CoarseInitializer.cpp
        ${DSO_SOURCE_DIR}/FullSystem/ImmaturePoint.cpp
        ${DSO_SOURCE_DIR}/FullSystem/HessianBlocks.cpp
        ${DSO_SOURCE_DIR}/FullSystem/PixelSelector2.cpp
        ${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctional.cpp
        ${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedTopHessian.cpp
        ${DSO_SOURCE_DIR}/OptimizationBackend/AccumulatedSCHessian.cpp
        ${DSO_SOURCE_DIR}/OptimizationBackend/EnergyFunctionalStructs.cpp
        ${DSO_SOURCE_DIR}/util/settings.cpp
        ${DSO_SOURCE_DIR}/util/Undistort.cpp
        ${DSO_SOURCE_DIR}/util/globalCalib.cpp
        )

set(dmvio_SOURCE_FILES
        src/IMU/IMUIntegration.cpp
        src/GTSAMIntegration/Sim3GTSAM.cpp
        src/IMUInitialization/GravityInitializer.cpp
        src/IMU/IMUTypes.cpp
        src/IMU/IMUSettings.cpp
        src/util/TimeMeasurement.cpp
        src/util/SettingsUtil.cpp
        src/GTSAMIntegration/BAGTSAMIntegration.cpp
        src/IMU/CoarseIMULogic.cpp
        src/IMU/BAIMULogic.cpp
        src/GTSAMIntegration/PoseTransformation.cpp
        src/GTSAMIntegration/Marginalization.cpp
        src/GTSAMIntegration/PoseTransformationIMU.cpp
        src/GTSAMIntegration/PoseTransformationFactor.cpp
        src/IMUInitialization/CoarseIMUInitOptimizer.cpp
        src/IMUInitialization/IMUInitializer.cpp
        src/IMU/IMUUtils.cpp
        src/IMUInitialization/IMUInitSettings.cpp
        src/GTSAMIntegration/GTSAMUtils.cpp
        src/GTSAMIntegration/DelayedMarginalization.cpp
        src/IMUInitialization/PoseGraphBundleAdjustment.cpp
        src/GTSAMIntegration/FEJValues.cpp
        src/IMUInitialization/IMUInitializerStates.cpp
        src/IMUInitialization/IMUInitializerLogic.cpp
        src/IMUInitialization/IMUInitializerTransitions.cpp
        src/GTSAMIntegration/AugmentedScatter.cpp
        src/live/FrameContainer.cpp
        src/live/IMUInterpolator.cpp
        src/util/MainSettings.cpp
        src/live/FrameSkippingStrategy.cpp
        src/live/DatasetSaver.cpp
        )


include_directories(
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/src/dso
        ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
        ${PROJECT_SOURCE_DIR}/thirdparty/sse2neon
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
)


add_executable(vo_d345i src/vo_d435i.cpp)
target_link_libraries(vo_d345i ${realsense2_LIBRARY})