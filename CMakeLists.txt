cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(PointCloudPoseEstimation)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (PointCloudPoseEstimation src/PointCloudPoseEstimation.cpp include/PointCloudToolbox.h include/PointCloudPoseEstimation.h include/PointCloudToolbox.cpp)
target_link_libraries (PointCloudPoseEstimation ${PCL_LIBRARIES} ${OpenCV_LIBS})
