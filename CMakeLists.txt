project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
   src/ukf.cpp
   src/main.cpp
   src/tools.cpp src/UKFMeasurement.cpp src/UKFMeasurement.h src/LidarUKFMeasurement.cpp src/LidarUKFMeasurement.h src/RadarUKFMeasurement.cpp src/RadarUKFMeasurement.h)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
add_executable(UnscentedKF ${sources})
