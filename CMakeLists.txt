cmake_minimum_required(VERSION 3.2)
project(aruco_test)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# DEPENDENCIES ################################################################
# CUDA
# set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
# find_package(CUDA 12 REQUIRED)

# set(CMAKE_CUDA_STANDARD 12)
# set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# OpenCV
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

# 
set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS})

# #############################################################################
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include/camera)
include_directories(${PROJECT_SOURCE_DIR}/include/data_types)
include_directories(${PROJECT_SOURCE_DIR}/include/door)
include_directories(${PROJECT_SOURCE_DIR}/include/imu)
include_directories(${PROJECT_SOURCE_DIR}/include/marker)
include_directories(${PROJECT_SOURCE_DIR}/include/port)
include_directories(${PROJECT_SOURCE_DIR}/include/tool)
include_directories(${PROJECT_SOURCE_DIR}/include/toolbox)
include_directories(${PROJECT_SOURCE_DIR}/third-party)
add_subdirectory(app)
add_subdirectory(src)
add_subdirectory(test)
