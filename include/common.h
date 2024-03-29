// common.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_COMMON_H
#define ARUCOTEST_COMMON_H


// std ////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <thread>

// OpenCV /////////////////////////////////////////////////////////////////////
#include <opencv2/opencv.hpp>

// Eigen //////////////////////////////////////////////////////////////////////
#include <Eigen/Core>
#include <Eigen/Geometry>

// typedefs ===================================================================
// double matricies
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatXX;
typedef Eigen::Matrix<double, 2, 2> Mat22;
typedef Eigen::Matrix<double, 3, 3> Mat33;

// double vectors
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecX;
typedef Eigen::Matrix<double, 2, 1> Vec2;
typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;

// double quaternion
typedef Eigen::Quaterniond Quaternion;


#endif // ARUCOTEST_COMMON_H
