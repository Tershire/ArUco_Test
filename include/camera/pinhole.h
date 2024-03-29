// pinhole.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_CAMERA_PINHOLE_H
#define ARUCOTEST_CAMERA_PINHOLE_H

// #include <assert.h>

#include "camera/camera.h"


namespace aruco_test
{

/**
 * 
 */
class Pinhole: public Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // constructor & destructor ///////////////////////////////////////////////
    Pinhole()
    {
        camera_model_ = PINHOLE;
    }

    Pinhole(const std::vector<cv::Mat>& intrinsic_parameters)
        : Camera(intrinsic_parameters)
    {     
        camera_model_ = PINHOLE;
    }

    // virtual member methods /////////////////////////////////////////////////
    Vec2 camera_to_pixel(const Vec3& p3D_camera)
    {
        return Vec2(fx_ * p3D_camera[0] / p3D_camera[2] + cx_,
                    fy_ * p3D_camera[1] / p3D_camera[2] + cy_);
    }

    Vec3 pixel_to_normalized_image_plane(const Vec2& p2D_pixel)
    {
        return Vec3((p2D_pixel[0] - cx_) / fx_,
                    (p2D_pixel[1] - cy_) / fy_,
                    1);
    }
};

} // namespace aruco_test

#endif // ARUCOTEST_CAMERA_PINHOLE_H
