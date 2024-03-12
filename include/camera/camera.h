// camera.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_CAMERA_CAMERA_H
#define ARUCOTEST_CAMERA_CAMERA_H

#include "common.h"


namespace aruco_test
{

/**
 * 
 */
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Camera_Model
    {
        PINHOLE,
        BROWN_CONRADY,
        KANNALA_BRANDT
    };

    // member data ////////////////////////////////////////////////////////////   
    std::vector<cv::Mat> intrinsic_parameters_;

    cv::Mat cameraMatrix_; // camera matrix
    cv::Mat distCoeffs_;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;

    Camera_Model camera_model_;

    // constructor & destructor ///////////////////////////////////////////////
    Camera() {}
    
    Camera(const std::vector<cv::Mat>& intrinsic_parameters);

    // virtual member methods /////////////////////////////////////////////////
    /**
     * project
     * @return p2D_pixel (of cN)
     */
    virtual Vec2 camera_to_pixel(const Vec3& p3D_camera) = 0;

    /**
     * unproject on normalized image plane
     * @return p3D_normalized (of cN)
     */
    virtual Vec3 pixel_to_normalized_image_plane(const Vec2& p2D_pixel) = 0;
};

} // namespace aruco_test

#endif // ARUCOTEST_CAMERA_CAMERA_H
