// aruco_detector.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_MARKER_ARUCODETECTOR_H
#define ARUCOTEST_MARKER_ARUCODETECTOR_H

#include <fstream>

#include "common.h"
#include "camera/camera.h"


namespace aruco_test
{

/**
 * detect, and estimate pose of ArUco code
 * uses IPPE PnP method enforced with RANSAC
 * calculates adjacent poses' error, if not good, it does not update current pose
 * if the pose continues to be bad, it resets within a given buffer time,
 * so that pose can be updated without too much delay.
 */
class ArUco_Detector
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<ArUco_Detector> Ptr;

    // state member ///////////////////////////////////////////////////////////
    enum Input_Mode
    {
        USB,
        VIDEO,
        TELLO,
        RASPBERRY,
        REALSENSE
    };

    // member data ////////////////////////////////////////////////////////////
    int target_id_;

    bool verbose_;

    // constructor & destructor ///////////////////////////////////////////////
    ArUco_Detector();

    ArUco_Detector(const int& target_id,
        const std::string& predifined_dictionary_name,
        const double& marker_length, const Camera::Ptr camera);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Vec3 get_euler_angles_cm() const {return euler_angles_cm_;}
    bool get_target_found() const {return target_found_;}

    // setter =================================================================
    void set_target_id(const int& target_id) {target_id_ = target_id;}
    void set_verbose(const bool& verbose) {verbose_ = verbose;}

    // port -------------------------------------------------------------------
    void set_input_mode(const Input_Mode& input_mode) {input_mode_ = input_mode;}

    // member methods /////////////////////////////////////////////////////////
    bool run_for_data_collection();
    bool run_for_data_collection_as_thread();
    void close();

    int find_target_index(const std::vector<int>& ids) const;

    // interface ==============================================================
    // int update_state_output();

private:
    // member data ////////////////////////////////////////////////////////////
    // ArUco ==================================================================
    double marker_length_;
    
    cv::aruco::Dictionary dictionary_;
    cv::aruco::DetectorParameters detector_parameters_;

    cv::Ptr<cv::aruco::ArucoDetector> detector_;

    std::thread thread_;

    // camera =================================================================
    Camera::Ptr camera_;
    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    // pose estimation ========================================================
    // PnP --------------------------------------------------------------------
    std::vector<cv::Point3d> p3Ds_target_;
    bool target_found_;

    bool is_pose_ok_;

    Vec3 euler_angles_cm_;

    // port ===================================================================
    Input_Mode input_mode_;
    float resize_scale_factor_;

    float mono_camera_scale_factor_;

    // data collection ========================================================
    long t_;
    std::ofstream ofstream_;
    std::string csv_file_name_;
};

} // namespace aruco_test

#endif // ARUCOTEST_MARKER_ARUCODETECTOR_H
