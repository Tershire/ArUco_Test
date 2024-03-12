// system.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_SYSTEM_H
#define ARUCOTEST_SYSTEM_H

#include "common.h"
#include "port/setting.h"
#include "camera/camera.h"
#include "marker/aruco_detector.h"
#include "imu/imu_reader.h"


namespace aruco_test
{

/**
 * vision system
 */
class System
{
public:
    typedef std::shared_ptr<System> Ptr;

    // member data ////////////////////////////////////////////////////////////
    std::string input_mode_;
    bool verbose_;

    // constructor & destructor ///////////////////////////////////////////////
    System(const std::string& configuration_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    ArUco_Detector::Ptr get_aruco_detector() const {return aruco_detector_;}
    IMU_Reader::Ptr get_imu_reader() const {return imu_reader_;}

    // member methods /////////////////////////////////////////////////////////
    /**
     * initialize system
     * @return true if success
     */
    bool initialize();
    
    /**
     * run system in infinite loop
     */
    void run();

    /**
     * investigate roll output delay from camera and IMU
     */
    void run_roll_reader();

private:
    // member data ////////////////////////////////////////////////////////////
    std::string configuration_file_path_;
    
    // port ===================================================================
    Setting::Ptr setting_ = nullptr;

    // camera -----------------------------------------------------------------
    std::string mono_camera_to_use_;
    Camera::Ptr mono_camera_;

    // system components ======================================================
    ArUco_Detector::Ptr aruco_detector_ = nullptr;
    IMU_Reader::Ptr imu_reader_ = nullptr;

    // ArUco Detector =========================================================
    std::string predifined_dictionary_name_;
    float marker_length_;
};

} // namespace aruco_test

#endif // ARUCOTEST_SYSTEM_H
