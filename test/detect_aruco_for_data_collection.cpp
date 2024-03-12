// detect_aruco_minimal.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 07
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace aruco_test;


int main(int argc, char **argv)
{
    // configure system =======================================================
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    System::Ptr system = std::make_shared<System>(configuration_file_path);    
    assert(system->initialize() == true);
    
    // connect to Tello =======================================================
    if (system->input_mode_ == "tello")
    {
        Tello tello;
        if (!tello.connect()) 
        {
            return -1;
        }

        tello.enable_video_stream();
    }

    // configure system components ============================================
    ArUco_Detector::Ptr aruco_detector = system->get_aruco_detector();
    
    // initiate threads =======================================================
    aruco_detector->run_for_data_collection_as_thread();

    // main thread task =======================================================
    while (true)
    {
        if (cv::waitKey(1) == 27)
        {
            break;
        }
    }
    
    // join threads ===========================================================
    aruco_detector->close();

    return 0;
}
