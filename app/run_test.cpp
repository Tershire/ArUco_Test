// run_test.cpp

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 MAR 21
// Wonhee LEE

// reference:


#include <iostream>

#include "port/config.h"
#include "system.h"
#include "marker/aruco_detector.h"
#include "tello.hpp"

using namespace aruco_test;


void draw(const cv::Mat& image);


int main(int argc, char **argv)
{
    // configure system =======================================================
    std::string configuration_file_path = "./config/vision_system_config.yaml";
    
    System::Ptr system = std::make_shared<System>(configuration_file_path);    
    assert(system->initialize() == true);

    // connect to Tello =======================================================
    Tello tello;
    if (!tello.connect()) 
    {
        return -1;
    }

    // ------------------------------------------------------------------------
    tello.enable_video_stream();
    cv::VideoCapture cap{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};

    // check capture
    if (!cap.isOpened()) 
    {
        std::cerr << "ERROR: capturer is not open\n";
        return -1;
    }

    // get FPS
    double fps = cap.get(cv::CAP_PROP_FPS);
    std::cout << "FPS: " << fps << std::endl;
    
    // configure system components ============================================
    ArUco_Detector::Ptr aruco_detector = system->get_aruco_detector();
    bool verbose = aruco_detector->verbose_;
    bool draw_and_show;
    bool save_video;

    int a = Config::read<int>("draw_and_show");
    if (a == 0)
        draw_and_show = false;
    else
        draw_and_show = true;

    int b = Config::read<int>("save_video");
    if (b == 0)
        save_video = false;
    else
        save_video = true;

    // data collection ========================================================
    std::ofstream ofstream;
    ofstream.open(Config::read<std::string>("csv_file_name") + ".csv");

    // create VideoWriter object ==============================================
    // get one frame to know video properties ---------------------------------
    cv::Mat image;

    cap >> image;

    if (image.empty())
    {
        std::cerr << "ERROR: blank frame\n";
        return -1;
    }
    // ------------------------------------------------------------------------
    
    int codec = cv::VideoWriter::fourcc('A', 'V', 'C', '1');

    cv::VideoWriter writer{Config::read<std::string>("csv_file_name") + ".mp4", codec, fps, image.size(), true};

    if (!writer.isOpened()) 
    {
        std::cerr << "ERROR: Writer is not open\n";
        return -1;
    }

    // main thread task =======================================================
    // tello.takeoff();
    // tello.move_up(20); // distance in [cm]

    std::cout << "START" << std::endl;

    for (;;)
    {
        // get image
        cap >> image;

        // get timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
        long timestamp = t.count();

        // check image
        if (image.empty()) 
        {
            std::cerr << "ERROR: blank frame\n";
            break;
        }
        
        // convert to grayscale
        cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);

        // detect and estimate pose
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> p2Dss_pixel;
        std::vector<cv::Point2f> p2Ds_pixel;
        cv::Vec3d rvec, tvec;

        aruco_detector->detect(image, ids, p2Dss_pixel);

        int target_index = aruco_detector->find_target_index(ids);
        bool target_found = target_index >= 0;
        if (target_found)
        {
            p2Ds_pixel = p2Dss_pixel.at(target_index);

            aruco_detector->estimate_pose(p2Ds_pixel, rvec, tvec);

            // output
            ofstream << timestamp << ',' << 
                rvec[0] << ',' << rvec[1] << ',' << rvec[2] << ',' <<
                tvec[0] << ',' << tvec[1] << ',' << tvec[2] << '\n';

            if (verbose)
            {
                std::cout << "timestamp: " << timestamp << std::endl;
                std::cout << "rvec: " << rvec << std::endl;
                std::cout << "tvec: " << tvec << std::endl;
            }
        }

        // auxiliary output ---------------------------------------------------
        cv::Mat image_out;

        // draw & show
        if (draw_and_show)
        {
            cv::cvtColor(image, image_out, cv::COLOR_GRAY2BGR);

            if (!ids.empty())
            {
                cv::aruco::drawDetectedMarkers(image_out, p2Dss_pixel, ids);
            }

            if (target_index >= 0)
            {
                cv::drawFrameAxes(image_out, 
                    system->get_mono_camera()->cameraMatrix_, system->get_mono_camera()->distCoeffs_, 
                    rvec, tvec, 0.1, 2);
            }

            cv::imshow("ArUco Tracker", image_out);
        }

        // save video
        if (save_video)
        {
            writer.write(image_out);
        }
        // --------------------------------------------------------------------
        
        // terminating condition
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
    
    ofstream.close();

    std::cout << "END" << std::endl;
    
    // tello.land();

    return 0;
}
