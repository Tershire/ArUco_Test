// setting.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference: ORB-SLAM3


#ifndef ARUCOTEST_PORT_SETTING_H
#define ARUCOTEST_PORT_SETTING_H

#include "common.h"
#include "camera/camera.h"
#include "camera/pinhole.h"
#include "camera/brown_conrady.h"


namespace aruco_test
{

/**
 * setting for camera and other sensors.
 * read camera parameters and set camera.
 */
class Setting
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Setting> Ptr;

    // constructor & destructor ///////////////////////////////////////////////
    Setting(const std::string& setting_file_path);

    // getter & setter ////////////////////////////////////////////////////////
    // getter =================================================================
    Camera::Ptr get_usb_camera() const {return usb_camera_;}

    Camera::Ptr get_raspberry_camera() const {return raspberry_camera_;}

    Camera::Ptr get_color_imager() const {return color_imager_;}
    Camera::Ptr get_depth_imager(const int& camera_id) const
    {
        return depth_imagers_.at(camera_id);
    }

private:
    // member data ////////////////////////////////////////////////////////////
    cv::FileStorage file_;

    // USB
    Camera::Ptr usb_camera_;

    // raspberry
    Camera::Ptr raspberry_camera_; 
    
    // realsense
    Camera::Ptr color_imager_;
    std::vector<Camera::Ptr> depth_imagers_;

    // member methods /////////////////////////////////////////////////////////
    /**
     * read parameter in setting file
     */
    template<typename T>
    T read_parameter(cv::FileStorage& file, 
        const std::string& parameter, 
        bool& found, const bool& required=true)
    {
        cv::FileNode node = file[parameter];
        if(node.empty())
        {
            if(required)
            {
                std::cerr << parameter 
                          << " required parameter does not exist, aborting..." 
                          << std::endl;
                exit(-1);
            }
            else
            {
                std::cerr << parameter 
                          << " optional parameter does not exist..." 
                          << std::endl;
                found = false;
                return T();
            }
        }
        else
        {
            found = true;
            return (T) node;
        }
    }

    // ========================================================================
    /**
     * read camera setting then create and set Camera object
     */
    void read_and_set_usb_camera();

    void read_and_set_raspberry_camera();

    void read_and_set_color_imager();
    void read_and_set_depth_imagers(const int& num_cameras);
};

} // namespace aruco_test

#endif // ARUCOTEST_PORT_SETTING_H
