// config.h

/////////////////////////
// IONLAB ISAE-SUPAERO //
// TELLO SLAM PROJECT  //
/////////////////////////

// 2024 JAN 15
// Wonhee LEE

// reference:


#ifndef ARUCOTEST_CONFIG_H
#define ARUCOTEST_CONFIG_H

#include "common.h"


namespace aruco_test
{

/**
 * get configuration file to initialize system setting
 */
class Config
{
public:
    // constructor & destructor ///////////////////////////////////////////////
    ~Config(); // close the file when deconstructing

    // member methods /////////////////////////////////////////////////////////
    /**
     * create a Config object and take the configuration file as its member data
     */
    static bool initialize(const std::string& file_path);

    // static methods /////////////////////////////////////////////////////////
    /**
     * access the parameter values
     */
    template <typename T>
    static T read(const std::string& parameter)
    {
        return T(Config::config_->file_[parameter]);
    }

private:
    // member data ////////////////////////////////////////////////////////////
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    // constructor & destructor ///////////////////////////////////////////////
    Config() {} // private constructor makes a singleton
};

} // namespace aruco_test

#endif // ARUCOTEST_CONFIG_H
