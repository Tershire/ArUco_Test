add_library(aruco_test SHARED
    camera/camera.cpp
    camera/brown_conrady.cpp
    camera/pinhole.cpp
    imu/imu_reader.cpp
    marker/aruco_detector.cpp
    port/config.cpp
    port/setting.cpp
    system.cpp)

target_link_libraries(aruco_test 
    ${THIRD_PARTY_LIBS})
