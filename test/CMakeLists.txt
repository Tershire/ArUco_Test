add_executable(tello_test tello_test.cpp)
add_executable(tello_flight_test tello_flight_test.cpp)
add_executable(tello_vision_test tello_vision_test.cpp)
add_executable(tello_sensor_test tello_sensor_test.cpp)
add_executable(detect_aruco_for_data_collection detect_aruco_for_data_collection.cpp)
add_executable(tello_vision_test_minimal tello_vision_test_minimal.cpp)

target_link_libraries(tello_test
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(tello_flight_test
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(tello_vision_test
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(tello_sensor_test
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(detect_aruco_for_data_collection
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(tello_vision_test_minimal
    aruco_test ${THIRD_PARTY_LIBS})
