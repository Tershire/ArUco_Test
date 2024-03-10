add_executable(collect_aruco_data collect_aruco_data.cpp)

target_link_libraries(collect_aruco_data
    aruco_test ${THIRD_PARTY_LIBS})
