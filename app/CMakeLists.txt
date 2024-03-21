add_executable(collect_aruco_data collect_aruco_data.cpp)
add_executable(run_test run_test.cpp)

target_link_libraries(collect_aruco_data
    aruco_test ${THIRD_PARTY_LIBS})
target_link_libraries(run_test
    aruco_test ${THIRD_PARTY_LIBS})
