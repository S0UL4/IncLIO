find_package(PCL REQUIRED COMPONENTS registration)

add_executable(benchmark_ndt benchmark_ndt.cpp)

target_link_libraries(benchmark_ndt
    inclio_common          # NDT, utils — propagates Eigen, Sophus, TBB, PCL, spdlog
    ${PCL_LIBRARIES}       # adds pcl_registration for pcl::NormalDistributionsTransform
)

add_executable(benchmark_undistort benchmark_undistort.cpp)

target_link_libraries(benchmark_undistort
    inclio                 # includes ct_undistort + lio; propagates inclio_common
)

# Wheel-odometry IESKF unit test (ObserveWheelSpeed)
add_executable(test_wheel_odom test_wheel_odom.cpp)

target_link_libraries(test_wheel_odom
    inclio_common          # IESKF::ObserveWheelSpeed lives in inclio_common
)

install(TARGETS benchmark_ndt benchmark_undistort test_wheel_odom
    DESTINATION lib/${PROJECT_NAME}
)
