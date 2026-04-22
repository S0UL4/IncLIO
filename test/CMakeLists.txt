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

install(TARGETS benchmark_ndt benchmark_undistort
    DESTINATION lib/${PROJECT_NAME}
)
