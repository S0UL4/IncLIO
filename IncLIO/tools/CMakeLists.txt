# Try system Pangolin first, fall back to thirdparty source
find_package(Pangolin QUIET)
if(NOT Pangolin_FOUND)
    message(STATUS "Pangolin not found on system, building from thirdparty/Pangolin")
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/Pangolin ${CMAKE_BINARY_DIR}/thirdparty/Pangolin)
endif()

add_library(${PROJECT_NAME}.ui
        ui/pangolin_window.cc
        ui/pangolin_window_impl.cc
        ui/ui_car.cc
        ui/ui_trajectory.cc
        ui/ui_cloud.cc
        )

target_include_directories(${PROJECT_NAME}.ui PUBLIC
        ${CMAKE_SOURCE_DIR}
        )

target_link_libraries(${PROJECT_NAME}.ui PUBLIC
        inclio_common
        ${Pangolin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(ui_test ui/ui_test.cc)
target_link_libraries(ui_test
        ${PROJECT_NAME}.ui
        )

# ── ROS2 bag player ──────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(SQLITE3 REQUIRED sqlite3)

add_executable(run_bag run_bag.cc)
target_include_directories(run_bag PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${SQLITE3_INCLUDE_DIRS}
        )
target_link_libraries(run_bag PRIVATE
        inclio
        ${PROJECT_NAME}.ui
        ${SQLITE3_LIBRARIES}
        yaml-cpp
        )
