cmake_minimum_required(VERSION 3.16)
project(IncLIO VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# In Release, compile out DEBUG/TRACE logs entirely (zero overhead).
# In Debug, keep everything.
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_INFO)
else()
  add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)
endif()

# ─────────────────────────────────────────────
# Dependencies
# ─────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)       # for parallel NDT if needed
find_package(spdlog REQUIRED)
find_package(MPI QUIET)           # needed by VTK (pulled in via PCL)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Sophus: try system install first, fall back to FetchContent
find_package(Sophus QUIET)
if(NOT Sophus_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    Sophus
    GIT_REPOSITORY https://github.com/strasdat/Sophus.git
    GIT_TAG        1.22.10
  )
  set(SOPHUS_USE_BASIC_LOGGING ON CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(Sophus)
endif()

# ─────────────────────────────────────────────
# Common library (ESKF, NDT, utilities)
# ─────────────────────────────────────────────
file(GLOB_RECURSE COMMON_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/common/ieskf/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/common/ndt/*.cpp
)

add_library(inclio_common ${COMMON_SRCS})
target_include_directories(inclio_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/common
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_include_directories(inclio_common PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(inclio_common PUBLIC
  Eigen3::Eigen
  Sophus::Sophus
  TBB::tbb
  spdlog::spdlog
  ${PCL_LIBRARIES}
)

# ─────────────────────────────────────────────
# Main IncLIO library
# ─────────────────────────────────────────────
file(GLOB LIO_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_library(inclio ${LIO_SRCS})
target_include_directories(inclio PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/common
)
target_link_libraries(inclio PUBLIC
  inclio_common
  yaml-cpp
)

# ─────────────────────────────────────────────
# Install
# ─────────────────────────────────────────────
install(TARGETS inclio inclio_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/inclio/ DESTINATION include/inclio)
install(DIRECTORY common/         DESTINATION include/inclio/common
  FILES_MATCHING PATTERN "*.hpp"
)

# ─────────────────────────────────────────────
# Activate the Viewer flag 
# ─────────────────────────────────────────────
option(BUILD_TOOLS "Build visualization tools (requires Pangolin)" ON)
if(BUILD_TOOLS)
  add_compile_definitions(BUILD_TOOLS)
endif()

# ─────────────────────────────────────────────
# Tests (optional)
# ─────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

# ─────────────────────────────────────────────
# Build the player as it is currently under tools/
# ─────────────────────────────────────────────
add_subdirectory(tools)