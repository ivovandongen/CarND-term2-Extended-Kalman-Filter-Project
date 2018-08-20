cmake_minimum_required(VERSION 3.10)
project(ExtendedKF)

set(CMAKE_CXX_STANDARD 14)

# CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Submodules
include(deps/eigen.cmake)
include(deps/uWebSockets.cmake)

if (UNIX)
    # Silence warnings in Eigen
    add_compile_options(-Wno-deprecated-register)
endif()

# Main executable

add_executable(ExtendedKF
    ${CMAKE_SOURCE_DIR}/src/main.cpp
    ${CMAKE_SOURCE_DIR}/src/tools.cpp
    ${CMAKE_SOURCE_DIR}/src/FusionEKF.cpp
    ${CMAKE_SOURCE_DIR}/src/kalman_filter.cpp
    ${CMAKE_SOURCE_DIR}/src/tools.h
    ${CMAKE_SOURCE_DIR}/src/FusionEKF.h
    ${CMAKE_SOURCE_DIR}/src/kalman_filter.h
)

target_link_libraries(ExtendedKF Eigen libuWS)

# Tests
add_subdirectory(test)
