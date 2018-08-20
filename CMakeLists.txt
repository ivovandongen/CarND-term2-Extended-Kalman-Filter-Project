cmake_minimum_required(VERSION 3.10)
project(ExtendedKF)

set(CMAKE_CXX_STANDARD 14)

# CMake modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

if (UNIX)
    # Silence warnings in Eigen
    add_compile_options(-Wno-deprecated-register)
endif()


# Main executable

add_subdirectory(src)

add_executable(ExtendedKF
    ${CMAKE_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(ExtendedKF core)

# Tests

add_subdirectory(test)
