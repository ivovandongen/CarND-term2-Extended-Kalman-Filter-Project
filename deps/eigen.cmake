add_library(Eigen INTERFACE)

target_include_directories(Eigen INTERFACE ${CMAKE_SOURCE_DIR}/deps/Eigen)