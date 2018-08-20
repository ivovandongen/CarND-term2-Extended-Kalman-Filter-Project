# Submodules
include(${CMAKE_SOURCE_DIR}/deps/eigen.cmake)
include(${CMAKE_SOURCE_DIR}/deps/json.cmake)
include(${CMAKE_SOURCE_DIR}/deps/uWebSockets.cmake)

add_library(core STATIC
	${CMAKE_SOURCE_DIR}/src/tools.cpp
	${CMAKE_SOURCE_DIR}/src/FusionEKF.cpp
	${CMAKE_SOURCE_DIR}/src/kalman_filter.cpp
	${CMAKE_SOURCE_DIR}/src/tools.h
	${CMAKE_SOURCE_DIR}/src/FusionEKF.h
	${CMAKE_SOURCE_DIR}/src/kalman_filter.h
)

target_link_libraries(core PUBLIC
	Eigen
	json
	libuWS
)