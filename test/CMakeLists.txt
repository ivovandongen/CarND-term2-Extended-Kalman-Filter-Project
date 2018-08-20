include(${CMAKE_SOURCE_DIR}/deps/googletest.cmake)

set(TEST_DIR ${CMAKE_SOURCE_DIR}/test)

add_executable(utest
	${TEST_DIR}/test.hpp
    ${TEST_DIR}/main.cpp
	${TEST_DIR}/kalman_filter.test.cpp
	${TEST_DIR}/tools.test.cpp
)

target_include_directories(utest PRIVATE ${TEST_DIR})

target_link_libraries(utest PUBLIC gtest core)
