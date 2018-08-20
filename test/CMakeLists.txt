include(${CMAKE_SOURCE_DIR}/deps/googletest.cmake)

set(TEST_DIR ${CMAKE_SOURCE_DIR}/test)

add_executable(utest
    ${TEST_DIR}/main.cpp
)

target_link_libraries(utest PUBLIC gtest)