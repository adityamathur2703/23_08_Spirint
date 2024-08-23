cmake_minimum_required(VERSION 3.10)
project(Testing)
set(CXX_STANDARD_REQUIRED true)
set(CXX_STANDARD 17)


enable_testing()

add_executable(Test Main.cpp)

find_package(GTest REQUIRED)
include_directories(${GTETS_INCLUDE_DIRS})

target_link_libraries(Test ${GTEST_LIBRARIES})