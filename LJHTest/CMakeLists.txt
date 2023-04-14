cmake_minimum_required(VERSION 3.15)
project(InterpolationTest LANGUAGES CXX)

find_package(GTest REQUIRED)
include(GoogleTest)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


add_executable(InterpTest test.cpp)
target_include_directories(InterpTest PUBLIC ./include)
target_link_libraries(InterpTest PUBLIC GTest::gtest)
add_compile_definitions(_DISABLE_EXTENDED_ALIGNED_STORAGE)
gtest_discover_tests(InterpTest)