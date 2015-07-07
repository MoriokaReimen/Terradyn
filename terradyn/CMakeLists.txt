cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")

project(Terradyn)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -fPIC")

set(SRC
src/angle.cpp
src/gsl.cpp
src/terradyn.c
src/wheel_soil.cpp
)

add_library(terradyn  STATIC ${SRC})

set(TEST_SRC
test/main.cpp
test/test-gsl.cpp
test/test-wheel_soil.cpp
)
add_executable(test-terradyn ${SRC} ${TEST_SRC})
target_link_libraries(test-terradyn gtest gsl gslcblas pthread)

