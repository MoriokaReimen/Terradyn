cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_COMPILER "clang++")
set(CMAKE_C_COMPILER "clang")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "lib")

project(Terradyn)
include_directories(include)
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -fPIC")

set(SRC
src/angle.cpp
src/intdiff.cpp
src/terradyn.cpp
src/wheel_soil.cpp
)

add_library(terradyn  STATIC ${SRC}
)
