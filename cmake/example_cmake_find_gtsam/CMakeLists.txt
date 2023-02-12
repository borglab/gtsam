# This file shows how to build and link a user project against GTSAM using CMake
###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 3.4)
project(example CXX)

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)

add_executable(example
  main.cpp
)

# By using CMake exported targets, a simple "link" dependency introduces the
# include directories (-I) flags, links against Boost, and add any other
# required build flags (e.g. C++11, etc.)
target_link_libraries(example PRIVATE gtsam)
