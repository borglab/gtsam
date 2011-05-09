CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

set(lib_target "gtsam")

add_subdirectory(base)
add_subdirectory(geometry)
add_subdirectory(inference)
add_subdirectory(linear)
add_subdirectory(nonlinear)
add_subdirectory(slam)

#add_library(${lib_target} base geometry inference linear nonlinear slam)
