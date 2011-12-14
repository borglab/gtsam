# build 3rd party libraries
add_subdirectory(3rdparty)

# Build full gtsam library as a single library
# and also build tests 
set (gtsam_subdirs 
    base 
    geometry 
    inference 
    linear 
    nonlinear 
    slam)

foreach(subdir ${gtsam_subdirs})
    file(GLOB sub_gtsam_srcs ${subdir} "*.cpp")
    list(APPEND gtsam_srcs sub_gtsam_srcs)
endforeach(subdir)

