set (excluded_examples "")
list(APPEND excluded_examples "EqVIOFilterRosbagReplay.cpp")

# Add examples to exclude if GTSAM_USE_BOOST_FEATURES is not set
if (NOT GTSAM_USE_BOOST_FEATURES) 
  # add to excluded examples
  list (APPEND excluded_examples
      "GncPoseAveragingExample.cpp"
      )
endif()


gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam_unstable" ${GTSAM_BUILD_EXAMPLES_ALWAYS})

option(GTSAM_UNSTABLE_BUILD_EQVIO_ROSBAG_EXAMPLE
  "Build EqVIO rosbag replay example (requires ROS1 rosbag + cv_bridge)" OFF)

if(GTSAM_UNSTABLE_BUILD_EQVIO_ROSBAG_EXAMPLE)
  find_package(OpenCV REQUIRED)
  find_package(rosbag REQUIRED)
  find_package(cv_bridge REQUIRED)

  add_executable(EqVIOFilterRosbagReplay EqVIOFilterRosbagReplay.cpp)
  target_link_libraries(EqVIOFilterRosbagReplay
    gtsam_unstable
    ${OpenCV_LIBS}
    ${rosbag_LIBRARIES}
    ${cv_bridge_LIBRARIES})
  target_include_directories(EqVIOFilterRosbagReplay PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${rosbag_INCLUDE_DIRS}
    ${cv_bridge_INCLUDE_DIRS})
  list(APPEND examples_list EqVIOFilterRosbagReplay)
endif()
