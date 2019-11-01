include(GtsamPybindWrap)

# Create the python toolbox for the gtsam library
if(GTSAM_INSTALL_PYTHON_TOOLBOX)
  # wrap gtsam add_custom_target(gtsam_header DEPENDS "../gtsam.h")
  set(ignore
      gtsam::Point2
      gtsam::Point3
      gtsam::LieVector
      gtsam::LieMatrix
      gtsam::ISAM2ThresholdMapValue
      gtsam::FactorIndexSet
      gtsam::KeyVector
      gtsam::Point2Vector
      gtsam::Pose3Vector
      gtsam::BetweenFactorPose3s
      gtsam::FactorIndices)

  # wrap gtsam
  add_custom_target(gtsam_header DEPENDS "../gtsam.h")

  pybind_wrapper(
    gtsam_py # target
    ${CMAKE_CURRENT_SOURCE_DIR}/../gtsam.h # interface_header
    "gtsam_py.cpp" # generated_cpp
    "gtsam_py" # module_name
    "" # top_namespace ignore_classes
    "${ignore}"
    gtsam # libs
    "gtsam;gtsam_header") # dependencies
endif()
