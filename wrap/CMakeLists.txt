if(GTSAM_INSTALL_PYTHON_TOOLBOX)
  add_subdirectory(pybind11)
  include(GtsamPybindWrap)

  add_custom_target(gtsam_header DEPENDS "../gtsam.h")
  set(ignore
      gtsam::Point2
      gtsam::Point3
      gtsam::LieVector
      gtsam::LieMatrix
      gtsam::ISAM2ThresholdMapValue
      gtsam::FactorIndices
      gtsam::FactorIndexSet)
  pybind_wrap(gtsam_py # target
              ${CMAKE_CURRENT_SOURCE_DIR}/../gtsam.h # interface_header
              "gtsam_py.cpp" # generated_cpp
              "gtsam_py" # module_name
              "" # top_namespace ignore_classes
              "${ignore}"
              gtsam # libs
              "gtsam;gtsam_header" # dependencies
              )
  install(TARGETS gtsam_py DESTINATION lib)

  file(READ "${PROJECT_SOURCE_DIR}/README.md" README_CONTENTS)

  configure_file(${PROJECT_SOURCE_DIR}/wrap/setup.py.in
                 ${PROJECT_BINARY_DIR}/wrap/setup.py)
  install_python_files("${PROJECT_BINARY_DIR}/wrap/setup.py"
                       "${CMAKE_INSTALL_PREFIX}/lib")
  # install scripts and tests
  install_python_scripts("${PROJECT_SOURCE_DIR}/wrap/python/tests/gtsam"
                         "${CMAKE_INSTALL_PREFIX}/lib" "*.py")

endif()
