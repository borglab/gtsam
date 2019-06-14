# Generate setup.py.
file(READ "${PROJECT_SOURCE_DIR}/README.md" README_CONTENTS)
configure_file(${PROJECT_SOURCE_DIR}/wrap/setup.py.in
               ${PROJECT_BINARY_DIR}/setup.py)

if(GTSAM_BUILD_PYTHON)
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
              "gtsam.cpp" # generated_cpp
              "gtsam" # module_name
              "gtsam" # top_namespace
              "${ignore}" # ignore_classes
              gtsam # libs
              "gtsam;gtsam_header" # dependencies
              )
  set_target_properties(gtsam_py PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
        OUTPUT_NAME "gtsam"
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/gtsam_py"
        )
  # Symlink all tests .py files to build folder.
  create_symlinks("${CMAKE_CURRENT_SOURCE_DIR}/python/gtsam_py/"
                  "${PROJECT_BINARY_DIR}/gtsam_py")

  # Install.
  install(TARGETS gtsam_py DESTINATION gtsam_py)
  install_python_files("${PROJECT_BINARY_DIR}/setup.py"
                       "${CMAKE_INSTALL_PREFIX}")
  install_python_scripts("${PROJECT_SOURCE_DIR}/wrap/python/gtsam_py/"
                         "${CMAKE_INSTALL_PREFIX}/gtsam_py" "*.py")

endif()
