# Install matlab components

# Check if flag is enabled
if(NOT GTSAM_INSTALL_MATLAB_TOOLBOX)
  return()
endif()

# Create the matlab toolbox for the gtsam library

# Set the wrapping script variable
set(MATLAB_WRAP_SCRIPT "${GTSAM_SOURCE_DIR}/wrap/scripts/matlab_wrap.py")

# Set up cache options
option(GTSAM_MEX_BUILD_STATIC_MODULE
       "Build MATLAB wrapper statically (increases build time)" OFF)
set(GTSAM_BUILD_MEX_BINARY_FLAGS
    ""
    CACHE STRING "Extra flags for running Matlab MEX compilation")
set(GTSAM_TOOLBOX_INSTALL_PATH
    ""
    CACHE
      PATH
      "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gtsam_toolbox"
)
if(NOT GTSAM_TOOLBOX_INSTALL_PATH)
  set(GTSAM_TOOLBOX_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/gtsam_toolbox")
endif()

set(WRAP_MEX_BUILD_STATIC_MODULE ${GTSAM_MEX_BUILD_STATIC_MODULE})
set(WRAP_BUILD_MEX_BINARY_FLAGS ${GTSAM_BUILD_MEX_BINARY_FLAGS})
set(WRAP_TOOLBOX_INSTALL_PATH ${GTSAM_TOOLBOX_INSTALL_PATH})
set(WRAP_CUSTOM_MATLAB_PATH ${GTSAM_CUSTOM_MATLAB_PATH})
set(WRAP_BUILD_TYPE_POSTFIXES ${GTSAM_BUILD_TYPE_POSTFIXES})

# Fixup the Python paths
if(GTWRAP_DIR)
  # packaged
  set(GTWRAP_PACKAGE_DIR ${GTWRAP_DIR})
else()
  set(GTWRAP_PACKAGE_DIR ${GTSAM_SOURCE_DIR}/wrap)
endif()

include(MatlabWrap)

if(NOT GTSAM_SHARED_LIB)
  message(
    FATAL_ERROR
      "GTSAM_INSTALL_MATLAB_TOOLBOX and BUILD_SHARED_LIBS=OFF."
      "The MATLAB wrapper cannot be compiled with a static GTSAM library because mex modules are themselves shared libraries."
      "If you want a self-contained mex module, enable GTSAM_MEX_BUILD_STATIC_MODULE instead of BUILD_SHARED_LIBS=OFF."
  )
endif()

# ##############################################################################
# Generate, build and install toolbox
set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
if(NOT GTSAM_SHARED_LIB)
  list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
endif()

# ignoring the non-concrete types (type aliases)
set(ignore
    gtsam::Point2
    gtsam::Point3
    gtsam::CustomFactor)

set(interface_files
    ${GTSAM_SOURCE_DIR}/gtsam/gtsam.i
    ${GTSAM_SOURCE_DIR}/gtsam/base/base.i
    ${GTSAM_SOURCE_DIR}/gtsam/basis/basis.i
    ${PROJECT_SOURCE_DIR}/gtsam/inference/inference.i
    ${PROJECT_SOURCE_DIR}/gtsam/discrete/discrete.i
    ${GTSAM_SOURCE_DIR}/gtsam/geometry/geometry.i
    ${GTSAM_SOURCE_DIR}/gtsam/linear/linear.i
    ${GTSAM_SOURCE_DIR}/gtsam/nonlinear/nonlinear.i
    ${GTSAM_SOURCE_DIR}/gtsam/nonlinear/values.i
    ${GTSAM_SOURCE_DIR}/gtsam/nonlinear/custom.i
    ${GTSAM_SOURCE_DIR}/gtsam/symbolic/symbolic.i
    ${GTSAM_SOURCE_DIR}/gtsam/sam/sam.i
    ${GTSAM_SOURCE_DIR}/gtsam/slam/slam.i
    ${GTSAM_SOURCE_DIR}/gtsam/sfm/sfm.i
    ${GTSAM_SOURCE_DIR}/gtsam/navigation/navigation.i
)
# Wrap
matlab_wrap("${interface_files}" "gtsam" "${GTSAM_ADDITIONAL_LIBRARIES}"
            "" "${mexFlags}" "${ignore}")

# Wrap version for gtsam_unstable
if(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX)
  # Generate, build and install toolbox
  set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
  if(NOT GTSAM_SHARED_LIB)
    list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
  endif()

  # Wrap
  matlab_wrap(${GTSAM_SOURCE_DIR}/gtsam_unstable/gtsam_unstable.i "gtsam_unstable"
              "${GTSAM_ADDITIONAL_LIBRARIES}" "" "${mexFlags}" "${ignore}")
endif(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX)

# Record the root dir for gtsam - needed during external builds, e.g., ROS
set(GTSAM_SOURCE_ROOT_DIR ${GTSAM_SOURCE_DIR})

# Tests message(STATUS "Installing Matlab Toolbox")
install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/" "*.m;*.fig")
install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/"
                       "README-gtsam-toolbox.txt")

# Examples message(STATUS "Installing Matlab Toolbox Examples") Matlab files:
# *.m and *.fig
# install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/gtsam_examples"
# "*.m;*.fig")

# Utilities message(STATUS "Installing Matlab Toolbox Utilities")
# install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/+gtsam" "*.m")

# message(STATUS "Installing Matlab Toolbox Example Data") Data files: *.graph,
# *.mat, and *.txt
file(GLOB matlab_examples_data_graph
     "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.graph")
file(GLOB matlab_examples_data_mat
     "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.mat")
file(GLOB matlab_examples_data_txt
     "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.txt")
set(matlab_examples_data
    ${matlab_examples_data_graph} ${matlab_examples_data_mat}
    ${matlab_examples_data_txt})
if(GTSAM_BUILD_TYPE_POSTFIXES)
  foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${build_type}" build_type_upper)
    if(${build_type_upper} STREQUAL "RELEASE")
      set(build_type_tag "") # Don't create release mode tag on installed
      # directory
    else()
      set(build_type_tag "${build_type}")
    endif()
    # Split up filename to strip trailing '/' in GTSAM_TOOLBOX_INSTALL_PATH if
    # there is one
    get_filename_component(location "${GTSAM_TOOLBOX_INSTALL_PATH}" PATH)
    get_filename_component(name "${GTSAM_TOOLBOX_INSTALL_PATH}" NAME)
    install(
      FILES ${matlab_examples_data}
      DESTINATION "${location}/${name}${build_type_tag}/gtsam_examples/Data"
      CONFIGURATIONS "${build_type}")
  endforeach()
else()
  install(FILES ${matlab_examples_data}
          DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples/Data)
endif()
