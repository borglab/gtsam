# Set the wrapping script variable
set(MATLAB_WRAP_SCRIPT "${GTSAM_SOURCE_DIR}/wrap/scripts/matlab_wrap.py")

# Set up cache options
option(GTSAM_MEX_BUILD_STATIC_MODULE "Build MATLAB wrapper statically (increases build time)" OFF)
set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(GTSAM_TOOLBOX_INSTALL_PATH "" CACHE PATH "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gtsam_toolbox")
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

if(NOT BUILD_SHARED_LIBS)
    message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX and BUILD_SHARED_LIBS=OFF. The MATLAB wrapper cannot be compiled with a static GTSAM library because mex modules are themselves shared libraries.  If you want a self-contained mex module, enable GTSAM_MEX_BUILD_STATIC_MODULE instead of BUILD_SHARED_LIBS=OFF.")
endif()
