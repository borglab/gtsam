###############################################################################
# Set up options

# See whether gtsam_unstable is available (it will be present only if we're using a git checkout)
if(EXISTS "${PROJECT_SOURCE_DIR}/gtsam_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/gtsam_unstable")
    set(GTSAM_UNSTABLE_AVAILABLE 1)
else()
    set(GTSAM_UNSTABLE_AVAILABLE 0)
endif()

# Configurable Options
if(GTSAM_UNSTABLE_AVAILABLE)
    option(GTSAM_BUILD_UNSTABLE              "Enable/Disable libgtsam_unstable"          ON)
    option(GTSAM_UNSTABLE_BUILD_PYTHON       "Enable/Disable Python wrapper for libgtsam_unstable" ON)
    option(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX "Enable/Disable MATLAB wrapper for libgtsam_unstable" OFF)
endif()
option(BUILD_SHARED_LIBS                 "Build shared gtsam library, instead of static" ON)
option(GTSAM_USE_QUATERNIONS             "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices. If enable, Rot3::EXPMAP is enforced by default." OFF)
option(GTSAM_POSE3_EXPMAP                "Enable/Disable using Pose3::EXPMAP as the default mode. If disabled, Pose3::FIRST_ORDER will be used." ON)
option(GTSAM_ROT3_EXPMAP                 "Ignore if GTSAM_USE_QUATERNIONS is OFF (Rot3::EXPMAP by default). Otherwise, enable Rot3::EXPMAP, or if disabled, use Rot3::CAYLEY." ON)
option(GTSAM_ENABLE_CONSISTENCY_CHECKS   "Enable/Disable expensive consistency checks"       OFF)
option(GTSAM_WITH_TBB                    "Use Intel Threaded Building Blocks (TBB) if available" ON)
option(GTSAM_WITH_EIGEN_MKL              "Eigen will use Intel MKL if available" OFF)
option(GTSAM_WITH_EIGEN_MKL_OPENMP       "Eigen, when using Intel MKL, will also use OpenMP for multithreading if available" OFF)
option(GTSAM_THROW_CHEIRALITY_EXCEPTION  "Throw exception when a triangulated point is behind a camera" ON)
option(GTSAM_BUILD_PYTHON                "Enable/Disable building & installation of Python module with pybind11" OFF)
option(GTSAM_ALLOW_DEPRECATED_SINCE_V41  "Allow use of methods/functions deprecated in GTSAM 4.1" ON)
option(GTSAM_SUPPORT_NESTED_DISSECTION   "Support Metis-based nested dissection" ON)
option(GTSAM_TANGENT_PREINTEGRATION      "Use new ImuFactor with integration on tangent space" ON)
if(NOT MSVC AND NOT XCODE_VERSION)
    option(GTSAM_BUILD_WITH_CCACHE           "Use ccache compiler cache" ON)
endif()

# Options relating to MATLAB wrapper
# TODO: Check for matlab mex binary before handling building of binaries
option(GTSAM_INSTALL_MATLAB_TOOLBOX      "Enable/Disable installation of matlab toolbox"  OFF)
set(GTSAM_PYTHON_VERSION "Default" CACHE STRING "The version of Python to build the wrappers against.")

# Check / set dependent variables for MATLAB wrapper
if(GTSAM_INSTALL_MATLAB_TOOLBOX AND GTSAM_BUILD_TYPE_POSTFIXES)
    set(CURRENT_POSTFIX ${CMAKE_${CMAKE_BUILD_TYPE_UPPER}_POSTFIX})
endif()

if(GTSAM_INSTALL_MATLAB_TOOLBOX AND NOT BUILD_SHARED_LIBS)
    message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX and BUILD_SHARED_LIBS=OFF. The MATLAB wrapper cannot be compiled with a static GTSAM library because mex modules are themselves shared libraries.  If you want a self-contained mex module, enable GTSAM_MEX_BUILD_STATIC_MODULE instead of BUILD_SHARED_LIBS=OFF.")
endif()
