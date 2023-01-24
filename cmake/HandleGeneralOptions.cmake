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
option(BUILD_SHARED_LIBS                    "Build shared gtsam library, instead of static" ON)
option(GTSAM_USE_QUATERNIONS                "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices. If enable, Rot3::EXPMAP is enforced by default." OFF)
option(GTSAM_POSE3_EXPMAP                   "Enable/Disable using Pose3::EXPMAP as the default mode. If disabled, Pose3::FIRST_ORDER will be used." ON)
option(GTSAM_ROT3_EXPMAP                    "Ignore if GTSAM_USE_QUATERNIONS is OFF (Rot3::EXPMAP by default). Otherwise, enable Rot3::EXPMAP, or if disabled, use Rot3::CAYLEY." ON)
option(GTSAM_ENABLE_CONSISTENCY_CHECKS      "Enable/Disable expensive consistency checks"       OFF)
option(GTSAM_WITH_TBB                       "Use Intel Threaded Building Blocks (TBB) if available" ON)
option(GTSAM_WITH_EIGEN_MKL                 "Eigen will use Intel MKL if available" OFF)
option(GTSAM_WITH_EIGEN_MKL_OPENMP          "Eigen, when using Intel MKL, will also use OpenMP for multithreading if available" OFF)
option(GTSAM_THROW_CHEIRALITY_EXCEPTION     "Throw exception when a triangulated point is behind a camera" ON)
option(GTSAM_BUILD_PYTHON                   "Enable/Disable building & installation of Python module with pybind11" OFF)
option(GTSAM_INSTALL_MATLAB_TOOLBOX         "Enable/Disable installation of matlab toolbox"  OFF)
option(GTSAM_ALLOW_DEPRECATED_SINCE_V43     "Allow use of methods/functions deprecated in GTSAM 4.3" ON)
option(GTSAM_SUPPORT_NESTED_DISSECTION      "Support Metis-based nested dissection" ON)
option(GTSAM_TANGENT_PREINTEGRATION         "Use new ImuFactor with integration on tangent space" ON)
option(GTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR "Use the slower but correct version of BetweenFactor" OFF)
option(GTSAM_ENABLE_BOOST_SERIALIZATION "Enable Boost serialization" ON)

#TODO(kartikarcot) defining it in config.h.in did not work
if (GTSAM_ENABLE_BOOST_SERIALIZATION)
    add_definitions(-DGTSAM_ENABLE_BOOST_SERIALIZATION)
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
    option(GTSAM_BUILD_WITH_CCACHE           "Use ccache compiler cache" ON)
endif()

# Enable GTSAM_ROT3_EXPMAP if GTSAM_POSE3_EXPMAP is enabled, and vice versa.
if(GTSAM_POSE3_EXPMAP)
    message(STATUS "GTSAM_POSE3_EXPMAP=ON, enabling GTSAM_ROT3_EXPMAP as well")
    set(GTSAM_ROT3_EXPMAP 1 CACHE BOOL "" FORCE)
elseif(GTSAM_ROT3_EXPMAP)
    message(STATUS "GTSAM_ROT3_EXPMAP=ON, enabling GTSAM_POSE3_EXPMAP as well")
    set(GTSAM_POSE3_EXPMAP 1 CACHE BOOL "" FORCE)
endif()

# Set the default Python version. This is later updated in HandlePython.cmake.
set(GTSAM_PYTHON_VERSION "Default" CACHE STRING "The version of Python to build the wrappers against.")
