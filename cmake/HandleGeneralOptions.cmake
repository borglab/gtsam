###############################################################################
# Set up options

# See whether gtsam_unstable is available (it will be present only if we're using a git checkout)
if(EXISTS "${PROJECT_SOURCE_DIR}/gtsam_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/gtsam_unstable")
    set(GTSAM_UNSTABLE_AVAILABLE 1)
else()
    set(GTSAM_UNSTABLE_AVAILABLE 0)
endif()

### GtsamTesting related options
option(GTSAM_BUILD_EXAMPLES_ALWAYS       "Build examples with 'make all' (build with 'make examples' if not)"       ON)
option(GTSAM_BUILD_TIMING_ALWAYS         "Build timing scripts with 'make all' (build with 'make timing' if not"    OFF)
###

# Add option for using build type postfixes to allow installing multiple build modes
option(GTSAM_BUILD_TYPE_POSTFIXES        "Enable/Disable appending the build type to the name of compiled libraries" ON)

if (NOT MSVC)
    option(GTSAM_BUILD_WITH_MARCH_NATIVE  "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" OFF)
endif()

# Configurable Options
if(GTSAM_UNSTABLE_AVAILABLE)
    option(GTSAM_BUILD_UNSTABLE              "Enable/Disable libgtsam_unstable"          ON)
    option(GTSAM_UNSTABLE_BUILD_PYTHON       "Enable/Disable Python wrapper for libgtsam_unstable" ON)
    option(GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX "Enable/Disable MATLAB wrapper for libgtsam_unstable" OFF)
endif()
option(GTSAM_FORCE_SHARED_LIB               "Force gtsam to be a shared library, overriding BUILD_SHARED_LIBS" ON)
option(GTSAM_FORCE_STATIC_LIB               "Force gtsam to be a static library, overriding BUILD_SHARED_LIBS" OFF)
option(GTSAM_USE_QUATERNIONS                "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices. If enable, Rot3::EXPMAP is enforced by default." OFF)
option(GTSAM_POSE3_EXPMAP                   "Enable/Disable using Pose3::EXPMAP as the default mode. If disabled, Pose3::FIRST_ORDER will be used." ON)
option(GTSAM_ROT3_EXPMAP                    "Ignore if GTSAM_USE_QUATERNIONS is OFF (Rot3::EXPMAP by default). Otherwise, enable Rot3::EXPMAP, or if disabled, use Rot3::CAYLEY." ON)
option(GTSAM_DT_MERGING                     "Enable/Disable merging of equal leaf nodes in DecisionTrees. This leads to significant speed up and memory savings." ON)
option(GTSAM_ENABLE_CONSISTENCY_CHECKS      "Enable/Disable expensive consistency checks" OFF)
option(GTSAM_ENABLE_MEMORY_SANITIZER        "Enable/Disable memory sanitizer" OFF)
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
option(GTSAM_SLOW_BUT_CORRECT_EXPMAP        "Use slower but correct expmap for Pose2"  OFF)

if (GTSAM_FORCE_SHARED_LIB)
    message(STATUS "GTSAM is a shared library due to GTSAM_FORCE_SHARED_LIB")
    set(GTSAM_LIBRARY_TYPE SHARED CACHE STRING "" FORCE)
    set(GTSAM_SHARED_LIB 1 CACHE BOOL "" FORCE)
elseif (GTSAM_FORCE_STATIC_LIB)
    message(STATUS "GTSAM is a static library due to GTSAM_FORCE_STATIC_LIB")
    set(GTSAM_LIBRARY_TYPE STATIC CACHE STRING "" FORCE)
    set(GTSAM_SHARED_LIB 0 CACHE BOOL "" FORCE)
elseif (BUILD_SHARED_LIBS)
    message(STATUS "GTSAM is a shared library due to BUILD_SHARED_LIBS is ON")
    set(GTSAM_LIBRARY_TYPE SHARED CACHE STRING "" FORCE)
    set(GTSAM_SHARED_LIB 1 CACHE BOOL "" FORCE)
elseif((DEFINED BUILD_SHARED_LIBS) AND (NOT BUILD_SHARED_LIBS))
    message(STATUS "GTSAM is a static library due to BUILD_SHARED_LIBS is OFF")
    set(GTSAM_LIBRARY_TYPE STATIC CACHE STRING "" FORCE)
    set(GTSAM_SHARED_LIB 0 CACHE BOOL "" FORCE)
else()
    message(FATAL_ERROR "Please, to unambiguously select the desired library type to use to build GTSAM, set one of GTSAM_FORCE_SHARED_LIB=ON, GTSAM_FORCE_STATIC_LIB=ON, or BUILD_SHARED_LIBS={ON/OFF}")
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
