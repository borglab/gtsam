###############################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
print_config("CMAKE_CXX_COMPILER_ID type" "${CMAKE_CXX_COMPILER_ID}")
print_config("CMAKE_CXX_COMPILER_VERSION" "${CMAKE_CXX_COMPILER_VERSION}")
print_config("CMake version"    "${CMAKE_VERSION}")
print_config("CMake generator"  "${CMAKE_GENERATOR}")
print_config("CMake build tool" "${CMAKE_BUILD_TOOL}")
message(STATUS "Build flags                                               ")
print_enabled_config(${GTSAM_BUILD_TESTS}                 "Build Tests")
print_enabled_config(${GTSAM_BUILD_EXAMPLES_ALWAYS}       "Build examples with 'make all'")
print_enabled_config(${GTSAM_BUILD_TIMING_ALWAYS}         "Build timing scripts with 'make all'")
if (DOXYGEN_FOUND)
    print_enabled_config(${GTSAM_BUILD_DOCS}              "Build Docs")
endif()
print_enabled_config(${BUILD_SHARED_LIBS}                 "Build shared GTSAM libraries")
print_enabled_config(${GTSAM_BUILD_TYPE_POSTFIXES}        "Put build type in library name")
if(GTSAM_UNSTABLE_AVAILABLE)
    print_enabled_config(${GTSAM_BUILD_UNSTABLE}          "Build libgtsam_unstable        ")
    print_enabled_config(${GTSAM_UNSTABLE_BUILD_PYTHON}   "Build GTSAM unstable Python    ")
    print_enabled_config(${GTSAM_UNSTABLE_INSTALL_MATLAB_TOOLBOX} "Build MATLAB Toolbox for unstable")
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
    print_enabled_config(${GTSAM_BUILD_WITH_MARCH_NATIVE}     "Build for native architecture  ")
    print_config("Build type" "${CMAKE_BUILD_TYPE}")
    print_config("C compilation flags" "${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
    print_config("C++ compilation flags" "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
endif()

print_build_options_for_target(gtsam)

print_config("Use System Eigen" "${GTSAM_USE_SYSTEM_EIGEN} (Using version: ${GTSAM_EIGEN_VERSION})")
print_config("Use System Metis" "${GTSAM_USE_SYSTEM_METIS}")
print_config("Using Boost version" "${Boost_VERSION}")

if(GTSAM_USE_TBB)
    print_config("Use Intel TBB" "Yes (Version: ${TBB_VERSION})")
elseif(TBB_FOUND)
    print_config("Use Intel TBB" "TBB (Version: ${TBB_VERSION}) found but GTSAM_WITH_TBB is disabled")
else()
    print_config("Use Intel TBB" "TBB not found")
endif()
if(GTSAM_USE_EIGEN_MKL)
    print_config("Eigen will use MKL" "Yes")
elseif(MKL_FOUND)
    print_config("Eigen will use MKL" "MKL found but GTSAM_WITH_EIGEN_MKL is disabled")
else()
    print_config("Eigen will use MKL" "MKL not found")
endif()
if(GTSAM_USE_EIGEN_MKL_OPENMP)
    print_config("Eigen will use MKL and OpenMP" "Yes")
elseif(OPENMP_FOUND AND NOT GTSAM_WITH_EIGEN_MKL)
    print_config("Eigen will use MKL and OpenMP" "OpenMP found but GTSAM_WITH_EIGEN_MKL is disabled")
elseif(OPENMP_FOUND AND NOT MKL_FOUND)
    print_config("Eigen will use MKL and OpenMP" "OpenMP found but MKL not found")
elseif(OPENMP_FOUND)
    print_config("Eigen will use MKL and OpenMP" "OpenMP found but GTSAM_WITH_EIGEN_MKL_OPENMP is disabled")
else()
    print_config("Eigen will use MKL and OpenMP" "OpenMP not found")
endif()
if(GTSAM_USE_SUITESPARSE)
    print_config("GTSAM will use SuiteSparse" "Yes")
elseif(SUITESPARSE_FOUND)
    print_config("GTSAM will use SuiteSparse" "SuiteSparse found but GTSAM_WITH_SUITESPARSE is disabled")
else()
    print_config("GTSAM will use SuiteSparse" "SuiteSparse not found")
endif()
if(GTSAM_USE_CUSPARSE)
    print_config("GTSAM will use cuSparse" "Yes")
elseif(CUDAToolkit_FOUND)
    print_config("GTSAM will use cuSparse" "cuSparse found but GTSAM_WITH_CUSPARSE is disabled")
else()
    print_config("GTSAM will use cuSparse" "cuSparse not found")
endif()
print_config("Default allocator" "${GTSAM_DEFAULT_ALLOCATOR}")

if(GTSAM_THROW_CHEIRALITY_EXCEPTION)
    print_config("Cheirality exceptions enabled" "YES")
else()
    print_config("Cheirality exceptions enabled" "NO")
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
    if(CCACHE_FOUND AND GTSAM_BUILD_WITH_CCACHE)
        print_config("Build with ccache" "Yes")
    elseif(CCACHE_FOUND)
        print_config("Build with ccache" "ccache found but GTSAM_BUILD_WITH_CCACHE is disabled")
    else()
        print_config("Build with ccache" "No")
    endif()
endif()

message(STATUS "Packaging flags")
print_config("CPack Source Generator" "${CPACK_SOURCE_GENERATOR}")
print_config("CPack Generator" "${CPACK_GENERATOR}")

message(STATUS "GTSAM flags                                               ")
print_enabled_config(${GTSAM_USE_QUATERNIONS}             "Quaternions as default Rot3     ")
print_enabled_config(${GTSAM_ENABLE_CONSISTENCY_CHECKS}   "Runtime consistency checking    ")
print_enabled_config(${GTSAM_ROT3_EXPMAP}                 "Rot3 retract is full ExpMap     ")
print_enabled_config(${GTSAM_POSE3_EXPMAP}                "Pose3 retract is full ExpMap    ")
print_enabled_config(${GTSAM_ALLOW_DEPRECATED_SINCE_V42}  "Allow features deprecated in GTSAM 4.1")
print_enabled_config(${GTSAM_SUPPORT_NESTED_DISSECTION}   "Metis-based Nested Dissection   ")
print_enabled_config(${GTSAM_TANGENT_PREINTEGRATION}      "Use tangent-space preintegration")

message(STATUS "MATLAB toolbox flags")
print_enabled_config(${GTSAM_INSTALL_MATLAB_TOOLBOX}      "Install MATLAB toolbox          ")
if (${GTSAM_INSTALL_MATLAB_TOOLBOX})
    print_config("MATLAB root" "${MATLAB_ROOT}")
    print_config("MEX binary" "${MEX_COMMAND}")
endif()
message(STATUS "Python toolbox flags                                      ")
print_enabled_config(${GTSAM_BUILD_PYTHON}                "Build Python module with pybind ")
if(GTSAM_BUILD_PYTHON)
    print_config("Python version" ${GTSAM_PYTHON_VERSION})
endif()

message(STATUS "===============================================================")
