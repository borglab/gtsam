# JLBC: These should ideally be ported to "modern cmake" via target properties.
#

if (CMAKE_GENERATOR STREQUAL "Ninja" AND
    ((CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.9) OR
     (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 3.5)))
    # Force colored warnings in Ninja's output, if the compiler has -fdiagnostics-color support.
    # Rationale in https://github.com/ninja-build/ninja/issues/814
    add_compile_options(-fdiagnostics-color=always)
endif()


# If building DLLs in MSVC, we need to avoid EIGEN_STATIC_ASSERT()
# or explicit instantiation will generate build errors.
# See: https://bitbucket.org/gtborg/gtsam/issues/417/fail-to-build-on-msvc-2017
#
if(MSVC AND GTSAM_SHARED_LIB)
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC EIGEN_NO_STATIC_ASSERT)
endif()

if (APPLE AND GTSAM_SHARED_LIB)
    # Set the default install directory on macOS
    set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

###############################################################################
# Global compile options

if(MSVC)
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PRIVATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE /wd4251 /wd4275 /wd4251 /wd4661 /wd4344 /wd4503) # Disable non-DLL-exported base class and other warnings
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE /bigobj) # Allow large object files for template-based code
endif()

# GCC 4.8+ complains about local typedefs which we use for shared_ptr etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Wno-unused-local-typedefs)
  endif()
endif()

# As of XCode 7, clang also complains about this
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Wno-unused-local-typedefs)
  endif()
endif()

if(GTSAM_ENABLE_CONSISTENCY_CHECKS)
  # This should be made PUBLIC if GTSAM_EXTRA_CONSISTENCY_CHECKS is someday used in a public .h
  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PRIVATE GTSAM_EXTRA_CONSISTENCY_CHECKS)
endif()
