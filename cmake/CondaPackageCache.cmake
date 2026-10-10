# Only the pixi package build includes this file. Keep the test environment
# and other workflows on their existing compiler-cache configuration.
include("${CMAKE_CURRENT_LIST_DIR}/CondaDefaults.cmake")

# Windows C++ files install under PREFIX/Library, but Python's site-packages
# belongs under PREFIX. The backend supplies the host environment as PREFIX.
if(DEFINED ENV{PREFIX})
  file(TO_CMAKE_PATH "$ENV{PREFIX}" _conda_python_prefix)
  set(GTSAM_PYTHON_INSTALL_PREFIX "${_conda_python_prefix}" CACHE PATH "")
endif()

# The backend sanitizes its environment and sets HOME to its work directory.
# Workflow CCACHE_* / SCCACHE_* variables therefore do not reach the compiler.
# CMAKE_SOURCE_DIR is the checkout, whereas backend SRC_DIR is the work tree.
# Pass the settings on every compile so configure and build use the same cache.
if(CMAKE_HOST_WIN32)
  find_program(_conda_cache_program sccache REQUIRED)
  set(_conda_cache_dir "${CMAKE_SOURCE_DIR}/.sccache-pkg")
  set(_conda_cache_env
    "SCCACHE_DIR=${_conda_cache_dir}"
    "SCCACHE_CACHE_SIZE=1G"
    # A server retains its original environment. Keep package and test
    # servers distinct, including when building locally after running tests.
    "SCCACHE_SERVER_PORT=4227")
  set(_conda_cache_diagnostics --show-stats)
else()
  find_program(_conda_cache_program ccache REQUIRED)
  set(_conda_cache_dir "${CMAKE_SOURCE_DIR}/.ccache-pkg")
  set(_conda_cache_env
    "CCACHE_DIR=${_conda_cache_dir}"
    "CCACHE_BASEDIR=${CMAKE_SOURCE_DIR}"
    "CCACHE_NOHASHDIR=1"
    "CCACHE_MAXSIZE=1G"
    "CCACHE_COMPILERCHECK=content")
  set(_conda_cache_diagnostics --show-config)
endif()

# Use explicit C and C++ launchers, including under MSVC. Disable GTSAM's
# global RULE_LAUNCH_COMPILE ccache hook to avoid wrapping the launcher twice.
set(GTSAM_BUILD_WITH_CCACHE OFF CACHE BOOL "" FORCE)
set(CMAKE_C_COMPILER_LAUNCHER
  "${CMAKE_COMMAND}" -E env ${_conda_cache_env} "${_conda_cache_program}")
set(CMAKE_CXX_COMPILER_LAUNCHER ${CMAKE_C_COMPILER_LAUNCHER})

message(STATUS "Conda package cache directory: ${_conda_cache_dir}")
message(STATUS "Conda package C launcher: ${CMAKE_C_COMPILER_LAUNCHER}")
message(STATUS "Conda package C++ launcher: ${CMAKE_CXX_COMPILER_LAUNCHER}")
execute_process(COMMAND "${_conda_cache_program}" --version)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env ${_conda_cache_env}
    "${_conda_cache_program}" ${_conda_cache_diagnostics}
  RESULT_VARIABLE _conda_cache_diagnostic_result)
if(NOT _conda_cache_diagnostic_result EQUAL 0)
  message(WARNING "Could not report Conda package cache configuration")
endif()

# Temporary measurement snapshot; not part of the proposed PR.
file(GLOB _measure_records "$ENV{PREFIX}/conda-meta/*.json"
  "$ENV{BUILD_PREFIX}/conda-meta/*.json")
file(REMOVE_RECURSE "${CMAKE_SOURCE_DIR}/measurements/package-dependencies")
file(COPY ${_measure_records} DESTINATION
  "${CMAKE_SOURCE_DIR}/measurements/package-dependencies")
