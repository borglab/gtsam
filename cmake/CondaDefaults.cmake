# Cache defaults for building GTSAM against conda-forge dependencies. Pass with
#   cmake -DCMAKE_PROJECT_TOP_LEVEL_INCLUDES=cmake/CondaDefaults.cmake
# which, unlike `cmake -C`, takes a path relative to the source tree.
#
# Shared by `pixi build` and the pixi `test` environment so the packaged build
# and the tested build cannot drift apart. See pixi.toml.

set(CMAKE_BUILD_TYPE Release CACHE STRING "")

# Binaries have to run on machines other than the builder.
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "")

# Warnings-as-errors is useful upstream, but it would make the packaged build
# hostage to every new compiler version conda-forge ships.
set(GTSAM_BUILD_WITH_WERROR OFF CACHE BOOL "")

# Nothing here needs the examples or the timing scripts.
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "")
set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "")

# Use the conda-forge packages rather than the bundled copies.
set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "")
set(GTSAM_USE_SYSTEM_METIS ON CACHE BOOL "")
set(GTSAM_USE_SYSTEM_PYBIND ON CACHE BOOL "")

# conda-forge ships only shared Boost.
set(Boost_USE_STATIC_LIBS OFF CACHE BOOL "")

# One executable per test file, as on Linux. MSVC and Xcode default to
# combining each group into a single binary, which reports failures as one
# opaque check_<group>_program and makes a crash impossible to attribute to a
# test. It is also unsound: same-named TEST(group, name) pairs in different
# files collide at link time.
set(GTSAM_SINGLE_TEST_EXE OFF CACHE BOOL "")

set(GTSAM_BUILD_PYTHON ON CACHE BOOL "")
set(GTSAM_INSTALL_CPPUNITLITE OFF CACHE BOOL "")
