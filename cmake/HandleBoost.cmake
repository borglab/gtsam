################################################################################
# Find and configure the Boost libraries.
#
# To customize the Boost installation path, you can set the following variables
# when running CMake:
# - BOOST_ROOT: Path to the Boost installation prefix.
# - BOOST_INCLUDEDIR: Path to the Boost include directory.
# - BOOST_LIBRARYDIR: Path to the Boost library directory.
#
# These hints are particularly important for manual installations on Windows.
################################################################################

if(MSVC)
    # By default, Boost pre-compiled binaries for Windows are static libraries.
    set(Boost_USE_STATIC_LIBS ON)
    if(NOT Boost_USE_STATIC_LIBS)
        list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC BOOST_ALL_NO_LIB BOOST_ALL_DYN_LINK)
    endif()
    if(MSVC_VERSION LESS 1910) # older than VS2017
      list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Zm295)
    endif()
endif()

# --- GUIDANCE FOR LOCAL WINDOWS DEVELOPMENT ---
#
# If you enable Boost features on Windows, there are two primary ways to provide
# the Boost libraries:
#
# 1. RECOMMENDED: Use a package manager like vcpkg.
#    - Vcpkg (https://github.com/microsoft/vcpkg) handles the download, build,
#      and integration of Boost. It provides modern CMake config files and works
#      seamlessly with this script without any special configuration.
#
# 2. ALTERNATIVE: Manual Installation (e.g., pre-compiled binaries).
#    - If you download pre-compiled binaries, you MUST provide hints to CMake so
#      it can find your installation. At a minimum, set BOOST_ROOT, e.g.:
#      cmake .. -DBOOST_ROOT=C:/local/boost_1_87_0
#
################################################################################

# Set minimum required Boost version and components.
# Note: Keep this in sync with vcpkg.json.
# optional, program_options, random, range are all used in tests/examples/Python, but are not library dependencies. timer/chrono is used conditionally.
# concept_check, fusion, move, phoenix, pool, smart_ptr, spirit, tokenizer, type_traits, optional, range are header only and are not components.
set(BOOST_FIND_MINIMUM_VERSION 1.70)
set(BOOST_FIND_MINIMUM_COMPONENTS graph serialization program_options random timer chrono)

# Find the Boost package. On systems with modern installations (vcpkg, Homebrew),
# this will use CMake's "Config mode". With manual installations (especially on
# Windows), providing the hints above will trigger the legacy "Module mode".
find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} REQUIRED
             COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS}
             )

set(GTSAM_BOOST_LIBRARIES Boost::graph Boost::serialization)
# Verify that the required Boost component targets were successfully found and imported.
foreach(_t IN ITEMS ${GTSAM_BOOST_LIBRARIES})
  if(NOT TARGET ${_t})
    message(FATAL_ERROR "Missing required Boost component target: ${_t}. Please install/upgrade Boost or set BOOST_ROOT/Boost_DIR correctly.")
  endif()
endforeach()

option(GTSAM_DISABLE_NEW_TIMERS "Disables using Boost.chrono for timing" OFF)

if(GTSAM_DISABLE_NEW_TIMERS)
  message("WARNING:  GTSAM timing instrumentation manually disabled")
  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC DGTSAM_DISABLE_NEW_TIMERS)
else()
  # Link against compiled timer libraries if they exist.
  if(TARGET Boost::timer AND TARGET Boost::chrono)
    list(APPEND GTSAM_BOOST_LIBRARIES Boost::timer Boost::chrono)
  else()
    # Fallback for header-only timer: link librt on Linux.
    if(UNIX AND NOT APPLE)
      list(APPEND GTSAM_BOOST_LIBRARIES rt)
      message("WARNING:  Using header-only Boost timer; adding -lrt on Linux.")
    else()
      message("WARNING:  Using header-only Boost timer; no extra libs required on this platform.")
    endif()
  endif()
endif()