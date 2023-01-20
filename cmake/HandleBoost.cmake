###############################################################################
# Find boost

# To change the path for boost, you will need to set:
# BOOST_ROOT: path to install prefix for boost
# Boost_NO_SYSTEM_PATHS: set to true to keep the find script from ignoring BOOST_ROOT

if(MSVC)
    # By default, boost only builds static libraries on windows
    set(Boost_USE_STATIC_LIBS ON)  # only find static libs
    # If we ever reset above on windows and, ...
    # If we use Boost shared libs, disable auto linking.
    # Some libraries, at least Boost Program Options, rely on this to export DLL symbols.
    if(NOT Boost_USE_STATIC_LIBS)
        list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC BOOST_ALL_NO_LIB BOOST_ALL_DYN_LINK)
    endif()
    # Virtual memory range for PCH exceeded on VS2015
    if(MSVC_VERSION LESS 1910) # older than VS2017
      list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Zm295)
    endif()
endif()


# Store these in variables so they are automatically replicated in GTSAMConfig.cmake and such.
set(BOOST_FIND_MINIMUM_VERSION 1.65)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS})

# Required components
if(NOT Boost_SERIALIZATION_LIBRARY OR NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY OR
    NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY)
  message(FATAL_ERROR "Missing required Boost components >= v1.65, please install/upgrade Boost or configure your search paths.")
endif()

option(GTSAM_DISABLE_NEW_TIMERS "Disables using Boost.chrono for timing" OFF)
# Allow for not using the timer libraries on boost < 1.48 (GTSAM timing code falls back to old timer library)
set(GTSAM_BOOST_LIBRARIES
  Boost::serialization
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::date_time
  Boost::regex
)
if (GTSAM_DISABLE_NEW_TIMERS)
    message("WARNING:  GTSAM timing instrumentation manually disabled")
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC DGTSAM_DISABLE_NEW_TIMERS)
else()
    if(Boost_TIMER_LIBRARY)
      list(APPEND GTSAM_BOOST_LIBRARIES Boost::timer Boost::chrono)
    else()
      list(APPEND GTSAM_BOOST_LIBRARIES rt) # When using the header-only boost timer library, need -lrt
      message("WARNING:  GTSAM timing instrumentation will use the older, less accurate, Boost timer library because boost older than 1.48 was found.")
    endif()
endif()
