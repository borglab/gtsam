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
set(BOOST_FIND_MINIMUM_VERSION 1.67)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS})

set(GTSAM_BOOST_LIBRARIES
  Boost::serialization
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::program_options
  Boost::date_time
  Boost::timer
  Boost::chrono
  Boost::regex
)
