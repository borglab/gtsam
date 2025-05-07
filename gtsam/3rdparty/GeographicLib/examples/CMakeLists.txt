# This CMakeLists.txt is invoked in two different ways

# (1) With "add_subdirectory (examples)" from GeographicLib's top-level
# CMakeLists.txt.  This mode of invocation is flagged by the variable
#
#   CALLED_FROM_TOPLEVEL
#
# In this case, the only action taken is to install the examples and
# this CMakeLists.txt in ${EXAMPLEDIR}.

# (2) As an independent invocation of
#
#   cmake -S <this-directory> -B <build-directory>
#
# In this case, find_package (GeographicLib) is called and the examples
# are compiled.  This mode of invocation is triggered by the
# exampleprograms target in the top-level CMakeLists.txt.  In this case,
# the current version of GeographicLib is found by specifying
#
#   -D GeographicLib_DIR=${PROJECT_BINARY_DIR}

cmake_minimum_required (VERSION 3.17.0)

set (EXAMPLES0
  example-Accumulator.cpp
  example-AlbersEqualArea.cpp
  example-AuxAngle.cpp
  example-AuxLatitude.cpp
  example-AzimuthalEquidistant.cpp
  example-CassiniSoldner.cpp
  example-CircularEngine.cpp
  example-Constants.cpp
  example-DMS.cpp
  example-DST.cpp
  example-Ellipsoid.cpp
  example-EllipticFunction.cpp
  example-GARS.cpp
  example-GeoCoords.cpp
  example-Geocentric.cpp
  example-Geodesic.cpp
  example-Geodesic-small.cpp
  example-GeodesicExact.cpp
  example-GeodesicLine.cpp
  example-GeodesicLineExact.cpp
  example-GeographicErr.cpp
  example-Geohash.cpp
  example-Geoid.cpp
  example-Georef.cpp
  example-Gnomonic.cpp
  example-GravityCircle.cpp
  example-GravityModel.cpp
  example-Intersect.cpp
  example-LambertConformalConic.cpp
  example-LocalCartesian.cpp
  example-MGRS.cpp
  example-MagneticCircle.cpp
  example-MagneticModel.cpp
  example-Math.cpp
  example-NearestNeighbor.cpp
  example-NormalGravity.cpp
  example-OSGB.cpp
  example-PolarStereographic.cpp
  example-PolygonArea.cpp
  example-Rhumb.cpp
  example-RhumbLine.cpp
  example-SphericalEngine.cpp
  example-SphericalHarmonic.cpp
  example-SphericalHarmonic1.cpp
  example-SphericalHarmonic2.cpp
  example-TransverseMercator.cpp
  example-TransverseMercatorExact.cpp
  example-UTMUPS.cpp
  example-Utility.cpp
  )
set (EXAMPLES1
  GeoidToGTX.cpp make-egmcof.cpp)

if (CALLED_FROM_TOPLEVEL)
  if (EXAMPLEDIR)
    install (FILES CMakeLists.txt ${EXAMPLES0} ${EXAMPLES1} ${EXAMPLEHEADERS}
      DESTINATION ${EXAMPLEDIR})
  endif ()
  # No more to do in add_subdirectory mode, so exit
  return ()
endif ()

project (GeographicLib-examples)

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # For multi-config systems and for Visual Studio, the debug version of
  # the library is called Geographic_d.
  set (CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "The suffix for debug objects")
else ()
  set (CMAKE_DEBUG_POSTFIX "" CACHE STRING "The suffix for debug objects")
endif ()

find_package (GeographicLib 2.5 REQUIRED)
include_directories (${GeographicLib_INCLUDE_DIRS})

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)

option (USE_BOOST_FOR_EXAMPLES
  "Look for Boost library when compiling examples" ON)

if (USE_BOOST_FOR_EXAMPLES)
  # quad precision numbers appeared in Boost 1.54.  Various
  # workarounds stopped being needed with Boost 1.64.
  find_package (Boost 1.64 COMPONENTS serialization)
elseif (GEOGRAPHICLIB_PRECISION EQUAL 4)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    find_package (Boost 1.64)
  endif ()
endif ()

# Compile a bunch of tiny example programs.  These are built with the
# "exampleprograms" target.  These are mainly for including as examples
# within the doxygen documentation; however, compiling them catches some
# obvious blunders.

if (NOT GEOGRAPHICLIB_PRECISION OR GEOGRAPHICLIB_PRECISION EQUAL 2)
  # These examples all assume real = double, so check
  # GEOGRAPHICLIB_PRECISION.  Allow GEOGRAPHICLIB_PRECISION to be unset
  # to accommodate lame FindGeographicLib.cmake.
  set (EXAMPLE_SOURCES ${EXAMPLES0})
  if (USE_BOOST_FOR_EXAMPLES AND Boost_FOUND)
    add_definitions (-DGEOGRAPHICLIB_HAVE_BOOST_SERIALIZATION=1)
    include_directories ("${Boost_INCLUDE_DIRS}")
  endif ()
else ()
  set (EXAMPLE_SOURCES)
endif ()
set (EXAMPLE_SOURCES ${EXAMPLE_SOURCES} ${EXAMPLES1})

set (EXAMPLES)
foreach (EXAMPLE_SOURCE ${EXAMPLE_SOURCES})
  get_filename_component (EXAMPLE ${EXAMPLE_SOURCE} NAME_WE)
  set (EXAMPLES ${EXAMPLES} ${EXAMPLE})
  add_executable (${EXAMPLE} ${EXAMPLE_SOURCE})
  target_link_libraries (${EXAMPLE}
    ${GeographicLib_LIBRARIES} ${GeographicLib_HIGHPREC_LIBRARIES})
endforeach ()
if (Boost_FOUND AND GEOGRAPHICLIB_PRECISION EQUAL 2)
  target_link_libraries (example-NearestNeighbor ${Boost_LIBRARIES})
endif ()

find_package (OpenMP QUIET)

if (OPENMP_FOUND OR OpenMP_FOUND)
  set_target_properties (GeoidToGTX PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS})
  if (OpenMP_CXX_INCLUDE_DIRS)
    set_target_properties (GeoidToGTX PROPERTIES
      INCLUDE_DIRECTORIES ${OpenMP_CXX_INCLUDE_DIRS})
  endif ()
  if (NOT WIN32)
    set_target_properties (GeoidToGTX PROPERTIES
      LINK_FLAGS ${OpenMP_CXX_FLAGS})
  endif ()
  target_link_libraries (GeoidToGTX ${OpenMP_CXX_LIBRARIES})
endif ()

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # Add _d suffix for your debug versions of the tools
  set_target_properties (${EXAMPLES} PROPERTIES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif ()
