# Build the library...

add_custom_target (libs ALL)

# Include all the .cpp files in the library.
set (SOURCES
  Accumulator.cpp
  AlbersEqualArea.cpp
  AuxAngle.cpp
  AuxLatitude.cpp
  AzimuthalEquidistant.cpp
  CassiniSoldner.cpp
  CircularEngine.cpp
  DAuxLatitude.cpp
  DMS.cpp
  DST.cpp
  Ellipsoid.cpp
  EllipticFunction.cpp
  GARS.cpp
  GeoCoords.cpp
  Geocentric.cpp
  Geodesic.cpp
  GeodesicExact.cpp
  GeodesicLine.cpp
  GeodesicLineExact.cpp
  Geohash.cpp
  Geoid.cpp
  Georef.cpp
  Gnomonic.cpp
  GravityCircle.cpp
  GravityModel.cpp
  Intersect.cpp
  LambertConformalConic.cpp
  LocalCartesian.cpp
  MGRS.cpp
  MagneticCircle.cpp
  MagneticModel.cpp
  Math.cpp
  NormalGravity.cpp
  OSGB.cpp
  PolarStereographic.cpp
  PolygonArea.cpp
  Rhumb.cpp
  SphericalEngine.cpp
  TransverseMercator.cpp
  TransverseMercatorExact.cpp
  UTMUPS.cpp
  Utility.cpp
  )

set (HEADERS
  kissfft.hh
  ${PROJECT_BINARY_DIR}/include/GeographicLib/Config.h
  ../include/GeographicLib/Accumulator.hpp
  ../include/GeographicLib/AlbersEqualArea.hpp
  ../include/GeographicLib/AzimuthalEquidistant.hpp
  ../include/GeographicLib/CassiniSoldner.hpp
  ../include/GeographicLib/CircularEngine.hpp
  ../include/GeographicLib/Constants.hpp
  ../include/GeographicLib/DMS.hpp
  ../include/GeographicLib/Ellipsoid.hpp
  ../include/GeographicLib/EllipticFunction.hpp
  ../include/GeographicLib/GARS.hpp
  ../include/GeographicLib/GeoCoords.hpp
  ../include/GeographicLib/Geocentric.hpp
  ../include/GeographicLib/Geodesic.hpp
  ../include/GeographicLib/GeodesicExact.hpp
  ../include/GeographicLib/GeodesicLine.hpp
  ../include/GeographicLib/GeodesicLineExact.hpp
  ../include/GeographicLib/Geohash.hpp
  ../include/GeographicLib/Geoid.hpp
  ../include/GeographicLib/Georef.hpp
  ../include/GeographicLib/Gnomonic.hpp
  ../include/GeographicLib/GravityCircle.hpp
  ../include/GeographicLib/GravityModel.hpp
  ../include/GeographicLib/LambertConformalConic.hpp
  ../include/GeographicLib/LocalCartesian.hpp
  ../include/GeographicLib/MGRS.hpp
  ../include/GeographicLib/MagneticCircle.hpp
  ../include/GeographicLib/MagneticModel.hpp
  ../include/GeographicLib/Math.hpp
  ../include/GeographicLib/NearestNeighbor.hpp
  ../include/GeographicLib/NormalGravity.hpp
  ../include/GeographicLib/OSGB.hpp
  ../include/GeographicLib/PolarStereographic.hpp
  ../include/GeographicLib/PolygonArea.hpp
  ../include/GeographicLib/Rhumb.hpp
  ../include/GeographicLib/SphericalEngine.hpp
  ../include/GeographicLib/SphericalHarmonic.hpp
  ../include/GeographicLib/SphericalHarmonic1.hpp
  ../include/GeographicLib/SphericalHarmonic2.hpp
  ../include/GeographicLib/TransverseMercator.hpp
  ../include/GeographicLib/TransverseMercatorExact.hpp
  ../include/GeographicLib/UTMUPS.hpp
  ../include/GeographicLib/Utility.hpp
  )

# Define the library and specify whether it is shared or not.
if (GEOGRAPHICLIB_SHARED_LIB)
  add_library (${PROJECT_SHARED_LIBRARIES} SHARED ${SOURCES} ${HEADERS})
  add_library (${PROJECT_NAME}::${PROJECT_SHARED_LIBRARIES}
    ALIAS ${PROJECT_SHARED_LIBRARIES})
  add_dependencies (libs ${PROJECT_SHARED_LIBRARIES})
endif ()
if (GEOGRAPHICLIB_STATIC_LIB)
  add_library (${PROJECT_STATIC_LIBRARIES} STATIC ${SOURCES} ${HEADERS})
  add_library (${PROJECT_NAME}::${PROJECT_STATIC_LIBRARIES}
    ALIAS ${PROJECT_STATIC_LIBRARIES})
  add_dependencies (libs ${PROJECT_STATIC_LIBRARIES})
endif ()

add_library (${PROJECT_INTERFACE_LIBRARIES} INTERFACE)
add_library (${PROJECT_NAME}::${PROJECT_INTERFACE_LIBRARIES}
  ALIAS ${PROJECT_INTERFACE_LIBRARIES})
target_link_libraries (${PROJECT_INTERFACE_LIBRARIES}
  INTERFACE ${PROJECT_LIBRARIES})

# Set the version number on the library
if (MSVC)
  if (GEOGRAPHICLIB_SHARED_LIB)
    set_target_properties (${PROJECT_SHARED_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME}
      IMPORT_SUFFIX -i.lib)
    target_compile_definitions (${PROJECT_SHARED_LIBRARIES}
      PUBLIC GEOGRAPHICLIB_SHARED_LIB=1)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    set_target_properties (${PROJECT_STATIC_LIBRARIES} PROPERTIES
      VERSION "${LIBVERSION_BUILD}" OUTPUT_NAME ${LIBNAME})
    target_compile_definitions (${PROJECT_STATIC_LIBRARIES}
      PUBLIC GEOGRAPHICLIB_SHARED_LIB=0)
  endif ()
else ()
  set_target_properties (
    ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES} PROPERTIES
    VERSION "${LIBVERSION_BUILD}" SOVERSION "${LIBVERSION_API}"
    OUTPUT_NAME ${LIBNAME})
  if (APPLE AND GEOGRAPHICLIB_PRECISION EQUAL 5)
    if (GEOGRAPHICLIB_SHARED_LIB)
      target_link_libraries (${PROJECT_SHARED_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
    if (GEOGRAPHICLIB_STATIC_LIB)
      target_link_libraries (${PROJECT_STATIC_LIBRARIES} ${HIGHPREC_LIBRARIES})
    endif ()
  endif ()
endif ()

if (GEOGRAPHICLIB_SHARED_LIB)
  target_include_directories (${PROJECT_SHARED_LIBRARIES} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCDIR}>)
endif ()
if (GEOGRAPHICLIB_STATIC_LIB)
  target_include_directories (${PROJECT_STATIC_LIBRARIES} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCDIR}>)
endif ()

# Specify where the library is installed, adding it to the export targets
if (LIBDIR)
  install (TARGETS ${PROJECT_ALL_LIBRARIES}
    EXPORT targets
    # A potentially useful option.  However it's only supported in recent
    # versions of cmake (2.8.12 and later?).  So comment out for now.
    # INCLUDES DESTINATION include
    RUNTIME DESTINATION ${DLLDIR}
    LIBRARY DESTINATION ${LIBDIR}
    ARCHIVE DESTINATION ${LIBDIR})
endif ()

if (MSVC AND PACKAGE_DEBUG_LIBS AND LIBDIR)
  if (GEOGRAPHICLIB_SHARED_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}-i.lib"
      DESTINATION ${LIBDIR} CONFIGURATIONS Release)
    install (PROGRAMS
      "${PROJECT_BINARY_DIR}/bin/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.dll"
      DESTINATION ${DLLDIR} CONFIGURATIONS Release)
  endif ()
  if (GEOGRAPHICLIB_STATIC_LIB)
    install (FILES
      "${PROJECT_BINARY_DIR}/lib/Debug/${LIBNAME}${CMAKE_DEBUG_POSTFIX}.lib"
      DESTINATION ${LIBDIR} CONFIGURATIONS Release)
  endif ()
endif ()

if (MSVC AND GEOGRAPHICLIB_SHARED_LIB)
  install (FILES $<TARGET_PDB_FILE:${PROJECT_SHARED_LIBRARIES}>
    DESTINATION bin OPTIONAL)
endif ()

# Put the library into a folder in the IDE
set_target_properties (
  ${PROJECT_SHARED_LIBRARIES} ${PROJECT_STATIC_LIBRARIES}
  PROPERTIES FOLDER library)
