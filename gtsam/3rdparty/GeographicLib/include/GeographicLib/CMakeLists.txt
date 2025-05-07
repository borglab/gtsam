# Install the header files including the generated Config.h (which is in
# the build tree).
set (HEADERS
  Accumulator.hpp
  AlbersEqualArea.hpp
  AuxAngle.hpp
  AuxLatitude.hpp
  AzimuthalEquidistant.hpp
  CassiniSoldner.hpp
  CircularEngine.hpp
  Constants.hpp
  DAuxLatitude.hpp
  DMS.hpp
  DST.hpp
  Ellipsoid.hpp
  EllipticFunction.hpp
  GARS.hpp
  GeoCoords.hpp
  Geocentric.hpp
  Geodesic.hpp
  GeodesicExact.hpp
  GeodesicLine.hpp
  GeodesicLineExact.hpp
  Geohash.hpp
  Geoid.hpp
  Georef.hpp
  Gnomonic.hpp
  GravityCircle.hpp
  GravityModel.hpp
  Intersect.hpp
  LambertConformalConic.hpp
  LocalCartesian.hpp
  MGRS.hpp
  MagneticCircle.hpp
  MagneticModel.hpp
  Math.hpp
  NearestNeighbor.hpp
  NormalGravity.hpp
  OSGB.hpp
  PolarStereographic.hpp
  PolygonArea.hpp
  Rhumb.hpp
  SphericalEngine.hpp
  SphericalHarmonic.hpp
  SphericalHarmonic1.hpp
  SphericalHarmonic2.hpp
  TransverseMercator.hpp
  TransverseMercatorExact.hpp
  UTMUPS.hpp
  Utility.hpp
  )
if (INCDIR)
  install (FILES ${HEADERS} DESTINATION ${INCDIR}/GeographicLib)
  install (FILES ${CMAKE_CURRENT_BINARY_DIR}/Config.h
    DESTINATION ${INCDIR}/GeographicLib)
endif ()
