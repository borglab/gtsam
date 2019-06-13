#
# Makefile.am
#
# Copyright (C) 2009, Francesco P. Lovergine <frankie@debian.org>

nobase_include_HEADERS = GeographicLib/Accumulator.hpp \
			GeographicLib/AlbersEqualArea.hpp \
			GeographicLib/AzimuthalEquidistant.hpp \
			GeographicLib/CassiniSoldner.hpp \
			GeographicLib/CircularEngine.hpp \
			GeographicLib/Constants.hpp \
			GeographicLib/DMS.hpp \
			GeographicLib/Ellipsoid.hpp \
			GeographicLib/EllipticFunction.hpp \
			GeographicLib/GARS.hpp \
			GeographicLib/GeoCoords.hpp \
			GeographicLib/Geocentric.hpp \
			GeographicLib/Geodesic.hpp \
			GeographicLib/GeodesicExact.hpp \
			GeographicLib/GeodesicLine.hpp \
			GeographicLib/GeodesicLineExact.hpp \
			GeographicLib/Geohash.hpp \
			GeographicLib/Geoid.hpp \
			GeographicLib/Georef.hpp \
			GeographicLib/Gnomonic.hpp \
			GeographicLib/GravityCircle.hpp \
			GeographicLib/GravityModel.hpp \
			GeographicLib/LambertConformalConic.hpp \
			GeographicLib/LocalCartesian.hpp \
			GeographicLib/MGRS.hpp \
			GeographicLib/MagneticCircle.hpp \
			GeographicLib/MagneticModel.hpp \
			GeographicLib/Math.hpp \
			GeographicLib/NearestNeighbor.hpp \
			GeographicLib/NormalGravity.hpp \
			GeographicLib/OSGB.hpp \
			GeographicLib/PolarStereographic.hpp \
			GeographicLib/PolygonArea.hpp \
			GeographicLib/Rhumb.hpp \
			GeographicLib/SphericalEngine.hpp \
			GeographicLib/SphericalHarmonic.hpp \
			GeographicLib/SphericalHarmonic1.hpp \
			GeographicLib/SphericalHarmonic2.hpp \
			GeographicLib/TransverseMercator.hpp \
			GeographicLib/TransverseMercatorExact.hpp \
			GeographicLib/UTMUPS.hpp \
			GeographicLib/Utility.hpp \
			GeographicLib/Config.h

geographiclib_data=$(datadir)/GeographicLib

DEFS=-DGEOGRAPHICLIB_DATA=\"$(geographiclib_data)\" @DEFS@

EXTRA_DIST = Makefile.mk GeographicLib/CMakeLists.txt GeographicLib/Config.h.in
