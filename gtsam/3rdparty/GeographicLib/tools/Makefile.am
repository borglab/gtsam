#
# Makefile.am
#
# Copyright (C) 2009, Francesco P. Lovergine <frankie@debian.org>

AM_CPPFLAGS = -I$(top_builddir)/include -I$(top_srcdir)/include \
           -I$(top_builddir)/man -I$(top_srcdir)/man -Wall -Wextra

LDADD = $(top_builddir)/src/libGeographic.la
DEPS = $(top_builddir)/src/libGeographic.la

bin_PROGRAMS = CartConvert \
	ConicProj \
	GeoConvert \
	GeodSolve \
	GeodesicProj \
	GeoidEval \
	Gravity \
	MagneticField \
	Planimeter \
	RhumbSolve \
	TransverseMercatorProj

CartConvert_SOURCES = CartConvert.cpp \
	../man/CartConvert.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Geocentric.hpp \
	../include/GeographicLib/LocalCartesian.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/Utility.hpp
ConicProj_SOURCES = ConicProj.cpp \
	../man/ConicProj.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/AlbersEqualArea.hpp \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/LambertConformalConic.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/Utility.hpp
GeoConvert_SOURCES = GeoConvert.cpp \
	../man/GeoConvert.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/GeoCoords.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/UTMUPS.hpp \
	../include/GeographicLib/Utility.hpp
GeodSolve_SOURCES = GeodSolve.cpp \
	../man/GeodSolve.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Geodesic.hpp \
	../include/GeographicLib/GeodesicExact.hpp \
	../include/GeographicLib/GeodesicLine.hpp \
	../include/GeographicLib/GeodesicLineExact.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/Utility.hpp
GeodesicProj_SOURCES = GeodesicProj.cpp \
	../man/GeodesicProj.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/AzimuthalEquidistant.hpp \
	../include/GeographicLib/CassiniSoldner.hpp \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Geodesic.hpp \
	../include/GeographicLib/GeodesicLine.hpp \
	../include/GeographicLib/Gnomonic.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/Utility.hpp
GeoidEval_SOURCES = GeoidEval.cpp \
	../man/GeoidEval.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/GeoCoords.hpp \
	../include/GeographicLib/Geoid.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/UTMUPS.hpp \
	../include/GeographicLib/Utility.hpp
Gravity_SOURCES = Gravity.cpp \
	../man/Gravity.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/CircularEngine.hpp \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Geocentric.hpp \
	../include/GeographicLib/GravityCircle.hpp \
	../include/GeographicLib/GravityModel.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/NormalGravity.hpp \
	../include/GeographicLib/SphericalEngine.hpp \
	../include/GeographicLib/SphericalHarmonic.hpp \
	../include/GeographicLib/SphericalHarmonic1.hpp \
	../include/GeographicLib/Utility.hpp
MagneticField_SOURCES = MagneticField.cpp \
	../man/MagneticField.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/CircularEngine.hpp \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Geocentric.hpp \
	../include/GeographicLib/MagneticCircle.hpp \
	../include/GeographicLib/MagneticModel.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/SphericalEngine.hpp \
	../include/GeographicLib/SphericalHarmonic.hpp \
	../include/GeographicLib/Utility.hpp
Planimeter_SOURCES = Planimeter.cpp \
	../man/Planimeter.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Accumulator.hpp \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Ellipsoid.hpp \
	../include/GeographicLib/GeoCoords.hpp \
	../include/GeographicLib/Geodesic.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/PolygonArea.hpp \
	../include/GeographicLib/UTMUPS.hpp \
	../include/GeographicLib/Utility.hpp
RhumbSolve_SOURCES = RhumbSolve.cpp \
	../man/RhumbSolve.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/Ellipsoid.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/Utility.hpp
TransverseMercatorProj_SOURCES = TransverseMercatorProj.cpp \
	../man/TransverseMercatorProj.usage \
	../include/GeographicLib/Config.h \
	../include/GeographicLib/Constants.hpp \
	../include/GeographicLib/DMS.hpp \
	../include/GeographicLib/EllipticFunction.hpp \
	../include/GeographicLib/Math.hpp \
	../include/GeographicLib/TransverseMercator.hpp \
	../include/GeographicLib/TransverseMercatorExact.hpp \
	../include/GeographicLib/Utility.hpp

sbin_SCRIPTS = geographiclib-get-geoids \
	geographiclib-get-gravity \
	geographiclib-get-magnetic

geographiclib_data = $(datadir)/GeographicLib
geographiclib-get-geoids: geographiclib-get-geoids.sh
	sed -e "s%@GEOGRAPHICLIB_DATA@%$(geographiclib_data)%" $< > $@
	chmod +x $@
geographiclib-get-gravity: geographiclib-get-gravity.sh
	sed -e "s%@GEOGRAPHICLIB_DATA@%$(geographiclib_data)%" $< > $@
	chmod +x $@
geographiclib-get-magnetic: geographiclib-get-magnetic.sh
	sed -e "s%@GEOGRAPHICLIB_DATA@%$(geographiclib_data)%" $< > $@
	chmod +x $@

CLEANFILES = $(sbin_SCRIPTS)
EXTRA_DIST = Makefile.mk CMakeLists.txt tests.cmake \
	geographiclib-get-geoids.sh geographiclib-get-gravity.sh \
	geographiclib-get-magnetic.sh
