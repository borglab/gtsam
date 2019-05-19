LIBSTEM = Geographic
LIBRARY = lib$(LIBSTEM).a

all: $(LIBRARY)

INCLUDEPATH = ../include

PREFIX = /usr/local
GEOGRAPHICLIB_DATA = $(PREFIX)/share/GeographicLib

MODULES = Accumulator \
	AlbersEqualArea \
	AzimuthalEquidistant \
	CassiniSoldner \
	CircularEngine \
	DMS \
	Ellipsoid \
	EllipticFunction \
	GARS \
	GeoCoords \
	Geocentric \
	Geodesic \
	GeodesicExact \
	GeodesicLine \
	GeodesicLineExact \
	Geohash \
	Geoid \
	Georef \
	Gnomonic \
	GravityCircle \
	GravityModel \
	LambertConformalConic \
	LocalCartesian \
	MGRS \
	MagneticCircle \
	MagneticModel \
	Math \
	NormalGravity \
	OSGB \
	PolarStereographic \
	PolygonArea \
	Rhumb \
	SphericalEngine \
	TransverseMercator \
	TransverseMercatorExact \
	UTMUPS \
	Utility
EXTRAHEADERS = Constants \
	NearestNeighbor \
	SphericalHarmonic \
	SphericalHarmonic1 \
	SphericalHarmonic2
EXTRASOURCES = GeodesicExactC4
HEADERS = Config.h $(addsuffix .hpp,$(EXTRAHEADERS) $(MODULES))
SOURCES = $(addsuffix .cpp,$(MODULES) $(EXTRASOURCES))
OBJECTS = $(addsuffix .o,$(MODULES) $(EXTRASOURCES))

CC = g++ -g
CXXFLAGS = -g -Wall -Wextra -O3 -std=c++0x

CPPFLAGS = -I$(INCLUDEPATH) $(DEFINES) \
	-DGEOGRAPHICLIB_DATA=\"$(GEOGRAPHICLIB_DATA)\"
LDFLAGS = $(LIBRARY)

$(LIBRARY): $(OBJECTS)
	$(AR) r $@ $?

VPATH = ../include/GeographicLib

INSTALL = install -b

install: $(LIBRARY)
	test -f $(PREFIX)/lib || mkdir -p $(PREFIX)/lib
	$(INSTALL) -m 644 $^ $(PREFIX)/lib

clean:
	rm -f *.o $(LIBRARY)

TAGS: $(HEADERS) $(SOURCES)
	etags $^

Accumulator.o: Accumulator.hpp Config.h Constants.hpp Math.hpp
AlbersEqualArea.o: AlbersEqualArea.hpp Config.h Constants.hpp Math.hpp
AzimuthalEquidistant.o: AzimuthalEquidistant.hpp Config.h Constants.hpp \
	Geodesic.hpp Math.hpp
CassiniSoldner.o: CassiniSoldner.hpp Config.h Constants.hpp Geodesic.hpp \
	GeodesicLine.hpp Math.hpp
CircularEngine.o: CircularEngine.hpp Config.h Constants.hpp Math.hpp \
	SphericalEngine.hpp
DMS.o: Config.h Constants.hpp DMS.hpp Math.hpp Utility.hpp
Ellipsoid.o: Config.h Constants.hpp Ellipsoid.hpp AlbersEqualArea.hpp \
	EllipticFunction.hpp Math.hpp TransverseMercator.hpp
EllipticFunction.o: Config.h Constants.hpp EllipticFunction.hpp Math.hpp
GARS.o: Config.h Constants.hpp GARS.hpp Utility.hpp
GeoCoords.o: Config.h Constants.hpp DMS.hpp GeoCoords.hpp MGRS.hpp Math.hpp \
	UTMUPS.hpp Utility.hpp
Geocentric.o: Config.h Constants.hpp Geocentric.hpp Math.hpp
Geodesic.o: Config.h Constants.hpp Geodesic.hpp GeodesicLine.hpp Math.hpp
GeodesicExact.o: Config.h Constants.hpp GeodesicExact.hpp \
	GeodesicLineExact.hpp Math.hpp
GeodesicExactC4.o: Config.h Constants.hpp GeodesicExact.hpp Math.hpp
GeodesicLine.o: Config.h Constants.hpp Geodesic.hpp GeodesicLine.hpp Math.hpp
GeodesicLineExact.o: Config.h Constants.hpp GeodesicExact.hpp \
	GeodesicLineExact.hpp Math.hpp
Geohash.o: Config.h Constants.hpp Geohash.hpp Utility.hpp
Geoid.o: Config.h Constants.hpp Geoid.hpp Math.hpp
Georef.o: Config.h Constants.hpp Georef.hpp Utility.hpp
Gnomonic.o: Config.h Constants.hpp Geodesic.hpp GeodesicLine.hpp Gnomonic.hpp \
	Math.hpp
GravityCircle.o: CircularEngine.hpp Config.h Constants.hpp Geocentric.hpp \
	GravityCircle.hpp GravityModel.hpp Math.hpp NormalGravity.hpp \
	SphericalEngine.hpp SphericalHarmonic.hpp SphericalHarmonic1.hpp
GravityModel.o: CircularEngine.hpp Config.h Constants.hpp Geocentric.hpp \
	GravityCircle.hpp GravityModel.hpp Math.hpp NormalGravity.hpp \
	SphericalEngine.hpp SphericalHarmonic.hpp SphericalHarmonic1.hpp \
	Utility.hpp
LambertConformalConic.o: Config.h Constants.hpp LambertConformalConic.hpp \
	Math.hpp
LocalCartesian.o: Config.h Constants.hpp Geocentric.hpp LocalCartesian.hpp \
	Math.hpp
MGRS.o: Config.h Constants.hpp MGRS.hpp Math.hpp UTMUPS.hpp Utility.hpp
MagneticCircle.o: CircularEngine.hpp Config.h Constants.hpp Geocentric.hpp \
	MagneticCircle.hpp Math.hpp SphericalEngine.hpp
MagneticModel.o: CircularEngine.hpp Config.h Constants.hpp Geocentric.hpp \
	MagneticCircle.hpp MagneticModel.hpp Math.hpp SphericalEngine.hpp \
	SphericalHarmonic.hpp Utility.hpp
Math.o: Config.h Constants.hpp Math.hpp
NormalGravity.o: Config.h Constants.hpp Geocentric.hpp Math.hpp \
	NormalGravity.hpp
OSGB.o: Config.h Constants.hpp Math.hpp OSGB.hpp TransverseMercator.hpp \
	Utility.hpp
PolarStereographic.o: Config.h Constants.hpp Math.hpp PolarStereographic.hpp
PolygonArea.o: Accumulator.hpp Config.h Constants.hpp Geodesic.hpp Math.hpp \
	PolygonArea.hpp
Rhumb.o: Config.h Constants.hpp Ellipsoid.hpp Math.hpp Rhumb.hpp \
	AlbersEqualArea.hpp EllipticFunction.hpp TransverseMercator.hpp
SphericalEngine.o: CircularEngine.hpp Config.h Constants.hpp Math.hpp \
	SphericalEngine.hpp Utility.hpp
TransverseMercator.o: Config.h Constants.hpp Math.hpp TransverseMercator.hpp
TransverseMercatorExact.o: Config.h Constants.hpp EllipticFunction.hpp \
	Math.hpp TransverseMercatorExact.hpp
UTMUPS.o: Config.h Constants.hpp MGRS.hpp Math.hpp PolarStereographic.hpp \
	TransverseMercator.hpp UTMUPS.hpp Utility.hpp
Utility.o: Config.h Constants.hpp Math.hpp Utility.hpp

.PHONY: all install clean
