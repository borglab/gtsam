PROGRAMS = CartConvert \
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
SCRIPTS = geographiclib-get-geoids \
	geographiclib-get-gravity \
	geographiclib-get-magnetic

all: $(PROGRAMS) $(SCRIPTS)

LIBSTEM = Geographic
LIBRARY = lib$(LIBSTEM).a

INCLUDEPATH = ../include
LIBPATH = ../src

# After installation, use these values of INCLUDEPATH and LIBPATH
# INCLUDEPATH = $(PREFIX)/include
# LIBPATH = $(PREFIX)/lib

PREFIX = /usr/local
GEOGRAPHICLIB_DATA = $(PREFIX)/share/GeographicLib

CC = g++ -g
CXXFLAGS = -g -Wall -Wextra -O3 -std=c++0x

CPPFLAGS = -I$(INCLUDEPATH) -I../man $(DEFINES)
LDLIBS = -L$(LIBPATH) -l$(LIBSTEM)
EXTRALIBS =

$(PROGRAMS): $(LIBPATH)/$(LIBRARY)
	$(CC) $(LDFLAGS) -o $@ $@.o $(LDLIBS) $(EXTRALIBS)

VPATH = ../include/GeographicLib ../man

clean:
	rm -f *.o $(SCRIPTS)

CartConvert: CartConvert.o
ConicProj: ConicProj.o
GeoConvert: GeoConvert.o
GeodSolve: GeodSolve.o
GeodesicProj: GeodesicProj.o
GeoidEval: GeoidEval.o
Gravity: Gravity.o
MagneticField: MagneticField.o
Planimeter: Planimeter.o
RhumbSolve: RhumbSolve.o
TransverseMercatorProj: TransverseMercatorProj.o

CartConvert.o: CartConvert.usage Config.h Constants.hpp DMS.hpp \
	Geocentric.hpp LocalCartesian.hpp Math.hpp Utility.hpp
ConicProj.o: ConicProj.usage Config.h AlbersEqualArea.hpp Constants.hpp \
	DMS.hpp LambertConformalConic.hpp Math.hpp Utility.hpp
GeoConvert.o: GeoConvert.usage Config.h Constants.hpp DMS.hpp GeoCoords.hpp \
	Math.hpp UTMUPS.hpp Utility.hpp
GeodSolve.o: GeodSolve.usage Config.h Constants.hpp DMS.hpp Geodesic.hpp \
	GeodesicExact.hpp GeodesicLine.hpp GeodesicLineExact.hpp Math.hpp \
	Utility.hpp
GeodesicProj.o: GeodesicProj.usage Config.h AzimuthalEquidistant.hpp \
	CassiniSoldner.hpp Constants.hpp DMS.hpp Geodesic.hpp \
	GeodesicLine.hpp Gnomonic.hpp Math.hpp Utility.hpp
GeoidEval.o: GeoidEval.usage Config.h Constants.hpp DMS.hpp GeoCoords.hpp \
	Geoid.hpp Math.hpp UTMUPS.hpp Utility.hpp
Gravity.o: Gravity.usage Config.h CircularEngine.hpp Constants.hpp DMS.hpp \
	Geocentric.hpp GravityCircle.hpp GravityModel.hpp Math.hpp \
	NormalGravity.hpp SphericalEngine.hpp SphericalHarmonic.hpp \
	SphericalHarmonic1.hpp Utility.hpp
MagneticField.o: MagneticField.usage Config.h CircularEngine.hpp \
	Constants.hpp DMS.hpp Geocentric.hpp MagneticCircle.hpp \
	MagneticModel.hpp Math.hpp SphericalEngine.hpp SphericalHarmonic.hpp \
	Utility.hpp
Planimeter.o: Planimeter.usage Config.h Accumulator.hpp Constants.hpp DMS.hpp \
	Ellipsoid.hpp GeoCoords.hpp Geodesic.hpp Math.hpp PolygonArea.hpp \
	UTMUPS.hpp Utility.hpp
RhumbSolve.o: RhumbSolve.usage Config.h Constants.hpp DMS.hpp Ellipsoid.hpp \
	Math.hpp Utility.hpp
TransverseMercatorProj.o: TransverseMercatorProj.usage Config.h Constants.hpp \
	DMS.hpp EllipticFunction.hpp Math.hpp TransverseMercator.hpp \
	TransverseMercatorExact.hpp Utility.hpp

%: %.sh
	sed -e "s%@GEOGRAPHICLIB_DATA@%$(GEOGRAPHICLIB_DATA)%" $< > $@
	chmod +x $@

INSTALL = install -b
DEST = $(PREFIX)/bin
SDEST = $(PREFIX)/sbin

install: $(PROGRAMS) $(SCRIPTS)
	test -f $(DEST) || mkdir -p $(DEST)
	$(INSTALL) $(PROGRAMS) $(DEST)
	test -f $(SDEST) || mkdir -p $(SDEST)
	$(INSTALL) $(SCRIPTS) $(SDEST)
