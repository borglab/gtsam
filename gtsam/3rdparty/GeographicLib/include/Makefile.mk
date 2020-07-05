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

PREFIX = /usr/local
LIBNAME = GeographicLib
HEADERS = $(LIBNAME)/Config.h \
	$(patsubst %,$(LIBNAME)/%.hpp,$(EXTRAHEADERS) $(MODULES))
DEST = $(PREFIX)/include/$(LIBNAME)

INSTALL = install -b

all:
	@:

install:
	test -d $(DEST) || mkdir -p $(DEST)
	$(INSTALL) -m 644 $(HEADERS) $(DEST)/
clean:
	@:

.PHONY: all install clean
