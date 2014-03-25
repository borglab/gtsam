EXTRAFILES = $(srcdir)/tmseries30.html $(srcdir)/geodseries30.html

FIGURES = $(srcdir)/gauss-krueger-graticule.png \
	$(srcdir)/thompson-tm-graticule.png \
	$(srcdir)/gauss-krueger-convergence-scale.png \
	$(srcdir)/gauss-schreiber-graticule-a.png \
	$(srcdir)/gauss-krueger-graticule-a.png \
	$(srcdir)/thompson-tm-graticule-a.png \
	$(srcdir)/gauss-krueger-error.png \
	$(srcdir)/meridian-measures.png

SCRIPTDRIVERS = \
	$(srcdir)/scripts/geod-calc.html \
	$(srcdir)/scripts/geod-google.html \
	$(srcdir)/scripts/geod-google-instructions.html

JSSCRIPTS = \
	$(srcdir)/scripts/GeographicLib/Math.js \
	$(srcdir)/scripts/GeographicLib/Geodesic.js \
	$(srcdir)/scripts/GeographicLib/GeodesicLine.js \
	$(srcdir)/scripts/GeographicLib/PolygonArea.js \
	$(srcdir)/scripts/GeographicLib/DMS.js \
	$(srcdir)/scripts/GeographicLib/Interface.js

HPPFILES = \
	$(top_srcdir)/include/GeographicLib/Accumulator.hpp \
	$(top_srcdir)/include/GeographicLib/AlbersEqualArea.hpp \
	$(top_srcdir)/include/GeographicLib/AzimuthalEquidistant.hpp \
	$(top_srcdir)/include/GeographicLib/CassiniSoldner.hpp \
	$(top_srcdir)/include/GeographicLib/Constants.hpp \
	$(top_srcdir)/include/GeographicLib/DMS.hpp \
	$(top_srcdir)/include/GeographicLib/Ellipsoid.hpp \
	$(top_srcdir)/include/GeographicLib/EllipticFunction.hpp \
	$(top_srcdir)/include/GeographicLib/Geocentric.hpp \
	$(top_srcdir)/include/GeographicLib/GeoCoords.hpp \
	$(top_srcdir)/include/GeographicLib/Geodesic.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicExact.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicLine.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicLineExact.hpp \
	$(top_srcdir)/include/GeographicLib/Geohash.hpp \
	$(top_srcdir)/include/GeographicLib/Geoid.hpp \
	$(top_srcdir)/include/GeographicLib/Gnomonic.hpp \
	$(top_srcdir)/include/GeographicLib/LambertConformalConic.hpp \
	$(top_srcdir)/include/GeographicLib/LocalCartesian.hpp \
	$(top_srcdir)/include/GeographicLib/Math.hpp \
	$(top_srcdir)/include/GeographicLib/MGRS.hpp \
	$(top_srcdir)/include/GeographicLib/OSGB.hpp \
	$(top_srcdir)/include/GeographicLib/PolarStereographic.hpp \
	$(top_srcdir)/include/GeographicLib/PolygonArea.hpp \
	$(top_srcdir)/include/GeographicLib/TransverseMercatorExact.hpp \
	$(top_srcdir)/include/GeographicLib/TransverseMercator.hpp \
	$(top_srcdir)/include/GeographicLib/UTMUPS.hpp

ALLSOURCES = \
	$(top_srcdir)/src/AlbersEqualArea.cpp \
	$(top_srcdir)/src/AzimuthalEquidistant.cpp \
	$(top_srcdir)/src/CassiniSoldner.cpp \
	$(top_srcdir)/src/DMS.cpp \
	$(top_srcdir)/src/Ellipsoid.cpp \
	$(top_srcdir)/src/EllipticFunction.cpp \
	$(top_srcdir)/src/Geocentric.cpp \
	$(top_srcdir)/src/GeoCoords.cpp \
	$(top_srcdir)/src/Geodesic.cpp \
	$(top_srcdir)/src/GeodesicLine.cpp \
	$(top_srcdir)/src/Geohash.cpp \
	$(top_srcdir)/src/Geoid.cpp \
	$(top_srcdir)/src/Gnomonic.cpp \
	$(top_srcdir)/src/LambertConformalConic.cpp \
	$(top_srcdir)/src/LocalCartesian.cpp \
	$(top_srcdir)/src/MGRS.cpp \
	$(top_srcdir)/src/OSGB.cpp \
	$(top_srcdir)/src/PolarStereographic.cpp \
	$(top_srcdir)/src/PolygonArea.cpp \
	$(top_srcdir)/src/TransverseMercator.cpp \
	$(top_srcdir)/src/TransverseMercatorExact.cpp \
	$(top_srcdir)/src/UTMUPS.cpp \
	$(top_srcdir)/tools/CartConvert.cpp \
	$(top_srcdir)/tools/ConicProj.cpp \
	$(top_srcdir)/tools/GeodesicProj.cpp \
	$(top_srcdir)/tools/GeoConvert.cpp \
	$(top_srcdir)/tools/GeodSolve.cpp \
	$(top_srcdir)/tools/GeoidEval.cpp \
	$(top_srcdir)/tools/Gravity.cpp \
	$(top_srcdir)/tools/Planimeter.cpp \
	$(top_srcdir)/tools/TransverseMercatorProj.cpp

MANPAGES = \
	../man/CartConvert.1.html \
	../man/ConicProj.1.html \
	../man/GeodesicProj.1.html \
	../man/GeoConvert.1.html \
	../man/GeodSolve.1.html \
	../man/GeoidEval.1.html \
	../man/Gravity.1.html \
	../man/MagneticField.1.html \
	../man/Planimeter.1.html \
	../man/TransverseMercatorProj.1.html

LEGACYFILES = \
	$(top_srcdir)/legacy/C/geodesic.c \
	$(top_srcdir)/legacy/C/geodesic.h \
	$(top_srcdir)/legacy/C/direct.c \
	$(top_srcdir)/legacy/C/inverse.c \
	$(top_srcdir)/legacy/C/planimeter.c \
	$(top_srcdir)/legacy/Fortran/geodesic.for \
	$(top_srcdir)/legacy/Fortran/geodesic.inc \
	$(top_srcdir)/legacy/Fortran/geoddirect.for \
	$(top_srcdir)/legacy/Fortran/geodinverse.for \
	$(top_srcdir)/legacy/Fortran/planimeter.for

doc: html/index.html

if HAVE_DOXYGEN
manpages: $(MANPAGES)
	if test -d html; then rm -rf html/*; else mkdir html; fi
	cp $^ html/
	touch $@

html/index.html: manpages doxyfile.in GeographicLib.dox \
	$(HPPFILES) $(ALLSOURCES) $(EXTRAFILES) $(FIGURES) \
	doxyfile-c.in geodesic-c.dox doxyfile-for.in geodesic-for.dox \
	$(LEGACYFILES)
	cp -p $(EXTRAFILES) $(top_srcdir)/maxima/*.mac \
	$(top_srcdir)/LICENSE.txt html/
	sed -e "s%@PROJECT_SOURCE_DIR@%$(top_srcdir)%g" \
	-e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/doxyfile.in | $(DOXYGEN) -
	sed -e "s%@PROJECT_SOURCE_DIR@%$(top_srcdir)%g" \
	-e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/doxyfile-c.in | $(DOXYGEN) -
	sed -e "s%@PROJECT_SOURCE_DIR@%$(top_srcdir)%g" \
	-e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/doxyfile-for.in | $(DOXYGEN) -
else
html/index.html: index.html.in utilities.html.in
	if test -d html; then rm -rf html/*; else mkdir html; fi
	cp $(top_srcdir)/LICENSE.txt html/
	sed -e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/utilities.html.in > html/utilities.html
	sed -e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/index.html.in > html/index.html
endif

maintainer-clean-local:
	rm -rf html manpages

install-doc: html/index.html
	$(INSTALL) -d $(DESTDIR)$(docdir)/html
	$(INSTALL) -m 644 `dirname $<`/*.* $(DESTDIR)$(docdir)/html
	-test -f `dirname $<`/C/index.html && \
	$(INSTALL) -d $(DESTDIR)$(docdir)/html/C && \
	$(INSTALL) -m 644 `dirname $<`/C/*.* $(DESTDIR)$(docdir)/html/C
	-test -f `dirname $<`/Fortran/index.html && \
	$(INSTALL) -d $(DESTDIR)$(docdir)/html/Fortran && \
	$(INSTALL) -m 644 `dirname $<`/Fortran/*.* \
	$(DESTDIR)$(docdir)/html/Fortran
	$(INSTALL) -d $(DESTDIR)$(docdir)/scripts
	$(INSTALL) -m 644 $(SCRIPTDRIVERS) $(DESTDIR)$(docdir)/scripts
	$(INSTALL) -d $(DESTDIR)$(docdir)/scripts/GeographicLib
	$(INSTALL) -m 644 $(JSSCRIPTS) $(DESTDIR)$(docdir)/scripts/GeographicLib
