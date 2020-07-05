EXTRAFILES = $(srcdir)/tmseries30.html $(srcdir)/geodseries30.html

FIGURES = \
	$(srcdir)/gauss-krueger-graticule.png \
	$(srcdir)/thompson-tm-graticule.png \
	$(srcdir)/gauss-krueger-convergence-scale.png \
	$(srcdir)/gauss-schreiber-graticule-a.png \
	$(srcdir)/gauss-krueger-graticule-a.png \
	$(srcdir)/thompson-tm-graticule-a.png \
	$(srcdir)/gauss-krueger-error.png \
	$(srcdir)/meridian-measures.png \
	$(srcdir)/normal-gravity-potential-1.svg \
	$(srcdir)/vptree.gif

HPPFILES = \
	$(top_srcdir)/include/GeographicLib/Accumulator.hpp \
	$(top_srcdir)/include/GeographicLib/AlbersEqualArea.hpp \
	$(top_srcdir)/include/GeographicLib/AzimuthalEquidistant.hpp \
	$(top_srcdir)/include/GeographicLib/CassiniSoldner.hpp \
	$(top_srcdir)/include/GeographicLib/Constants.hpp \
	$(top_srcdir)/include/GeographicLib/DMS.hpp \
	$(top_srcdir)/include/GeographicLib/Ellipsoid.hpp \
	$(top_srcdir)/include/GeographicLib/EllipticFunction.hpp \
	$(top_srcdir)/include/GeographicLib/GARS.hpp \
	$(top_srcdir)/include/GeographicLib/GeoCoords.hpp \
	$(top_srcdir)/include/GeographicLib/Geocentric.hpp \
	$(top_srcdir)/include/GeographicLib/Geodesic.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicExact.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicLine.hpp \
	$(top_srcdir)/include/GeographicLib/GeodesicLineExact.hpp \
	$(top_srcdir)/include/GeographicLib/Geohash.hpp \
	$(top_srcdir)/include/GeographicLib/Geoid.hpp \
	$(top_srcdir)/include/GeographicLib/Georef.hpp \
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
	$(top_srcdir)/src/GARS.cpp \
	$(top_srcdir)/src/GeoCoords.cpp \
	$(top_srcdir)/src/Geocentric.cpp \
	$(top_srcdir)/src/Geodesic.cpp \
	$(top_srcdir)/src/GeodesicLine.cpp \
	$(top_srcdir)/src/Geohash.cpp \
	$(top_srcdir)/src/Geoid.cpp \
	$(top_srcdir)/src/Georef.cpp \
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
	../man/RhumbSolve.1.html \
	../man/TransverseMercatorProj.1.html

doc: html/index.html

if HAVE_DOXYGEN
manpages: $(MANPAGES)
	if test -d html; then rm -rf html/*; else mkdir html; fi
	cp $^ html/
	touch $@

html/index.html: manpages doxyfile.in GeographicLib.dox.in \
	$(HPPFILES) $(ALLSOURCES) $(EXTRAFILES) $(FIGURES)
	cp -p $(EXTRAFILES) $(top_srcdir)/maxima/*.mac \
	$(top_srcdir)/LICENSE.txt html/
	sed -e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/GeographicLib.dox.in > GeographicLib.dox
	sed -e "s%@PROJECT_SOURCE_DIR@%$(top_srcdir)%g" \
	-e "s%@PROJECT_BINARY_DIR@%..%g" \
	-e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	$(srcdir)/doxyfile.in | $(DOXYGEN) -
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

htmldir=$(DESTDIR)$(docdir)/html

install-doc: html/index.html
	$(INSTALL) -d $(htmldir)
	$(INSTALL) -m 644 `dirname $<`/*.* $(htmldir)
	-test -f `dirname $<`/search/search.js && \
	$(INSTALL) -d $(htmldir)/search && \
	$(INSTALL) -m 644 `dirname $<`/search/*.* $(htmldir)/search || true
