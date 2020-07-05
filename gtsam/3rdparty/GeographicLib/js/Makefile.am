EXTRAFILES = $(srcdir)/HEADER.js

SAMPLES = \
	geod-calc.html \
	geod-google.html \
	geod-google-instructions.html

# The order here is significant
JSSCRIPTS = \
	$(srcdir)/src/Math.js \
	$(srcdir)/src/Geodesic.js \
	$(srcdir)/src/GeodesicLine.js \
	$(srcdir)/src/PolygonArea.js \
	$(srcdir)/src/DMS.js
TESTSCRIPTS = $(srcdir)/test/geodesictest.js

all: geographiclib.js geographiclib.min.js $(SAMPLES)

geod-calc.html: samples/geod-calc.html
	cp $^ $@

geod-google.html: samples/geod-google.html
	cp $^ $@

geod-google-instructions.html: samples/geod-google-instructions.html
	cp $^ $@

geographiclib.js: HEADER.js $(JSSCRIPTS)
	$(srcdir)/js-cat.sh $^ > $@

geographiclib.min.js: HEADER.js $(JSSCRIPTS)
	$(srcdir)/js-compress.sh $^ > $@

jsdir=$(DESTDIR)$(libdir)/node_modules/geographiclib

install: all
	$(INSTALL) -d $(jsdir)
	$(INSTALL) -m 644 geographiclib.js geographiclib.min.js $(jsdir)
	$(INSTALL) -m 644 $(top_srcdir)/LICENSE.txt $(srcdir)/README.md \
		$(srcdir)/package.json $(jsdir)
	$(INSTALL) -d $(jsdir)/src
	$(INSTALL) -m 644 $(JSSCRIPTS) $(jsdir)/src
	$(INSTALL) -d $(jsdir)/test
	$(INSTALL) -m 644 $(TESTSCRIPTS) $(jsdir)/test
