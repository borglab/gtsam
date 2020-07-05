# The order here is significant
JS_MODULES=Math Geodesic GeodesicLine PolygonArea DMS
JSSCRIPTS = $(patsubst %,src/%.js,$(JS_MODULES))
TESTSCRIPTS = $(wildcard test/*.js)

SAMPLESIN = $(wildcard samples/geod-*.html)
SAMPLES = $(patsubst samples/%,%,$(SAMPLESIN))

all: geographiclib.js geographiclib.min.js $(SAMPLES)

%.html: samples/%.html
	cp $^ $@

geographiclib.js: HEADER.js $(JSSCRIPTS)
	./js-cat.sh $^ > $@

geographiclib.min.js: HEADER.js $(JSSCRIPTS)
	./js-compress.sh $^ > $@

clean:
	rm -f geographiclib.js geographiclib.min.js *.html

PREFIX = /usr/local
DEST = $(PREFIX)/lib/node_modules/geographiclib
INSTALL = install -b

install: all
	test -d $(DEST) || mkdir -p $(DEST)
	$(INSTALL) -m 644 geographiclib.js geographiclib.min.js $(DEST)/
	$(INSTALL) -m 644 ../LICENSE.txt README.md package.json $(DEST)/
	test -d $(DEST)/src || mkdir -p $(DEST)/src
	$(INSTALL) -m 644 $(JSSCRIPTS) $(DEST)/src/
	test -d $(DEST)/test || mkdir -p $(DEST)/test
	$(INSTALL) -m 644 $(TESTSCRIPTS) $(DEST)/test/

.PHONY: install clean
