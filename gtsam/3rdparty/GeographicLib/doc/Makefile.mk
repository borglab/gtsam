SCRIPTDRIVERS = $(wildcard scripts/[A-Za-z]*.html)
JSSCRIPTS = $(wildcard scripts/GeographicLib/[A-Za-z]*.js)

VERSION:=$(shell grep '\bVERSION=' ../configure | cut -f2 -d\' | head -1)

doc: html/index.html

html/index.html: index.html.in utilities.html.in
	if test -d html; then rm -rf html/*; else mkdir html; fi
	cp ../LICENSE.txt html/
	sed -e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	utilities.html.in > html/utilities.html
	sed -e "s%@PROJECT_VERSION@%$(VERSION)%g" \
	index.html.in > html/index.html

PREFIX = /usr/local
DEST = $(PREFIX)/share/doc/GeographicLib
DOCDEST = $(DEST)/html
SCRIPTDEST = $(DEST)/scripts
INSTALL = install -b

install: html/index.html
	test -d $(DOCDEST) || mkdir -p $(DOCDEST)
	$(INSTALL) -m 644 html/* $(DOCDEST)/
	test -d $(SCRIPTDEST)/GeographicLib || \
	mkdir -p $(SCRIPTDEST)/GeographicLib
	$(INSTALL) -m 644 $(SCRIPTDRIVERS) $(SCRIPTDEST)/
	$(INSTALL) -m 644 $(JSSCRIPTS) $(SCRIPTDEST)/GeographicLib/

.PHONY: doc install clean
