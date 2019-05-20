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
INSTALL = install -b

install: html/index.html
	test -d $(DOCDEST) || mkdir -p $(DOCDEST)
	$(INSTALL) -m 644 html/* $(DOCDEST)/

.PHONY: doc install clean
