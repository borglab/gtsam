DEST = $(PREFIX)/share/cmake/GeographicLib

INSTALL=install -b

all:
	@:
install:
	test -d $(DEST) || mkdir -p $(DEST)
	$(INSTALL) -m 644 FindGeographicLib.cmake $(DEST)
clean:
	@:

.PHONY: all install clean
