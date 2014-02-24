#
# Makefile.am
#
# Copyright (C) 2011, Charles Karney <charles@karney.com>

cmakedir=$(datadir)/cmake/GeographicLib

install:
	$(INSTALL) -d $(DESTDIR)$(cmakedir)
	$(INSTALL) -m 644 $(srcdir)/FindGeographicLib.cmake \
		$(DESTDIR)$(cmakedir)

EXTRA_DIST = Makefile.mk CMakeLists.txt FindGeographicLib.cmake \
	project-config-version.cmake.in project-config.cmake.in
