#
# Makefile.am
#
# Copyright (C) 2009, Francesco P. Lovergine <frankie@debian.org>

AUTOMAKE_OPTIONS = foreign

ACLOCAL_AMFLAGS = -I m4

SUBDIRS = src man tools doc include cmake examples tests experimental

EXTRA_DIST = AUTHORS LICENSE.txt NEWS README.md \
	CMakeLists.txt maxima doc wrapper

# Install the pkg-config file; the directory is set using
# PKG_INSTALLDIR in configure.ac.
pkgconfig_DATA = cmake/geographiclib.pc

dist-hook:
	rm -rf $(distdir)/doc/html $(distdir)/doc/manpages \
		$(distdir)/doc/GeographicLib.dox \
		$(distdir)/include/GeographicLib/Config.h
	find $(distdir)/maxima -type f -name '*.lsp' | xargs rm -rf
	find $(distdir)/wrapper -mindepth 2 -type d | xargs rm -rf
	find $(distdir)/wrapper -type f -name '*.o' -o -name '*.mex*' | \
		xargs rm -f
	find $(distdir) \
		\( -name '.git*' -o -name Makefile -o -name '*~' -o \
		-name '*#*' -o -name 'CMakeFiles' -o -name '*.log' -o \
		-name '*.tmp' -o -name '*.bak' -o -name '*.BAK' \) | \
		xargs rm -rf
	echo $(VERSION) > $(distdir)/VERSION

# Custom rules

all-local: man doc
install-data-local: install-doc

doc: man
	$(MAKE) -C doc doc

install-doc:
	$(MAKE) -C doc install-doc

man:
	$(MAKE) -C man man

.PHONY: doc install-doc man
