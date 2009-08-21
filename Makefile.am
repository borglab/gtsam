#The option -I m4 tells Autoconf to look for additional Autoconf macros in the m4 subdirectory.
ACLOCAL_AMFLAGS = -I m4

# make automake install some standard but missing files
AUTOMAKE_OPTIONS = foreign
SUBDIRS = cpp wrap

# install the matlab toolbox
install-exec-hook:
	install -d ${toolbox}/gtsam && \
	cp -rf toolbox/* ${toolbox}/gtsam && \
	install -d ${prefix}/include/wrap && \
	cp -f wrap/wrap-matlab.h  ${prefix}/include/wrap/matlab.h && \
	install -d $(prefix)/include && \
	install -d $(prefix)/include/colamd && \
	cp -f colamd/colamd.h $(prefix)/include/colamd/ && \
	cp -f colamd/UFconfig.h $(prefix)/include/colamd/

dist-hook:
	mkdir $(distdir)/CppUnitLite
	cp -p $(srcdir)/CppUnitLite/Makefile $(distdir)/CppUnitLite
	cp -p $(srcdir)/CppUnitLite/*.h $(distdir)/CppUnitLite
	cp -p $(srcdir)/CppUnitLite/*.cpp $(distdir)/CppUnitLite
	mkdir $(distdir)/colamd
	cp -p $(srcdir)/colamd/Makefile $(distdir)/colamd
	cp -p $(srcdir)/colamd/*.h $(distdir)/colamd
	cp -p $(srcdir)/colamd/*.c $(distdir)/colamd
	cp -p $(srcdir)/colamd/*.cpp $(distdir)/colamd
	mkdir $(distdir)/matlab
	cp -p $(srcdir)/matlab/*.m $(distdir)/matlab
