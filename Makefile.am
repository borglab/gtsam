#The option -I m4 tells Autoconf to look for additional Autoconf macros in the m4 subdirectory.
ACLOCAL_AMFLAGS = -I m4

# make automake install some standard but missing files
AUTOMAKE_OPTIONS = foreign
SUBDIRS = colamd CppUnitLite ldl spqr_mini cpp wrap

# install the matlab toolbox
install-exec-hook:
	install -d ${toolbox}/gtsam && \
	cp -rf toolbox/* ${toolbox}/gtsam && \
	install -d ${prefix}/include/wrap && \
	cp -f wrap/wrap-matlab.h  ${prefix}/include/wrap/matlab.h && \
	install -d $(prefix)/include && \
	install -d $(prefix)/include/colamd && \
	cp -f colamd/colamd.h $(prefix)/include/colamd/ && \
	cp -f colamd/UFconfig.h $(prefix)/include/colamd/ && \
	cp -f colamd/libcolamd.a $(prefix)/lib/ && \
	install -d $(prefix)/include/ldl && \
	cp -f ldl/ldl.h $(prefix)/include/ldl/ && \
	cp -f ldl/UFconfig.h $(prefix)/include/ldl/ && \
	cp -f ldl/libldl.a $(prefix)/lib/
	install -d $(prefix)/include/spqr_mini && \
	cp -f spqr_mini/*.h   $(prefix)/include/spqr_mini/ && \
	cp -f spqr_mini/*.hpp $(prefix)/include/spqr_mini/ && \
	cp -f spqr_mini/libspqr_mini.a $(prefix)/lib/

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
	mkdir $(distdir)/ldl
	cp -p $(srcdir)/ldl/Makefile $(distdir)/ldl
	cp -p $(srcdir)/ldl/*.h $(distdir)/ldl
	cp -p $(srcdir)/ldl/*.c $(distdir)/ldl
	cp -p $(srcdir)/ldl/*.cpp $(distdir)/ldl
	mkdir $(distdir)/spqr_mini
	cp -p $(srcdir)/spqr_mini/Makefile $(distdir)/spqr_mini
	cp -p $(srcdir)/spqr_mini/*.h $(distdir)/spqr_mini
	cp -p $(srcdir)/spqr_mini/*.c $(distdir)/spqr_mini
	cp -p $(srcdir)/spqr_mini/*.cpp $(distdir)/spqr_mini
	mkdir $(distdir)/matlab
	cp -p $(srcdir)/matlab/*.m $(distdir)/matlab
