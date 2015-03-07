#
# Makefile.am
#
# Copyright (C) 2011, Charles Karney <charles@karney.com>

MATLAB_INTERFACE = \
$(srcdir)/geodesicdirect.cpp \
$(srcdir)/geodesicinverse.cpp \
$(srcdir)/geodesicline.cpp \
$(srcdir)/geoidheight.cpp \
$(srcdir)/mgrsforward.cpp \
$(srcdir)/mgrsreverse.cpp \
$(srcdir)/utmupsforward.cpp \
$(srcdir)/utmupsreverse.cpp \
$(srcdir)/geocentricforward.cpp \
$(srcdir)/geocentricreverse.cpp \
$(srcdir)/localcartesianforward.cpp \
$(srcdir)/localcartesianreverse.cpp \
$(srcdir)/polygonarea.cpp

MATLAB_DOC = \
$(srcdir)/geodesicdirect.m \
$(srcdir)/geodesicinverse.m \
$(srcdir)/geodesicline.m \
$(srcdir)/geoidheight.m \
$(srcdir)/mgrsforward.m \
$(srcdir)/mgrsreverse.m \
$(srcdir)/utmupsforward.m \
$(srcdir)/utmupsreverse.m \
$(srcdir)/geocentricforward.m \
$(srcdir)/geocentricreverse.m \
$(srcdir)/localcartesianforward.m \
$(srcdir)/localcartesianreverse.m \
$(srcdir)/polygonarea.m

MATLAB_COMPILESCRIPT = $(srcdir)/geographiclibinterface.m

MATLAB_GEOD = \
$(srcdir)/geoddoc.m \
$(srcdir)/geodreckon.m \
$(srcdir)/geoddistance.m \
$(srcdir)/geodarea.m \
$(srcdir)/defaultellipsoid.m \
$(srcdir)/ecc2flat.m \
$(srcdir)/flat2ecc.m \
$(srcdir)/geodproj.m \
$(srcdir)/eqdazim_fwd.m \
$(srcdir)/eqdazim_inv.m \
$(srcdir)/cassini_fwd.m \
$(srcdir)/cassini_inv.m \
$(srcdir)/tranmerc_fwd.m \
$(srcdir)/tranmerc_inv.m \
$(srcdir)/gnomonic_fwd.m \
$(srcdir)/gnomonic_inv.m \
$(srcdir)/utm_fwd.m \
$(srcdir)/utm_inv.m

MATLAB_GEOD_PRIVATE = \
$(srcdir)/private/A1m1f.m \
$(srcdir)/private/A2m1f.m \
$(srcdir)/private/A3coeff.m \
$(srcdir)/private/A3f.m \
$(srcdir)/private/AngDiff.m \
$(srcdir)/private/AngNormalize2.m \
$(srcdir)/private/AngNormalize.m \
$(srcdir)/private/AngRound.m \
$(srcdir)/private/atanhee.m \
$(srcdir)/private/C1f.m \
$(srcdir)/private/C1pf.m \
$(srcdir)/private/C2f.m \
$(srcdir)/private/C3coeff.m \
$(srcdir)/private/C3f.m \
$(srcdir)/private/C4coeff.m \
$(srcdir)/private/C4f.m \
$(srcdir)/private/cbrt.m \
$(srcdir)/private/cvmgt.m \
$(srcdir)/private/SinCosNorm.m \
$(srcdir)/private/SinCosSeries.m \
$(srcdir)/private/sumx.m \
$(srcdir)/private/swap.m

MATLAB_ALL = $(MATLAB_INTERFACE) $(MATLAB_DOC) $(MATLAB_COMPILESCRIPT) \
	$(MATLAB_GEOD)

matlabdir=$(libexecdir)/GeographicLib/matlab

install:
	$(INSTALL) -d $(DESTDIR)$(matlabdir)/private
	$(INSTALL) -m 644 $(MATLAB_ALL) $(DESTDIR)$(matlabdir)
	$(INSTALL) -m 644 $(MATLAB_GEOD_PRIVATE) $(DESTDIR)$(matlabdir)/private

clean-local:
	rm -rf *.mex* *.oct

EXTRA_DIST = Makefile.mk CMakeLists.txt $(MATLAB_ALL) $(MATLAB_GEOD_PRIVATE)
