#
# Makefile.am
#
# Copyright (c) Charles Karney (2011-2016) <charles@karney.com>

MATLAB_FILES = \
	$(srcdir)/geographiclib/Contents.m \
	$(srcdir)/geographiclib/geographiclib_test.m \
	$(srcdir)/geographiclib/cassini_fwd.m \
	$(srcdir)/geographiclib/cassini_inv.m \
	$(srcdir)/geographiclib/defaultellipsoid.m \
	$(srcdir)/geographiclib/ecc2flat.m \
	$(srcdir)/geographiclib/eqdazim_fwd.m \
	$(srcdir)/geographiclib/eqdazim_inv.m \
	$(srcdir)/geographiclib/flat2ecc.m \
	$(srcdir)/geographiclib/gedistance.m \
	$(srcdir)/geographiclib/gedoc.m \
	$(srcdir)/geographiclib/geocent_fwd.m \
	$(srcdir)/geographiclib/geocent_inv.m \
	$(srcdir)/geographiclib/geodarea.m \
	$(srcdir)/geographiclib/geoddistance.m \
	$(srcdir)/geographiclib/geoddoc.m \
	$(srcdir)/geographiclib/geodreckon.m \
	$(srcdir)/geographiclib/geoid_height.m \
	$(srcdir)/geographiclib/geoid_load.m \
	$(srcdir)/geographiclib/gereckon.m \
	$(srcdir)/geographiclib/gnomonic_fwd.m \
	$(srcdir)/geographiclib/gnomonic_inv.m \
	$(srcdir)/geographiclib/loccart_fwd.m \
	$(srcdir)/geographiclib/loccart_inv.m \
	$(srcdir)/geographiclib/mgrs_fwd.m \
	$(srcdir)/geographiclib/mgrs_inv.m \
	$(srcdir)/geographiclib/polarst_fwd.m \
	$(srcdir)/geographiclib/polarst_inv.m \
	$(srcdir)/geographiclib/projdoc.m \
	$(srcdir)/geographiclib/tranmerc_fwd.m \
	$(srcdir)/geographiclib/tranmerc_inv.m \
	$(srcdir)/geographiclib/utmups_fwd.m \
	$(srcdir)/geographiclib/utmups_inv.m

MATLAB_PRIVATE = \
	$(srcdir)/geographiclib/private/A1m1f.m \
	$(srcdir)/geographiclib/private/A2m1f.m \
	$(srcdir)/geographiclib/private/A3coeff.m \
	$(srcdir)/geographiclib/private/A3f.m \
	$(srcdir)/geographiclib/private/AngDiff.m \
	$(srcdir)/geographiclib/private/AngNormalize.m \
	$(srcdir)/geographiclib/private/AngRound.m \
	$(srcdir)/geographiclib/private/C1f.m \
	$(srcdir)/geographiclib/private/C1pf.m \
	$(srcdir)/geographiclib/private/C2f.m \
	$(srcdir)/geographiclib/private/C3coeff.m \
	$(srcdir)/geographiclib/private/C3f.m \
	$(srcdir)/geographiclib/private/C4coeff.m \
	$(srcdir)/geographiclib/private/C4f.m \
	$(srcdir)/geographiclib/private/G4coeff.m \
	$(srcdir)/geographiclib/private/GeoRotation.m \
	$(srcdir)/geographiclib/private/LatFix.m \
	$(srcdir)/geographiclib/private/SinCosSeries.m \
	$(srcdir)/geographiclib/private/atan2dx.m \
	$(srcdir)/geographiclib/private/cbrtx.m \
	$(srcdir)/geographiclib/private/copysignx.m \
	$(srcdir)/geographiclib/private/cvmgt.m \
	$(srcdir)/geographiclib/private/eatanhe.m \
	$(srcdir)/geographiclib/private/geoid_file.m \
	$(srcdir)/geographiclib/private/geoid_load_file.m \
	$(srcdir)/geographiclib/private/norm2.m \
	$(srcdir)/geographiclib/private/sincosdx.m \
	$(srcdir)/geographiclib/private/sumx.m \
	$(srcdir)/geographiclib/private/swap.m \
	$(srcdir)/geographiclib/private/tauf.m \
	$(srcdir)/geographiclib/private/taupf.m

MATLAB_LEGACY = \
	$(srcdir)/geographiclib-legacy/Contents.m \
	$(srcdir)/geographiclib-legacy/geocentricforward.m \
	$(srcdir)/geographiclib-legacy/geocentricreverse.m \
	$(srcdir)/geographiclib-legacy/geodesicdirect.m \
	$(srcdir)/geographiclib-legacy/geodesicinverse.m \
	$(srcdir)/geographiclib-legacy/geodesicline.m \
	$(srcdir)/geographiclib-legacy/geoidheight.m \
	$(srcdir)/geographiclib-legacy/localcartesianforward.m \
	$(srcdir)/geographiclib-legacy/localcartesianreverse.m \
	$(srcdir)/geographiclib-legacy/mgrsforward.m \
	$(srcdir)/geographiclib-legacy/mgrsreverse.m \
	$(srcdir)/geographiclib-legacy/polygonarea.m \
	$(srcdir)/geographiclib-legacy/utmupsforward.m \
	$(srcdir)/geographiclib-legacy/utmupsreverse.m

MATLAB_ALL = $(MATLAB_FILES) $(MATLAB_PRIVATE)

matlabdir=$(DESTDIR)$(datadir)/matlab

install:
	$(INSTALL) -d $(matlabdir)/geographiclib/private
	$(INSTALL) -d $(matlabdir)/geographiclib-legacy
	$(INSTALL) -m 644 $(MATLAB_FILES) $(matlabdir)/geographiclib
	$(INSTALL) -m 644 $(MATLAB_PRIVATE) $(matlabdir)/geographiclib/private
	$(INSTALL) -m 644 $(MATLAB_LEGACY) $(matlabdir)/geographiclib-legacy

clean-local:
	rm -rf *.mex* *.oct

EXTRA_DIST = Makefile.mk CMakeLists.txt $(MATLAB_ALL) $(MATLAB_LEGACY)
