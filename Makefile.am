#----------------------------------------------------------------------------------------------------
# GTSAM top-level automake file
#----------------------------------------------------------------------------------------------------

#The option -I m4 tells Autoconf to look for additional Autoconf macros in the m4 subdirectory.
ACLOCAL_AMFLAGS = -I m4

# make automake install some standard but missing files
# also use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = foreign nostdinc

# All the sub-directories that need to be built
SUBDIRS = CppUnitLite colamd spqr_mini base geometry inference linear nonlinear slam . tests wrap  

# And the corresponding libraries produced
SUBLIBS = colamd/libcolamd.la \
          base/libbase.la geometry/libgeometry.la inference/libinference.la \
          linear/liblinear.la nonlinear/libnonlinear.la slam/libslam.la

if USE_LAPACK
SUBLIBS += spqr_mini/libspqr_mini.la
endif

# TODO: UFconfig, CCOLAMD, and LDL automake magic without adding or touching any file
# in those directories as to not invalidate the LGPL license
# See some possibilities in
# http://www.gnu.org/software/hello/manual/automake/Third_002dParty-Makefiles.html

# The following lines specify the actual shared library to be built with libtool
lib_LTLIBRARIES = libgtsam.la
libgtsam_la_SOURCES =
libgtsam_la_LIBADD = $(SUBLIBS)
libgtsam_la_LDFLAGS = -version-info 0:0:0

# Store the absolute path to the directory above gtsam for the include path,
# saves us from having an extra 'gtsam' subdirectory.
BORG_SRCROOT := $(shell cd $(top_srcdir)/.. && pwd)
AM_MAKEFLAGS = "BORG_SRCROOT=$(BORG_SRCROOT)"

# Add these files to make sure they're in the distribution
noinst_HEADERS = gtsam.h
EXTRA_DIST = autogen.sh configure.ac THANKS 

# Todo: Also do CppUnitLite with automake 
dist-hook:
	mkdir $(distdir)/config
	mkdir $(distdir)/matlab
	cp -p $(srcdir)/matlab/*.m $(distdir)/matlab
	cp -r $(srcdir)/wrap/expected $(distdir)/wrap


