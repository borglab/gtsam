#----------------------------------------------------------------------------------------------------
# GTSAM 3rdparty: CCOLAMD
# Tim Davis' sparse library for column ordering - part of suitesparse
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 

# CCOLAMD (with UFconfig files included)
headers += ccolamd.h UFconfig.h
sources += ccolamd.c ccolamd_global.c UFconfig.c 

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam/3rdparty: 
#----------------------------------------------------------------------------------------------------
ccolamddir = $(pkgincludedir)/3rdparty/ccolamd
ccolamd_HEADERS = $(headers)
noinst_LTLIBRARIES = libccolamd.la
libccolamd_la_SOURCES = $(sources)

AM_CPPFLAGS =

AM_CPPFLAGS += $(BOOST_CPPFLAGS) -I$(top_srcdir) -DDLONG 
AM_LDFLAGS = $(BOOST_LDFLAGS)
