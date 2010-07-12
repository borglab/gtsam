#----------------------------------------------------------------------------------------------------
# colamd
# replaced Makefile with automake for easy linking
#----------------------------------------------------------------------------------------------------

# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
noinst_LTLIBRARIES = libcolamd.la

# We normally would not install these headers
# but they are included in the templated class FactorGraph-inl.h so we need them
pkginclude_HEADERS = colamd.h ccolamd.h UFconfig.h 

# These are the sources for the library:
libcolamd_la_SOURCES = colamd.c colamd_global.c ccolamd.c ccolamd_global.c
