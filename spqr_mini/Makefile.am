# Only install if LAPACK enabled

if USE_LAPACK

sources = cholmod_error.c cholmod_common.c cholmod_memory.c spqr_front.cpp spqr_larftb.cpp
headers = UFconfig.h cholmod_common.h cholmod_internal.h cholmod_blas.h cholmod_core.h
headers += SuiteSparseQR_definitions.h SuiteSparseQR_subset.hpp spqr_subset.hpp spqr_larftb.h spqr_front.h

# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are not installed either 
noinst_LTLIBRARIES = libspqr_mini.la
libspqr_mini_la_SOURCES = $(sources)
noinst_HEADERS = $(headers)

AM_CPPFLAGS = -DDLONG # Compiles cholmod in double/long mode 

# On Mac, we compile using the BLAS/LAPACK headers in the Accelerate framework 
if USE_ACCELERATE_MACOS
AM_CPPFLAGS += -F Accelerate
endif

endif


