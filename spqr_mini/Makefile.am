# the install destination
includedir = ${prefix}/include/spqr_mini
libdir = ${exec_prefix}/lib

if USE_LAPACK_MACOS
   sources = cholmod_common.c cholmod_memory.c spqr_front.cpp spqr_larftb.cpp
   headers = UFconfig.h cholmod_common.h cholmod_internal.h cholmod_blas.h cholmod_core.h
   headers += SuiteSparseQR_definitions.h SuiteSparseQR_subset.hpp spqr_subset.hpp spqr_larftb.h spqr_front.h

   AM_CXXFLAGS = -fPIC

   # create both dynamic and static libraries
   lib_LTLIBRARIES = libspqr_mini.la
   libspqr_mini_la_SOURCES = $(sources)

   include_HEADERS = $(headers)

   AM_CXXFLAGS += -DGT_USE_LAPACK -DDLONG

if USE_VECLIB_MACOS
AM_CXXFLAGS += -DYA_BLAS -DYA_LAPACK -DYA_BLASMULT -I/System/Library/Frameworks/vecLib.framework/Headers 
libspqr_mini_la_CPPFLAGS = -DDLONG -DGT_USE_CBLAS -DYA_BLAS -DYA_LAPACK -DYA_BLASMULT -I/System/Library/Frameworks/vecLib.framework/Headers 
AM_LDFLAGS = -lcblas -latlas 
libspqr_mini_la_LDFLAGS = -framework vecLib -lcblas -latlas
endif

endif


