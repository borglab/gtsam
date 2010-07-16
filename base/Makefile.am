#----------------------------------------------------------------------------------------------------
# GTSAM base
# provides some base Math and data structures, as well as test-related utilities
#----------------------------------------------------------------------------------------------------

headers =
sources = 
check_PROGRAMS = 

# base Math
sources += Vector.cpp svdcmp.cpp Matrix.cpp
check_PROGRAMS += tests/testVector tests/testMatrix

if USE_LAPACK
sources += SPQRUtil.cpp
check_PROGRAMS += tests/testSPQRUtil
endif

# Testing
headers += Testable.h numericalDerivative.h 

# Lie Groups
headers += Lie.h Lie-inl.h

# Data structures
headers += BTree.h DSF.h
sources += DSFVector.cpp
check_PROGRAMS += tests/testBTree tests/testDSF tests/testDSFVector

# Timing tests
noinst_PROGRAMS = tests/timeMatrix

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_LTLIBRARIES = libbase.la
libbase_la_SOURCES = $(sources)

AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)

if USE_BLAS
AM_CPPFLAGS += -DGT_USE_CBLAS
endif

if USE_LAPACK
AM_CPPFLAGS += -DGT_USE_LAPACK
endif

if USE_LDL
AM_CPPFLAGS += -DGT_USE_LDL
endif

# On Mac, we compile using the BLAS/LAPACK headers in the Accelerate framework 
if USE_ACCELERATE_MACOS
AM_CPPFLAGS += -F Accelerate
endif

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization)
LDADD = libbase.la ../CppUnitLite/libCppUnitLite.a
if USE_LDL
LDADD += libldl.la 
endif 

if USE_BLAS_LINUX
AM_LDFLAGS += -lcblas -latlas  
endif

if USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

if USE_LAPACK_LINUX
AM_LDFLAGS += -llapack
endif

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -framework Accelerate  
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

#----------------------------------------------------------------------------------------------------
