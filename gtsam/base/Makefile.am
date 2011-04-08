#----------------------------------------------------------------------------------------------------
# GTSAM base
# provides some base Math and data structures, as well as test-related utilities
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

# base Math

headers += FixedVector.h types.h blockMatrices.h Matrix-inl.h lapack.h
sources += Vector.cpp Matrix.cpp cholesky.cpp
check_PROGRAMS += tests/testFixedVector tests/testVector tests/testMatrix tests/testCholesky
check_PROGRAMS += tests/testNumericalDerivative

if USE_LAPACK
sources += DenseQR.cpp DenseQRUtil.cpp
check_PROGRAMS += tests/testDenseQRUtil
endif

# Testing
headers += Testable.h TestableAssertions.h numericalDerivative.h
sources += timing.cpp debug.cpp
check_PROGRAMS += tests/testDebug tests/testTestableAssertions

# Lie Groups
headers += Lie.h Lie-inl.h lieProxies.h LieScalar.h
sources += LieVector.cpp
check_PROGRAMS += tests/testLieVector tests/testLieScalar

# Data structures
headers += BTree.h DSF.h FastMap.h FastSet.h FastList.h FastVector.h
sources += DSFVector.cpp
check_PROGRAMS += tests/testBTree tests/testDSF tests/testDSFVector

# Timing tests
noinst_PROGRAMS = tests/timeMatrix tests/timeublas tests/timeVirtual tests/timeTest

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
basedir = $(pkgincludedir)/base
base_HEADERS = $(headers)
noinst_LTLIBRARIES = libbase.la
libbase_la_SOURCES = $(sources)

AM_CPPFLAGS =

# On Mac, we compile using the BLAS/LAPACK headers in the Accelerate framework 
if USE_ACCELERATE_MACOS
AM_CPPFLAGS += -I/System/Library/Frameworks/vecLib.framework/Headers
endif

AM_CPPFLAGS += $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)

if USE_BLAS
AM_CPPFLAGS += -DGT_USE_CBLAS
endif

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS += $(boost_serialization) -L$(CCOLAMDLib) -lccolamd
LDADD = libbase.la ../../CppUnitLite/libCppUnitLite.a

if USE_BLAS_LINUX
AM_LDFLAGS += -lcblas -latlas  
endif

if USE_LAPACK
AM_CPPFLAGS += -DGT_USE_LAPACK
endif

if USE_LAPACK_LINUX
AM_LDFLAGS += -llapack
endif

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
