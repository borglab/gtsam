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

headers += FixedVector.h types.h blockMatrices.h Matrix-inl.h 
sources += Vector.cpp Matrix.cpp 
sources += cholesky.cpp
check_PROGRAMS += tests/testFixedVector tests/testVector tests/testMatrix tests/testBlockMatrices 
check_PROGRAMS += tests/testCholesky
check_PROGRAMS += tests/testNumericalDerivative

#sources += DenseQR.cpp DenseQRUtil.cpp
#check_PROGRAMS += tests/testDenseQRUtil

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
noinst_PROGRAMS = tests/timeMatrix tests/timeVirtual tests/timeTest
#noinst_PROGRAMS += tests/timeublas 

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

AM_CPPFLAGS += $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS += $(boost_serialization) -L$(CCOLAMDLib) -lccolamd
LDADD = libbase.la ../../CppUnitLite/libCppUnitLite.a

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
