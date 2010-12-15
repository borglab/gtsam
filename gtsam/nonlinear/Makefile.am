#----------------------------------------------------------------------------------------------------
# GTSAM nonlinear
# Non-linear optimization
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

#----------------------------------------------------------------------------------------------------
# nonlinear
#----------------------------------------------------------------------------------------------------

# Lie Groups
headers += LieValues.h LieValues-inl.h TupleValues.h TupleValues-inl.h 
check_PROGRAMS += tests/testLieValues

# Nonlinear nonlinear
headers += Key.h 
headers += NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer-inl.h NonlinearOptimization.h NonlinearOptimization-inl.h NonlinearOptimizationParameters.h
headers += NonlinearFactor.h
sources += NonlinearOptimizer.cpp Ordering.cpp
check_PROGRAMS += tests/testKey

# Nonlinear iSAM
headers += NonlinearISAM.h NonlinearISAM-inl.h

# Nonlinear constraints 
headers += NonlinearEquality.h

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
nonlineardir = $(pkgincludedir)/nonlinear
nonlinear_HEADERS = $(headers)
noinst_LTLIBRARIES = libnonlinear.la
libnonlinear_la_SOURCES = $(sources)
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization)
LDADD = libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la
LDADD += ../../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
endif

