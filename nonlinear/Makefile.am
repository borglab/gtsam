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
headers += LieConfig.h LieConfig-inl.h TupleConfig.h TupleConfig-inl.h 
check_PROGRAMS += tests/testLieConfig 

# Nonlinear nonlinear 
headers += NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer-inl.h 
headers += NonlinearFactor.h
sources += NonlinearOptimizer.cpp

# Nonlinear constraints 
headers += NonlinearConstraint.h 
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
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)/..
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la 
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

