#----------------------------------------------------------------------------------------------------
# GTSAM nonlinear
# Non-linear optimization
#----------------------------------------------------------------------------------------------------

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
headers += NonlinearOptimizer.h NonlinearOptimizer-inl.h 
headers += NonlinearFactor.h

# Nonlinear constraints 
headers += NonlinearConstraint.h NonlinearConstraint-inl.h
headers += NonlinearEquality.h
check_PROGRAMS += tests/testNonlinearConstraint  

# SQP
if USE_LDL
sources += ConstraintOptimizer.cpp
check_PROGRAMS += tests/testConstraintOptimizer
endif

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_LTLIBRARIES = libnonlinear.la
libnonlinear_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir) -I$(top_srcdir)/colamd -I$(top_srcdir)/base -I$(top_srcdir)/inference -I$(top_srcdir)/linear
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la 
AM_DEFAULT_SOURCE_EXT = .cpp
if USE_LDL
LDADD += libldl.la 
endif 

# rule to run an executable
%.run: % $(LDADD)
	./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

