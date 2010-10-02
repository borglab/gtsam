#----------------------------------------------------------------------------------------------------
# GTSAM linear: inference in Gaussian factor graphs
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

# Noise Model
headers += SharedGaussian.h SharedDiagonal.h 
sources += NoiseModel.cpp Errors.cpp
check_PROGRAMS += tests/testNoiseModel tests/testErrors

# Vector Configurations
headers += VectorConfig.h 
sources += VectorMap.cpp VectorBTree.cpp  
check_PROGRAMS += tests/testVectorMap tests/testVectorBTree  

# Gaussian Factor Graphs
headers += GaussianFactorSet.h Factorization.h
sources += GaussianFactor.cpp GaussianFactorGraph.cpp
sources += GaussianJunctionTree.cpp 
sources += GaussianConditional.cpp GaussianBayesNet.cpp
sources += GaussianISAM.cpp
check_PROGRAMS += tests/testGaussianFactor tests/testGaussianJunctionTree tests/testGaussianConditional

# Iterative Methods
headers += iterative-inl.h SubgraphSolver.h SubgraphSolver-inl.h
sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
check_PROGRAMS += tests/testBayesNetPreconditioner

# Timing tests
noinst_PROGRAMS = tests/timeGaussianFactor tests/timeVectorConfig

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
lineardir = $(pkgincludedir)/linear
linear_HEADERS = $(headers)
noinst_LTLIBRARIES = liblinear.la
liblinear_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)/..
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = liblinear.la ../inference/libinference.la ../base/libbase.la 
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

