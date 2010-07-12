#----------------------------------------------------------------------------------------------------
# GTSAM linear: inference in Gaussian factor graphs
#----------------------------------------------------------------------------------------------------

headers =
sources = 
check_PROGRAMS = 

# Noise Model
headers += SharedGaussian.h SharedDiagonal.h 
sources += NoiseModel.cpp Errors.cpp
check_PROGRAMS += testNoiseModel testErrors

# Vector Configurations
headers += VectorConfig.h 
sources += VectorMap.cpp VectorBTree.cpp  
check_PROGRAMS += testVectorMap testVectorBTree  

# Gaussian Factor Graphs
headers += GaussianFactorSet.h Factorization.h
sources += GaussianFactor.cpp GaussianFactorGraph.cpp
headers += GaussianJunctionTree.h GaussianJunctionTree-inl.h
sources += GaussianConditional.cpp GaussianBayesNet.cpp
sources += GaussianISAM.cpp
check_PROGRAMS += testGaussianFactor testGaussianJunctionTree testGaussianConditional

# Iterative Methods
headers += iterative-inl.h SubgraphSolver.h SubgraphSolver-inl.h
sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
check_PROGRAMS += testBayesNetPreconditioner

# Timing tests
noinst_PROGRAMS = timeGaussianFactor timeVectorConfig

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_LTLIBRARIES = liblinear.la
liblinear_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I../colamd -I../base -I../inference
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = liblinear.la ../inference/libinference.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la ../ldl/libldl.la 
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

