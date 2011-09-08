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
headers += SharedGaussian.h SharedDiagonal.h  SharedNoiseModel.h
sources += NoiseModel.cpp Errors.cpp Sampler.cpp
check_PROGRAMS += tests/testNoiseModel tests/testErrors tests/testSampler

# Vector Configurations
sources += VectorValues.cpp 
check_PROGRAMS += tests/testVectorValues

# Solvers
sources += GaussianSequentialSolver.cpp GaussianMultifrontalSolver.cpp

# Gaussian Factor Graphs
sources += JacobianFactor.cpp HessianFactor.cpp
sources += GaussianFactor.cpp GaussianFactorGraph.cpp
sources += GaussianJunctionTree.cpp 
sources += GaussianConditional.cpp GaussianBayesNet.cpp
sources += GaussianISAM.cpp
check_PROGRAMS += tests/testHessianFactor tests/testJacobianFactor tests/testGaussianConditional
check_PROGRAMS += tests/testGaussianFactorGraph tests/testGaussianJunctionTree

# Kalman Filter
sources += KalmanFilter.cpp
check_PROGRAMS += tests/testKalmanFilter

# Iterative Methods
headers += iterative-inl.h
sources += iterative.cpp SubgraphPreconditioner.cpp
headers += IterativeSolver.h IterativeOptimizationParameters.h 
headers += SubgraphSolver.h SubgraphSolver-inl.h

# Timing tests
noinst_PROGRAMS = tests/timeGaussianFactor tests/timeFactorOverhead tests/timeSLAMlike

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
AM_CPPFLAGS = $(BOOST_CPPFLAGS)  -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization)
LDADD = liblinear.la ../inference/libinference.la ../base/libbase.la ../3rdparty/libccolamd.la
LDADD += ../../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

