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
headers += VectorValues.h 
check_PROGRAMS += tests/testVectorValues
#sources += VectorMap.cpp VectorBTree.cpp  
#check_PROGRAMS += tests/testVectorMap tests/testVectorBTree  

# Solvers
sources += GaussianSequentialSolver.cpp GaussianMultifrontalSolver.cpp

# Gaussian Factor Graphs
headers += GaussianFactorSet.h
sources += GaussianFactor.cpp GaussianFactorGraph.cpp
sources += GaussianJunctionTree.cpp 
sources += GaussianConditional.cpp GaussianBayesNet.cpp
sources += GaussianISAM.cpp
check_PROGRAMS += tests/testGaussianFactor tests/testGaussianConditional
check_PROGRAMS += tests/testGaussianJunctionTree

# Iterative Methods
headers += iterative-inl.h
sources += iterative.cpp SubgraphPreconditioner.cpp
headers += IterativeSolver.h IterativeOptimizationParameters.h 
headers += SubgraphSolver.h SubgraphSolver-inl.h
#sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
#check_PROGRAMS += tests/testBayesNetPreconditioner

# Timing tests
noinst_PROGRAMS = tests/timeGaussianFactor tests/timeFactorOverhead tests/timeSLAMlike
#noinst_PROGRAMS += tests/timeVectorValues

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
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization)
LDADD = liblinear.la ../inference/libinference.la ../base/libbase.la
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

