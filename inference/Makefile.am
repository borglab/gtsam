#----------------------------------------------------------------------------------------------------
# GTSAM core functionality: discrete, linear, and non-linear
#----------------------------------------------------------------------------------------------------

headers =
sources = 
check_PROGRAMS = 

#----------------------------------------------------------------------------------------------------
# base
#----------------------------------------------------------------------------------------------------

# GTSAM core
headers += Factor.h Conditional.h Ordering.h IndexTable.h 
sources += Ordering.cpp 
check_PROGRAMS += testOrdering 

# Symbolic Inference
headers += SymbolicConditional.h
sources += SymbolicFactor.cpp SymbolicFactorGraph.cpp SymbolicBayesNet.cpp
check_PROGRAMS += testSymbolicFactor testSymbolicFactorGraph testSymbolicBayesNet  

# Inference
headers += inference.h inference-inl.h
headers += graph.h graph-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += JunctionTree.h JunctionTree-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += ISAM.h ISAM-inl.h GaussianISAM.h
headers += ISAM2.h ISAM2-inl.h
sources += GaussianISAM.cpp
check_PROGRAMS += testFactorGraph testJunctionTree testOrdering
check_PROGRAMS += testBayesTree testISAM

#----------------------------------------------------------------------------------------------------
# discrete
#----------------------------------------------------------------------------------------------------

# Binary Inference
headers += BinaryConditional.h
check_PROGRAMS += testBinaryBayesNet

#----------------------------------------------------------------------------------------------------
# linear
#----------------------------------------------------------------------------------------------------

# Gaussian inference 
headers += GaussianFactorSet.h SharedGaussian.h SharedDiagonal.h VectorConfig.h Factorization.h
sources += NoiseModel.cpp Errors.cpp VectorMap.cpp VectorBTree.cpp GaussianFactor.cpp 
sources += GaussianFactorGraph.cpp GaussianConditional.cpp GaussianBayesNet.cpp
check_PROGRAMS += testVectorMap testVectorBTree testGaussianFactor 
check_PROGRAMS += testGaussianConditional testNoiseModel testErrors

# Iterative Methods
headers += iterative-inl.h SubgraphSolver.h SubgraphSolver-inl.h
sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
check_PROGRAMS += testBayesNetPreconditioner

# Timing tests
noinst_PROGRAMS = timeGaussianFactor timeSymbolMaps timeVectorConfig

#----------------------------------------------------------------------------------------------------
# nonlinear
#----------------------------------------------------------------------------------------------------

# Lie Groups
headers += LieConfig.h LieConfig-inl.h TupleConfig.h TupleConfig-inl.h
check_PROGRAMS += testLieConfig 

# Nonlinear inference 
headers += Key.h SymbolMap.h NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer.h NonlinearOptimizer-inl.h 
headers += NonlinearFactor.h
check_PROGRAMS += testKey

# Nonlinear constraints 
headers += NonlinearConstraint.h NonlinearConstraint-inl.h
headers += NonlinearEquality.h
check_PROGRAMS += testNonlinearConstraint  

# SQP
sources += ConstraintOptimizer.cpp
check_PROGRAMS += testConstraintOptimizer

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_HEADERS = gtsam.h
noinst_LTLIBRARIES = libcpp.la
libcpp_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I../colamd -I../base 

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libcpp.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la ../ldl/libldl.la 
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

#----------------------------------------------------------------------------------------------------
# OLD, need to figure where these go!
#----------------------------------------------------------------------------------------------------

AM_CXXFLAGS = 

# enable debug if --enable-debug is set in configure
if DEBUG
AM_CXXFLAGS += -g
endif

if USE_PROFILING
AM_CXXFLAGS += -pg
AM_LDFLAGS += -pg
endif

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

