#----------------------------------------------------------------------------------------------------
# GTSAM tests
# More elaborate unit tests that test functionality with slam examples
#----------------------------------------------------------------------------------------------------

check_PROGRAMS = testBayesNetPreconditioner testConstraintOptimizer 
check_PROGRAMS += testGaussianBayesNet testGaussianFactor testGaussianFactorGraph 
check_PROGRAMS += testGaussianISAM testGraph
check_PROGRAMS += testInference testIterative testGaussianJunctionTree 
check_PROGRAMS += testNonlinearEquality testNonlinearFactor testNonlinearFactorGraph
check_PROGRAMS += testNonlinearOptimizer testSQP testSubgraphPreconditioner
check_PROGRAMS += testSymbolicBayesNet testSymbolicFactorGraph testTupleConfig

# Timing tests
noinst_PROGRAMS = timeGaussianFactorGraph

#----------------------------------------------------------------------------------------------------
# rules to build unit tests
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_CPPFLAGS = -I$(boost) -I../colamd 
AM_CPPFLAGS += -I../base -I../geometry -I../inference  -I../linear -I../nonlinear -I../slam 
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = ../libgtsam.la ../CppUnitLite/libCppUnitLite.a  
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

