#----------------------------------------------------------------------------------------------------
# GTSAM tests
# More elaborate unit tests that test functionality with slam examples
#----------------------------------------------------------------------------------------------------

check_PROGRAMS = testBayesNetPreconditioner testConstraintOptimizer 
check_PROGRAMS += testGaussianBayesNet testGaussianFactor testGaussianFactorGraph 
check_PROGRAMS += testGaussianISAM testGaussianISAM2 testGraph
check_PROGRAMS += testInference testIterative testGaussianJunctionTree 
check_PROGRAMS += testNonlinearEquality testNonlinearFactor testNonlinearFactorGraph
check_PROGRAMS += testNonlinearOptimizer testSQP testSubgraphPreconditioner
check_PROGRAMS += testSymbolicBayesNet testSymbolicFactorGraph testTupleConfig

# Timing tests
noinst_PROGRAMS = timeGaussianFactorGraph

#----------------------------------------------------------------------------------------------------
# rules to build local library
#----------------------------------------------------------------------------------------------------
noinst_HEADERS = smallExample.h
noinst_LIBRARIES = libtests.a
libtests_a_SOURCES = smallExample.cpp
AM_CPPFLAGS = -I$(boost) -I../colamd 
AM_CPPFLAGS += -I../base -I../geometry -I../inference  -I../linear -I../nonlinear -I../slam 

#----------------------------------------------------------------------------------------------------
# rules to build unit tests
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libtests.a ../libgtsam.la ../CppUnitLite/libCppUnitLite.a  
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

