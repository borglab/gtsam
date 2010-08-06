#----------------------------------------------------------------------------------------------------
# GTSAM tests
# More elaborate unit tests that test functionality with slam examples
#----------------------------------------------------------------------------------------------------

check_PROGRAMS = testBayesNetPreconditioner 
check_PROGRAMS += testGaussianBayesNet testGaussianFactor testGaussianFactorGraph 
check_PROGRAMS += testGaussianISAM testGraph
check_PROGRAMS += testInference testIterative testGaussianJunctionTree 
check_PROGRAMS += testNonlinearEquality testNonlinearFactor testNonlinearFactorGraph
check_PROGRAMS += testNonlinearOptimizer testSubgraphPreconditioner
check_PROGRAMS += testSymbolicBayesNet testSymbolicFactorGraph testTupleConfig
# check_PROGRAMS += testNonlinearConstraint # disabled until examples rearranged 
check_PROGRAMS += testNonlinearEqualityConstraint

if USE_LDL
check_PROGRAMS += testConstraintOptimizer 
endif

# Timing tests
noinst_PROGRAMS = timeGaussianFactorGraph

#----------------------------------------------------------------------------------------------------
# rules to build unit tests
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)/colamd -I$(top_srcdir)/base -I$(top_srcdir)/geometry 
AM_CPPFLAGS += -I$(top_srcdir) -I$(top_srcdir)/inference  -I$(top_srcdir)/linear -I$(top_srcdir)/nonlinear -I$(top_srcdir)/slam 
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = ../libgtsam.la ../CppUnitLite/libCppUnitLite.a  
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

