#----------------------------------------------------------------------------------------------------
# GTSAM tests
# More elaborate unit tests that test functionality with slam examples
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

check_PROGRAMS = testBayesNetPreconditioner 
check_PROGRAMS += testGaussianBayesNet testGaussianFactor testGaussianFactorGraph 
check_PROGRAMS += testGaussianISAM testGraph
check_PROGRAMS += testInference testIterative testGaussianJunctionTree 
check_PROGRAMS += testNonlinearEquality testNonlinearFactor testNonlinearFactorGraph
check_PROGRAMS += testNonlinearOptimizer testSubgraphPreconditioner
check_PROGRAMS += testSymbolicBayesNet testSymbolicFactorGraph testTupleConfig
check_PROGRAMS += testNonlinearEqualityConstraint testBoundingConstraint 
check_PROGRAMS += testTransformConstraint testLinearApproxFactor

# experimental
#check_PROGRAMS += testFusionTupleConfig

if USE_LDL
check_PROGRAMS += testConstraintOptimizer 
endif

# Timing tests
noinst_PROGRAMS = timeGaussianFactorGraph

#----------------------------------------------------------------------------------------------------
# rules to build unit tests
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_CPPFLAGS = -I$(boost) -I$(BORG_SRCROOT)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = ../libgtsam.la ../CppUnitLite/libCppUnitLite.a  
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

