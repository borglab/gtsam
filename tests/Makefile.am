#----------------------------------------------------------------------------------------------------
# GTSAM tests
# More elaborate unit tests that test functionality with slam examples
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

check_PROGRAMS =
#check_PROGRAMS = testBayesNetPreconditioner testSubgraphPreconditioner 
check_PROGRAMS += testGaussianBayesNet testGaussianFactor testGaussianFactorGraph
check_PROGRAMS += testGaussianISAM 
check_PROGRAMS += testGraph
check_PROGRAMS += testInference
#check_PROGRAMS += testIterative
check_PROGRAMS += testGaussianJunctionTree 
check_PROGRAMS += testNonlinearEquality testNonlinearFactor testNonlinearFactorGraph
check_PROGRAMS += testNonlinearOptimizer
check_PROGRAMS += testSymbolicBayesNet testSymbolicFactorGraph
check_PROGRAMS += testTupleValues

# only if serialization is available
if ENABLE_SERIALIZATION
check_PROGRAMS += testSerialization
endif

# Timing tests
noinst_PROGRAMS = timeGaussianFactorGraph timeLinearOnDataset

#----------------------------------------------------------------------------------------------------
# rules to build unit tests
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_CPPFLAGS = -I$(boost) -I$(CCOLAMDInc) -I$(top_srcdir)/..
AM_LDFLAGS = $(BOOST_LDFLAGS) 

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif

# link to serialization library for test 
if ENABLE_SERIALIZATION
AM_LDFLAGS += -lboost_serialization
endif

LDADD = ../libgtsam.la ../CppUnitLite/libCppUnitLite.a  
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
