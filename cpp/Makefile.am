
# the install destination
includedir = ${prefix}/include/gtsam
libdir = ${exec_prefix}/lib

# library version
current = 0  # The most recent interface number that this library implements. 
revision = 0 # The implementation number of the current interface 
age = 0 # The difference between the newest and oldest interfaces that	\
this library implements. In other words, the library implements all	\
the interface numbers in the range from number current - age to		\
current.

# from libtool manual:
# Here are a set of rules to help you update your library version information:
#   Start with version information of ‘0:0:0’ for each libtool library.
#   - Update the version information only immediately before a public
#   release of your software.
#   - If the library source code has changed at all since the last
#   update, then increment revision
#   - If any interfaces have been added, removed, or changed since the
#   last update, increment current, and set revision to 0.
#   - If any interfaces have been added since the last public release,
#   then increment age.
#   - If any interfaces have been removed since the last public release,
#   then set age to 0.
version = $(current):$(revision):$(age)

external_libs:
	@echo Compiling CppUnitLite...; cd ../CppUnitLite; make all
	@echo Compiling Colamd...;      cd ../colamd; make all

clean : clean_external_libs
	@echo Cleaning Cpp...;  rm -f *.o *.lo *.*~ libgtsam.la $(check_PROGRAMS)

clean_external_libs:
	@echo Cleaning CppUnitLite...; cd ../CppUnitLite; make clean
	@echo Cleaning Colamd...; cd ../colamd; make clean

# we specify the library in levels

# basic Math
sources = Vector.cpp svdcmp.cpp Matrix.cpp 
check_PROGRAMS = testVector testMatrix 
testVector_SOURCES = testVector.cpp
testVector_LDADD   = libgtsam.la
testMatrix_SOURCES = testMatrix.cpp
testMatrix_LDADD   = libgtsam.la

# GTSAM basics
# The header files will be installed in ~/include/gtsam
headers = gtsam.h Value.h Testable.h Factor.h Conditional.h 
sources += Ordering.cpp  
example = smallExample.cpp 

# Symbolic Inference
headers += SymbolicConditional.h
sources += SymbolicFactor.cpp SymbolicFactorGraph.cpp SymbolicBayesNet.cpp
check_PROGRAMS += testSymbolicFactor testSymbolicFactorGraph testSymbolicBayesNet  
testSymbolicFactor_SOURCES      = $(example) testSymbolicFactor.cpp
testSymbolicFactor_LDADD        = libgtsam.la
testSymbolicFactorGraph_SOURCES = $(example) testSymbolicFactorGraph.cpp
testSymbolicFactorGraph_LDADD   = libgtsam.la
testSymbolicBayesNet_SOURCES    = $(example) testSymbolicBayesNet.cpp
testSymbolicBayesNet_LDADD      = libgtsam.la

# Inference
headers += inference.h inference-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += ISAM.h ISAM-inl.h GaussianISAM.h
headers += ISAM2.h ISAM2-inl.h GaussianISAM2.h
sources += GaussianISAM.cpp GaussianISAM2.cpp
check_PROGRAMS += testFactorgraph testInference testOrdering
check_PROGRAMS += testBayesTree testISAM testGaussianISAM testGaussianISAM2
testFactorgraph_SOURCES        = testFactorgraph.cpp
testInference_SOURCES          = $(example) testInference.cpp
testFactorgraph_LDADD          = libgtsam.la
testInference_LDADD            = libgtsam.la
testOrdering_SOURCES   		   = testOrdering.cpp
testOrdering_LDADD			   = libgtsam.la
testBayesTree_SOURCES          = $(example) testBayesTree.cpp
testBayesTree_LDADD            = libgtsam.la
testGaussianISAM_SOURCES       = $(example) testGaussianISAM.cpp
testGaussianISAM_LDADD         = libgtsam.la
testGaussianISAM2_SOURCES      = $(example) testGaussianISAM2.cpp
testGaussianISAM2_LDADD        = libgtsam.la
testISAM_SOURCES               = $(example) testISAM.cpp
testISAM_LDADD                 = libgtsam.la

# Binary Inference
headers += BinaryConditional.h
check_PROGRAMS += testBinaryBayesNet
testBinaryBayesNet_SOURCES    = testBinaryBayesNet.cpp
testBinaryBayesNet_LDADD      = libgtsam.la 

# Gaussian inference 
headers += GaussianFactorSet.h
sources += Errors.cpp VectorConfig.cpp GaussianFactor.cpp GaussianFactorGraph.cpp GaussianConditional.cpp GaussianBayesNet.cpp
check_PROGRAMS += testVectorConfig testGaussianFactor testGaussianFactorGraph testGaussianConditional testGaussianBayesNet
testVectorConfig_SOURCES        = testVectorConfig.cpp
testVectorConfig_LDADD          = libgtsam.la
testGaussianFactor_SOURCES      = $(example) testGaussianFactor.cpp
testGaussianFactor_LDADD        = libgtsam.la
testGaussianFactorGraph_SOURCES = $(example) testGaussianFactorGraph.cpp
testGaussianFactorGraph_LDADD   = libgtsam.la
testGaussianConditional_SOURCES = $(example) testGaussianConditional.cpp
testGaussianConditional_LDADD   = libgtsam.la
testGaussianBayesNet_SOURCES    = $(example) testGaussianBayesNet.cpp
testGaussianBayesNet_LDADD      = libgtsam.la

# Iterative Methods
headers += iterative-inl.h
sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
check_PROGRAMS += testIterative testBayesNetPreconditioner testSubgraphPreconditioner
testIterative_SOURCES              = $(example) testIterative.cpp
testIterative_LDADD                = libgtsam.la
testBayesNetPreconditioner_SOURCES = $(example) testBayesNetPreconditioner.cpp
testBayesNetPreconditioner_LDADD   = libgtsam.la
testSubgraphPreconditioner_SOURCES = $(example) testSubgraphPreconditioner.cpp
testSubgraphPreconditioner_LDADD   = libgtsam.la

# the timing tests below are broken for now.
#noinst_PROGRAMS = timeGaussianFactor timeGaussianFactorGraph

#timeGaussianFactor_SOURCES = timeGaussianFactor.cpp
#timeGaussianFactorGraph_SOURCES = timeGaussianFactorGraph.cpp

#timeGaussianFactor_LDADD = libgtsam.la 
#timeGaussianFactorGraph_LDADD = libgtsam.la

# Nonlinear inference 
headers += NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer.h NonlinearOptimizer-inl.h 
sources += NonlinearFactor.cpp 
check_PROGRAMS += testNonlinearFactor testNonlinearFactorGraph testNonlinearOptimizer
testNonlinearFactor_SOURCES      = $(example) testNonlinearFactor.cpp
testNonlinearFactor_LDADD        = libgtsam.la
testNonlinearFactorGraph_SOURCES = $(example) testNonlinearFactorGraph.cpp
testNonlinearFactorGraph_LDADD   = libgtsam.la
testNonlinearOptimizer_SOURCES   = $(example) testNonlinearOptimizer.cpp
testNonlinearOptimizer_LDADD     = libgtsam.la

# Nonlinear constraints 
headers += SQPOptimizer.h SQPOptimizer-inl.h  
headers += NonlinearConstraint.h NonlinearConstraint-inl.h
headers += NonlinearEquality.h
check_PROGRAMS += testNonlinearConstraint testNonlinearEquality testSQP testSQPOptimizer
testNonlinearConstraint_SOURCES = testNonlinearConstraint.cpp
testNonlinearConstraint_LDADD   = libgtsam.la
testNonlinearEquality_SOURCES   = testNonlinearEquality.cpp
testNonlinearEquality_LDADD     = libgtsam.la
testSQP_SOURCES                 = $(example) testSQP.cpp
testSQP_LDADD                   = libgtsam.la
testSQPOptimizer_SOURCES   		  = $(example) testSQPOptimizer.cpp
testSQPOptimizer_LDADD     		  = libgtsam.la

# geometry
headers += Lie.h Lie-inl.h
sources += Point2.cpp Rot2.cpp Pose2.cpp Point3.cpp Rot3.cpp Pose3.cpp Cal3_S2.cpp
check_PROGRAMS += testPoint2 testRot2 testPose2 testPoint3 testRot3 testPose3 testCal3_S2 testLieConfig
testPoint2_SOURCES  = testPoint2.cpp
testRot2_SOURCES    = testRot2.cpp
testPose2_SOURCES   = testPose2.cpp
testPoint3_SOURCES  = testPoint3.cpp
testRot3_SOURCES    = testRot3.cpp
testPose3_SOURCES   = testPose3.cpp
testCal3_S2_SOURCES = testCal3_S2.cpp
testLieConfig_SOURCES = testLieConfig.cpp               

testPoint2_LDADD    = libgtsam.la
testRot2_LDADD      = libgtsam.la
testPose2_LDADD     = libgtsam.la
testPoint3_LDADD    = libgtsam.la
testRot3_LDADD      = libgtsam.la
testPose3_LDADD     = libgtsam.la
testCal3_S2_LDADD   = libgtsam.la
testLieConfig_LDADD = libgtsam.la

noinst_PROGRAMS = timeRot3
timeRot3_SOURCES = timeRot3.cpp
timeRot3_LDADD = libgtsam.la

# simulated2D example
sources += simulated2D.cpp 
testSimulated2D_SOURCES = testSimulated2D.cpp
testSimulated2D_LDADD = libgtsam.la
check_PROGRAMS += testSimulated2D 

# simulated3D example
sources += Simulated3D.cpp 
testSimulated3D_SOURCES = testSimulated3D.cpp
testSimulated3D_LDADD = libgtsam.la
check_PROGRAMS += testSimulated3D 

# Pose SLAM headers
headers += BetweenFactor.h LieConfig.h LieConfig-inl.h 

# 2D Pose constraints
headers += Pose2Factor.h Pose2Prior.h 
sources += Pose2Config.cpp Pose2Graph.cpp 
check_PROGRAMS += testPose2Factor testPose2Config testPose2Graph 
testPose2Factor_SOURCES = $(example) testPose2Factor.cpp
testPose2Factor_LDADD   = libgtsam.la
testPose2Config_SOURCES = $(example) testPose2Config.cpp
testPose2Config_LDADD   = libgtsam.la
testPose2Graph_SOURCES  = $(example) testPose2Graph.cpp
testPose2Graph_LDADD    = libgtsam.la

# 3D Pose constraints
headers += Pose3Factor.h  
sources += Pose3Config.cpp Pose3Graph.cpp 
check_PROGRAMS += testPose3Factor testPose3Config testPose3Graph 
testPose3Factor_SOURCES = $(example) testPose3Factor.cpp
testPose3Factor_LDADD   = libgtsam.la
testPose3Config_SOURCES = $(example) testPose3Config.cpp
testPose3Config_LDADD   = libgtsam.la
testPose3Graph_SOURCES  = $(example) testPose3Graph.cpp
testPose3Graph_LDADD    = libgtsam.la

# Cameras
sources += CalibratedCamera.cpp SimpleCamera.cpp
check_PROGRAMS += testCalibratedCamera testSimpleCamera
testCalibratedCamera_SOURCES = testCalibratedCamera.cpp
testCalibratedCamera_LDADD = libgtsam.la
testSimpleCamera_SOURCES = testSimpleCamera.cpp
testSimpleCamera_LDADD = libgtsam.la

# Visual SLAM
sources += VSLAMConfig.cpp VSLAMGraph.cpp VSLAMFactor.cpp
check_PROGRAMS += testVSLAMFactor testVSLAMGraph testVSLAMConfig
testVSLAMFactor_SOURCES = testVSLAMFactor.cpp
testVSLAMFactor_LDADD = libgtsam.la
testVSLAMGraph_SOURCES = testVSLAMGraph.cpp
testVSLAMGraph_LDADD = libgtsam.la
testVSLAMConfig_SOURCES = testVSLAMConfig.cpp
testVSLAMConfig_LDADD = libgtsam.la

headers += Point2Prior.h Simulated2DOdometry.h Simulated2DMeasurement.h smallExample.h 
headers += $(sources:.cpp=.h)

# create both dynamic and static libraries
AM_CXXFLAGS = -I$(boost) -fPIC
lib_LTLIBRARIES = libgtsam.la
libgtsam_la_SOURCES = $(sources)
libgtsam_la_CPPFLAGS = $(AM_CXXFLAGS)
libgtsam_la_LDFLAGS = -version-info $(version) -L../colamd -lcolamd #-lboost_serialization-mt

# enable debug if --enable-debug is set in configure
if DEBUG
     AM_CXXFLAGS += -g
endif

# install the header files
include_HEADERS = $(headers)

AM_CXXFLAGS += -I.. 
AM_LDFLAGS = -L../CppUnitLite -lCppUnitLite $(BOOST_LDFLAGS) $(boost_serialization) #-L. -lgtsam

TESTS = $(check_PROGRAMS)
CXXLINK = $(LIBTOOL) $(AM_LIBTOOLFLAGS) $(LIBTOOLFLAGS) --mode=link \
	$(CXXLD) -g $(AM_CXXFLAGS) $(CXXFLAGS) $(AM_LDFLAGS) $(LDFLAGS) \
	-o $@ # CXXLINK is only used for unit tests

# rule to run an executable
%.run: % libgtsam.la
	./$^

# to compile colamd and cppunitlite first when make all and make install. 
# The part after external_libs is copied from automatically generated Makefile when without the following line
libgtsam.la: external_libs $(libgtsam_la_OBJECTS) $(libgtsam_la_DEPENDENCIES) 
	$(libgtsam_la_LINK) -rpath $(libdir) $(libgtsam_la_OBJECTS) $(libgtsam_la_LIBADD) $(LIBS)



