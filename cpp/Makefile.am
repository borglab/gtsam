
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
headers += Ordering.h IndexTable.h numericalDerivative.h
headers += BTree.h DSF.h
sources += Ordering.cpp smallExample.cpp 
check_PROGRAMS += testOrdering testBTree testDSF
testOrdering_SOURCES = testOrdering.cpp
testOrdering_LDADD   = libgtsam.la
testBTree_SOURCES    = testBTree.cpp
testBTree_LDADD		   = libgtsam.la
testDSF_SOURCES    = testDSF.cpp
testDSF_LDADD      = libgtsam.la

# Symbolic Inference
headers += SymbolicConditional.h
sources += SymbolicFactor.cpp SymbolicFactorGraph.cpp SymbolicBayesNet.cpp
check_PROGRAMS += testSymbolicFactor testSymbolicFactorGraph testSymbolicBayesNet  
testSymbolicFactor_SOURCES      = testSymbolicFactor.cpp
testSymbolicFactor_LDADD        = libgtsam.la
testSymbolicFactorGraph_SOURCES = testSymbolicFactorGraph.cpp
testSymbolicFactorGraph_LDADD   = libgtsam.la
testSymbolicBayesNet_SOURCES    = testSymbolicBayesNet.cpp
testSymbolicBayesNet_LDADD      = libgtsam.la

# Inference
headers += inference.h inference-inl.h
headers += graph.h graph-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += ISAM.h ISAM-inl.h GaussianISAM.h
headers += ISAM2.h ISAM2-inl.h GaussianISAM2.h
sources += GaussianISAM.cpp GaussianISAM2.cpp
check_PROGRAMS += testGraph testFactorGraph testInference testOrdering
check_PROGRAMS += testBayesTree testISAM testGaussianISAM testGaussianISAM2
testGraph_SOURCES              = testGraph.cpp
testGraph_LDADD                = libgtsam.la
testFactorGraph_SOURCES        = testFactorgraph.cpp
testInference_SOURCES          = testInference.cpp
testFactorGraph_LDADD          = libgtsam.la
testInference_LDADD            = libgtsam.la
testBayesTree_SOURCES          = testBayesTree.cpp
testBayesTree_LDADD            = libgtsam.la
testGaussianISAM_SOURCES       = testGaussianISAM.cpp
testGaussianISAM_LDADD         = libgtsam.la
testGaussianISAM2_SOURCES      = testGaussianISAM2.cpp
testGaussianISAM2_LDADD        = libgtsam.la
testISAM_SOURCES               = testISAM.cpp
testISAM_LDADD                 = libgtsam.la

# Binary Inference
headers += BinaryConditional.h
check_PROGRAMS += testBinaryBayesNet
testBinaryBayesNet_SOURCES    = testBinaryBayesNet.cpp
testBinaryBayesNet_LDADD      = libgtsam.la 

# Gaussian inference 
headers += GaussianFactorSet.h SharedGaussian.h SharedDiagonal.h VectorConfig.h Factorization.h
sources += NoiseModel.cpp Errors.cpp VectorMap.cpp VectorBTree.cpp GaussianFactor.cpp 
sources += GaussianFactorGraph.cpp GaussianConditional.cpp GaussianBayesNet.cpp
check_PROGRAMS += testVectorMap testVectorBTree testGaussianFactor testGaussianFactorGraph 
check_PROGRAMS += testGaussianConditional testGaussianBayesNet testNoiseModel testErrors
testVectorMap_SOURCES           = testVectorMap.cpp
testVectorMap_LDADD             = libgtsam.la
testVectorBTree_SOURCES         = testVectorBTree.cpp
testVectorBTree_LDADD           = libgtsam.la
testGaussianFactor_SOURCES      = testGaussianFactor.cpp
testGaussianFactor_LDADD        = libgtsam.la
testGaussianFactorGraph_SOURCES = testGaussianFactorGraph.cpp
testGaussianFactorGraph_LDADD   = libgtsam.la
testGaussianConditional_SOURCES = testGaussianConditional.cpp
testGaussianConditional_LDADD   = libgtsam.la
testGaussianBayesNet_SOURCES    = testGaussianBayesNet.cpp
testGaussianBayesNet_LDADD      = libgtsam.la
testNoiseModel_SOURCES          = testNoiseModel.cpp
testNoiseModel_LDADD            = libgtsam.la
testErrors_SOURCES              = testErrors.cpp
testErrors_LDADD                = libgtsam.la

# Iterative Methods
headers += iterative-inl.h SubgraphSolver.h SubgraphSolver-inl.h
sources += iterative.cpp BayesNetPreconditioner.cpp SubgraphPreconditioner.cpp
check_PROGRAMS += testIterative testBayesNetPreconditioner testSubgraphPreconditioner
testIterative_SOURCES              = testIterative.cpp
testIterative_LDADD                = libgtsam.la
testBayesNetPreconditioner_SOURCES = testBayesNetPreconditioner.cpp
testBayesNetPreconditioner_LDADD   = libgtsam.la
testSubgraphPreconditioner_SOURCES = testSubgraphPreconditioner.cpp
testSubgraphPreconditioner_LDADD   = libgtsam.la

# Nonlinear inference 
headers += Key.h SymbolMap.h NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer.h NonlinearOptimizer-inl.h 
headers += NonlinearFactor.h
check_PROGRAMS += testNonlinearFactor testNonlinearFactorGraph testNonlinearOptimizer testKey
testNonlinearFactor_SOURCES      = testNonlinearFactor.cpp
testNonlinearFactor_LDADD        = libgtsam.la
testNonlinearFactorGraph_SOURCES = testNonlinearFactorGraph.cpp
testNonlinearFactorGraph_LDADD   = libgtsam.la
testNonlinearOptimizer_SOURCES   = testNonlinearOptimizer.cpp
testNonlinearOptimizer_LDADD     = libgtsam.la
testKey_SOURCES   				 = testKey.cpp
testKey_LDADD     				 = libgtsam.la

# Nonlinear constraints 
headers += NonlinearConstraint.h NonlinearConstraint-inl.h
headers += NonlinearEquality.h
check_PROGRAMS += testNonlinearConstraint testNonlinearEquality 
testNonlinearConstraint_SOURCES = testNonlinearConstraint.cpp
testNonlinearConstraint_LDADD   = libgtsam.la
testNonlinearEquality_SOURCES   = testNonlinearEquality.cpp
testNonlinearEquality_LDADD     = libgtsam.la

# SQP
check_PROGRAMS += testSQP 
testSQP_SOURCES                 = $(example) testSQP.cpp
testSQP_LDADD                   = libgtsam.la

# geometry
headers += Lie.h Lie-inl.h
sources += Point2.cpp Rot2.cpp Pose2.cpp Point3.cpp Rot3.cpp Pose3.cpp Cal3_S2.cpp
check_PROGRAMS += testPoint2 testRot2 testPose2 testPoint3 testRot3 testPose3 testCal3_S2 testLieConfig testTupleConfig
testPoint2_SOURCES  = testPoint2.cpp
testRot2_SOURCES    = testRot2.cpp
testPose2_SOURCES   = testPose2.cpp
testPoint3_SOURCES  = testPoint3.cpp
testRot3_SOURCES    = testRot3.cpp
testPose3_SOURCES   = testPose3.cpp
testCal3_S2_SOURCES = testCal3_S2.cpp
testLieConfig_SOURCES = testLieConfig.cpp               
testTupleConfig_SOURCES = testTupleConfig.cpp               

testPoint2_LDADD    = libgtsam.la
testRot2_LDADD      = libgtsam.la
testPose2_LDADD     = libgtsam.la
testPoint3_LDADD    = libgtsam.la
testRot3_LDADD      = libgtsam.la
testPose3_LDADD     = libgtsam.la
testCal3_S2_LDADD   = libgtsam.la
testLieConfig_LDADD = libgtsam.la
testTupleConfig_LDADD = libgtsam.la

# simulated2D example
headers += simulated2D.h simulated2DOriented.h
headers += Simulated2DConfig.h Simulated2DOrientedConfig.h
headers += Simulated2DPosePrior.h Simulated2DPointPrior.h Simulated2DOrientedPosePrior.h
headers += Simulated2DOdometry.h Simulated2DMeasurement.h Simulated2DOrientedOdometry.h
sources += simulated2D.cpp simulated2DOriented.cpp 
testSimulated2D_SOURCES = testSimulated2D.cpp
testSimulated2D_LDADD = libgtsam.la
check_PROGRAMS += testSimulated2D 

# simulated3D example
sources += Simulated3D.cpp 
testSimulated3D_SOURCES = testSimulated3D.cpp
testSimulated3D_LDADD = libgtsam.la
check_PROGRAMS += testSimulated3D 

# Pose SLAM headers
headers += BetweenFactor.h PriorFactor.h 
headers += LieConfig.h LieConfig-inl.h TupleConfig.h TupleConfig-inl.h

# 2D Pose SLAM
headers +=  
sources += pose2SLAM.cpp Pose2SLAMOptimizer.cpp dataset.cpp
check_PROGRAMS += testPose2Factor testPose2Config testPose2SLAM testPose2Prior
testPose2Prior_SOURCES  = testPose2Prior.cpp
testPose2Prior_LDADD    = libgtsam.la
testPose2Factor_SOURCES = testPose2Factor.cpp
testPose2Factor_LDADD   = libgtsam.la
testPose2Config_SOURCES = testPose2Config.cpp
testPose2Config_LDADD   = libgtsam.la
testPose2SLAM_SOURCES  = testPose2SLAM.cpp
testPose2SLAM_LDADD    = libgtsam.la

# 2D SLAM using Bearing and Range
headers += BearingFactor.h RangeFactor.h BearingRangeFactor.h
sources += planarSLAM.cpp
check_PROGRAMS += testPlanarSLAM
testPlanarSLAM_SOURCES = testPlanarSLAM.cpp
testPlanarSLAM_LDADD   = libgtsam.la

# 3D Pose constraints
headers +=   
sources += pose3SLAM.cpp 
check_PROGRAMS += testPose3Factor testPose3Config testPose3SLAM 
testPose3Factor_SOURCES = testPose3Factor.cpp
testPose3Factor_LDADD   = libgtsam.la
testPose3Config_SOURCES = testPose3Config.cpp
testPose3Config_LDADD   = libgtsam.la
testPose3SLAM_SOURCES  = testPose3SLAM.cpp
testPose3SLAM_LDADD    = libgtsam.la

# Cameras
sources += CalibratedCamera.cpp SimpleCamera.cpp
check_PROGRAMS += testCalibratedCamera testSimpleCamera
testCalibratedCamera_SOURCES = testCalibratedCamera.cpp
testCalibratedCamera_LDADD = libgtsam.la
testSimpleCamera_SOURCES = testSimpleCamera.cpp
testSimpleCamera_LDADD = libgtsam.la

# Tensors
headers += tensors.h Tensor1.h Tensor2.h Tensor3.h Tensor4.h Tensor5.h
headers += Tensor1Expression.h Tensor2Expression.h Tensor3Expression.h Tensor5Expression.h
headers += projectiveGeometry.h tensorInterface.h 
sources += projectiveGeometry.cpp tensorInterface.cpp
check_PROGRAMS += testTensors testHomography2 testTrifocal
testHomography2_SOURCES = testHomography2.cpp
testHomography2_LDADD = libgtsam.la
testTensors_SOURCES = testTensors.cpp
testTensors_LDADD = libgtsam.la
testTrifocal_SOURCES = testTrifocal.cpp
testTrifocal_LDADD = libgtsam.la

# Visual SLAM
sources += visualSLAM.cpp
check_PROGRAMS += testVSLAMFactor testVSLAMGraph testVSLAMConfig
testVSLAMFactor_SOURCES = testVSLAMFactor.cpp
testVSLAMFactor_LDADD = libgtsam.la
testVSLAMGraph_SOURCES = testVSLAMGraph.cpp
testVSLAMGraph_LDADD = libgtsam.la
testVSLAMConfig_SOURCES = testVSLAMConfig.cpp
testVSLAMConfig_LDADD = libgtsam.la

headers += $(sources:.cpp=.h)

# Timing tests
noinst_PROGRAMS = timeGaussianFactor timeGaussianFactorGraph timeRot3 timeMatrix timeSymbolMaps timeVectorConfig
timeRot3_SOURCES = timeRot3.cpp
timeRot3_LDADD   = libgtsam.la
timeGaussianFactor_SOURCES = timeGaussianFactor.cpp
timeGaussianFactor_LDADD   = libgtsam.la 
timeGaussianFactorGraph_SOURCES = timeGaussianFactorGraph.cpp
timeGaussianFactorGraph_LDADD   = libgtsam.la
timeMatrix_SOURCES 			 = timeMatrix.cpp 
timeMatrix_LDADD 			   = libgtsam.la
timeSymbolMaps_SOURCES   = timeSymbolMaps.cpp
timeSymbolMaps_LDADD     = libgtsam.la
timeVectorConfig_SOURCES = timeVectorConfig.cpp 
timeVectorConfig_LDADD 	 = libgtsam.la

# create both dynamic and static libraries
AM_CXXFLAGS = -I$(boost) -fPIC
lib_LTLIBRARIES = libgtsam.la
libgtsam_la_SOURCES = $(sources)
libgtsam_la_CPPFLAGS = $(AM_CXXFLAGS)
libgtsam_la_LDFLAGS = -version-info $(version) ../ldl/libldl.a -L../colamd -lcolamd # -lboost_serialization-mt

# enable debug if --enable-debug is set in configure
if DEBUG
AM_CXXFLAGS += -g
endif

# install the header files
include_HEADERS = $(headers)

AM_CXXFLAGS += -I.. 
AM_LDFLAGS = -L../CppUnitLite -lCppUnitLite ../ldl/libldl.a $(BOOST_LDFLAGS) $(boost_serialization)

# adding cblas implementation - split into a default linux version using the 
# autotools script, and a mac version that is hardcoded 
# NOTE: the GT_USE_CBLAS is just used as a means of detecting when blas is available
if USE_BLAS_LINUX
AM_CXXFLAGS += -DGT_USE_CBLAS
libgtsam_la_CPPFLAGS += -DGT_USE_CBLAS
AM_LDFLAGS += $(BLAS_LIBS) $(LIBS) $(FLIBS)  
libgtsam_la_LDFLAGS += $(BLAS_LIBS) $(LIBS) $(FLIBS) 
endif

if USE_BLAS_MACOS
AM_CXXFLAGS += -DGT_USE_CBLAS -DYA_BLAS -DYA_LAPACK -DYA_BLASMULT -I/System/Library/Frameworks/vecLib.framework/Headers 
libgtsam_la_CPPFLAGS += -DGT_USE_CBLAS -DYA_BLAS -DYA_LAPACK -DYA_BLASMULT -I/System/Library/Frameworks/vecLib.framework/Headers 
AM_LDFLAGS += -lcblas -latlas 
libgtsam_la_LDFLAGS += -framework vecLib -lcblas -latlas
endif


TESTS = $(check_PROGRAMS)
CXXLINK = $(LIBTOOL) $(AM_LIBTOOLFLAGS) $(LIBTOOLFLAGS) --mode=link \
	$(CXXLD) -g $(AM_CXXFLAGS) $(CXXFLAGS) $(AM_LDFLAGS) $(LDFLAGS) \
	-o $@ # CXXLINK is only used for unit tests

# rule to run an executable
%.run: % libgtsam.la
	./$^
	
