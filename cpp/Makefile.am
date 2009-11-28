# on the Mac, libtool is called glibtool :-(
# documentation see /opt/local/share/doc/libtool-1.5.26/manual.html
if DARWIN
  LIBTOOL = glibtool --tag=CXX
else
  LIBTOOL = libtool
endif

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
sources = Vector.cpp svdcmp.cpp Matrix.cpp numericalDerivative.cpp 
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

# Inference
headers += inference.h inference-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += BayesNet.h BayesNet-inl.h BayesTree.h BayesTree-inl.h
check_PROGRAMS += testFactorgraph testBayesTree testInference testOrdering
testFactorgraph_SOURCES        = testFactorgraph.cpp
testBayesTree_SOURCES          = $(example) testBayesTree.cpp
testInference_SOURCES          = $(example) testInference.cpp
testFactorgraph_LDADD          = libgtsam.la
testBayesTree_LDADD            = libgtsam.la
testInference_LDADD            = libgtsam.la
testOrdering_SOURCES   		   = testOrdering.cpp
testOrdering_LDADD			   = libgtsam.la

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

# Gaussian inference 
headers += GaussianFactorSet.h
sources += VectorConfig.cpp GaussianFactor.cpp GaussianFactorGraph.cpp GaussianConditional.cpp GaussianBayesNet.cpp
check_PROGRAMS += testVectorConfig testGaussianFactor testGaussianFactorGraph testGaussianConditional testGaussianBayesNet 
testVectorConfig_SOURCES        = testVectorConfig.cpp
testVectorConfig_LDADD          = libgtsam.la
testGaussianFactor_SOURCES        = $(example) testGaussianFactor.cpp
testGaussianFactor_LDADD          = libgtsam.la
testGaussianFactorGraph_SOURCES   = $(example) testGaussianFactorGraph.cpp
testGaussianFactorGraph_LDADD     = libgtsam.la
testGaussianConditional_SOURCES = $(example) testGaussianConditional.cpp
testGaussianConditional_LDADD   = libgtsam.la
testGaussianBayesNet_SOURCES    = $(example) testGaussianBayesNet.cpp
testGaussianBayesNet_LDADD      = libgtsam.la

# not the correct way, I'm sure: Kai ?
timeGaussianFactor: timeGaussianFactor.cpp  
timeGaussianFactor: CXXFLAGS += -I /opt/local/include
timeGaussianFactor: LDFLAGS += -L.libs -lgtsam

# not the correct way, I'm sure: Kai ?
timeGaussianFactorGraph: timeGaussianFactorGraph.cpp  
timeGaussianFactorGraph: CXXFLAGS += -I /opt/local/include -I ..
timeGaussianFactorGraph: LDFLAGS += smallExample.o -L.libs -lgtsam -L../CppUnitLite -lCppUnitLite

# Nonlinear inference 
headers += NonlinearFactorGraph.h NonlinearFactorGraph-inl.h
headers += NonlinearOptimizer.h NonlinearOptimizer-inl.h 
headers += SQPOptimizer.h SQPOptimizer-inl.h  
headers += NonlinearConstraint.h NonlinearConstraint-inl.h
sources += NonlinearFactor.cpp 
sources += NonlinearEquality.h
check_PROGRAMS += testNonlinearFactor testNonlinearFactorGraph testNonlinearOptimizer testNonlinearEquality testSQP testNonlinearConstraint testSQPOptimizer
testNonlinearFactor_SOURCES      = $(example) testNonlinearFactor.cpp
testNonlinearFactor_LDADD        = libgtsam.la
testNonlinearConstraint_SOURCES  = $(example) testNonlinearConstraint.cpp
testNonlinearConstraint_LDADD    = libgtsam.la
testNonlinearFactorGraph_SOURCES = $(example) testNonlinearFactorGraph.cpp
testNonlinearFactorGraph_LDADD   = libgtsam.la
testNonlinearOptimizer_SOURCES   = $(example) testNonlinearOptimizer.cpp
testNonlinearOptimizer_LDADD     = libgtsam.la
testSQPOptimizer_SOURCES   		 = $(example) testSQPOptimizer.cpp
testSQPOptimizer_LDADD     		 = libgtsam.la
testNonlinearEquality_SOURCES    = testNonlinearEquality.cpp
testNonlinearEquality_LDADD      = libgtsam.la
testSQP_SOURCES                  = $(example) testSQP.cpp
testSQP_LDADD                    = libgtsam.la

# geometry
sources += Point2.cpp Pose2.cpp Point3.cpp Rot3.cpp Pose3.cpp Cal3_S2.cpp
check_PROGRAMS += testPoint2 testPose2 testPoint3 testRot3 testPose3 testCal3_S2
testPoint2_SOURCES  = testPoint2.cpp
testPose2_SOURCES   = testPose2.cpp
testPoint3_SOURCES  = testPoint3.cpp
testRot3_SOURCES    = testRot3.cpp
testPose3_SOURCES   = testPose3.cpp
testCal3_S2_SOURCES = testCal3_S2.cpp               

testPoint2_LDADD    = libgtsam.la
testPose2_LDADD     = libgtsam.la
testPoint3_LDADD    = libgtsam.la
testRot3_LDADD      = libgtsam.la
testPose3_LDADD     = libgtsam.la
testCal3_S2_LDADD   = libgtsam.la

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

# Robot Control example system
sources += ControlConfig.cpp ControlPoint.cpp ControlGraph.cpp
check_PROGRAMS += testControlConfig testControlPoint testControlGraph
testControlConfig_SOURCES = testControlConfig.cpp
testControlConfig_LDADD = libgtsam.la
testControlPoint_SOURCES = testControlPoint.cpp
testControlPoint_LDADD = libgtsam.la
testControlGraph_SOURCES = testControlGraph.cpp
testControlGraph_LDADD = libgtsam.la

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



