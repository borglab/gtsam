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

all: external_libs

external_libs:
	@echo Compiling CppUnitLite...; cd ../CppUnitLite; make all
	@echo Compiling Colamd...;      cd ../colamd; make all

install: external_libs

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
testMatrix_SOURCES = testMatrix.cpp

testVector_LDADD = libgtsam.la
testMatrix_LDADD = libgtsam.la

# nodes
sources += FGConfig.cpp Ordering.cpp LinearFactor.cpp NonlinearFactor.cpp ConditionalGaussian.cpp EqualityFactor.cpp DeltaFunction.cpp  
check_PROGRAMS += testFGConfig testLinearFactor testConditionalGaussian testNonlinearFactor testEqualityFactor testDeltaFunction
example = smallExample.cpp 
testFGConfig_SOURCES            = testFGConfig.cpp
testLinearFactor_SOURCES        = $(example) testLinearFactor.cpp
testConditionalGaussian_SOURCES = $(example) testConditionalGaussian.cpp
testNonlinearFactor_SOURCES     = $(example) testNonlinearFactor.cpp
testEqualityFactor_SOURCES      = $(example) testEqualityFactor.cpp
testDeltaFunction_SOURCES       = testDeltaFunction.cpp

testFGConfig_LDADD            = libgtsam.la
testLinearFactor_LDADD        = libgtsam.la
testConditionalGaussian_LDADD = libgtsam.la
testNonlinearFactor_LDADD     = libgtsam.la
testEqualityFactor_LDADD	  = libgtsam.la
testDeltaFunction_LDADD       = libgtsam.la

# not the correct way, I'm sure: Kai ?
timeLinearFactor: timeLinearFactor.cpp  
timeLinearFactor: CXXFLAGS += -I /opt/local/include
timeLinearFactor: LDFLAGS += -L.libs -lgtsam

# graphs 
sources += ConstrainedChordalBayesNet.cpp ChordalBayesNet.cpp FactorGraph.cpp LinearFactorGraph.cpp NonlinearFactorGraph.cpp 
sources += ConstrainedNonlinearFactorGraph.cpp ConstrainedLinearFactorGraph.cpp 
check_PROGRAMS += testChordalBayesNet testConstrainedChordalBayesNet testFactorgraph testLinearFactorGraph testNonlinearFactorGraph testConstrainedNonlinearFactorGraph testConstrainedLinearFactorGraph
testChordalBayesNet_SOURCES      = $(example) testChordalBayesNet.cpp
testConstrainedChordalBayesNet_SOURCES      = $(example) testConstrainedChordalBayesNet.cpp
testFactorgraph_SOURCES          = testFactorgraph.cpp
testLinearFactorGraph_SOURCES    = $(example) testLinearFactorGraph.cpp
testNonlinearFactorGraph_SOURCES = $(example) testNonlinearFactorGraph.cpp
testConstrainedNonlinearFactorGraph_SOURCES = $(example) testConstrainedNonlinearFactorGraph.cpp 
testConstrainedLinearFactorGraph_SOURCES    = $(example) testConstrainedLinearFactorGraph.cpp

testChordalBayesNet_LDADD      = libgtsam.la
testFactorgraph_LDADD          = libgtsam.la
testLinearFactorGraph_LDADD    = libgtsam.la
testNonlinearFactorGraph_LDADD = libgtsam.la
testConstrainedNonlinearFactorGraph_LDADD = libgtsam.la 
testConstrainedLinearFactorGraph_LDADD    = libgtsam.la
testConstrainedChordalBayesNet_LDADD      = libgtsam.la

# geometry
sources += Point2.cpp Point3.cpp Rot3.cpp Pose3.cpp Cal3_S2.cpp
check_PROGRAMS += testPoint2 testPoint3 testRot3 testPose3 testCal3_S2
testPoint2_SOURCES  = testPoint2.cpp
testPoint3_SOURCES  = testPoint3.cpp
testRot3_SOURCES    = testRot3.cpp
testPose3_SOURCES   = testPose3.cpp
testCal3_S2_SOURCES = testCal3_S2.cpp               

testPoint2_LDADD    = libgtsam.la
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

# Visual SLAM
sources += CalibratedCamera.cpp SimpleCamera.cpp VSLAMFactor.cpp
check_PROGRAMS += testCalibratedCamera testSimpleCamera testVSLAMFactor
testCalibratedCamera_SOURCES = testCalibratedCamera.cpp
testCalibratedCamera_LDADD = libgtsam.la
testSimpleCamera_SOURCES = testSimpleCamera.cpp
testSimpleCamera_LDADD = libgtsam.la
testVSLAMFactor_SOURCES = testVSLAMFactor.cpp
testVSLAMFactor_LDADD = libgtsam.la


# The header files will be installed in ~/include/gtsam
headers = gtsam.h Value.h Factor.h LinearFactorSet.h Point2Prior.h Simulated2DOdometry.h Simulated2DMeasurement.h smallExample.h $(sources:.cpp=.h)

# create both dynamic and static libraries
AM_CXXFLAGS = -I$(boost) -fPIC
lib_LTLIBRARIES = libgtsam.la
libgtsam_la_SOURCES = $(sources)
libgtsam_la_CPPFLAGS = $(AM_CXXFLAGS)
libgtsam_la_LDFLAGS = -version-info $(version) -L../colamd -lcolamd #-lboost_serialization-mt

# install the header files
include_HEADERS = $(headers)

AM_CXXFLAGS += -I.. 
AM_LDFLAGS = -L../CppUnitLite -lCppUnitLite $(BOOST_LDFLAGS) $(boost_serialization) #-L. -lgtsam

TESTS = $(check_PROGRAMS)

# rule to run an executable
%.run: % libgtsam.la
	./$^
