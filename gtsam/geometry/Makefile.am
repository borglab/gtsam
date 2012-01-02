#----------------------------------------------------------------------------------------------------
# GTSAM geometry
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

# Concept check
headers += concepts.h

# Points and poses
sources += Point2.cpp Rot2.cpp Pose2.cpp Point3.cpp Pose3.cpp
check_PROGRAMS += tests/testPoint2 tests/testRot2 tests/testPose2 tests/testPoint3 tests/testRot3M tests/testRot3Q tests/testPose3

# Cameras
headers += GeneralCameraT.h Cal3_S2Stereo.h
sources += Cal3_S2.cpp Cal3DS2.cpp Cal3Bundler.cpp CalibratedCamera.cpp SimpleCamera.cpp
check_PROGRAMS += tests/testCal3DS2 tests/testCal3_S2 tests/testCal3Bundler tests/testCalibratedCamera tests/testSimpleCamera

# Stereo
sources += StereoPoint2.cpp StereoCamera.cpp
check_PROGRAMS += tests/testStereoCamera tests/testStereoPoint2

# Tensors
headers += tensors.h Tensor1.h Tensor2.h Tensor3.h Tensor4.h Tensor5.h
headers += Tensor1Expression.h Tensor2Expression.h Tensor3Expression.h Tensor5Expression.h
sources += projectiveGeometry.cpp tensorInterface.cpp
check_PROGRAMS += tests/testTensors tests/testHomography2 tests/testFundamental

# Timing tests
noinst_PROGRAMS = tests/timeRot3 tests/timeCalibratedCamera

# Rot3M and Rot3Q both use Rot3.h, they do not have individual header files
allsources = $(sources)
allsources += Rot3M.cpp Rot3Q.cpp
headers += Rot3.h

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
geometrydir = $(pkgincludedir)/geometry
geometry_HEADERS = $(headers)
noinst_LTLIBRARIES = libgeometry.la
libgeometry_la_SOURCES = $(allsources)
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization) 
LDADD = libgeometry.la ../base/libbase.la ../../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
