#----------------------------------------------------------------------------------------------------
# GTSAM geometry
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

# Points and poses
sources += Point2.cpp Rot2.cpp Pose2.cpp Point3.cpp Rot3.cpp Pose3.cpp 
check_PROGRAMS += tests/testPoint2 tests/testRot2 tests/testPose2 tests/testPoint3 tests/testRot3 tests/testPose3

# Cameras
sources += Cal3_S2.cpp CalibratedCamera.cpp SimpleCamera.cpp
check_PROGRAMS += tests/testCal3_S2 tests/testCalibratedCamera tests/testSimpleCamera

# Stereo
sources += StereoPoint2.cpp StereoCamera.cpp
check_PROGRAMS += tests/testStereoCamera

# Tensors
headers += tensors.h Tensor1.h Tensor2.h Tensor3.h Tensor4.h Tensor5.h
headers += Tensor1Expression.h Tensor2Expression.h Tensor3Expression.h Tensor5Expression.h
sources += projectiveGeometry.cpp tensorInterface.cpp
check_PROGRAMS += tests/testTensors tests/testHomography2 tests/testTrifocal tests/testFundamental

# Timing tests
noinst_PROGRAMS = tests/timeRot3 tests/timeCalibratedCamera

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
geometrydir = $(pkgincludedir)/geometry
geometry_HEADERS = $(headers)
noinst_LTLIBRARIES = libgeometry.la
libgeometry_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)/..

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libgeometry.la ../base/libbase.la ../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp
if USE_LDL
LDADD += libldl.la 
endif  

# rule to run an executable
%.run: % $(LDADD)
	./$^

#----------------------------------------------------------------------------------------------------
