#----------------------------------------------------------------------------------------------------
# GTSAM geometry
#----------------------------------------------------------------------------------------------------

headers =
sources = 
check_PROGRAMS = 

# Points and poses
sources += Point2.cpp Rot2.cpp Pose2.cpp Point3.cpp Rot3.cpp Pose3.cpp 
check_PROGRAMS += testPoint2 testRot2 testPose2 testPoint3 testRot3 testPose3

# Cameras
sources += Cal3_S2.cpp CalibratedCamera.cpp SimpleCamera.cpp
check_PROGRAMS += testCal3_S2 testCalibratedCamera testSimpleCamera

# Tensors
headers += tensors.h Tensor1.h Tensor2.h Tensor3.h Tensor4.h Tensor5.h
headers += Tensor1Expression.h Tensor2Expression.h Tensor3Expression.h Tensor5Expression.h
sources += projectiveGeometry.cpp tensorInterface.cpp
check_PROGRAMS += testTensors testHomography2 testTrifocal

# Timing tests
noinst_PROGRAMS = timeRot3

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_LTLIBRARIES = libgeometry.la
libgeometry_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir) -I$(top_srcdir)/base

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libgeometry.la ../base/libbase.la ../ldl/libldl.la ../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

#----------------------------------------------------------------------------------------------------
