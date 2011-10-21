#----------------------------------------------------------------------------------------------------
# SLAM and SFM sources
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

# simulated2D example
headers += simulated2DConstraints.h
sources += simulated2D.cpp smallExample.cpp
check_PROGRAMS += tests/testSimulated2D 

# MATLAB Wrap headers for simulated2D
headers += Simulated2DMeasurement.h
headers += Simulated2DOdometry.h
headers += Simulated2DPointPrior.h
headers += Simulated2DPosePrior.h
headers += Simulated2DValues.h

# simulated2DOriented example
sources +=  simulated2DOriented.cpp 
check_PROGRAMS += tests/testSimulated2DOriented

# MATLAB Wrap headers for simulated2DOriented
headers += Simulated2DOrientedOdometry.h
headers += Simulated2DOrientedPosePrior.h
headers += Simulated2DOrientedValues.h

# simulated3D example
sources += Simulated3D.cpp 
check_PROGRAMS += tests/testSimulated3D 

# Generic SLAM headers
headers += BetweenFactor.h PriorFactor.h PartialPriorFactor.h
headers += BearingFactor.h RangeFactor.h BearingRangeFactor.h

# Generic constraint headers
headers += BetweenConstraint.h BoundingConstraint.h

# 2D Pose SLAM
sources += pose2SLAM.cpp dataset.cpp
check_PROGRAMS += tests/testPose2SLAM 

# 2D SLAM using Bearing and Range
sources += planarSLAM.cpp
check_PROGRAMS += tests/testPlanarSLAM

# MATLAB Wrap headers for planarSLAM
headers += Landmark2.h
headers += PlanarSLAMGraph.h
headers += PlanarSLAMValues.h

# 3D Pose constraints
sources += pose3SLAM.cpp 
check_PROGRAMS += tests/testPose3SLAM 

# Visual SLAM
headers += GeneralSFMFactor.h ProjectionFactor.h
sources += visualSLAM.cpp
check_PROGRAMS += tests/testProjectionFactor tests/testVSLAM tests/testGeneralSFMFactor tests/testGeneralSFMFactor_Cal3Bundler

# StereoFactor
headers += StereoFactor.h
check_PROGRAMS += tests/testStereoFactor

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
slamdir = $(pkgincludedir)/slam
slam_HEADERS = $(headers)
noinst_LTLIBRARIES = libslam.la
libslam_la_SOURCES = $(sources)
AM_CPPFLAGS = $(BOOST_CPPFLAGS)  -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS += $(boost_serialization) 
LDADD = libslam.la ../geometry/libgeometry.la  ../nonlinear/libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la ../3rdparty/libccolamd.la 
LDADD += ../../CppUnitLite/libCppUnitLite.a

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
