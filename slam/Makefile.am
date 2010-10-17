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
headers += Simulated2DValues.h 
headers += Simulated2DPosePrior.h Simulated2DPointPrior.h 
headers += Simulated2DOdometry.h Simulated2DMeasurement.h 
sources += simulated2D.cpp smallExample.cpp
check_PROGRAMS += tests/testSimulated2D 

# simulated2DOriented example
headers +=  Simulated2DOrientedValues.h
headers +=  Simulated2DOrientedPosePrior.h
headers +=  Simulated2DOrientedOdometry.h
sources +=  simulated2DOriented.cpp 
check_PROGRAMS += tests/testSimulated2DOriented

# simulated3D example
sources += Simulated3D.cpp 
check_PROGRAMS += tests/testSimulated3D 

# Pose SLAM headers
headers += BetweenFactor.h PriorFactor.h

# 2D Pose SLAM
sources += pose2SLAM.cpp dataset.cpp
#sources += Pose2SLAMOptimizer.cpp
check_PROGRAMS += tests/testPose2Factor tests/testPose2Values tests/testPose2SLAM tests/testPose2Prior

# 2D SLAM using Bearing and Range
headers += BearingFactor.h RangeFactor.h BearingRangeFactor.h
sources += planarSLAM.cpp
check_PROGRAMS += tests/testPlanarSLAM

# 3D Pose constraints
sources += pose3SLAM.cpp 
check_PROGRAMS += tests/testPose3Factor tests/testPose3Values tests/testPose3SLAM 

# Visual SLAM
sources += visualSLAM.cpp
check_PROGRAMS += tests/testVSLAMFactor tests/testVSLAMGraph tests/testVSLAMValues

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
AM_CPPFLAGS = -I$(CCOLAMDInc) -I$(top_srcdir)/..

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS = $(boost_serialization) -L$(CCOLAMDLib) -lccolamd
LDADD = libslam.la ../geometry/libgeometry.la  ../nonlinear/libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la
LDADD += ../CppUnitLite/libCppUnitLite.a

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
