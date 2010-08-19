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
headers += Simulated2DConfig.h 
headers += Simulated2DPosePrior.h Simulated2DPointPrior.h 
headers += Simulated2DOdometry.h Simulated2DMeasurement.h 
sources += simulated2D.cpp smallExample.cpp
check_PROGRAMS += tests/testSimulated2D 

# simulated2DOriented example
headers +=  Simulated2DOrientedConfig.h
headers +=  Simulated2DOrientedPosePrior.h
headers +=  Simulated2DOrientedOdometry.h
sources +=  simulated2DOriented.cpp 
check_PROGRAMS += tests/testSimulated2DOriented

# simulated3D example
sources += Simulated3D.cpp 
check_PROGRAMS += tests/testSimulated3D 

# Pose SLAM headers
headers += BetweenFactor.h PriorFactor.h

# General constraints
headers += BetweenConstraint.h BoundingConstraint.h TransformConstraint.h

# Utility factors
headers += LinearApproxFactor.h

# 2D Pose SLAM
sources += pose2SLAM.cpp Pose2SLAMOptimizer.cpp dataset.cpp
check_PROGRAMS += tests/testPose2Factor tests/testPose2Config tests/testPose2SLAM tests/testPose2Prior

# 2D SLAM using Bearing and Range
headers += BearingFactor.h RangeFactor.h BearingRangeFactor.h
sources += planarSLAM.cpp
check_PROGRAMS += tests/testPlanarSLAM

# 3D Pose constraints
sources += pose3SLAM.cpp 
check_PROGRAMS += tests/testPose3Factor tests/testPose3Config tests/testPose3SLAM 

# Visual SLAM
sources += visualSLAM.cpp
check_PROGRAMS += tests/testVSLAMFactor tests/testVSLAMGraph tests/testVSLAMConfig

# GaussianISAM2 is fairly SLAM-specific
sources += GaussianISAM2.cpp
check_PROGRAMS += tests/testGaussianISAM2

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
AM_CPPFLAGS = -I$(boost) -I$(BORG_SRCROOT)

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_DEFAULT_SOURCE_EXT = .cpp
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization)
LDADD = libslam.la ../geometry/libgeometry.la  ../nonlinear/libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la  
if USE_LDL
LDADD += libldl.la 
endif 

if USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

