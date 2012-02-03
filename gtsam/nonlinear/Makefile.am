#----------------------------------------------------------------------------------------------------
# GTSAM nonlinear
# Non-linear optimization
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

#----------------------------------------------------------------------------------------------------
# nonlinear
#----------------------------------------------------------------------------------------------------

# Lie Groups
headers += Values-inl.h
sources += Values.cpp
check_PROGRAMS += tests/testValues tests/testKey tests/testOrdering

# Nonlinear nonlinear
headers += Key.h 
headers += NonlinearFactorGraph.h
headers += NonlinearOptimizer-inl.h NonlinearOptimization.h NonlinearOptimization-inl.h 
headers += NonlinearFactor.h
sources += NonlinearFactorGraph.cpp NonlinearOptimizer.cpp Ordering.cpp DoglegOptimizerImpl.cpp NonlinearOptimizationParameters.cpp
headers += DoglegOptimizer.h DoglegOptimizer-inl.h

# Nonlinear iSAM(2)
headers += NonlinearISAM.h NonlinearISAM-inl.h
headers += ISAM2.h ISAM2-inl.h ISAM2-impl-inl.h
sources += GaussianISAM2.cpp
headers += GaussianISAM2-inl.h

# Nonlinear constraints 
headers += NonlinearEquality.h 

# White noise factor
headers += WhiteNoiseFactor.h
#check_PROGRAMS += tests/testWhiteFactor

# Kalman Filter
headers += ExtendedKalmanFilter.h ExtendedKalmanFilter-inl.h

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
nonlineardir = $(pkgincludedir)/nonlinear
nonlinear_HEADERS = $(headers)
noinst_LTLIBRARIES = libnonlinear.la
libnonlinear_la_SOURCES = $(sources)
AM_CPPFLAGS = $(BOOST_CPPFLAGS)  -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization)
LDADD = libnonlinear.la ../linear/liblinear.la ../inference/libinference.la ../geometry/libgeometry.la ../base/libbase.la ../3rdparty/libccolamd.la
LDADD += ../../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

