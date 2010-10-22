#----------------------------------------------------------------------------------------------------
# GTSAM Examples
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS =

# Examples
noinst_PROGRAMS  = SimpleRotation		      		# Optimizes a single nonlinear rotation variable
noinst_PROGRAMS += PlanarSLAMExample_easy			# Solves SLAM example from tutorial by using planarSLAM
noinst_PROGRAMS += PlanarSLAMSelfContained_advanced	# Solves SLAM example from tutorial with all typedefs in the script
noinst_PROGRAMS += Pose2SLAMExample_easy 			# Solves SLAM example from tutorial by using Pose2SLAM and easy optimization interface
noinst_PROGRAMS += Pose2SLAMExample_advanced		# Solves SLAM example from tutorial by using Pose2SLAM and advanced optimization interface
#noinst_PROGRAMS += Pose2SLAMwSPCG_easy      		# Solves a simple Pose2 SLAM example with advanced SPCG solver
#noinst_PROGRAMS += Pose2SLAMwSPCG_advanced  		# Solves a simple Pose2 SLAM example with easy SPCG solver
SUBDIRS = vSLAMexample # does not compile....
#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)/..
AM_LDFLAGS = $(BOOST_LDFLAGS)
LDADD = ../libgtsam.la
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
