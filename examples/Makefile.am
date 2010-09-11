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
noinst_PROGRAMS  = SimpleRotation      # Optimizes a single nonlinear rotation variable
noinst_PROGRAMS += SLAMSelfContained   # Solves SLAM example from tutorial with all typedefs in the script
noinst_PROGRAMS += PlanarSLAMExample   # Solves SLAM example from tutorial by using planarSLAM

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
AM_LDFLAGS = $(BOOST_LDFLAGS) 
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir)/..
LDADD = ../libgtsam.la
AM_DEFAULT_SOURCE_EXT = .cpp
if USE_LDL
LDADD += libldl.la 
endif  

# rule to run an executable
%.run: % $(LDADD)
	./$^

#----------------------------------------------------------------------------------------------------
