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
noinst_PROGRAMS = vSLAMexample
vSLAMexample_includedir = .
vSLAMexample_SOURCES = vSLAMexample.cpp Feature2D.cpp landmarkUtils.cpp
#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
AM_LDFLAGS = $(BOOST_LDFLAGS) 
AM_CPPFLAGS = -I$(boost) -I$(SparseInc) -I$(top_srcdir)/..
LDADD = ../../libgtsam.la
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
