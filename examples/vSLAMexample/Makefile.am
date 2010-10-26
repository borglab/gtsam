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
noinst_PROGRAMS = vSFMexample
vSFMexample_SOURCES = vSFMexample.cpp Feature2D.cpp vSLAMutils.cpp

noinst_PROGRAMS += vISAMexample
vISAMexample_SOURCES = vISAMexample.cpp Feature2D.cpp vSLAMutils.cpp

headers += ISAMLoop.h ISAMLoop-inl.h
headers += Feature2D.h vSLAMutils.h

noinst_HEADERS = $(headers)

EXTRA_DIST = Data
dist-hook:
	rm -rf $(distdir)/Data/.svn

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(CCOLAMDInc) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
LDADD = ../../gtsam/libgtsam.la
AM_DEFAULT_SOURCE_EXT = .cpp


# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
#----------------------------------------------------------------------------------------------------
