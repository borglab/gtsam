#----------------------------------------------------------------------------------------------------
# GTSAM Matlab wrap toolset
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc
AM_DEFAULT_SOURCE_EXT = .cpp

headers =
sources = 
check_PROGRAMS = 

# disable all of matlab toolbox build by default
if ENABLE_BUILD_TOOLBOX

# Build a library from the core sources
sources += utilities.cpp Argument.cpp Constructor.cpp Method.cpp Class.cpp Module.cpp
check_PROGRAMS += tests/testSpirit tests/testWrap 
noinst_PROGRAMS = wrap

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
# Only install the header necessary for wrap interfaces to build with mex
headers += matlab.h 
wrapdir = $(pkgincludedir)/wrap
wrap_HEADERS = $(headers)
noinst_LTLIBRARIES = libwrap.la
libwrap_la_SOURCES = $(sources)
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(top_srcdir) -DTOPSRCDIR="\"$(top_srcdir)\""
AM_LDFLAGS = $(BOOST_LDFLAGS)

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization) 
LDADD = libwrap.la ../CppUnitLite/libCppUnitLite.a  

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^
	
# generate local toolbox dir
interfacePath = $(top_srcdir)
moduleName = gtsam
toolboxpath = ../toolbox  
nameSpace = "gtsam"
mexFlags = "${BOOST_CPPFLAGS} -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
all:
	./wrap ${interfacePath} ${moduleName} ${toolboxpath} ${nameSpace} ${mexFlags}
	
# install the headers and matlab toolbox
install-exec-hook: all
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam

# clean local toolbox dir
clean:
	@test -z "wrap" || rm -f wrap
	@test -z "../toolbox" || rm -rf ../toolbox

endif
#----------------------------------------------------------------------------------------------------
