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
noinst_PROGRAMS =  
wrap_PROGRAMS = 
wrapdir = $(pkgincludedir)/wrap

# disable all of matlab toolbox build by default
if ENABLE_BUILD_TOOLBOX

# Build a library from the core sources
sources += utilities.cpp Argument.cpp ReturnValue.cpp Constructor.cpp Method.cpp StaticMethod.cpp Class.cpp Module.cpp
check_PROGRAMS += tests/testSpirit tests/testWrap
if ENABLE_INSTALL_WRAP
wrap_PROGRAMS += wrap
else 
noinst_PROGRAMS += wrap
endif

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
# Only install the header necessary for wrap interfaces to build with mex
headers += matlab.h 
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
mexFlags = 
mexFlags += "${BOOST_CPPFLAGS} -DUNSAFE_WRAP -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
if ENABLE_UNSAFE_WRAP
	mexFlags += "${BOOST_CPPFLAGS} -DUNSAFE_WRAP -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
else
	mexFlags += "${BOOST_CPPFLAGS} -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
endif

# Find the extension for mex binaries - FIXME: this should be done with mexext with matlab
mexextension = mexa64
if LINUX
if IS_64BIT
	mexextension += mexa64
else
	mexextension += mexglx
endif
else # Linux
if DARWIN
	mexextension += mexmaci64
else
	mexextension += mex_bin   	
endif
endif # Linux

all:
	./wrap ${mexextension} ${interfacePath} ${moduleName} ${toolboxpath} ${nameSpace} ${mexFlags}

wrap-install-bin: all
	install -d ${wrap} && \
	install ./wrap ${wrap}

wrap-install-matlab-tests: all
	install -d ${toolbox}/gtsam/tests && \
	cp -rf ../../tests/matlab/*.m ${toolbox}/gtsam/tests

# install the headers and matlab toolbox
if ENABLE_INSTALL_WRAP
if ENABLE_INSTALL_MATLAB_TESTS
install-exec-hook: wrap-install-bin wrap-install-matlab-tests
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam 
else
install-exec-hook: wrap-install-bin
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam/tests
endif
else
if ENABLE_INSTALL_MATLAB_TESTS
install-exec-hook: wrap-install-matlab-tests
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam
else
install-exec-hook: all
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam/tests
endif
endif

# clean local toolbox dir
clean:
	@test -z "wrap" || rm -f wrap
	@test -z "../toolbox" || rm -rf ../toolbox

endif
#----------------------------------------------------------------------------------------------------
