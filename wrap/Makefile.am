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
sources += utilities.cpp Argument.cpp ReturnValue.cpp Constructor.cpp 
sources += Method.cpp StaticMethod.cpp Class.cpp Module.cpp FileWriter.cpp
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

# Set flags to pass to mex
mexFlags = 
if ENABLE_UNSAFE_WRAP
mexFlags += "${BOOST_CPPFLAGS} -DUNSAFE_WRAP -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
else
mexFlags += "${BOOST_CPPFLAGS} -I${prefix}/include -I${prefix}/include/gtsam -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
endif

# Find the extension for mex binaries 
# this should be done with mexext with matlab
mexextension = 
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

# Choose correct cp command by OS
cp_install =
if LINUX
cp_install += cp -ru
endif
if DARWIN
cp_install += cp -Rf
endif

all: generate_toolbox

generate_toolbox: $(top_srcdir)/gtsam.h wrap
	./wrap ${mexextension} ${interfacePath} ${moduleName} ${toolboxpath} ${mexFlags}

source_mode = -m 644

wrap-install-matlab-toolbox: generate_toolbox
	install -d ${toolbox}/gtsam
	install ${source_mode} -c ../toolbox/*.m ${toolbox}/gtsam
	install ${source_mode} -c ../toolbox/*.cpp ${toolbox}/gtsam
	install ${source_mode} -c ../toolbox/Makefile ${toolbox}/gtsam
	${cp_install} ../toolbox/@* ${toolbox}/gtsam

wrap-install-bin: wrap
	install -d ${wrap} 
	install -c ./wrap ${wrap}

wrap-install-matlab-tests: 
	install -d ${toolbox}/gtsam/tests
	install ${source_mode} -c ../../tests/matlab/*.m ${toolbox}/gtsam/tests

wrap-install-matlab-examples: 
	install -d ${toolbox}/gtsam/examples 
	install ${source_mode} -c ../../examples/matlab/*.m ${toolbox}/gtsam/examples

wrap_install_targets =
wrap_install_targets += wrap-install-matlab-toolbox 

if ENABLE_INSTALL_WRAP
wrap_install_targets += wrap-install-bin
endif

if ENABLE_INSTALL_MATLAB_TESTS
wrap_install_targets += wrap-install-matlab-tests
endif

if ENABLE_INSTALL_MATLAB_EXAMPLES
wrap_install_targets += wrap-install-matlab-examples
endif

install-exec-hook: ${wrap_install_targets}

# clean local toolbox dir
clean:
	@test -z "wrap" || rm -f wrap
	@test -z "../toolbox" || rm -rf ../toolbox

endif
#----------------------------------------------------------------------------------------------------
