INSTALL = install

AM_CXXFLAGS = -MMD -I$(boost) -I.. -O5

noinst_PROGRAMS = wrap
common = utilities.cpp Argument.cpp Constructor.cpp Method.cpp Class.cpp Module.cpp
wrap_SOURCES = ${common} wrap.cpp

check_PROGRAMS = testSpirit testWrap 
testSpirit_SOURCES  = testSpirit.cpp
testWrap_SOURCES = testWrap.cpp ${common}

# generate local toolbox dir
interfacePath = ../cpp
moduleName = gtsam
toolboxpath = ../toolbox
nameSpace = "gtsam"
mexFlags = "-I${boost} -I${prefix}/include -I${prefix}/include/gtsam -L${exec_prefix}/lib -lgtsam"
all:
	./wrap ${interfacePath} ${moduleName} ${toolboxpath} ${nameSpace} ${mexFlags}

# install the header files
include_HEADERS = utilities.h Argument.h Constructor.h Method.h Class.h Module.h wrap-matlab.h

install: all

AM_CXXFLAGS += -I.. 
AM_LDFLAGS = -L../CppUnitLite -lCppUnitLite $(BOOST_LDFLAGS)

TESTS = $(check_PROGRAMS)

# clean local toolbox dir
clean:
	@test -z "wrap" || rm -f wrap
	@test -z "../toolbox" || rm -rf ../toolbox

# rule to run an executable
%.run: % libgtsam.la
	./$^

	