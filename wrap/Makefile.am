common = utilities.cpp Argument.cpp Constructor.cpp Method.cpp Class.cpp Module.cpp

check_PROGRAMS = testSpirit testWrap 
testSpirit_SOURCES  = testSpirit.cpp
testWrap_SOURCES = testWrap.cpp ${common}

# generate local toolbox dir
interfacePath = $(top_srcdir)
moduleName = gtsam
toolboxpath = ../toolbox
nameSpace = "gtsam"
mexFlags = "${BOOST_CPPFLAGS} -I${prefix}/include -I${prefix}/include/gtsam/linear -I${prefix}/include/gtsam/nonlinear -I${prefix}/include/gtsam/base -I${prefix}/include/gtsam/geometry -I${prefix}/include/gtsam/slam -L${exec_prefix}/lib -lgtsam"
all:
	./wrap ${interfacePath} ${moduleName} ${toolboxpath} ${nameSpace} ${mexFlags}

# install the header files
noinst_HEADERS = geometry.h utilities.h Argument.h Constructor.h Method.h Class.h Module.h wrap-matlab.h

noinst_PROGRAMS = wrap
wrap_SOURCES = ${common} wrap.cpp
AM_CPPFLAGS = $(BOOST_CPPFLAGS) -I$(top_srcdir)/.. -DTOPSRCDIR="\"$(top_srcdir)\""
AM_CXXFLAGS = -MMD
AM_LDFLAGS = $(BOOST_LDFLAGS) -L../CppUnitLite -lCppUnitLite

TESTS = $(check_PROGRAMS)

# install the headers and matlab toolbox
install-exec-hook: all
	install -d ${toolbox}/gtsam && \
	cp -rf ../toolbox/* ${toolbox}/gtsam && \
	install -d ${includedir}/wrap && \
	cp -f ${srcdir}/wrap-matlab.h  ${includedir}/wrap/matlab.h

# clean local toolbox dir
clean:
	@test -z "wrap" || rm -f wrap
	@test -z "../toolbox" || rm -rf ../toolbox

# rule to run an executable
%.run: % libgtsam.la
	./$^
