# Makefile for gtsam/cpp
# Author Frank Dellaert

# on the Mac, libtool is called glibtool :-(
# documentation see /opt/local/share/doc/libtool-1.5.26/manual.html
ifeq ($(shell uname),Darwin)
  LIBTOOL = glibtool
else
  LIBTOOL = libtool
endif

INSTALL = install

# C++ flags
CXXFLAGS += -isystem $(BOOST_DIR) -O5
CXXFLAGS += -DBOOST_UBLAS_NDEBUG

# specify the source files
# basic
sources = Vector.cpp svdcmp.cpp Matrix.cpp numericalDerivative.cpp Ordering.cpp
# nodes
sources += FGConfig.cpp GaussianFactor.cpp ConditionalGaussian.cpp NonlinearFactor.cpp 
# graphs
sources += FactorGraph.cpp GaussianFactorGraph.cpp NonlinearFactorGraph.cpp ChordalBayesNet.cpp
# geometry
sources += Point2.cpp Point3.cpp Rot3.cpp Pose3.cpp Cal3_S2.cpp

# The header files will be installed in ~/include/gtsam
headers = Value.h factor.h linearfactorset.h $(sources:.cpp=.h)

# conventional object files
object_files = $(sources:.cpp=.o)

# For libtool to build a shared library, we need "shared" object files with extension .lo
shared_object_files = $(sources:.cpp=.lo)

# rule for shared compiling shared_object_files
%.lo: %.o

# rule for shared compiling shared_object_files
%.lo: %.cpp
	$(LIBTOOL) --tag=CXX --mode=compile $(COMPILE.cpp) $(OUTPUT_OPTION) $<

# library version
current = 0  # The most recent interface number that this library implements. 
revision = 0 # The implementation number of the current interface 
age = 0 # The difference between the newest and oldest interfaces that	\
this library implements. In other words, the library implements all	\
the interface numbers in the range from number current - age to		\
current.
# from libtool manual:
# Here are a set of rules to help you update your library version information:
#   Start with version information of ‘0:0:0’ for each libtool library.
#   Update the version information only immediately before a public release of your software.
#   If the library source code has changed at all since the last update, then increment revision
#   If any interfaces have been added, removed, or changed since the last update, increment current, and set revision to 0.
#   If any interfaces have been added since the last public release, then increment age.
#   If any interfaces have been removed since the last public release, then set age to 0.
version = $(current):$(revision):$(age)

# this builds the shared library
# note that libgtsam.la is the libtool target
# the actual library is built in the hidden subdirectory .libs
libgtsam.la : $(shared_object_files) 
	$(LIBTOOL) --tag=CXX --mode=link g++ -version-info $(version) -o libgtsam.la -rpath $(HOME)/lib $(shared_object_files)

# shortcut
lib: libgtsam.la

# this builds the static library (used for unit testing)
# object files will be only ones remade if a file is touched because deps broken
libgtsam.a : $(shared_object_files) $(object_files) 
	$(LIBTOOL) --tag=CXX --mode=link g++ -o libgtsam.a -static $(HOME)/lib $(shared_object_files)

# and this installs the shared library
install: libgtsam.la
	$(INSTALL) -d $(HOME)/include/gtsam
	rm -f $(HOME)/include/gtsam/typedefs.h
	cp -f $(headers) $(HOME)/include/gtsam
	$(LIBTOOL) --mode=install cp libgtsam.la $(HOME)/lib/libgtsam.la

# create the MATLAB toolbox
interfacePath = .
moduleName = gtsam
toolboxpath = $(HOME)/toolbox/gtsam
mexFlags = "-I$(BOOST_DIR) -I$(HOME)/include -I$(HOME)/include/gtsam -L$(HOME)/lib -lgtsam"
matlab:
	wrap $(interfacePath) $(moduleName) $(toolboxpath) $(mexFlags)

# unit tests
unit-tests = $(shell ls test*.cpp) 
unit-tests: $(unit-tests:.cpp=.run)

# timing tests
timing-tests = $(shell ls time*.cpp) 
timing-tests: $(timing-tests:.cpp=.run)

# local executables are for testing and timing
executables =  $(unit-tests:.cpp=) $(timing-tests:.cpp=)

# link flags
INCDIR ?= $(HOME)/include
LIBDIR ?= $(HOME)/lib
$(executables) : simulated2D.o smallExample.o libgtsam.a
$(executables) : LDFLAGS += -I. -I$(INCDIR)
$(executables) : LDLIBS += libgtsam.a -L$(LIBDIR) -lCppUnitLite

tests: unit-tests timing-tests
clean-tests:
	-rm -rf $(executables)

# make a version of timeGaussianFactor instrumented for Saturn profiler
saturn: timeGaussianFactor
saturn: CXXFLAGS += -finstrument-functions
saturn: LDLIBS += -lSaturn

# rule to run an executable
%.run: %
	./$^

# clean will remove the hidden .libs directory by libtool as well
clean: clean-tests
	-rm -rf *.d *.o *.lo *.a *.la .libs *.dSYM

.PHONY: clean clean-tests unit-tests timing-tests matlab deps

# dependecy generation as described in
# http://www.wlug.org.nz/MakefileHowto
all-sources = $(sources) smallExample.cpp simulated2D.cpp

# when building object files, -MMD specifies dependency generation into .d files
(all-sources:.cpp=.o): CXXFLAGS += -MMD

deps := $(all-sources:.cpp=.d)
deps: $(all-sources)
	$(CXX) -MMD -E $(CXXFLAGS) -I. -I$(INCDIR) $(all-sources) > /dev/null

-include $(deps)

