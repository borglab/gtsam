#----------------------------------------------------------------------------------------------------
# GTSAM core functionality: base classes for inference, as well as symbolic and discrete
#----------------------------------------------------------------------------------------------------

# use nostdinc to turn off -I. and -I.., we do not need them because 
# header files are qualified so they can be included in external projects.
AUTOMAKE_OPTIONS = nostdinc

headers =
sources = 
check_PROGRAMS = 

#----------------------------------------------------------------------------------------------------
# base
#----------------------------------------------------------------------------------------------------

# GTSAM core
headers += Factor.h Factor-inl.h Conditional.h 

# Symbolic Inference
sources += SymbolicFactorGraph.cpp SymbolicMultifrontalSolver.cpp SymbolicSequentialSolver.cpp
check_PROGRAMS += tests/testSymbolicFactor tests/testSymbolicFactorGraph tests/testConditional
check_PROGRAMS += tests/testSymbolicBayesNet tests/testVariableIndex tests/testVariableSlots

# Inference
headers += GenericMultifrontalSolver.h GenericMultifrontalSolver-inl.h GenericSequentialSolver.h GenericSequentialSolver-inl.h
sources += inference.cpp VariableSlots.cpp Permutation.cpp VariableIndex.cpp
sources += IndexFactor.cpp IndexConditional.cpp
headers += graph.h graph-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += ClusterTree.h ClusterTree-inl.h
headers += JunctionTree.h JunctionTree-inl.h
headers += EliminationTree.h EliminationTree-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += BayesTreeCliqueBase.h BayesTreeCliqueBase-inl.h
headers += ISAM.h ISAM-inl.h
check_PROGRAMS += tests/testInference
check_PROGRAMS += tests/testFactorGraph
check_PROGRAMS += tests/testFactorGraph
check_PROGRAMS += tests/testBayesTree 
check_PROGRAMS += tests/testISAM
check_PROGRAMS += tests/testEliminationTree
check_PROGRAMS += tests/testClusterTree
check_PROGRAMS += tests/testJunctionTree
check_PROGRAMS += tests/testPermutation

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------

# CCOLAMD include flags are needed due to the bad include paths within the library
# but will not be exposed to users.  
ccolamd_inc = -I$(top_srcdir)/gtsam/3rdparty/CCOLAMD/Include -I$(top_srcdir)/gtsam/3rdparty/UFconfig

headers += $(sources:.cpp=.h)
inferencedir = $(pkgincludedir)/inference
inference_HEADERS = $(headers)
noinst_LTLIBRARIES = libinference.la
libinference_la_SOURCES = $(sources)
AM_CPPFLAGS = $(ccolamd_inc) $(BOOST_CPPFLAGS) -I$(top_srcdir)
AM_LDFLAGS = $(BOOST_LDFLAGS)
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS += $(boost_serialization)
LDADD = libinference.la ../base/libbase.la ../3rdparty/libccolamd.la
LDADD += ../../CppUnitLite/libCppUnitLite.a 
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

