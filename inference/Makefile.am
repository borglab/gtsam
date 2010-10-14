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
headers += SymbolMap.h Factor-inl.h Conditional.h IndexTable.h 

# Symbolic Inference
sources += Factor.cpp SymbolicFactorGraph.cpp
check_PROGRAMS += tests/testSymbolicFactor tests/testSymbolicFactorGraph tests/testConditional
check_PROGRAMS += tests/testSymbolicBayesNet tests/testVariableIndex tests/testVariableSlots

# Inference
headers += inference-inl.h VariableSlots-inl.h
sources += inference.cpp VariableSlots.cpp Permutation.cpp
headers += graph.h graph-inl.h
headers += VariableIndex.h
headers += FactorGraph.h FactorGraph-inl.h
headers += ClusterTree.h ClusterTree-inl.h
headers += JunctionTree.h JunctionTree-inl.h
#headers += EliminationTree.h EliminationTree-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += ISAM.h ISAM-inl.h
check_PROGRAMS += tests/testFactorGraph
check_PROGRAMS += tests/testFactorGraph
check_PROGRAMS += tests/testBayesTree 
check_PROGRAMS += tests/testISAM
#check_PROGRAMS += tests/testEliminationTree
check_PROGRAMS += tests/testClusterTree
check_PROGRAMS += tests/testJunctionTree

#----------------------------------------------------------------------------------------------------
# discrete
#----------------------------------------------------------------------------------------------------

# Binary Inference
#headers += BinaryConditional.h
#check_PROGRAMS += tests/testBinaryBayesNet

# Timing tests
#noinst_PROGRAMS = tests/timeSymbolMaps

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
inferencedir = $(pkgincludedir)/inference
inference_HEADERS = $(headers)
noinst_LTLIBRARIES = libinference.la
libinference_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(SparseInc) -I$(top_srcdir)/..
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) -L$(SparseLib) -lccolamd
LDADD = libinference.la ../base/libbase.la
LDADD += ../CppUnitLite/libCppUnitLite.a
AM_DEFAULT_SOURCE_EXT = .cpp

if USE_ACCELERATE_MACOS
AM_LDFLAGS += -Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate
endif

# rule to run an executable
%.run: % $(LDADD)
	./$^

# rule to run executable with valgrind
%.valgrind: % $(LDADD)
	valgrind ./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
endif

