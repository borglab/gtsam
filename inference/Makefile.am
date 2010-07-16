#----------------------------------------------------------------------------------------------------
# GTSAM core functionality: base classes for inference, as well as symbolic and discrete
#----------------------------------------------------------------------------------------------------

headers =
sources = 
check_PROGRAMS = 

#----------------------------------------------------------------------------------------------------
# base
#----------------------------------------------------------------------------------------------------

# GTSAM core
headers += Key.h SymbolMap.h Factor.h Conditional.h IndexTable.h 
sources += Ordering.cpp 
check_PROGRAMS += tests/testOrdering tests/testKey

# Symbolic Inference
headers += SymbolicConditional.h
sources += SymbolicFactor.cpp SymbolicFactorGraph.cpp SymbolicBayesNet.cpp
check_PROGRAMS += tests/testSymbolicFactor tests/testSymbolicFactorGraph tests/testSymbolicBayesNet  

# Inference
headers += inference.h inference-inl.h
headers += graph.h graph-inl.h
headers += FactorGraph.h FactorGraph-inl.h
headers += ClusterTree.h ClusterTree-inl.h
headers += JunctionTree.h JunctionTree-inl.h
headers += EliminationTree.h EliminationTree-inl.h
headers += BayesNet.h BayesNet-inl.h
headers += BayesTree.h BayesTree-inl.h
headers += ISAM.h ISAM-inl.h
headers += ISAM2.h ISAM2-inl.h
check_PROGRAMS += tests/testFactorGraph tests/testClusterTree tests/testEliminationTree tests/testJunctionTree tests/testBayesTree tests/testISAM

#----------------------------------------------------------------------------------------------------
# discrete
#----------------------------------------------------------------------------------------------------

# Binary Inference
headers += BinaryConditional.h
check_PROGRAMS += tests/testBinaryBayesNet

# Timing tests
noinst_PROGRAMS = tests/timeSymbolMaps

#----------------------------------------------------------------------------------------------------
# Create a libtool library that is not installed
# It will be packaged in the toplevel libgtsam.la as specfied in ../Makefile.am 
# The headers are installed in $(includedir)/gtsam: 
#----------------------------------------------------------------------------------------------------
headers += $(sources:.cpp=.h)
pkginclude_HEADERS = $(headers)
noinst_LTLIBRARIES = libinference.la
libinference_la_SOURCES = $(sources)
AM_CPPFLAGS = -I$(boost) -I$(top_srcdir) -I$(top_srcdir)/colamd -I$(top_srcdir)/base 
AM_CXXFLAGS =

#----------------------------------------------------------------------------------------------------
# rules to build local programs
#----------------------------------------------------------------------------------------------------
TESTS = $(check_PROGRAMS)
AM_LDFLAGS = $(BOOST_LDFLAGS) $(boost_serialization) 
LDADD = libinference.la ../base/libbase.la 
LDADD += ../CppUnitLite/libCppUnitLite.a ../colamd/libcolamd.la ../ldl/libldl.la 
AM_DEFAULT_SOURCE_EXT = .cpp

# rule to run an executable
%.run: % $(LDADD)
	./$^

if USE_LAPACK
AM_CXXFLAGS += -DGT_USE_LAPACK
LDADD += ../spqr_mini/libspqr_mini.la
endif

