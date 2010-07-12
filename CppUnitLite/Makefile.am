#----------------------------------------------------------------------------------------------------
# CppUnitLite
# replaced Makefile with automake for easy linking
#----------------------------------------------------------------------------------------------------

headers = TestHarness.h
sources = Failure.cpp SimpleString.cpp Test.cpp TestRegistry.cpp TestResult.cpp
headers += $(sources:.cpp=.h)

noinst_HEADERS = $(headers)
noinst_LIBRARIES = libCppUnitLite.a
libCppUnitLite_a_SOURCES = $(sources)
