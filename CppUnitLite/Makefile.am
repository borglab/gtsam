#----------------------------------------------------------------------------------------------------
# CppUnitLite
# replaced Makefile with automake for easy linking
#----------------------------------------------------------------------------------------------------

headers = TestHarness.h
sources = Failure.cpp SimpleString.cpp Test.cpp TestRegistry.cpp TestResult.cpp
headers += $(sources:.cpp=.h)

if ENABLE_INSTALL_CPPUNITLITE
CppUnitLitedir = $(includedir)/CppUnitLite
lib_LIBRARIES = libCppUnitLite.a
CppUnitLite_HEADERS = $(headers) 
libCppUnitLite_a_SOURCES = $(sources)
else 
noinst_LIBRARIES = libCppUnitLite.a
noinst_HEADERS = $(headers)
libCppUnitLite_a_SOURCES = $(sources)
endif



