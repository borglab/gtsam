# This is FindCppUnitLite.cmake
# CMake module to locate the CppUnit package
# The following variables will be defined:
#
# CppUnitLite_FOUND          : TRUE if the package has been successfully found
# CppUnitLite_INCLUDE_DIR    : paths to CppUnitLite's INCLUDE directories
# CppUnitLite_LIBS           : paths to CppUnitLite's libraries

# Find include dirs
find_path(_CppUnitLite_INCLUDE_DIR CppUnitLite/Test.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" /usr/local/include /usr/include )

# Find libraries
find_library(_CppUnitLite_LIB NAMES CppUnitLite
  HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" /usr/local/lib /usr/lib)

set (CppUnitLite_INCLUDE_DIR  ${_CppUnitLite_INCLUDE_DIR} CACHE STRING "CppUnitLite INCLUDE directories")
set (CppUnitLite_LIBS         ${_CppUnitLite_LIB} CACHE STRING "CppUnitLite libraries")

# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CppUnitLite DEFAULT_MSG
                                  _CppUnitLite_INCLUDE_DIR _CppUnitLite_LIB)

mark_as_advanced(_CppUnitLite_INCLUDE_DIR _CppUnitLite_LIB )
 



