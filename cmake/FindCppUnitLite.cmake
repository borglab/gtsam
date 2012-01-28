# This is FindCppUnitLite.cmake
# CMake module to locate the CppUnit package
# The following variables will be defined:
#
# CppUnitLite_FOUND          : TRUE if the package has been successfully found
# CppUnitLite_INCLUDE_DIRS   : paths to CppUnitLite's INCLUDE directories
# CppUnitLite_LIBS           : paths to CppUnitLite's libraries


# Find include dirs
find_path(_CppUnitLite_INCLUDE_DIR CppUnitLite/Test.h
    PATHS ${GTSAM_ROOT} ${CMAKE_INSTALL_PREFIX}/include ${HOME}/include /usr/local/include /usr/include )

# Find libraries
find_library(_CppUnitLite_LIB NAMES CppUnitLite
    HINTS ${_CppUnitLite_INCLUDE_DIR}/build/CppUnitLite  ${_CppUnitLite_INCLUDE_DIR}/CppUnitLite)

set (CppUnitLite_INCLUDE_DIRS ${_CppUnitLite_INCLUDE_DIR})
set (CppUnitLite_LIBS         ${_CppUnitLite_LIB})



# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CppUnitLite DEFAULT_MSG
                                  _CppUnitLite_INCLUDE_DIR _CppUnitLite_LIB)

mark_as_advanced(_CppUnitLite_INCLUDE_DIR _CppUnitLite_LIB )
 



