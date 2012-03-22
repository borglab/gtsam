# This is FindWrap.cmake
# CMake module to locate the Wrap tool and header after installation package
# The following variables will be defined:
#
# Wrap_FOUND          : TRUE if the package has been successfully found
# Wrap_CMD            : command for executing wrap
# Wrap_INCLUDE_DIR    : paths to Wrap's INCLUDE directories

# Find include dir
find_path(_Wrap_INCLUDE_DIR wrap/matlab.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" /usr/local/include /usr/include )

# Find the installed executable
find_program(_Wrap_CMD NAMES wrap 
    PATHS ${CMAKE_INSTALL_PREFIX}/bin "$ENV{HOME}/bin" /usr/local/bin /usr/bin )
        
set (Wrap_INCLUDE_DIR  ${_Wrap_INCLUDE_DIR} CACHE STRING "Wrap INCLUDE directories")
set (Wrap_CMD  ${_Wrap_CMD} CACHE STRING "Wrap executable location")

# handle the QUIETLY and REQUIRED arguments and set LIBXML2_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Wrap DEFAULT_MSG
                                  _Wrap_INCLUDE_DIR _Wrap_CMD)

mark_as_advanced(_Wrap_INCLUDE_DIR _Wrap_CMD )
 



