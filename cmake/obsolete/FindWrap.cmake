# This is FindWrap.cmake
# DEPRECIATED: Use config file approach to pull in targets from gtsam
# CMake module to locate the Wrap tool and header after installation package
# The following variables will be defined:
#
# Wrap_FOUND          : TRUE if the package has been successfully found
# Wrap_CMD            : command for executing wrap
# Wrap_INCLUDE_DIR    : paths to Wrap's INCLUDE directories

# If gtsam was found by a previous call to FindGTSAM, prefer to find the
# wrap in the same place as that gtsam
if(GTSAM_LIBS)
  # Find the gtsam library path found by a previous call to FindGTSAM
  get_filename_component(_gtsam_lib_dir "${GTSAM_LIBS}" PATH)
  get_filename_component(_gtsam_lib_dir_name "${_gtsam_lib_dir}" NAME)
  # If the gtsam library was in a directory called 'gtsam', it means we found
  # gtsam in the source tree, otherwise (probably 'lib') in an installed location.
  if(_gtsam_lib_dir_name STREQUAL "gtsam")
    get_filename_component(_gtsam_build_dir "${_gtsam_lib_dir}" PATH)
    set(_gtsam_wrap_dir "${_gtsam_build_dir}/wrap")
  else()
    set(_gtsam_wrap_dir "${_gtsam_lib_dir}/../bin")
  endif()
endif()

# Find include dir
find_path(Wrap_INCLUDE_DIR wrap/matlab.h
    PATHS "${GTSAM_INCLUDE_DIR}" ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" /usr/local/include /usr/include
    DOC "Wrap INCLUDE directories")

# Find the installed executable
find_program(Wrap_CMD NAMES wrap 
    PATHS "${_gtsam_wrap_dir}" ${CMAKE_INSTALL_PREFIX}/bin "$ENV{HOME}/bin" /usr/local/bin /usr/bin
    DOC "Wrap executable location")

# handle the QUIETLY and REQUIRED arguments and set Wrap_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Wrap DEFAULT_MSG
                                  Wrap_CMD Wrap_INCLUDE_DIR)

