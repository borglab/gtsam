# Look for lpsolve
#
# Set
#  lpsolve_FOUND = TRUE
#  lpsolve_INCLUDE_DIRS = /usr/local/include
#  lpsolve_LIBRARY = /usr/local/lib/liblpsolve.so
#  lpsolve_LIBRARY_DIRS = /usr/local/lib

find_library (lpsolve_LIBRARY liblpsolve55.a
  PATHS "${CMAKE_SOURCE_DIR}/gtsam/3rdparty/lp_solve_5.5/lpsolve55")

message("Finding lpsolve lib: " ${lpsolve_LIBRARY})

if (lpsolve_LIBRARY)
  get_filename_component (lpsolve_LIBRARY_DIRS
    "${lpsolve_LIBRARY}" PATH)
  get_filename_component (_ROOT_DIR "${lpsolve_LIBRARY_DIRS}" PATH)
  set (lpsolve_FOUND TRUE)
  set (lpsolve_INCLUDE_DIRS "${_ROOT_DIR}")
  unset (_ROOT_DIR)
  message("lpsolve_INCLUDE_DIRS: " ${lpsolve_INCLUDE_DIRS})
  if (NOT EXISTS "${lpsolve_INCLUDE_DIRS}/lp_lib.h")
    unset (lpsolve_INCLUDE_DIRS)
    unset (lpsolve_LIBRARY)
    unset (lpsolve_LIBRARY_DIRS)
  endif ()
endif ()

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (lpsolve DEFAULT_MSG lpsolve_LIBRARY_DIRS lpsolve_LIBRARY lpsolve_INCLUDE_DIRS)
mark_as_advanced (lpsolve_LIBRARY_DIRS lpsolve_LIBRARY lpsolve_INCLUDE_DIRS)
