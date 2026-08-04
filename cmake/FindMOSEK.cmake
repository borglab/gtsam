# FindMOSEK.cmake - Find MOSEK Fusion API.
#
# Sets:
#   MOSEK_FOUND          - True if MOSEK was found.
#   MOSEK_INCLUDE_DIRS   - Include directories for Fusion.
#   MOSEK_LIBRARIES      - Libraries to link.
#   MOSEK_LIBRARY_DIR    - Runtime library directory, for rpath.
#   MOSEK_FUSION_SOURCES - Optional Fusion C++ sources when the package does
#                          not ship a prebuilt Fusion runtime library.

set(MOSEK_ROOT "" CACHE PATH
    "Path to MOSEK platform directory (contains h/ and bin/), e.g. .../tools/platform/osxaarch64")

set(_MOSEK_PLATFORMS)
if(APPLE)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm64|aarch64)$")
    list(APPEND _MOSEK_PLATFORMS osxaarch64)
  endif()
  list(APPEND _MOSEK_PLATFORMS osx64x86)
elseif(UNIX)
  list(APPEND _MOSEK_PLATFORMS linux64x86)
elseif(WIN32)
  list(APPEND _MOSEK_PLATFORMS win64x86)
endif()

set(_MOSEK_BASE_CANDIDATES)
if(MOSEK_ROOT)
  list(APPEND _MOSEK_BASE_CANDIDATES "${MOSEK_ROOT}")
endif()
if(DEFINED ENV{MOSEK_ROOT} AND NOT "$ENV{MOSEK_ROOT}" STREQUAL "")
  list(APPEND _MOSEK_BASE_CANDIDATES "$ENV{MOSEK_ROOT}")
endif()
if(DEFINED ENV{MOSEK_HOME} AND NOT "$ENV{MOSEK_HOME}" STREQUAL "")
  list(APPEND _MOSEK_BASE_CANDIDATES "$ENV{MOSEK_HOME}")
endif()
if(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
  list(APPEND _MOSEK_BASE_CANDIDATES
      "$ENV{HOME}/Desktop/mosek"
      "$ENV{HOME}/Desktop/MOSEK/mosek"
      "$ENV{HOME}/mosek")
endif()

set(_MOSEK_HINT_PATHS)
foreach(_base IN LISTS _MOSEK_BASE_CANDIDATES)
  if(NOT _base)
    continue()
  endif()

  list(APPEND _MOSEK_HINT_PATHS "${_base}")

  file(GLOB _mosek_version_dirs "${_base}/*/tools/platform")
  foreach(_platform_root IN LISTS _mosek_version_dirs)
    foreach(_platform IN LISTS _MOSEK_PLATFORMS)
      list(APPEND _MOSEK_HINT_PATHS "${_platform_root}/${_platform}")
    endforeach()
  endforeach()

  foreach(_platform IN LISTS _MOSEK_PLATFORMS)
    list(APPEND _MOSEK_HINT_PATHS "${_base}/tools/platform/${_platform}")
  endforeach()
endforeach()
list(REMOVE_DUPLICATES _MOSEK_HINT_PATHS)

find_path(MOSEK_INCLUDE_DIR
    NAMES fusion.h
    HINTS ${_MOSEK_HINT_PATHS}
    PATH_SUFFIXES h
)

find_library(MOSEK_LIBRARY
    NAMES mosek64 mosek64.12.0 mosek64.11.1 mosek64.11.0 mosek64.10.2
    HINTS ${_MOSEK_HINT_PATHS}
    PATH_SUFFIXES bin
)

find_library(MOSEK_FUSION_LIBRARY
    NAMES fusion64 fusion
    HINTS ${_MOSEK_HINT_PATHS}
    PATH_SUFFIXES bin
)

set(MOSEK_FUSION_SOURCES "")
set(MOSEK_FUSION_AVAILABLE FALSE)
set(_MOSEK_FUSION_SRC_DIR "")

if(MOSEK_FUSION_LIBRARY)
  set(MOSEK_FUSION_AVAILABLE TRUE)
else()
  set(_MOSEK_FUSION_SRC_CANDIDATES)
  foreach(_hint IN LISTS _MOSEK_HINT_PATHS)
    list(APPEND _MOSEK_FUSION_SRC_CANDIDATES "${_hint}/src/fusion_cxx")
  endforeach()
  if(MOSEK_ROOT)
    list(APPEND _MOSEK_FUSION_SRC_CANDIDATES "${MOSEK_ROOT}/src/fusion_cxx")
  endif()
  if(MOSEK_INCLUDE_DIR)
    get_filename_component(_MOSEK_PLATFORM_ROOT "${MOSEK_INCLUDE_DIR}" DIRECTORY)
    list(APPEND _MOSEK_FUSION_SRC_CANDIDATES
        "${_MOSEK_PLATFORM_ROOT}/src/fusion_cxx")
  endif()
  list(REMOVE_DUPLICATES _MOSEK_FUSION_SRC_CANDIDATES)

  # Some MOSEK packages ship Fusion C++ as source instead of a runtime library.
  # Check all resolved platform candidates, not just the user-supplied root.
  foreach(_src_dir IN LISTS _MOSEK_FUSION_SRC_CANDIDATES)
    if(EXISTS "${_src_dir}/fusion.cc")
      set(_MOSEK_FUSION_SRC_DIR "${_src_dir}")
      file(GLOB MOSEK_FUSION_SOURCES "${_MOSEK_FUSION_SRC_DIR}/*.cc")
      if(MOSEK_FUSION_SOURCES)
        set(MOSEK_FUSION_AVAILABLE TRUE)
      endif()
      break()
    endif()
  endforeach()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MOSEK DEFAULT_MSG
    MOSEK_INCLUDE_DIR MOSEK_LIBRARY MOSEK_FUSION_AVAILABLE)

if(MOSEK_FOUND)
  set(MOSEK_INCLUDE_DIRS ${MOSEK_INCLUDE_DIR})
  if(_MOSEK_FUSION_SRC_DIR)
    list(APPEND MOSEK_INCLUDE_DIRS "${_MOSEK_FUSION_SRC_DIR}")
    list(REMOVE_DUPLICATES MOSEK_INCLUDE_DIRS)
  endif()
  if(MOSEK_FUSION_LIBRARY)
    set(MOSEK_LIBRARIES ${MOSEK_FUSION_LIBRARY} ${MOSEK_LIBRARY})
  else()
    set(MOSEK_LIBRARIES ${MOSEK_LIBRARY})
  endif()
  get_filename_component(MOSEK_LIBRARY_DIR ${MOSEK_LIBRARY} DIRECTORY)

  if(NOT MOSEK_ROOT)
    get_filename_component(_MOSEK_INCLUDE_PARENT "${MOSEK_INCLUDE_DIR}" DIRECTORY)
    set(MOSEK_ROOT "${_MOSEK_INCLUDE_PARENT}" CACHE PATH
        "Path to MOSEK platform directory (contains h/ and bin/)" FORCE)
  endif()
endif()

mark_as_advanced(MOSEK_INCLUDE_DIR MOSEK_LIBRARY MOSEK_FUSION_LIBRARY)
