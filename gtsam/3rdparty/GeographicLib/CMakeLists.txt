cmake_minimum_required (VERSION 3.17.0)
project (GeographicLib)

# Version information
set (PROJECT_VERSION_MAJOR 2)
set (PROJECT_VERSION_MINOR 5)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
if (PROJECT_VERSION_PATCH GREATER 0)
  set (PROJECT_VERSION "${PROJECT_VERSION}.${PROJECT_VERSION_PATCH}")
endif ()
set (PROJECT_VERSION_SUFFIX "")
set (PROJECT_FULLVERSION ${PROJECT_VERSION}${PROJECT_VERSION_SUFFIX})

if (DEFINED CPACK_PACKAGE_VERSION_COUNT)

  # majic (version 0.1.9 and later) invokes cmake defining, e.g.,
  #   -D CPACK_PACKAGE_VERSION=1.37-001-SNAPSHOT
  #   -D CPACK_PACKAGE_VERSION_COUNT=2
  #   -D CPACK_PACKAGE_VERSION_MAJOR=1
  #   -D CPACK_PACKAGE_VERSION_MINOR=36
  #   -D CPACK_PACKAGE_VERSION_SUFFIX=-001-SNAPSHOT
  # Check that the version numbers are consistent.
  if (CPACK_PACKAGE_VERSION_COUNT EQUAL 2)
    set (CPACK_PACKAGE_VERSION_PATCH 0)
  elseif (CPACK_PACKAGE_VERSION_COUNT LESS 2)
    message (FATAL_ERROR "CPACK_PACKAGE_VERSION_COUNT must be 2 or more")
  endif ()
  if (NOT (
        CPACK_PACKAGE_VERSION_MAJOR EQUAL PROJECT_VERSION_MAJOR AND
        CPACK_PACKAGE_VERSION_MINOR EQUAL PROJECT_VERSION_MINOR AND
        CPACK_PACKAGE_VERSION_PATCH EQUAL PROJECT_VERSION_PATCH))
    message (FATAL_ERROR "Inconsistency in CPACK and PROJECT version numbers")
  endif ()
  set (PROJECT_VERSION ${CPACK_PACKAGE_VERSION})

else ()

  set (CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
  set (CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
  set (CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
  set (CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
  set (CPACK_PACKAGE_VERSION_SUFFIX ${PROJECT_VERSION_SUFFIX})

endif ()

set (PACKAGE_DIR "${PROJECT_NAME}-${PROJECT_VERSION}")
set (PACKAGE_NAME "${PROJECT_NAME}-${PROJECT_FULLVERSION}")

# The library version tracks the numbering given by libtool in the
# autoconf set up.
set (LIBVERSION_API 26)
set (LIBVERSION_BUILD 26.1.1)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

if (EXISTS ${PROJECT_SOURCE_DIR}/VERSION)
  # If the file, VERSION, exists then this is a released version of
  # GeographicLib and the processed man pages are available in the
  # source tree
  set (RELEASE ON)
else ()
  set (RELEASE OFF)
endif ()

# User-settable variables

# (1) Where to look for data files.  Various classes look in the geoids,
# gravity, magnetic, subdirectories of ${GEOGRAPHICLIB_DATA}.
if (WIN32)
  # The binary installers for the data files for Windows are created
  # with Inno Setup which uses {commonappdata} which (since Windows
  # Vista) is C:/ProgramData.
  set (GEOGRAPHICLIB_DATA
    "C:/ProgramData/${PROJECT_NAME}"
    CACHE PATH "Location for data for GeographicLib")
else ()
  set (GEOGRAPHICLIB_DATA
    "/usr/local/share/${PROJECT_NAME}"
    CACHE PATH "Location for data for GeographicLib")
endif ()

# (2) Build which libraries?
option (BUILD_SHARED_LIBS "Build as a shared library" ON)

# Ignore BUILD_SHARED_LIBS and build both shared and static libraries.
# (This replaces the variable GEOGRAPHICLIB_LIB_TYPE in earlier
# versions.)
option (BUILD_BOTH_LIBS "Build both shared and static libraries" OFF)

# (3) What sort of build?
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  # Set a default build type for single-configuration cmake generators
  # if no build type is set.
  set (CMAKE_BUILD_TYPE "Release")
endif ()

# (4) Create the documentation?  This depends on whether doxygen can be
# found.  If this is OFF, then links will be provided to the online
# documentation on Sourceforge.
option (BUILD_DOCUMENTATION "Use doxygen to create the documentation" OFF)

# (5) Set the default "real" precision.  This should probably be left
# at 2 (double).
set (GEOGRAPHICLIB_PRECISION 2 CACHE STRING
  "Precision: 1 = float, 2 = double, 3 = extended, 4 = quadruple, 5 = variable")
set_property (CACHE GEOGRAPHICLIB_PRECISION PROPERTY STRINGS 1 2 3 4 5)

# (6) Try to link against boost when building the examples.  The
# NearestNeighbor example optionally uses the Boost library.  Set to ON,
# if you want to exercise this functionality.  Default is OFF, so that
# cmake configuration isn't slowed down looking for Boost.
option (USE_BOOST_FOR_EXAMPLES
  "Look for Boost library when compiling examples" OFF)

# (7) On Mac OS X, build multiple architectures?  Set to ON to build
# i386 and x86_64.  Default is OFF, meaning build for default
# architecture.
option (APPLE_MULTIPLE_ARCHITECTURES
  "Build multiple architectures for Apple systems" OFF)

# (8) Convert warnings into errors?  Default is OFF if RELEASE, else ON.
if (RELEASE)
  option (CONVERT_WARNINGS_TO_ERRORS "Convert warnings into errors?" OFF)
  # BUILD_MANPAGES is ignored in RELEASE mode
  option (BUILD_MANPAGES "Convert pod files to manpages" OFF)
else ()
  option (CONVERT_WARNINGS_TO_ERRORS "Convert warnings into errors?" ON)
  # Convert pod files to man pages.  This only necessary with a
  # non-release tree.
  option (BUILD_MANPAGES "Convert pod files to manpages" ON)
endif ()

# (9) When making a binary package, should we include the debug version
# of the library?  This applies to MSVC only, because that's the
# platform where debug and release compilations do not inter-operate.
# It requires building as follows:
#   cmake --build . --config Debug   --target ALL_BUILD
#   cmake --build . --config Release --target ALL_BUILD
#   cmake --build . --config Release --target PACKAGE
option (PACKAGE_DEBUG_LIBS
  "Include debug versions of library in binary package" OFF)

# Figure out which libraries to build and set GEOGRAPHICLIB_LIB_TYPE_VAL
# (used to initialize GEOGRAPHICLIB_SHARED_LIB in
# include/GeographicLib/Config.h.in)
if (BUILD_BOTH_LIBS)
  set (GEOGRAPHICLIB_SHARED_LIB ON)
  set (GEOGRAPHICLIB_STATIC_LIB ON)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 2)
elseif (BUILD_SHARED_LIBS)
  set (GEOGRAPHICLIB_SHARED_LIB ON)
  set (GEOGRAPHICLIB_STATIC_LIB OFF)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 1)
else ()
  set (GEOGRAPHICLIB_SHARED_LIB OFF)
  set (GEOGRAPHICLIB_STATIC_LIB ON)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 0)
endif ()

if (GEOGRAPHICLIB_STATIC_LIB)
  set (PROJECT_STATIC_LIBRARIES GeographicLib_STATIC)
  set (PROJECT_STATIC_DEFINITIONS -DGEOGRAPHICLIB_SHARED_LIB=0)
else ()
  set (PROJECT_STATIC_LIBRARIES)
  set (PROJECT_STATIC_DEFINITIONS)
endif ()

if (GEOGRAPHICLIB_SHARED_LIB)
  set (PROJECT_SHARED_LIBRARIES GeographicLib_SHARED)
  set (PROJECT_LIBRARIES ${PROJECT_SHARED_LIBRARIES})
  set (PROJECT_SHARED_DEFINITIONS -DGEOGRAPHICLIB_SHARED_LIB=1)
  set (PROJECT_DEFINITIONS ${PROJECT_SHARED_DEFINITIONS})
else ()
  set (PROJECT_SHARED_LIBRARIES)
  set (PROJECT_LIBRARIES ${PROJECT_STATIC_LIBRARIES})
  set (PROJECT_SHARED_DEFINITIONS)
  set (PROJECT_DEFINITIONS ${PROJECT_STATIC_DEFINITIONS})
endif ()

set (PROJECT_INTERFACE_LIBRARIES GeographicLib)
set (PROJECT_ALL_LIBRARIES
  ${PROJECT_STATIC_LIBRARIES}
  ${PROJECT_SHARED_LIBRARIES}
  ${PROJECT_INTERFACE_LIBRARIES})

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # For multi-config systems and for Visual Studio, the debug version of
  # the library is called Geographic_d.
  set (CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "The suffix for debug objects")
else ()
  set (CMAKE_DEBUG_POSTFIX "" CACHE STRING "The suffix for debug objects")
endif ()

# Installation directories.  An empty string or "OFF" disables
# installation of that component.  These are all relative to
# CMAKE_INSTALL_PREFIX.

# Installation is to ${INCDIR}/GeographicLib
set (INCDIR "include" CACHE STRING "Where to install header files")
set (BINDIR "bin" CACHE STRING "Where to install tools")
if (WIN32)
  set (SBINDIR "" CACHE STRING "Where to install admin tools")
else ()
  set (SBINDIR "sbin" CACHE STRING "Where to install admin tools")
endif ()
if (CMAKE_INSTALL_LIBDIR)
  # If it's defined, use this GNUInstallDirs path to support lib, lib64,
  # and lib/<muliarch-tuple> variants
  set (LIBDIR "${CMAKE_INSTALL_LIBDIR}"
    CACHE STRING "Where to install libraries")
else ()
  set (LIBDIR "lib" CACHE STRING "Where to install libraries")
endif ()
set (DLLDIR "bin" CACHE STRING "Where to install dlls")
# Installation is to ${MANDIR}/man1 and ${MANDIR}/man8
set (MANDIR "share/man" CACHE STRING "Where to install the man pages")
set (CMAKEDIR "lib/cmake/${PROJECT_NAME}"
  CACHE STRING "Where to install cmake configs")
set (PKGDIR "lib/pkgconfig" CACHE STRING "Where to install package config")
set (DOCDIR "share/doc/${PROJECT_NAME}"
  CACHE STRING "Where to install documentation")
set (EXAMPLEDIR "share/doc/${PROJECT_NAME}-dev"
  CACHE STRING "Where to install examples")

# The relative path to library directory from the bin directory
if (LIBDIR AND BINDIR)
  file (RELATIVE_PATH RELATIVE_LIBDIR
    "${CMAKE_INSTALL_PREFIX}/${BINDIR}" "${CMAKE_INSTALL_PREFIX}/${LIBDIR}")
else ()
  set (RELATIVE_LIBDIR "")
endif ()

set (LIBNAME "GeographicLib")

if (NOT MSVC)
  # Set the run time path for shared libraries for non-Windows machines.
  # (1) include link path for external packages (not needed with
  # GeographicLib because there are no external packages).  This only
  # makes sense for native builds.
  if (NOT CMAKE_CROSSCOMPILING)
    set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  endif ()
  # (2) include installed path for GeographicLib.
  if (NOT APPLE AND RELATIVE_LIBDIR)
    # Use relative path so that package is relocatable
    set (CMAKE_INSTALL_RPATH "\$ORIGIN/${RELATIVE_LIBDIR}")
  else ()
    # cmake 2.8.12 introduced a way to make the package relocatable.
    # See also INSTALL_RPATH property on the tools.
    set (CMAKE_MACOSX_RPATH ON)
  endif ()
endif ()

include (CheckTypeSize)
check_type_size ("long double" LONG_DOUBLE BUILTIN_TYPES_ONLY)
check_type_size ("double" DOUBLE BUILTIN_TYPES_ONLY)
if (HAVE_LONG_DOUBLE AND (LONG_DOUBLE GREATER DOUBLE))
  set (GEOGRAPHICLIB_HAVE_LONG_DOUBLE TRUE)
else ()
  set (GEOGRAPHICLIB_HAVE_LONG_DOUBLE FALSE)
endif ()
include (TestBigEndian)
test_big_endian (GEOGRAPHICLIB_WORDS_BIGENDIAN)

# We require C++14
set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_STANDARD_REQUIRED ON)
if (GEOGRAPHICLIB_PRECISION EQUAL 4)
  set (CMAKE_CXX_EXTENSIONS ON)  # Need gnu++11 for quadmath
else ()
  set (CMAKE_CXX_EXTENSIONS OFF) # Otherwise stick with the standard
endif ()

# Make the compiler more picky.
include (CheckCXXCompilerFlag)
if (MSVC)
  string (REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  # Turn on parallel builds for Visual Studio
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /std:c++latest /MP")
else ()
  set (FLOAT_CONVERSION_FLAG "-Wfloat-conversion")
  check_cxx_compiler_flag (${FLOAT_CONVERSION_FLAG} FLOAT_CONVERSION)
  if (NOT FLOAT_CONVERSION)
    set (FLOAT_CONVERSION_FLAG)
  endif ()
  set (CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra ${FLOAT_CONVERSION_FLAG}")
endif ()
if (CONVERT_WARNINGS_TO_ERRORS)
  if (MSVC)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /WX")
    set (CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} /WX")
    set (CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /WX")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /WX")
  else ()
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
  endif ()
endif ()

# Tell Intel compiler to do arithmetic accurately.  This is needed to
# stop the compiler from ignoring parentheses in expressions like
# (a + b) + c and from simplifying 0.0 + x to x (which is wrong if
# x = -0.0).
if (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
  if (MSVC)
    set (CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} /fp:precise /Qdiag-disable:11074,11076")
  else ()
    set (CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fp-model precise -diag-disable=11074,11076")
  endif ()
endif ()

# Include directories are specified via target_include_directories in src.

set (HIGHPREC_LIBRARIES)
if (GEOGRAPHICLIB_PRECISION EQUAL 1)
  message (WARNING "Compiling with floats which results in poor accuracy")
elseif (GEOGRAPHICLIB_PRECISION EQUAL 2)
  # This is the default
elseif (GEOGRAPHICLIB_PRECISION EQUAL 3)
  if (WIN32)
    message (WARNING
      "Cannot support long double on Windows, switching to double")
    set (GEOGRAPHICLIB_PRECISION 2)
  endif ()
elseif (GEOGRAPHICLIB_PRECISION EQUAL 4)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    find_package (Boost 1.70 CONFIG)
    if (Boost_FOUND)
      set (HIGHPREC_LIBRARIES quadmath)
    endif ()
  endif ()
  if (NOT HIGHPREC_LIBRARIES)
    message (WARNING "Cannot support quad precision, switching to double")
    set (GEOGRAPHICLIB_PRECISION 2)
  endif ()
elseif (GEOGRAPHICLIB_PRECISION EQUAL 5)
  # Install MPFR C++ version 3.6.9 (2022-01-18) or later from
  # https://github.com/advanpix/mpreal and install mpreal.h in the
  # include directory.  NOTE: MPFR C++ is covered by the GPL; be sure
  # to abide by the terms of this license.
  #
  # For Linux, use system versions of mpfr and gmp.  This is routinely
  # used only on Linux.  It can be made to work on Apple and Windows
  # systems.  However, the following notes are likely out of data...
  #
  # For Apple, use
  # brew install mpfr.  Recent versions of mpfr (3.0 or later) work
  # fine.
  #
  # For Windows, download MPFR-MPIR-x86-x64-MSVC2010.zip from
  # the MPFR C++ site and unpack in the top-level directory.  The
  # Windows build only works with static libraries.
  # NOTE: The Windows build hasn't been tested for a long time.
  # NOTE: mpfr, gmp, and mpir are covered by the LGPL; be sure to
  # abide by the terms of this license.
  if (WIN32)
    if (MSVC AND NOT MSVC_VERSION LESS 1800)
      if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set (_ARCH x64)
      else ()
        set (_ARCH Win32)
      endif ()
      include_directories (mpfr_mpir_x86_x64_msvc2010/mpfr
        mpfr_mpir_x86_x64_msvc2010/mpir/dll/${_ARCH}/Release)
      # These are C libraries so it's OK to use release versions for
      # debug builds.  Also these work for later versions of Visual
      # Studio (specifically version 12).
      link_directories (mpfr_mpir_x86_x64_msvc2010/mpfr/dll/${_ARCH}/Release
        mpfr_mpir_x86_x64_msvc2010/mpir/dll/${_ARCH}/Release)
      set (HIGHPREC_LIBRARIES mpfr mpir)
      # Suppress the myriad of "conditional expression is constant"
      # warnings
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4127")
    endif ()
  else ()
    if (APPLE)
      include_directories (/usr/local/include)
      link_directories (/usr/local/lib)
    endif ()
    # g++ before 4.5 doesn't work (no explicit cast operator)
    if (NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND
          CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.5))
      set (HIGHPREC_LIBRARIES mpfr gmp)
    endif ()
  endif ()
  if (NOT HIGHPREC_LIBRARIES)
    message (WARNING "Cannot support mpfr, switching to double")
    set (GEOGRAPHICLIB_PRECISION 2)
  endif ()
endif ()

if (APPLE AND APPLE_MULTIPLE_ARCHITECTURES)
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "i.86" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "amd64" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    set (CMAKE_OSX_ARCHITECTURES "i386;x86_64")
  endif ()
endif ()

# Create a Config.h to expose system information to the compiler
configure_file (
  include/GeographicLib/Config.h.in
  include/GeographicLib/Config.h
  @ONLY)

# The documentation depends on doxygen.
if (BUILD_DOCUMENTATION)
  set (DOXYGEN_SKIP_DOT ON)
  # Version 1.8.7 or later needed for &hellip;
  find_package (Doxygen 1.8.7)
  if (NOT DOXYGEN_FOUND)
    message (WARNING "Doxygen not found, not building the documentation")
    set (BUILD_DOCUMENTATION OFF)
  endif ()
endif ()

# The man pages are written as pod files and converted to nroff format,
# C++ code, and html.  Because this require tools that may not be
# available on an end-user's system, the creation of the final
# documentation is therefore only done in "MAINTAINER" mode.  The
# maintainer runs "make distrib-man" which installs the transformed
# documentation files into the source tree.  Skip Apple here because
# man/makeusage.sh uses "head --lines -4" to drop the last 4 lines of a
# file and there's no simple equivalent for MacOSX.  MAINTAINER mode
# also supported (a) running autogen.sh to configure the autoconf
# builds, and (b) running git ls-files to run some sanity checks.
# MAINTAINTER mode is not set in RELEASE mode.
if (NOT RELEASE AND BUILD_MANPAGES AND NOT WIN32 AND NOT APPLE)
  find_program (POD2MAN pod2man)
  find_program (POD2HTML pod2html)
  find_program (COL col)
endif ()
if (POD2MAN AND POD2HTML AND COL AND IS_DIRECTORY ${PROJECT_SOURCE_DIR}/.git)
  set (MAINTAINER ON)
  find_program (RSYNC rsync)
else ()
  set (MAINTAINER OFF)
endif ()

# Set output directories for Windows so that executables and dlls are
# put in the same place
if (WIN32)
  # static libaries
  set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  # shared libraries
  set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
  # executables and dlls
  set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
endif ()

# The list of tools (to be installed into, e.g., /usr/local/bin)
set (TOOLS CartConvert ConicProj GeodesicProj GeoConvert GeodSolve
  GeoidEval Gravity IntersectTool MagneticField Planimeter RhumbSolve
  TransverseMercatorProj)
# The list of scripts (to be installed into, e.g., /usr/local/sbin)
set (SCRIPTS geographiclib-get-geoids geographiclib-get-gravity
  geographiclib-get-magnetic)

set_property (GLOBAL PROPERTY USE_FOLDERS ON)

# The list of subdirectories to process
add_subdirectory (src)
add_subdirectory (include/GeographicLib)
add_subdirectory (tools)
add_subdirectory (man)
add_subdirectory (doc)
if (EXAMPLEDIR)
  set (CALLED_FROM_TOPLEVEL ON)
  add_subdirectory (examples)   # This just installs the examples
endif ()
add_subdirectory (cmake)
enable_testing ()
add_subdirectory (tests)
add_subdirectory (experimental)
if (NOT RELEASE)
  add_subdirectory (develop)
endif ()

# make exampleprograms does a fresh cmake configuration and so uses
# find_package to find the just-built version of GeographicLib (via the
# GeographicLib_DIR option to cmake).
if (CMAKE_GENERATOR_PLATFORM)
  set (PLATFORM_FLAG "-A")
endif ()
add_custom_target (exampleprograms
  COMMAND ${CMAKE_COMMAND}
  -G ${CMAKE_GENERATOR} ${PLATFORM_FLAG} ${CMAKE_GENERATOR_PLATFORM}
  -B exampleprograms -S ${PROJECT_SOURCE_DIR}/examples
  -D USE_BOOST_FOR_EXAMPLES=${USE_BOOST_FOR_EXAMPLES}
  -D ${PROJECT_NAME}_DIR=${PROJECT_BINARY_DIR}
  COMMAND ${CMAKE_COMMAND} --build exampleprograms --parallel 8)
add_dependencies (exampleprograms ${PROJECT_LIBRARIES})

# Packaging support; we deal with
# (1) a source distribution: cmake make a tar.gz file and the zip file
# is created from this.  Only the maintainer can do this, because of
# the need to generate additional documentation files.
# (2) a binary distribution: code is included for Linux, Apple, and
# Windows, but only the Windows distribution has been exercised.

# Need to ensure that system dlls get included in a binary distribution
if (NOT DEFINED CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS)
  # Visual Studio Express does include redistributable components so
  # squelch the warning.
  set (CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_NO_WARNINGS ON)
endif ()
set (CMAKE_INSTALL_DEBUG_LIBRARIES ON)
include (InstallRequiredSystemLibraries)

# The configuration of CPack is via variables that need to be set before
# the include (CPack).
set (CPACK_PACKAGE_CONTACT karney@alum.mit.edu)
set (CPACK_PACKAGE_VENDOR "GeographicLib")
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "GeographicLib library, utilities, and documentation")
# The list of files to be excluded from the source distribution.
set (CPACK_SOURCE_IGNORE_FILES
  "#"
  "~\$"
  "/\\\\.git"
  "${PROJECT_SOURCE_DIR}/BUILD"
  "${PROJECT_SOURCE_DIR}/man/(.*\\\\.pod|makeusage\\\\.sh|dummy\\\\..*)\$"
  "${PROJECT_SOURCE_DIR}/cmake/maintainer-.*\\\\.cmake\$"
  "${PROJECT_SOURCE_DIR}/(develop|cgi-bin|.*\\\\.cache)/"
  "${PROJECT_SOURCE_DIR}/(data-distrib|data-installer)/"
  "${PROJECT_SOURCE_DIR}/(archive|scratch|mpfr_mpir_x86_x64_msvc2010)/"
  "${PROJECT_SOURCE_DIR}/.*\\\\.(zip|tar\\\\.gz|bak|lsp)\$"
  "${PROJECT_SOURCE_DIR}/(autogen|biblio)\\\\.sh\$"
  "${PROJECT_SOURCE_DIR}/(makefile-admin|HOWTO-RELEASE.txt)\$")
set (CPACK_SOURCE_GENERATOR "TGZ;ZIP")

set (CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.txt)
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${PACKAGE_DIR}")
set (CPACK_PACKAGE_INSTALL_DIRECTORY "${PACKAGE_DIR}")

if (WIN32)
  # The Windows binary packager is NSIS.  Set the necessary variables
  # for this.
  set (CPACK_NSIS_CONTACT "karney@alum.mit.edu")
  set (CPACK_NSIS_URL_INFO_ABOUT "https://geographiclib.sourceforge.io")
  set (CPACK_NSIS_HELP_LINK "mailto:karney@alum.mit.edu")
  # No provision for accommodating different compiler versions.  However
  # the Visual Studio 14 2015 build works also with Visual Studio 15
  # 2017, Visual Studio 16 2019, and Visual Studio 17 2022.
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\Program Files")
    set (CPACK_PACKAGE_FILE_NAME "${PACKAGE_NAME}-win64")
    set (CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME} x64 ${PROJECT_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-x64-${PROJECT_VERSION}")
  else ()
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\Program Files (x86)")
    set (CPACK_PACKAGE_FILE_NAME "${PACKAGE_NAME}-win32")
    set (CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME} win32 ${PROJECT_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-win32-${PROJECT_VERSION}")
  endif ()
  set (CPACK_NSIS_DISPLAY_NAME ${CPACK_NSIS_PACKAGE_NAME})
  set (CPACK_NSIS_MENU_LINKS
    "https://geographiclib.sourceforge.io/C++/${PROJECT_VERSION}/index.html"
    "Library documentation"
    "https://geographiclib.sourceforge.io/C++/${PROJECT_VERSION}/utilities.html"
    "Utilities documentation"
    "https://geographiclib.sourceforge.io" "GeographicLib home page"
    "https://sourceforge.net/projects/geographiclib/" "Main project page")
  set (CPACK_NSIS_MODIFY_PATH ON)
elseif (APPLE)
  # Not tested
  set (CPACK_GENERATOR Bundle)
  set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-darwin")
else ()
  # Not tested
  set (CPACK_GENERATOR TGZ)
endif ()

include (CPack)

# If MAINTAINER, then prepare the source distribution for RELEASE mode.
# "make dist" does the following:
#   "make package_source" to call the CPack source generator
#   remove stray files which aren't in git
#   runs autogen.sh in the package source directory
#   creates the processing man documentation and installs it too
#   creates a file VERSION to signal that this is a RELEASE source
#   create .tar.gz and .zip archives of the results
if (MAINTAINER)
  include (cmake/maintainer-top.cmake)
endif ()
