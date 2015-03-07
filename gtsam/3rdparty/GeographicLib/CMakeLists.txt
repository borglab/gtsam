project (GeographicLib)

# Version information
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 35)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
if (PROJECT_VERSION_PATCH GREATER 0)
  set (PROJECT_VERSION "${PROJECT_VERSION}.${PROJECT_VERSION_PATCH}")
endif ()

if (DEFINED CPACK_PACKAGE_VERSION_COUNT)

  # majic (version 0.1.9 and later) invokes cmake defining, e.g.,
  #   -D CPACK_PACKAGE_VERSION=1.35-SNAPSHOT
  #   -D CPACK_PACKAGE_VERSION_COUNT=2
  #   -D CPACK_PACKAGE_VERSION_MAJOR=1
  #   -D CPACK_PACKAGE_VERSION_MINOR=35
  #   -D CPACK_PACKAGE_VERSION_SUFFIX=-SNAPSHOT
  # Check that the first two version numbers are consistent.
  if (CPACK_PACKAGE_VERSION_COUNT EQUAL 2)
    set (CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
  elseif (CPACK_PACKAGE_VERSION_COUNT LESS 2)
    message (FATAL_ERROR "CPACK_PACKAGE_VERSION_COUNT must be 2 or more")
  endif ()
  if (NOT (
        CPACK_PACKAGE_VERSION_MAJOR EQUAL PROJECT_VERSION_MAJOR AND
        CPACK_PACKAGE_VERSION_MINOR EQUAL PROJECT_VERSION_MINOR))
    message (FATAL_ERROR "Inconsistency in CPACK and PROJECT version numbers")
  endif ()
  set (PROJECT_VERSION_PATCH ${CPACK_PACKAGE_VERSION_PATCH})
  set (PROJECT_VERSION ${CPACK_PACKAGE_VERSION})

else ()

  set (CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
  set (CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
  set (CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})
  set (CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

endif ()

# The library version tracks the numbering given by libtool in the
# autoconf set up.
set (LIBVERSION 10)
set (LIBVERSIONFULL 10.1.2)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

cmake_minimum_required (VERSION 2.8.4) # This version was released 2011-02-16

# User-settable variables

# (1) COMMON_INSTALL_PATH governs the installation convention.  If it
# is on ON (the Linux default), the installation is to a common
# directory, e.g., /usr/local.  If it is OFF (the Windows default),
# the installation directory contains the package name, e.g.,
# c:/pkg/GeographicLib-1.22.  The installation directories for the
# documentation, cmake configuration, python and matlab interfaces all
# depend on the variable with deeper paths relative to
# CMAKE_INSTALL_PREFIX being used when it's ON.

if (WIN32)
  option (COMMON_INSTALL_PATH "Use a common installation path for packages" OFF)
else ()
  option (COMMON_INSTALL_PATH "Use a common installation path for packages" ON)
endif ()

# The use of PACKAGE_PATH and INSTALL_PATH is now DEPRECATED.
# (2) PACKAGE_PATH and INSTALL_PATH govern the find_package search
# path and the installation directory.  (find_package is not used by
# GeographicLib since it doesn't depend on other packages.  However
# PACKAGE_PATH is used here for uniformity with other packages which
# adopt the same conventions.)
#
# If PACKAGE_PATH is defined, it is prepended to CMAKE_PREFIX_PATH.
#
# If INSTALL_PATH is not specified but PACKAGE_PATH is, then
# INSTALL_PATH is set to
#   ${PACKAGE_PATH}, if COMMON_INSTALL_PATH is ON;
#   ${PACKAGE_PATH}/${PROJECT_NAME}-${PROJECT_VERSION}, otherwise.
#
# If INSTALL_PATH is now defined, then set CMAKE_INSTALL_PREFIX to
# INSTALL_PATH.
#
# Typically, only PACKAGE_PATH needs to be specified, e.g.,
# cmake -D PACKAGE_PATH=/opt .. (on Linux)
#   => CMAKE_PREFIX_PATH=/opt   CMAKE_INSTALL_PREFIX=/opt
# cmake -D PACKAGE_PATH=C:/pkg .. (on Windows)
#   => CMAKE_PREFIX_PATH=C:/pkg CMAKE_INSTALL_PREFIX=C:/pkg/GeographicLib-1.22

if (PACKAGE_PATH)
  set (CMAKE_PREFIX_PATH ${PACKAGE_PATH} ${CMAKE_PREFIX_PATH})
  message (STATUS "CMAKE_PREFIX_PATH set to ${CMAKE_PREFIX_PATH}")
endif ()

if (NOT INSTALL_PATH AND PACKAGE_PATH)
  if (COMMON_INSTALL_PATH)
    set (INSTALL_PATH ${PACKAGE_PATH} CACHE PATH "Installation directory" FORCE)
  else ()
    set (INSTALL_PATH ${PACKAGE_PATH}/${PROJECT_NAME}-${PROJECT_VERSION}
      CACHE PATH "Installation directory" FORCE)
  endif ()
endif ()
if (INSTALL_PATH)
  file (TO_CMAKE_PATH ${INSTALL_PATH} CMAKE_INSTALL_PREFIX)
  message (STATUS "CMAKE_INSTALL_PREFIX set to ${CMAKE_INSTALL_PREFIX}")
endif ()

# (3) Where to look for data files.  Various classes look in the geoids,
# gravity, magnetic, subdirectories of ${GEOGRAPHICLIB_DATA}.
if (WIN32)
  # The binary installers for the data files for Windows are created
  # with Inno Setup which uses {commonappdata}.  On most Windows
  # systems this is
  # "C:/Documents and Settings/All Users/Application Data", while on
  # newer systems (Windows 7), it is C:/ProgramData.  However the
  # longer name "works" on all Windows systems.
  set (GEOGRAPHICLIB_DATA
    "C:/Documents and Settings/All Users/Application Data/GeographicLib"
    CACHE PATH "Location for data for GeographicLib")
else ()
  set (GEOGRAPHICLIB_DATA
    "/usr/local/share/GeographicLib"
    CACHE PATH "Location for data for GeographicLib")
endif ()

# (4) Build which libraries?  Possible values are SHARED, STATIC, BOTH.
if (MSVC)
  set (GEOGRAPHICLIB_LIB_TYPE STATIC CACHE STRING
    "Types of library generated: SHARED, STATIC (default), or BOTH")
else ()
  set (GEOGRAPHICLIB_LIB_TYPE SHARED CACHE STRING
    "Types of library generated: SHARED (default), STATIC, or BOTH")
endif ()
set_property (CACHE GEOGRAPHICLIB_LIB_TYPE
  PROPERTY STRINGS "SHARED" "STATIC" "BOTH")

if ("${GEOGRAPHICLIB_LIB_TYPE}" STREQUAL "BOTH")
  set (GEOGRAPHICLIB_SHARED_LIB ON)
  set (GEOGRAPHICLIB_STATIC_LIB ON)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 2)
elseif ("${GEOGRAPHICLIB_LIB_TYPE}" STREQUAL "SHARED")
  set (GEOGRAPHICLIB_SHARED_LIB ON)
  set (GEOGRAPHICLIB_STATIC_LIB OFF)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 1)
elseif ("${GEOGRAPHICLIB_LIB_TYPE}" STREQUAL "STATIC")
  set (GEOGRAPHICLIB_SHARED_LIB OFF)
  set (GEOGRAPHICLIB_STATIC_LIB ON)
  set (GEOGRAPHICLIB_LIB_TYPE_VAL 0)
else ()
  message (FATAL_ERROR
    "Bad value of GEOGRAPHICLIB_LIB_TYPE, \"${GEOGRAPHICLIB_LIB_TYPE}\" "
    "(should be SHARED, STATIC or BOTH)")
endif ()

if (GEOGRAPHICLIB_STATIC_LIB)
  set (PROJECT_STATIC_LIBRARIES GeographicLib_STATIC)
  set (PROJECT_STATIC_DEFINITIONS -DGEOGRAPHICLIB_SHARED_LIB=0)
else ()
  set (PROJECT_STATIC_LIBRARIES)
  set (PROJECT_STATIC_DEFINITIONS)
endif ()

if (GEOGRAPHICLIB_SHARED_LIB)
  set (PROJECT_SHARED_LIBRARIES GeographicLib)
  set (PROJECT_LIBRARIES ${PROJECT_SHARED_LIBRARIES})
  set (PROJECT_SHARED_DEFINITIONS -DGEOGRAPHICLIB_SHARED_LIB=1)
  set (PROJECT_DEFINITIONS ${PROJECT_SHARED_DEFINITIONS})
else ()
  set (PROJECT_SHARED_LIBRARIES)
  set (PROJECT_LIBRARIES ${PROJECT_STATIC_LIBRARIES})
  set (PROJECT_SHARED_DEFINITIONS)
  set (PROJECT_DEFINITIONS ${PROJECT_STATIC_DEFINITIONS})
endif ()

# (5) Compile the Matlab interfaces?  Skip Matlab compilation if OFF
set (MATLAB_COMPILER OFF CACHE STRING
  "Compiler for matlab/octave interface: mex or mkoctfile or OFF")
set_property (CACHE MATLAB_COMPILER PROPERTY STRINGS "mex" "mkoctfile" OFF)

# (6) Create the documentation?  This depends on whether doxygen can be
# found.  If this is OFF, then links will be provided to the online
# documentation on Sourceforge.
option (GEOGRAPHICLIB_DOCUMENTATION
  "Use doxygen to create the documentation" OFF)

# (7) Build .NET wrapper library NETGeographicLib.  This only applies to
# Windows.  Default is OFF, because, currently, most people don't use
# this interface.
option (BUILD_NETGEOGRAPHICLIB "Build NETGeographicLib library" OFF)

# (8) Set the default "real" precision.  This should probably be left
# at 2 (double).
set (GEOGRAPHICLIB_PRECISION 2 CACHE STRING
  "Default real precision: 1 = float, 2 = double, 3 = long double")
set_property (CACHE GEOGRAPHICLIB_PRECISION PROPERTY STRINGS 1 2 3)

# (9) When making a binary package, should we include the debug version
# of the library?  This applies to MSVC only, because that's the
# platform where debug and release compilations do not inter-operate.
# It requires building as follows:
#   cmake --build . --config Debug   --target ALL_BUILD
#   cmake --build . --config Release --target ALL_BUILD
#   cmake --build . --config Release --target PACKAGE
option (PACKAGE_DEBUG_LIBS
  "Include debug versions of library in binary package" OFF)

set (LIBNAME Geographic)
if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # For multi-config systems and for Visual Studio, the debug version of
  # the library is called Geographic_d.
  set (CMAKE_DEBUG_POSTFIX _d)
endif ()

if (NOT MSVC)
  # Set the run time path for shared libraries for non-Windows machines.
  # (1) include link path for external packages (not needed with
  # GeographicLib because there are no external packages).
  set (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  # (2) include installed path for GeographicLib.
  if (NOT APPLE)
    # Use relative path so that package is relocatable
    set (CMAKE_INSTALL_RPATH "\$ORIGIN/../lib${LIB_SUFFIX}")
  else ()
    # Need absolute path with MacOSx
    set (CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib${LIB_SUFFIX}")
  endif ()
endif ()

if (NOT (CYGWIN OR ANDROID))
  # cygwin and android have a long double but the support for ::cbrtl,
  # etc., is missing
  include (CheckTypeSize)
  check_type_size ("long double" LONG_DOUBLE BUILTIN_TYPES_ONLY)
endif ()
include (TestBigEndian)
test_big_endian (WORDS_BIGENDIAN)

# Create a Config.h to expose system information to the compiler
configure_file (
  include/GeographicLib/Config.h.in
  include/GeographicLib/Config.h )

# The documentation depends on doxygen.  Need version 1.8.1.2 or later
# for support of greek letters and math symbols.
if (GEOGRAPHICLIB_DOCUMENTATION)
  set (DOXYGEN_SKIP_DOT ON)
  find_package (Doxygen 1.8.1.2)
  if (DOXYGEN_FOUND)
    execute_process (COMMAND ${DOXYGEN_EXECUTABLE} --version
      OUTPUT_VARIABLE DOXYGEN_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)
    if (DOXYGEN_VERSION VERSION_LESS 1.4.0)
      set (DOXYGEN_FOUND FALSE)
      message (STATUS "Doxygen version found, ${DOXYGEN_VERSION}, is too old")
    endif ()
  endif ()
endif ()

# The man pages are written as pod files and converted to nroff format,
# C++ code, and html.  Because this require tools that may not be
# available on an end-user's system, the creation of the final
# documentation is therefore only done in "MAINTAINER" mode.  The
# maintainer runs "make distrib-all" which installs the transformed
# documentation files into the source tree.
if (NOT WIN32 AND NOT APPLE)
  find_program (HAVE_POD2MAN pod2man)
  find_program (HAVE_POD2HTML pod2html)
  find_program (HAVE_COL col)
endif ()
if (HAVE_POD2MAN AND HAVE_POD2HTML AND HAVE_COL)
  set (MAINTAINER ON)
else ()
  set (MAINTAINER OFF)
endif ()
if (MAINTAINER)
  add_custom_target (distrib-all)
  add_dependencies (distrib-all distrib-man)
endif ()

# Look for the tool to compile the Matlab interfaces.
if (MATLAB_COMPILER)
  if (WIN32)
    set (MATLAB_COMPILER_EXT ".bat")
  else ()
    set (MATLAB_COMPILER_EXT "")
  endif ()
  find_program (MEX "${MATLAB_COMPILER}${MATLAB_COMPILER_EXT}")
  if (MATLAB_COMPILER MATCHES "mex")
    get_filename_component (MATLABDIR "${MEX}" REALPATH)
    get_filename_component (MATLABDIR "${MATLABDIR}" PATH)
    find_program (MEXEXTPROG "mexext${MATLAB_COMPILER_EXT}"
      PATHS "${MATLABDIR}")
    execute_process (COMMAND "${MEXEXTPROG}"
      OUTPUT_VARIABLE MEXEXT OUTPUT_STRIP_TRAILING_WHITESPACE)
    set (MEXOPTIONS "-largeArrayDims")
  else ()
    set (MEXEXT "mex")
    set (MEXOPTIONS "--mex")
  endif ()
  if (NOT MSVC)
    # mex files are shared objects => require static lib to be built with
    # position independent code
    set (CMAKE_POSITION_INDEPENDENT_CODE ON)
  endif ()
  if (NOT MEX)
    message (WARNING
      "Cannot find Matlab compiler ${MATLAB_COMPILER}${MATLAB_COMPILER_EXT}")
  elseif (NOT MEXEXT)
    set (MEX OFF)
    message (WARNING "Cannot determine extension for Matlab compiled code")
  endif ()
endif ()

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# Make the compiler more picky.
if (MSVC)
  string (REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else ()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  # check for C++11 support.  If available, the C++11 static_assert and
  # various math functions (std::atanh, etc.) are used.  This flag is
  # *not* propagated to clients that use GeographicLib.  However, this
  # is of no consequence.  When the client code is being compiled (and
  # the GeographicLib headers being included), work-alike substitutions
  # for static_assert and std::atanh are used.
  include (CheckCXXCompilerFlag)
  set (CXX11FLAG "-std=c++11")
  check_cxx_compiler_flag (${CXX11FLAG} CXX11TEST1)
  if (NOT CXX11TEST1)
    set (CXX11FLAG "-std=c++0x")
    check_cxx_compiler_flag (${CXX11FLAG} CXX11TEST2)
    if (NOT CXX11TEST2)
      unset (CXX11FLAG)
    endif ()
  endif ()
  if (CXX11FLAG)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11FLAG}")
  endif ()
endif ()

if (APPLE)
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "i.86" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "amd64" OR
      CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    set (CMAKE_OSX_ARCHITECTURES "i386 -arch x86_64")
  endif ()
endif ()

# The list of tools (to be installed into, e.g., /usr/local/bin)
#set (TOOLS CartConvert ConicProj GeodesicProj GeoConvert GeodSolve
#  GeoidEval Gravity MagneticField Planimeter TransverseMercatorProj)
# The list of scripts (to be installed into, e.g., /usr/local/sbin)
set (SCRIPTS
  geographiclib-get-geoids geographiclib-get-gravity geographiclib-get-magnetic)

set_property (GLOBAL PROPERTY USE_FOLDERS ON)

# Set the include directories.  Look in ${PROJECT_BINARY_DIR}/include
# first because that's where Config.h will be
include_directories ("${PROJECT_BINARY_DIR}/include" include)

# The list of subdirectories to process
add_subdirectory (src)
add_subdirectory (include/GeographicLib)
# add_subdirectory (tools)
add_subdirectory (man)
add_subdirectory (doc)
add_subdirectory (matlab)
add_subdirectory (python/geographiclib)
# if (GEOGRAPHICLIB_PRECISION EQUAL 2)
#   # The examples assume double precision
#   add_subdirectory (examples)
# endif ()
if (BUILD_NETGEOGRAPHICLIB)
  set (NETGEOGRAPHICLIB_LIBRARIES NETGeographicLib)
  set (NETLIBNAME NETGeographic)
  add_subdirectory (dotnet/NETGeographicLib)
  if (GEOGRAPHICLIB_PRECISION EQUAL 2)
    add_subdirectory (dotnet/examples/ManagedCPP)
  endif ()
endif ()
add_subdirectory (cmake)
if (EXISTS ${PROJECT_SOURCE_DIR}/tests/CMakeLists.txt)
  add_subdirectory (tests)
endif ()

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

# The configuration of CPack is via variable that need to be set before
# the include (CPack).
set (CPACK_PACKAGE_CONTACT charles@karney.com)
set (CPACK_PACKAGE_VENDOR "GeographicLib")
set (CPACK_PACKAGE_DESCRIPTION_SUMMARY
  "GeographicLib library, utilities, and documentation")
# The list of files to be excluded from the source distribution.
set (CPACK_SOURCE_IGNORE_FILES
  "#"
  "~\$"
  "/\\\\.git"
  "${PROJECT_SOURCE_DIR}/BUILD"
  "${PROJECT_SOURCE_DIR}/(tests|testdata|cgi-bin|.*\\\\.cache)/"
  "${PROJECT_SOURCE_DIR}/(distrib|.*-distrib|.*-installer|geodesic-papers)/"
  "${PROJECT_SOURCE_DIR}/[^/]*\\\\.(html|kmz|pdf)\$"
  "${PROJECT_SOURCE_DIR}/(autogen|biblio|js-compress)\\\\.sh\$"
  "${PROJECT_SOURCE_DIR}/(geodesic-biblio.txt|makefile-admin|[^/]*\\\\.png)\$"
  "${PROJECT_SOURCE_DIR}/matlab/matlab-.*blurb.txt\$" )
set (CPACK_SOURCE_GENERATOR TGZ)

set (CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE.txt)
set (CPACK_PACKAGE_INSTALL_DIRECTORY "${PROJECT_NAME}-${PROJECT_VERSION}")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}")

if (WIN32)
  # The Windows binary packager is NSIS.  Set the necessary variables
  # for this.
  set (CPACK_NSIS_CONTACT "charles@karney.com")
  set (CPACK_NSIS_URL_INFO_ABOUT "http://geographiclib.sf.net")
  set (CPACK_NSIS_HELP_LINK "mailto:charles@karney.com")
  if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    # Hardcode the prefix for Visual Studio 10
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\pkg-vc10-x64")
    set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-win64")
    set (CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME} x64 ${PROJECT_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-x64-${PROJECT_VERSION}")
  else ()
    # Hardcode the prefix for Visual Studio 10
    set (CPACK_NSIS_INSTALL_ROOT "C:\\\\pkg-vc10")
    set (CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY}-win32")
    set (CPACK_NSIS_PACKAGE_NAME "${PROJECT_NAME} ${PROJECT_VERSION}")
    set (CPACK_PACKAGE_INSTALL_REGISTRY_KEY
      "${PROJECT_NAME}-${PROJECT_VERSION}")
  endif ()
  set (CPACK_NSIS_DISPLAY_NAME ${CPACK_NSIS_PACKAGE_NAME})
  set (CPACK_NSIS_MENU_LINKS
    "http://geographiclib.sf.net/${PROJECT_VERSION}/index.html"
    "Library documentation"
    "http://geographiclib.sf.net/${PROJECT_VERSION}/utilities.html"
    "Utilities documentation"
    "http://geographiclib.sf.net" "GeographicLib home page"
    "http://sf.net/projects/geographiclib/" "Main project page")
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

# Another maintainer-specific target is building the source distribution
# via the target dist.  This calls package_source to make a tar.gz file.
# However this needs to be touched up to support the vanilla Makefiles
# provided with GeographicLib.  This entails
# (1) creating Makefile (which includes Makefile.mk);
# (2) creating a bare-bones Config.h (with just the version information);
# (3) making sure that make thinks the generated documentation files are
# up-to-date.
# Then a new tar.gz file and zip file are created.  To avoid potential
# problems with directory permissions, tar and zip are told only to
# archive the files.
if (MAINTAINER)
  add_custom_target (dist
    COMMAND ${CMAKE_MAKE_PROGRAM} package_source
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME} &&
      echo include Makefile.mk > Makefile &&
      sed -e "s/Unconfigured/${PROJECT_VERSION}/"
      -e "s/MAJOR .*/MAJOR ${CPACK_PACKAGE_VERSION_MAJOR}/"
      -e "s/MINOR .*/MINOR ${CPACK_PACKAGE_VERSION_MINOR}/"
      -e "s/PATCH .*/PATCH ${CPACK_PACKAGE_VERSION_PATCH}/"
      include/GeographicLib/Config.h > include/GeographicLib/Config.h.new &&
      mv include/GeographicLib/Config.h.new include/GeographicLib/Config.h
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME} &&
      touch man/[A-Za-z]*.usage man/[A-Za-z]*.1 man/[A-Za-z]*.1.html &&
      chmod -R g-w .
    COMMAND
      cd _CPack_Packages/Linux-Source/TGZ &&
      find ${CPACK_SOURCE_PACKAGE_FILE_NAME} -type f |
      tar cfzT ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.tar.gz -
    COMMAND
      rm -f ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.zip &&
      rsync -a --delete
      _CPack_Packages/Linux-Source/TGZ/${CPACK_SOURCE_PACKAGE_FILE_NAME}
      _CPack_Packages/Linux-Source/TGZ.DOS/ &&
      cd _CPack_Packages/Linux-Source/TGZ.DOS &&
      find . -type f |
      egrep  '/\(doxyfile.*\\.in|MANIFEST.in|NEWS|AUTHORS|INSTALL|pom\\.xml|dummy.*\\.in|.*\\.\(cpp|hpp|h\\.in|txt|pro|usage|pod|py|m|mac|cmake\\.in|cmake|h|js|c|for|dox|cs|vb|inc|java|html\\.in\)\)$$' |
      xargs unix2dos -q -k &&
      find ${CPACK_SOURCE_PACKAGE_FILE_NAME} -type f |
      zip -q ${CMAKE_BINARY_DIR}/${CPACK_SOURCE_PACKAGE_FILE_NAME}.zip -@
  )
  add_dependencies (dist distrib-all)
endif ()

# Add a test target; the tests are in tools.
enable_testing ()
