#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(gtsam, 0.9.3, dellaert@cc.gatech.edu)
AM_INIT_AUTOMAKE(gtsam, 0.9.3)
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_SRCDIR([CppUnitLite/Test.cpp])
AC_CONFIG_SRCDIR([gtsam/base/DSFVector.cpp])
AC_CONFIG_SRCDIR([gtsam/geometry/Cal3_S2.cpp])
AC_CONFIG_SRCDIR([gtsam/inference/SymbolicFactorGraph.cpp])
AC_CONFIG_SRCDIR([gtsam/linear/GaussianFactor.cpp])
AC_CONFIG_SRCDIR([gtsam/nonlinear/NonlinearOptimizer.cpp])
AC_CONFIG_SRCDIR([gtsam/slam/pose2SLAM.cpp])
AC_CONFIG_SRCDIR([tests/testTupleValues.cpp])
AC_CONFIG_SRCDIR([examples/SimpleRotation.cpp])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Makefile.am])

# For doxygen support
DX_HTML_FEATURE(ON)
DX_CHM_FEATURE(OFF)
DX_CHI_FEATURE(OFF)
DX_MAN_FEATURE(OFF)
DX_RTF_FEATURE(OFF)
DX_XML_FEATURE(OFF)
DX_PDF_FEATURE(OFF)
DX_PS_FEATURE(OFF)
DX_INIT_DOXYGEN(gtsam)

# Check for OS
AC_CANONICAL_HOST    # needs to be called at some point earlier
AM_CONDITIONAL([DARWIN], [case $host_os in darwin*) true;; *) false;; esac]) 

# enable debug variable
AC_ARG_ENABLE([debug],
     [  --enable-debug    Turn on debugging],
     [case "${enableval}" in
       yes) debug=true ;;
       no)  debug=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-debug]) ;;
     esac],[debug=false])

AM_CONDITIONAL([DEBUG], [test x$debug = xtrue])


AC_CANONICAL_HOST
# We need to determine what os we are on to determine if we need to do 
# special things because we are on a mac
case $host_os in
  darwin* )
        # Do something specific for mac
        ISMAC=true
        ;;
  *)
        ISMAC=false
        ;;
esac

# enable profiling
AC_ARG_ENABLE([profiling],
     [  --enable-profiling    Enable profiling],
     [case "${enableval}" in
       yes) profiling=true ;;
       no)  profiling=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-profiling]) ;;
     esac],[profiling=false])

AM_CONDITIONAL([USE_PROFILING], [test x$profiling = xtrue])

# enable serialization in serialization test
AC_ARG_ENABLE([serialization],
     [  --enable-serialization    Enable serialization with boost serialization],
     [case "${enableval}" in
       yes) serialization=true ;;
       no)  serialization=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-serialization]) ;;
     esac],[serialization=false])

AM_CONDITIONAL([ENABLE_SERIALIZATION], [test x$serialization = xtrue])

# enable installation of CppUnitLite with gtsam
AC_ARG_ENABLE([install_cppunitlite],
     [  --enable-install-cppunitlite    Enable installation of CppUnitLite],
     [case "${enableval}" in
       yes) install_cppunitlite=true ;;
       no)  install_cppunitlite=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-install-cppunitlite]) ;;
     esac],[install_cppunitlite=false])

AM_CONDITIONAL([ENABLE_INSTALL_CPPUNITLITE], [test x$install_cppunitlite = xtrue])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC

# Checks for libraries.
LT_INIT

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_CONST
AC_C_INLINE
AC_TYPE_SIZE_T

# Checks for library functions.
AC_FUNC_ERROR_AT_LINE
AC_CHECK_FUNCS([pow sqrt])

# Check for boost
AX_BOOST_BASE([1.40])

# ask for toolbox directory
#AC_ARG_WITH([toolbox],
#            [AS_HELP_STRING([--with-toolbox],
#              [specify the matlab toolbox directory for installation (mandatory)])],
#            [toolbox=$withval],
#            [AC_MSG_FAILURE(
#                   [--with-toolbox has to be specified])
#            ])
#AC_SUBST([toolbox])

AC_CONFIG_FILES([CppUnitLite/Makefile \
gtsam/3rdparty/Makefile \
gtsam/base/Makefile \
gtsam/geometry/Makefile \
gtsam/inference/Makefile \
gtsam/linear/Makefile \
gtsam/nonlinear/Makefile \
gtsam/slam/Makefile gtsam/Makefile \
tests/Makefile \
examples/Makefile \
examples/vSLAMexample/Makefile \
Makefile])
AC_OUTPUT
