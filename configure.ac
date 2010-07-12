#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(gtsam, 0.0.0, dellaert@cc.gatech.edu)
AM_INIT_AUTOMAKE(gtsam, 0.0.0)
AC_OUTPUT(Makefile CppUnitLite/Makefile colamd/Makefile ldl/Makefile spqr_mini/Makefile base/Makefile inference/Makefile geometry/Makefile nonlinear/Makefile slam/Makefile tests/Makefile wrap/Makefile)
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_HEADER([config.h])
AC_CONFIG_SRCDIR([CppUnitLite/Test.cpp])
AC_CONFIG_SRCDIR([colamd/colamd.c])
AC_CONFIG_SRCDIR([ldl/ldl.cpp])
AC_CONFIG_SRCDIR([spqr_mini/spqr_front.cpp])
AC_CONFIG_SRCDIR([base/DSFVector.cpp])
AC_CONFIG_SRCDIR([geometry/Cal3_S2.cpp])
AC_CONFIG_SRCDIR([inference/GaussianFactor.cpp])
AC_CONFIG_SRCDIR([nonlinear/ConstraintOptimizer.cpp])
AC_CONFIG_SRCDIR([slam/pose2SLAM.cpp])
AC_CONFIG_SRCDIR([tests/testSQP.cpp])
AC_CONFIG_SRCDIR([wrap/wrap.cpp])

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

# search for a blas implementation. Kai: this can not be easily turned off for Mac.
# AX_BLAS(true) 

# enable BLAS with general purpose script
AC_ARG_ENABLE([blas],
     [  --enable-blas    Enable external BLAS library],
     [case "${enableval}" in
       yes) blas=true ;;
       no)  blas=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-blas]) ;;
     esac],[blas=false])

AM_CONDITIONAL([USE_BLAS], test x$blas = xtrue)
AM_CONDITIONAL([USE_BLAS_MACOS], [test x$blas = xtrue && test x$ISMAC = xtrue])
AM_CONDITIONAL([USE_BLAS_LINUX], [test x$blas = xtrue && test x$ISMAC = xfalse])

# enable LAPACK
AC_ARG_ENABLE([lapack],
     [  --enable-lapack    Enable external LAPACK library],
     [case "${enableval}" in
       yes) lapack=true ;;
       no)  lapack=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-lapack]) ;;
     esac],[lapack=false])

AM_CONDITIONAL([USE_LAPACK], test x$lapack = xtrue)
AM_CONDITIONAL([USE_LAPACK_MACOS], [test x$lapack = xtrue && test x$ISMAC = xtrue])
AM_CONDITIONAL([USE_LAPACK_LINUX], [test x$lapack = xtrue && test x$ISMAC = xfalse])

# On Mac, we use the Accelerate framework for BLAS/LAPACK
AM_CONDITIONAL([USE_ACCELERATE_MACOS], [(test x$lapack = xtrue || test x$blas = xtrue) && test x$ISMAC = xtrue])

#enable SparseQR for linear solving
AC_ARG_ENABLE([spqr],
     [  --enable-spqr    Enable SparseQR library support],
     [case "${enableval}" in
       yes) spqr=true ;;
       no)  spqr=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-spqr]) ;;
     esac],[spqr=false])

AM_CONDITIONAL([USE_SPQR], [test x$spqr = xtrue])

# enable profiling
AC_ARG_ENABLE([profiling],
     [  --enable-profiling    Enable profiling],
     [case "${enableval}" in
       yes) profiling=true ;;
       no)  profiling=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-profiling]) ;;
     esac],[profiling=false])

AM_CONDITIONAL([USE_PROFILING], [test x$profiling = xtrue])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
# FIXME: Need to use boost macros to get serialization library linked
#AX_BOOST_BASE([1.37.0])
#AX_BOOST_SERIALIZATION 
#AX_BOOST_BASE([1.33]) # does not work on windows, even after compiling & installing boost manually

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

# ask for toolbox directory
AC_ARG_WITH([toolbox],
            [AS_HELP_STRING([--with-toolbox],
              [specify the matlab toolbox directory for installation (mandatory)])],
            [toolbox=$withval],
            [AC_MSG_FAILURE(
                   [--with-toolbox has to be specified])
            ])
AC_SUBST([toolbox])

# ask for boost directory
AC_ARG_WITH([boost],
            [AS_HELP_STRING([--with-boost],
              [specify the boost directory for installation (mandatory)])],
            [boost=$withval],
            [AC_MSG_FAILURE(
                   [--with-boost has to be specified])
            ])
AC_SUBST([boost])

# ask for boost serialization
AC_ARG_WITH([boost_serialization],
	     [AS_HELP_STRING([--with-boost-serialization],
			      [(optional) use the Serialization library from boost - specify the library linking command with the full name of the library
			        e.g. --with-boost-serialization=-lboost_serialization-gcc-mt-d-1_33_1])],
	     [AC_DEFINE([HAVE_BOOST_SERIALIZATION], ["<boost/foreach.hpp>"], [boost serialization flag]) 
                
		 boost_serialization=$withval ])
AC_SUBST([boost_serialization])
	
AC_OUTPUT
