#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(gtsam, 0.0.0, dellaert@cc.gatech.edu)
AM_INIT_AUTOMAKE(gtsam, 0.0.0)
AC_OUTPUT(Makefile cpp/Makefile wrap/Makefile)
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([cpp/cal3_S2.cpp])
AC_CONFIG_HEADER([cpp/config.h])
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

# ask for CppUnitLite directory
#AC_ARG_WITH([CppUnitLite],
#            [AS_HELP_STRING([--with-CppUnitLite],
#              [specify the directory of CppUnitLite library (mandatory)])],
#            [CppUnitLite=$withval],
#            [AC_MSG_FAILURE(
#                   [--with-CppUnitLite has to be specified])
#            ])
#AC_SUBST([CppUnitLite])

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

#AC_DEFINE([TEST_AC_DEFINE], ["<boost/foreach.hpp>"], [testing of ac_define])
	     

AC_OUTPUT
