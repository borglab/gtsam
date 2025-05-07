dnl
dnl Copyright (C) 2009, Francesco P. Lovergine <frankie@debian.org>

AC_INIT([GeographicLib],[2.5],[karney@alum.mit.edu])
AC_CANONICAL_TARGET
AC_PREREQ([2.71])
AC_CONFIG_SRCDIR(src/Geodesic.cpp)
AC_CONFIG_MACRO_DIR(m4)
AM_INIT_AUTOMAKE

GEOGRAPHICLIB_VERSION_MAJOR=2
GEOGRAPHICLIB_VERSION_MINOR=5
GEOGRAPHICLIB_VERSION_PATCH=0
AC_DEFINE_UNQUOTED([GEOGRAPHICLIB_VERSION_MAJOR],
        [$GEOGRAPHICLIB_VERSION_MAJOR],[major version number])
AC_DEFINE_UNQUOTED([GEOGRAPHICLIB_VERSION_MINOR],
        [$GEOGRAPHICLIB_VERSION_MINOR],[minor version number])
AC_DEFINE_UNQUOTED([GEOGRAPHICLIB_VERSION_PATCH],
        [$GEOGRAPHICLIB_VERSION_PATCH],[patch number])
AC_SUBST(GEOGRAPHICLIB_VERSION_MAJOR)
AC_SUBST(GEOGRAPHICLIB_VERSION_MINOR)
AC_SUBST(GEOGRAPHICLIB_VERSION_PATCH)

dnl
dnl This directive is deprecated by someone, but I prefer to avoid
dnl running autotools if not required explicitly. The reason is
dnl the need to be in sync with autoconf/automake.
dnl
AM_MAINTAINER_MODE

AC_CONFIG_HEADERS(include/GeographicLib/Config-ac.h)

dnl Library code modified:              REVISION++
dnl Interfaces changed/added/removed:   CURRENT++ REVISION=0
dnl Interfaces added:                   AGE++
dnl Interfaces removed:                 AGE=0
LT_CURRENT=27
LT_REVISION=1
LT_AGE=1
AC_SUBST(LT_CURRENT)
AC_SUBST(LT_REVISION)
AC_SUBST(LT_AGE)

AC_ARG_PROGRAM
AC_PROG_CPP
AC_PROG_MAKE_SET
AC_PROG_INSTALL
AC_PROG_CXX
AX_CXX_COMPILE_STDCXX_11([noext],[mandatory])
LT_INIT
AC_LANG([C++])

# Checks for long double
AC_TYPE_LONG_DOUBLE

# Check endianness
AC_C_BIGENDIAN

# Check flag for accurate arithmetic with Intel compiler.  This is
# needed to stop the compiler from ignoring parentheses in expressions
# like (a + b) + c and from simplifying 0.0 + x to x (which is wrong if
# x = -0.0).
AX_CHECK_COMPILE_FLAG([-fp-model precise -diag-disable=11074,11076],
        [CXXFLAGS="$CXXFLAGS -fp-model precise -diag-disable=11074,11076"],,
        [-Werror])

# Check for doxygen.  Version 1.8.7 or later needed for &hellip;
AC_CHECK_PROGS([DOXYGEN], [doxygen])
AM_CONDITIONAL([HAVE_DOXYGEN],
        [test "$DOXYGEN" && test `"$DOXYGEN" --version |
         sed 's/\b\([[0-9]]\)\b/0\1/g'` '>' 01.08.06.99])

dnl
dnl Add here new file to be generated
dnl
AC_CONFIG_FILES([
Makefile
src/Makefile
include/Makefile
tools/Makefile
doc/Makefile
man/Makefile
cmake/Makefile
examples/Makefile
tests/Makefile
experimental/Makefile
])

PKG_PROG_PKG_CONFIG
PKG_INSTALLDIR

AC_SUBST(lib_postfix,"")
AC_CONFIG_FILES([cmake/geographiclib.pc:cmake/project.pc.in])

AC_OUTPUT
