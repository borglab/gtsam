dnl
dnl Copyright (C) 2009, Francesco P. Lovergine <frankie@debian.org>

AC_INIT([GeographicLib],[1.35],[charles@karney.com])
AC_CANONICAL_SYSTEM
AC_PREREQ(2.61)
AC_CONFIG_SRCDIR(src/Geodesic.cpp)
AC_CONFIG_MACRO_DIR(m4)
AM_INIT_AUTOMAKE

GEOGRAPHICLIB_VERSION_MAJOR=1
GEOGRAPHICLIB_VERSION_MINOR=35
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
LT_CURRENT=11
LT_REVISION=2
LT_AGE=1
AC_SUBST(LT_CURRENT)
AC_SUBST(LT_REVISION)
AC_SUBST(LT_AGE)

AC_ARG_PROGRAM
AC_PROG_CPP
AC_PROG_MAKE_SET
AC_PROG_INSTALL
AC_PROG_CXX
AC_PROG_LIBTOOL

# Checks for header files.
AC_CHECK_HEADERS([float.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_HEADER_STDBOOL
AC_C_INLINE
AC_TYPE_LONG_DOUBLE

# Checks for library functions.
AC_FUNC_STRTOD
AC_CHECK_FUNCS([strtol])
AC_SEARCH_LIBS([floor],[m])
AC_SEARCH_LIBS([pow],[m])
AC_SEARCH_LIBS([sqrt],[m])

# Check endianness
AC_C_BIGENDIAN

# Check for doxygen.
# Need version 1.8.1.2 or later for greek and math symbols.
AC_CHECK_PROGS([DOXYGEN], [doxygen])
AM_CONDITIONAL([HAVE_DOXYGEN],
	[test "$DOXYGEN" && test `"$DOXYGEN" --version |
	sed 's/^\([[0-9]]\)\.\([[0-9]]\)\./\1.0\2./'` '>' 1.08.1.1])

AC_CHECK_PROGS([POD2MAN], [pod2man])
AC_CHECK_PROGS([POD2HTML], [pod2html])
AC_CHECK_PROGS([COL], [col])
AM_CONDITIONAL([HAVE_PODPROGS], [test "$POD2MAN" -a  "$POD2HTML" -a "$COL"])

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
matlab/Makefile
python/Makefile
cmake/Makefile
examples/Makefile
])
AC_OUTPUT
