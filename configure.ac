#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.59)
AC_INIT(gtsam, 0.9.2, dellaert@cc.gatech.edu)
AM_INIT_AUTOMAKE(gtsam, 0.9.2)
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
AC_CONFIG_SRCDIR([gtsam/3rdparty/ccolamd/Makefile.am])

# pull in all subdirectories of Eigen - FIXME: find a better way of handling this
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Cholesky/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/arch/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/arch/AltiVec/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/arch/Default/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/arch/NEON/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/arch/SSE/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/products/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Core/util/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Eigen2Support/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Eigen2Support/Geometry/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Eigenvalues/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Geometry/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Geometry/arch/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Householder/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/QR/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/SVD/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Jacobi/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/misc/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/Sparse/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/LU/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/LU/arch/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/plugins/Makefile.am])
AC_CONFIG_SRCDIR([gtsam/3rdparty/Eigen/src/StlSupport/Makefile.am])

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

# ask for boost directory
#AC_ARG_WITH([boost],
#            [AS_HELP_STRING([--with-boost],
#              [specify the boost directory for installation (mandatory)])],
#            [boost=$withval],
#            [AC_MSG_FAILURE(
#                   [--with-boost has to be specified])
#            ])
#AC_SUBST([boost])

AC_CONFIG_FILES([CppUnitLite/Makefile \
gtsam/3rdparty/Eigen/src/Cholesky/Makefile \
gtsam/3rdparty/Eigen/src/Core/Makefile \
gtsam/3rdparty/Eigen/src/Core/arch/Makefile \
gtsam/3rdparty/Eigen/src/Core/arch/AltiVec/Makefile \
gtsam/3rdparty/Eigen/src/Core/arch/Default/Makefile \
gtsam/3rdparty/Eigen/src/Core/arch/NEON/Makefile \
gtsam/3rdparty/Eigen/src/Core/arch/SSE/Makefile \
gtsam/3rdparty/Eigen/src/Core/products/Makefile \
gtsam/3rdparty/Eigen/src/Core/util/Makefile \
gtsam/3rdparty/Eigen/src/Eigenvalues/Makefile \
gtsam/3rdparty/Eigen/src/Geometry/Makefile \
gtsam/3rdparty/Eigen/src/Geometry/arch/Makefile \
gtsam/3rdparty/Eigen/src/Householder/Makefile \
gtsam/3rdparty/Eigen/src/QR/Makefile \
gtsam/3rdparty/Eigen/src/SVD/Makefile \
gtsam/3rdparty/Eigen/src/Jacobi/Makefile \
gtsam/3rdparty/Eigen/src/misc/Makefile \
gtsam/3rdparty/Eigen/src/Sparse/Makefile \
gtsam/3rdparty/Eigen/src/LU/Makefile \
gtsam/3rdparty/Eigen/src/LU/arch/Makefile \
gtsam/3rdparty/Eigen/src/plugins/Makefile \
gtsam/3rdparty/Eigen/src/StlSupport/Makefile \
gtsam/3rdparty/Eigen/src/Eigen2Support/Makefile \
gtsam/3rdparty/Eigen/src/Eigen2Support/Geometry/Makefile \
gtsam/3rdparty/Eigen/src/Makefile \
gtsam/3rdparty/Eigen/Makefile \
gtsam/3rdparty/ccolamd/Makefile \
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
