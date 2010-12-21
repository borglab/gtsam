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
AC_CONFIG_SRCDIR([tests/testSQP.cpp])
AC_CONFIG_SRCDIR([examples/SimpleRotation.cpp])

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

# enable BLAS with general purpose script
AC_ARG_ENABLE([blas],
     [  --enable-blas    Enable external BLAS library],
     [case "${enableval}" in
       yes) blas=true ;;
       no)  blas=false ;;
       *) AC_MSG_ERROR([bad value ${enableval} for --enable-blas]) ;;
     esac],[blas=true])
ak
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
     esac],[lapack=true])

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

# ask for ccolamd library include directory
AC_ARG_WITH([ccolamd-inc],
            [AS_HELP_STRING([--with-ccolamd-inc],
              [specify the CCOLAMD library include directory (defaults to /HOME/include/ccolamd)])],
            [CCOLAMDInc=$withval],
            [CCOLAMDInc=${HOME}/include/ccolamd])
AC_SUBST([CCOLAMDInc])

# ask for sparse library lib directory
AC_ARG_WITH([ccolamd-lib],
            [AS_HELP_STRING([--with-ccolamd-lib],
              [specify the CCOLAMD library lib directory (defaults to /HOME/lib)])],
            [CCOLAMDLib=$withval],
            [CCOLAMDLib=${HOME}/lib])
AC_SUBST([CCOLAMDLib])

# For now we require blas, atlas, and lapack
#AM_COND_IF([test x$ISMAC = xtrue],
#    [LINALG_CPPFLAGS="-I/System/Library/Frameworks/vecLib.framework/Headers ${CCOLAMDInc} -DGT_USE_LAPACK"],
#    [LINALG_CPPFLAGS="${CCOLAMDInc} -DGT_USE_LAPACK"])
#AM_COND_IF([test x$ISMAC = xtrue],
#    [LINALG_LDFLAGS="-Wl,/System/Library/Frameworks/Accelerate.framework/Accelerate ${CCOLAMDLib}"],
#    [LINALG_LDFLAGS="-lcblas -latlas -llapack ${CCOLAMDLib}"])

AC_CONFIG_FILES([CppUnitLite/Makefile gtsam/base/Makefile gtsam/geometry/Makefile gtsam/inference/Makefile gtsam/linear/Makefile gtsam/nonlinear/Makefile gtsam/slam/Makefile gtsam/Makefile tests/Makefile examples/Makefile examples/vSLAMexample/Makefile Makefile])
AC_OUTPUT
