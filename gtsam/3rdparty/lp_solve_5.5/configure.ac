AC_PREREQ(2.52)
AC_INIT([lpsolve],5.5.0.11,[], [])
AC_CONFIG_SRCDIR(lp_simplex.c)
AC_CANONICAL_HOST

SHARED_LIB=liblpsolve55.so
SO=.so
case $host_os in
  hp*|HP*)
    DEF=-ldld
    case `uname -m` in
      ia64) SO=.so;;
      *)    SO=.sl;;
    esac
    if test "$GCC" = yes;
      then CCSHARED="-fPIC";
      else CCSHARED="+z";
    fi;;
  CYGWIN*)
    SO=.dll;;
  linux)
    CCSHARED="-fPIC";;
  apple-darwin)
    DEF=-ldl -idirafter /usr/include/sys -DINTEGERTIME -Wno-long-double
    ;;
  OpenUNIX*|UnixWare*)
    DEF='-dy -DNOLONGLONG'
    if test "$GCC" = "yes"
      then CCSHARED="-fPIC"
      else CCSHARED="-K PIC"
    fi;;
  SCO_SV*)
    DEF='-dy -DNOLONGLONG'
    if test "$GCC" = "yes"
      then CCSHARED="-fPIC"
      else CCSHARED="-Kpic -belf"
    fi;;
 *)
esac

AC_SUBST(SO)
AC_SUBST(CCSHARED)
AC_SUBST(DEF)
AC_SUBST(SHARED_LIB)

AC_PROG_CC
if test "x$GCC" != "xyes"
then
  echo "*** non GNU CC compiler detected."
  echo "*** This package has not been tested very well with non GNU compilers"
fi

AC_PROG_INSTALL
AC_CONFIG_FILES([ \
	Makefile \
])
AC_OUTPUT
