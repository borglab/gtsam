#ifndef HEADER_lp_types
#define HEADER_lp_types

/* Define data types                                                         */
/* ------------------------------------------------------------------------- */
#ifndef LLONG
  #if defined __BORLANDC__
    #define LLONG __int64
  #elif !defined _MSC_VER || _MSC_VER >= 1310
    #define LLONG long long
  #else
    #define LLONG __int64
  #endif
#endif

#ifndef COUNTER
  #define COUNTER LLONG
#endif

#ifndef REAL
  #define REAL    double
#endif

#ifndef REALXP
  #if 1
    #define REALXP long double  /* Set local accumulation variable as long double */
  #else
    #define REALXP REAL          /* Set local accumulation as default precision */
  #endif
#endif

#ifndef LREAL
  #if 0
    #define LREAL long double   /* Set global solution update variable as long double */
  #else
    #define LREAL REAL           /* Set global solution update variable as default precision */
  #endif
#endif

#define RESULTVALUEMASK "%18.12g" /* Set fixed-format real-valued output precision;
                                  suggested width: ABS(exponent of DEF_EPSVALUE)+6. */
#define INDEXVALUEMASK  "%8d"     /* Set fixed-format integer-valued output width */

#ifndef DEF_STRBUFSIZE
  #define DEF_STRBUFSIZE   512
#endif
#ifndef MAXINT32
  #define MAXINT32  2147483647
#endif
#ifndef MAXUINT32
  #define MAXUINT32 4294967295
#endif

#ifndef MAXINT64
  #if defined _LONGLONG || defined __LONG_LONG_MAX__ || defined LLONG_MAX
    #define MAXINT64   9223372036854775807ll
  #else
    #define MAXINT64   9223372036854775807l
  #endif
#endif
#ifndef MAXUINT64
  #if defined _LONGLONG || defined __LONG_LONG_MAX__ || defined LLONG_MAX
    #define MAXUINT64 18446744073709551616ll
  #else
    #define MAXUINT64 18446744073709551616l
  #endif
#endif

#ifndef CHAR_BIT
  #define CHAR_BIT  8
#endif
#ifndef MYBOOL
  #define MYBOOL  unsigned char    /* Conserve memory, could be unsigned int */
#endif


/* Constants                                                                 */
/* ------------------------------------------------------------------------- */
#ifndef NULL
  #define NULL                   0
#endif

/* Byte-sized Booleans and extended options */
#define FALSE                    0
#define TRUE                     1
#define AUTOMATIC                2
#define DYNAMIC                  4

/* Sorting and comparison constants */
#define COMP_PREFERCANDIDATE     1
#define COMP_PREFERNONE          0
#define COMP_PREFERINCUMBENT    -1

/* Library load status values */
#define LIB_LOADED               0
#define LIB_NOTFOUND             1
#define LIB_NOINFO               2
#define LIB_NOFUNCTION           3
#define LIB_VERINVALID           4
#define LIB_STR_LOADED           "Successfully loaded"
#define LIB_STR_NOTFOUND         "File not found"
#define LIB_STR_NOINFO           "No version data"
#define LIB_STR_NOFUNCTION       "Missing function header"
#define LIB_STR_VERINVALID       "Incompatible version"
#define LIB_STR_MAXLEN           23


/* Compiler/target settings                                                  */
/* ------------------------------------------------------------------------- */
#if (defined _WIN32) || (defined WIN32) || (defined _WIN64) || (defined WIN64)
# define __WINAPI WINAPI
#else
# define __WINAPI
#endif

#if (defined _WIN32) || (defined WIN32) || (defined _WIN64) || (defined WIN64)
# define __VACALL __cdecl
#else
# define __VACALL
#endif

#ifndef __BORLANDC__

  #ifdef _USRDLL

    #if 1
      #define __EXPORT_TYPE __declspec(dllexport)
    #else
     /* Set up for the Microsoft compiler */
      #ifdef LP_SOLVE_EXPORTS
        #define __EXPORT_TYPE __declspec(dllexport)
      #else
        #define __EXPORT_TYPE __declspec(dllimport)
      #endif
    #endif

  #else

    #define __EXPORT_TYPE

  #endif

  #ifdef __cplusplus
    #define __EXTERN_C extern "C"
  #else
    #define __EXTERN_C
  #endif

#else  /* Otherwise set up for the Borland compiler */

  #ifdef __DLL__

    #define _USRDLL
    #define __EXTERN_C extern "C"

    #ifdef __READING_THE_DLL
      #define __EXPORT_TYPE __import
    #else
      #define __EXPORT_TYPE __export
    #endif

  #else

    #define __EXPORT_TYPE
    #define __EXTERN_C extern "C"

  #endif

#endif


#if 0
  #define STATIC static
#else
  #define STATIC
#endif

#if !defined INLINE
  #if defined __cplusplus
    #define INLINE inline
  #elif (defined _WIN32) || (defined WIN32) || (defined _WIN64) || (defined WIN64)
    #define INLINE __inline
  #else
    #define INLINE static
  #endif
#endif

/* Function macros                                                           */
/* ------------------------------------------------------------------------- */
#define my_limitrange(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#ifndef my_mod
  #define my_mod(n, m)          ((n) % (m))
#endif
#define my_if(t, x, y)          ((t) ? (x) : (y))
#define my_sign(x)              ((x) < 0 ? -1 : 1)
#if 0
  #define my_chsign(t, x)       ( ((t) && ((x) != 0)) ? -(x) : (x))
#else
  #define my_chsign(t, x)       ( (2*((t) == 0) - 1) * (x) )  /* "Pipelined" */
#endif
#define my_flipsign(x)          ( fabs((REAL) (x)) == 0 ? 0 : -(x) )
#define my_roundzero(val, eps)  if (fabs((REAL) (val)) < eps) val = 0
#define my_avoidtiny(val, eps)  (fabs((REAL) (val)) < eps ? 0 : val)

#if 1
  #define my_infinite(lp, val)  ( (MYBOOL) (fabs(val) >= lp->infinite) )
#else
  #define my_infinite(lp, val)  is_infinite(lp, val)
#endif
#define my_inflimit(lp, val)    ( my_infinite(lp, val) ? lp->infinite * my_sign(val) : (val) )
#if 0
  #define my_precision(val, eps) ((fabs((REAL) (val))) < (eps) ? 0 : (val))
#else
  #define my_precision(val, eps) restoreINT(val, eps)
#endif
#define my_reldiff(x, y)       (((x) - (y)) / (1.0 + fabs((REAL) (y))))
#define my_boundstr(x)         (fabs(x) < lp->infinite ? sprintf("%g",x) : ((x) < 0 ? "-Inf" : "Inf") )
#ifndef my_boolstr
  #define my_boolstr(x)          (!(x) ? "FALSE" : "TRUE")
#endif
#define my_basisstr(isbasic)     ((isbasic) ? "BASIC" : "NON-BASIC")
#define my_simplexstr(isdual)    ((isdual) ? "DUAL" : "PRIMAL")
#define my_plural_std(count)     (count == 1 ? "" : "s")
#define my_plural_y(count)       (count == 1 ? "y" : "ies")
#define my_lowbound(x)           ((FULLYBOUNDEDSIMPLEX) ? (x) : 0)


/* Bound macros usable for both the standard and fully bounded simplex       */
/* ------------------------------------------------------------------------- */
/*
#define my_lowbo(lp, varnr)      ( lp->isfullybounded ? lp->lowbo[varnr] : 0.0 )
#define my_upbo(lp, varnr)       ( lp->isfullybounded ? lp->upbo[varnr]  : lp->lowbo[varnr] + lp->upbo[varnr] )
#define my_rangebo(lp, varnr)    ( lp->isfullybounded ? lp->upbo[varnr] - lp->lowbo[varnr] : lp->upbo[varnr] )
*/
#define my_lowbo(lp, varnr)      ( 0.0 )
#define my_upbo(lp, varnr)       ( lp->lowbo[varnr] + lp->upbo[varnr] )
#define my_rangebo(lp, varnr)    ( lp->upbo[varnr] )

#define my_unbounded(lp, varnr)  ((lp->upbo[varnr] >= lp->infinite) && (lp->lowbo[varnr] <= -lp->infinite))
#define my_bounded(lp, varnr)    ((lp->upbo[varnr] < lp->infinite) && (lp->lowbo[varnr] > -lp->infinite))

/* Forward declarations                                                      */
/* ------------------------------------------------------------------------- */
typedef struct _lprec     lprec;
typedef struct _INVrec    INVrec;
union  QSORTrec;

#ifndef UNIONTYPE
  #ifdef __cplusplus
    #define UNIONTYPE
  #else
    #define UNIONTYPE union
  #endif
#endif

/* B4 factorization optimization data */
typedef struct _B4rec
{
  int  *B4_var;  /* Position of basic columns in the B4 basis */
  int  *var_B4;  /* Variable in the B4 basis */
  int  *B4_row;  /* B4 position of the i'th row */
  int  *row_B4;  /* Original position of the i'th row */
  REAL *wcol;
  int  *nzwcol;
} B4rec;

#define OBJ_STEPS   5
typedef struct _OBJmonrec {
  lprec  *lp;
  int    oldpivstrategy,
         oldpivrule, pivrule, ruleswitches,
         limitstall[2], limitruleswitches,
         idxstep[OBJ_STEPS], countstep, startstep, currentstep,
         Rcycle, Ccycle, Ncycle, Mcycle, Icount;
  REAL   thisobj, prevobj,
         objstep[OBJ_STEPS],
         thisinfeas, previnfeas,
         epsvalue;
  char   spxfunc[10];
  MYBOOL pivdynamic;
  MYBOOL isdual;
  MYBOOL active;
} OBJmonrec;

typedef struct _edgerec
{
  REAL      *edgeVector;
} edgerec;

typedef struct _pricerec
{
  REAL   theta;
  REAL   pivot;
  REAL   epspivot;
  int    varno;
  lprec  *lp;
  MYBOOL isdual;
} pricerec;
typedef struct _multirec
{
  lprec    *lp;
  int      size;                  /* The maximum number of multiply priced rows/columns */
  int      used;                  /* The current / active number of multiply priced rows/columns */
  int      limit;                 /* The active/used count at which a full update is triggered */
  pricerec *items;                /* Array of best multiply priced rows/columns */
  int      *freeList;             /* The indeces of available positions in "items" */
  UNIONTYPE QSORTrec *sortedList; /* List of pointers to "pricerec" items in sorted order */
  REAL     *stepList;             /* Working array (values in sortedList order) */
  REAL     *valueList;            /* Working array (values in sortedList order) */
  int      *indexSet;             /* The final exported index list of pivot variables */
  int      active;                /* Index of currently active multiply priced row/column */
  int      retries;
  REAL     step_base;
  REAL     step_last;
  REAL     obj_base;
  REAL     obj_last;
  REAL     epszero;
  REAL     maxpivot;
  REAL     maxbound;
  MYBOOL   sorted;
  MYBOOL   truncinf;
  MYBOOL   objcheck;
  MYBOOL   dirty;
} multirec;

#endif /* HEADER_lp_types */
