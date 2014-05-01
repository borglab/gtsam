#ifndef HEADER_commonlib
#define HEADER_commonlib

#include <stdlib.h>
#include <stdio.h>

static char SpaceChars[3] = {" " "\7"};
static char NumChars[14]  = {"0123456789-+."};

#define BIGNUMBER      1.0e+30
#define TINYNUMBER     1.0e-04
#define MACHINEPREC   2.22e-16
#define MATHPREC       1.0e-16
#define ERRLIMIT       1.0e-06

#ifndef LINEARSEARCH
  #define LINEARSEARCH 5
#endif

#if 0
  #define INTEGERTIME
#endif

/* ************************************************************************ */
/* Define loadable library function headers                                 */
/* ************************************************************************ */
#if (defined WIN32) || (defined WIN64)
  #define my_LoadLibrary(name)              LoadLibrary(name)
  #define my_GetProcAddress(handle, name)   GetProcAddress(handle, name)
  #define my_FreeLibrary(handle)            FreeLibrary(handle); \
                                            handle = NULL
#else
  #define my_LoadLibrary(name)              dlopen(name, RTLD_LAZY)
  #define my_GetProcAddress(handle, name)   dlsym(handle, name)
  #define my_FreeLibrary(handle)            dlclose(handle); \
                                            handle = NULL
#endif


/* ************************************************************************ */
/* Define sizes of standard number types                                    */
/* ************************************************************************ */
#ifndef LLONG
  #if defined __BORLANDC__
    #define LLONG __int64
  #elif !defined _MSC_VER || _MSC_VER >= 1310
    #define LLONG long long
  #else
    #define LLONG __int64
  #endif
#endif

#ifndef MYBOOL
  #if 0
    #define MYBOOL unsigned int
  #else
    #define MYBOOL unsigned char
  #endif
#endif

#ifndef REAL
  #define REAL     double
#endif
#ifndef BLAS_prec
  #define BLAS_prec "d" /* The BLAS precision prefix must correspond to the REAL type */
#endif

#ifndef REALXP
  #if 1
    #define REALXP long double  /* Set local accumulation variable as long double */
  #else
    #define REALXP REAL          /* Set local accumulation as default precision */
  #endif
#endif

#ifndef my_boolstr
  #define my_boolstr(x)          (!(x) ? "FALSE" : "TRUE")
#endif

#ifndef NULL
  #define NULL 	       0
#endif

#ifndef FALSE
  #define FALSE        0
  #define TRUE         1
#endif

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

#ifndef DOFASTMATH
  #define DOFASTMATH
#endif


#ifndef CALLOC
#define CALLOC(ptr, nr)\
  if(!((void *) ptr = calloc((size_t)(nr), sizeof(*ptr))) && nr) {\
    printf("calloc of %d bytes failed on line %d of file %s\n",\
           (size_t) nr * sizeof(*ptr), __LINE__, __FILE__);\
  }
#endif

#ifndef MALLOC
#define MALLOC(ptr, nr)\
  if(!((void *) ptr = malloc((size_t)((size_t) (nr) * sizeof(*ptr)))) && nr) {\
    printf("malloc of %d bytes failed on line %d of file %s\n",\
           (size_t) nr * sizeof(*ptr), __LINE__, __FILE__);\
  }
#endif

#ifndef REALLOC
#define REALLOC(ptr, nr)\
  if(!((void *) ptr = realloc(ptr, (size_t)((size_t) (nr) * sizeof(*ptr)))) && nr) {\
    printf("realloc of %d bytes failed on line %d of file %s\n",\
           (size_t) nr * sizeof(*ptr), __LINE__, __FILE__);\
  }
#endif

#ifndef FREE
#define FREE(ptr)\
  if((void *) ptr != NULL) {\
    free(ptr);\
    ptr = NULL; \
  }
#endif

#ifndef MEMCOPY
#define MEMCOPY(nptr, optr, nr)\
  memcpy((nptr), (optr), (size_t)((size_t)(nr) * sizeof(*(optr))))
#endif

#ifndef MEMMOVE
#define MEMMOVE(nptr, optr, nr)\
  memmove((nptr), (optr), (size_t)((size_t)(nr) * sizeof(*(optr))))
#endif

#ifndef MEMALLOCCOPY
#define MEMALLOCCOPY(nptr, optr, nr)\
  {MALLOC(nptr, (size_t)(nr));\
   MEMCOPY(nptr, optr, (size_t)(nr));}
#endif

#ifndef STRALLOCCOPY
#define STRALLOCCOPY(nstr, ostr)\
  {nstr = (char *) malloc((size_t) (strlen(ostr) + 1));\
   strcpy(nstr, ostr);}
#endif

#ifndef MEMCLEAR
/*#define useMMX*/
#ifdef useMMX
  #define MEMCLEAR(ptr, nr)\
    mem_set((ptr), '\0', (size_t)((size_t)(nr) * sizeof(*(ptr))))
#else
  #define MEMCLEAR(ptr, nr)\
    memset((ptr), '\0', (size_t)((size_t)(nr) * sizeof(*(ptr))))
#endif
#endif


#define MIN(x, y)         ((x) < (y) ? (x) : (y))
#define MAX(x, y)         ((x) > (y) ? (x) : (y))
#define SETMIN(x, y)      if((x) > (y)) x = y
#define SETMAX(x, y)      if((x) < (y)) x = y
#define LIMIT(lo, x, hi)  ((x < (lo) ? lo : ((x) > hi ? hi : x)))
#define BETWEEN(x, a, b)  (MYBOOL) (((x)-(a)) * ((x)-(b)) <= 0)
#define IF(t, x, y)       ((t) ? (x) : (y))
#define SIGN(x)           ((x) < 0 ? -1 : 1)

#define DELTA_SIZE(newSize, oldSize) ((int) ((newSize) * MIN(1.33, pow(1.5, fabs((double)newSize)/((oldSize+newSize)+1)))))

#ifndef CMP_CALLMODEL
#if (defined WIN32) || (defined WIN64)
  #define CMP_CALLMODEL _cdecl
#else
  #define CMP_CALLMODEL
#endif
#endif

typedef int (CMP_CALLMODEL findCompare_func)(const void *current, const void *candidate);
#define CMP_COMPARE(current, candidate) ( current < candidate ? -1 : (current > candidate ? 1 : 0) )
#define CMP_ATTRIBUTES(item)            (((char *) attributes)+(item)*recsize)
#define CMP_TAGS(item)                  (((char *) tags)+(item)*tagsize)

#ifndef UNIONTYPE
  #ifdef __cplusplus
    #define UNIONTYPE
  #else
    #define UNIONTYPE union
  #endif
#endif

/* This defines a 16 byte sort record (in both 32 and 64 bit OS-es) */
typedef struct _QSORTrec1
{
  void     *ptr;
  void     *ptr2;
} QSORTrec1;
typedef struct _QSORTrec2
{
  void     *ptr;
  double   realval;
} QSORTrec2;
typedef struct _QSORTrec3
{
  void     *ptr;
  int      intval;
  int      intpar1;
} QSORTrec3;
typedef struct _QSORTrec4
{
  REAL     realval;
  int      intval;
  int      intpar1;
} QSORTrec4;
typedef struct _QSORTrec5
{
  double   realval;
  long int longval;
} QSORTrec5;
typedef struct _QSORTrec6
{
  double   realval;
  double   realpar1;
} QSORTrec6;
typedef struct _QSORTrec7
{
  int      intval;
  int      intpar1;
  int      intpar2;
  int      intpar3;
} QSORTrec7;
union QSORTrec
{
  QSORTrec1 pvoid2;
  QSORTrec2 pvoidreal;
  QSORTrec3 pvoidint2;
  QSORTrec4 realint2;
  QSORTrec5 reallong;
  QSORTrec6 real2;
  QSORTrec7 int4;
};


#ifdef __cplusplus
  extern "C" {
#endif

int intpow(int base, int exponent);
int mod(int n, int d);

void strtoup(char *s);
void strtolo(char *s);
void strcpyup(char *t, char *s);
void strcpylo(char *t, char *s);

MYBOOL so_stdname(char *stdname, char *descname, int buflen);
int gcd(LLONG a, LLONG b, int *c, int *d);

int findIndex(int target, int *attributes, int count, int offset);
int findIndexEx(void *target, void *attributes, int count, int offset, int recsize, findCompare_func findCompare, MYBOOL ascending);

void qsortex_swap(void *attributes, int l, int r, int recsize,
                         void *tags, int tagsize, char *save, char *savetag);

int qsortex(void *attributes, int count, int offset, int recsize, MYBOOL descending, findCompare_func findCompare, void *tags, int tagsize);

int CMP_CALLMODEL compareCHAR(const void *current, const void *candidate);
int CMP_CALLMODEL compareINT(const void *current, const void *candidate);
int CMP_CALLMODEL compareREAL(const void *current, const void *candidate);
void hpsort(void *attributes, int count, int offset, int recsize, MYBOOL descending, findCompare_func findCompare);
void hpsortex(void *attributes, int count, int offset, int recsize, MYBOOL descending, findCompare_func findCompare, int *tags);

void QS_swap(UNIONTYPE QSORTrec a[], int i, int j);
int QS_addfirst(UNIONTYPE QSORTrec a[], void *mydata);
int QS_append(UNIONTYPE QSORTrec a[], int ipos, void *mydata);
void QS_replace(UNIONTYPE QSORTrec a[], int ipos, void *mydata);
void QS_insert(UNIONTYPE QSORTrec a[], int ipos, void *mydata, int epos);
void QS_delete(UNIONTYPE QSORTrec a[], int ipos, int epos);
MYBOOL QS_execute(UNIONTYPE QSORTrec a[], int count, findCompare_func findCompare, int *nswaps);

int sortByREAL(int *item, REAL *weight, int size, int offset, MYBOOL unique);
int sortByINT(int *item, int *weight, int size, int offset, MYBOOL unique);
REAL sortREALByINT(REAL *item, int *weight, int size, int offset, MYBOOL unique);

double timeNow(void);

void blockWriteBOOL(FILE *output, char *label, MYBOOL *myvector, int first, int last, MYBOOL asRaw);
void blockWriteINT(FILE *output, char *label, int *myvector, int first, int last);
void blockWriteREAL(FILE *output, char *label, REAL *myvector, int first, int last);

void printvec( int n, REAL *x, int modulo );
void printmatSQ( int size, int n, REAL *X, int modulo );
void printmatUT( int size, int n, REAL *U, int modulo );

unsigned int catchFPU(unsigned int mask);

#if defined _MSC_VER
int fileCount( char *filemask );
MYBOOL fileSearchPath( char *envvar, char *searchfile, char *foundpath );
#endif

#ifdef __cplusplus
  }
#endif

#endif /* HEADER_commonlib */
