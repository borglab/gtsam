#ifndef HEADER_LUSOL
#define HEADER_LUSOL

/* Include necessary libraries                                               */
/* ------------------------------------------------------------------------- */
#include <stdio.h>
#include "commonlib.h"

/* Version information                                                       */
/* ------------------------------------------------------------------------- */
#define LUSOL_VERMAJOR   2
#define LUSOL_VERMINOR   2
#define LUSOL_RELEASE    2
#define LUSOL_BUILD      0

/* Dynamic memory management macros                                          */
/* ------------------------------------------------------------------------- */
#ifdef MATLAB
  #define LUSOL_MALLOC(bytesize)        mxMalloc(bytesize)
  #define LUSOL_CALLOC(count, recsize)  mxCalloc(count, recsize)
  #define LUSOL_REALLOC(ptr, bytesize)  mxRealloc((void *) ptr, bytesize)
  #define LUSOL_FREE(ptr)               {mxFree(ptr); ptr=NULL;}
#else
  #define LUSOL_MALLOC(bytesize)        malloc(bytesize)
  #define LUSOL_CALLOC(count, recsize)  calloc(count, recsize)
  #define LUSOL_REALLOC(ptr, bytesize)  realloc((void *) ptr, bytesize)
  #define LUSOL_FREE(ptr)               {free(ptr); ptr=NULL;}
#endif

/* Performance compiler options                                              */
/* ------------------------------------------------------------------------- */
#if 1
  #define ForceInitialization      /* Satisfy compilers, check during debugging! */
  #define LUSOLFastDenseIndex           /* Increment the linearized dense address */
  #define LUSOLFastClear           /* Use intrinsic functions for memory zeroing */
  #define LUSOLFastMove              /* Use intrinsic functions for memory moves */
  #define LUSOLFastCopy               /* Use intrinsic functions for memory copy */
  #define LUSOLFastSolve           /* Use pointer operations in equation solving */
  #define LUSOLSafeFastUpdate      /* Use separate array for LU6L result storage */
/*#define UseOld_LU6CHK_20040510 */
/*#define AlwaysSeparateHamaxR */       /* Enabled when the pivot model is fixed */
  #if 0
    #define ForceRowBasedL0                  /* Create a row-sorted version of L0 */
  #endif
/*  #define SetSmallToZero*/
/*  #define DoTraceL0 */
#endif
/*#define UseTimer */


/* Legacy compatibility and testing options (Fortran-LUSOL)                  */
/* ------------------------------------------------------------------------- */
#if 0
  #define LegacyTesting
  #define StaticMemAlloc           /* Preallocated vs. dynamic memory allocation */
  #define ClassicdiagU                                  /* Store diagU at end of a */
  #define ClassicHamaxR                    /* Store H+AmaxR at end of a/indc/indr */
#endif


/* General constants and data type definitions                               */
/* ------------------------------------------------------------------------- */
#define LUSOL_ARRAYOFFSET            1
#ifndef ZERO
  #define ZERO                       0
#endif
#ifndef ONE
  #define ONE                        1
#endif
#ifndef FALSE
  #define FALSE                      0
#endif
#ifndef TRUE
  #define TRUE                       1
#endif
#ifndef NULL
  #define NULL                       0
#endif
#ifndef REAL
  #define REAL double
#endif
#ifndef REALXP
  #define REALXP long double
#endif
#ifndef MYBOOL
  #define MYBOOL unsigned char
#endif


/* User-settable default parameter values                                    */
/* ------------------------------------------------------------------------- */
#define LUSOL_DEFAULT_GAMMA        2.0
#define LUSOL_SMALLNUM         1.0e-20  /* IAEE doubles have precision 2.22e-16 */
#define LUSOL_BIGNUM           1.0e+20
#define LUSOL_MINDELTA_FACTOR        4
#define LUSOL_MINDELTA_a         10000
#if 1
  #define LUSOL_MULT_nz_a            2  /* Suggested by Yin Zhang */
#else
  #define LUSOL_MULT_nz_a            5  /* Could consider 6 or 7 */
#endif
#define LUSOL_MINDELTA_rc         1000
#define LUSOL_DEFAULT_SMARTRATIO 0.667

/* Fixed system parameters (changeable only by developers)                   */
/* ------------------------------------------------------------------------- */

/* parmlu INPUT parameters: */
#define LUSOL_RP_SMARTRATIO          0
#define LUSOL_RP_FACTORMAX_Lij       1
#define LUSOL_RP_UPDATEMAX_Lij       2
#define LUSOL_RP_ZEROTOLERANCE       3
#define LUSOL_RP_SMALLDIAG_U         4
#define LUSOL_RP_EPSDIAG_U           5
#define LUSOL_RP_COMPSPACE_U         6
#define LUSOL_RP_MARKOWITZ_CONLY     7
#define LUSOL_RP_MARKOWITZ_DENSE     8
#define LUSOL_RP_GAMMA               9

/* parmlu OUPUT parameters: */
#define LUSOL_RP_MAXELEM_A          10
#define LUSOL_RP_MAXMULT_L          11
#define LUSOL_RP_MAXELEM_U          12
#define LUSOL_RP_MAXELEM_DIAGU      13
#define LUSOL_RP_MINELEM_DIAGU      14
#define LUSOL_RP_MAXELEM_TCP        15
#define LUSOL_RP_GROWTHRATE         16
#define LUSOL_RP_USERDATA_1         17
#define LUSOL_RP_USERDATA_2         18
#define LUSOL_RP_USERDATA_3         19
#define LUSOL_RP_RESIDUAL_U         20
#define LUSOL_RP_LASTITEM            LUSOL_RP_RESIDUAL_U

/* luparm INPUT parameters: */
#define LUSOL_IP_USERDATA_0          0
#define LUSOL_IP_PRINTUNIT           1
#define LUSOL_IP_PRINTLEVEL          2
#define LUSOL_IP_MARKOWITZ_MAXCOL    3
#define LUSOL_IP_SCALAR_NZA          4
#define LUSOL_IP_UPDATELIMIT         5
#define LUSOL_IP_PIVOTTYPE           6
#define LUSOL_IP_ACCELERATION        7
#define LUSOL_IP_KEEPLU              8
#define LUSOL_IP_SINGULARLISTSIZE    9

/* luparm OUTPUT parameters: */
#define LUSOL_IP_INFORM             10
#define LUSOL_IP_SINGULARITIES      11
#define LUSOL_IP_SINGULARINDEX      12
#define LUSOL_IP_MINIMUMLENA        13
#define LUSOL_IP_MAXLEN             14
#define LUSOL_IP_UPDATECOUNT        15
#define LUSOL_IP_RANK_U             16
#define LUSOL_IP_COLCOUNT_DENSE1    17
#define LUSOL_IP_COLCOUNT_DENSE2    18
#define LUSOL_IP_COLINDEX_DUMIN     19
#define LUSOL_IP_COLCOUNT_L0        20
#define LUSOL_IP_NONZEROS_L0        21
#define LUSOL_IP_NONZEROS_U0        22
#define LUSOL_IP_NONZEROS_L         23
#define LUSOL_IP_NONZEROS_U         24
#define LUSOL_IP_NONZEROS_ROW       25
#define LUSOL_IP_COMPRESSIONS_LU    26
#define LUSOL_IP_MARKOWITZ_MERIT    27
#define LUSOL_IP_TRIANGROWS_U       28
#define LUSOL_IP_TRIANGROWS_L       29
#define LUSOL_IP_FTRANCOUNT         30
#define LUSOL_IP_BTRANCOUNT         31
#define LUSOL_IP_ROWCOUNT_L0        32
#define LUSOL_IP_LASTITEM            LUSOL_IP_ROWCOUNT_L0


/* Macros for matrix-based access for dense part of A and timer mapping      */
/* ------------------------------------------------------------------------- */
#define DAPOS(row, col)   (row + (col-1)*LDA)
#define timer(text, id)   LUSOL_timer(LUSOL, id, text)


/* Parameter/option defines                                                  */
/* ------------------------------------------------------------------------- */
#define LUSOL_MSG_NONE              -1
#define LUSOL_MSG_SINGULARITY        0
#define LUSOL_MSG_STATISTICS        10
#define LUSOL_MSG_PIVOT             50

#define LUSOL_BASEORDER              0
#define LUSOL_OTHERORDER             1
#define LUSOL_AUTOORDER              2
#define LUSOL_ACCELERATE_L0          4
#define LUSOL_ACCELERATE_U           8

#define LUSOL_PIVMOD_NOCHANGE       -2  /* Don't change active pivoting model */
#define LUSOL_PIVMOD_DEFAULT        -1  /* Set pivoting model to default */
#define LUSOL_PIVMOD_TPP             0  /* Threshold Partial   pivoting (normal) */
#define LUSOL_PIVMOD_TRP             1  /* Threshold Rook      pivoting */
#define LUSOL_PIVMOD_TCP             2  /* Threshold Complete  pivoting */
#define LUSOL_PIVMOD_TSP             3  /* Threshold Symmetric pivoting */
#define LUSOL_PIVMOD_MAX             LUSOL_PIVMOD_TSP

#define LUSOL_PIVTOL_NOCHANGE        0
#define LUSOL_PIVTOL_BAGGY           1
#define LUSOL_PIVTOL_LOOSE           2
#define LUSOL_PIVTOL_NORMAL          3
#define LUSOL_PIVTOL_SLIM            4
#define LUSOL_PIVTOL_TIGHT           5
#define LUSOL_PIVTOL_SUPER           6
#define LUSOL_PIVTOL_CORSET          7
#define LUSOL_PIVTOL_DEFAULT         LUSOL_PIVTOL_SLIM
#define LUSOL_PIVTOL_MAX             LUSOL_PIVTOL_CORSET

#define LUSOL_UPDATE_OLDEMPTY        0  /* No/empty current column. */
#define LUSOL_UPDATE_OLDNONEMPTY     1  /* Current column need not have been empty. */
#define LUSOL_UPDATE_NEWEMPTY        0  /* New column is taken to be zero. */
#define LUSOL_UPDATE_NEWNONEMPTY     1  /* v(*) contains the new column;
                                           on exit,  v(*)  satisfies  L*v = a(new). */
#define LUSOL_UPDATE_USEPREPARED     2  /* v(*)  must satisfy  L*v = a(new). */

#define LUSOL_SOLVE_Lv_v             1  /* v  solves   L v = v(input). w  is not touched. */
#define LUSOL_SOLVE_Ltv_v            2  /* v  solves   L'v = v(input). w  is not touched. */
#define LUSOL_SOLVE_Uw_v             3  /* w  solves   U w = v.        v  is not altered. */
#define LUSOL_SOLVE_Utv_w            4  /* v  solves   U'v = w.        w  is destroyed. */
#define LUSOL_SOLVE_Aw_v             5  /* w  solves   A w = v.        v  is altered as in 1. */
#define LUSOL_FTRAN   LUSOL_SOLVE_Aw_v
#define LUSOL_SOLVE_Atv_w            6  /* v  solves   A'v = w.        w  is destroyed. */
#define LUSOL_BTRAN  LUSOL_SOLVE_Atv_w

/* If mode = 3,4,5,6, v and w must not be the same arrays.
   If lu1fac has just been used to factorize a symmetric matrix A
   (which must be definite or quasi-definite), the factors A = L U
   may be regarded as A = LDL', where D = diag(U).  In such cases,
   the following (faster) solve codes may be used:                  */
#define LUSOL_SOLVE_Av_v             7  /* v  solves   A v = L D L'v = v(input). w  is not touched. */
#define LUSOL_SOLVE_LDLtv_v          8  /* v  solves       L |D| L'v = v(input). w  is not touched. */

#define LUSOL_INFORM_RANKLOSS       -1
#define LUSOL_INFORM_LUSUCCESS       0
#define LUSOL_INFORM_LUSINGULAR      1
#define LUSOL_INFORM_LUUNSTABLE      2
#define LUSOL_INFORM_ADIMERR         3
#define LUSOL_INFORM_ADUPLICATE      4
#define LUSOL_INFORM_ANEEDMEM        7  /* Set lena >= luparm[LUSOL_IP_MINIMUMLENA] */
#define LUSOL_INFORM_FATALERR        8
#define LUSOL_INFORM_NOPIVOT         9  /* No diagonal pivot found with TSP or TDP. */
#define LUSOL_INFORM_NOMEMLEFT      10

#define LUSOL_INFORM_MIN             LUSOL_INFORM_RANKLOSS
#define LUSOL_INFORM_MAX             LUSOL_INFORM_NOMEMLEFT

#define LUSOL_INFORM_GETLAST        10  /* Code for LUSOL_informstr. */
#define LUSOL_INFORM_SERIOUS         LUSOL_INFORM_LUUNSTABLE


/* Prototypes for call-back functions                                        */
/* ------------------------------------------------------------------------- */
typedef void LUSOLlogfunc(void *lp, void *userhandle, char *buf);


/* Sparse matrix data */
typedef struct _LUSOLmat {
  REAL *a;
  int  *lenx, *indr, *indc, *indx;
} LUSOLmat;


/* The main LUSOL data record */
/* ------------------------------------------------------------------------- */
typedef struct _LUSOLrec {

  /* General data */
  FILE         *outstream;           /* Output stream, initialized to STDOUT */
  LUSOLlogfunc *writelog;
    void       *loghandle;
  LUSOLlogfunc *debuginfo;

  /* Parameter storage arrays */
  int    luparm[LUSOL_IP_LASTITEM + 1];
  REAL   parmlu[LUSOL_RP_LASTITEM + 1];

  /* Arrays of length lena+1 */
  int    lena, nelem;
  int    *indc, *indr;
  REAL   *a;

  /* Arrays of length maxm+1 (row storage) */
  int    maxm, m;
  int    *lenr, *ip, *iqloc, *ipinv, *locr;

  /* Arrays of length maxn+1 (column storage) */
  int    maxn, n;
  int    *lenc, *iq, *iploc, *iqinv, *locc;
  REAL   *w, *vLU6L;

  /* List of singular columns, with dynamic size allocation */
  int    *isingular;

  /* Extra arrays of length n for TCP and keepLU == FALSE */
  REAL   *Ha, *diagU;
  int    *Hj, *Hk;

  /* Extra arrays of length m for TRP*/
  REAL   *amaxr;

  /* Extra array for L0 and U stored by row/column for faster btran/ftran */
  LUSOLmat *L0;
  LUSOLmat *U;

  /* Miscellaneous data */
  int    expanded_a;
  int    replaced_c;
  int    replaced_r;

} LUSOLrec;


LUSOLrec *LUSOL_create(FILE *outstream, int msgfil, int pivotmodel, int updatelimit);
MYBOOL LUSOL_sizeto(LUSOLrec *LUSOL, int init_r, int init_c, int init_a);
MYBOOL LUSOL_assign(LUSOLrec *LUSOL, int iA[], int jA[], REAL Aij[],
                                     int nzcount, MYBOOL istriplet);
void LUSOL_clear(LUSOLrec *LUSOL, MYBOOL nzonly);
void LUSOL_free(LUSOLrec *LUSOL);

LUSOLmat *LUSOL_matcreate(int dim, int nz);
void LUSOL_matfree(LUSOLmat **mat);

int LUSOL_loadColumn(LUSOLrec *LUSOL, int iA[], int jA, REAL Aij[], int nzcount, int offset1);
void LUSOL_setpivotmodel(LUSOLrec *LUSOL, int pivotmodel, int initlevel);
int LUSOL_factorize(LUSOLrec *LUSOL);
int LUSOL_replaceColumn(LUSOLrec *LUSOL, int jcol, REAL v[]);

MYBOOL LUSOL_tightenpivot(LUSOLrec *LUSOL);
MYBOOL LUSOL_addSingularity(LUSOLrec *LUSOL, int singcol, int *inform);
int LUSOL_getSingularity(LUSOLrec *LUSOL, int singitem);
int LUSOL_findSingularityPosition(LUSOLrec *LUSOL, int singcol);

char *LUSOL_pivotLabel(LUSOLrec *LUSOL);
char *LUSOL_informstr(LUSOLrec *LUSOL, int inform);
REAL LUSOL_vecdensity(LUSOLrec *LUSOL, REAL V[]);
void LUSOL_report(LUSOLrec *LUSOL, int msglevel, char *format, ...);
void LUSOL_timer(LUSOLrec *LUSOL, int timerid, char *text);

int LUSOL_ftran(LUSOLrec *LUSOL, REAL b[], int NZidx[], MYBOOL prepareupdate);
int LUSOL_btran(LUSOLrec *LUSOL, REAL b[], int NZidx[]);

void LU1FAC(LUSOLrec *LUSOL, int *INFORM);
MYBOOL LU1L0(LUSOLrec *LUSOL, LUSOLmat **mat, int *inform);
void LU6SOL(LUSOLrec *LUSOL, int MODE, REAL V[], REAL W[], int NZidx[], int *INFORM);
void LU8RPC(LUSOLrec *LUSOL, int MODE1, int MODE2,
            int JREP, REAL V[], REAL W[],
            int *INFORM, REAL *DIAG, REAL *VNORM);

void LUSOL_dump(FILE *output, LUSOLrec *LUSOL);


void print_L0(LUSOLrec *LUSOL);


#endif /* HEADER_LUSOL */
