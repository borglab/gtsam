
/* ---------------------------------------------------------------------------------- */
/* lp_solve v5+ headers for basis inversion / factorization libraries                 */
/* ---------------------------------------------------------------------------------- */
#define BFP_STATUS_RANKLOSS     -1
#define BFP_STATUS_SUCCESS       0
#define BFP_STATUS_SINGULAR      1
#define BFP_STATUS_UNSTABLE      2
#define BFP_STATUS_NOPIVOT       3
#define BFP_STATUS_DIMERROR      4
#define BFP_STATUS_DUPLICATE     5
#define BFP_STATUS_NOMEMORY      6
#define BFP_STATUS_ERROR         7             /* Unspecified, command-related error */
#define BFP_STATUS_FATAL         8

#define BFP_STAT_ERROR          -1
#define BFP_STAT_REFACT_TOTAL    0
#define BFP_STAT_REFACT_TIMED    1
#define BFP_STAT_REFACT_DENSE    2

#ifndef BFP_CALLMODEL
  #ifdef WIN32
    #define BFP_CALLMODEL __stdcall   /* "Standard" call model */
  #else
    #define BFP_CALLMODEL
  #endif
#endif

#ifdef RoleIsExternalInvEngine
  #define __BFP_EXPORT_TYPE __EXPORT_TYPE
#else
  #define __BFP_EXPORT_TYPE
#endif


/* Routines with UNIQUE implementations for each inversion engine                     */
/* ---------------------------------------------------------------------------------- */
char   __BFP_EXPORT_TYPE *(BFP_CALLMODEL bfp_name)(void);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_free)(lprec *lp);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_resize)(lprec *lp, int newsize);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_nonzeros)(lprec *lp, MYBOOL maximum);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_memallocated)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_preparefactorization)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_factorize)(lprec *lp, int uservars, int Bsize, MYBOOL *usedpos, MYBOOL final);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_finishupdate)(lprec *lp, MYBOOL changesign);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_ftran_normal)(lprec *lp, REAL *pcol, int *nzidx);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_ftran_prepare)(lprec *lp, REAL *pcol, int *nzidx);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_btran_normal)(lprec *lp, REAL *prow, int *nzidx);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_status)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_findredundant)(lprec *lp, int items, getcolumnex_func cb, int *maprow, int*mapcol);


/* Routines SHARED for all inverse implementations; located in lp_BFP1.c              */
/* ---------------------------------------------------------------------------------- */
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_compatible)(lprec *lp, int bfpversion, int lpversion, int sizeofvar);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_indexbase)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_rowoffset)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_pivotmax)(lprec *lp);
REAL   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_efficiency)(lprec *lp);
REAL   __BFP_EXPORT_TYPE *(BFP_CALLMODEL bfp_pivotvector)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_pivotcount)(lprec *lp);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_mustrefactorize)(lprec *lp);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_refactcount)(lprec *lp, int kind);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_isSetI)(lprec *lp);
int    *bfp_createMDO(lprec *lp, MYBOOL *usedpos, int count, MYBOOL doMDO);
void   BFP_CALLMODEL bfp_updaterefactstats(lprec *lp);
int    BFP_CALLMODEL bfp_rowextra(lprec *lp);

/* Routines with OPTIONAL SHARED code; template routines suitable for canned          */
/* inverse engines are located in lp_BFP2.c                                           */
/* ---------------------------------------------------------------------------------- */
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_init)(lprec *lp, int size, int deltasize, char *options);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_restart)(lprec *lp);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_implicitslack)(lprec *lp);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_pivotalloc)(lprec *lp, int newsize);
int    __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_colcount)(lprec *lp);
MYBOOL __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_canresetbasis)(lprec *lp);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_finishfactorization)(lprec *lp);
LREAL  __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_prepareupdate)(lprec *lp, int row_nr, int col_nr, REAL *pcol);
REAL   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_pivotRHS)(lprec *lp, LREAL theta, REAL *pcol);
void   __BFP_EXPORT_TYPE (BFP_CALLMODEL bfp_btran_double)(lprec *lp, REAL *prow, int *pnzidx, REAL *drow, int *dnzidx);

