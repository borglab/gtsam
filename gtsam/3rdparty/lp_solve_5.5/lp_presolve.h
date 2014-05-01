#ifndef HEADER_lp_presolve
#define HEADER_lp_presolve

#include "lp_types.h"
#include "lp_matrix.h"

/* -------------------------------------------------------------------------------------------- */
/* Defines for various presolve options                                                         */
/* -------------------------------------------------------------------------------------------- */

#define MAX_PSMERGELOOPS                2                 /* Max loops to merge compatible constraints */
#define MAX_PSLINDEPLOOPS               1   /* Max loops to detect linearly dependendent constraints */
#define MAX_PSBOUNDTIGHTENLOOPS         5     /* Maximumn number of loops to allow bound tightenings */
#define MIN_SOS1LENGTH                  4   /* Minimum length of a constraint for conversion to SOS1 */
#if 1
  #define PRESOLVE_EPSVALUE (0.1*lp->epsprimal)
#else
  #define PRESOLVE_EPSVALUE  lp->epsvalue
#endif
#define PRESOLVE_EPSPIVOT         1.0e-3        /* Looses robustness at values smaller than ~1.0e-3 */
#define PRESOLVE_BOUNDSLACK           10                     /* Extra error recovery/tolerance margin */

#define DoPresolveRounding              /* Use absolute and directed rounding (disable at own risk) */
/*#define DoPresolveRelativeTest*/

/*#define PresolveForceUpdateMax*/

/*#define DualFeasibilityLogicEQ2*/              /* Add low-order feasibility/accuracy logic to elimEQ2 */
#define DivisorIntegralityLogicEQ2                                   /* Always prefer integer divisors */
#define FindImpliedEqualities                               /* Detect equalities (default is enabled) */
#define Eq2Reldiff

/*#define SavePresolveEliminated */        /* Enable to activate storage of eliminated matrix data */
/*#define UseDualPresolve */                    /* Enable to use full dual information for presolve */


typedef struct _psrec
{
  LLrec *varmap;
  int  **next;
  int  *empty;
  int  *plucount;
  int  *negcount;
  int  *pluneg;
  int  *infcount;
  REAL  *plulower;
  REAL  *neglower;
  REAL  *pluupper;
  REAL  *negupper;
  int  allocsize;
} psrec;

typedef struct _presolverec
{
  psrec *rows;
  psrec *cols;
  LLrec *EQmap;
  LLrec *LTmap;
  LLrec *INTmap;
  REAL  *pv_upbo;
  REAL  *pv_lobo;
  REAL  *dv_upbo;
  REAL  *dv_lobo;
  lprec *lp;
  REAL  epsvalue;
  REAL  epspivot;
  int   innerloops;
  int   middleloops;
  int   outerloops;
  int   nzdeleted;
  MYBOOL forceupdate;
} presolverec;

#ifdef __cplusplus
extern "C" {
#endif

/* Put function headers here */

STATIC MYBOOL presolve_createUndo(lprec *lp);
STATIC MYBOOL presolve_rebuildUndo(lprec *lp, MYBOOL isprimal);
STATIC MYBOOL inc_presolve_space(lprec *lp, int delta, MYBOOL isrows);
STATIC MYBOOL presolve_setOrig(lprec *lp, int orig_rows, int orig_cols);
STATIC MYBOOL presolve_colfix(presolverec *psdata, int colnr, REAL newvalue, MYBOOL remove, int *tally);
STATIC MYBOOL presolve_fillUndo(lprec *lp, int orig_rows, int orig_cols, MYBOOL setOrig);
STATIC MYBOOL presolve_freeUndo(lprec *lp);

STATIC MYBOOL presolve_updatesums(presolverec *psdata);

INLINE int presolve_nextrow(presolverec *psdata, int colnr, int *previtem);
INLINE int presolve_nextcol(presolverec *psdata, int rownr, int *previtem);

STATIC presolverec *presolve_init(lprec *lp);
STATIC void presolve_free(presolverec **psdata);
STATIC int presolve_shrink(presolverec *psdata, int *nConRemove, int *nVarRemove);
STATIC void presolve_rowremove(presolverec *psdata, int rownr, MYBOOL allowcoldelete);
STATIC int presolve_colremove(presolverec *psdata, int colnr, MYBOOL allowrowdelete);

STATIC MYBOOL presolve_colfixdual(presolverec *psdata, int colnr, REAL *fixValue, int *status);

INLINE int presolve_rowlength(presolverec *psdata, int rownr)
{
  int *items = psdata->rows->next[rownr];

  if(items == NULL)
    return( 0 );
  else
    return( items[0] );
}
INLINE int presolve_collength(presolverec *psdata, int colnr)
{
  int *items = psdata->cols->next[colnr];
  if(items == NULL)
    return( 0 );
  else
    return( items[0] );
}

STATIC int presolve(lprec *lp);
STATIC MYBOOL postsolve(lprec *lp, int status);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_presolve */
