#ifndef HEADER_lp_price
#define HEADER_lp_price

/* Local defines                                                             */
/* ------------------------------------------------------------------------- */
#define UseSortOnBound_Improve
/*#define UseSortOnBound_Substitute*/

#if 0 /* Stricter feasibility-preserving tolerance; use w/ *_UseRejectionList */
  #define UseRelativeFeasibility       /* Use machine-precision and A-scale data */
#endif
#if 0          /* Stricter pivot-selection criteria; use w/ *UseRejectionList */
  #define UseRelativePivot_Primal             /* In rowprim based on A-scale data */
  #define UseRelativePivot_Dual               /* In coldual based on A-scale data */
#endif


/* Include required library headers                                          */
/* ------------------------------------------------------------------------- */
#include "lp_types.h"


#ifdef __cplusplus
extern "C" {
#endif

/* Comparison and validity routines */
int CMP_CALLMODEL compareImprovementVar(const pricerec *current, const pricerec *candidate);
int CMP_CALLMODEL compareSubstitutionVar(const pricerec *current, const pricerec *candidate);
int CMP_CALLMODEL compareBoundFlipVar(const pricerec *current, const pricerec *candidate);
STATIC int addCandidateVar(pricerec *candidate, multirec *multi, findCompare_func findCompare, MYBOOL allowSortedExpand);
STATIC MYBOOL collectMinorVar(pricerec *candidate, multirec *longsteps, MYBOOL isphase2, MYBOOL isbatch);
STATIC MYBOOL validImprovementVar(pricerec *candidate);
STATIC MYBOOL validSubstitutionVar(pricerec *candidate);

/* Row+column selection routines */
STATIC MYBOOL findImprovementVar(pricerec *current, pricerec *candidate, MYBOOL collectMP, int *candidatecount);
STATIC MYBOOL findSubstitutionVar(pricerec *current, pricerec *candidate, int *candidatecount);
INLINE REAL normalizeEdge(lprec *lp, int item, REAL edge, MYBOOL isdual);
STATIC void makePriceLoop(lprec *lp, int *start, int *end, int *delta);

/* Computation of reduced costs */
STATIC void update_reducedcosts(lprec *lp, MYBOOL isdual, int leave_nr, int enter_nr, REAL *prow, REAL *drow);
STATIC void compute_reducedcosts(lprec *lp, MYBOOL isdual, int row_nr, int *coltarget, MYBOOL dosolve,
                                                            REAL *prow, int *nzprow,
                                                            REAL *drow, int *nzdrow,
                                                            int roundmode);

/* Leaving variable selection and entering column pricing loops */
STATIC int find_rowReplacement(lprec *lp, int rownr, REAL *prow, int *nzprow);
STATIC int colprim(lprec *lp, REAL *drow, int *nzdrow,
                              MYBOOL skipupdate, int partialloop, int *candidatecount, MYBOOL updateinfeas, REAL *xviol);
STATIC int rowprim(lprec *lp, int colnr, LREAL *theta, REAL *pcol, int *nzpcol, MYBOOL forceoutEQ, REAL *xviol);
STATIC int rowdual(lprec *lp, REAL *rhvec, MYBOOL forceoutEQ, MYBOOL updateinfeas, REAL *xviol);
STATIC int coldual(lprec *lp, int row_nr,
                              REAL *prow, int *nzprow, REAL *drow, int *nzdrow,
                              MYBOOL dualphase1, MYBOOL skipupdate,
                              int *candidatecount, REAL *xviol);

/* Partial pricing management routines */
STATIC partialrec *partial_createBlocks(lprec *lp, MYBOOL isrow);
STATIC int partial_countBlocks(lprec *lp, MYBOOL isrow);
STATIC int partial_activeBlocks(lprec *lp, MYBOOL isrow);
STATIC void partial_freeBlocks(partialrec **blockdata);

/* Partial pricing utility routines */
STATIC int partial_findBlocks(lprec *lp, MYBOOL autodefine, MYBOOL isrow);
STATIC int partial_blockStart(lprec *lp, MYBOOL isrow);
STATIC int partial_blockEnd(lprec *lp, MYBOOL isrow);
STATIC int partial_blockNextPos(lprec *lp, int block, MYBOOL isrow);

STATIC MYBOOL partial_blockStep(lprec *lp, MYBOOL isrow);
STATIC MYBOOL partial_isVarActive(lprec *lp, int varno, MYBOOL isrow);

/* Multiple pricing / dual long step management routines */
STATIC multirec *multi_create(lprec *lp, MYBOOL truncinf);
STATIC MYBOOL multi_resize(multirec *multi, int blocksize, int blockdiv, MYBOOL doVlist, MYBOOL doIset);
STATIC int multi_restart(multirec *multi);
STATIC int multi_size(multirec *multi);
STATIC int multi_used(multirec *multi);
STATIC MYBOOL multi_truncatingvar(multirec *multi, int varnr);
STATIC MYBOOL multi_mustupdate(multirec *multi);
STATIC void multi_valueInit(multirec *multi, REAL step_base, REAL obj_base);
STATIC REAL *multi_valueList(multirec *multi);
STATIC int *multi_indexSet(multirec *multi, MYBOOL regenerate);
STATIC int multi_getvar(multirec *multi, int item);
STATIC MYBOOL multi_recompute(multirec *multi, int index, MYBOOL isphase2, MYBOOL fullupdate);
STATIC MYBOOL multi_removevar(multirec *multi, int varnr);
STATIC int multi_enteringvar(multirec *multi, pricerec *current, int priority);
STATIC REAL multi_enteringtheta(multirec *multi);
STATIC void multi_free(multirec **multi);
STATIC int multi_populateSet(multirec *multi, int **list, int excludenr);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_price */

