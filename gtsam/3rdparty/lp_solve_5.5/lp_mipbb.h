#ifndef HEADER_lp_mipbb
#define HEADER_lp_mipbb

#include "lp_types.h"
#include "lp_utils.h"


/* Bounds storage for B&B routines */
typedef struct _BBrec
{
  struct    _BBrec *parent;
  struct    _BBrec *child;
  lprec     *lp;
  int       varno;
  int       vartype;
  int       lastvarcus;            /* Count of non-int variables of the previous branch */
  int       lastrcf;
  int       nodesleft;
  int       nodessolved;
  int       nodestatus;
  REAL      noderesult;
  REAL      lastsolution;          /* Optimal solution of the previous branch */
  REAL      sc_bound;
  REAL      *upbo,   *lowbo;
  REAL      UPbound, LObound;
  int       UBtrack, LBtrack;      /* Signals that incoming bounds were changed */
  MYBOOL    contentmode;           /* Flag indicating if we "own" the bound vectors */
  MYBOOL    sc_canset;
  MYBOOL    isSOS;
  MYBOOL    isGUB;
  int       *varmanaged;           /* Extended list of variables managed by this B&B level */
  MYBOOL    isfloor;               /* State variable indicating the active B&B bound */
  MYBOOL    UBzerobased;           /* State variable indicating if bounds have been rebased */
} BBrec;

#ifdef __cplusplus
extern "C" {
#endif

STATIC BBrec *create_BB(lprec *lp, BBrec *parentBB, MYBOOL dofullcopy);
STATIC BBrec *push_BB(lprec *lp, BBrec *parentBB, int varno, int vartype, int varcus);
STATIC MYBOOL initbranches_BB(BBrec *BB);
STATIC MYBOOL fillbranches_BB(BBrec *BB);
STATIC MYBOOL nextbranch_BB(BBrec *BB);
STATIC MYBOOL strongbranch_BB(lprec *lp, BBrec *BB, int varno, int vartype, int varcus);
STATIC MYBOOL initcuts_BB(lprec *lp);
STATIC int updatecuts_BB(lprec *lp);
STATIC MYBOOL freecuts_BB(lprec *lp);
STATIC BBrec *findself_BB(BBrec *BB);
STATIC int solve_LP(lprec *lp, BBrec *BB);
STATIC int rcfbound_BB(BBrec *BB, int varno, MYBOOL isINT, REAL *newbound, MYBOOL *isfeasible);
STATIC MYBOOL findnode_BB(BBrec *BB, int *varno, int *vartype, int *varcus);
STATIC int solve_BB(BBrec *BB);
STATIC MYBOOL free_BB(BBrec **BB);
STATIC BBrec *pop_BB(BBrec *BB);

STATIC int run_BB(lprec *lp);

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_mipbb */

