
/*
    Mixed integer programming optimization drivers for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Michel Berkelaar (to lp_solve v3.2)
                   Kjell Eikland    (v4.0 and forward)
    Contact:
    License terms: LGPL.

    Requires:      string.h, float.h, commonlib.h, lp_lib.h, lp_report.h,
                   lp_simplex.h

    Release notes:
    v5.0.0 31 January 2004      New unit isolating B&B routines.
    v5.0.1 01 February 2004     Complete rewrite into non-recursive version.
    v5.0.2 05 April 2004        Expanded pseudocosting with options for MIP fraction
                                counts and "cost/benefit" ratio (KE special!).
                                Added GUB functionality based on SOS structures.
    v5.0.3    1 May 2004        Changed routine names to be more intuitive.
    v5.0.4    15 May 2004       Added functinality to pack bounds in order to
                                conserve memory in B&B-processing large MIP models.
    v5.1.0    25 July 2004      Added functions for dynamic cut generation.
    v5.2.0    15 December 2004  Added functions for reduced cost variable fixing
                                and converted to delta-model of B&B bound storage.
   ----------------------------------------------------------------------------------
*/

#include <string.h>
#include <float.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_scale.h"
#include "lp_report.h"
#include "lp_simplex.h"
#include "lp_mipbb.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


/* Allocation routine for the BB record structure */
STATIC BBrec *create_BB(lprec *lp, BBrec *parentBB, MYBOOL dofullcopy)
{
  BBrec *newBB;

  newBB = (BBrec *) calloc(1, sizeof(*newBB));
  if(newBB != NULL) {

    if(parentBB == NULL) {
      allocREAL(lp, &newBB->upbo,  lp->sum + 1, FALSE);
      allocREAL(lp, &newBB->lowbo, lp->sum + 1, FALSE);
      MEMCOPY(newBB->upbo,  lp->orig_upbo,  lp->sum + 1);
      MEMCOPY(newBB->lowbo, lp->orig_lowbo, lp->sum + 1);
    }
    else if(dofullcopy) {
      allocREAL(lp, &newBB->upbo,  lp->sum + 1, FALSE);
      allocREAL(lp, &newBB->lowbo, lp->sum + 1, FALSE);
      MEMCOPY(newBB->upbo,  parentBB->upbo,  lp->sum + 1);
      MEMCOPY(newBB->lowbo, parentBB->lowbo, lp->sum + 1);
    }
    else {
      newBB->upbo  = parentBB->upbo;
      newBB->lowbo = parentBB->lowbo;
    }
    newBB->contentmode = dofullcopy;

    newBB->lp = lp;

    /* Set parent by default, but not child */
    newBB->parent = parentBB;

  }
  return( newBB );
}


/* Pushing and popping routines for the B&B structure */

STATIC BBrec *push_BB(lprec *lp, BBrec *parentBB, int varno, int vartype, int varcus)
/* Push ingoing bounds and B&B data onto the stack */
{
  BBrec *newBB;

  /* Do initialization and updates */
  if(parentBB == NULL)
    parentBB = lp->bb_bounds;
  newBB = create_BB(lp, parentBB, FALSE);
  if(newBB != NULL) {

    newBB->varno = varno;
    newBB->vartype = vartype;
    newBB->lastvarcus = varcus;
    incrementUndoLadder(lp->bb_lowerchange);
    newBB->LBtrack++;
    incrementUndoLadder(lp->bb_upperchange);
    newBB->UBtrack++;

    /* Adjust variable fixing/bound tightening based on the last reduced cost */
    if((parentBB != NULL) && (parentBB->lastrcf > 0)) {
      MYBOOL isINT;
      int    k, ii, nfixed = 0, ntighten = 0;
      REAL   deltaUL;

      for(k = 1; k <= lp->nzdrow[0]; k++) {
        ii = lp->nzdrow[k];
#ifdef UseMilpSlacksRCF  /* Check if we should include ranged constraints */
        isINT = FALSE;
#else
        if(ii <= lp->rows)
          continue;
        isINT = is_int(lp, ii-lp->rows);
#endif
#ifndef UseMilpExpandedRCF  /* Don't include non-integers if it is not defined */
        if(!isINT)
          continue;
#endif
        switch(abs(rcfbound_BB(newBB, ii, isINT, &deltaUL, NULL))) {
          case LE: SETMIN(deltaUL, newBB->upbo[ii]);
                   SETMAX(deltaUL, newBB->lowbo[ii]);
                   modifyUndoLadder(lp->bb_upperchange, ii, newBB->upbo, deltaUL);
                   break;
          case GE: SETMAX(deltaUL, newBB->lowbo[ii]);
                   SETMIN(deltaUL, newBB->upbo[ii]);
                   modifyUndoLadder(lp->bb_lowerchange, ii, newBB->lowbo, deltaUL);
                   break;
          default: continue;
        }
        if(newBB->upbo[ii] == newBB->lowbo[ii])
          nfixed++;
        else
          ntighten++;
      }
      if(lp->bb_trace) {
        report(lp, DETAILED,
                 "push_BB: Used reduced cost to fix %d variables and tighten %d bounds\n",
                  nfixed, ntighten);
      }
    }

    /* Handle case where we are pushing at the end */
    if(parentBB == lp->bb_bounds)
      lp->bb_bounds = newBB;
    /* Handle case where we are pushing in the middle */
    else
      newBB->child = parentBB->child;
    if(parentBB != NULL)
      parentBB->child = newBB;

    lp->bb_level++;
    if(lp->bb_level > lp->bb_maxlevel)
      lp->bb_maxlevel = lp->bb_level;

    if(!initbranches_BB(newBB))
      newBB = pop_BB(newBB);
    else if(MIP_count(lp) > 0) {
      if( (lp->bb_level <= 1) && (lp->bb_varactive == NULL) &&
          (!allocINT(lp, &lp->bb_varactive, lp->columns+1, TRUE) ||
           !initcuts_BB(lp)) )
        newBB = pop_BB(newBB);
      if(varno > 0) {
        lp->bb_varactive[varno-lp->rows]++;
      }
    }
  }
  return( newBB );
}

STATIC MYBOOL free_BB(BBrec **BB)
{
  MYBOOL parentreturned = FALSE;

  if((BB != NULL) && (*BB != NULL)) {
    BBrec *parent = (*BB)->parent;

    if((parent == NULL) || (*BB)->contentmode) {
      FREE((*BB)->upbo);
      FREE((*BB)->lowbo);
    }
    FREE((*BB)->varmanaged);
    FREE(*BB);

    parentreturned = (MYBOOL) (parent != NULL);
    if(parentreturned)
      *BB = parent;

  }
  return( parentreturned );
}

STATIC BBrec *pop_BB(BBrec *BB)
/* Pop / free the previously "pushed" / saved bounds */
{
  int   k;
  BBrec *parentBB;
  lprec *lp = BB->lp;

  if(BB == NULL)
    return( BB );

  /* Handle case where we are popping the end of the chain */
  parentBB = BB->parent;
  if(BB == lp->bb_bounds) {
    lp->bb_bounds = parentBB;
    if(parentBB != NULL)
      parentBB->child = NULL;
  }
  /* Handle case where we are popping inside or at the beginning of the chain */
  else {
    if(parentBB != NULL)
      parentBB->child = BB->child;
    if(BB->child != NULL)
      BB->child->parent = parentBB;
  }

  /* Unwind other variables */
  if(lp->bb_upperchange != NULL) {
    restoreUndoLadder(lp->bb_upperchange, BB->upbo);
    for(; BB->UBtrack > 0; BB->UBtrack--) {
      decrementUndoLadder(lp->bb_upperchange);
      restoreUndoLadder(lp->bb_upperchange, BB->upbo);
    }
  }
  if(lp->bb_lowerchange != NULL) {
    restoreUndoLadder(lp->bb_lowerchange, BB->lowbo);
    for(; BB->LBtrack > 0; BB->LBtrack--) {
      decrementUndoLadder(lp->bb_lowerchange);
      restoreUndoLadder(lp->bb_lowerchange, BB->lowbo);
    }
  }
  lp->bb_level--;
  k = BB->varno - lp->rows;
  if(lp->bb_level == 0) {
    if(lp->bb_varactive != NULL) {
      FREE(lp->bb_varactive);
      freecuts_BB(lp);
    }
    if(lp->int_vars+lp->sc_vars > 0)
      free_pseudocost(lp);
    pop_basis(lp, FALSE);
    lp->rootbounds = NULL;
  }
  else
    lp->bb_varactive[k]--;

  /* Undo SOS/GUB markers */
  if(BB->isSOS && (BB->vartype != BB_INT))
    SOS_unmark(lp->SOS, 0, k);
  else if(BB->isGUB)
    SOS_unmark(lp->GUB, 0, k);

  /* Undo the SC marker */
  if(BB->sc_canset)
    lp->sc_lobound[k] *= -1;

  /* Pop the associated basis */
#if 1
  /* Original version that does not restore previous basis */
  pop_basis(lp, FALSE);
#else
  /* Experimental version that restores previous basis */
  pop_basis(lp, BB->isSOS);
#endif

  /* Finally free the B&B object */
  free_BB(&BB);

  /* Return the parent BB */
  return( parentBB );
}

/* Here are heuristic routines to see if we need bother with branching further

    1. A probing routine to see of the best OF can be better than incumbent
    2. A presolve routine to fix other variables and detect infeasibility

   THIS IS INACTIVE CODE, PLACEHOLDERS FOR FUTURE DEVELOPMENT!!! */
STATIC REAL probe_BB(BBrec *BB)
{
  int  i, ii;
  REAL coefOF, sum = 0;
  lprec *lp = BB->lp;

  /* Loop over all ints to see if the best possible solution
     stands any chance of being better than the incumbent solution */
  if(lp->solutioncount == 0)
    return( lp->infinite );
  for(i = 1; i <= lp->columns; i++) {
    if(!is_int(lp, i))
      continue;
    ii = lp->rows + i;
    coefOF = lp->obj[i];
    if(coefOF < 0) {
      if(is_infinite(lp, BB->lowbo[ii]))
        return( lp->infinite );
      sum += coefOF * (lp->solution[ii]-BB->lowbo[ii]);
    }
    else {
      if(is_infinite(lp, BB->upbo[ii]))
        return( lp->infinite );
      sum += coefOF * (BB->upbo[ii] - lp->solution[ii]);
    }
  }
  return( sum );
}

STATIC REAL presolve_BB(BBrec *BB)
{
  return( 0 );
}

/* Node and branch management routines */
STATIC MYBOOL initbranches_BB(BBrec *BB)
{
  REAL   new_bound, temp;
  int    k;
  lprec  *lp = BB->lp;

 /* Create and initialize local bounds and basis */
  BB->nodestatus = NOTRUN;
  BB->noderesult = lp->infinite;
  push_basis(lp, NULL, NULL, NULL);

 /* Set default number of branches at the current B&B branch */
  if(BB->vartype == BB_REAL)
    BB->nodesleft = 1;

  else {
   /* The default is a binary up-low branching */
    BB->nodesleft = 2;

   /* Initialize the MIP status code pair and set reference values */
    k = BB->varno - lp->rows;
    BB->lastsolution = lp->solution[BB->varno];

   /* Determine if we must process in the B&B SOS mode */
    BB->isSOS = (MYBOOL) ((BB->vartype == BB_SOS) || SOS_is_member(lp->SOS, 0, k));
#ifdef Paranoia
    if((BB->vartype == BB_SOS) && !SOS_is_member(lp->SOS, 0, k))
      report(lp, SEVERE, "initbranches_BB: Inconsistent identification of SOS variable %s (%d)\n",
                         get_col_name(lp, k), k);
#endif

   /* Check if we have a GUB-member variable that needs a triple-branch */
    BB->isGUB = (MYBOOL) ((BB->vartype == BB_INT) && SOS_can_activate(lp->GUB, 0, k));
    if(BB->isGUB) {
      /* Obtain variable index list from applicable GUB - now the first GUB is used,
        but we could also consider selecting the longest */
      BB->varmanaged = SOS_get_candidates(lp->GUB, -1, k, TRUE, BB->upbo, BB->lowbo);
      BB->nodesleft++;
    }


   /* Set local pruning info, automatic, or user-defined strategy */
    if(BB->vartype == BB_SOS) {
      if(!SOS_can_activate(lp->SOS, 0, k)) {
        BB->nodesleft--;
        BB->isfloor = TRUE;
      }
      else
        BB->isfloor = (MYBOOL) (BB->lastsolution == 0);
    }

    /* First check if the user wishes to select the branching direction */
    else if(lp->bb_usebranch != NULL)
      BB->isfloor = (MYBOOL) lp->bb_usebranch(lp, lp->bb_branchhandle, k);

    /* Otherwise check if we should do automatic branching */
    else if(get_var_branch(lp, k) == BRANCH_AUTOMATIC) {
      new_bound = modf(BB->lastsolution/get_pseudorange(lp->bb_PseudoCost, k, BB->vartype), &temp);
      if(_isnan(new_bound))
        new_bound = 0;
      else if(new_bound < 0)
        new_bound += 1.0;
      BB->isfloor = (MYBOOL) (new_bound <= 0.5);

      /* Set direction by OF value; note that a zero-value in
         the OF gives priority to floor_first = TRUE */
      if(is_bb_mode(lp, NODE_GREEDYMODE)) {
        if(is_bb_mode(lp, NODE_PSEUDOCOSTMODE))
          BB->sc_bound = get_pseudonodecost(lp->bb_PseudoCost, k, BB->vartype, BB->lastsolution);
        else
          BB->sc_bound = mat_getitem(lp->matA, 0, k);
        new_bound -= 0.5;
        BB->sc_bound *= new_bound;
        BB->isfloor = (MYBOOL) (BB->sc_bound > 0);
      }
      /* Set direction by pseudocost (normally used in tandem with NODE_PSEUDOxxxSELECT) */
      else if(is_bb_mode(lp, NODE_PSEUDOCOSTMODE)) {
        BB->isfloor = (MYBOOL) (get_pseudobranchcost(lp->bb_PseudoCost, k, TRUE) >
                                get_pseudobranchcost(lp->bb_PseudoCost, k, FALSE));
        if(is_maxim(lp))
          BB->isfloor = !BB->isfloor;
      }

      /* Check for reversal */
      if(is_bb_mode(lp, NODE_BRANCHREVERSEMODE))
        BB->isfloor = !BB->isfloor;
    }
    else
      BB->isfloor = (MYBOOL) (get_var_branch(lp, k) == BRANCH_FLOOR);

    /* SC logic: If the current SC variable value is in the [0..NZLOBOUND> range, then

      UP: Set lower bound to NZLOBOUND, upper bound is the original
      LO: Fix the variable by setting upper/lower bound to zero

      ... indicate that the variable is B&B-active by reversing sign of sc_lobound[]. */
    new_bound = fabs(lp->sc_lobound[k]);
    BB->sc_bound = new_bound;
    BB->sc_canset = (MYBOOL) (new_bound != 0);

   /* Must make sure that we handle fractional lower bounds properly;
      also to ensure that we do a full binary tree search */
    new_bound = unscaled_value(lp, new_bound, BB->varno);
    if(is_int(lp, k) && ((new_bound > 0) &&
                         (BB->lastsolution > floor(new_bound)))) {
      if(BB->lastsolution < ceil(new_bound))
        BB->lastsolution += 1;
      modifyUndoLadder(lp->bb_lowerchange, BB->varno, BB->lowbo,
                       scaled_floor(lp, BB->varno, BB->lastsolution, 1));
    }
  }

  /* Now initialize the brances and set to first */
  return( fillbranches_BB(BB) );
}

STATIC MYBOOL fillbranches_BB(BBrec *BB)
{
  int    K, k;
  REAL   ult_upbo, ult_lowbo;
  REAL   new_bound, SC_bound, intmargin = BB->lp->epsprimal;
  lprec  *lp = BB->lp;
  MYBOOL OKstatus = FALSE;

  if(lp->bb_break || userabort(lp, MSG_MILPSTRATEGY))
    return( OKstatus );

  K = BB->varno;
  if(K > 0) {

  /* Shortcut variables */
    k = BB->varno - lp->rows;
    ult_upbo  = lp->orig_upbo[K];
    ult_lowbo = lp->orig_lowbo[K];
    SC_bound  = unscaled_value(lp, BB->sc_bound, K);

    /* First, establish the upper bound to be applied (when isfloor == TRUE)
       --------------------------------------------------------------------- */
/*SetUB:*/
    BB->UPbound = lp->infinite;

    /* Handle SC-variables for the [0-LoBound> range */
    if((SC_bound > 0) && (fabs(BB->lastsolution) < SC_bound-intmargin)) {
      new_bound = 0;
    }
    /* Handle pure integers (non-SOS, non-SC) */
    else if(BB->vartype == BB_INT) {
#if 1
      if(((ult_lowbo >= 0) &&
          ((floor(BB->lastsolution) < /* Skip cases where the lower bound becomes violated */
            unscaled_value(lp, MAX(ult_lowbo, fabs(lp->sc_lobound[k])), K)-intmargin))) ||
         ((ult_upbo <= 0) &&   /*  Was  ((ult_lowbo < 0) && */
          ((floor(BB->lastsolution) > /* Skip cases where the upper bound becomes violated */
            unscaled_value(lp, MIN(ult_upbo, -fabs(lp->sc_lobound[k])), K)-intmargin)))) {
#else
      if((floor(BB->lastsolution) <  /* Skip cases where the lower bound becomes violated */
          unscaled_value(lp, MAX(ult_lowbo, fabs(lp->sc_lobound[k])), K)-intmargin)) {
#endif
        BB->nodesleft--;
        goto SetLB;
      }
      new_bound = scaled_floor(lp, K, BB->lastsolution, 1);
    }
    else if(BB->isSOS) {           /* Handle all SOS variants */
      new_bound = ult_lowbo;
      if(is_int(lp, k))
        new_bound = scaled_ceil(lp, K, unscaled_value(lp, new_bound, K), -1);
    }
    else                           /* Handle all other variable incarnations */
      new_bound = BB->sc_bound;

    /* Check if the new bound might conflict and possibly make adjustments */
    if(new_bound < BB->lowbo[K])
      new_bound = BB->lowbo[K] - my_avoidtiny(new_bound-BB->lowbo[K], intmargin);
    if(new_bound < BB->lowbo[K]) {
#ifdef Paranoia
      debug_print(lp,
          "fillbranches_BB: New upper bound value %g conflicts with old lower bound %g\n",
          new_bound, BB->lowbo[K]);
#endif
      BB->nodesleft--;
      goto SetLB;
    }
#ifdef Paranoia
    /* Do additional consistency checking */
    else if(!check_if_less(lp, new_bound, BB->upbo[K], K)) {
      BB->nodesleft--;
      goto SetLB;
    }
#endif
    /* Bound (at least near) feasible */
    else {
      /* Makes a difference with models like QUEEN
         (note consistent use of epsint for scaled integer variables) */
      if(fabs(new_bound - BB->lowbo[K]) < intmargin*SCALEDINTFIXRANGE)
        new_bound = BB->lowbo[K];
    }

    BB->UPbound = new_bound;


    /* Next, establish the lower bound to be applied (when isfloor == FALSE)
       --------------------------------------------------------------------- */
SetLB:
    BB->LObound = -lp->infinite;

    /* Handle SC-variables for the [0-LoBound> range */
    if((SC_bound > 0) && (fabs(BB->lastsolution) < SC_bound)) {
      if(is_int(lp, k))
        new_bound = scaled_ceil(lp, K, SC_bound, 1);
      else
        new_bound = BB->sc_bound;
    }
    /* Handle pure integers (non-SOS, non-SC, but Ok for GUB!) */
    else if((BB->vartype == BB_INT)) {
      if(((ceil(BB->lastsolution) == BB->lastsolution)) ||    /* Skip branch 0 if the current solution is integer */
         (ceil(BB->lastsolution) >   /* Skip cases where the upper bound becomes violated */
          unscaled_value(lp, ult_upbo, K)+intmargin) ||
          (BB->isSOS && (BB->lastsolution == 0))) {           /* Don't branch 0 since this is handled in SOS logic */
        BB->nodesleft--;
        goto Finish;
      }
      new_bound = scaled_ceil(lp, K, BB->lastsolution, 1);
    }
    else if(BB->isSOS) {             /* Handle all SOS variants */
      if(SOS_is_member_of_type(lp->SOS, k, SOS3))
        new_bound = scaled_floor(lp, K, 1, 1);
      else {
        new_bound = ult_lowbo;
        if(is_int(lp, k))
          new_bound = scaled_floor(lp, K, unscaled_value(lp, new_bound, K), 1);
        /* If we have a high-order SOS (SOS3+) and this variable is "intermediate"
          between members previously lower-bounded at a non-zero level, then we should
          set this and similar neighbouring variables at non-zero lowbo-values (remember
          that SOS3+ members are all either integers or semi-continuous). Flag this
          situation and prune tree, since we cannot lower-bound. */
        if((lp->SOS->maxorder > 2) && (BB->lastsolution == 0) &&
           SOS_is_member_of_type(lp->SOS, k, SOSn)) {
          BB->isSOS = AUTOMATIC;
        }
      }
    }
    else                              /* Handle all other variable incarnations */
      new_bound = BB->sc_bound;

    /* Check if the new bound might conflict and possibly make adjustments */
    if(new_bound > BB->upbo[K])
      new_bound = BB->upbo[K] + my_avoidtiny(new_bound-BB->upbo[K], intmargin);
    if(new_bound > BB->upbo[K]) {
#ifdef Paranoia
      debug_print(lp,
        "fillbranches_BB: New lower bound value %g conflicts with old upper bound %g\n",
        new_bound, BB->upbo[K]);
#endif
      BB->nodesleft--;
      goto Finish;
    }
#ifdef Paranoia
    /* Do additional consistency checking */
    else if(!check_if_less(lp, BB->lowbo[K], new_bound, K)) {
      BB->nodesleft--;
      goto Finish;
    }
#endif
    /* Bound (at least near-)feasible */
    else {
      /* Makes a difference with models like QUEEN
         (note consistent use of lp->epsprimal for scaled integer variables) */
      if(fabs(BB->upbo[K]-new_bound) < intmargin*SCALEDINTFIXRANGE)
        new_bound = BB->upbo[K];
    }

    BB->LObound = new_bound;

    /* Prepare for the first branch by making sure we are pointing correctly */
Finish:
    if(BB->nodesleft > 0) {

      /* Make sure the change tracker levels are "clean" for the B&B */
      if(countsUndoLadder(lp->bb_upperchange) > 0) {
        incrementUndoLadder(lp->bb_upperchange);
        BB->UBtrack++;
      }
      if(countsUndoLadder(lp->bb_lowerchange) > 0) {
        incrementUndoLadder(lp->bb_lowerchange);
        BB->LBtrack++;
      }

      /* Do adjustments */
      if((BB->vartype != BB_SOS) && (fabs(BB->LObound-BB->UPbound) < intmargin)) {
        BB->nodesleft--;
        if(fabs(BB->lowbo[K]-BB->LObound) < intmargin)
          BB->isfloor = FALSE;
        else if(fabs(BB->upbo[K]-BB->UPbound) < intmargin)
          BB->isfloor = TRUE;
        else
          report(BB->lp, IMPORTANT, "fillbranches_BB: Inconsistent equal-valued bounds for %s\n",
                                    get_col_name(BB->lp, k));
      }
      if((BB->nodesleft == 1) &&
         ((BB->isfloor && (BB->UPbound >= lp->infinite)) ||
          (!BB->isfloor && (BB->LObound <= -lp->infinite))))
        BB->isfloor = !BB->isfloor;
      /* Header initialization */
      BB->isfloor = !BB->isfloor;
      while(!OKstatus && !lp->bb_break && (BB->nodesleft > 0))
        OKstatus = nextbranch_BB( BB );
    }

    /* Set an SC variable active, if necessary */
    if(BB->sc_canset)
      lp->sc_lobound[k] *= -1;

  }
  else {
    BB->nodesleft--;
    OKstatus = TRUE;
  }

  return( OKstatus );
}

STATIC MYBOOL nextbranch_BB(BBrec *BB)
{
  int    k;
  lprec  *lp = BB->lp;
  MYBOOL OKstatus = FALSE;

  /* Undo the most recently imposed B&B bounds using the data
     in the last level of change tracker; this code handles changes
     to both upper and lower bounds */
  if(BB->nodessolved > 0) {
      restoreUndoLadder(lp->bb_upperchange, BB->upbo);
      restoreUndoLadder(lp->bb_lowerchange, BB->lowbo);
  }

  if(lp->bb_break || userabort(lp, MSG_MILPSTRATEGY)) {
    /* Handle the special case of B&B restart;
       (typically used with the restart after pseudocost initialization) */
    if((lp->bb_level == 1) && (lp->bb_break == AUTOMATIC)) {
      lp->bb_break = FALSE;
      OKstatus = TRUE;
    }
    return( OKstatus );
  }

  if(BB->nodesleft > 0) {

    /* Step and update remaining branch count */
    k = BB->varno - lp->rows;
    BB->isfloor = !BB->isfloor;
    BB->nodesleft--;

    /* Special SOS handling:
       1) Undo and set new marker for k,
       2) In case that previous branch was ceiling restore upper bounds of the
          non-k variables outside of the SOS window set to 0 */
    if(BB->isSOS && (BB->vartype != BB_INT)) {

      /* First undo previous marker */
      if((BB->nodessolved > 0) || ((BB->nodessolved == 0) && (BB->nodesleft == 0))) {
        if(BB->isfloor) {
          if((BB->nodesleft == 0) && (lp->orig_lowbo[BB->varno] != 0))
            return( OKstatus );
        }
        SOS_unmark(lp->SOS, 0, k);
      }

      /* Set new SOS marker */
      if(BB->isfloor) {
        SOS_set_marked(lp->SOS, 0, k, (MYBOOL) (BB->UPbound != 0));
        /* Do case of high-order SOS where intervening variables need to be set */
        if(BB->isSOS == AUTOMATIC) {

/*          SOS_fix_list(lp->SOS, 0, k, BB->lowbo, NULL, AUTOMATIC, lp->bb_lowerchange); */
        }
      }
      else {
        SOS_set_marked(lp->SOS, 0, k, TRUE);
        if(SOS_fix_unmarked(lp->SOS, 0, k, BB->upbo, 0, TRUE,
                            NULL, lp->bb_upperchange) < 0)
          return( OKstatus );
      }
    }

    /* Special GUB handling (three branches):
       1) Undo and set new marker for k,
       2) Restore upper bounds of the left/right/all non-k variables
          set to 0 in the previous branch
       3) Set new upper bounds for the non-k variables (k is set later) */
    else if(BB->isGUB) {

      /* First undo previous marker */
      if(BB->nodessolved > 0)
        SOS_unmark(lp->GUB, 0, k);

      /* Make sure we take floor bound twice */
      if((BB->nodesleft == 0) && !BB->isfloor)
        BB->isfloor = !BB->isfloor;

      /* Handle two floor instances;
         (selected variable and left/right halves of non-selected variables at 0) */
      SOS_set_marked(lp->GUB, 0, k, (MYBOOL) !BB->isfloor);
      if(BB->isfloor) {
        if(SOS_fix_list(lp->GUB, 0, k, BB->upbo,
                        BB->varmanaged, (MYBOOL) (BB->nodesleft > 0), lp->bb_upperchange) < 0)
          return( OKstatus );
      }
      /* Handle one ceil instance;
         (selected variable at 1, all other at 0) */
      else {
        if(SOS_fix_unmarked(lp->GUB, 0, k, BB->upbo, 0, TRUE,
                            NULL, lp->bb_upperchange) < 0)
          return( OKstatus );
      }
    }

    OKstatus = TRUE;

  }
  /* Initialize simplex status variables */
  if(OKstatus) {
    lp->bb_totalnodes++;
    BB->nodestatus = NOTRUN;
    BB->noderesult = lp->infinite;
  }
  return( OKstatus );
}


/* Cut generation and management routines */
STATIC MYBOOL initcuts_BB(lprec *lp)
{
  return( TRUE );
}

STATIC int updatecuts_BB(lprec *lp)
{
  return( 0 );
}

STATIC MYBOOL freecuts_BB(lprec *lp)
{
  if(lp->bb_cuttype != NULL)
    FREE(lp->bb_cuttype);
  return( TRUE );
}

/* B&B solver routines */
STATIC int solve_LP(lprec *lp, BBrec *BB)
{
  int    tilted, restored, status;
  REAL   testOF, *upbo = BB->upbo, *lowbo = BB->lowbo;
  BBrec  *perturbed = NULL;

  if(lp->bb_break)
    return(PROCBREAK);

#ifdef Paranoia
  debug_print(lp, "solve_LP: Starting solve for iter %.0f, B&B node level %d.\n",
                   (double) lp->total_iter, lp->bb_level);
  if(lp->bb_trace &&
     !validate_bounds(lp, upbo, lowbo))
    report(lp, SEVERE, "solve_LP: Inconsistent bounds at iter %.0f, B&B node level %d.\n",
                       (double) lp->total_iter, lp->bb_level);
#endif

  /* Copy user-specified entering bounds into lp_solve working bounds */
  impose_bounds(lp, upbo, lowbo);

  /* Restore previously pushed / saved basis for this level if we are in
     the B&B mode and it is not the first call of the binary tree */
  if(BB->nodessolved > 1)
    restore_basis(lp);

  /* Solve and possibly handle degeneracy cases via bound relaxations */
  status   = RUNNING;
  tilted   = 0;
  restored = 0;

  while(status == RUNNING) {

    /* Copy user-specified entering bounds into lp_solve working bounds and run */
    status = spx_run(lp, (MYBOOL) (tilted+restored > 0));
    lp->bb_status     = status;
    lp->spx_perturbed = FALSE;

    if(tilted < 0)
      break;

    else if((status == OPTIMAL) && (tilted > 0)) {
      if(lp->spx_trace)
        report(lp, DETAILED, "solve_LP: Restoring relaxed bounds at level %d.\n",
                              tilted);

    /* Restore original pre-perturbed problem bounds, and solve again using the basis
       found for the perturbed problem; also make sure we rebase and recompute. */
      free_BB(&perturbed);
      if((perturbed == NULL) || (perturbed == BB)) {
        perturbed = NULL;
        impose_bounds(lp, upbo, lowbo);
      }
      else
        impose_bounds(lp, perturbed->upbo, perturbed->lowbo);
      set_action(&lp->spx_action, ACTION_REBASE | ACTION_RECOMPUTE);
      BB->UBzerobased = FALSE;
      if(lp->bb_totalnodes == 0)
        lp->real_solution = lp->infinite;
      status = RUNNING;
      tilted--;
      restored++;
      lp->spx_perturbed = TRUE;
    }

    else if(((lp->bb_level <= 1) ||     is_anti_degen(lp, ANTIDEGEN_DURINGBB)) &&
            (((status == LOSTFEAS) &&   is_anti_degen(lp, ANTIDEGEN_LOSTFEAS)) ||
             ((status == INFEASIBLE) && is_anti_degen(lp, ANTIDEGEN_INFEASIBLE)) ||
             ((status == NUMFAILURE) && is_anti_degen(lp, ANTIDEGEN_NUMFAILURE)) ||
             ((status == DEGENERATE) && is_anti_degen(lp, ANTIDEGEN_STALLING)))) {
     /* Allow up to .. consecutive relaxations for non-B&B phases */
      if((tilted <= DEF_MAXRELAX) &&                       /* Conventional recovery case,...  */
         !((tilted == 0) && (restored > DEF_MAXRELAX))) {  /* but not iterating infeasibility */

        /* Create working copy of ingoing bounds if this is the first perturbation */
        if(tilted == 0)
          perturbed = BB;
        perturbed = create_BB(lp, perturbed, TRUE);

        /* Perturb/shift variable bounds; also make sure we rebase and recompute
           (no refactorization is necessary, since the basis is unchanged) */
#if 1
        perturb_bounds(lp, perturbed, TRUE, TRUE, TRUE);
#else
        perturb_bounds(lp, perturbed, TRUE, TRUE, FALSE);
#endif
        impose_bounds(lp, perturbed->upbo, perturbed->lowbo);
        set_action(&lp->spx_action, ACTION_REBASE | ACTION_RECOMPUTE);
        BB->UBzerobased = FALSE;
        status = RUNNING;
        tilted++;
        lp->perturb_count++;
        lp->spx_perturbed = TRUE;
        if(lp->spx_trace)
          report(lp, DETAILED, "solve_LP: Starting bound relaxation #%d ('%s')\n",
                               tilted, get_statustext(lp, status));
      }
      else  {
        if(lp->spx_trace)
          report(lp, DETAILED, "solve_LP: Relaxation limit exceeded in resolving infeasibility\n");
        while((perturbed != NULL) && (perturbed != BB))
          free_BB(&perturbed);
        perturbed = NULL;
      }
    }
  }

  /* Handle the different simplex outcomes */
  if(status != OPTIMAL) {
    if(lp->bb_level <= 1)
      lp->bb_parentOF = lp->infinite;
    if((status == USERABORT) || (status == TIMEOUT)) {
      /* Construct the last feasible solution, if available */
      if((lp->solutioncount == 0) &&
         ((lp->simplex_mode & (SIMPLEX_Phase2_PRIMAL | SIMPLEX_Phase2_DUAL)) > 0)) {
        lp->solutioncount++;
        construct_solution(lp, NULL);
        transfer_solution(lp, TRUE);
      }
      /* Return messages */
      report(lp, NORMAL, "\nlp_solve optimization was stopped %s.\n",
                         ((status == USERABORT) ? "by the user" : "due to time-out"));
    }
    else if(BB->varno == 0)
      report(lp, NORMAL, "The model %s\n",
      (status == UNBOUNDED) ? "is UNBOUNDED" :
      ((status == INFEASIBLE) ? "is INFEASIBLE" : "FAILED"));
    else {
#ifdef Paranoia
      if((status != FATHOMED) && (status != INFEASIBLE))
        report(lp, SEVERE, "spx_solve: Invalid return code %d during B&B\n", status);
#endif
      /* If we fathomed a node due to an inferior OF having been detected, return infeasible */
      if(status == FATHOMED)
        lp->spx_status = INFEASIBLE;
    }
  }

  else { /* ... there is a good solution */
    construct_solution(lp, NULL);
    if((lp->bb_level <= 1) && (restored > 0))
      report(lp, DETAILED, "%s numerics encountered; validate accuracy\n",
                 (restored == 1) ? "Difficult" : "Severe");
    /* Handle case where a user bound on the OF was found to
       have been set too aggressively, giving an infeasible model */
    if(lp->spx_status != OPTIMAL)
      status = lp->spx_status;

    else if((lp->bb_totalnodes == 0) && (MIP_count(lp) > 0)) {
      if(lp->lag_status != RUNNING) {
       report(lp, NORMAL, "\nRelaxed solution  " RESULTVALUEMASK " after %10.0f iter is B&B base.\n",
                          lp->solution[0], (double) lp->total_iter);
        report(lp, NORMAL, " \n");
      }
      if((lp->usermessage != NULL) && (lp->msgmask & MSG_LPOPTIMAL))
        lp->usermessage(lp, lp->msghandle, MSG_LPOPTIMAL);
      set_var_priority(lp);
    }

   /* Check if we have a numeric problem (an earlier version of this code used the
      absolute difference, but it is not robust for large-valued OFs) */
    testOF = my_chsign(is_maxim(lp), my_reldiff(lp->solution[0], lp->real_solution));
    if(testOF < -lp->epsprimal) {
      report(lp, DETAILED, "solve_LP: A MIP subproblem returned a value better than the base.\n");
      status = INFEASIBLE;
      lp->spx_status = status;
      set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
    }
    else if(testOF < 0)  /* Avoid problems later (could undo integer roundings, but usually Ok) */
      lp->solution[0] = lp->real_solution;

  }

  /* status can have the following values:
     OPTIMAL, SUBOPTIMAL, TIMEOUT, USERABORT, PROCFAIL, UNBOUNDED and INFEASIBLE. */

  return( status );
} /* solve_LP */

STATIC BBrec *findself_BB(BBrec *BB)
{
  int   varno = BB->varno, vartype = BB->vartype;

  BB = BB->parent;
  while((BB != NULL) && (BB->vartype != vartype) && (BB->varno != varno))
    BB = BB->parent;
  return( BB );
}

/* Function to determine the opportunity for variable fixing and bound
   tightening based on a previous best MILP solution and a variable's
   reduced cost at the current relaxation - inspired by Wolsley */
STATIC int rcfbound_BB(BBrec *BB, int varno, MYBOOL isINT, REAL *newbound, MYBOOL *isfeasible)
{
  int   i = FR;
  lprec *lp = BB->lp;
  REAL  deltaRC, rangeLU, deltaOF, lowbo, upbo;

  /* Make sure we only accept non-basic variables */
  if(lp->is_basic[varno])
    return( i );

  /* Make sure we only accept non-fixed variables */
  lowbo = BB->lowbo[varno];
  upbo  = BB->upbo[varno];
  rangeLU = upbo - lowbo;

  if(rangeLU > lp->epsprimal) {
    deltaOF = lp->rhs[0] - lp->bb_workOF;
    deltaRC = my_chsign(!lp->is_lower[varno], lp->drow[varno]);
    /* Protect against divisions with tiny numbers and stray sign
       reversals of the reduced cost */
    if(deltaRC < lp->epspivot)
      return( i );
    deltaRC = deltaOF / deltaRC;  /* Should always be a positive number! */
#ifdef Paranoia
    if(deltaRC <= 0)
      report(lp, SEVERE, "rcfbound_BB: A negative bound fixing level was encountered after node %.0f\n",
                         (double) lp->bb_totalnodes);
#endif

    /* Check if bound implied by the reduced cost is less than existing range */
    if(deltaRC < rangeLU + lp->epsint) {
      if(lp->is_lower[varno]) {
        if(isINT)
          deltaRC = scaled_floor(lp, varno, unscaled_value(lp, deltaRC, varno)+lp->epsprimal, 1);
        upbo = lowbo + deltaRC;
        deltaRC = upbo;
        i = LE;  /* Sets the upper bound */
      }
      else {
        if(isINT)
          deltaRC = scaled_ceil(lp, varno, unscaled_value(lp, deltaRC, varno)+lp->epsprimal, 1);
        lowbo = upbo - deltaRC;
        deltaRC = lowbo;
        i = GE;  /* Sets the lower bound */
      }

      /* Check and set feasibility status */
      if((isfeasible != NULL) && (upbo - lowbo < -lp->epsprimal))
        *isfeasible = FALSE;

      /* Flag that we can fix the variable by returning the relation code negated */
      else if(fabs(upbo - lowbo) < lp->epsprimal)
        i = -i;
      if(newbound != NULL) {
        my_roundzero(deltaRC, lp->epsprimal);
        *newbound = deltaRC;
      }
    }

  }
  return( i );
}


STATIC MYBOOL findnode_BB(BBrec *BB, int *varno, int *vartype, int *varcus)
{
  int    countsossc, countnint, k;
  REAL   varsol;
  MYBOOL is_better = FALSE, is_equal = FALSE, is_feasible = TRUE;
  lprec  *lp = BB->lp;

  /* Initialize result and return variables */
  *varno    = 0;
  *vartype  = BB_REAL;
  *varcus   = 0;
  countnint = 0;
  BB->nodestatus = lp->spx_status;
  BB->noderesult = lp->solution[0];

  /* If this solution is worse than the best so far, this branch dies.
     If we can only have integer OF values, and we only need the first solution
     then the OF must be at least (unscaled) 1 better than the best so far */
  if((lp->bb_limitlevel != 1) && (MIP_count(lp) > 0)) {

    /* Check that we don't have a limit on the recursion level; two versions supported:
        1) Absolute B&B level (bb_limitlevel > 0), and
        2) B&B level relative to the "B&B order" (bb_limitlevel < 0). */
    countsossc =  lp->sos_vars + lp->sc_vars;
    if((lp->bb_limitlevel > 0) && (lp->bb_level > lp->bb_limitlevel+countsossc))
      return( FALSE );
    else if((lp->bb_limitlevel < 0) &&
            (lp->bb_level > 2*(lp->int_vars+countsossc)*abs(lp->bb_limitlevel))) {
      if(lp->bb_limitlevel == DEF_BB_LIMITLEVEL)
        report(lp, IMPORTANT, "findnode_BB: Default B&B limit reached at %d; optionally change strategy or limit.\n\n",
                              lp->bb_level);
      return( FALSE );
    }

    /* First initialize or update pseudo-costs from previous optimal solution */
    if(BB->varno == 0) {
      varsol = lp->infinite;
      if((lp->int_vars+lp->sc_vars > 0) && (lp->bb_PseudoCost == NULL))
        lp->bb_PseudoCost = init_pseudocost(lp, get_bb_rule(lp));
    }
    else {
      varsol = lp->solution[BB->varno];
      if( ((lp->int_vars > 0) && (BB->vartype == BB_INT)) ||
          ((lp->sc_vars > 0) && (BB->vartype == BB_SC) && !is_int(lp, BB->varno-lp->rows)) )
        update_pseudocost(lp->bb_PseudoCost, BB->varno-lp->rows, BB->vartype, BB->isfloor, varsol);
    }

    /* Make sure we don't have numeric problems (typically due to integer scaling) */
    if((lp->bb_totalnodes > 0) && !bb_better(lp, OF_RELAXED, OF_TEST_WE)) {
      if(lp->bb_trace)
        report(lp, IMPORTANT, "findnode_BB: Simplex failure due to loss of numeric accuracy\n");
      lp->spx_status = NUMFAILURE;
      return( FALSE );
    }

    /* Abandon this branch if the solution is "worse" than a heuristically
      determined limit or the previous best MIP solution */
    if(((lp->solutioncount == 0) && !bb_better(lp, OF_HEURISTIC, OF_TEST_BE)) ||
       ((lp->solutioncount > 0) &&
        (!bb_better(lp, OF_INCUMBENT | OF_DELTA, OF_TEST_BE | OF_TEST_RELGAP) ||
         !bb_better(lp, OF_INCUMBENT | OF_DELTA, OF_TEST_BE)))) {
      return( FALSE );
    }

    /* Collect violated SC variables (since they can also be real-valued); the
       approach is to get them out of the way, since a 0-value is assumed to be "cheap" */
    if(lp->sc_vars > 0) {
      *varno = find_sc_bbvar(lp, &countnint);
      if(*varno > 0)
        *vartype = BB_SC;
    }

    /* Look among SOS variables if no SC candidate was found */
    if((SOS_count(lp) > 0) && (*varno == 0)) {
      *varno = find_sos_bbvar(lp, &countnint, FALSE);
      if(*varno < 0)
        *varno = 0;
      else if(*varno > 0)
        *vartype = BB_SOS;
    }

    /* Then collect INTS that are not integer valued, and verify bounds */
    if((lp->int_vars > 0) && (*varno == 0)) {
      *varno = find_int_bbvar(lp, &countnint, BB, &is_feasible);
      if(*varno > 0) {
        *vartype = BB_INT;
        if((countnint == 1) && !is_feasible) {
          BB->lastrcf = 0;
          return( FALSE );
        }
      }
    }

#if 1 /* peno */
    /* Check if we have reached the depth limit for any individual variable
      (protects against infinite recursions of mainly integer variables) */
    k = *varno-lp->rows;
    if((*varno > 0) && (lp->bb_varactive[k] >= abs(DEF_BB_LIMITLEVEL))) {
      /* if(!is_action(lp->nomessage, NOMSG_BBLIMIT)) {*/
/*
        report(lp, IMPORTANT, "findnode_BB: Reached B&B depth limit %d for variable %d; will not dive further.\n\n",
                              lp->bb_varactive[k], k);
*/
      /*  set_action(&lp->nomessage, NOMSG_BBLIMIT); */
      /* } */
      return( FALSE );
    }
#endif

    /* Check if the current MIP solution is optimal; equal or better */
    if(*varno == 0) {
      is_better = (MYBOOL) (lp->solutioncount == 0) || bb_better(lp, OF_INCUMBENT | OF_DELTA, OF_TEST_BT);
      is_better &= bb_better(lp, OF_INCUMBENT | OF_DELTA, OF_TEST_BT | OF_TEST_RELGAP);
      is_equal  = !is_better;

      if(is_equal) {
        if((lp->solutionlimit <= 0) || (lp->solutioncount < lp->solutionlimit)) {
          lp->solutioncount++;
          SETMIN(lp->bb_solutionlevel, lp->bb_level);
          if((lp->usermessage != NULL) && (lp->msgmask & MSG_MILPEQUAL))
            lp->usermessage(lp, lp->msghandle, MSG_MILPEQUAL);
        }
      }

      /* Current solution is better */
      else if(is_better) {

        /* Update grand total solution count and check if we should go from
           depth-first to best-first variable selection mode */
        if(lp->bb_varactive != NULL) {
          lp->bb_varactive[0]++;
          if((lp->bb_varactive[0] == 1) &&
             is_bb_mode(lp, NODE_DEPTHFIRSTMODE) && is_bb_mode(lp, NODE_DYNAMICMODE))
            lp->bb_rule &= !NODE_DEPTHFIRSTMODE;
        }

        if(lp->bb_trace ||
           ((lp->verbose >= NORMAL) && (lp->print_sol == FALSE) && (lp->lag_status != RUNNING))) {
          report(lp, IMPORTANT,
                 "%s solution " RESULTVALUEMASK " after %10.0f iter, %9.0f nodes (gap %.1f%%)\n",
                 (lp->bb_improvements == 0) ? "Feasible" : "Improved",
                 lp->solution[0], (double) lp->total_iter, (double) lp->bb_totalnodes,
                 100.0*fabs(my_reldiff(lp->solution[0], lp->bb_limitOF)));
        }
        if((lp->usermessage != NULL) && (MIP_count(lp) > 0)) {
          if((lp->msgmask & MSG_MILPFEASIBLE) && (lp->bb_improvements == 0))
            lp->usermessage(lp, lp->msghandle, MSG_MILPFEASIBLE);
          else if((lp->msgmask & MSG_MILPBETTER) && (lp->msgmask & MSG_MILPBETTER))
            lp->usermessage(lp, lp->msghandle, MSG_MILPBETTER);
        }

        lp->bb_status = FEASFOUND;
        lp->bb_solutionlevel = lp->bb_level;
        lp->solutioncount = 1;
        lp->bb_improvements++;
        lp->bb_workOF = lp->rhs[0];

        if(lp->bb_breakfirst ||
           (!is_infinite(lp, lp->bb_breakOF) && bb_better(lp, OF_USERBREAK, OF_TEST_BE)))
          lp->bb_break = TRUE;
      }
    }
  }
  else {
    is_better = TRUE;
    lp->solutioncount = 1;
  }

  /* Transfer the successful solution vector */
  if(is_better || is_equal) {
#ifdef ParanoiaMIP
    if((lp->bb_level > 0) &&
       (check_solution(lp, lp->columns, lp->solution,
                           lp->orig_upbo, lp->orig_lowbo, lp->epssolution) != OPTIMAL)) {
      lp->solutioncount = 0;
      lp->spx_status = NUMFAILURE;
      lp->bb_status = lp->spx_status;
      lp->bb_break = TRUE;
      return( FALSE );
    }
#endif
    transfer_solution(lp, (MYBOOL) ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE));
    if((MIP_count(lp) > 0) && (lp->bb_totalnodes > 0)) {
      if ((!construct_duals(lp)) ||
          (is_presolve(lp, PRESOLVE_SENSDUALS) &&
           (!construct_sensitivity_duals(lp) || !construct_sensitivity_obj(lp))
          )
         ) {
      }
    }
    if(lp->print_sol != FALSE) {
      print_objective(lp);
      print_solution(lp, 1);
    }
  }

  /* Do tracing and determine if we have arrived at the estimated lower MIP limit */
  *varcus = countnint;
  if(MIP_count(lp) > 0) {
    if((countnint == 0) && (lp->solutioncount == 1) && (lp->solutionlimit == 1) &&
       (bb_better(lp, OF_DUALLIMIT, OF_TEST_BE) || bb_better(lp, OF_USERBREAK, OF_TEST_BE | OF_TEST_RELGAP))) {
      lp->bb_break = (MYBOOL) (countnint == 0);
      return( FALSE );
    }
    else if(lp->bb_level > 0) {
#ifdef MIPboundWithOF
      if((lp->constraintOF > 0) && (countnint == 0))
         set_rh(lp, lp->constraintOF, lp->solution[0] + my_chsign(!is_maxim(lp), lp->bb_deltaOF));
#endif
      if(lp->spx_trace)
        report(lp, DETAILED, "B&B level %5d OPT %16s value " RESULTVALUEMASK "\n",
                             lp->bb_level, (*varno) ? "   " : "INT", lp->solution[0]);
    }
    return( (MYBOOL) (*varno > 0));
  }
  else
    return( FALSE );

}

STATIC int solve_BB(BBrec *BB)
{
  int   K, status;
  lprec *lp = BB->lp;

  /* Protect against infinite recursions do to integer rounding effects */
  status = PROCFAIL;

  /* Shortcut variables, set default bounds */
  K = BB->varno;

  /* Load simple MIP bounds */
  if(K > 0) {

    /* Update cuts, if specified */
    updatecuts_BB(lp);

    /* BRANCH_FLOOR: Force the variable to be smaller than the B&B upper bound */
    if(BB->isfloor)
      modifyUndoLadder(lp->bb_upperchange, K, BB->upbo, BB->UPbound);

    /* BRANCH_CEILING: Force the variable to be greater than the B&B lower bound */
    else
      modifyUndoLadder(lp->bb_lowerchange, K, BB->lowbo, BB->LObound);

    /* Update MIP node count */
    BB->nodessolved++;

  }

  /* Solve! */
  status = solve_LP(lp, BB);

  /* Do special feasibility assessment of high order SOS'es */
#if 1
  if((status == OPTIMAL) && (BB->vartype == BB_SOS) && !SOS_is_feasible(lp->SOS, 0, lp->solution))
    status = INFEASIBLE;
#endif

  return( status );
}

/* Routine to compute a "strong" pseudo-cost update for a node */
STATIC MYBOOL strongbranch_BB(lprec *lp, BBrec *BB, int varno, int vartype, int varcus)
{
  MYBOOL   success = FALSE;
  int      i;
  BBrec    *strongBB;

  /* Create new B&B level and solve each of the branches */
  lp->is_strongbranch = TRUE;
  push_basis(lp, lp->var_basic, lp->is_basic, lp->is_lower);
  strongBB = push_BB(lp, BB, lp->rows+varno, vartype, varcus);
  if(strongBB == BB)
    return( success );

  do {

    /* Solve incremental problem to local optimality */
    lp->bb_strongbranches++;
/*    set_action(&lp->spx_action, ACTION_REBASE | ACTION_RECOMPUTE); */
    if(solve_BB(strongBB) == OPTIMAL) {

      /* Update result indicator*/
      success |= 1 << strongBB->isfloor;

      /* Compute new count of non-ints */
      strongBB->lastvarcus = 0;
      for(i = 1; i <= lp->columns; i++) {
        if(is_int(lp, i) && !solution_is_int(lp, lp->rows+i, FALSE))
          strongBB->lastvarcus++;
      }

      /* Perform the pseudo-cost update */
      update_pseudocost(lp->bb_PseudoCost, varno, strongBB->vartype, strongBB->isfloor,
                                           lp->solution[strongBB->varno]);
    }
  }
  while(nextbranch_BB(strongBB));

  strongBB = pop_BB(strongBB);
  if(strongBB != BB)
    report(lp, SEVERE, "strongbranch_BB: Invalid bound settings restored for variable %d\n",
                       varno);
  pop_basis(lp, TRUE);
  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);

  lp->is_strongbranch = FALSE;

  return( success );
}

/* Future functions */
STATIC MYBOOL pre_BB(lprec *lp)
{
  return( TRUE );
}
STATIC MYBOOL post_BB(lprec *lp)
{
  return( TRUE );
}

/* This is the non-recursive B&B driver routine - beautifully simple, yet so subtle! */
STATIC int run_BB(lprec *lp)
{
  BBrec *currentBB;
  int   varno, vartype, varcus, prevsolutions;
  int   status = NOTRUN;

  /* Initialize */
  pre_BB(lp);
  prevsolutions = lp->solutioncount;
#ifdef UseMilpSlacksRCF  /* Check if we should include ranged constraints */
  varno = lp->sum;
#else
  varno = lp->columns;
#endif
  lp->bb_upperchange = createUndoLadder(lp, varno, 2*MIP_count(lp));
  lp->bb_lowerchange = createUndoLadder(lp, varno, 2*MIP_count(lp));
  lp->rootbounds = currentBB = push_BB(lp, NULL, 0, BB_REAL, 0);

  /* Perform the branch & bound loop */
  while(lp->bb_level > 0) {
    status = solve_BB(currentBB);

    if((status == OPTIMAL) && findnode_BB(currentBB, &varno, &vartype, &varcus))
      currentBB = push_BB(lp, currentBB, varno, vartype, varcus);

    else while((lp->bb_level > 0) && !nextbranch_BB(currentBB))
      currentBB = pop_BB(currentBB);

  }

  /* Finalize */
  freeUndoLadder(&(lp->bb_upperchange));
  freeUndoLadder(&(lp->bb_lowerchange));

  /* Check if we should adjust status */
  if(lp->solutioncount > prevsolutions) {
    if((status == PROCBREAK) || (status == USERABORT) || (status == TIMEOUT))
      status = SUBOPTIMAL;
    else
      status = OPTIMAL;
    if(lp->bb_totalnodes > 0)
      lp->spx_status = OPTIMAL;
  }
  post_BB(lp);
  return( status );
}

