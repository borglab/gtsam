
#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_report.h"
#include "lp_pricePSE.h"
#include "lp_price.h"

#if libBLAS > 0
  #include "myblas.h"
#endif

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

/* Simplex pricing utility module - w/interface for lp_solve v5.0+
   -------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h, commonlib.h

    Release notes:
    v1.0.0  1 July 2004         Routines extracted from lp_lib.
    v1.0.1 10 July 2004         Added comparison operators for determination
                                of entering and leaving variables.
                                Added routines for multiple and partial
                                pricing and made corresponding changes to
                                colprim and rowdual.
    v1.0.2 20 August 2004       Implemented relative pivot size control in
                                rowprim and rowdual.
    v1.1.0 15 October 2004      Added dual long step logic.
    v1.1.1 22 October 2004      Added bound sort order to variable selections.
    v1.2.0 24 March 2005        Completed multiple pricing logic.
   ------------------------------------------------------------------------- */


/* Comparison operators for entering and leaving variables for both the primal and
   dual simplexes.  The functions compare a candidate variable with an incumbent. */
int CMP_CALLMODEL compareImprovementVar(const pricerec *current, const pricerec *candidate)
{
  register int   result = COMP_PREFERNONE;
  register lprec *lp = current->lp;
  register REAL  testvalue, margin = PREC_IMPROVEGAP;
  int currentcolno, currentvarno = current->varno,
      candidatecolno, candidatevarno = candidate->varno;
  MYBOOL isdual = candidate->isdual;

  if(isdual) {
    candidatevarno = lp->var_basic[candidatevarno];
    currentvarno   = lp->var_basic[currentvarno];
  }
  candidatecolno = candidatevarno - lp->rows;
  currentcolno   = currentvarno - lp->rows;

  /* Do pivot-based selection unless Bland's (first index) rule is active */
  if(lp->_piv_rule_ != PRICER_FIRSTINDEX) {

    MYBOOL candbetter;

    /* Find the largest value - normalize in case of the dual, since
       constraint violation is expressed as a negative number. */
    /* Use absolute test for "small numbers", relative otherwise */
    testvalue = candidate->pivot;
    if(fabs(testvalue) < LIMIT_ABS_REL)
      testvalue -= current->pivot;
    else
      testvalue = my_reldiff(testvalue, current->pivot);
    if(isdual)
      testvalue = -testvalue;

    candbetter = (MYBOOL) (testvalue > 0);
    if(candbetter) {
      if(testvalue > margin)
        result = COMP_PREFERCANDIDATE;
    }
#if 0 /* Give more opportunity to optimize on non-primary criteria */
    else if (testvalue < -margin)
#else /* Give reduced opportunity to optimize on non-primary criteria */
    else if (testvalue < -lp->epsvalue)
#endif
      result = COMP_PREFERINCUMBENT;

#ifdef UseSortOnBound
      /* Extra selection criterion based on the variable's range;
        variable with - DUAL: small bound out; PRIMAL: large bound in */
    if(result == COMP_PREFERNONE) {
      testvalue = lp->upbo[candidatevarno] - lp->upbo[currentvarno];
      if(testvalue < -margin)
        result = COMP_PREFERINCUMBENT;
      else if(testvalue > margin)
        result = COMP_PREFERCANDIDATE;
      result = my_chsign(isdual, result);
    }
#endif

#ifdef UseSortOnColumnLength
    /* Prevent long columns from entering the basis */
    if(result == COMP_PREFERNONE) {
      if(candidatecolno > 0)
        testvalue = mat_collength(lp->matA, candidatecolno) +
                    (is_obj_in_basis(lp) && (lp->obj[candidatecolno] != 0) ? 1 : 0);
      else
        testvalue = 1;
      if(currentcolno > 0)
        testvalue -= mat_collength(lp->matA, currentcolno) +
                     (is_obj_in_basis(lp) && (lp->obj[currentcolno] != 0) ? 1 : 0);
      else
        testvalue -= 1;
      if(testvalue > 0)
        result = COMP_PREFERINCUMBENT;
      else if(testvalue < 0)
        result = COMP_PREFERCANDIDATE;
      result = my_chsign(isdual, result);
    }
#endif

    /* Select absolute best if the non-primary criteria failed to separate */
    if((result == COMP_PREFERNONE) && candbetter) {
      result = COMP_PREFERCANDIDATE;
      goto Finish;
    }
  }

  /* Final tie-breakers */
  if(result == COMP_PREFERNONE) {

    /* Add randomization tie-braker */
    if(lp->piv_strategy & PRICE_RANDOMIZE) {
      result = my_sign(PRICER_RANDFACT - rand_uniform(lp, 1.0));
      if(candidatevarno < currentvarno)
        result = -result;
    }

    /* Resolve ties via index ordinal */
    if(result == COMP_PREFERNONE) {
      if(candidatevarno < currentvarno)
        result = COMP_PREFERCANDIDATE;
      else /* if(candidatevarno > currentvarno) */
        result = COMP_PREFERINCUMBENT;
      if(lp->_piv_left_)
        result = -result;
    }
  }

Finish:
  return( result );

}

int CMP_CALLMODEL compareSubstitutionVar(const pricerec *current, const pricerec *candidate)
{
  register int    result = COMP_PREFERNONE;
  register lprec  *lp = current->lp;
  register REAL   testvalue = candidate->theta,
                  margin = current->theta;
  MYBOOL isdual = candidate->isdual, candbetter;
  int    currentcolno, currentvarno = current->varno,
         candidatecolno, candidatevarno = candidate->varno;

  if(!isdual) {
    candidatevarno = lp->var_basic[candidatevarno];
    currentvarno   = lp->var_basic[currentvarno];
  }
  candidatecolno = candidatevarno - lp->rows;
  currentcolno   = currentvarno - lp->rows;

  /* Compute the ranking test metric. */
  if(isdual) {
    testvalue = fabs(testvalue);
    margin    = fabs(margin);
  }

  /* Use absolute test for "small numbers", relative otherwise */
  if(fabs(testvalue) < LIMIT_ABS_REL)
    testvalue -= margin;
  else
    testvalue = my_reldiff(testvalue, margin);

  /* Find if the new Theta is smaller or near equal (i.e. testvalue <= eps)
     compared to the previous best; ties will be broken by pivot size or index
     NB! The margin below is essential in maintaining primal/dual feasibility
         during the primal/dual simplex, respectively.  Sometimes a small
         value prevents the selection of a suitable pivot, thereby weakening
         the numerical stability of some models */
  margin = PREC_SUBSTFEASGAP;
  candbetter = (MYBOOL) (testvalue < 0);
  if(candbetter) {
    if(testvalue < -margin)
      result = COMP_PREFERCANDIDATE;
  }
  else if(testvalue > margin)
    result = COMP_PREFERINCUMBENT;

  /* Resolve a tie */
  if(result == COMP_PREFERNONE) {
    REAL currentpivot = fabs(current->pivot),
         candidatepivot = fabs(candidate->pivot);

    /* Handle first index / Bland's rule specially */
    if(lp->_piv_rule_ == PRICER_FIRSTINDEX) {
#if 1
      /* Special secondary selection by pivot size (limited stability protection) */
      margin = candidate->epspivot;
      if((candidatepivot >= margin) && (currentpivot < margin))
        result = COMP_PREFERCANDIDATE;
#endif
    }

    else {

      /* General secondary selection based on pivot size */
#if 0
      if(candidatepivot > MIN_STABLEPIVOT)
        testvalue = my_reldiff(testvalue, currentpivot);
      else
#endif
        testvalue = candidatepivot - currentpivot;
      if(testvalue > margin)
        result = COMP_PREFERCANDIDATE;
      else if(testvalue < -margin)
        result = COMP_PREFERINCUMBENT;

#ifdef UseSortOnBound
      /* Extra selection criterion based on the variable's range;
        variable with - PRIMAL: small bound out; DUAL: large bound in */
      if(result == COMP_PREFERNONE) {
        testvalue = lp->upbo[candidatevarno] - lp->upbo[currentvarno];
        if(testvalue < -margin)
          result = COMP_PREFERCANDIDATE;
        else if(testvalue > margin)
          result = COMP_PREFERINCUMBENT;
        result = my_chsign(isdual, result);
      }
#endif

#ifdef UseSortOnColumnLength
      /* Prevent long columns from entering the basis */
      if(result == COMP_PREFERNONE) {
        if(candidatecolno > 0)
          testvalue = mat_collength(lp->matA, candidatecolno) +
                      (is_obj_in_basis(lp) && (lp->obj[candidatecolno] != 0) ? 1 : 0);
        else
          testvalue = 1;
        if(currentcolno > 0)
          testvalue -= mat_collength(lp->matA, currentcolno) +
                       (is_obj_in_basis(lp) && (lp->obj[currentcolno] != 0) ? 1 : 0);
        else
          testvalue -= 1;
        if(testvalue > 0)
          result = COMP_PREFERCANDIDATE;
        else if(testvalue < 0)
          result = COMP_PREFERINCUMBENT;
        result = my_chsign(isdual, result);
      }
#endif

    }
  }

  /* Select absolute best if the non-primary criteria failed to separate */
  if((result == COMP_PREFERNONE) && candbetter) {
    result = COMP_PREFERCANDIDATE;
    goto Finish;
  }

  /* Final tie-breakers */
  if(result == COMP_PREFERNONE) {

    /* Add randomization tie-braker */
    if(lp->piv_strategy & PRICE_RANDOMIZE) {
      result = my_sign(PRICER_RANDFACT - rand_uniform(lp, 1.0));
      if(candidatevarno < currentvarno)
        result = -result;
    }

    /* Resolve ties via index ordinal (also prefers slacks over user variables) */
    if(result == COMP_PREFERNONE) {
      if(candidatevarno < currentvarno)
        result = COMP_PREFERCANDIDATE;
      else /* if(candidatevarno > currentvarno) */
        result = COMP_PREFERINCUMBENT;
      if(lp->_piv_left_)
        result = -result;
    }
  }

Finish:
  return( result );
}
int CMP_CALLMODEL compareBoundFlipVar(const pricerec *current, const pricerec *candidate)
{
  register REAL  testvalue, margin;
  register int   result = COMP_PREFERNONE;
  register lprec *lp = current->lp;
  MYBOOL    candbetter;
  int currentvarno = current->varno,
      candidatevarno = candidate->varno;

  if(!current->isdual) {
    candidatevarno = lp->var_basic[candidatevarno];
    currentvarno   = lp->var_basic[currentvarno];
  }

  /* Compute the ranking test metric. */
  testvalue = candidate->theta;
  margin    = current->theta;
  if(candidate->isdual) {
    testvalue = fabs(testvalue);
    margin    = fabs(margin);
  }
  if(fabs(margin) < LIMIT_ABS_REL)
    testvalue -= margin;
  else
    testvalue = my_reldiff(testvalue, margin);

  /* Find if the new Theta is smaller or near equal (i.e. testvalue <= eps)
     compared to the previous best; ties will be broken by pivot size or index */
  margin = PREC_SUBSTFEASGAP;
  candbetter = (MYBOOL) (testvalue < 0);
  if(candbetter) {
    if(testvalue < -margin)
      result = COMP_PREFERCANDIDATE;
  }
  else if(testvalue > margin)
    result = COMP_PREFERINCUMBENT;

  /* Resolve a tie */
  if(result == COMP_PREFERNONE) {

    /* Tertiary selection based on priority for large pivot sizes */
    if(result == COMP_PREFERNONE) {
      REAL currentpivot   = fabs(current->pivot),
           candidatepivot = fabs(candidate->pivot);
      if(candidatepivot > currentpivot+margin)
        result = COMP_PREFERCANDIDATE;
      else if(candidatepivot < currentpivot-margin)
        result = COMP_PREFERINCUMBENT;
    }

    /* Secondary selection based on priority for narrow-bounded variables */
    if(result == COMP_PREFERNONE)
      result = compareREAL(&(lp->upbo[currentvarno]),
                           &(lp->upbo[candidatevarno]));

  }

  /* Select absolute best if the non-primary criteria failed to separate */
  if((result == COMP_PREFERNONE) && candbetter) {
    result = COMP_PREFERCANDIDATE;
    goto Finish;
  }

  /* Quaternary selection by index value */
  if(result == COMP_PREFERNONE) {
    if(candidatevarno < currentvarno)
      result = COMP_PREFERCANDIDATE;
    else
      result = COMP_PREFERINCUMBENT;
    if(lp->_piv_left_)
      result = -result;
  }

Finish:
  return( result );
}

/* Validity operators for entering and leaving columns for both the primal and dual
   simplex.  All candidates must satisfy these tests to qualify to be allowed to be
   a subject for the comparison functions/operators. */
STATIC MYBOOL validImprovementVar(pricerec *candidate)
{
  register REAL candidatepivot = fabs(candidate->pivot);

#ifdef Paranoia
  return( (MYBOOL) ((candidate->varno > 0) && (candidatepivot > candidate->lp->epsvalue)) );
#else
  return( (MYBOOL) (candidatepivot > candidate->lp->epsvalue) );
#endif
}

STATIC MYBOOL validSubstitutionVar(pricerec *candidate)
{
  register lprec *lp   = candidate->lp;
  register REAL  theta = (candidate->isdual ? fabs(candidate->theta) : candidate->theta);

#ifdef Paranoia
  if(candidate->varno <= 0)
    return( FALSE );
  else
#endif
  if(fabs(candidate->pivot) >= lp->infinite)
    return( (MYBOOL) (theta < lp->infinite) );
  else
    return( (MYBOOL) ((theta < lp->infinite) &&
                      (fabs(candidate->pivot) >= candidate->epspivot)) );
}

int CMP_CALLMODEL compareImprovementQS(const UNIONTYPE QSORTrec *current, const UNIONTYPE QSORTrec *candidate)
{
  return( compareImprovementVar((pricerec *) current->pvoidint2.ptr, (pricerec *) candidate->pvoidint2.ptr) );
}
int CMP_CALLMODEL compareSubstitutionQS(const UNIONTYPE QSORTrec *current, const UNIONTYPE QSORTrec *candidate)
{
  return( compareBoundFlipVar((pricerec *) current->pvoidint2.ptr, (pricerec *) candidate->pvoidint2.ptr) );
/*  return( compareSubstitutionVar((pricerec *) current->self, (pricerec *) candidate->self) ); */
}

/* Function to add a valid pivot candidate into the specified list */
STATIC int addCandidateVar(pricerec *candidate, multirec *multi, findCompare_func findCompare, MYBOOL allowSortedExpand)
{
  int     insertpos, delta = 1;
  pricerec *targetrec;

  /* Find the insertion point (if any) */
  if((multi->freeList[0] == 0) ||
     (multi->sorted && allowSortedExpand) ||
     (candidate->isdual && (multi->used == 1) && ((multi->step_last >= multi->epszero) ||
                                                  multi_truncatingvar(multi, ((pricerec *) (multi->sortedList[0].pvoidreal.ptr))->varno)))
     ) {
    UNIONTYPE QSORTrec searchTarget;

    /* Make sure that the list is sorted before the search for an insertion point */
    if((multi->freeList[0] == 0) && !multi->sorted) {
      multi->sorted = QS_execute(multi->sortedList, multi->used, findCompare, &insertpos);
      multi->dirty  = (MYBOOL) (insertpos > 0);
    }

    /* Perform the search */
    searchTarget.pvoidint2.ptr = (void *) candidate;
    insertpos = sizeof(searchTarget);
    insertpos = findIndexEx(&searchTarget, multi->sortedList-delta, multi->used, delta, insertpos, findCompare, TRUE);
    if(insertpos > 0)
      return( -1 );
    insertpos = -insertpos - delta;

    /* Check if the candidate is worse than the worst of the list */
    if(((insertpos >= multi->size) && (multi->freeList[0] == 0)) ||
       ((insertpos == multi->used) && (!allowSortedExpand ||
                                       (multi->step_last >= multi->epszero))))
      return( -1 );

#ifdef Paranoia
    /* Do validation */
    if((insertpos < 0) || (insertpos > multi->used))
      return( -1 );
#endif

    /* Define the target for storing the candidate;
       Case 1: List is full and we must discard the previously worst candidate
       Case 2: List is not full and we simply use the next free position */
    if(multi->freeList[0] == 0)
      targetrec = (pricerec *) multi->sortedList[multi->used-1].pvoidreal.ptr;
    else {
      delta = multi->freeList[0]--;
      delta = multi->freeList[delta];
      targetrec = &(multi->items[delta]);
    }
  }
  else {
    delta = multi->freeList[0]--;
    delta = multi->freeList[delta];
    targetrec = &(multi->items[delta]);
    insertpos = multi->used;
  }

  /* Insert the new candidate record in the data store */
  MEMCOPY(targetrec, candidate, 1);

  /* Store the pointer data and handle tree cases:
     Case 1: The list is unsorted and not full; simply append pointer to list,
     Case 2: The list is sorted and full; insert the pointer by discarding previous last,
     Case 3: The list is sorted and not full; shift the inferior items down, and increment count */
  if((multi->used < multi->size) && (insertpos >= multi->used)) {
    QS_append(multi->sortedList, insertpos, targetrec);
    multi->used++;
  }
  else {
    if(multi->used == multi->size)
      QS_insert(multi->sortedList, insertpos, targetrec, multi->size-1); /* Discard previous last */
    else {
      QS_insert(multi->sortedList, insertpos, targetrec, multi->used);   /* Keep previous last    */
      multi->used++;
    }
  }
  multi->active = insertpos;

#ifdef Paranoia
  if((insertpos >= multi->size) || (insertpos >= multi->used))
    report(multi->lp, SEVERE, "addCandidateVar: Insertion point beyond limit!\n");
#endif

  return( insertpos );
}

STATIC MYBOOL findImprovementVar(pricerec *current, pricerec *candidate, MYBOOL collectMP, int *candidatecount)
/* PRIMAL: Find a variable to enter the basis
   DUAL:   Find a variable to leave the basis

   Allowed variable set: Any pivot PRIMAL:larger or DUAL:smaller than threshold value of 0 */
{
  MYBOOL Action = FALSE,
#ifdef ExtractedValidityTest
         Accept = TRUE;
#else    /* Check for validity and compare result with previous best */
         Accept = validImprovementVar(candidate);
#endif
  if(Accept) {
    if(candidatecount != NULL)
      (*candidatecount)++;
    if(collectMP) {
      if(addCandidateVar(candidate, current->lp->multivars, (findCompare_func *) compareImprovementQS, FALSE) < 0)
        return(Action);
    }
    if(current->varno > 0)
      Accept = (MYBOOL) (compareImprovementVar(current, candidate) > 0);
  }

 /* Apply candidate if accepted */
  if(Accept) {
    (*current) = *candidate;

    /* Force immediate acceptance for Bland's rule using the primal simplex */
    if(!candidate->isdual)
      Action = (MYBOOL) (candidate->lp->_piv_rule_ == PRICER_FIRSTINDEX);
  }
  return(Action);
}

/* Bound flip variable accumulation routine */
STATIC MYBOOL collectMinorVar(pricerec *candidate, multirec *longsteps, MYBOOL isphase2, MYBOOL isbatch)
{
  int   inspos;

  /* 1. Check for ratio and pivot validity (to have the extra flexibility that all
        bound-flip candidates are also possible as basis-entering variables */
  if(!validSubstitutionVar(candidate))
    return( FALSE );

  /* 2. If the free-list is empty we need to see if we have a better candidate,
        and for this the candidate list has to be sorted by merit */
  if(!isbatch &&
     !longsteps->sorted && (longsteps->used > 1) &&
     ((longsteps->freeList[0] == 0) ||
      multi_truncatingvar(longsteps, candidate->varno) ||
      (longsteps->step_last >= longsteps->epszero) )) {
    longsteps->sorted = QS_execute(longsteps->sortedList, longsteps->used,
                                   (findCompare_func *) compareSubstitutionQS, &inspos);
    longsteps->dirty  = (MYBOOL) (inspos > 0);
    if(longsteps->dirty)
      multi_recompute(longsteps, 0, isphase2, TRUE);
  }

  /* 3. Now handle three cases...
        - Add to the list when the list is not full and there is opportunity for improvement,
        - Check if we should replace an incumbent when the list is full,
        - Check if we should replace an incumbent when the list is not full, there is no room
          for improvement, but the current candidate is better than an incumbent. */
  inspos = addCandidateVar(candidate, longsteps, (findCompare_func *) compareSubstitutionQS, TRUE);

  /* 4. Recompute steps and objective, and (if relevant) determine if we
        may be suboptimal in relation to an incumbent MILP solution. */
  return( (MYBOOL) (inspos >= 0) &&
           ((isbatch == TRUE) || multi_recompute(longsteps, inspos, isphase2, TRUE)) );
}

STATIC MYBOOL findSubstitutionVar(pricerec *current, pricerec *candidate, int *candidatecount)
/* PRIMAL: Find a variable to leave the basis
   DUAL:   Find a variable to enter the basis

   Allowed variable set: "Equal-valued" smallest thetas! */
{
  MYBOOL Action = FALSE,
#ifdef ExtractedValidityTest
         Accept = TRUE;
#else  /* Check for validity and comparison result with previous best */
         Accept = validSubstitutionVar(candidate);
#endif
  if(Accept) {
    if(candidatecount != NULL)
      (*candidatecount)++;
    if(current->varno != 0)
      Accept = (MYBOOL) (compareSubstitutionVar(current, candidate) > 0);
  }

 /* Apply candidate if accepted */
  if(Accept) {
    (*current) = *candidate;

    /* Force immediate acceptance for Bland's rule using the dual simplex */
#ifdef ForceEarlyBlandRule
    if(candidate->isdual)
      Action = (MYBOOL) (candidate->lp->_piv_rule_ == PRICER_FIRSTINDEX);
#endif
  }
  return(Action);
}

/* Partial pricing management routines */
STATIC partialrec *partial_createBlocks(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = (partialrec *) calloc(1, sizeof(*blockdata));
  blockdata->lp = lp;
  blockdata->blockcount = 1;
  blockdata->blocknow = 1;
  blockdata->isrow = isrow;

  return(blockdata);
}
STATIC int partial_countBlocks(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata = IF(isrow, lp->rowblocks, lp->colblocks);

  if(blockdata == NULL)
    return( 1 );
  else
    return( blockdata->blockcount );
}
STATIC int partial_activeBlocks(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata = IF(isrow, lp->rowblocks, lp->colblocks);

  if(blockdata == NULL)
    return( 1 );
  else
    return( blockdata->blocknow );
}
STATIC void partial_freeBlocks(partialrec **blockdata)
{
  if((blockdata == NULL) || (*blockdata == NULL))
    return;
  FREE((*blockdata)->blockend);
  FREE((*blockdata)->blockpos);
  FREE(*blockdata);
}


/* Function to provide for left-right or right-left scanning of entering/leaving
   variables; note that *end must have been initialized by the calling routine! */
STATIC void makePriceLoop(lprec *lp, int *start, int *end, int *delta)
{
  int offset = is_piv_mode(lp, PRICE_LOOPLEFT);

  if((offset) ||
     (((lp->total_iter+offset) % 2 == 0) && is_piv_mode(lp, PRICE_LOOPALTERNATE))) {
    *delta = -1; /* Step backwards - "left" */
    swapINT(start, end);
    lp->_piv_left_ = TRUE;
  }
  else {
    *delta = 1;  /* Step forwards - "right" */
    lp->_piv_left_ = FALSE;
  }
}

/* Routine to verify accuracy of the current basis factorization */
STATIC MYBOOL serious_facterror(lprec *lp, REAL *bvector, int maxcols, REAL tolerance)
{
  int    i, j, ib, ie, nz, nc;
  REAL   sum, tsum = 0, err = 0;
  MATrec *mat = lp->matA;

  if(bvector == 0)
    bvector = lp->bsolveVal;
  nc =0;
  nz = 0;
  for(i = 1; (i <= lp->rows) && (nc <= maxcols); i++) {

    /* Do we have a non-slack variable? (we choose to skip slacks,
      since they have "natural" good accuracy properties) */
    j = lp->var_basic[i] - lp->rows;
    if(j <= 0)
      continue;
    nc++;

    /* Compute cross product for basic, non-slack column */
    ib = mat->col_end[j-1];
    ie = mat->col_end[j];
    nz += ie - ib;
    sum = get_OF_active(lp, j+lp->rows, bvector[0]);
    for(; ib < ie; ib++)
      sum += COL_MAT_VALUE(ib)*bvector[COL_MAT_ROWNR(ib)];

    /* Catch high precision early, so we don't to uneccessary work */
    tsum += sum;
    SETMAX(err, fabs(sum));
    if((tsum / nc > tolerance / 100) && (err < tolerance / 100))
      break;
  }
  err /= mat->infnorm;
  return( (MYBOOL) (err >= tolerance) );
}

/* Computation of reduced costs */
STATIC void update_reducedcosts(lprec *lp, MYBOOL isdual, int leave_nr, int enter_nr, REAL *prow, REAL *drow)
{
  /* "Fast" update of the dual reduced cost vector; note that it must be called
     after the pivot operation and only applies to a major "true" iteration */
  int  i;
  REAL hold;

  if(isdual) {
    hold = -drow[enter_nr]/prow[enter_nr];
    for(i=1; i <= lp->sum; i++)
      if(!lp->is_basic[i]) {
        if(i == leave_nr)
          drow[i] = hold;
        else {
          drow[i] += hold*prow[i];
          my_roundzero(drow[i], lp->epsmachine);
        }
      }
  }
  else
    report(lp, SEVERE, "update_reducedcosts: Cannot update primal reduced costs!\n");
}


STATIC void compute_reducedcosts(lprec *lp, MYBOOL isdual, int row_nr, int *coltarget, MYBOOL dosolve,
                                            REAL *prow, int *nzprow,
                                            REAL *drow, int *nzdrow,
                                            int roundmode)
{
  REAL epsvalue = lp->epsvalue;  /* Any larger value can produce a suboptimal result */
  roundmode |=  MAT_ROUNDRC;

  if(isdual) {
    bsolve_xA2(lp, coltarget,
                   row_nr, prow, epsvalue, nzprow,  /* Calculate net sensitivity given a leaving variable */
                        0, drow, epsvalue, nzdrow,  /* Calculate the net objective function values */
                   roundmode);
  }
  else {
    REAL *bVector;

#if 1 /* Legacy mode, that is possibly a little faster */
    if((lp->multivars == NULL) && (lp->P1extraDim == 0))
      bVector = drow;
    else
#endif
      bVector = lp->bsolveVal;
    if(dosolve) {
      bsolve(lp, 0, bVector, lp->bsolveIdx, epsvalue*DOUBLEROUND, 1.0);
      if(!isdual && (row_nr == 0) && (lp->improve & IMPROVE_SOLUTION) && !refactRecent(lp) &&
         serious_facterror(lp, bVector, lp->rows, lp->epsvalue))
        set_action(&lp->spx_action, ACTION_REINVERT);
    }
    prod_xA(lp,   coltarget,
                  bVector, lp->bsolveIdx, epsvalue, 1.0,
                  drow, nzdrow, roundmode);
  }
}


/* Primal: Prevent acceptance of an entering variable when the magnitude of
           other candidates is also very small.
   Dual:   Prevent acceptance of a leaving variable when the magnitude of
           other candidates is also very small.

   Both of these cases are associated with numerical stalling, which we could
   argue should be detected and handled by the stalling monitor routine. */
STATIC MYBOOL verify_stability(lprec *lp, MYBOOL isprimal, REAL xfeas, REAL sfeas, int nfeas)
{
  MYBOOL testOK = TRUE;
  return( testOK );

#if 1
  /* Try to make dual feasibility as tight as possible */
  if(!isprimal)
/*  if(lp->P1extraVal == 0) */
  {
    xfeas /= (1+lp->rhsmax);
    sfeas /= (1+lp->rhsmax);
  }
#endif
  xfeas = fabs(xfeas);             /* Maximum (positive) infeasibility */
/*  if(xfeas < lp->epspivot) { */
  if(xfeas < lp->epssolution) {
    REAL f;
    sfeas = fabs(sfeas);           /* Make sum of infeasibilities positive */
    xfeas = (sfeas-xfeas)/nfeas;   /* Average "residual" feasibility */
    f = 1 + log10((REAL) nfeas);   /* Some numerical complexity scalar */
    /* Numerical errors can interact to cause non-convergence, and the
      idea is to relax the tolerance to account for this and only
      marginally weakening the (user-specified) tolerance. */
    if((sfeas-xfeas) < f*lp->epsprimal)
      testOK = FALSE;
  }
  return( testOK );
}


/* Find an entering column for the case that the specified basic variable
   is fixed or zero - typically used for artificial variable elimination */
STATIC int find_rowReplacement(lprec *lp, int rownr, REAL *prow, int *nzprow)
/* The logic in this section generally follows Chvatal: Linear Programming, p. 130
   Basically, the function is a specialized coldual(). */
{
  int  i, bestindex;
  REAL bestvalue;

 /* Solve for "local reduced cost" */
  set_action(&lp->piv_strategy, PRICE_FORCEFULL);
    compute_reducedcosts(lp, TRUE, rownr, NULL, TRUE,
                             prow, nzprow, NULL, NULL, MAT_ROUNDDEFAULT);
  clear_action(&lp->piv_strategy, PRICE_FORCEFULL);

 /* Find a suitably non-singular variable to enter ("most orthogonal") */
  bestindex = 0;
  bestvalue = 0;
  for(i = 1; i <= lp->sum-abs(lp->P1extraDim); i++) {
    if(!lp->is_basic[i] && !is_fixedvar(lp, i) &&
      (fabs(prow[i]) > bestvalue)) {
      bestindex = i;
      bestvalue = fabs(prow[i]);
    }
  }

  /* Prepare to update inverse and pivot/iterate (compute Bw=a) */
  if(i > lp->sum-abs(lp->P1extraDim))
    bestindex = 0;
  else
    fsolve(lp, bestindex, prow, nzprow, lp->epsmachine, 1.0, TRUE);

  return( bestindex );
}

/* Find the primal simplex entering non-basic column variable */
STATIC int colprim(lprec *lp, REAL *drow, int *nzdrow, MYBOOL skipupdate, int partialloop, int *candidatecount, MYBOOL updateinfeas, REAL *xviol)
{
  int      i, ix, iy, iz, ninfeas, nloop = 0;
  REAL     f, sinfeas, xinfeas, epsvalue = lp->epsdual;
  pricerec current, candidate;
  MYBOOL   collectMP = FALSE;
  int      *coltarget = NULL;

  /* Identify pivot column according to pricing strategy; set
     entering variable initial threshold reduced cost value to "0" */
  current.pivot    = lp->epsprimal;    /* Minimum acceptable improvement */
  current.varno    = 0;
  current.lp       = lp;
  current.isdual   = FALSE;
  candidate.lp     = lp;
  candidate.isdual = FALSE;
  *candidatecount  = 0;

  /* Update local value of pivot setting and determine active multiple pricing set */
  lp->_piv_rule_ = get_piv_rule(lp);
doLoop:
  nloop++;
  if((lp->multivars != NULL) && ((lp->simplex_mode & SIMPLEX_PRIMAL_PRIMAL) != 0)) {
    collectMP = multi_mustupdate(lp->multivars);
    if(collectMP) {
      multi_restart(lp->multivars);
      coltarget = NULL;
    }
    else
      coltarget = multi_indexSet(lp->multivars, FALSE);
  }

  /* Compute reduced costs c - c*Inv(B), if necessary
     (i.e. the previous iteration was not a "minor" iteration/bound flip) */
  if(!skipupdate) {
#ifdef UsePrimalReducedCostUpdate
    /* Recompute from scratch only at the beginning, otherwise update */
    if((lp->current_iter > 0) && (refactRecent(lp) == AUTOMATIC))
#endif
    compute_reducedcosts(lp, FALSE, 0, coltarget, (MYBOOL) ((nloop <= 1) || (partialloop > 1)),
                             NULL, NULL,
                             drow, nzdrow,
                             MAT_ROUNDDEFAULT);
  }

  /* Loop over active partial column set; we presume that reduced costs
     have only been updated for columns in the active partial range. */
  ix = 1;
  iy = nzdrow[0];
  ninfeas = 0;
  xinfeas = 0;
  sinfeas = 0;
  makePriceLoop(lp, &ix, &iy, &iz);
  iy *= iz;
  for(; ix*iz <= iy; ix += iz) {
    i = nzdrow[ix];
#if 0 /* Not necessary since we masked them out in compute_reducedcosts() */
    if(i > lp->sum-abs(lp->P1extraDim))
      continue;
#endif

    /* Check if the pivot candidate is on the block-list */
    if(lp->rejectpivot[0] > 0) {
      int kk;
      for(kk = 1; (kk <= lp->rejectpivot[0]) && (i != lp->rejectpivot[kk]); kk++);
      if(kk <= lp->rejectpivot[0])
        continue;
    }

   /* Retrieve the applicable reduced cost - threshold should not be smaller than 0 */
    f = my_chsign(lp->is_lower[i], drow[i]);
    if(f <= epsvalue)
      continue;

   /* Find entering variable according to strategy (largest positive f) */
    ninfeas++;
    SETMAX(xinfeas, f);
    sinfeas += f;
    candidate.pivot = normalizeEdge(lp, i, f, FALSE);
    candidate.varno = i;
    if(findImprovementVar(&current, &candidate, collectMP, candidatecount))
      break;
  }

  /* Check if we should loop again after a multiple pricing update */
  if(lp->multivars != NULL) {
    if(collectMP) {
      if(!lp->multivars->sorted)
        lp->multivars->sorted = QS_execute(lp->multivars->sortedList, lp->multivars->used,
                                           (findCompare_func *) compareImprovementQS, NULL);
      coltarget = multi_indexSet(lp->multivars, TRUE);
    }
    else if((current.varno == 0) && (lp->multivars->retries == 0)) {
      ix = partial_blockStart(lp, FALSE);
      iy = partial_blockEnd(lp, FALSE);
      lp->multivars->used = 0;
      lp->multivars->retries++;
      goto doLoop;
    }
    /* Shrink the candidate list */
    lp->multivars->retries = 0;
    if(current.varno != 0)
      multi_removevar(lp->multivars, current.varno);
  }

  /* Check for optimality */
  if(xviol != NULL)
    *xviol = xinfeas;
  if(updateinfeas)
    lp->suminfeas = fabs(sinfeas);
  if((lp->multivars == NULL) && (current.varno > 0) &&
     !verify_stability(lp, TRUE, xinfeas, sinfeas, ninfeas))
    current.varno = 0;

  /* Produce statistics */
  if(lp->spx_trace) {
    if(current.varno > 0)
      report(lp, DETAILED, "colprim: Column %d reduced cost = " RESULTVALUEMASK "\n",
                          current.varno, current.pivot);
    else
      report(lp, DETAILED, "colprim: No positive reduced costs found, optimality!\n");
  }

  return( current.varno );
} /* colprim */

/* Find the primal simplex leaving basic column variable */
STATIC int rowprim(lprec *lp, int colnr, LREAL *theta, REAL *pcol, int *nzpcol, MYBOOL forceoutEQ, REAL *xviol)
{
  int      i, ii, iy, iz, Hpass, k, *nzlist;
  LREAL    f, savef;
  REAL     Heps, Htheta, Hlimit, epsvalue, epspivot, p;
  pricerec current, candidate;
  MYBOOL   isupper = !lp->is_lower[colnr], HarrisTwoPass = FALSE;

  /* Update local value of pivot setting */
  lp->_piv_rule_ = get_piv_rule(lp);
  if(nzpcol == NULL)
    nzlist = (int *) mempool_obtainVector(lp->workarrays, lp->rows+1, sizeof(*nzlist));
  else
    nzlist = nzpcol;

  /* Find unconditional non-zeros and optionally compute relative size of epspivot */
  epspivot = lp->epspivot;
  epsvalue = lp->epsvalue;
  Hlimit = 0;
  Htheta = 0;
  k = 0;
  for(i = 1; i <= lp->rows; i++) {
    p = fabs(pcol[i]);
    if(p > Hlimit)
      Hlimit = p;
    if(p > epsvalue) {
      k++;
      nzlist[k] = i;
      SETMAX(Htheta, p);
    }
#ifdef Paranoia
    else {
      if(lp->spx_trace)
        report(lp, FULL, "rowprim: Row %d with pivot " RESULTVALUEMASK " rejected as too small\n",
                         i, p);
    }
#endif
  }
  if(xviol != NULL)
    *xviol = Htheta;
  Htheta = 0;

  /* Update non-zero list based on the new pivot threshold */
#ifdef UseRelativePivot_Primal
/*  epspivot *= sqrt(lp->matA->dynrange) / lp->matA->infnorm; */
  epspivot /= MAX(1, sqrt(lp->matA->colmax[colnr]));
  iy = k;
  k = 0;
  p = 0;
  for(ii = 1; ii <= iy; ii++) {
    i = nzlist[ii];
    p = fabs(pcol[i]);

    /* Compress the list of valid alternatives, if appropriate */
    if(p > epspivot) {
      k++;
      nzlist[k] = i;
    }
#ifdef Paranoia
    else {
      if(lp->spx_trace)
        report(lp, FULL, "rowprim: Row %d with pivot " RESULTVALUEMASK " rejected as too small\n",
                         i, p);
    }
#endif
  }
#endif

  /* Initialize counters */
  nzlist[0] = k;
  k = 0;

Retry:
  k++;
  HarrisTwoPass = is_piv_mode(lp, PRICE_HARRISTWOPASS);
  if(HarrisTwoPass)
    Hpass = 1;
  else
    Hpass = 2;
  current.theta    = lp->infinite;
  current.pivot    = 0;
  current.varno    = 0;
  current.isdual   = FALSE;
  current.epspivot = epspivot;
  current.lp       = lp;
  candidate.epspivot = epspivot;
  candidate.isdual = FALSE;
  candidate.lp     = lp;
  savef  = 0;
  for(; Hpass <= 2; Hpass++) {
    Htheta = lp->infinite;
    if(Hpass == 1) {
      Hlimit = lp->infinite;           /* Don't apply any limit in the first pass */
      Heps   = epspivot/lp->epsprimal; /* Scaled to lp->epsprimal used in compute_theta() */
    }
    else {
      Hlimit = current.theta;          /* This is the smallest Theta of the first pass */
      Heps   = 0.0;
    }
    current.theta = lp->infinite;
    current.pivot = 0;
    current.varno = 0;
    savef = 0;

    ii = 1;
    iy = nzlist[0];
    makePriceLoop(lp, &ii, &iy, &iz);
    iy *= iz;
    for(; ii*iz <= iy; ii += iz) {
      i = nzlist[ii];
      f = pcol[i];
      candidate.theta = f;
      candidate.pivot = f;
      candidate.varno = i;

      /*i =*/ compute_theta(lp, i, &candidate.theta, isupper,
                            my_if(lp->upbo[lp->var_basic[i]] < lp->epsprimal, Heps/10, Heps), TRUE);

      if(fabs(candidate.theta) >= lp->infinite) {
        savef = f;
        candidate.theta = 2*lp->infinite;
        continue;
      }

      /* Find the candidate leaving variable according to strategy (smallest theta) */
      if((Hpass == 2) && (candidate.theta > Hlimit))
        continue;

      /* Give a slight preference to fixed variables (mainly equality slacks) */
      if(forceoutEQ) {
        p = candidate.pivot;
        if(lp->upbo[lp->var_basic[i]] < lp->epsprimal) {
          /* Give an extra early boost to equality slack elimination, if specified */
          if(forceoutEQ == AUTOMATIC)
            candidate.pivot *= 1.0+lp->epspivot;
          else
            candidate.pivot *= 10.0;

        }
      }
      if(HarrisTwoPass) {
        f = candidate.theta;
        if(Hpass == 2)
          candidate.theta = 1;
        if(findSubstitutionVar(&current, &candidate, NULL))
          break;
        if((Hpass == 2) && (current.varno == candidate.varno))
          Htheta = f;
      }
      else
        if(findSubstitutionVar(&current, &candidate, NULL))
          break;
      /* Restore temporarily modified pivot */
      if(forceoutEQ && (current.varno == candidate.varno))
        current.pivot = p;
    }
  }
  if(HarrisTwoPass)
    current.theta = Htheta;

  /* Handle case of no available leaving variable */
  if(current.varno == 0) {
    if(lp->upbo[colnr] >= lp->infinite) {
      /* Optionally try again with reduced pivot threshold level */
      if(k < 2) {
        epspivot = epspivot / 10;
        goto Retry;
      }
    }
    else {
#if 1
      i = 1;
      while((pcol[i] >= 0) && (i <= lp->rows))
        i++;
      if(i > lp->rows) { /* Empty column with upper bound! */
        lp->is_lower[colnr] = !lp->is_lower[colnr];
/*        lp->is_lower[colnr] = FALSE; */
        lp->rhs[0] += lp->upbo[colnr]*pcol[0];
      }
      else /* if(pcol[i]<0) */
      {
        current.varno = i;
      }
#endif
    }
  }
  else if(current.theta >= lp->infinite) {
    report(lp, IMPORTANT, "rowprim: Numeric instability pcol[%d] = %g, rhs[%d] = %g, upbo = %g\n",
                          current.varno, savef, current.varno, lp->rhs[current.varno],
                          lp->upbo[lp->var_basic[current.varno]]);
  }

 /* Return working array to pool */
  if(nzpcol == NULL)
    mempool_releaseVector(lp->workarrays, (char *) nzlist, FALSE);

  if(lp->spx_trace)
    report(lp, DETAILED, "row_prim: %d, pivot size = " RESULTVALUEMASK "\n",
                         current.varno, current.pivot);

/*  *theta = current.theta; */
  *theta = fabs(current.theta);

  return(current.varno);
} /* rowprim */


/* Find the dual simplex leaving basic variable */
STATIC int rowdual(lprec *lp, REAL *rhvec, MYBOOL forceoutEQ, MYBOOL updateinfeas, REAL *xviol)
{
  int       k, i, iy, iz, ii, ninfeas;
  register REAL     rh;
  REAL      up, lo = 0,
            epsvalue, sinfeas, xinfeas;
  pricerec  current, candidate;
  MYBOOL    collectMP = FALSE;

  /* Initialize */
  if(rhvec == NULL)
    rhvec = lp->rhs;
  epsvalue = lp->epsdual;
  current.pivot    = -epsvalue;  /* Initialize leaving variable threshold; "less than 0" */
  current.theta    = 0;
  current.varno    = 0;
  current.isdual   = TRUE;
  current.lp       = lp;
  candidate.isdual = TRUE;
  candidate.lp     = lp;

  /* Loop over active partial row set */
  if(is_action(lp->piv_strategy, PRICE_FORCEFULL)) {
    k  = 1;
    iy = lp->rows;
  }
  else {
    k = partial_blockStart(lp, TRUE);
    iy = partial_blockEnd(lp, TRUE);
  }
  ninfeas = 0;
  xinfeas = 0;
  sinfeas = 0;
  makePriceLoop(lp, &k, &iy, &iz);
  iy *= iz;
  for(; k*iz <= iy; k += iz) {

    /* Map loop variable to target */
    i = k;

    /* Check if the pivot candidate is on the block-list */
    if(lp->rejectpivot[0] > 0) {
      int kk;
      for(kk = 1; (kk <= lp->rejectpivot[0]) && (i != lp->rejectpivot[kk]); kk++);
      if(kk <= lp->rejectpivot[0])
        continue;
    }

    /* Set local variables - express violation as a negative number */
    ii = lp->var_basic[i];
    up = lp->upbo[ii];
    lo = 0;
    rh = rhvec[i];
    if(rh > up)
      rh = up - rh;
    else
      rh -= lo;
    up -= lo;

   /* Analyze relevant constraints ...
      KE version skips uninteresting alternatives and gives a noticeable speedup */
/*    if((rh < -epsvalue*sqrt(lp->matA->rowmax[i])) || */
    if((rh < -epsvalue) ||
       ((forceoutEQ == TRUE) && (up < epsvalue))) {  /* It causes instability to remove the "TRUE" test */

     /* Accumulate stats */
      ninfeas++;
      SETMIN(xinfeas, rh);
      sinfeas += rh;

     /* Give a slight preference to fixed variables (mainly equality slacks) */
      if(up < epsvalue) {
        /* Break out immediately if we are directed to force slacks out of the basis */
        if(forceoutEQ == TRUE) {
          current.varno = i;
          current.pivot = -1;
          break;
        }
        /* Give an extra early boost to equality slack elimination, if specified */
        if(forceoutEQ == AUTOMATIC)
          rh *= 10.0;
        else /* .. or just the normal. marginal boost */
          rh *= 1.0+lp->epspivot;
      }

     /* Select leaving variable according to strategy (the most negative/largest violation) */
      candidate.pivot = normalizeEdge(lp, i, rh, TRUE);
      candidate.varno = i;
      if(findImprovementVar(&current, &candidate, collectMP, NULL))
        break;
    }
  }

  /* Verify infeasibility */
  if(updateinfeas)
    lp->suminfeas = fabs(sinfeas);
  if((ninfeas > 1) &&
     !verify_stability(lp, FALSE, xinfeas, sinfeas, ninfeas)) {
    report(lp, IMPORTANT, "rowdual: Check for reduced accuracy and tolerance settings.\n");
    current.varno = 0;
  }

  /* Produce statistics */
  if(lp->spx_trace) {
    report(lp, NORMAL, "rowdual: Infeasibility sum " RESULTVALUEMASK " in %7d constraints.\n",
                        sinfeas, ninfeas);
    if(current.varno > 0) {
      report(lp, DETAILED, "rowdual: rhs[%d] = " RESULTVALUEMASK "\n",
                           current.varno, lp->rhs[current.varno]);
    }
    else
      report(lp, FULL, "rowdual: Optimality - No primal infeasibilities found\n");
  }
  if(xviol != NULL)
    *xviol = fabs(xinfeas);

  return(current.varno);
} /* rowdual */


STATIC void longdual_testset(lprec *lp, int which, int rownr, REAL *prow, int *nzprow,
                                                    REAL *drow, int *nzdrow)
{
  int i,j;
  REAL F = lp->infinite;
  if(which == 0) {             /* Maros Example-1 - raw data */
    j =  1; i = lp->rows+j; lp->upbo[i] = 0;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  2; drow[i] = -1;
    j =  2; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -2; drow[i] =  2;
    j =  3; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  1; drow[i] =  5;
    j =  4; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] =  3; drow[i] = -6;
    j =  5; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -4; drow[i] = -2;
    j =  6; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -1; drow[i] =  0;
    j =  7; i = lp->rows+j; lp->upbo[i] = 2;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] =  1; drow[i] =  0;
    j =  8; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -2; drow[i] =  0;
    j =  9; i = lp->rows+j; lp->upbo[i] = 5;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -1; drow[i] =  4;
    j = 10; i = lp->rows+j; lp->upbo[i] = F;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -2; drow[i] = 10;
    nzprow[0] = i-lp->rows;
    lp->rhs[rownr] = -11;
    lp->upbo[lp->var_basic[rownr]] = F;
    lp->rhs[0] = 1;
  }
  else if(which == 1) {       /* Maros Example-1 - presorted in correct order */
    j =  1; i = lp->rows+j; lp->upbo[i] = 0;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  2; drow[i] = -1;
    j =  2; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  1; drow[i] =  5;
    j =  3; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -4; drow[i] = -2;
    j =  4; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -2; drow[i] =  0;

    j =  5; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -1; drow[i] =  0;
    j =  6; i = lp->rows+j; lp->upbo[i] = 2;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] =  1; drow[i] =  0;
    j =  7; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -2; drow[i] =  2;
    j =  8; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] =  3; drow[i] = -6;
    j =  9; i = lp->rows+j; lp->upbo[i] = 5;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -1; drow[i] =  4;
    j = 10; i = lp->rows+j; lp->upbo[i] = F;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -2; drow[i] = 10;
    nzprow[0] = i-lp->rows;
    lp->rhs[rownr] = -11;
    lp->upbo[lp->var_basic[rownr]] = F;
    lp->rhs[0] = 1;
  }

  else if(which == 10) {       /* Maros Example-2 - raw data */
    j =  1; i = lp->rows+j; lp->upbo[i] = 5;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] = -2; drow[i] =  2;
    j =  2; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  3; drow[i] =  3;
    j =  3; i = lp->rows+j; lp->upbo[i] = 1;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -2; drow[i] =  0;
    j =  4; i = lp->rows+j; lp->upbo[i] = 2;  lp->is_lower[i] = FALSE; nzprow[j] = i; prow[i] = -1; drow[i] = -2;
    j =  5; i = lp->rows+j; lp->upbo[i] = 2;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  1; drow[i] =  0;
    j =  6; i = lp->rows+j; lp->upbo[i] = F;  lp->is_lower[i] =  TRUE; nzprow[j] = i; prow[i] =  3; drow[i] =  9;
    nzprow[0] = i-lp->rows;
    lp->rhs[rownr] = 14;
    lp->upbo[lp->var_basic[rownr]] = 2;
    lp->rhs[0] = 6;
  }
}


/* Find the dual simplex entering non-basic variable */
STATIC int coldual(lprec *lp, int row_nr, REAL *prow, int *nzprow,
                                          REAL *drow, int *nzdrow,
                                          MYBOOL dualphase1, MYBOOL skipupdate,
                                          int *candidatecount, REAL *xviol)
{
  int      i, iy, iz, ix, k, nbound;
  LREAL    w, g, quot;
  REAL     viol, p, epspivot = lp->epspivot;
#ifdef MachinePrecRoundRHS
  REAL     epsvalue = lp->epsmachine;
#else
  REAL     epsvalue = lp->epsvalue;
#endif
  pricerec current, candidate;
  MYBOOL   isbatch = FALSE, /* Requires that lp->longsteps->size > lp->sum */
           dolongsteps = (MYBOOL) (lp->longsteps != NULL);

  /* Initialize */
  if(xviol != NULL)
    *xviol = lp->infinite;
  if(dolongsteps && !dualphase1)
    dolongsteps = AUTOMATIC;  /* Sets Phase1 = TRUE, Phase2 = AUTOMATIC */
  current.theta    = lp->infinite;
  current.pivot    = 0;
  current.varno    = 0;
  current.epspivot = epspivot;
  current.isdual   = TRUE;
  current.lp       = lp;
  candidate.epspivot = epspivot;
  candidate.isdual = TRUE;
  candidate.lp     = lp;
  *candidatecount  = 0;

  /* Compute reduced costs */
  if(!skipupdate) {
#ifdef UseDualReducedCostUpdate
    /* Recompute from scratch only at the beginning, otherwise update */
    if((lp->current_iter > 0) && (refactRecent(lp) < AUTOMATIC))
      compute_reducedcosts(lp, TRUE, row_nr, NULL, TRUE,
                               prow, nzprow,
                               NULL, NULL,
                               MAT_ROUNDDEFAULT);
    else
#endif
      compute_reducedcosts(lp, TRUE, row_nr, NULL, TRUE,
                               prow, nzprow,
                               drow, nzdrow,
                               MAT_ROUNDDEFAULT);
  }

#if 0
  /* Override all above to do in-line testing with fixed test set */
  if(lp->rows > 1 && lp->columns > 10)
    longdual_testset(lp, 10, row_nr, prow, nzprow, drow, nzdrow);
#endif

  /* Compute the current violation of the bounds of the outgoing variable,
     negative for violation of lower bound, positive for upper bound violation.
     (Basic variables are always lower-bounded, by lp_solve convention) */
  g = 1;
  viol = lp->rhs[row_nr];
  if(viol > 0) {   /* Check if the leaving variable is >= its upper bound */
    p = lp->upbo[lp->var_basic[row_nr]];
    if(p < lp->infinite) {
      viol -= p;
      my_roundzero(viol, epsvalue);
      if(viol > 0)
        g = -1;
    }
    /* Do validation of numerics */
    if(g == 1) {
      if(viol >= lp->infinite) {
        report(lp, IMPORTANT, "coldual: Large basic solution value %g at iter %.0f indicates numerical instability\n",
                               lp->rhs[row_nr], (double) get_total_iter(lp));
        lp->spx_status = NUMFAILURE;
        return( 0 );

      }
      if(skipupdate)
        report(lp, DETAILED, "coldual: Inaccurate bound-flip accuracy at iter %.0f\n",
                              (double) get_total_iter(lp));
      else
        report(lp, SEVERE,   "coldual: Leaving variable %d does not violate bounds at iter %.0f\n",
                              row_nr, (double) get_total_iter(lp));
      return( -1 );
    }
  }

  /* Update local value of pivot setting */
  lp->_piv_rule_ = get_piv_rule(lp);

  /* Condense list of relevant targets */
  p = 0;
  k = 0;
  nbound = 0;
  ix = 1;
  iy = nzprow[0];
  for(ix = 1; ix <= iy; ix++) {
    i = nzprow[ix];
    w = prow[i] * g;            /* Change sign if upper bound of the leaving variable is violated   */
    w *= 2*lp->is_lower[i] - 1; /* Change sign if the non-basic variable is currently upper-bounded */

    /* Check if the candidate is worth using for anything */
    if(w < -epsvalue) {
      /* Tally bounded variables */
      if(lp->upbo[i] < lp->infinite)
        nbound++;

      /* Update the nz-index */
      k++;
      nzprow[k] = i;
      SETMAX(p, -w);
    }
#ifdef Paranoia
    else {
      if(lp->spx_trace) {
        report(lp, FULL, "coldual: Candidate variable prow[%d] rejected with %g too small\n",
                         i, w);
      }
    }
#endif

  }
  nzprow[0] = k;
  if(xviol != NULL)
    *xviol = p;

#ifdef UseRelativePivot_Dual
/*  epspivot *= sqrt(lp->matA->dynrange) / lp->matA->infnorm; */
  epspivot /= MAX(1, sqrt(lp->matA->rowmax[row_nr]));
#endif
  current.epspivot   = epspivot;
  candidate.epspivot = epspivot;

  /* Initialize the long-step structures if indicated */
  if(dolongsteps) {
    if((nzprow[0] <= 1) || (nbound == 0)) {  /* Don't bother */
      dolongsteps = FALSE;
      lp->longsteps->indexSet[0] = 0;
    }
    else {
      multi_restart(lp->longsteps);
      multi_valueInit(lp->longsteps, g*viol, lp->rhs[0]);
    }
  }

  /* Loop over all entering column candidates */
  ix = 1;
  iy = nzprow[0];
  makePriceLoop(lp, &ix, &iy, &iz);
  iy *= iz;
  for(; ix*iz <= iy; ix += iz) {
    i = nzprow[ix];

    /* Compute the dual ratio (prow = w and drow = cbar in Chvatal's "nomenclatura") */
    w    = prow[i] * g;         /* Change sign if upper bound of the leaving variable is violated   */
    quot = -drow[i] / w;        /* Remember this sign-reversal in multi_recompute!                  */

    /* Apply the selected pivot strategy (smallest theta) */
    candidate.theta = quot;  /* Note that abs() is applied in findSubstitutionVar */
    candidate.pivot = w;
    candidate.varno = i;

    /* Collect candidates for minor iterations/bound flips */
    if(dolongsteps) {
      if(isbatch && (ix == iy))
        isbatch = AUTOMATIC;
      if(collectMinorVar(&candidate, lp->longsteps, (MYBOOL) (dolongsteps == AUTOMATIC), isbatch) &&
         lp->spx_trace)
        report(lp, DETAILED, "coldual: Long-dual break point with %d bound-flip variables\n",
                             lp->longsteps->used);
      if(lp->spx_status == FATHOMED)
        return( 0 );
    }

    /* We have a candidate for entering the basis; check if it is better than the incumbent */
    else if(findSubstitutionVar(&current, &candidate, candidatecount))
      break;
  }

  /* Set entering variable and long-step bound swap variables */
  if(dolongsteps) {
    *candidatecount = lp->longsteps->used;
    i = multi_enteringvar(lp->longsteps, NULL, 3);
  }
  else
    i = current.varno;

  if(lp->spx_trace)
    report(lp, NORMAL, "coldual: Entering column %d, reduced cost %g, pivot value %g, bound swaps %d\n",
                       i, drow[i], prow[i], multi_used(lp->longsteps));

  return( i );
} /* coldual */


INLINE REAL normalizeEdge(lprec *lp, int item, REAL edge, MYBOOL isdual)
{
#if 1
  /* Don't use the pricer "close to home", since this can possibly
    worsen the final feasibility picture (mainly a Devex issue?) */
  if(fabs(edge) > lp->epssolution)
#endif
    edge /= getPricer(lp, item, isdual);
  if((lp->piv_strategy & PRICE_RANDOMIZE) != 0)
    edge *= (1.0-PRICER_RANDFACT) + PRICER_RANDFACT*rand_uniform(lp, 1.0);
  return( edge );

}

/* Support routines for block detection and partial pricing */
STATIC int partial_findBlocks(lprec *lp, MYBOOL autodefine, MYBOOL isrow)
{
  int    i, jj, n, nb, ne, items;
  REAL   hold, biggest, *sum = NULL;
  MATrec *mat = lp->matA;
  partialrec *blockdata;

  if(!mat_validate(mat))
    return( 1 );

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
  items     = IF(isrow, lp->rows, lp->columns);
  allocREAL(lp, &sum, items+1, FALSE);

  /* Loop over items and compute the average column index for each */
  sum[0] = 0;
  for(i = 1; i <= items; i++) {
    n = 0;
    if(isrow) {
      nb = mat->row_end[i-1];
      ne = mat->row_end[i];
    }
    else {
      nb = mat->col_end[i-1];
      ne = mat->col_end[i];
    }
    n = ne-nb;
    sum[i] = 0;
    if(n > 0) {
      if(isrow)
        for(jj = nb; jj < ne; jj++)
          sum[i] += ROW_MAT_COLNR(jj);
      else
        for(jj = nb; jj < ne; jj++)
          sum[i] += COL_MAT_ROWNR(jj);
      sum[i] /= n;
    }
    else
      sum[i] = sum[i-1];
  }

  /* Loop over items again, find largest difference and make monotone */
  hold = 0;
  biggest = 0;
  for(i = 2; i <= items; i++) {
    hold = sum[i] - sum[i-1];
    if(hold > 0) {
      if(hold > biggest)
        biggest = hold;
    }
    else
      hold = 0;
    sum[i-1] = hold;
  }

  /* Loop over items again and find differences exceeding threshold;
     the discriminatory power of this routine depends strongly on the
     magnitude of the scaling factor - from empirical evidence > 0.9 */
  biggest = MAX(1, 0.9*biggest);
  n = 0;
  nb = 0;
  ne = 0;
  for(i = 1; i < items; i++)
    if(sum[i] > biggest) {
      ne += i-nb;        /* Compute sum of index gaps between maxima */
      nb = i;
      n++;               /* Increment count */
    }

  /* Clean up */
  FREE(sum);

  /* Require that the maxima are spread "nicely" across the columns,
     otherwise return that there is only one monolithic block.
     (This is probably an area for improvement in the logic!) */
  if(n > 0) {
    ne /= n;                 /* Average index gap between maxima */
    i = IF(isrow, lp->columns, lp->rows);
    nb = i / ne;             /* Another estimated block count */
    if(abs(nb - n) > 2)      /* Probably Ok to require equality (nb==n)*/
      n = 1;
    else if(autodefine)      /* Generate row/column break-indeces for partial pricing */
      set_partialprice(lp, nb, NULL, isrow);
  }
  else
    n = 1;

  return( n );
}
STATIC int partial_blockStart(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
  if(blockdata == NULL)
    return( 1 );
  else {
    if((blockdata->blocknow < 1) || (blockdata->blocknow > blockdata->blockcount))
      blockdata->blocknow = 1;
    return( blockdata->blockend[blockdata->blocknow-1] );
  }
}
STATIC int partial_blockEnd(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
  if(blockdata == NULL)
    return( IF(isrow, lp->rows, lp->sum) );
  else {
    if((blockdata->blocknow < 1) || (blockdata->blocknow > blockdata->blockcount))
      blockdata->blocknow = 1;
    return( blockdata->blockend[blockdata->blocknow]-1 );
  }
}
STATIC int partial_blockNextPos(lprec *lp, int block, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
#ifdef Paranoia
  if((blockdata == NULL) || (block <= 1) || (block > blockdata->blockcount)) {
    report(lp, SEVERE, "partial_blockNextPos: Invalid block %d specified.\n",
                       block);
    return( -1 );
  }
#endif
  block--;
  if(blockdata->blockpos[block] == blockdata->blockend[block+1])
    blockdata->blockpos[block] = blockdata->blockend[block];
  else
    blockdata->blockpos[block]++;
  return( blockdata->blockpos[block] );
}
STATIC MYBOOL partial_blockStep(lprec *lp, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
  if(blockdata == NULL)
    return( FALSE );
  else if(blockdata->blocknow < blockdata->blockcount) {
    blockdata->blocknow++;
    return( TRUE);
  }
  else {
    blockdata->blocknow = 1;
    return( TRUE );
  }
}
STATIC MYBOOL partial_isVarActive(lprec *lp, int varno, MYBOOL isrow)
{
  partialrec *blockdata;

  blockdata = IF(isrow, lp->rowblocks, lp->colblocks);
  if(blockdata == NULL)
    return( TRUE );
  else {
    return( (MYBOOL) ((varno >= blockdata->blockend[blockdata->blocknow-1]) &&
                      (varno < blockdata->blockend[blockdata->blocknow])) );
  }
}


/* Multiple pricing routines */
STATIC multirec *multi_create(lprec *lp, MYBOOL truncinf)
{
  multirec *multi;

  multi = (multirec *) calloc(1, sizeof(*multi));
  if(multi != NULL) {
    multi->active = 1;
    multi->lp = lp;
    multi->epszero = lp->epsprimal;
    multi->truncinf = truncinf;
  }

  return(multi);
}
STATIC void multi_free(multirec **multi)
{
  if((multi == NULL) || (*multi == NULL))
    return;
  FREE((*multi)->items);
  FREE((*multi)->valueList);
  FREE((*multi)->indexSet);
  FREE((*multi)->freeList);
  FREE((*multi)->sortedList);
  FREE(*multi);
}
STATIC MYBOOL multi_mustupdate(multirec *multi)
{
  return( (MYBOOL) ((multi != NULL) &&
                     (multi->used < multi->limit)) );
}
STATIC MYBOOL multi_resize(multirec *multi, int blocksize, int blockdiv, MYBOOL doVlist, MYBOOL doIset)
{
  MYBOOL ok = TRUE;

  if((blocksize > 1) && (blockdiv > 0)) {
    int oldsize = multi->size;

    multi->size = blocksize;
    if(blockdiv > 1)
      multi->limit += (multi->size-oldsize) / blockdiv;

    multi->items = (pricerec *) realloc(multi->items, (multi->size+1)*sizeof(*(multi->items)));
    multi->sortedList = (UNIONTYPE QSORTrec *) realloc(multi->sortedList, (multi->size+1)*sizeof(*(multi->sortedList)));
    ok = (multi->items != NULL) && (multi->sortedList != NULL) &&
         allocINT(multi->lp, &(multi->freeList), multi->size+1, AUTOMATIC);
    if(ok) {
      int i, n;

      if(oldsize == 0)
        i = 0;
      else
        i = multi->freeList[0];
      multi->freeList[0] = i + (multi->size-oldsize);
      for(n = multi->size - 1, i++; i <= multi->freeList[0]; i++, n--)
        multi->freeList[i] = n;
    }
    if(doVlist)
      ok &= allocREAL(multi->lp, &(multi->valueList), multi->size+1, AUTOMATIC);
    if(doIset) {
      ok &= allocINT(multi->lp, &(multi->indexSet), multi->size+1, AUTOMATIC);
      if(ok && (oldsize == 0))
        multi->indexSet[0] = 0;
    }
    if(!ok)
      goto Undo;

  }
  else {
Undo:
    multi->size = 0;
    FREE(multi->items);
    FREE(multi->valueList);
    FREE(multi->indexSet);
    FREE(multi->freeList);
    FREE(multi->sortedList);
  }
  multi->active = 1;

  return( ok );
}

STATIC int multi_size(multirec *multi)
{
  if(multi == NULL)
    return( 0 );
  else
    return( multi->size );
}

STATIC int multi_used(multirec *multi)
{
  if(multi == NULL)
    return( 0 );
  else
    return( multi->used );
}

STATIC int multi_restart(multirec *multi)
{
  int i, n = multi->used;

  multi->used   = 0;
  multi->sorted = FALSE;
  multi->dirty  = FALSE;
  if(multi->freeList != NULL) {
    for(i = 1; i <= multi->size; i++)
      multi->freeList[i] = multi->size - i;
    multi->freeList[0] = multi->size;
  }
#if 0
  if(multi->indexSet != NULL)
    multi->indexSet[0] = 0;
#endif
  return( n );
}

STATIC void multi_valueInit(multirec *multi, REAL step_base, REAL obj_base)
{
  multi->step_base = multi->step_last = step_base;
  multi->obj_base  = multi->obj_last  = obj_base;
#ifdef Paranoia
  if(step_base > 0)
    report(multi->lp, SEVERE, "multi_valueInit: Positive constraint violation %g provided at iteration %6.0f\n",
                              step_base, (double) get_total_iter(multi->lp));
#endif
}

STATIC REAL *multi_valueList(multirec *multi)
{
  return(multi->valueList);
}

STATIC int *multi_indexSet(multirec *multi, MYBOOL regenerate)
{
  if(regenerate)
    multi_populateSet(multi, NULL, -1);
  return(multi->indexSet);
}

STATIC int multi_getvar(multirec *multi, int item)
{
#ifdef Paranoia
  if((item < 1) || (item >= multi->size))
    return(-1);
#endif
  return( ((pricerec *) &(multi->sortedList[item].pvoidreal.ptr))->varno );
}

STATIC MYBOOL multi_recompute(multirec *multi, int index, MYBOOL isphase2, MYBOOL fullupdate)
{
  int      i, n;
  REAL     lB, uB, Alpha, this_theta, prev_theta;
  lprec    *lp = multi->lp;
  pricerec *thisprice;

  /* Define target update window */
  if(multi->dirty) {
    index = 0;
    n = multi->used - 1;
  }
  else if(fullupdate)
    n = multi->used - 1;
  else
    n = index;

  /* Initialize accumulators from the specified update index */
  if(index == 0) {
    multi->maxpivot = 0;
    multi->maxbound = 0;
    multi->step_last = multi->step_base;
    multi->obj_last  = multi->obj_base;
    thisprice  = NULL;
    this_theta  = 0;
  }
  else {
    multi->obj_last  = multi->valueList[index-1];
    multi->step_last = multi->sortedList[index-1].pvoidreal.realval;
    thisprice  = (pricerec *) (multi->sortedList[index-1].pvoidreal.ptr);
    this_theta = thisprice->theta;
  }

  /* Update step lengths and objective values */
  while((index <= n) && (multi->step_last < multi->epszero)) {

    /* Update parameters for this loop */
    prev_theta = this_theta;
    thisprice  = (pricerec *) (multi->sortedList[index].pvoidreal.ptr);
    this_theta = thisprice->theta;
    Alpha = fabs(thisprice->pivot);
    uB = lp->upbo[thisprice->varno];
    lB = 0;
    SETMAX(multi->maxpivot, Alpha);
    SETMAX(multi->maxbound, uB);

    /* Do the value updates */
    if(isphase2) {
      multi->obj_last += (this_theta - prev_theta) * multi->step_last; /* Sign-readjusted from coldual()/Maros */
      if(uB >= lp->infinite)
        multi->step_last  = lp->infinite;
      else
        multi->step_last += Alpha*(uB-lB);
    }
    else {
      multi->obj_last += (this_theta - prev_theta) * multi->step_last; /* Sign-readjusted from coldual()/Maros */
      multi->step_last += Alpha;
    }

    /* Store updated values at the indexed locations */
    multi->sortedList[index].pvoidreal.realval = multi->step_last;
    multi->valueList[index] = multi->obj_last;
#ifdef Paranoia
    if(lp->spx_trace &&
       (multi->step_last > lp->infinite))
      report(lp, SEVERE, "multi_recompute: A very large step-size %g was generated at iteration %6.0f\n",
                         multi->step_last, (double) get_total_iter(lp));
#endif
    index++;
  }

  /* Discard candidates entered earlier that now make the OF worsen, and
     make sure that the released positions are added to the free list. */
  n = index;
  while(n < multi->used) {
    i = ++multi->freeList[0];
    multi->freeList[i] = ((pricerec *) multi->sortedList[n].pvoidreal.ptr) - multi->items;
    n++;
  }
  multi->used  = index;
  if(multi->sorted && (index == 1))
    multi->sorted = FALSE;
  multi->dirty = FALSE;

  /* Return TRUE if the step is now positive */
  return( (MYBOOL) (multi->step_last >= multi->epszero) );
}

STATIC MYBOOL multi_truncatingvar(multirec *multi, int varnr)
{
  return( multi->truncinf && is_infinite(multi->lp, multi->lp->upbo[varnr]) );
}

STATIC MYBOOL multi_removevar(multirec *multi, int varnr)
{
  int i = 1;
  int *coltarget = multi->indexSet;

  if(coltarget == NULL)
    return( FALSE );

  while((i <= multi->used) && (coltarget[i] != varnr))
    i++;
  if(i > multi->used)
    return( FALSE );

  for(; i < multi->used; i++)
    coltarget[i] = coltarget[i+1];
  coltarget[0]--;
  multi->used--;
  multi->dirty = TRUE;
  return( TRUE );
}

STATIC int multi_enteringvar(multirec *multi, pricerec *current, int priority)
{
  lprec    *lp = multi->lp;
  int      i, bestindex, colnr;
  REAL     bound, score, bestscore = -lp->infinite;
  REAL     b1, b2, b3;
  pricerec *candidate, *bestcand;

  /* Check that we have a candidate */
  multi->active = bestindex = 0;
  if((multi == NULL) || (multi->used == 0))
    return( bestindex );

  /* Check for pruning possibility of the B&B tree */
  if(multi->objcheck && (lp->solutioncount > 0) &&
     bb_better(lp, OF_WORKING | OF_PROJECTED, OF_TEST_WE)) {
    lp->spx_status = FATHOMED;
    return( bestindex );
  }

  /* Check the trivial case */
  if(multi->used == 1) {
    bestcand = (pricerec *) (multi->sortedList[bestindex].pvoidreal.ptr);
    goto Finish;
  }

  /* Set priority weights */
Redo:
  switch(priority) {
    case 0:  b1 = 0.0, b2 = 0.0, b3 = 1.0;          /* Only OF          */
              bestindex = multi->used - 2;   break;
    case 1:  b1 = 0.2, b2 = 0.3, b3 = 0.5; break;  /* Emphasize OF     */
    case 2:  b1 = 0.3, b2 = 0.5, b3 = 0.2; break;  /* Emphasize bound  */
    case 3:  b1 = 0.6, b2 = 0.2, b3 = 0.2; break;  /* Emphasize pivot  */
    case 4:  b1 = 1.0, b2 = 0.0, b3 = 0.0; break;  /* Only pivot       */
    default: b1 = 0.4, b2 = 0.2, b3 = 0.4;         /* Balanced default */
  }
  bestcand = (pricerec *) (multi->sortedList[bestindex].pvoidreal.ptr);

  /* Loop over all candidates to get the best entering candidate;
     start at the end to try to maximize the chain length */
  for(i = multi->used - 1; i >= 0; i--) {
    candidate = (pricerec *) (multi->sortedList[i].pvoidreal.ptr);
    colnr = candidate->varno;
    bound = lp->upbo[colnr];
    score = fabs(candidate->pivot) / multi->maxpivot;
    score = pow(1.0 + score                           , b1) *
            pow(1.0 + log(bound / multi->maxbound + 1), b2) *
            pow(1.0 + (REAL) i / multi->used          , b3);
    if(score > bestscore) {
      bestscore = score;
      bestindex = i;
      bestcand  = candidate;
    }
  }

  /* Do pivot protection */
  if((priority < 4) && (fabs(bestcand->pivot) < lp->epssolution)) {
    bestindex = 0;
    priority++;
    goto Redo;
  }

Finish:
  /* Make sure we shrink the list and update */
  multi->active = colnr = bestcand->varno;
  if(bestindex < multi->used - 1) {
#if 0
/*    if(lp->upbo[colnr] >= lp->infinite) */
    QS_swap(multi->sortedList, bestindex, multi->used-1);
    multi_recompute(multi, bestindex, (bestcand->isdual == AUTOMATIC), TRUE);
#else
    multi->used = i + 1;
#endif
  }
  multi_populateSet(multi, NULL, multi->active);

  /* Compute the entering theta and update parameters */
  score = (multi->used == 1 ? multi->step_base : multi->sortedList[multi->used-2].pvoidreal.realval);
  score /= bestcand->pivot;
  score = my_chsign(!lp->is_lower[multi->active], score);

  if(lp->spx_trace &&
     (fabs(score) > 1/lp->epsprimal))
    report(lp, IMPORTANT, "multi_enteringvar: A very large Theta %g was generated (pivot %g)\n",
                       score, bestcand->pivot);
  multi->step_base = score;
  if(current != NULL)
    *current = *bestcand;

  return( multi->active );
}

STATIC REAL multi_enteringtheta(multirec *multi)
{
  return( multi->step_base );
}

STATIC int multi_populateSet(multirec *multi, int **list, int excludenr)
{
  int n = 0;
  if(list == NULL)
    list = &(multi->indexSet);
  if((multi->used > 0) &&
     ((*list != NULL) || allocINT(multi->lp, list, multi->size+1, FALSE))) {
    int i, colnr;

    for(i = 0; i < multi->used; i++) {
      colnr = ((pricerec *) (multi->sortedList[i].pvoidreal.ptr))->varno;
      if((colnr != excludenr) &&
        /* Prevent an unbounded variable from "bound-flip"; this could
          actually indicate that we should let the entering variable be
          bound-swapped (in the case that it is bounded), but we
          disregard this possibility here, since it brings with it
          issues of pivot size, etc. */
        ((excludenr > 0) && (multi->lp->upbo[colnr] < multi->lp->infinite))) {
        n++;
        (*list)[n] = colnr;
      }
    }
    (*list)[0] = n;
  }
  return( n );
}

