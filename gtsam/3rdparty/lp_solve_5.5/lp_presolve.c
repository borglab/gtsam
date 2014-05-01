
/* -------------------------------------------------------------------------
   Presolve routines for lp_solve v5.0+
   -------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h, lp_presolve, lp_crash.h, lp_scale.h, lp_report.h

    Release notes:
    v1.0.0  1 January 2003      Initial crude version used with lp_solve v4.
    v5.0.0  1 January 2004      Significantly expanded and repackaged
                                presolve routines for lp_solve v5 release.
    v5.0.1  1 April   2004      Added reference to new crash module
    v5.1.0  20 August 2004      Reworked infeasibility detection.
                                Added encapsulation of presolve undo logic.
    v5.1.1  10 September 2004   Added variable bound tightening based on
                                full-constraint information, as well as
                                variable fixing by duality.
    v5.2.0  1 January 2005      Fixes to bound fixing handling.
                                Added fast batch compression after presolve.
                                Restructured calls by adding presolve wrapper.
                                Major optimization of identification logic
                                  along with bug fixes.
                                Enabled storage of eliminated matrix data.
                                Added function to report on constraint classes.
    v5.5.0  1 June 2005         Added implied slack presolve, restructured
                                looping logic to be more modular, and made
                                active row/column selection logic faster.
    v5.5.1  18 June 2005        Finished sparsity-enhancing logic and added
                                initial version of column aggregation code.
   ------------------------------------------------------------------------- */

#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_presolve.h"
#include "lp_crash.h"
#include "lp_scale.h"
#include "lp_report.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


#define presolve_setstatus(one, two)  presolve_setstatusex(one, two, __LINE__, __FILE__)
STATIC int presolve_setstatusex(presolverec *psdata, int status, int lineno, char *filename)
{
  if((status == INFEASIBLE) || (status == UNBOUNDED)) {
    report(psdata->lp,
#ifdef Paranoia
           NORMAL,
#else
           DETAILED,
#endif
           "presolve_setstatus: Status set to '%s' on code line %d, file '%s'\n",
           (status == INFEASIBLE ? "INFEASIBLE" : "UNBOUNDED"), lineno, (filename == NULL ? "Unknown" : filename));
  }
  return( status );
}

STATIC MYBOOL presolve_statuscheck(presolverec *psdata, int *status)
{
  if(*status == RUNNING) {
    lprec *lp = psdata->lp;
    if(!mat_validate(lp->matA))
      *status = MATRIXERROR;
    else if(userabort(lp, -1))
      *status = lp->spx_status;
  }
  return( (MYBOOL) (*status == RUNNING) );
}

STATIC MYBOOL presolve_createUndo(lprec *lp)
{
  if(lp->presolve_undo != NULL)
    presolve_freeUndo(lp);
  lp->presolve_undo = (presolveundorec *) calloc(1, sizeof(presolveundorec));
  lp->presolve_undo->lp = lp;
  if(lp->presolve_undo == NULL)
    return( FALSE );
  return( TRUE );
}
STATIC MYBOOL inc_presolve_space(lprec *lp, int delta, MYBOOL isrows)
{
  int i, ii,
      oldrowcolalloc, rowcolsum, oldrowalloc, oldcolalloc;
  presolveundorec *psundo = lp->presolve_undo;

  if(psundo == NULL) {
    presolve_createUndo(lp);
    psundo = lp->presolve_undo;
  }

  /* Set constants */
  oldrowalloc = lp->rows_alloc-delta;
  oldcolalloc = lp->columns_alloc-delta;
  oldrowcolalloc = lp->sum_alloc-delta;
  rowcolsum = lp->sum_alloc + 1;

  /* Reallocate lp memory */
  if(isrows)
    allocREAL(lp, &psundo->fixed_rhs,   lp->rows_alloc+1, AUTOMATIC);
  else
    allocREAL(lp, &psundo->fixed_obj,   lp->columns_alloc+1, AUTOMATIC);
  allocINT(lp,  &psundo->var_to_orig, rowcolsum, AUTOMATIC);
  allocINT(lp,  &psundo->orig_to_var, rowcolsum, AUTOMATIC);

  /* Fill in default values, where appropriate */
  if(isrows)
    ii = oldrowalloc+1;
  else
    ii = oldcolalloc+1;
  for(i = oldrowcolalloc+1; i < rowcolsum; i++, ii++) {
    psundo->var_to_orig[i] = 0;
    psundo->orig_to_var[i] = 0;
    if(isrows)
      psundo->fixed_rhs[ii] = 0;
    else
      psundo->fixed_obj[ii] = 0;
  }

  return(TRUE);
}
STATIC MYBOOL presolve_setOrig(lprec *lp, int orig_rows, int orig_cols)
{
  presolveundorec *psundo = lp->presolve_undo;

  if(psundo == NULL)
    return( FALSE );
  psundo->orig_rows = orig_rows;
  psundo->orig_columns = orig_cols;
  psundo->orig_sum = orig_rows + orig_cols;
  if(lp->wasPresolved)
    presolve_fillUndo(lp, orig_rows, orig_cols, FALSE);
  return( TRUE );
}
STATIC MYBOOL presolve_fillUndo(lprec *lp, int orig_rows, int orig_cols, MYBOOL setOrig)
{
  int i;
  presolveundorec *psundo = lp->presolve_undo;

  for(i = 0; i <= orig_rows; i++) {
    psundo->var_to_orig[i] = i;
    psundo->orig_to_var[i] = i;
    psundo->fixed_rhs[i]   = 0;
  }
  for(i = 1; i <= orig_cols; i++) {
    psundo->var_to_orig[orig_rows + i] = i;
    psundo->orig_to_var[orig_rows + i] = i;
    psundo->fixed_obj[i] = 0;
  }
  if(setOrig)
    presolve_setOrig(lp, orig_rows, orig_cols);

  return( TRUE );
}
STATIC MYBOOL presolve_rebuildUndo(lprec *lp, MYBOOL isprimal)
{
  int             ik, ie, ix, j, k, *colnrDep;
  REAL             hold, *value, *solution, *slacks;
  presolveundorec *psdata = lp->presolve_undo;
  MATrec          *mat = NULL;

  /* Point to and initialize undo structure at first call */
  if(isprimal) {
    if(psdata->primalundo != NULL)
      mat = psdata->primalundo->tracker;
    solution = lp->full_solution + lp->presolve_undo->orig_rows;
    slacks   = lp->full_solution;
  }
  else {
    if(psdata->dualundo != NULL)
      mat = psdata->dualundo->tracker;
    solution = lp->full_duals;
    slacks   = lp->full_duals + lp->presolve_undo->orig_rows;
  }
  if(mat == NULL)
    return( FALSE );

  /* Loop backward over the undo chain */
  for(j = mat->col_tag[0]; j > 0; j--) {
    ix = mat->col_tag[j];
    ik = mat->col_end[j-1];
    ie = mat->col_end[j];
    colnrDep = &COL_MAT_ROWNR(ik);
    value    = &COL_MAT_VALUE(ik);
    hold = 0;
    k = 0;
    for(; ik < ie; ik++, colnrDep += matRowColStep, value += matValueStep) {

      /* Constant term */
      if(*colnrDep == 0)
        hold += *value;

      /* Special case with dependence on a slack variable */
      else if(isprimal && (*colnrDep > lp->presolve_undo->orig_columns)) {
        k = (*colnrDep) - lp->presolve_undo->orig_columns;
        hold -= (*value) * slacks[k];
        slacks[k] = 0;
      }
      else if(!isprimal && (*colnrDep > lp->presolve_undo->orig_rows)) {
        k = (*colnrDep) - lp->presolve_undo->orig_rows;
        hold -= (*value) * slacks[k];
        slacks[k] = 0;
      }

      /* Dependence on other user variable */
      else
        hold -= (*value) * solution[*colnrDep];

      *value = 0;
    }
    if(fabs(hold) > lp->epsvalue)
      solution[ix] = hold;
  }

  return( TRUE );
}
STATIC MYBOOL presolve_freeUndo(lprec *lp)
{
  presolveundorec *psundo = lp->presolve_undo;

  if(psundo == NULL)
    return( FALSE );
  FREE(psundo->orig_to_var);
  FREE(psundo->var_to_orig);
  FREE(psundo->fixed_rhs);
  FREE(psundo->fixed_obj);
  if(psundo->deletedA != NULL)
    freeUndoLadder(&(psundo->deletedA));
  if(psundo->primalundo != NULL)
    freeUndoLadder(&(psundo->primalundo));
  if(psundo->dualundo != NULL)
    freeUndoLadder(&(psundo->dualundo));
  FREE(lp->presolve_undo);
  return( TRUE );
}

STATIC void presolve_storeDualUndo(presolverec *psdata, int rownr, int colnr)
{
  lprec    *lp = psdata->lp;
  MYBOOL   firstdone = FALSE;
  int      ix, iix, item;
  REAL     Aij = get_mat(lp, rownr, colnr);
  MATrec   *mat = lp->matA;

  if(presolve_collength(psdata, colnr) == 0)
    return;

  /* Add undo information for the dual of the deleted constraint */
  item = 0;
  for(ix = presolve_nextrow(psdata, colnr, &item); ix >= 0;
      ix = presolve_nextrow(psdata, colnr, &item)) {
    iix = COL_MAT_ROWNR(ix);
    if(iix == rownr)
      continue;
    if(!firstdone)
      firstdone = addUndoPresolve(lp, FALSE, rownr, get_mat(lp, 0, colnr)/Aij,
                                                    get_mat_byindex(lp, ix, FALSE, TRUE)/Aij, iix);
    else
      appendUndoPresolve(lp, FALSE, get_mat_byindex(lp, ix, FALSE, TRUE)/Aij, iix);
  }
}

/* ----------------------------------------------------------------------------- */
/* Presolve debugging routines                                                   */
/* ----------------------------------------------------------------------------- */
STATIC MYBOOL presolve_SOScheck(presolverec *psdata)
{
  MYBOOL status = TRUE;
  lprec  *lp = psdata->lp;
  int    *list, i, j, n, k, nk, colnr, nSOS = SOS_count(lp), nerr = 0;
  SOSrec *SOS;

  if(nSOS == 0)
    return( status );

  /* For each SOS and each member check validity */
  for(i = 1; i<= nSOS; i++) {
    SOS = lp->SOS->sos_list[i-1];
    list = SOS->members;
    n = list[0];
    for(j = 1; j<= n; j++) {
      colnr = list[j];
      /* Check valid range */
      if((colnr < 1) || (colnr > lp->columns)) {
        nerr++;
        report(lp, IMPORTANT, "presolve_SOScheck: A - Column index %d is outside of valid range\n",
                              colnr);
      }
      /* Check for deletion */
      if(!isActiveLink(psdata->cols->varmap, colnr)) {
        nerr++;
        report(lp, IMPORTANT, "presolve_SOScheck: B - Column index %d has been marked for deletion\n",
                              colnr);
      }
      /* Check if sorted member array is Ok */
      if(SOS_member_index(lp->SOS, i, colnr) != j) {
        nerr++;
        report(lp, IMPORTANT, "presolve_SOScheck: C - Column index %d not found in fast search array\n",
                              colnr);
      }
      /* Check for variable membership in this SOS record of the sparse storage */
      k = lp->SOS->memberpos[colnr-1];
      nk = lp->SOS->memberpos[colnr];
      while((k < nk) && (lp->SOS->membership[k] != i))
        k++;
      if(k >= nk) {
        nerr++;
        report(lp, IMPORTANT, "presolve_SOScheck: D - Column index %d was not found in sparse array\n",
                              colnr);
      }
    }
  }

  /* Check that all members in the sparse array can be validated as SOS members */
  for(colnr = 1; colnr <= lp->columns; colnr++) {
    k = lp->SOS->memberpos[colnr-1];
    nk = lp->SOS->memberpos[colnr];
    for(; k < nk; k++) {
      if(!SOS_is_member(lp->SOS, lp->SOS->membership[k], colnr)) {
        nerr++;
        report(lp, IMPORTANT, "presolve_SOScheck: E - Sparse array did not indicate column index %d as member of SOS %d\n",
                              colnr, lp->SOS->membership[k]);
      }
    }
  }
  status = (MYBOOL) (nerr == 0);
  if(!status)
    report(lp, IMPORTANT, "presolve_SOScheck: There were %d errors\n",
                           nerr);


  return( status );
}

/* ----------------------------------------------------------------------------- */
/* Presolve routines for tightening the model                                    */
/* ----------------------------------------------------------------------------- */

INLINE REAL presolve_roundrhs(lprec *lp, REAL value, MYBOOL isGE)
{
#ifdef DoPresolveRounding
  REAL eps = PRESOLVE_EPSVALUE*1000,
  /* REAL eps = PRESOLVE_EPSVALUE*pow(10.0,MAX(0,log10(1+fabs(value)))), */
  testout = my_precision(value, eps);
#if 1
  if(my_chsign(isGE, value-testout) < 0)
    value = testout;
#elif 0
  if(my_chsign(isGE, value-testout) < 0)
    value = testout;
  else if(value != testout)
    value += my_chsign(isGE, (value-testout)/2);
    /* value = testout + my_chsign(isGE, (value-testout)/2); */
#else
  if(testout != value)
    value += my_chsign(isGE, eps*1000);              /* BASE OPTION */
#endif

#endif
  return( value );
}

INLINE REAL presolve_roundval(lprec *lp, REAL value)
{
#ifdef DoPresolveRounding
  /* value = my_precision(value, PRESOLVE_EPSVALUE*MAX(1,log10(1+fabs(value)))); */
  value = my_precision(value, PRESOLVE_EPSVALUE);    /* BASE OPTION */
#endif
  return( value );
}

INLINE MYBOOL presolve_mustupdate(lprec *lp, int colnr)
{
#if 0
  return( my_infinite(lp, get_lowbo(lp, colnr)) ||
          my_infinite(lp, get_upbo(lp, colnr)) );
#else
  return( my_infinite(lp, lp->orig_lowbo[lp->rows+colnr]) ||
          my_infinite(lp, lp->orig_upbo[lp->rows+colnr]) );
#endif
}

INLINE REAL presolve_sumplumin(lprec *lp, int item, psrec *ps, MYBOOL doUpper)
{
  REAL *plu = (doUpper ? ps->pluupper : ps->plulower),
       *neg = (doUpper ? ps->negupper : ps->neglower);

  if(fabs(plu[item]) >= lp->infinite)
    return( plu[item] );
  else if(fabs(neg[item]) >= lp->infinite)
    return( neg[item] );
  else
    return( plu[item]+neg[item] );
}

INLINE void presolve_range(lprec *lp, int rownr, psrec *ps, REAL *loValue, REAL *hiValue)
{
  *loValue = presolve_sumplumin(lp, rownr,   ps, FALSE);
  *hiValue = presolve_sumplumin(lp, rownr,   ps, TRUE);
}

STATIC void presolve_rangeorig(lprec *lp, int rownr, psrec *ps, REAL *loValue, REAL *hiValue, REAL delta)
{
  delta = my_chsign(is_chsign(lp, rownr), lp->presolve_undo->fixed_rhs[rownr] + delta);
  *loValue = presolve_sumplumin(lp, rownr,   ps, FALSE) + delta;
  *hiValue = presolve_sumplumin(lp, rownr,   ps, TRUE) + delta;
}

STATIC MYBOOL presolve_rowfeasible(presolverec *psdata, int rownr, MYBOOL userowmap)
{
  lprec    *lp = psdata->lp;
  MYBOOL   status = TRUE;
  int      contype, origrownr = rownr;
  REAL     LHS, RHS, value;

  /* Optionally loop across all active rows in the provided map (debugging) */
  if(userowmap)
    rownr = firstActiveLink(psdata->rows->varmap);

  /* Now do once for ingoing rownr or loop across rowmap */
  while((status == TRUE) && (rownr != 0)) {

    /* Check the lower bound */
    value = presolve_sumplumin(lp, rownr, psdata->rows, TRUE);
    LHS = get_rh_lower(lp, rownr);
    if(value < LHS-lp->epssolution) {
      contype = get_constr_type(lp, rownr);
      report(lp, NORMAL, "presolve_rowfeasible: Lower bound infeasibility in %s row %s (%g << %g)\n",
                          get_str_constr_type(lp, contype), get_row_name(lp, rownr), value, LHS);
      if(rownr != origrownr)
      report(lp, NORMAL, "        ...           Input row base used for testing was %s\n",
                                                    get_row_name(lp, origrownr));
      status = FALSE;
    }

    /* Check the upper bound */
    value = presolve_sumplumin(lp, rownr, psdata->rows, FALSE);
    RHS = get_rh_upper(lp, rownr);
    if(value > RHS+lp->epssolution) {
      contype = get_constr_type(lp, rownr);
      report(lp, NORMAL, "presolve_rowfeasible: Upper bound infeasibility in %s row %s (%g >> %g)\n",
                          get_str_constr_type(lp, contype), get_row_name(lp, rownr), value, RHS);
      status = FALSE;
    }
    if(userowmap)
      rownr = nextActiveLink(psdata->rows->varmap, rownr);
    else
      rownr = 0;
  }
  return( status );
}

STATIC MYBOOL presolve_debugmap(presolverec *psdata, char *caption)
{
  lprec *lp = psdata->lp;
  MATrec *mat = lp->matA;
  int    colnr, ix, ie, nx, jx, je, *cols, *rows, n;
  int    nz = mat->col_end[lp->columns] - 1;
  MYBOOL status = FALSE;

  for(colnr = 1; colnr <= lp->columns; colnr++) {
    rows = psdata->cols->next[colnr];
    if(!isActiveLink(psdata->cols->varmap, colnr)) {
      if(rows != NULL) {
        report(lp, SEVERE, "presolve_debugmap: Inactive column %d is non-empty\n",
                           colnr);
        goto Done;
      }
      else
        continue;
    }
    if(rows == NULL)
      report(lp, SEVERE, "presolve_debugmap: Active column %d is empty\n",
                         colnr);
    je = *rows;
    rows++;
    for(jx = 1; jx <= je; jx++, rows++) {
      if((*rows < 0) || (*rows > nz)) {
        report(lp, SEVERE, "presolve_debugmap: NZ index %d for column %d out of range (index %d<=%d)\n",
                           *rows, colnr, jx, je);
        goto Done;
      }
      cols = psdata->rows->next[COL_MAT_ROWNR(*rows)];
      ie = cols[0];
      n = 0;
      for(ix = 1; ix <= ie; ix++) {
        nx = cols[ix];
        if((nx < 0) || (nx > nz)) {
          report(lp, SEVERE, "presolve_debugmap: NZ index %d for column %d to row %d out of range\n",
                             nx, colnr, jx);
          goto Done;
        }
      }
    }
  }
  status = TRUE;
Done:
  if(!status && (caption != NULL))
    report(lp, SEVERE, "...caller was '%s'\n", caption);
  return( status );
}

STATIC MYBOOL presolve_validate(presolverec *psdata, MYBOOL forceupdate)
{
  int    i, ie, j, je, k, rownr, *items;
  REAL   upbound, lobound, value;
  lprec  *lp = psdata->lp;
  MATrec *mat = lp->matA;
  MYBOOL status = mat->row_end_valid && !forceupdate;

  if(status)
    return( status );
  else if(!mat->row_end_valid)
    status = mat_validate(mat);
  else
    status = forceupdate;
  if(status) {

    /* First update rows... */
    for(i = 1; i <= lp->rows; i++) {

      psdata->rows->plucount[i] = 0;
      psdata->rows->negcount[i] = 0;
      psdata->rows->pluneg[i]   = 0;

      if(!isActiveLink(psdata->rows->varmap, i)) {
        FREE(psdata->rows->next[i]);
      }
      else {
        /* Create next column pointers by row */
        k = mat_rowlength(mat, i);
        allocINT(lp, &(psdata->rows->next[i]), k+1, AUTOMATIC);
        items = psdata->rows->next[i];
        je = mat->row_end[i];
        k = 0;
        for(j = mat->row_end[i-1]; j < je; j++)
          if(isActiveLink(psdata->cols->varmap, ROW_MAT_COLNR(j))) {
            k++;
            items[k] = j;
          }
        items[0] = k;
      }
    }

    /* ...then update columns */
    for(j = 1; j <= lp->columns; j++) {

      psdata->cols->plucount[j] = 0;
      psdata->cols->negcount[j] = 0;
      psdata->cols->pluneg[j]   = 0;

      if(!isActiveLink(psdata->cols->varmap, j)) {
        FREE(psdata->cols->next[j]);
      }
      else {
        upbound = get_upbo(lp, j);
        lobound = get_lowbo(lp, j);
        if(is_semicont(lp, j) && (upbound > lobound)) {
          if(lobound > 0)
            lobound = 0;
          else if(upbound < 0)
            upbound = 0;
        }

        /* Create next row pointers by column */
        k = mat_collength(mat, j);
        allocINT(lp, &(psdata->cols->next[j]), k+1, AUTOMATIC);
        items = psdata->cols->next[j];
        ie = mat->col_end[j];
        k = 0;
        for(i = mat->col_end[j-1]; i < ie; i++) {
          rownr = COL_MAT_ROWNR(i);
          if(isActiveLink(psdata->rows->varmap, rownr)) {
            k++;
            items[k] = i;

            /* Cumulate counts */
            value = COL_MAT_VALUE(i);
            if(my_chsign(is_chsign(lp, rownr), value) > 0) {
              psdata->rows->plucount[rownr]++;
              psdata->cols->plucount[j]++;
            }
            else {
              psdata->rows->negcount[rownr]++;
              psdata->cols->negcount[j]++;
            }
            if((lobound < 0) && (upbound >= 0)) {
              psdata->rows->pluneg[rownr]++;
              psdata->cols->pluneg[j]++;
            }
          }
        }
        items[0] = k;
      }
    }
#ifdef Paranoia
    presolve_debugmap(psdata, "presolve_validate");
#endif
  }
  return( status );
}

STATIC MYBOOL presolve_rowtallies(presolverec *psdata, int rownr, int *plu, int *neg, int *pluneg)
{
  REAL   value;
  lprec  *lp = psdata->lp;
  MATrec *mat = lp->matA;
  int    ix, jx, ib = 0;
  MYBOOL chsign = is_chsign(lp, rownr);

  /* Initialize */
  *plu = 0;
  *neg = 0;
  *pluneg = 0;

  /* Loop over still active row members */
  for(ix = presolve_nextcol(psdata, rownr, &ib); ix >= 0; ix = presolve_nextcol(psdata, rownr, &ib)) {

    /* Get matrix column and value */
    jx    = ROW_MAT_COLNR(ix);
    value = ROW_MAT_VALUE(ix);

    /* Cumulate counts */
    if(my_chsign(chsign, value) > 0)
      (*plu)++;
    else
      (*neg)++;
    if((get_lowbo(lp, jx) < 0) && (get_upbo(lp, jx) >= 0))
      (*pluneg)++;
  }
  return( TRUE );
}
STATIC MYBOOL presolve_debugrowtallies(presolverec *psdata)
{
  lprec  *lp = psdata->lp;
  int    i, plu, neg, pluneg, nerr = 0;

  for(i = 1; i <= lp->rows; i++)
    if(isActiveLink(psdata->rows->varmap, i) &&
       presolve_rowtallies(psdata, i, &plu, &neg, &pluneg)) {
      if((psdata->rows->plucount[i] != plu) ||
         (psdata->rows->negcount[i] != neg) ||
         (psdata->rows->pluneg[i] != pluneg)) {
        nerr++;
        report(lp, SEVERE, "presolve_debugrowtallies: Detected inconsistent count for row %d\n", i);
      }
    }
  return( (MYBOOL) (nerr == 0) );
}

STATIC int presolve_debugcheck(lprec *lp, LLrec *rowmap, LLrec *colmap)
{
  int i, j, errc = 0;

  /* Validate constraint bounds */
  for(i = 1; i < lp->rows; i++) {
    if((rowmap != NULL) && !isActiveLink(rowmap, i))
      continue;
    /* Check if we have a negative range */
    if(lp->orig_upbo[i] < 0) {
      errc++;
      report(lp, SEVERE, "presolve_debugcheck: Detected negative range %g for row %d\n",
                         lp->orig_upbo[i], i);
    }
  }
  /* Validate variables */
  for(j = 1; j < lp->columns; j++) {
    if((colmap != NULL) && !isActiveLink(colmap, j))
      continue;
    i = lp->rows+j;
    /* Check if we have infeasible  bounds */
    if(lp->orig_lowbo[i] > lp->orig_upbo[i]) {
      errc++;
      report(lp, SEVERE, "presolve_debugcheck: Detected UB < LB for column %d\n",
                         j);
    }
  }
  /* Return total number of errors */
  return( errc );
}

STATIC MYBOOL presolve_candeletevar(presolverec *psdata, int colnr)
{
  lprec    *lp = psdata->lp;
  int      usecount = SOS_memberships(lp->SOS, colnr);

  return( (MYBOOL) ((lp->SOS == NULL) || (usecount == 0) ||
                    (/*is_presolve(lp, PRESOLVE_SOS) &&*/
                     (((lp->SOS->sos1_count == lp->SOS->sos_count)) ||
                      (usecount == SOS_is_member_of_type(lp->SOS, colnr, SOS1))))) );
}

STATIC int presolve_rowlengthex(presolverec *psdata, int rownr)
{
  int j1 = psdata->rows->plucount[rownr] + psdata->rows->negcount[rownr];
#ifdef Paranoia
  int j2 = presolve_rowlength(psdata, rownr);

  if(j1 != j2) {
    report(psdata->lp, SEVERE, "presolve_rowlengthex: Expected row length %d, but found %d in row %s\n",
                                j2, j1, get_row_name(psdata->lp, rownr));
    j1 = -j1;
  }
#endif

  return( j1 );
}
STATIC int presolve_rowlengthdebug(presolverec *psdata)
{
  int rownr, n = 0;

  for(rownr = firstActiveLink(psdata->rows->varmap); rownr != 0;
    rownr = nextActiveLink(psdata->rows->varmap, rownr))
    n += presolve_rowlengthex(psdata, rownr);
  return( n );
}

INLINE int presolve_nextrecord(psrec *ps, int recnr, int *previtem)
{
  int *nzlist = ps->next[recnr], nzcount = nzlist[0], status = -1;

  /* Check if we simply wish the last active column */
  if(previtem == NULL) {
    if(nzlist != NULL)
      status = nzlist[*nzlist];
    return( status );
  }

  /* Step to next */
#ifdef Paranoia
  else if((*previtem < 0) || (*previtem > nzcount))
    return( status );
#endif
  (*previtem)++;

  /* Set the return values */
  if(*previtem > nzcount)
    (*previtem) = 0;
  else
    status = nzlist[*previtem];

  return( status );
}
INLINE int presolve_nextcol(presolverec *psdata, int rownr, int *previtem)
/* Find the first active (non-eliminated) nonzero column in rownr after prevcol */
{
  return( presolve_nextrecord(psdata->rows, rownr, previtem) );
}
INLINE int presolve_lastcol(presolverec *psdata, int rownr)
{
  return( presolve_nextrecord(psdata->rows, rownr, NULL) );
}
INLINE int presolve_nextrow(presolverec *psdata, int colnr, int *previtem)
/* Find the first active (non-eliminated) nonzero row in colnr after prevrow */
{
  return( presolve_nextrecord(psdata->cols, colnr, previtem) );
}
INLINE int presolve_lastrow(presolverec *psdata, int colnr)
{
  return( presolve_nextrecord(psdata->cols, colnr, NULL) );
}

INLINE void presolve_adjustrhs(presolverec *psdata, int rownr, REAL fixdelta, REAL epsvalue)
{
  lprec *lp = psdata->lp;

  lp->orig_rhs[rownr] -= fixdelta;
  if(epsvalue > 0)
#if 1
    my_roundzero(lp->orig_rhs[rownr], epsvalue);
#else
    lp->orig_rhs[rownr] = presolve_roundrhs(lp, lp->orig_rhs[rownr], FALSE);
#endif
  lp->presolve_undo->fixed_rhs[rownr] += fixdelta;
}

STATIC int presolve_shrink(presolverec *psdata, int *nConRemove, int *nVarRemove)
{
  SOSgroup *SOS = psdata->lp->SOS;
  int     status = RUNNING, countR = 0, countC = 0,
          i, ix, n, *list;
  REAL    fixValue;

  /* Remove empty rows */
  list = psdata->rows->empty;
  if(list != NULL) {
    n = list[0];
    for(i = 1; i <= n; i++)
      if(isActiveLink(psdata->rows->varmap, list[i])) {
        presolve_rowremove(psdata, list[i], FALSE);
        countR++;
      }
    if(nConRemove != NULL)
      (*nConRemove) += countR;
    list[0] = 0;
  }

  /* Fix and remove empty columns (unless they are in a SOS) */
  list = psdata->cols->empty;
  if(list != NULL) {
    n = list[0];
    for(i = 1; i <= n; i++) {
      ix = list[i];
      if(isActiveLink(psdata->cols->varmap, ix)) {
        if(presolve_colfixdual(psdata, ix, &fixValue, &status)) {
          if(!presolve_colfix(psdata, ix, fixValue, TRUE, nVarRemove)) {
            status = presolve_setstatus(psdata, INFEASIBLE);
            break;
          }
          presolve_colremove(psdata, ix, FALSE);
          countC++;
        }
        else if(SOS_is_member(SOS, 0, ix))
          report(psdata->lp, DETAILED, "presolve_shrink: Empty column %d is member of a SOS\n", ix);
      }
    }
    list[0] = 0;
  }

  return( status );
}

STATIC void presolve_rowremove(presolverec *psdata, int rownr, MYBOOL allowcoldelete)
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  int      ix, ie, nx, jx, je, *cols, *rows, n, colnr;

#ifdef Paranoia
  if((rownr < 1) || (rownr > lp->rows))
    report(lp, SEVERE, "presolve_rowremove: Row %d out of range\n", rownr);
#endif

  /* Remove this row for each column that is active in the row */
  cols = psdata->rows->next[rownr];
  ie = *cols;
  cols++;
  for(ix = 1; ix <= ie; ix++, cols++) {
    n = 0;
    colnr = ROW_MAT_COLNR(*cols);
    rows = psdata->cols->next[colnr];
    je = rows[0];
    /* See if we can narrow the search window */
    jx = je / 2;
    if((jx > 5) && (rownr >= COL_MAT_ROWNR(rows[jx])))
      n = jx-1;
    else
      jx = 1;
    /* Do the compression loop */
    for(; jx <= je; jx++) {
      nx = rows[jx];
      if(COL_MAT_ROWNR(nx) != rownr) {
        n++;
        rows[n] = nx;
      }
    }
    rows[0] = n;

    /* Make sure we delete columns that have become empty */
#if 1
    if((n == 0) && allowcoldelete) {
      int *list = psdata->cols->empty;
      n = ++list[0];
      list[n] = colnr;
    }
#endif

  }
  FREE(psdata->rows->next[rownr]);

  removeLink(psdata->rows->varmap, rownr);
  switch(get_constr_type(lp, rownr)) {
    case LE: removeLink(psdata->LTmap, rownr);
              break;
    case EQ: removeLink(psdata->EQmap, rownr);
              break;
  }
  if(isActiveLink(psdata->INTmap, rownr))
    removeLink(psdata->INTmap, rownr);
}

STATIC int presolve_colremove(presolverec *psdata, int colnr, MYBOOL allowrowdelete)
{
  lprec    *lp = psdata->lp;

#ifdef Paranoia
  if((colnr < 1) || (colnr > lp->columns))
    report(lp, SEVERE, "presolve_colremove: Column %d out of range\n", colnr);
  if(!isActiveLink(psdata->cols->varmap, colnr) || !presolve_candeletevar(psdata, colnr))
    colnr = -1;
  else
#endif
  {
    MATrec *mat = lp->matA;
    int    ix, ie, nx, jx, je, *cols, *rows, n, rownr;

    /* Remove this column for each row that is active in the column */
    rows = psdata->cols->next[colnr];
    je = *rows;
    rows++;
    for(jx = 1; jx <= je; jx++, rows++) {
      n = 0;
      rownr = COL_MAT_ROWNR(*rows);
      cols = psdata->rows->next[rownr];
      ie = cols[0];
      /* See if we can narrow the search window */
      ix = ie / 2;
      if((ix > 5) && (colnr >= ROW_MAT_COLNR(cols[ix])))
        n = ix-1;
      else
        ix = 1;
      /* Do the compression loop */
      for(; ix <= ie; ix++) {
        nx = cols[ix];
        if(ROW_MAT_COLNR(nx) != colnr) {
          n++;
          cols[n] = nx;
        }
      }
      cols[0] = n;

      /* Make sure we delete rows that become empty */
#if 1
      if((n == 0) && allowrowdelete) {
        int *list = psdata->rows->empty;
        n = ++list[0];
        list[n] = rownr;
      }
#endif

    }
    FREE(psdata->cols->next[colnr]);

    /* Update other counts */
    if(SOS_is_member(lp->SOS, 0, colnr)) {
      if(lp->sos_priority != NULL) {
        lp->sos_vars--;
        if(is_int(lp, colnr))
          lp->sos_ints--;
      }
      SOS_member_delete(lp->SOS, 0, colnr);
      clean_SOSgroup(lp->SOS, TRUE);
      if(SOS_count(lp) == 0)
        free_SOSgroup(&(lp->SOS));
    }

    /* Finally remove the column from the active column list */
    colnr = removeLink(psdata->cols->varmap, colnr);
  }
  return( colnr );
}

STATIC int presolve_redundantSOS(presolverec *psdata, int *nb, int *nSum)
{
  lprec    *lp = psdata->lp;
  int      i, ii, k, kk, j, nrows = lp->rows, *fixed = NULL,
           iBoundTighten = 0, status = RUNNING;
  SOSrec   *SOS;

  /* Is there anything to do? */
  i = ii = SOS_count(lp);
  if(ii == 0)
    return( status );

  /* Allocate working member list */
  if(!allocINT(lp, &fixed, lp->columns+1, FALSE) )
    return( lp->spx_status );

  /* Check if we have SOS'es that are already satisfied or fixable/satisfiable */
  while(i > 0) {
    SOS = lp->SOS->sos_list[i-1];
    kk = SOS->members[0];
    fixed[0] = 0;
    for(k = 1; k <= kk; k++) {
      j = SOS->members[k];
      if((get_lowbo(lp, j) > 0) && !is_semicont(lp, j)) {
        fixed[++fixed[0]] = k;
        /* Abort if we have identified SOS infeasibility */
        if(fixed[0] > SOS->type) {
          status = presolve_setstatus(psdata, INFEASIBLE);
          goto Done;
        }
      }
    }
    /* If there were an exact number of non-zero SOS members, check their sequentiality */
    if(fixed[0] == SOS->type) {
      /* Check sequentiality of members with non-zero lower bounds */
      for(k = 2; k <= fixed[0]; k++) {
        if(fixed[k] != fixed[k-1]+1) {
          status = presolve_setstatus(psdata, INFEASIBLE);
          goto Done;
        }
      }
      /* Fix other member variables to zero, if necessary */
      for(k = kk; k > 0; k--) {
        j = SOS->members[k];
        if((get_lowbo(lp, j) > 0) && !is_semicont(lp, j))
          continue;
        if(!presolve_colfix(psdata, j, 0.0, AUTOMATIC, &iBoundTighten)) {
          status = presolve_setstatus(psdata, INFEASIBLE);
          goto Done;
        }
      }
      /* Remove the SOS */
      delete_SOSrec(lp->SOS, i /* , FALSE */);
    }
    /* Otherwise, try to fix variables outside the SOS type window */
    else if(fixed[0] > 0) {
      for(k = kk; k > 0; k--) {
        if((k > fixed[fixed[0]]-SOS->type) && /* After leading entries   */
           (k < fixed[1]+SOS->type))          /* Before trailing entries */
          continue;
        j = SOS->members[k];
        SOS_member_delete(lp->SOS, i, j);
	/* if(get_upbo(lp, j) - get_lowbo(lp, j) < lp->epsprimal) */
        if(is_fixedvar(lp, nrows+j))
          continue;
        if(!presolve_colfix(psdata, j, 0.0, AUTOMATIC, &iBoundTighten)) {
          status = presolve_setstatus(psdata, INFEASIBLE);
          goto Done;
        }
      }
    }
    i--;
  }

  /* Update the sparse member map if there were SOS deletions;
     Remember that delete_SOSrec() above specified deferred updating! */
  i = SOS_count(lp);
  if((i < ii) || (iBoundTighten > 0)) {
    SOS_member_updatemap(lp->SOS);
  }

  /* Update tag orders */
  for(; i > 0; i--)
    lp->SOS->sos_list[i-1]->tagorder = i;

Done:
  FREE(fixed);
  (*nb) += iBoundTighten;
  (*nSum) += iBoundTighten;

  return( status );
}

STATIC MYBOOL presolve_fixSOS1(presolverec *psdata, int colnr, REAL fixvalue, int *nr, int *nv)
{
  lprec    *lp = psdata->lp;
  int      i, k, j;
  SOSrec   *SOS;
  REAL     newvalue;
  MYBOOL   *fixed = NULL, status = FALSE;

  /* Allocate working member list */
  if(!allocMYBOOL(lp, &fixed, lp->columns+1, TRUE) )
    return(FALSE);

  /* Fix variables in SOS's where colnr is a member */
  i = SOS_count(lp);
  while(i > 0) {
    /* Set next SOS target (note that colnr has been tested earlier as not being a member of a higher order SOS) */
    SOS = lp->SOS->sos_list[i-1];
    if(SOS_is_member(lp->SOS, i, colnr)) {
      for(k = SOS->members[0]; k > 0; k--) {
        j = SOS->members[k];
        if(fixed[j])
          continue;
        if(j == colnr) {
          fixed[j] = TRUE;
          newvalue = fixvalue;
        }
        else {
          fixed[j] = AUTOMATIC;
          newvalue = 0.0;
        }
        /* If it is a member of a higher order SOS then just change bounds */
        if(!presolve_candeletevar(psdata, j)) {
          set_bounds(lp, j, newvalue, newvalue);
          fixed[j] = TRUE | AUTOMATIC;
          psdata->forceupdate = TRUE;
        }
        /* Otherwise fix it in preparation for removal */
        else if(!presolve_colfix(psdata, j, newvalue, TRUE, nv))
          goto Done;
      }
    }
    i--;
  }

  /* Delete SOS'es or SOS member variables where we can */
  k = i = SOS_count(lp);
  while(i > 0) {
    /* Set next SOS target */
    SOS = lp->SOS->sos_list[i-1];
    if(SOS_is_member(lp->SOS, i, colnr)) {
      /* Always delete SOS1's */
      if(SOS->type == SOS1)
        delete_SOSrec(lp->SOS, i /* , FALSE */);
      /* Only delete leading or trailing SOS members in higher-order SOS'es that are fixed at 0;
        (note that this section of the code will never be called in the current setup) */
      else {
        /* First the leading entries... */
        for(j = 1; j <= SOS->members[0]; j++) {
          if(fixed[SOS->members[j]] == AUTOMATIC)
            SOS_member_delete(lp->SOS, i, SOS->members[j]);
        }
        /* ...then trailing entries */
        for(j = SOS->members[0]; j > 0; j--) {
          if(fixed[SOS->members[j]] == AUTOMATIC)
            SOS_member_delete(lp->SOS, i, SOS->members[j]);
        }
      }
    }
    i--;
  }

  /* Update the sparse member map if there were SOS deletions; delete_SOSrec() above
     specified deferred updating */
  i = SOS_count(lp);
  if(i < k)
    SOS_member_updatemap(lp->SOS);

  /* Delete the variables that have been fixed */
  k = 0;
  for(j = lp->columns; j > 0; j--) {
    if((fixed[j] == TRUE) || (fixed[j] == AUTOMATIC)) {
       presolve_colremove(psdata, j, TRUE);
       k++;
    }
  }

  /* Update tag orders */
  i = SOS_count(lp);
  for(; i > 0; i--)
    lp->SOS->sos_list[i-1]->tagorder = i;

  status = TRUE;

Done:
  FREE(fixed);
  return( status );
}

STATIC void presolve_setEQ(presolverec *psdata, int rownr)
{
  lprec *lp = psdata->lp;

  if(is_constr_type(lp, rownr, LE))
     removeLink(psdata->LTmap, rownr);
   setLink(psdata->EQmap, rownr);
   set_constr_type(lp, rownr, EQ);
   psdata->dv_lobo[rownr] = -lp->infinite;
   psdata->dv_upbo[rownr] = lp->infinite;
}

STATIC MYBOOL presolve_singletonbounds(presolverec *psdata, int rownr, int colnr, REAL *lobound, REAL *upbound, REAL *aval)
{
  lprec  *lp = psdata->lp;
  REAL   coeff_a, epsvalue = psdata->epsvalue;
  MYBOOL isneg;

  /* Compute row singleton variable range */
  if(is_constr_type(lp, rownr, EQ) && (fabs(*lobound) < epsvalue))
    *lobound = *upbound = 0;
  else {
    if(aval == NULL)
      coeff_a = get_mat(lp, rownr, colnr);
    else
      coeff_a = *aval;
    isneg = (MYBOOL) (coeff_a < 0);
    if(*lobound > -lp->infinite)
      *lobound /= coeff_a;
    else if(isneg)
      *lobound = -(*lobound);
    if(*upbound < lp->infinite)
      *upbound /= coeff_a;
    else if(isneg)
      *upbound = -(*upbound);
    if(isneg)
      swapREAL(lobound, upbound);
  }

  /* Check against bound - handle SC variables specially */
  if(is_semicont(lp, colnr)) {
    coeff_a = get_lowbo(lp, colnr);
    if(coeff_a > 0) {
      SETMAX(*lobound, 0.0);
      SETMIN(*upbound, get_upbo(lp, colnr));
    }
    else {
      coeff_a = get_upbo(lp, colnr);
      if(coeff_a > 0) {
        SETMAX(*lobound, get_lowbo(lp, colnr));
        SETMIN(*upbound, 0.0);
      }
    }
  }
  else {
    SETMAX(*lobound, get_lowbo(lp, colnr));
    SETMIN(*upbound, get_upbo(lp, colnr));
  }

  /* Return with consistency status */
#ifdef DoPresolveRelativeTest
  isneg = (MYBOOL) (my_reldiff(*upbound, *lobound) >= - epsvalue);
#else
  isneg = (MYBOOL) (*upbound >= *lobound - epsvalue);
#endif
  if(!isneg) {
    /* Attempt bound-related error correction */
    if(fabs(my_reldiff(*lobound, get_upbo(lp, colnr))) < PRESOLVE_BOUNDSLACK*epsvalue)
      *lobound = get_upbo(lp, colnr);
    else if(fabs(my_reldiff(*upbound, get_lowbo(lp, colnr))) < PRESOLVE_BOUNDSLACK*epsvalue)
      *upbound = get_lowbo(lp, colnr);
#ifdef DoPresolveRelativeTest
    isneg = (MYBOOL) (my_reldiff(*upbound, *lobound) >= - epsvalue);
#else
    isneg = (MYBOOL) (*upbound >= *lobound - epsvalue);
#endif
    if(!isneg)
      report(lp, NORMAL, "presolve_singletonbounds: Singleton variable %s in row %s infeasibility (%g << %g)\n",
                         get_col_name(lp, colnr), get_row_name(lp, rownr), *lobound, *upbound);
  }
  return( isneg );
}

STATIC MYBOOL presolve_altsingletonvalid(presolverec *psdata, int rownr, int colnr, REAL reflotest, REAL refuptest)
{
  lprec *lp = psdata->lp;
  REAL  coeff_bl, coeff_bu, epsvalue = psdata->epsvalue;

  coeff_bl = get_rh_lower(lp, rownr);
  coeff_bu = get_rh_upper(lp, rownr);

  /* Check base data validity */
#ifdef DoPresolveRelativeTest
  if((my_reldiff(refuptest, reflotest) < -epsvalue) ||
#else
  if((reflotest > refuptest + epsvalue) ||
#endif
     !presolve_singletonbounds(psdata, rownr, colnr, &coeff_bl, &coeff_bu, NULL))
    return( FALSE );

  /* Base data is Ok, now check against against each other */
  epsvalue = MAX(reflotest-coeff_bu, coeff_bl-refuptest) / epsvalue;
  if(epsvalue > PRESOLVE_BOUNDSLACK) {
    report(lp, NORMAL, "presolve_altsingletonvalid: Singleton variable %s in row %s infeasible (%g)\n",
                       get_col_name(lp, colnr), get_row_name(lp, rownr), MAX(reflotest-coeff_bu, coeff_bl-refuptest));
    return( FALSE );
  }
  else
    return( TRUE );
}

STATIC MYBOOL presolve_multibounds(presolverec *psdata, int rownr, int colnr,
                                   REAL *lobound, REAL *upbound, REAL *aval, MYBOOL *rowbinds)
{
  lprec    *lp = psdata->lp;
  MYBOOL   rowbindsvar = FALSE, status = FALSE;
  REAL     coeff_a, LHS, RHS, netX, Xupper, Xlower, epsvalue = psdata->epsvalue;

  /* Get variable bounds for netting */
  LHS = *lobound;
  RHS = *upbound;
  Xlower = get_lowbo(lp, colnr);
  Xupper = get_upbo(lp, colnr);

  /* Identify opportunity for bound tightening */
  if(aval == NULL)
    coeff_a = get_mat(lp, rownr, colnr);
  else
    coeff_a = *aval;

  netX = presolve_sumplumin(lp, rownr, psdata->rows, TRUE);
  if(!my_infinite(lp, LHS) && !my_infinite(lp, netX)) {
    if(coeff_a > 0) {
      LHS -= netX-coeff_a*Xupper;
      LHS /= coeff_a;
      if(LHS > Xlower + epsvalue) {
        Xlower = presolve_roundrhs(lp, LHS, TRUE);
        status = TRUE;
      }
      else if(LHS > Xlower - epsvalue)
        rowbindsvar = TRUE;
    }
    else {
      LHS -= netX-coeff_a*Xlower;
      LHS /= coeff_a;
      if(LHS < Xupper - epsvalue) {
        Xupper = presolve_roundrhs(lp, LHS, FALSE);
        status = AUTOMATIC;
      }
      else if(LHS < Xupper + epsvalue)
        rowbindsvar = AUTOMATIC;
    }
  }

  netX = presolve_sumplumin(lp, rownr, psdata->rows, FALSE);
  if(!my_infinite(lp, RHS) && !my_infinite(lp, netX)) {
    if(coeff_a < 0) {
      if(!my_infinite(lp, Xupper)) {
        RHS -= netX-coeff_a*Xupper;
        RHS /= coeff_a;
        if(RHS > Xlower + epsvalue) {
          Xlower = presolve_roundrhs(lp, RHS, TRUE);
          status |= TRUE;
        }
        else if(RHS > Xlower - epsvalue)
          rowbindsvar |= TRUE;
      }
    }
    else if(!my_infinite(lp, Xlower)) {
      RHS -= netX-coeff_a*Xlower;
      RHS /= coeff_a;
      if(RHS < Xupper - epsvalue) {
        Xupper = presolve_roundrhs(lp, RHS, FALSE);
        status |= AUTOMATIC;
      }
      else if(RHS < Xupper + epsvalue)
        rowbindsvar |= AUTOMATIC;
    }
  }

  *lobound = Xlower;
  *upbound = Xupper;
  if(rowbinds != NULL)
    *rowbinds = rowbindsvar;

  return(status);
}

STATIC MYBOOL isnz_origobj(lprec *lp, int colnr)
{
  return( (MYBOOL) (lp->orig_obj[colnr] != 0) );
}

STATIC MYBOOL presolve_testrow(presolverec *psdata, int lastrow)
{
  if(psdata->forceupdate) {
    presolve_updatesums(psdata);
    psdata->forceupdate = FALSE;
  }
  if(!presolve_rowfeasible(psdata, 0, TRUE))
    return( FALSE );
  else
    return( TRUE );
}

STATIC MYBOOL presolve_coltighten(presolverec *psdata, int colnr, REAL LOnew, REAL UPnew, int *count)
{
  lprec    *lp = psdata->lp;
  int      elmnr, elmend, k, oldcount = 0, newcount = 0, deltainf;
  REAL     LOold, UPold, Value, margin = psdata->epsvalue;
  MATrec   *mat = lp->matA;
  REAL     *value;
  int      *rownr;

  /* Attempt correction of marginally equal, but inconsistent input values */
  Value = UPnew - LOnew;
  if((Value <= -margin) && (Value > -lp->epsprimal)) {
    if(fabs(fmod(UPnew, 1.0)) < margin)
      LOnew = UPnew;
    else
      UPnew = LOnew;
  }

  /* Check if there is anything to do */
  LOold = get_lowbo(lp, colnr);
  UPold = get_upbo(lp, colnr);
#ifdef Paranoia
  if(((LOold > LOnew) && !is_semicont(lp, colnr)) || (UPold < UPnew)) {
    report(lp, SEVERE, "presolve_coltighten: Inconsistent new bounds requested for column %d\n", colnr);
    return( FALSE );
  }
#endif
  if(count != NULL)
    newcount = *count;
  oldcount = newcount;

  /* Modify inf-count */
  deltainf = 0;
  if((UPold < lp->infinite) || (LOold > -lp->infinite))
    deltainf -= 1;
  if((UPnew < lp->infinite) || (LOnew > -lp->infinite))
    deltainf += 1;
  if(isnz_origobj(lp, colnr))
    psdata->rows->infcount[0] += deltainf;
  elmnr = mat->col_end[colnr-1];
  elmend = mat->col_end[colnr];
  rownr = &COL_MAT_ROWNR(elmnr);
  for(; elmnr < elmend; elmnr++, rownr += matRowColStep) {
    k = *rownr;
    if(isActiveLink(psdata->rows->varmap, k))
      psdata->rows->infcount[k] += deltainf;
  }

  /* Look for opportunity to tighten upper variable bound */
  if((UPnew < lp->infinite) && (UPnew+margin < UPold)) {
    if(is_int(lp, colnr))
      UPnew = floor(UPnew+margin);
    if(UPold < lp->infinite) {
      /* First do OF */
      k = 0;
      Value = my_chsign(is_chsign(lp, k), lp->orig_obj[colnr]);
      if((Value > 0) && (psdata->rows->pluupper[k] < lp->infinite))
        psdata->rows->pluupper[k] += (UPnew-UPold)*Value;
      else if((Value < 0) && (psdata->rows->negupper[k] < lp->infinite))
        psdata->rows->negupper[k] += (LOnew-LOold)*Value;
      psdata->rows->infcount[k] += deltainf;

      /* Then scan the constraint rows */
      elmnr = mat->col_end[colnr-1];
      elmend = mat->col_end[colnr];
      rownr = &COL_MAT_ROWNR(elmnr);
      value = &COL_MAT_VALUE(elmnr);
      for(; elmnr < elmend;
          elmnr++, rownr += matRowColStep, value += matValueStep) {
        k = *rownr;
        if(!isActiveLink(psdata->rows->varmap, k))
          continue;
        Value = my_chsign(is_chsign(lp, k), *value);
        if((Value > 0) && (psdata->rows->pluupper[k] < lp->infinite))
          psdata->rows->pluupper[k] += (UPnew-UPold)*Value;
        else if((Value < 0) && (psdata->rows->negupper[k] < lp->infinite))
          psdata->rows->negupper[k] += (LOnew-LOold)*Value;
      }
    }
    else
      psdata->forceupdate = TRUE;
    if(UPnew < UPold) {
      UPold = UPnew;
      newcount++;
    }
  }

  /* Look for opportunity to tighten lower variable bound */
  if((LOnew > -lp->infinite) && (LOnew-margin > LOold)) {
    if(is_int(lp, colnr))
       LOnew = ceil(LOnew-margin);
    if(LOold > -lp->infinite) {
      /* First do OF */
      k = 0;
      Value = my_chsign(is_chsign(lp, k), lp->orig_obj[colnr]);
      if((Value > 0) && (psdata->rows->plulower[k] > -lp->infinite))
        psdata->rows->plulower[k] += (LOnew-LOold)*Value;
      else if((Value < 0) && (psdata->rows->neglower[k] > -lp->infinite))
        psdata->rows->neglower[k] += (UPnew-UPold)*Value;

      /* Then scan the constraint rows */
      elmnr = mat->col_end[colnr-1];
      elmend = mat->col_end[colnr];
      rownr = &COL_MAT_ROWNR(elmnr);
      value = &COL_MAT_VALUE(elmnr);
      for(; elmnr < elmend;
          elmnr++, rownr += matRowColStep, value += matValueStep) {
        k = *rownr;
        if(!isActiveLink(psdata->rows->varmap, k))
          continue;
        Value = my_chsign(is_chsign(lp, k), *value);
        if((Value > 0) && (psdata->rows->plulower[k] > -lp->infinite))
          psdata->rows->plulower[k] += (LOnew-LOold)*Value;
        else if((Value < 0) && (psdata->rows->neglower[k] > -lp->infinite))
          psdata->rows->neglower[k] += (UPnew-UPold)*Value;
      }
    }
    else
      psdata->forceupdate = TRUE;
    if(LOnew > LOold) {
      LOold = LOnew;
      newcount++;
    }
  }

  /* Now set the new variable bounds, if they are tighter */
  if(newcount > oldcount) {
    UPnew = presolve_roundval(lp, UPnew);
    LOnew = presolve_roundval(lp, LOnew);
    if(LOnew > UPnew) {
      if(LOnew-UPnew < margin) {
        LOnew = UPnew;
      }
      else {
        report(lp, NORMAL, "presolve_coltighten: Found column %s with LB %g > UB %g\n",
                            get_col_name(lp, colnr), LOnew, UPnew);
        return( FALSE );
      }
    }
    if(lp->spx_trace || (lp->verbose > DETAILED))
      report(lp, NORMAL, "presolve_coltighten: Replaced bounds on column %s to [%g ... %g]\n",
                         get_col_name(lp, colnr), LOnew, UPnew);
    set_bounds(lp, colnr, LOnew, UPnew);
  }
  if(count != NULL)
    *count = newcount;

  return( TRUE );
}

STATIC int presolve_rowtighten(presolverec *psdata, int rownr, int *tally, MYBOOL intsonly)
{
  lprec  *lp = psdata->lp;
  MYBOOL rowbinds;
  int    item = 0, jx, jjx, ix, idxn = 0, *idxbound = NULL, status = RUNNING;
  REAL   *newbound = NULL, RHlo = get_rh_lower(lp, rownr), RHup = get_rh_upper(lp, rownr),
         VARlo, VARup, Aval;
  MATrec *mat = lp->matA;

  jx = presolve_rowlength(psdata, rownr);
  allocREAL(lp, &newbound, 2*jx, TRUE);
  allocINT (lp, &idxbound, 2*jx, TRUE);

  /* Identify bound tightening for each active variable in the constraint */
  for(jx = presolve_nextcol(psdata, rownr, &item); jx >= 0;
      jx = presolve_nextcol(psdata, rownr, &item)) {
    jjx = ROW_MAT_COLNR(jx);
    Aval = ROW_MAT_VALUE(jx);
    Aval = my_chsign(rownr, Aval);

    VARlo = RHlo;
    VARup = RHup;
    presolve_multibounds(psdata, rownr,jjx, &VARlo, &VARup, &Aval, &rowbinds);
    if(rowbinds & TRUE) {
      idxbound[idxn] = -jjx;
      newbound[idxn] = VARlo;
      idxn++;
    }
    if(rowbinds & AUTOMATIC) {
      idxbound[idxn] = jjx;
      newbound[idxn] = VARup;
      idxn++;
    }
  }

  /* Loop over the bounds identified for tightening and perform update */
  ix = 0;
  while(ix < idxn) {
    jjx = idxbound[ix];
    jx = abs(jjx);

    /* Skip free variables and non-ints, if specified */
    if(is_unbounded(lp, jx) ||
       (intsonly && !is_int(lp, jx)))
      continue;

    VARlo = get_lowbo(lp, jx);
    VARup = get_upbo(lp, jx);
    /* while((ix < idxn) && (jx == abs(jjx))) { */
    while((ix < idxn) && (jx == abs((jjx = idxbound[ix])))) {
      if(jjx < 0)
        VARlo = newbound[ix];
      else
        VARup = newbound[ix];
      ix++;
    }
    if(!presolve_coltighten(psdata, jx, VARlo, VARup, tally)) {
      status = presolve_setstatus(psdata, INFEASIBLE);
      break;
    }
  }

  FREE(newbound);
  FREE(idxbound);

  return(status);
}

STATIC void set_dv_bounds(presolverec *psdata, int rownr, REAL lowbo, REAL upbo)
{
  psdata->dv_lobo[rownr] = lowbo;
  psdata->dv_upbo[rownr] = upbo;
}
STATIC REAL get_dv_lower(presolverec *psdata, int rownr)
{
  return( psdata->dv_lobo[rownr] );
}

STATIC REAL get_dv_upper(presolverec *psdata, int rownr)
{
  return( psdata->dv_upbo[rownr] );
}

STATIC MYBOOL presolve_rowfix(presolverec *psdata, int rownr, REAL newvalue, MYBOOL remove, int *tally)
{
  lprec    *lp = psdata->lp;
  int      i, ix, ie;
  MYBOOL   isneg, lofinite, upfinite, doupdate = FALSE, chsign = is_chsign(lp, rownr);
  REAL     lobound, upbound, lovalue, upvalue,
           Value, fixvalue, fixprod, mult;
  MATrec   *mat = lp->matA;
  psrec    *ps = psdata->cols;

  /* Set "fixed" value in case we are deleting a variable */
  upbound = get_dv_upper(psdata, rownr);
  lobound = get_dv_lower(psdata, rownr);
  if(remove) {
    if(upbound-lobound < psdata->epsvalue) {
      if((newvalue > lobound) && (newvalue < upbound))
        fixvalue = newvalue;
      else
        fixvalue = lobound;
    }
    else {
      if(my_infinite(lp, newvalue) && (get_rh(lp, rownr) == 0))
        fixvalue = ((lobound <= 0) && (upbound >= 0) ? 0 : MIN(upbound, lobound));
      else
        fixvalue = newvalue;
    }
    set_dv_bounds(psdata, rownr, fixvalue, fixvalue);
    if(fixvalue != 0)
      addUndoPresolve(lp, FALSE, rownr, fixvalue, 0, 0);
    mult = -1;
  }
  else {
    mult = 1;
    fixvalue = 0;
  }

  /* Loop over rows to update statistics */
  ix = mat->row_end[rownr - 1];
  ie = mat->row_end[rownr];
  for(; ix < ie; ix++) {

   /* Retrieve row data and adjust RHS if we are deleting a variable */
    i     = ROW_MAT_COLNR(ix);
    Value = ROW_MAT_VALUE(ix);
    if(Value == 0)
      continue;

    if(remove && (fixvalue != 0)) {
      fixprod = Value*fixvalue;
      lp->orig_obj[i] -= fixprod;
      my_roundzero(lp->orig_obj[i], psdata->epsvalue);
      lp->presolve_undo->fixed_obj[i] += fixprod;
    }

   /* Prepare for further processing */
    Value = my_chsign(chsign, Value);
    isneg = (MYBOOL) (Value < 0);

   /* Reduce row variable counts if we are removing the variable */
    if(!isActiveLink(ps->varmap, i))
      continue;
    if(remove) {
      if(isneg) {
        ps->negcount[i]--;
      }
      else {
        ps->plucount[i]--;
      }
      if((lobound < 0) && (upbound >= 0)) {
        ps->pluneg[i]--;
      }
    }

   /* Compute associated constraint contribution values */
    upfinite = (MYBOOL) (upbound < lp->infinite);
    lofinite = (MYBOOL) (lobound > -lp->infinite);
    if(upfinite || lofinite) {
      if(remove)
        ps->infcount[i]--;
      else
        ps->infcount[i]++;
    }
    upvalue = my_if(upfinite, Value*upbound, my_chsign(isneg, lp->infinite));
    lovalue = my_if(lofinite, Value*lobound, my_chsign(isneg, -lp->infinite));

   /* Cumulate effective upper column bound (only bother with non-finite bound) */
    if(isneg) {
      if((ps->negupper[i] < lp->infinite) && lofinite) {
        ps->negupper[i] += mult*lovalue;
        ps->negupper[i] = presolve_roundrhs(lp, ps->negupper[i], FALSE);
      }
      else if(remove && !lofinite)
        doupdate = TRUE;
      else
        ps->negupper[i] = lp->infinite;
    }
    else {
      if((ps->pluupper[i] < lp->infinite) && upfinite) {
        ps->pluupper[i] += mult*upvalue;
        ps->pluupper[i] = presolve_roundrhs(lp, ps->pluupper[i], FALSE);
      }
      else if(remove && !upfinite)
        doupdate = TRUE;
      else
        ps->pluupper[i] = lp->infinite;
    }

   /* Cumulate effective lower column bound (only bother with non-finite bound) */
    if(isneg) {
      if((ps->neglower[i] > -lp->infinite) && upfinite) {
        ps->neglower[i] += mult*upvalue;
        ps->neglower[i] = presolve_roundrhs(lp, ps->neglower[i], TRUE);
      }
      else if(remove && !upfinite)
        doupdate = TRUE;
      else
        ps->neglower[i] = -lp->infinite;
    }
    else {
      if((ps->plulower[i] > -lp->infinite) && lofinite) {
        ps->plulower[i] += mult*lovalue;
        ps->plulower[i] = presolve_roundrhs(lp, ps->plulower[i], TRUE);
      }
      else if(remove && !lofinite)
        doupdate = TRUE;
      else
        ps->plulower[i] = -lp->infinite;
    }

   /* Validate consistency of eliminated singleton */
    if(remove && ((i == 0) || (ps->next[i][0] == 1)) && !psdata->forceupdate) {
      presolve_range(lp, i, ps, &lovalue, &upvalue);
      Value = get_mat(lp, 0, i);
      if((upvalue < Value) ||
         (lovalue > Value)) {
        report(lp, IMPORTANT, "presolve: Row %s (%g << %g) infeasibility in column %s (OF=%g)\n",
                              get_row_name(lp, rownr), lovalue, upvalue, get_col_name(lp, i), Value);
        return( FALSE );
      }
    }
  }
  if(remove) {
    psdata->forceupdate |= doupdate;
    if(tally != NULL)
      (*tally)++;
  }
  return( TRUE );
}


STATIC int presolve_colsingleton(presolverec *psdata, int i, int j, int *count)
{
  lprec    *lp = psdata->lp;
  REAL     RHlow, RHup, LObound, UPbound, Value;

#ifdef Paranoia
  if(!isActiveLink(psdata->cols->varmap, j))
    report(lp, SEVERE, "presolve_colsingleton: Nothing to do, column %d was eliminated earlier\n",
                       j);
#endif

  Value = get_mat(lp,i,j);
  if(Value == 0)
    return( RUNNING );

  /* Initialize and identify semicontinuous variable */
  LObound = get_lowbo(lp, j);
  UPbound = get_upbo(lp, j);
  if(is_semicont(lp, j) && (UPbound > LObound)) {
    if(LObound > 0)
      LObound = 0;
    else if(UPbound < 0)
      UPbound = 0;
  }

  /* Get singleton variable bounds */
  RHlow = get_rh_lower(lp, i);
  RHup  = get_rh_upper(lp, i);
  if(!presolve_singletonbounds(psdata, i,j, &RHlow, &RHup, &Value))
    return( presolve_setstatus(psdata, INFEASIBLE) );

  if(presolve_coltighten(psdata, j, RHlow, RHup, count))
    return( RUNNING );
  else
    return( presolve_setstatus(psdata, INFEASIBLE) );
}

STATIC MYBOOL presolve_colfix(presolverec *psdata, int colnr, REAL newvalue, MYBOOL remove, int *tally)
{
  lprec    *lp = psdata->lp;
  int      i, ix, ie;
  MYBOOL   isneg, lofinite, upfinite, doupdate = FALSE, doOF = TRUE;
  REAL     lobound, upbound, lovalue, upvalue,
           Value, fixvalue, mult;
  MATrec   *mat = lp->matA;
  psrec    *ps = psdata->rows;
  REAL     *value;
  int      *rownr;

  /* Set "fixed" value in case we are deleting a variable */
  upbound = get_upbo(lp, colnr);
  lobound = get_lowbo(lp, colnr);
  if(remove) {
    if(upbound-lobound < psdata->epsvalue) {
      if((newvalue > lobound) && (newvalue < upbound))
        fixvalue = newvalue;
      else
        fixvalue = lobound;
    }
    else {
      if(my_infinite(lp, newvalue) && (get_mat(lp, 0, colnr) == 0))
        fixvalue = ((lobound <= 0) && (upbound >= 0) ? 0 : MIN(upbound, lobound));
      else
        fixvalue = newvalue;
    }
#if 1 /* Fast normal version */
    set_bounds(lp, colnr, fixvalue, fixvalue);
#else /* Slower version that can be used for debugging/control purposes */
    presolve_coltighten(psdata, colnr, fixvalue, fixvalue, NULL);
    lobound = fixvalue;
    upbound = fixvalue;
#endif
    if(fixvalue != 0)
      addUndoPresolve(lp, TRUE, colnr, fixvalue, 0, 0);
    mult = -1;
  }
  else {
    mult = 1;
    fixvalue = 0;
  }

  /* Adjust semi-continuous variable bounds to zero-base */
  if(is_semicont(lp, colnr) && (upbound > lobound)) {
    if(lobound > 0)
      lobound = 0;
    else if(upbound < 0)
      upbound = 0;
  }

  /* Loop over rows to update statistics */
  ix = mat->col_end[colnr - 1];
  ie = mat->col_end[colnr];
  rownr = &COL_MAT_ROWNR(ix);
  value = &COL_MAT_VALUE(ix);
  for(; doOF || (ix < ie);
      ix++, rownr += matRowColStep, value += matValueStep) {

   /* Retrieve row data and adjust RHS if we are deleting a variable */
Restart:
    if(doOF) {
      i = 0;
      Value = lp->orig_obj[colnr];
    }
    else {
      i = *rownr;
      Value = *value;
      if(!isActiveLink(ps->varmap, i))
        continue;
    }
    if(Value == 0)
      goto BlockEnd;

    if(remove && (fixvalue != 0))
      presolve_adjustrhs(psdata, i, Value*fixvalue, psdata->epsvalue);

   /* Prepare for further processing */
    Value = my_chsign(is_chsign(lp, i), Value);
    isneg = (MYBOOL) (Value < 0);

   /* Reduce row variable counts if we are removing the variable */
    if(remove == TRUE) {
      if(isneg) {
        ps->negcount[i]--;
      }
      else {
        ps->plucount[i]--;
      }
      if((lobound < 0) && (upbound >= 0)) {
        ps->pluneg[i]--;
      }
    }

   /* Compute associated constraint contribution values */
    upfinite = (MYBOOL) (upbound < lp->infinite);
    lofinite = (MYBOOL) (lobound > -lp->infinite);
    if(upfinite || lofinite) {
      if(remove)
        ps->infcount[i]--;
      else
        ps->infcount[i]++;
    }
    upvalue = my_if(upfinite, Value*upbound, my_chsign(isneg, lp->infinite));
    lovalue = my_if(lofinite, Value*lobound, my_chsign(isneg, -lp->infinite));

   /* Cumulate effective upper row bound (only bother with non-finite bound) */
    if(isneg) {
      if((ps->negupper[i] < lp->infinite) && lofinite) {
        ps->negupper[i] += mult*lovalue;
        ps->negupper[i] = presolve_roundrhs(lp, ps->negupper[i], FALSE);
      }
      else if(remove && !lofinite)
        doupdate = TRUE;
      else
        ps->negupper[i] = lp->infinite;
    }
    else {
      if((ps->pluupper[i] < lp->infinite) && upfinite) {
        ps->pluupper[i] += mult*upvalue;
        ps->pluupper[i] = presolve_roundrhs(lp, ps->pluupper[i], FALSE);
      }
      else if(remove && !upfinite)
        doupdate = TRUE;
      else
        ps->pluupper[i] = lp->infinite;
    }

   /* Cumulate effective lower row bound (only bother with non-finite bound) */
    if(isneg) {
      if((ps->neglower[i] > -lp->infinite) && upfinite) {
        ps->neglower[i] += mult*upvalue;
        ps->neglower[i] = presolve_roundrhs(lp, ps->neglower[i], TRUE);
      }
      else if(remove && !upfinite)
        doupdate = TRUE;
      else
        ps->neglower[i] = -lp->infinite;
    }
    else {
      if((ps->plulower[i] > -lp->infinite) && lofinite) {
        ps->plulower[i] += mult*lovalue;
        ps->plulower[i] = presolve_roundrhs(lp, ps->plulower[i], TRUE);
      }
      else if(remove && !lofinite)
        doupdate = TRUE;
      else
        ps->plulower[i] = -lp->infinite;
    }

   /* Validate consistency of eliminated singleton */
    if(remove && ((i == 0) || (ps->next[i][0] == 1)) && !psdata->forceupdate) {
      if(i == 0) {
        lovalue = get_rh_lower(lp, i);
        upvalue = get_rh_upper(lp, i);
        report(lp, DETAILED, "presolve_colfix: Objective determined by presolve as %18g\n",
                             (is_maxim(lp) ? upvalue : lovalue));
      }
      else {
        presolve_range(lp, i, ps, &lovalue, &upvalue);
#if 1
        Value = 0;
#else
        Value = MAX(fabs(upvalue), fabs(lovalue));
        Value = psdata->epsvalue * MAX(1, Value);
#endif
        if((upvalue < get_rh_lower(lp, i)-Value) ||
           (lovalue > get_rh_upper(lp, i)+Value)) {
          report(lp, NORMAL, "presolve_colfix: Variable %s (%g << %g) infeasibility in row %s (%g << %g)\n",
                              get_col_name(lp, colnr), lovalue, upvalue,
                              get_row_name(lp, i), get_rh_lower(lp,i), get_rh_upper(lp, i));
          return( FALSE );
        }
      }
    }
BlockEnd:
    if(doOF) {
      doOF = FALSE;
      if(ix < ie)
        goto Restart;
    }

  }
  if(remove) {
    psdata->forceupdate |= doupdate;
    if(tally != NULL)
      (*tally)++;
  }
  return( TRUE );
}

/* Delete the columns of the specified row, but make sure we don't delete SOS variables.
   Note that we cannot use presolve_nextcol() here, since the variables are deleted. */
STATIC int presolve_rowfixzero(presolverec *psdata, int rownr, int *nv)
{
  lprec  *lp = psdata->lp;
  MATrec *mat = lp->matA;
  int    ix, jx, ib = mat->row_end[rownr-1];
  for(ix = mat->row_end[rownr]-1; ix >= ib; ix--) {
    jx = ROW_MAT_COLNR(ix);
    if(isActiveLink(psdata->cols->varmap, jx)) {
      if(!presolve_colfix(psdata, jx, 0.0, TRUE, nv))
        return( presolve_setstatus(psdata, INFEASIBLE) );
      if(presolve_candeletevar(psdata, jx))
        presolve_colremove(psdata, jx, TRUE);
    }
  }
#ifdef xxParanoia
  if(!presolve_debugrowtallies(psdata))
    return( INFEASIBLE );
#endif
  return( RUNNING );
}

/* Function to find if a variable can be fixed based on considering the dual */
STATIC MYBOOL presolve_colfixdual(presolverec *psdata, int colnr, REAL *fixValue, int *status)
{
  lprec   *lp = psdata->lp;
  MYBOOL  hasOF, isMI, isDualFREE = TRUE;
  int     i, ix, ie, *rownr, signOF;
  REAL    *value, loX, upX, eps = psdata->epsvalue;
  MATrec  *mat = lp->matA;

  /* First check basic variable range */
  loX = get_lowbo(lp, colnr);
  upX = get_upbo(lp, colnr);
  if(((loX < 0) && (upX > 0)) ||
     (fabs(upX-loX) < lp->epsvalue) ||
     SOS_is_member_of_type(lp->SOS, colnr, SOSn))
    return( FALSE );
  isMI = (MYBOOL) (upX <= 0);

  /* Retrieve OF (standard form assuming maximization) */
  ix = mat->col_end[colnr - 1];
  ie = mat->col_end[colnr];
  rownr = &COL_MAT_ROWNR(ix);
  value = &COL_MAT_VALUE(ix);
  hasOF = isnz_origobj(lp, colnr);
  if(hasOF)
    signOF = my_sign(lp->orig_obj[colnr]);
  else
    signOF = 0;

  /* Loop over all constraints involving active variable (standard form with LE constraints)*/
  for(; (ix < ie) && isDualFREE;
      ix++, rownr += matRowColStep, value += matValueStep) {
    i = *rownr;
    if(!isActiveLink(psdata->rows->varmap, i))
      continue;
    if(presolve_rowlength(psdata, i) == 1) {
      REAL val = my_chsign(is_chsign(lp, i), *value),
           loR = get_rh_lower(lp, i),
           upR = get_rh_upper(lp, i);
      if(!presolve_singletonbounds(psdata, i, colnr, &loR, &upR, &val)) {
        *status = presolve_setstatus(psdata, INFEASIBLE);
        return( FALSE );
      }
      if(loR > loX + psdata->epsvalue)
        loX = presolve_roundrhs(lp, loR, TRUE);
      if(upR < upX - psdata->epsvalue)
        upX = presolve_roundrhs(lp, upR, FALSE);
      continue;
    }
    else
      isDualFREE = my_infinite(lp, get_rh_range(lp, i)) ||                                          /* Explicitly free */
                   ((presolve_sumplumin(lp, i, psdata->rows, TRUE)-eps <= get_rh_upper(lp, i)) &&   /* Implicitly free */
                    (presolve_sumplumin(lp, i, psdata->rows, FALSE)+eps >= get_rh_lower(lp, i)));
    if(isDualFREE) {
      if(signOF == 0)  /* Test on the basis of identical signs in the constraints */
        signOF = my_sign(*value);
      else             /* Test on the basis of constraint sign equal to OF sign */
        isDualFREE = (MYBOOL) (signOF == my_sign(*value));
    }
  }

  /* Set fixing value if we were successful */
  if(isDualFREE) {
    if(signOF == 0) {
      SETMAX(loX, 0);
      *fixValue = MIN(loX, upX);
    }
    else if(signOF > 0) {
      if(my_infinite(lp, loX))
        isDualFREE = FALSE;
      else {
        if(is_int(lp, colnr))
          *fixValue = ceil(loX-PRESOLVE_EPSVALUE);
        else
          *fixValue = loX;
      }
    }
    else {
      if(my_infinite(lp, upX))
        isDualFREE = FALSE;
      else {
        if(is_int(lp, colnr) && (upX != 0))
          *fixValue = floor(upX+PRESOLVE_EPSVALUE);
        else
          *fixValue = upX;
      }
    }
    if((*fixValue != 0) && SOS_is_member(lp->SOS, 0, colnr))
      return( FALSE );

  }

  return( isDualFREE );
}

#if 0
STATIC MYBOOL presolve_probefix01(presolverec *psdata, int colnr, REAL *fixvalue)
{
  lprec    *lp = psdata->lp;
  int      i, ix, item;
  REAL     loLim, absvalue, epsvalue = psdata->epsvalue;
  MATrec   *mat = lp->matA;
  MYBOOL   chsign, canfix = FALSE;

  if(!is_binary(lp, colnr))
    return( canfix );

  /* Loop over all active rows to search for fixing opportunity */
  item = 0;
  for(ix = presolve_nextrow(psdata, colnr, &item);
      (ix >= 0) && !canfix;
      ix = presolve_nextrow(psdata, colnr, &item)) {
    i = COL_MAT_ROWNR(ix);
    *fixvalue = COL_MAT_VALUE(ix);
    chsign = is_chsign(lp, i);

    /* First check the lower bound of the normalized constraint */
    loLim = presolve_sumplumin(lp, i, psdata->rows, chsign);
    loLim = my_chsign(chsign, loLim);
    absvalue = fabs(*fixvalue);
    canfix = (MYBOOL) ((loLim + absvalue > lp->orig_rhs[i]+epsvalue*MAX(1, absvalue)));

    /* If we were unsuccessful in fixing above, try the upper bound
       of the normalized constraint - if it is finite */
    if(!canfix && !my_infinite(lp, get_rh_range(lp, i))) {
      loLim = presolve_sumplumin(lp, i, psdata->rows, (MYBOOL) !chsign);
      loLim = my_chsign(!chsign, loLim);
      *fixvalue = -(*fixvalue);
      canfix = (MYBOOL) ((loLim + absvalue > get_rh_range(lp, i)-lp->orig_rhs[i]+epsvalue*MAX(1, absvalue)));
    }
  }

  /* Check if we were successful in identifying fixing opportunity */
  if(canfix) {
    if(*fixvalue < 0)
      *fixvalue = 1;
    else
      *fixvalue = 0;
  }
  return( canfix );
}
#else
STATIC MYBOOL presolve_probefix01(presolverec *psdata, int colnr, REAL *fixvalue)
{
  lprec    *lp = psdata->lp;
  int      i, ix, item;
  REAL     loLim, upLim, range, absvalue, epsvalue = psdata->epsvalue, tolgap;
  MATrec   *mat = lp->matA;
  MYBOOL   chsign, status = FALSE;

  if(!is_binary(lp, colnr))
    return( status );

  /* Loop over all active rows to search for fixing opportunity.  The logic is that if a
     constraint gets violated by setting a variable at one of its bounds, then it can be
     fixed at its opposite bound. */
  item = 0;

  for(ix = presolve_nextrow(psdata, colnr, &item); (ix >= 0); ix = presolve_nextrow(psdata, colnr, &item)) {
    i = COL_MAT_ROWNR(ix);
    *fixvalue = COL_MAT_VALUE(ix);
    absvalue = fabs(*fixvalue);
    SETMIN(absvalue, 100);
    tolgap = epsvalue*MAX(1, absvalue);
    chsign = is_chsign(lp, i);

    /* Get the constraint value limits based on variable bounds, normalized to LE constraint */
    loLim = presolve_sumplumin(lp, i, psdata->rows, FALSE);
    upLim = presolve_sumplumin(lp, i, psdata->rows, TRUE);
    if(chsign) {
      loLim = my_chsign(chsign, loLim);
      upLim = my_chsign(chsign, upLim);
      swapREAL(&loLim, &upLim);
    }

    /* Check the upper constraint bound for possible violation if the value were to be fixed at 1 */
    if(loLim + *fixvalue > lp->orig_rhs[i]+tolgap) {
      if(*fixvalue < 0)
        presolve_setstatus(psdata, INFEASIBLE);
      *fixvalue = 0;
      break;
    }

    /* Check the lower constraint bound for possible violation if the value were to be fixed at 1 */
    range = get_rh_range(lp, i);
    if(!my_infinite(lp, range) &&
       (upLim + *fixvalue < lp->orig_rhs[i]-range-tolgap)) {
      if(*fixvalue > 0)
        presolve_setstatus(psdata, INFEASIBLE);
      *fixvalue = 0;
      break;
    }

    /* Check if we have to fix the value at 1 to avoid constraint infeasibility */
    if(psdata->rows->infcount[i] >= 1)
      continue;
    if(((*fixvalue < 0) && (upLim + *fixvalue >= loLim-tolgap) && (upLim > lp->orig_rhs[i]+tolgap)) ||
       ((*fixvalue > 0) && (loLim + *fixvalue <= upLim+tolgap) && (loLim < lp->orig_rhs[i]-range-tolgap) && !my_infinite(lp, range))) {
      *fixvalue = 1;
      break;
    }
  }
  status = (MYBOOL) (ix >= 0);

  /* Returns TRUE if fixing opportunity was identified */
  return( status );
}
#endif

STATIC int presolve_probetighten01(presolverec *psdata, int colnr)
{
  lprec    *lp = psdata->lp;
  MYBOOL   chsign;
  int      i, ix, item, n = 0;
  REAL     upLim, value, absvalue, epsvalue = psdata->epsvalue;
  MATrec   *mat = lp->matA;

#if 0 /* Handled in calling routine */
  if(!is_binary(lp, colnr))
    return( n );
#endif

  /* Loop over all active rows and do coefficient tightening for qualifying constraints */
  item = 0;
  for(ix = presolve_nextrow(psdata, colnr, &item); ix >= 0;
      ix = presolve_nextrow(psdata, colnr, &item)) {
    i = COL_MAT_ROWNR(ix);
    value = COL_MAT_VALUE(ix);
    chsign = is_chsign(lp, i);
    upLim = presolve_sumplumin(lp, i, psdata->rows, (MYBOOL) !chsign);
    upLim = my_chsign(chsign, upLim);

    /* Does this constraint qualify for coefficient tightening? */
    absvalue = fabs(value);
    if(upLim - absvalue < lp->orig_rhs[i]-epsvalue*MAX(1, absvalue)) {
      REAL delta = lp->orig_rhs[i] - upLim;
      lp->orig_rhs[i] = upLim;
      upLim = value - my_chsign(value < 0, delta);
      COL_MAT_VALUE(ix) = upLim;
      if(my_sign(value) != my_sign(upLim)) {
        if(chsign) {
          psdata->rows->negcount[i]--;
          psdata->rows->plucount[i]++;
        }
        else {
          psdata->rows->negcount[i]++;
          psdata->rows->plucount[i]--;
        }
      }
      n++;
    }
  }
  return( n );
}

STATIC int presolve_mergerows(presolverec *psdata, int *nRows, int *nSum)
{
  lprec *lp = psdata->lp;
  MYBOOL candelete;
  int    status = RUNNING, item1, item2,
         firstix, RT1, RT2, i, ix, iix, j, jjx, n = 0;
  REAL   Value1, Value2, bound;
  MATrec *mat = lp->matA;

  for(i = lastActiveLink(psdata->rows->varmap); (i > 0) && (status == RUNNING); ) {

    /* First scan for rows with identical row lengths */
    ix = prevActiveLink(psdata->rows->varmap, i);
    if(ix == 0)
      break;

    /* Don't bother about empty rows or row singletons, since they are
       handled by PRESOLVE_ROWS */
    j = presolve_rowlength(psdata, i);
    if(j <= 1) {
      i = ix;
      continue;
    }

#if 0
    /* Enable this to scan all rows back */
    RT2 = lp->rows;

    /* Check abort since this section can be pretty "expensive" */
    if(!presolve_statuscheck(psdata, &status))
      return( status );
#else
    RT2 = 2+1;
#endif
    firstix = ix;
    for(RT1 = 0; (ix > 0) && (RT1 < RT2) && (status == RUNNING);
        ix = prevActiveLink(psdata->rows->varmap, ix), RT1++)  {
      candelete = FALSE;
      if(presolve_rowlength(psdata, ix) != j)
        continue;

      /* Check if the beginning columns are identical; if not, continue */
      item1 = 0;
      iix = presolve_nextcol(psdata, ix, &item1);
      item2 = 0;
      jjx = presolve_nextcol(psdata, i,  &item2);

      if(ROW_MAT_COLNR(iix) != ROW_MAT_COLNR(jjx))
        continue;

      /* We have a candidate row; check if the entries have a fixed non-zero ratio */
      Value1 = get_mat_byindex(lp, iix, TRUE, FALSE);
      Value2 = get_mat_byindex(lp, jjx, TRUE, FALSE);
      bound = Value1 / Value2;
      Value1 = bound;

      /* Loop over remaining entries */
      jjx = presolve_nextcol(psdata, i, &item2);
      for(; (jjx >= 0) && (Value1 == bound);
          jjx = presolve_nextcol(psdata, i, &item2)) {
        iix = presolve_nextcol(psdata, ix, &item1);
        if(ROW_MAT_COLNR(iix) != ROW_MAT_COLNR(jjx))
          break;
        Value1 = get_mat_byindex(lp, iix, TRUE, FALSE);
        Value2 = get_mat_byindex(lp, jjx, TRUE, FALSE);

        /* If the ratio is different from the reference value we have a mismatch */
        Value1 = Value1 / Value2;
        if(bound == lp->infinite)
          bound = Value1;
        else if(fabs(Value1 - bound) > psdata->epsvalue)
          break;
      }

      /* Check if we found a match (we traversed all active columns without a break) */
      if(jjx < 0) {

        /* Get main reference values */
        Value1 = lp->orig_rhs[ix];
        Value2 = lp->orig_rhs[i] * bound;

        /* First check for inconsistent equalities */
        if((fabs(Value1 - Value2) > psdata->epsvalue) &&
           ((get_constr_type(lp, ix) == EQ) && (get_constr_type(lp, i) == EQ))) {
          report(lp, NORMAL, "presolve_mergerows: Inconsistent equalities %d and %d found\n",
                             ix, i);
          status = presolve_setstatus(psdata, INFEASIBLE);
        }

        else {

          /* Update lower and upper bounds */
          if(is_chsign(lp, i) != is_chsign(lp, ix))
            bound = -bound;

          Value1 = get_rh_lower(lp, i);
          if(Value1 <= -lp->infinite)
            Value1 *= my_sign(bound);
          else
            Value1 *= bound;
          my_roundzero(Value1, lp->epsdual);      /* Extra rounding tolerance *** */

          Value2 = get_rh_upper(lp, i);
          if(Value2 >= lp->infinite)
            Value2 *= my_sign(bound);
          else
            Value2 *= bound;
          my_roundzero(Value2, lp->epsdual);      /* Extra rounding tolerance *** */

          if((bound < 0))
            swapREAL(&Value1, &Value2);

          bound = get_rh_lower(lp, ix);
          if(Value1 > bound + psdata->epsvalue)
            set_rh_lower(lp, ix, Value1);
          else
            Value1 = bound;
          bound = get_rh_upper(lp, ix);
          if(Value2 < bound - psdata->epsvalue)
            set_rh_upper(lp, ix, Value2);
          else
            Value2 = bound;

          /* Check results and make equality if appropriate */
          if(fabs(Value2-Value1) < psdata->epsvalue)
            presolve_setEQ(psdata, ix);
          else if(Value2 < Value1) {
            status = presolve_setstatus(psdata, INFEASIBLE);
          }

          /* Verify if we can continue */
          candelete = (MYBOOL) (status == RUNNING);
          if(!candelete) {
            report(lp, NORMAL, "presolve: Range infeasibility found involving rows %s and %s\n",
                                get_row_name(lp, ix), get_row_name(lp, i));
          }
        }
      }
      /* Perform i-row deletion if authorized */
      if(candelete) {
        presolve_rowremove(psdata, i, TRUE);
        n++;
        break;
      }
    }
    i = firstix;
  }
  (*nRows) += n;
  (*nSum)  += n;

  return( status );
}

STATIC MYBOOL presolve_reduceGCD(presolverec *psdata, int *nn, int *nb, int *nsum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   status = TRUE;
  int      i, jx, je, in = 0, ib = 0;
  LLONG    GCDvalue;
  REAL     *Avalue, Rvalue, epsvalue = psdata->epsvalue;
  MATrec   *mat = lp->matA;

  for(i = firstActiveLink(psdata->INTmap); i != 0; i = nextActiveLink(psdata->INTmap, i)) {

    /* Obtain the row GCD */
    jx = mat->row_end[i - 1];
    je = mat->row_end[i];
    Rvalue = ROW_MAT_VALUE(jx);
    GCDvalue = abs((int) Rvalue);
    jx++;
    if(jx < je)
    for(; (jx < je) && (GCDvalue > 1); jx++) {
      Rvalue = fabs(ROW_MAT_VALUE(jx));
      GCDvalue = gcd((LLONG) Rvalue, GCDvalue, NULL, NULL);
    }

    /* Reduce the coefficients, if possible */
    if(GCDvalue > 1) {
      jx = mat->row_end[i - 1];
      je = mat->row_end[i];
      for(; jx < je; jx++) {
        Avalue = &ROW_MAT_VALUE(jx);
        *Avalue /= GCDvalue;
        in++;
      }
      Rvalue = (lp->orig_rhs[i] / GCDvalue) + epsvalue;
      lp->orig_rhs[i] = floor(Rvalue);
      Rvalue = fabs(lp->orig_rhs[i]-Rvalue);
      if(is_constr_type(lp, i, EQ) && (Rvalue > epsvalue)) {
        report(lp, NORMAL, "presolve_reduceGCD: Infeasible equality constraint %d\n", i);
        status = FALSE;
        break;
      }
      if(!my_infinite(lp, lp->orig_upbo[i]))
        lp->orig_upbo[i] = floor(lp->orig_upbo[i] / GCDvalue);
      ib++;
    }
  }
  if(status && (in > 0))
    report(lp, DETAILED, "presolve_reduceGCD: Did %d constraint coefficient reductions.\n", in);

  (*nn)   += in;
  (*nb)   += ib;
  (*nsum) += in + ib;

  return( status );
}

STATIC int presolve_knapsack(presolverec *psdata, int *nn)
{
  lprec *lp = psdata->lp;
  int    m, n, i, ix, j, jx, colnr, *rownr = NULL,
         status = RUNNING;
  REAL   *colOF = lp->orig_obj, value, *ratio = NULL;
  LLrec  *map = psdata->EQmap;
  MATrec *mat = lp->matA;

  /* Check if it is worth trying */
  m = mat->row_end[0];
  if((map->count == 0) || (m < 2))
    return( status );

  /* Get the OF row */
  allocINT(lp, &rownr,  map->count+1, FALSE);
  allocREAL(lp, &ratio, map->count+1, FALSE);

  /* Loop over each row trying to find equal entries in the OF */
  rownr[0] = 0;
  for(i = firstActiveLink(map); i != 0; i = nextActiveLink(map, i)) {
    if(get_rh(lp, i) <= 0)
      continue;
    jx = mat->row_end[i];
    n = 0;
    for(j = mat->row_end[i-1]; j  < jx; j++, n++) {
      colnr = ROW_MAT_COLNR(j);
      value = ROW_MAT_VALUE(j);
      if(colOF[colnr] == 0)
        break;
      if(n == 0) {
        ratio[0] = colOF[colnr] / value;
      }
      else if(fabs(value * ratio[0] - colOF[colnr]) > psdata->epsvalue) {
        n = -1;
        break;
      }
    }
    /* Register row if we were successful (and row long enough) */
    if(n >= 2) {
      ix = ++rownr[0];
      rownr[ix] = i;
      ratio[ix] = ratio[0];
    }
  }
  n = rownr[0];
  if(n == 0)
    goto Finish;

  /* Process the identified rows, eliminating the OF value */
  for(ix = 1; ix <= n; ix++) {
    i = rownr[ix];
    jx = mat->row_end[i];
    for(j = mat->row_end[i-1]; j  < jx; j++) {
      colnr = ROW_MAT_COLNR(j);
      colOF[colnr] = 0;
    }
  }

  /* Update key mapper structures */
  j = lp->columns;
  psdata->cols->varmap = cloneLink(psdata->cols->varmap, j+n, TRUE);
  psdata->forceupdate = TRUE;

  /* Finally, add helper columns */
  for(ix = 1; ix <= n; ix++) {
    i = rownr[ix];
    rownr[0] = 0;
    colOF[0] = my_chsign(is_maxim(lp), ratio[ix]);
    rownr[1] = i;
    colOF[1] = -1;
    value = get_rh(lp, i);
/*    j = get_constr_type(lp, i); */
    add_columnex(lp, 2, colOF, rownr);
    set_bounds(lp, lp->columns, value, value);
/*    presolve_setEQ(psdata, i); */
    set_rh(lp, i, 0);
    appendLink(psdata->cols->varmap, j+ix);
  }
  presolve_validate(psdata, TRUE);

  /* Clean up before returning */
Finish:
  FREE(rownr);
  FREE(ratio);
  (*nn) += n;

  return( status );
}

STATIC MYBOOL presolve_invalideq2(lprec *lp, presolverec *psdata)
{
  int    jx, jjx, i = 0, item;
  MATrec *mat = lp->matA;
  MYBOOL error = FALSE;

  do {

    if(i == 0)
      i = firstActiveLink(psdata->EQmap);
    else
      i = nextActiveLink(psdata->EQmap, i);
    if(i == 0)
      return( error );

    /* Get the row index of the first 2-element equality */
    for(; i > 0; i = nextActiveLink(psdata->EQmap, i))
      if(presolve_rowlength(psdata, i) == 2)
        break;
    if(i == 0)
      return( error );

    /* Get the first column */
    item = 0;
    jx  = presolve_nextcol(psdata, i, &item);
    if(jx < 0)
      error = TRUE;
    jx  = ROW_MAT_COLNR(jx);

    /* Get the second column */
    jjx = presolve_nextcol(psdata, i, &item);
    if(jjx < 0)
      error = AUTOMATIC;
  } while(!error);

  return( error );
}

/* Callback to obtain the non-zero rows of equality constraints */
int BFP_CALLMODEL presolve_getcolumnEQ(lprec *lp, int colnr, REAL nzvalues[], int nzrows[], int mapin[])
{
  int    i, ib, ie, nn = 0;
  MATrec *mat = lp->matA;

  ib = mat->col_end[colnr-1];
  ie = mat->col_end[colnr];
  for(; ib < ie; ib++) {
    i = COL_MAT_ROWNR(ib);
    if(!is_constr_type(lp, i, EQ) ||  /* It has to be an equality constraint         */
       (mapin[i] == 0))               /* And it should not already have been deleted */
      continue;
    if(nzvalues != NULL) {
      nzrows[nn] = mapin[i];
      nzvalues[nn] = COL_MAT_VALUE(ib);
    }
    nn++;
  }
  return( nn );
}
STATIC int presolve_singularities(presolverec *psdata, int *nn, int *nr, int *nv, int *nSum)
{
  lprec *lp = psdata->lp;
  int i, j, n, *rmapin = NULL, *rmapout = NULL, *cmapout = NULL;

  if(lp->bfp_findredundant(lp, 0, NULL, NULL, NULL) == 0)
    return( 0 );

  /* Create condensed row map */
  allocINT(lp, &rmapin, lp->rows+1, TRUE);
  allocINT(lp, &rmapout, psdata->EQmap->count+1, FALSE);
  allocINT(lp, &cmapout, lp->columns+1, FALSE);
  n = 0;
  for(i = firstActiveLink(psdata->EQmap); i != 0; i = nextActiveLink(psdata->EQmap, i)) {
    n++;
    rmapout[n] = i;
    rmapin[i]  = n;
  }
  rmapout[0] = n;
  n = 0;
  for(i = firstActiveLink(psdata->cols->varmap); i != 0; i = nextActiveLink(psdata->cols->varmap, i)) {
    n++;
    cmapout[n]  = i;
  }
  cmapout[0] = n;

  /* Do the rank-revealing factorization */
  n = lp->bfp_findredundant(lp, psdata->EQmap->count, presolve_getcolumnEQ, rmapin, cmapout);

  /* Delete the redundant rows */
  for(i = 1; i <= n; i++) {
    j = rmapin[i];
    j = rmapout[j];
    presolve_rowremove(psdata, j, TRUE);
  }
  (*nn)   += n;
  (*nr)   += n;
  (*nSum) += n;

  /* Clean up */
  FREE(rmapout);
  FREE(rmapin);
  FREE(cmapout);

  return( n );
}

STATIC int presolve_elimeq2(presolverec *psdata, int *nn, int *nr, int *nc, int *nSum)
{
  lprec     *lp = psdata->lp;
  int       n, i, jx, jjx, k, item, *plucount, *negcount, colplu, colneg,
            iCoeffChanged = 0, iRowsRemoved = 0, iVarsFixed = 0, nrows = lp->rows,
            status = RUNNING, *colindex = NULL;
  MYBOOL    freshupdate;
  REAL      Coeff1, Coeff2, Value1, Value2, lobound, upbound, bound, test, product,
            *colvalue = NULL, *delvalue = NULL, *colitem;
  MATrec    *mat = lp->matA, *rev = NULL;
  DeltaVrec *DV = NULL;
  LLrec     *EQ2 = NULL;

  /* See if there is anything to do */
  if(psdata->EQmap->count == 0) {
    (*nSum) = 0;
    return( status );
  }

  /* Tally counts */
  createLink(lp->rows, &EQ2, NULL);
  if((EQ2 == NULL) || !allocREAL(lp, &colvalue, nrows+1, FALSE) ||
                      !allocREAL(lp, &delvalue, nrows+1, FALSE))
    goto Finish;
  for(i = firstActiveLink(psdata->EQmap); i > 0; i = nextActiveLink(psdata->EQmap, i)) {
    if(presolve_rowlength(psdata, i) == 2)
      appendLink(EQ2, i);
  }
  if(EQ2->count == 0)
    goto Finish;
  n = 0;

  /* Do the elimination loop for all identified 2-element equalities */
  for(i = firstActiveLink(EQ2); i > 0; i = nextActiveLink(EQ2, i)) {

    /* Check if the constraint has been modified by a previous elimination */
    if(presolve_rowlength(psdata, i) != 2)
      continue;

    /* Get the column indeces of NZ-values of the "pivot" row */
    item = 0;
    jx  = presolve_nextcol(psdata, i, &item);   /* Eliminated variable coefficient    b */
#ifdef Paranoia
    if(jx < 0)
      report(lp, SEVERE, "presolve_elimeq2: No qualifying %dst column was found in row %s (ostensible length %d)\n",
                         1, get_row_name(lp, i), presolve_rowlength(psdata, i));
#endif
    Coeff2 = ROW_MAT_VALUE(jx);
    jx  = ROW_MAT_COLNR(jx);
    jjx = presolve_nextcol(psdata, i, &item);  /* Non-eliminated variable coefficient a */
#ifdef Paranoia
    if(jjx < 0)
      report(lp, SEVERE, "presolve_elimeq2: No qualifying %dnd column was found in row %s (ostensible length %d)\n",
                          2, get_row_name(lp, i), presolve_rowlength(psdata, i));
#endif
    Coeff1 = ROW_MAT_VALUE(jjx);
    jjx = ROW_MAT_COLNR(jjx);

    /* Check if at least one of the coefficients is large enough to preserve stability;
       use opposing maximum column values for stability testing. */
    if((fabs(Coeff1) < psdata->epspivot*mat->colmax[jx]) &&
       ((fabs(Coeff1) != 1) && (fabs(Coeff2) != 1)) &&
       (fabs(Coeff2) < psdata->epspivot*mat->colmax[jjx]))
      continue;

    /* Cannot eliminate a variable if both are SOS members or SC variables */
    if((is_semicont(lp, jx) && is_semicont(lp, jjx)) ||
        (SOS_is_member(lp->SOS, 0, jx) && SOS_is_member(lp->SOS, 0, jjx)))
      continue;

    /* First check if we are allowed to swap; set swap "blockers" */
    k = 0;
    if(!is_int(lp, jx) && is_int(lp, jjx))
      k += 1;
    else if(!is_semicont(lp, jx) && is_semicont(lp, jjx))
      k += 2;
    else if(!SOS_is_member(lp->SOS, 0, jx) && SOS_is_member(lp->SOS, 0, jjx))
      k += 4;

    /* If there were no blockers, determine if we MUST swap the variable to be eliminated */
    if(k == 0) {
      if(is_int(lp, jx) && !is_int(lp, jjx))
        k += 8;
      else if(is_semicont(lp, jx) && !is_semicont(lp, jjx))
        k += 16;
      else if(SOS_is_member(lp->SOS, 0, jx) && !SOS_is_member(lp->SOS, 0, jjx))
        k += 32;

      /* If we are not forced to swap, decide if it otherwise makes sense - high order */
      if(k == 0) {
        if((fabs(Coeff2) < psdata->epspivot*mat->colmax[jjx]) &&
           (fabs(Coeff1) > psdata->epspivot*mat->colmax[jx]))
          k += 64;
        else if(presolve_collength(psdata, jx) > presolve_collength(psdata, jjx))
          k += 128;
      }

      /* If we are not forced to swap, decide if it otherwise makes sense - low order */
      if(k == 0) {
        Value2 = Coeff1/Coeff2;
#ifdef DualFeasibilityLogicEQ2
        if((Value2*lp->orig_obj[jx] < 0) &&
          (Value2*lp->orig_obj[jjx] > 0))                     /* Seek increased dual feasibility */
          k += 256;
#endif
#ifdef DivisorIntegralityLogicEQ2
        if((fabs(modf(Coeff2, &Value2)) >= lp->epsvalue) &&    /* Seek integrality of result */
           (fabs(modf(Coeff1, &Value2)) < lp->epsvalue))
          k += 512;
        else if((fabs(fabs(Coeff2)-1) >= lp->epsvalue) &&    /* Seek integrality of divisor */
                 (fabs(fabs(Coeff1)-1) < lp->epsvalue))
          k += 1024;
#endif
      }

    }
    else
      k = 0;

    /* Perform variable index swap if indicated */
    if(k != 0) {
      swapINT(&jx, &jjx);
      swapREAL(&Coeff1, &Coeff2);
    }

    Value1 = lp->orig_rhs[i]/Coeff2; /* Delta constant term */
    Value2 = Coeff1/Coeff2;          /* Delta variable term */
    upbound = lp->orig_upbo[lp->rows+jx];
    lobound = lp->orig_lowbo[lp->rows+jx];
    if(lp->spx_trace) {
      report(lp, DETAILED, "Row %3d : Elim %g %s - %d\n", i, Coeff2, get_col_name(lp, jx), jx);
      report(lp, DETAILED, "          Keep %g %s - %d\n",    Coeff1, get_col_name(lp, jjx), jjx);
    }

    /* Get the coefficient vectors of the independent (jjx) and dependent (jx) columns;
      the dependent column will be deleted and reconstructed during postsolve. */
    freshupdate = (MYBOOL) ((colindex == NULL) || (colindex[jjx] == 0));
    if(freshupdate)
      mat_expandcolumn(mat, jjx, colvalue, NULL, TRUE);
    else
      mat_expandcolumn(rev, colindex[jjx], colvalue, NULL, FALSE);
    if((colindex == NULL) || (colindex[jx] == 0))
      mat_expandcolumn(mat, jx, delvalue, NULL, TRUE);
    else
      mat_expandcolumn(rev, colindex[jx], delvalue, NULL, FALSE);

    /* Add variable reconstruction information */
    addUndoPresolve(lp, TRUE, jx, Value1, Value2, jjx);

    /* If possible, tighten the bounds of the uneliminated variable based
       on the bounds of the eliminated variable. Also handle roundings
       and attempt precision management. */
    bound = lobound;
    k = nrows+jjx;
    if(bound > -lp->infinite) {
      bound = (lp->orig_rhs[i] - Coeff2*bound) / Coeff1;
      if(Value2 > 0) {
        test = lp->orig_upbo[k];
        if(bound < test - psdata->epsvalue)
          if(is_int(lp, jjx))
            lp->orig_upbo[k] = floor(bound + lp->epsint);
          else
            lp->orig_upbo[k] = presolve_roundrhs(lp, bound, FALSE);
      }
      else {
        test = lp->orig_lowbo[k];
        if(bound > test + psdata->epsvalue)
          if(is_int(lp, jjx))
            lp->orig_lowbo[k] = ceil(bound - lp->epsint);
          else
            lp->orig_lowbo[k] = presolve_roundrhs(lp, bound, TRUE);
      }
    }
    bound = upbound;
    if(bound < lp->infinite) {
      bound = (lp->orig_rhs[i] - Coeff2*bound) / Coeff1;
      if(Value2 < 0) {
        test = lp->orig_upbo[k];
        if(bound < test - psdata->epsvalue)
          if(is_int(lp, jjx))
            lp->orig_upbo[k] = floor(bound + lp->epsint);
          else
            lp->orig_upbo[k] = presolve_roundrhs(lp, bound, FALSE);
      }
      else {
        test = lp->orig_lowbo[k];
        if(bound > test + psdata->epsvalue)
          if(is_int(lp, jjx))
            lp->orig_lowbo[k] = ceil(bound - lp->epsint);
          else
            lp->orig_lowbo[k] = presolve_roundrhs(lp, bound, TRUE);
      }
    }

#ifdef Eq2Reldiff
    test = 2*lp->epsvalue;
#else
    test = psdata->epsvalue;
#endif
    if(/*(lp->orig_upbo[k] < lp->orig_lowbo[k]) ||*/
#ifdef Eq2Reldiff
       (fabs(my_reldiff(lp->orig_upbo[k],lp->orig_lowbo[k])) < test)) {
#else
       (fabs(lp->orig_upbo[k] - lp->orig_lowbo[k]) < test)) {
#endif
      my_roundzero(lp->orig_lowbo[k], test);
      lp->orig_upbo[k] = lp->orig_lowbo[k];
    }
    else {
      my_roundzero(lp->orig_upbo[k], test);
      my_roundzero(lp->orig_lowbo[k], test);
    }

    if(/*(upbound < lobound) ||*/
#ifdef Eq2Reldiff
       (fabs(my_reldiff(upbound, lobound)) < test)) {
#else
       (fabs(upbound - lobound) < test)) {
#endif
      my_roundzero(lobound, test);
      lp->orig_upbo[nrows+jx] = lobound;
      upbound = lobound;
    }

    /* Loop over the non-zero rows of the column (jx) to be eliminated;
      substitute jx-variable by updating rhs and jjx coefficients */
    colitem = colvalue;
    plucount = psdata->rows->plucount;
    negcount = psdata->rows->negcount;
    colplu = 0;
    colneg = 0;
    /* Count of non-zeros in the independent column jjx */
    item = presolve_collength(psdata, jjx) - 1;
    if(isnz_origobj(lp, jjx))
      item++;
    for(k = 0; k <= nrows; k++, colitem++) {

      bound = delvalue[k];
      if((k == i) || (bound == 0) ||
         ((k > 0) && !isActiveLink(psdata->rows->varmap, k)))
        continue;

      /* Do constraint and nz-count updates for the substituted variable */
      product = bound*Value1;

      /* "Raw"/unsigned data */
      presolve_adjustrhs(psdata, k, my_chsign(is_chsign(lp, k), product), test);

      /* Change back to signed part */
      if(*colitem != 0) {
        if(*colitem > 0) {
          colplu--;
          plucount[k]--;
        }
        else {
          colneg--;
          negcount[k]--;
        }
        if((lobound < 0) && (upbound >= 0)) {
          psdata->cols->pluneg[jjx]--;
          psdata->rows->pluneg[k]--;
        }
        item--;
      }
      (*colitem) -= bound*Value2;
      iCoeffChanged++;

      /* Update counts */
      if(fabs(*colitem) >= mat->epsvalue) {
        if(*colitem > 0) {
          colplu++;
          plucount[k]++;
        }
        else {
          colneg++;
          negcount[k]++;
        }
        if((lobound < 0) && (upbound >= 0)) {
          psdata->cols->pluneg[jjx]++;
          psdata->rows->pluneg[k]++;
        }
        item++;
      }
      else {
        *colitem = 0;
      }

      /* Also reduce count if the row contains the deleted variable */
      if(bound > 0)
        plucount[k]--;
      else
        negcount[k]--;
    }
    psdata->cols->plucount[jjx] += colplu;
    psdata->cols->negcount[jjx] += colneg;

    /* Save the new column */
    if(rev == NULL) {
      DV = createUndoLadder(lp, nrows, lp->columns / RESIZEFACTOR);
      rev = DV->tracker;
      rev->epsvalue = mat->epsvalue;
      allocINT(lp, &(rev->col_tag), mat->columns_alloc+1, FALSE);
      allocINT(lp, &colindex, lp->columns+1, TRUE);
      rev->col_tag[0] = 0;
    }
    n = rev->col_tag[0] = incrementUndoLadder(DV);
    mat_setcol(rev, n, 0, colvalue, NULL, FALSE, FALSE);
    rev->col_tag[n] = jjx;

    /* Save index to updated vector, but specially handle case where we have
      the same independent variable for multiple equations! */
    if(!freshupdate)
      rev->col_tag[colindex[jjx]] *= -1;
    colindex[jjx] = n;

    /* Delete the column dependent variable */
    jx = presolve_colremove(psdata, jx, FALSE);
    iVarsFixed++;

    /* Check if we have been lucky enough to have eliminated the independent
       variable via substitution of the dependent variable */
    if(item == 0) {
#ifdef Paranoia
      report(lp, DETAILED, "presolve_elimeq2: Was able to remove variables %d and %d in row %s\n",
                         jx, jjx, get_row_name(lp, i));
#endif
      if(presolve_colfix(psdata, jjx, 0.0, TRUE, nc))
        jjx = presolve_colremove(psdata, jjx, FALSE);
    }

    /* Delete the row */
    presolve_rowremove(psdata, i, FALSE);
    iRowsRemoved++;
  }

  /* Perform the column updates collected above */
  if(n > 0) {
    mat_mapreplace(mat, psdata->rows->varmap, psdata->cols->varmap, rev);
    presolve_validate(psdata, TRUE);
#ifdef PresolveForceUpdateMax
    mat_computemax(mat /* , FALSE */);
#endif
    psdata->forceupdate = TRUE;
  }

  /* Free work arrays */
Finish:
  if(DV != NULL)
    freeUndoLadder(&DV);
  freeLink(&EQ2);
  FREE(colvalue);
  FREE(delvalue);
  FREE(colindex);

  /* Update counters */
  (*nn)   += iCoeffChanged;
  (*nr)   += iRowsRemoved;
  (*nc)   += iVarsFixed;
  (*nSum) += iCoeffChanged + iRowsRemoved + iVarsFixed;

  return( status );
}

STATIC MYBOOL presolve_impliedfree(lprec *lp, presolverec *psdata, int colnr)
{
  int    i, ix, ie;
  REAL   Tlower, Tupper;
  MYBOOL status, rowbinds, isfree = FALSE;
  MATrec *mat = lp->matA;

  if(my_infinite(lp, get_lowbo(lp, colnr)) && my_infinite(lp, get_upbo(lp, colnr)))
    return( TRUE );

  ie = mat->col_end[colnr];
  for(ix = mat->col_end[colnr-1]; (isfree != (TRUE | AUTOMATIC)) && (ix < ie); ix++) {
    i = COL_MAT_ROWNR(ix);
    if(!isActiveLink(psdata->rows->varmap, i))
      continue;
    Tlower = get_rh_lower(lp, i);
    Tupper = get_rh_upper(lp, i);
    status = presolve_multibounds(psdata, i, colnr, &Tlower, &Tupper, NULL, &rowbinds);
    isfree = isfree | status | rowbinds;
  }

  return( (MYBOOL) (isfree == (TRUE | AUTOMATIC)) );
}

STATIC MYBOOL presolve_impliedcolfix(presolverec *psdata, int rownr, int colnr, MYBOOL isfree)
{
  lprec    *lp = psdata->lp;
  MYBOOL   signflip, undoadded = FALSE;
  MATrec   *mat = lp->matA;
  int      jx, i, ib, ie = mat->row_end[rownr];
  REAL     varLo = 0, varHi = 0, varRange, conRange = 0, matValue = 0, dual, RHS = lp->orig_rhs[rownr],
           pivot, matAij = mat_getitem(mat, rownr, colnr), *vecOF = lp->orig_obj;

  /* We cannot have semi-continuous or non-qualifying integers */
  if(is_semicont(lp, colnr) || is_SOS_var(lp, colnr))
    return( FALSE );
  if(is_int(lp, colnr)) {
    if(!isActiveLink(psdata->INTmap, rownr) || !is_presolve(lp, PRESOLVE_KNAPSACK))
      return( FALSE );
    /* colnr must have a coefficient equal to the smallest in the row */
    varRange = lp->infinite;
    i = 0;
    pivot = 0;
    for(ib = presolve_nextcol(psdata, rownr, &i); i != 0; ib = presolve_nextcol(psdata, rownr, &i)) {
      jx = ROW_MAT_COLNR(ib);
      dual = fabs(ROW_MAT_VALUE(ib));
      /* Check if we have the target column and save the pivot value */
      if(jx == colnr) {
        /* Always accept unit coefficient */
        if(fabs(dual - 1) < psdata->epsvalue)
          break;
        pivot = dual;
        /* Otherwise continue scan */
      }
      /* Cannot accept case where result can be fractional */
      else if((pivot > dual + psdata->epsvalue) ||
               ((pivot > 0) && (fabs(fmod(dual, pivot)) > psdata->epsvalue)))
        return( FALSE );
    }
  }

  /* Ascertain that the pivot value is large enough to preserve stability */
  pivot = matAij;
  if(fabs(pivot) < psdata->epspivot*mat->colmax[colnr])
    return( FALSE );

  /* Must ascertain that the row variables are not SOS'es; this is because
     the eliminated variable will be a function of another variable. */
  if(SOS_count(lp) > 0) {
    for(ib = mat->row_end[rownr-1]; ib < ie; ib++)
      if(SOS_is_member(lp->SOS, 0, ROW_MAT_COLNR(ib)))
        return( FALSE );
  }

  /* Calculate the dual value */
  dual = vecOF[colnr]/pivot;

  /* Here we have free variable in an equality constraint; this means we can
     can adjust the OF for the deleted variable and also delete the constraint. */
  if(isfree && is_constr_type(lp, rownr, EQ)) {
    matValue = RHS/pivot;
    if(matValue != 0)
      undoadded = addUndoPresolve(lp, TRUE, colnr, matValue, 0.0, 0);
  }

  else {

    /* IMPLIEDFREE: For simplicity, ensure that we can keep the slack based at 0,
                   and not its upper bound. Effectively, we consider the constraint
                   an equality, using the information of the sign of the dual.
       IMPLIEDSLK: Since we already have an equality constraint, we wish to make sure
                   that the ensuing inequality constraint will have an RHS that is
                   non-infinite. */
    if(isfree) {
      SETMIN(RHS, presolve_sumplumin(lp, rownr, psdata->rows, TRUE));
      matValue = presolve_sumplumin(lp, rownr, psdata->rows, FALSE);
      conRange = get_rh_lower(lp, rownr);
      conRange = RHS - MAX(matValue, conRange);
      signflip = (MYBOOL) ((dual > 0) &&
                           !my_infinite(lp, conRange));
    }
    else {
      varLo = get_lowbo(lp, colnr);
      varLo *= (my_infinite(lp, varLo) ? my_sign(pivot) : pivot);
      varHi = get_upbo(lp, colnr);
      varHi *= (my_infinite(lp, varHi) ? my_sign(pivot) : pivot);
      if(pivot < 0)
        swapREAL(&varHi, &varLo);
      signflip = my_infinite(lp, varLo);
    }
    if(signflip) {
      mat_multrow(mat, rownr, -1);
      RHS -= conRange;
      RHS = -RHS;
      lp->orig_rhs[rownr] = RHS;
      pivot = -pivot;
      dual  = -dual;
      if(!isfree) {
        varLo = -varLo;
        varHi = -varHi;
        swapREAL(&varHi, &varLo);
      }
    }
    matValue = RHS/pivot;

    /* Prepare for deleting free or implied free variable in inequality constraint.
       Different strategies need to be used:

       ACTUAL:  Find the proper constraint bound and store undo information for
                recovering the value of the implied free variable.  The constraint
                is then deleted.  We have to adjust the objective function if the
                OF coefficient for the implied free variable is non-zero.
       IMPLIED: Convert the constraint to an inequality at the proper bound.
                For given models, the new equality constraint can later provide
                an implied slack, which means that a further variable is eliminated,
                and the constraint again becomes an inequality constraint.

      Note that this version only implements the ACTUAL mode */
    if(isfree) {
      /* Add undo information connecting the deleted variable to the RHS */
      if(matValue != 0)
        undoadded = addUndoPresolve(lp, TRUE, colnr, matValue, 0.0, 0);
      /* Add undo information for the dual of the deleted constraint */
      if(dual != 0)
        addUndoPresolve(lp, FALSE, rownr, dual, 0.0, 0);
    }

    /* Prepare for deleting implied slack variable.  The following two cases are
      handled:

      1. Equality constraint: Convert the constraint to an inequality constraint
                              that is possibly ranged
      2. Other constraints:   Expand existing slack variable / constraint
                              range, if required. */
    else {
      if(my_infinite(lp, varHi))
        varRange = lp->infinite;
#ifdef Paranoia
      else if(my_infinite(lp, varLo)) {
        report(lp, SEVERE, "presolve_impliedcolfix: Negative infinite limit for variable %d\n", colnr);
        varRange = lp->infinite;
      }
#endif
      else
        varRange = my_precision(fabs(varHi - varLo) + lp->epsvalue, psdata->epsvalue);
      presolve_adjustrhs(psdata, rownr, varLo, psdata->epsvalue);

      /* Handle case 1 of an equality constraint */
      if(is_constr_type(lp, rownr, EQ)) {
        /* Make sure we actually have a ranged constraint */
        if(varRange > 0) {
          set_constr_type(lp, rownr, LE);
          if(!my_infinite(lp, varRange))
            lp->orig_upbo[rownr] = varRange;
          setLink(psdata->LTmap, rownr);
          removeLink(psdata->EQmap, rownr);
        }
      }
      /* Handle case 2 of an inequality constraint (UNDER CONSTRUCTION!)*/
      else {
        if(!my_infinite(lp, lp->orig_upbo[rownr])) {
          if(my_infinite(lp, varRange))
            lp->orig_upbo[rownr] = lp->infinite;
          else
            lp->orig_upbo[rownr] += varHi - varLo;
        }
      }
      /* Update counts */
      if(matAij > 0)
        psdata->rows->plucount[rownr]--;
      else
        psdata->rows->negcount[rownr]--;
      if(my_sign(varLo) != my_sign(varHi))
        psdata->rows->pluneg[rownr]--;

      /* Add undo information for the deleted variable; note that we cannot link the
        deleted variable to the slack, since it may not be available during undo.
        We really should have a mini LP to compute this allocation ex-post. */
      if(RHS != 0)
        undoadded = addUndoPresolve(lp, TRUE, colnr, RHS/pivot, 0.0, 0);
    }
  }

  /* Update the OF constant */
  if(dual != 0) {
    presolve_adjustrhs(psdata, 0, dual * RHS, 0);
/*    lp->orig_rhs[0] -= dual * RHS; */
    vecOF[colnr] = 0;
  }

  /* Do affine transformation with the constraint row */
  i = 0;
  for(ib = presolve_nextcol(psdata, rownr, &i); ib >= 0;
      ib = presolve_nextcol(psdata, rownr, &i)) {

    /* Get the constraint element */
    jx = ROW_MAT_COLNR(ib);
    if(jx == colnr)
      continue;
    matValue = ROW_MAT_VALUE(ib);

    /* Adjust OF for the variable to be deleted */
    if(dual != 0)
      vecOF[jx] -= dual * matValue;

    /* Add reconstruction/undo parameters for the deleted variable */
    if(!undoadded)
      undoadded = addUndoPresolve(lp, TRUE, colnr, 0.0, matValue/pivot, jx);
    else
      appendUndoPresolve(lp, TRUE, matValue/pivot, jx);
  }

  return( TRUE );
}

STATIC psrec *presolve_initpsrec(lprec *lp, int size)
{
  psrec *ps = (psrec *) calloc(1, sizeof(*ps));

  createLink(size, &ps->varmap, NULL);
    fillLink(ps->varmap);

  size++;

  allocINT(lp, &ps->empty, size, FALSE);
  ps->empty[0] = 0;

  allocREAL(lp, &ps->pluupper,  size, FALSE);
  allocREAL(lp, &ps->negupper,  size, FALSE);
  allocREAL(lp, &ps->plulower,  size, FALSE);
  allocREAL(lp, &ps->neglower,  size, FALSE);
  allocINT(lp,  &ps->infcount,  size, FALSE);

  ps->next = (int **) calloc(size, sizeof(*(ps->next)));

  allocINT(lp,  &ps->plucount,  size, TRUE);
  allocINT(lp,  &ps->negcount,  size, TRUE);
  allocINT(lp,  &ps->pluneg,    size, TRUE);

  ps->allocsize = size;

  return( ps );
}
STATIC void presolve_freepsrec(psrec **ps)
{
  FREE((*ps)->plucount);
  FREE((*ps)->negcount);
  FREE((*ps)->pluneg);
  FREE((*ps)->infcount);

  if((*ps)->next != NULL) {
    int i, n = (*ps)->allocsize;
    for(i = 0; i < n; i++)
      FREE((*ps)->next[i]);
    FREE((*ps)->next);
  }

  FREE((*ps)->plulower);
  FREE((*ps)->neglower);
  FREE((*ps)->pluupper);
  FREE((*ps)->negupper);

  FREE((*ps)->empty);

  freeLink(&(*ps)->varmap);

  FREE(*ps);
}

STATIC presolverec *presolve_init(lprec *lp)
{
  int         k, i, ix, ixx, colnr,
              ncols = lp->columns,
              nrows = lp->rows;
  REAL        hold;
  MATrec      *mat = lp->matA;
  presolverec *psdata = NULL;

  /* Optimize memory usage if we have a very large model;
     this is to reduce the risk of out-of-memory situations. */
  ix  = get_nonzeros(lp);
  ixx = lp->matA->mat_alloc;
  if((ixx - ix > MAT_START_SIZE) && ((ixx - ix) * 20 > ixx))
    mat_memopt(lp->matA, nrows / 20, ncols / 20, ix / 20);

  psdata = (presolverec *) calloc(1, sizeof(*psdata));

  psdata->lp   = lp;
  psdata->rows = presolve_initpsrec(lp, nrows);
  psdata->cols = presolve_initpsrec(lp, ncols);

  psdata->epsvalue = PRESOLVE_EPSVALUE;
  psdata->epspivot = PRESOLVE_EPSPIVOT;
  psdata->forceupdate = TRUE;

  /* Save incoming primal bounds */
  k = lp->sum + 1;
  allocREAL(lp, &psdata->pv_lobo, k, FALSE);
  MEMCOPY(psdata->pv_lobo, lp->orig_lowbo, k);
  allocREAL(lp, &psdata->pv_upbo, k, FALSE);
  MEMCOPY(psdata->pv_upbo, lp->orig_upbo, k);

  /* Create and initialize dual value (Langrangean and slack) limits */
  allocREAL(lp, &psdata->dv_lobo, k, FALSE);
  allocREAL(lp, &psdata->dv_upbo, k, FALSE);
  for(i = 0; i <= nrows; i++) {
    psdata->dv_lobo[i] = (is_constr_type(lp, i, EQ) ? -lp->infinite : 0);
    psdata->dv_upbo[i] = lp->infinite;
  }
  k--;
  for(; i <= k; i++) {
    psdata->dv_lobo[i] = 0;
    psdata->dv_upbo[i] = lp->infinite;
  }

 /* Create NZ count and sign arrays, and do general initialization of row bounds */
  createLink(nrows, &psdata->EQmap, NULL);
  createLink(nrows, &psdata->LTmap, NULL);
  createLink(nrows, &psdata->INTmap, NULL);
  for(i = 1; i <= nrows; i++) {
    switch (get_constr_type(lp, i)) {
      case LE: appendLink(psdata->LTmap, i);
                break;
      case EQ: appendLink(psdata->EQmap, i);
                break;
    }
    k = mat_rowlength(mat, i);
    if((lp->int_vars > 0) && (k > 0))
      appendLink(psdata->INTmap, i);
  }

  /* Seek to reduce set of sum(INT*INT) rows (mainly for GCD coefficient reductions) */
  if(psdata->INTmap->count > 0)
  for(i = 1; i <= nrows; i++) {
    if(!isActiveLink(psdata->INTmap, i))
      continue;
    /* Disqualify if there is a non-int variable, otherwise find smallest absolute fractional row value */
    ix = mat->row_end[i - 1];
    ixx = mat->row_end[i];
    colnr = 0;
    for(; ix < ixx; ix++) {
      if(!is_int(lp, ROW_MAT_COLNR(ix))) {
        removeLink(psdata->INTmap, i);
        break;
      }
      hold = fabs(ROW_MAT_VALUE(ix));
      hold = fmod(hold, 1);
      /* Adjust colnr to be a decimal scalar */
      for(k = 0; (k <= MAX_FRACSCALE) && (hold+psdata->epsvalue < 1); k++)
        hold *= 10;
      if(k > MAX_FRACSCALE) {
        removeLink(psdata->INTmap, i);
        break;
      }
      SETMAX(colnr, k);
    }
    if(!isActiveLink(psdata->INTmap, i))
      continue;
    hold = pow(10.0, colnr);
    /* Also disqualify if the RHS is fractional after scaling */
    if(fabs(fmod(lp->orig_rhs[i] * hold, 1)) > psdata->epsvalue) {
      removeLink(psdata->INTmap, i);
      continue;
    }
    /* We have an all-int constraint, see if we should scale it up */
    if(k > 0) {
      ix = mat->row_end[i - 1];
      for(; ix < ixx; ix++) {
        ROW_MAT_VALUE(ix) *= hold;
      }
      lp->orig_rhs[i] *= hold;
	  if(!my_infinite(lp, lp->orig_upbo[i]))
        lp->orig_upbo[i] *= hold; /* KE: Fix due to Andy Loto - 20070619 */
    }
  }

  /* Do the real tallying and ordering work */
  presolve_validate(psdata, TRUE);

  return( psdata );
}

STATIC void presolve_free(presolverec **psdata)
{
  presolve_freepsrec(&(*psdata)->rows);
  presolve_freepsrec(&(*psdata)->cols);
  FREE((*psdata)->dv_lobo);
  FREE((*psdata)->dv_upbo);
  FREE((*psdata)->pv_lobo);
  FREE((*psdata)->pv_upbo);
  freeLink(&(*psdata)->EQmap);
  freeLink(&(*psdata)->LTmap);
  freeLink(&(*psdata)->INTmap);
  FREE(*psdata);
}

STATIC int presolve_makefree(presolverec *psdata)
{
  lprec    *lp = psdata->lp;
  int      i, ix, j, nn = 0;
  REAL     Xlower, Xupper, losum, upsum, lorhs, uprhs, freeinf = lp->infinite / 10;
  MATrec   *mat = lp->matA;
  LLrec    *colLL = NULL;

  /* First see if we can relax ranged constraints */
  for(i = firstActiveLink(psdata->rows->varmap); i != 0; i = nextActiveLink(psdata->rows->varmap, i)) {
    if(is_constr_type(lp, i, EQ))
      continue;
    presolve_range(lp, i, psdata->rows, &losum, &upsum);
    lorhs = get_rh_lower(lp, i);
    uprhs = get_rh_upper(lp, i);

    /* Look for opportunity to relax constraint bounds */
    if(presolve_rowlength(psdata, i) > 1) {
      if((is_constr_type(lp, i, GE) && (upsum <= uprhs)) ||
         (is_constr_type(lp, i, LE) && (losum >= lorhs)))
        set_rh_range(lp, i, lp->infinite);
    }
  }

  /* Collect columns available for bound relaxation (find implied free variables)
     (consider sorting the list in decending order of column lengths or do call to
      COLAMD to maximize impact) */
  createLink(lp->columns, &colLL, NULL);
  for(j = firstActiveLink(psdata->cols->varmap); j != 0; j = nextActiveLink(psdata->cols->varmap, j))
    if(presolve_impliedfree(lp, psdata, j))
      appendLink(colLL, j);

  /* Find what columns to relax (ideally one per row) */
  if(colLL->count > 0) {
    LLrec  *rowLL = NULL;
    MYBOOL canfree;

    /* Create row tracker */
    createLink(lp->rows, &rowLL, NULL);
    fillLink(rowLL);

    /* Loop over all column candidates */
    for(j = firstActiveLink(colLL); (j > 0) && (rowLL->count > 0); j = nextActiveLink(colLL, j)) {

      /* Verify that the variable is applicable */
      canfree = TRUE;
      for(ix = mat->col_end[j-1]; canfree && (ix < mat->col_end[j]); ix++)
        canfree = isActiveLink(rowLL, COL_MAT_ROWNR(ix));

      /* If so, then open the bounds and update the row availability mapper */
      if(canfree) {
        nn++;
        Xlower = get_lowbo(lp, j);
        Xupper = get_upbo(lp, j);
        if(Xlower >= 0)
          set_bounds(lp, j, 0, freeinf);
        else if(Xupper <= 0)
          set_bounds(lp, j, -freeinf, 0);
        else
/*          set_bounds(lo, j, -freeinf, freeinf); */
          set_unbounded(lp, j);
        for(ix = mat->col_end[j-1]; ix < mat->col_end[j]; ix++)
          removeLink(rowLL, COL_MAT_ROWNR(ix));
      }
    }
    freeLink(&rowLL);
  }

  /* Free list and return */
  freeLink(&colLL);
  return( nn );
}

STATIC MYBOOL presolve_updatesums(presolverec *psdata)
{
  lprec    *lp = psdata->lp;
  int      j;

  /* Initialize row accumulation arrays */
  MEMCLEAR(psdata->rows->pluupper, lp->rows + 1);
  MEMCLEAR(psdata->rows->negupper, lp->rows + 1);
  MEMCLEAR(psdata->rows->plulower, lp->rows + 1);
  MEMCLEAR(psdata->rows->neglower, lp->rows + 1);
  MEMCLEAR(psdata->rows->infcount, lp->rows + 1);

  /* Loop over active columns */
  for(j = firstActiveLink(psdata->cols->varmap); j != 0;
      j = nextActiveLink(psdata->cols->varmap, j)) {
    presolve_colfix(psdata, j, lp->infinite, FALSE, NULL);
  }

#ifdef UseDualPresolve
  /* Initialize column accumulation arrays */
  MEMCLEAR(psdata->cols->pluupper, lp->columns + 1);
  MEMCLEAR(psdata->cols->negupper, lp->columns + 1);
  MEMCLEAR(psdata->cols->plulower, lp->columns + 1);
  MEMCLEAR(psdata->cols->neglower, lp->columns + 1);
  MEMCLEAR(psdata->cols->infcount, lp->columns + 1);

  /* Loop over active rows */
  for(j = firstActiveLink(psdata->rows->varmap); j != 0;
      j = nextActiveLink(psdata->rows->varmap, j)) {
    presolve_rowfix(psdata, j, lp->infinite, FALSE, NULL);
  }
#endif

  return( TRUE );
}

STATIC MYBOOL presolve_finalize(presolverec *psdata)
{
  lprec    *lp = psdata->lp;
  MYBOOL   compactvars = FALSE;
  int      ke, n;

  /* Save eliminated rows and columns for restoration purposes */
#ifdef SavePresolveEliminated
  psdata->deletedA = mat_extractmat(lp->matA, rowmap, colmap, TRUE);
  if(!mat_validate(psdata->deletedA))
    report(lp, SEVERE, "presolve_finalize: Could not validate matrix with undo data\n");
#endif

  /* Check if OF columns are to be deleted */
  lp->presolve_undo->OFcolsdeleted = FALSE;
  for(n = firstInactiveLink(psdata->cols->varmap); (n != 0) && !lp->presolve_undo->OFcolsdeleted;
      n = nextInactiveLink(psdata->cols->varmap, n))
    lp->presolve_undo->OFcolsdeleted = (MYBOOL) (lp->orig_obj[n] != 0);

  /* Delete eliminated columns */
  ke = lastInactiveLink(psdata->cols->varmap);
  n = countInactiveLink(psdata->cols->varmap);
  if((n > 0) && (ke > 0)) {
    del_columnex(lp, psdata->cols->varmap);
    mat_colcompact(lp->matA, lp->presolve_undo->orig_rows,
                             lp->presolve_undo->orig_columns);
    compactvars = TRUE;
  }

  /* Delete eliminated rows */
  ke = lastInactiveLink(psdata->rows->varmap);
  n = countInactiveLink(psdata->rows->varmap);
  if((n > 0) && (ke > 0)) {
    del_constraintex(lp, psdata->rows->varmap);
    mat_rowcompact(lp->matA, TRUE);
    compactvars = TRUE;
  }
  else if(psdata->nzdeleted > 0)
    mat_zerocompact(lp->matA);

  /* Do compacting and updating of variable maps */
  if(compactvars)
    varmap_compact(lp, lp->presolve_undo->orig_rows,
                       lp->presolve_undo->orig_columns);

  /* Reduce memory usage of postsolve matrices */
  if(lp->presolve_undo->primalundo != NULL)
    mat_memopt(lp->presolve_undo->primalundo->tracker, 0, 0, 0);
  if(lp->presolve_undo->dualundo != NULL)
    mat_memopt(lp->presolve_undo->dualundo->tracker, 0, 0, 0);

  /* Round near-zero objective function coefficients and RHS values */
  ke = lp->columns;
  for(n = 1; n <= ke; n++)
    my_roundzero(lp->orig_obj[n], lp->epsvalue);
  ke = lp->rows;
  for(n = 1; n <= ke; n++)
    my_roundzero(lp->orig_rhs[n], lp->epsvalue);

  /* Update the SOS sparse mapping */
  if(SOS_count(lp) > 0)
    SOS_member_updatemap(lp->SOS);

  /* Validate matrix and reconstruct row indexation */
  return(mat_validate(lp->matA));
}

STATIC MYBOOL presolve_debugdump(lprec *lp, presolverec *psdata, char *filename, MYBOOL doappend)
{
  FILE   *output = stdout;
  int   size;
  MYBOOL ok;

  ok = (MYBOOL) ((filename == NULL) || ((output = fopen(filename, my_if(doappend, "a", "w"))) != NULL));
  if(!ok)
    return(ok);
  if((filename == NULL) && (lp->outstream != NULL))
    output = lp->outstream;

  fprintf(output, "\nPRESOLVE - Status at loop %d:%d:%d\n",
                  psdata->outerloops, psdata->middleloops, psdata->innerloops);
  fprintf(output, "Model size:     %d rows (%d equalities, %d less than), %d columns\n",
                  psdata->rows->varmap->count, psdata->EQmap->count, psdata->LTmap->count, psdata->cols->varmap->count);

  fprintf(output, "\nMAPPERS\n-------\n\n");
  size = 1;
  blockWriteINT(output,  "colmap", psdata->cols->varmap->map, 0, size*psdata->cols->varmap->size);
  blockWriteINT(output,  "rowmap", psdata->rows->varmap->map, 0, size*psdata->rows->varmap->size);
  blockWriteINT(output,  "EQmap",  psdata->EQmap->map,  0, size*psdata->EQmap->size);
  blockWriteINT(output,  "LTmap",  psdata->LTmap->map,  0, size*psdata->LTmap->size);

  fprintf(output, "\nCOUNTS\n------\n\n");
  blockWriteINT(output, "plucount",  psdata->rows->plucount,  0, lp->rows);
  blockWriteINT(output, "negcount",  psdata->rows->negcount,  0, lp->rows);
  blockWriteINT(output, "pluneg",    psdata->rows->pluneg,    0, lp->rows);

  fprintf(output, "\nSUMS\n----\n\n");
  blockWriteREAL(output, "pluupper", psdata->rows->pluupper, 0, lp->rows);
  blockWriteREAL(output, "negupper", psdata->rows->negupper, 0, lp->rows);
  blockWriteREAL(output, "plulower", psdata->rows->pluupper, 0, lp->rows);
  blockWriteREAL(output, "neglower", psdata->rows->negupper, 0, lp->rows);

  if(filename != NULL)
    fclose(output);
  return(ok);
}

int CMP_CALLMODEL compRedundant(const UNIONTYPE QSORTrec *current, const UNIONTYPE QSORTrec *candidate)
{
  int start1 = (int) (current->int4.intpar1),
      start2 = (int) (candidate->int4.intpar1),
      result = CMP_COMPARE(start1, start2);

  if(result == 0) {
    start1 = (int) (current->int4.intpar2);
    start2 = (int) (candidate->int4.intpar2);
    result = -CMP_COMPARE(start1, start2);
  }
  return( result );
}
int CMP_CALLMODEL compSparsity(const UNIONTYPE QSORTrec *current, const UNIONTYPE QSORTrec *candidate)
{
  int start1 = (int) (current->int4.intpar1),
      start2 = (int) (candidate->int4.intpar1),
      result = CMP_COMPARE(start1, start2);

  if(result == 0) {
    start1 = (int) (current->int4.intpar2);
    start2 = (int) (candidate->int4.intpar2);
    result = -CMP_COMPARE(start1, start2);
  }

  if(result == 0) {
    start1 = (int) (current->int4.intval);
    start2 = (int) (candidate->int4.intval);
    result = CMP_COMPARE(start1, start2);
  }
  return( result );
}
int CMP_CALLMODEL compAggregate(const UNIONTYPE QSORTrec *current, const UNIONTYPE QSORTrec *candidate)
{
  int  index1 = (int) (current->pvoidint2.intval),
       index2 = (int) (candidate->pvoidint2.intval);
  lprec *lp   = (lprec *) current->pvoidint2.ptr;
  REAL value1 = lp->orig_obj[index1],
       value2 = lp->orig_obj[index2];

  /* Smallest objective coefficient (largest contribution to OF) */
  int  result = CMP_COMPARE(value1, value2);

  /* Smallest lower variable bound */
  if(result == 0) {
    index1 += lp->rows;
    index2 += lp->rows;
    value1 = lp->orig_lowbo[index1];
    value2 = lp->orig_lowbo[index2];
    result = CMP_COMPARE(value1, value2);
  }

  /* Largest upper variable bound */
  if(result == 0) {
    value1 = lp->orig_upbo[index1];
    value2 = lp->orig_upbo[index2];
    result = -CMP_COMPARE(value1, value2);
  }
  return( result );
}

STATIC int presolve_rowdominance(presolverec *psdata, int *nCoeffChanged, int *nRowsRemoved, int *nVarsFixed, int *nSum)
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  int      i, ii, ib, ie, n, jb, je, jx, *coldel = NULL, status = RUNNING, item,
           iCoeffChanged = 0, iRowRemoved = 0, iVarFixed = 0;
  REAL     ratio, *rowvalues = NULL;
  UNIONTYPE QSORTrec *QS = (UNIONTYPE QSORTrec *) calloc(lp->rows+1, sizeof(*QS));

  /* Check if we were able to obtain working memory */
  if(QS == NULL)
    return( status);

  /* A dominating row of variables always satisfy the following criteria:
      1) The starting column position is never lower, but could be the same
      2) The non-zero row count is always lower */
  n = 0;
  for(i = firstActiveLink(psdata->EQmap); i != 0; i = nextActiveLink(psdata->EQmap, i)) {
    /* Make sure we have no SOS or semi-continuous variables */
    jb = je = 0;
    if((SOS_count(lp) > 0) || (lp->sc_vars > 0)) {
      item = 0;
      for(jb = presolve_nextcol(psdata, i, &item); jb >= 0;
          jb = presolve_nextcol(psdata, i, &item)) {
        jx = ROW_MAT_COLNR(jb);
        if(SOS_is_member(lp->SOS, 0, jx) || is_semicont(lp, jx))
          break;
      }
    }

    /* Add to list if we are Ok */
    if(jb < 0) {
      QS[n].int4.intval = i;
      item = 0;
      ii = presolve_nextcol(psdata, i, &item);
      QS[n].int4.intpar1 = ROW_MAT_COLNR(ii);
      QS[n].int4.intpar2 = presolve_rowlength(psdata, i);
      n++;
    }
  }
  if(n <= 1)
    goto Finish;
  QS_execute(QS, n, (findCompare_func *) compRedundant, NULL);

  /* Let us start from the top of the list, going forward and looking
    for the longest possible dominating row */
  if(!allocREAL(lp, &rowvalues, lp->columns + 1, TRUE) ||
     !allocINT(lp, &coldel, lp->columns + 1, FALSE))
    goto Finish;

  for(ib = 0; ib < n; ib++) {

    /* Get row and check if it was previously eliminated */
    i = QS[ib].int4.intval;
    if(i < 0)
      continue;

    /* Load the non-zero row values */
    item = 0;
    for(jb = presolve_nextcol(psdata, i, &item); jb >= 0;
        jb = presolve_nextcol(psdata, i, &item)) {
      jx = ROW_MAT_COLNR(jb);
      rowvalues[jx] = ROW_MAT_VALUE(jb);
    }

    for(ie = ib+1; ie < n; ie++) {

      /* Get row and check if it was previously eliminated */
      ii = QS[ie].int4.intval;
      if(ii < 0)
        continue;

#ifdef Paranoia
      if((QS[ib].int4.intpar1 > QS[ie].int4.intpar1) ||
         ((QS[ib].int4.intpar1 == QS[ie].int4.intpar1) && (QS[ib].int4.intpar2 < QS[ie].int4.intpar2)))
        report(lp, SEVERE, "presolve_rowdominance: Invalid sorted row order\n");
#endif

      /* Loop over every row member to confirm that the candidate
        actually dominates in every position */
      if((lp->orig_rhs[i] == 0) && (lp->orig_rhs[ii] == 0))
        ratio = 0;
      else if((lp->orig_rhs[i] != 0) && (lp->orig_rhs[ii] != 0))
        ratio = lp->orig_rhs[i] / lp->orig_rhs[ii];
      else
        continue;
      item = 0;
      for(jb = presolve_nextcol(psdata, ii, &item); jb >= 0;
          jb = presolve_nextcol(psdata, ii, &item)) {
        jx = ROW_MAT_COLNR(jb);
        if(rowvalues[jx] == 0)
          break;
        if(ratio == 0)
          ratio = rowvalues[jx] / ROW_MAT_VALUE(jb);
        else if(fabs(rowvalues[jx] - ratio*ROW_MAT_VALUE(jb)) > psdata->epsvalue)
          break;
      }

      /* "We have contact" */
      if(jb < 0) {
        int sign_1 = 0, sign_j = 0;

        /* Need to fix any superset columns, but require that they have equal signs */
        coldel[0] = 0;
        item = 0;
        for(jb = presolve_nextcol(psdata, i, &item); jb >= 0;
            jb = presolve_nextcol(psdata, i, &item)) {
          jx = ROW_MAT_COLNR(jb);
          if(mat_findelm(mat, ii, jx) <= 0) {

            /* Cancel if we detect a free or "quasi-free" variable */
            if((lp->orig_lowbo[lp->rows + jx] < 0) &&
               (lp->orig_upbo[lp->rows + jx] > 0)) {
              coldel[0] = -1;
              break;
            }

            /* Ensure that we are feasible */
            else if((lp->orig_lowbo[lp->rows + jx] > 0) ||
               (lp->orig_upbo[lp->rows + jx] < 0)) {
              report(lp, DETAILED, "presolve_rowdominate: Column %s is infeasible due to conflict in rows %s and %s\n",
                                    get_col_name(lp, jx), get_row_name(lp, i), get_row_name(lp, ii));
              coldel[0] = -1;
              break;
            }

            /* Check consistency / uniformity of signs */
            sign_j = my_sign(ROW_MAT_VALUE(jb));
            sign_j = my_chsign(is_negative(lp, jx), sign_j);
            if(coldel[0] == 0) {
              sign_1 = sign_j;
              coldel[++coldel[0]] = jx;
            }
            else if(sign_j == sign_1) {
              coldel[++coldel[0]] = jx;
            }
            else {
              coldel[0] = -1;
              break;
            }
          }
        }

        /* Force break / continuation if the superset columns were incompatible */
        if(coldel[0] < 0)
          continue;

        /* Do the column fixing and deletion (check for infeasibility in the process) */
        for(jb = 1; jb <= coldel[0]; jb++) {
          jx = coldel[jb];
          if(!presolve_colfix(psdata, jx, 0, TRUE, &iVarFixed)) {
             status = presolve_setstatus(psdata, INFEASIBLE);
             goto Finish;
          }
          presolve_colremove(psdata, jx, TRUE);
          rowvalues[jx] = 0;
        }

        /* Then delete the row */
        presolve_rowremove(psdata, ii, TRUE);
        iRowRemoved++;
        QS[ie].int4.intval = -ii;
      }
    }

    /* Clear the non-zero row values ahead of the next row candidate */
    ie = mat->row_end[i-1];
    ii = mat->row_end[i];
    for(; ie < ii; ie++)
      rowvalues[ROW_MAT_COLNR(ie)] = 0;

  }
Finish:
  FREE(QS);
  FREE(rowvalues);
  FREE(coldel);

  (*nCoeffChanged) += iCoeffChanged;
  (*nRowsRemoved)  += iRowRemoved;
  (*nVarsFixed)    += iVarFixed;
  (*nSum)          += iCoeffChanged + iRowRemoved + iVarFixed;

  return( status );
}

#if 0
STATIC int presolve_coldominance01(presolverec *psdata, int *nConRemoved, int *nVarsFixed, int *nSum)
/* The current version of this routine eliminates binary variables
   that are dominated via set coverage or unit knapsack constraints */
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  MYBOOL   first;
  int      i, ii, ib, ie, n, jb, je, jx, jj, item, item2,
           *coldel = NULL, status = RUNNING, iVarFixed = 0;
  REAL     scale, rhsval, *colvalues = NULL;
  UNIONTYPE QSORTrec *QS = (UNIONTYPE QSORTrec *) calloc(lp->columns+1, sizeof(*QS));

  /* Check if we were able to obtain working memory */
  if(QS == NULL)
    return( status);
  if(lp->int_vars == 0)
    goto Finish;

  /* A column dominates another binary variable column with the following criteria:
      1) The relative matrix non-zero entries are identical
      2) The relative objective coefficient is worse than the other;
         if the OF coefficients are identical, we can delete an arbitrary variable */
  n = 0;
  for(i = firstActiveLink(psdata->cols->varmap); i != 0; i = nextActiveLink(psdata->cols->varmap, i))
    if(is_binary(lp, i) && !SOS_is_member(lp->SOS, 0, i)) {
      /* Make sure we have an all-binary, unit-coefficient row */
      je = mat->col_end[i];
      item = 0;
      for(jb = presolve_nextrow(psdata, i, &item); jb >= 0;
          jb = presolve_nextrow(psdata, i, &item)) {
        jx = COL_MAT_ROWNR(jb);
        if(COL_MAT_VALUE(jb) != 1)
          break;
      }

      /* Add to list if we are Ok */
      if(jb < 0) {
        QS[n].int4.intval = i;
        item = 0;
        ii = presolve_nextrow(psdata, i, &item);
        QS[n].int4.intpar1 = COL_MAT_ROWNR(ii);
        ii = presolve_collength(psdata, i);
        QS[n].int4.intpar2 = ii;
        n++;
      }
    }
  if(n <= 1) {
    FREE(QS);
    return( status );
  }
  QS_execute(QS, n, (findCompare_func *) compRedundant, NULL);

  /* Let us start from the top of the list, going forward and looking
    for the longest possible dominated column */
  if(!allocREAL(lp, &colvalues, lp->rows + 1, TRUE) ||
     !allocINT(lp, &coldel, lp->columns + 1, FALSE))
    goto Finish;

  for(ib = 0; ib < n; ib++) {

    /* Get column and check if it was previously eliminated */
    i = QS[ib].int4.intval;
    if(i < 0)
      continue;

    /* Load the non-zero column values */
    item = 0;
    for(jb = presolve_nextrow(psdata, i, &item); jb >= 0;
        jb = presolve_nextrow(psdata, i, &item)) {
      jx = COL_MAT_ROWNR(jb);
      colvalues[jx] = COL_MAT_VALUE(jb);
    }

    coldel[0] = 0;
    for(ie = ib+1; ie < n; ie++) {

      /* Insist on identical column lengths (sort is decending in column lengths) */
      ii = QS[ib].int4.intpar2 - QS[ie].int4.intpar2;
      if(ii != 0)
        break;

      /* Also insist on identical starting positions */
      ii = QS[ib].int4.intpar1 - QS[ie].int4.intpar1;
      if(ii != 0)
        break;

      /* Get column and check if it was previously eliminated */
      ii = QS[ie].int4.intval;
      if(ii < 0)
        continue;

      /* Also make sure that the variables have "compatible" bounds */
#if 1
      if((fabs(my_reldiff(lp->orig_lowbo[lp->rows + i], lp->orig_lowbo[lp->rows + ii])) > psdata->epsvalue) ||
         (fabs(my_reldiff(lp->orig_upbo[lp->rows + i],  lp->orig_upbo[lp->rows + ii] )) > psdata->epsvalue))
        continue;
#endif

#ifdef Paranoia
      if((QS[ib].int4.intpar1 > QS[ie].int4.intpar1) ||
         ((QS[ib].int4.intpar1 == QS[ie].int4.intpar1) && (QS[ib].int4.intpar2 < QS[ie].int4.intpar2)))
        report(lp, SEVERE, "presolve_coldominance01: Invalid sorted column order\n");
#endif

      /* Loop over every column member to confirm that the candidate is
        relatively identical in every position */
      first = TRUE;
      item = 0;
      item2 = 0;
      scale = 1;
      for(jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2); jb >= 0;
          jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2)) {
        jx = COL_MAT_ROWNR(jb);
        if(jx != COL_MAT_ROWNR(jj))
          break;
        if(first) {
          first = !first;
          scale = colvalues[jx] / COL_MAT_VALUE(jb);
        }
        else {
          if(fabs(colvalues[jx] - scale * COL_MAT_VALUE(jb)) > psdata->epsvalue)
            break;
        }
        /* Also make sure we have a compatible RHS (since this version of the
          dominance logic only applies to "sets") */
        rhsval = scale*lp->orig_rhs[jx] - 1.0;
        /* if((rhsval < 0) || (rhsval > 1 + psdata->epsvalue)) */
		if(fabs(rhsval) > psdata->epsvalue)
          break;
      }

      /* "We have contact" */
      if(jb < 0) {
        coldel[++coldel[0]] = ii;
        QS[ie].int4.intval = -ii;
      }
    }

    /* Find the dominant column and delete / fix the others;
       if there is a tie, simply delete the second candidate */
    ii = i;
    for(jb = 1; jb <= coldel[0]; jb++) {
      jx = coldel[jb];
      if(lp->orig_obj[jx] < lp->orig_obj[ii])
        swapINT(&ii, &coldel[jb]);
    }
    for(jb = 1; jb <= coldel[0]; jb++) {
      jx = coldel[jb];
      if(!presolve_colfix(psdata, jx, lp->orig_lowbo[lp->rows+jx], TRUE, &iVarFixed)) {
         status = presolve_setstatus(psdata, INFEASIBLE);
         goto Finish;
      }
      presolve_colremove(psdata, jx, TRUE);
    }

    /* Clear the non-zero row values ahead of the next row candidate */
    if(ib + 1 < n) {
      ie = mat->col_end[i-1];
      ii = mat->col_end[i];
      for(; ie < ii; ie++)
        colvalues[COL_MAT_ROWNR(ie)] = 0;
    }
  }
Finish:
  FREE(QS);
  FREE(colvalues);
  FREE(coldel);

  (*nVarsFixed) += iVarFixed;
  (*nSum)       += iVarFixed;

  return( status );
}
#else

/* DEVELOPMENT/TEST CODE FOR POSSIBLE REPLACEMENT OF SIMILAR FUNCTION IN lp_presolve.c */

#define NATURAL int

STATIC int presolve_coldominance01(presolverec *psdata, NATURAL *nConRemoved, NATURAL *nVarsFixed, NATURAL *nSum)
/* The current version of this routine eliminates binary variables
   that are dominated via set coverage or unit knapsack constraints */
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  NATURAL  i, ib, ie, jx, item, item2,
           n = lp->int_vars, iVarFixed = 0, nrows = lp->rows,
           *coldel = NULL;
  int      jb, jj, ii,
           status = RUNNING;
  REAL     rhsval,
           *colvalues = NULL, *colobj = NULL;
  LLrec    *sets = NULL;
  UNIONTYPE QSORTrec *QS = (UNIONTYPE QSORTrec *) calloc(n+1, sizeof(*QS));

  /* Check if we were able to obtain working memory */
  if(QS == NULL)
    return( status);
  if(n == 0)
    goto Finish;

  /* Create list of set coverage and knapsack constraints */
  createLink(nrows, &sets, NULL);
  for(i = firstActiveLink(psdata->rows->varmap); i != 0; i = nextActiveLink(psdata->rows->varmap, i)) {
    if((lp->orig_rhs[i] < 0) || (psdata->rows->negcount[i] > 0))
      continue;
    item = 0;
    for(jb = presolve_nextcol(psdata, i, &item); jb >= 0;
        jb = presolve_nextcol(psdata, i, &item)) {
      jx = ROW_MAT_COLNR(jb);
      if(!is_binary(lp, jx))
        break;
      rhsval = ROW_MAT_VALUE(jb) - 1;
      if(fabs(rhsval) > lp->epsvalue)
        break;
    }
    if(jb < 0)
      setLink(sets, i);
  }
  if(countActiveLink(sets) == 0)
    goto Finish;

  /* A column dominates another binary variable column with the following criteria:
      1) The relative matrix non-zero entries are identical
      2) The relative objective coefficient is worse than the other;
         if the OF coefficients are identical, we can delete an arbitrary variable */
  n = 0;
  for(i = firstActiveLink(psdata->cols->varmap); i != 0; i = nextActiveLink(psdata->cols->varmap, i))
    if(is_binary(lp, i) && !SOS_is_member(lp->SOS, 0, i)) {
      /* Make sure the column is member of at least one set */
      item = 0;
      for(jb = presolve_nextrow(psdata, i, &item); jb >= 0;
          jb = presolve_nextrow(psdata, i, &item)) {
        jx = COL_MAT_ROWNR(jb);
        if(isActiveLink(sets, jx))
          break;
      }

      /* Add to list if set membership test is Ok */
      if(jb >= 0) {
        QS[n].int4.intval = i;
        item = 0;
        ii = presolve_nextrow(psdata, i, &item);
        QS[n].int4.intpar1 = COL_MAT_ROWNR(ii);
        ii = presolve_collength(psdata, i);
        QS[n].int4.intpar2 = ii;
        n++;
      }
    }
  if(n <= 1) {
    FREE(QS);
    return( status );
  }
  QS_execute(QS, n, (findCompare_func *) compRedundant, NULL);

  /* Let us start from the top of the list, going forward and looking
    for the longest possible dominated column */
  if(!allocREAL(lp, &colvalues, nrows + 1, TRUE) ||
     !allocREAL(lp, &colobj, n + 1, FALSE) ||
     !allocINT(lp, &coldel, n + 1, FALSE))
    goto Finish;

  for(ib = 0; ib < n; ib++) {

    /* Get column and check if it was previously eliminated */
    i = QS[ib].int4.intval;
    if(!isActiveLink(psdata->cols->varmap, i))
      continue;

    /* Load the non-zero column values */
    item = 0;
    for(jb = presolve_nextrow(psdata, i, &item); jb >= 0;
        jb = presolve_nextrow(psdata, i, &item)) {
      jx = COL_MAT_ROWNR(jb);
      colvalues[jx] = COL_MAT_VALUE(jb);
    }

    /* Store data for current column */
    coldel[0] = 1;
    coldel[1] = i;
    colobj[1] = lp->orig_obj[i];

    /* Loop over all other columns to see if they have equal constraint coefficients */
    for(ie = ib+1; ie < n; ie++) {

      /* Check if this column was previously eliminated */
      ii = QS[ie].int4.intval;
      if(!isActiveLink(psdata->cols->varmap, ii))
        continue;

      /* Insist on identical column lengths (sort is decending in column lengths) */
      ii = QS[ib].int4.intpar2 - QS[ie].int4.intpar2;
      if(ii != 0)
        break;

      /* Also insist on identical starting positions */
      ii = QS[ib].int4.intpar1 - QS[ie].int4.intpar1;
      if(ii != 0)
        break;

      /* Get column and check if it was previously eliminated */
      ii = QS[ie].int4.intval;

#ifdef Paranoia
      if((QS[ib].int4.intpar1 > QS[ie].int4.intpar1) ||
         ((QS[ib].int4.intpar1 == QS[ie].int4.intpar1) && (QS[ib].int4.intpar2 < QS[ie].int4.intpar2)))
        report(lp, SEVERE, "presolve_coldominance01: Invalid sorted column order\n");
#endif

      /* Loop over every column member to confirm that the candidate is identical in every row;
         we also compute the minimal set order */
      rhsval = lp->infinite;
      item = 0;
      item2 = 0;
      for(jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2); jb >= 0;
          jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2)) {
        jx = COL_MAT_ROWNR(jb);
        if(jx != COL_MAT_ROWNR(jj))
          break;
        if(isActiveLink(sets, jx))
          SETMIN(rhsval, lp->orig_rhs[jx]);
      }

      /* "We have contact" */
      if(jb < 0) {
        coldel[++coldel[0]] = ii;
        colobj[coldel[0]] = lp->orig_obj[ii];
      }
    }

    /* Find the dominant columns, fix and delete the others */
    if(coldel[0] > 1) {
      qsortex(colobj+1, coldel[0], 0, sizeof(*colobj), FALSE, compareREAL, coldel+1, sizeof(*coldel));
      jb = (NATURAL) (rhsval+lp->epsvalue);
      for(jb++; jb <= coldel[0]; jb++) {
        jx = coldel[jb];
        if(!presolve_colfix(psdata, jx, lp->orig_lowbo[nrows+jx], TRUE, &iVarFixed)) {
           status = presolve_setstatus(psdata, INFEASIBLE);
           goto Finish;
        }
        presolve_colremove(psdata, jx, TRUE);
      }
    }

    /* Clear the non-zero row values ahead of the next row candidate */
    if(ib + 1 < n) {
      ie = mat->col_end[i-1];
      ii = mat->col_end[i];
      for(; ie < ii; ie++)
        colvalues[COL_MAT_ROWNR(ie)] = 0;
    }
  }
Finish:
  freeLink(&sets);
  FREE(QS);
  FREE(colvalues);
  FREE(coldel);
  FREE(colobj);

  (*nVarsFixed) += iVarFixed;
  (*nSum)       += iVarFixed;

  return( status );
}

#endif

STATIC int presolve_aggregate(presolverec *psdata, int *nConRemoved, int *nVarsFixed, int *nSum)
/* This routine combines compatible or identical columns */
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  MYBOOL   first;
  int      i, ii, ib, ie, ix, n, jb, je, jx, jj, item, item2,
           *coldel = NULL, status = RUNNING, iVarFixed = 0;
  REAL     scale, *colvalues = NULL;
  UNIONTYPE QSORTrec *QScand = (UNIONTYPE QSORTrec *) calloc(lp->columns+1, sizeof(*QScand));

  /* Check if we were able to obtain working memory */
  if(QScand == NULL)
    return( status);

  /* Obtain the list of qualifying columns to be sorted */
  n = 0;
  for(i = firstActiveLink(psdata->cols->varmap); i != 0; i = nextActiveLink(psdata->cols->varmap, i))
    if(!is_semicont(lp, i) && !SOS_is_member(lp->SOS, 0, i)) {
      QScand[n].int4.intval = i;
      item = 0;
      ii = presolve_nextrow(psdata, i, &item);
      QScand[n].int4.intpar1 = COL_MAT_ROWNR(ii);
      ii = presolve_collength(psdata, i);
      QScand[n].int4.intpar2 = ii;
      n++;
    }
  if(n <= 1) {
    FREE(QScand);
    return( status );
  }
  QS_execute(QScand, n, (findCompare_func *) compRedundant, NULL);

  /* Let us start from the top of the list, going forward and looking
    for the longest possible identical column */
  if(!allocREAL(lp, &colvalues, lp->rows + 1, TRUE) ||
     !allocINT(lp, &coldel, lp->columns + 1, FALSE))
    goto Finish;

  for(ib = 0; ib < n; ib++) {

    /* Get column and check if it was previously eliminated */
    i = QScand[ib].int4.intval;
    if(i < 0)
      continue;

    /* Load the non-zero column values of this active/reference column */
    item = 0;
    for(jb = presolve_nextrow(psdata, i, &item); jb >= 0;
        jb = presolve_nextrow(psdata, i, &item)) {
      jx = COL_MAT_ROWNR(jb);
      colvalues[jx] = COL_MAT_VALUE(jb);
    }

    coldel[0] = 0;
    for(ie = ib+1; ie < n; ie++) {

      /* Insist on identical column lengths (sort is decending in column lengths) */
      ii = QScand[ib].int4.intpar2 - QScand[ie].int4.intpar2;
      if(ii != 0)
        break;

      /* Also insist on identical starting positions */
      ii = QScand[ib].int4.intpar1 - QScand[ie].int4.intpar1;
      if(ii != 0)
        break;

      /* Get column and check if it was previously eliminated */
      ii = QScand[ie].int4.intval;
      if(ii < 0)
        continue;

      /* Loop over every column member to confirm that the candidate is
        relatively identical in every position */
      first = TRUE;
      item = 0;
      item2 = 0;
      scale = 1;
      for(jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2); jb >= 0;
          jb = presolve_nextrow(psdata, ii, &item),
          jj = presolve_nextrow(psdata, i, &item2)) {
        jx = COL_MAT_ROWNR(jb);
        if(jx != COL_MAT_ROWNR(jj))
          break;
        if(first) {
          first = !first;
          scale = colvalues[jx] / COL_MAT_VALUE(jb);
        }
        else {
          if(fabs(colvalues[jx] - scale * COL_MAT_VALUE(jb)) > psdata->epsvalue)
            break;
        }
      }

      /* "We have contact", store the column in the aggregation list */
      if(jb < 0) {
        coldel[++coldel[0]] = ii;
        QScand[ie].int4.intval = -ii;
      }
    }

    /* Sort the aggregation list if we have aggregation candidates */
    if(coldel[0] > 1) {
      REAL     of, ofelim, fixvalue;
      MYBOOL   isint;
      UNIONTYPE QSORTrec *QSagg = (UNIONTYPE QSORTrec *) calloc(coldel[0], sizeof(*QSagg));

      for(jb = 1; jb <= coldel[0]; jb++) {
        ii = jb - 1;
        QSagg[ii].pvoidint2.intval = coldel[jb];
        QSagg[ii].pvoidint2.ptr    = (void *) lp;
      }
      QS_execute(QSagg, coldel[0], (findCompare_func *) compAggregate, NULL);

      /* Process columns with identical OF coefficients */
      jb = 0;
      while((status == RUNNING) && (jb < coldel[0])) {
        ii = QSagg[jb].pvoidint2.intval;
        of = lp->orig_obj[ii];
        isint = is_int(lp, ii);
        je = jb + 1;
        while((status == RUNNING) && (je < coldel[0]) &&
              (fabs(lp->orig_obj[ix = QSagg[je].pvoidint2.intval] - of) < psdata->epsvalue)) {
           /* We now have two columns with equal OFs; the following cases are possible:

             1) The first column has Inf upper bound, which means that it can
                "absorb" compatible columns, which are then fixed at the appropriate
                bounds (or zero in case of free variables).
             2) The first column has a -Inf lower bound, and further columns are
                Inf upper bounds, which means steps towards forming a free variable
                can be made.
             3) The first column is a non-Inf upper bound, in which case the bounds
                are summed into a helper variable and the variable simply deleted.
                The deleted variables' value are allocated/distributed via a simple
                linear programming routine at postsolve.

             In the current version of this code, we only handle case 1. */
          if(is_int(lp, ix) == isint) {
            ofelim = lp->orig_obj[ix];
            if(of == 0)
              scale = 1;
            else
              scale = ofelim / of;

            if(my_infinite(lp, lp->orig_upbo[lp->rows+ii])) { /* Case 1 (recipe.mps) */
              if(is_unbounded(lp, ix))
                fixvalue = 0;
              else if(ofelim < 0)
                fixvalue = lp->orig_upbo[lp->rows+ix];
              else
                fixvalue = lp->orig_lowbo[lp->rows+ix];
              if(my_infinite(lp, fixvalue))
                status = presolve_setstatus(psdata, UNBOUNDED);
              else if(!presolve_colfix(psdata, ix, fixvalue, TRUE, &iVarFixed))
                status = presolve_setstatus(psdata, INFEASIBLE);
              else
                presolve_colremove(psdata, ix, TRUE);
            }

            else if(my_infinite(lp, lp->orig_lowbo[lp->rows+ii])) { /* Case 2 */
              /* Do nothing */
            }

            else {                                            /* Case 3 */
#if 0
              /* Do nothing */
#else
              if(ofelim >= 0) {
                fixvalue = lp->orig_lowbo[lp->rows+ix];
                lp->orig_upbo[lp->rows+ii] += scale * (lp->orig_upbo[lp->rows+ix] - fixvalue);
              }
              else {
                fixvalue = lp->orig_upbo[lp->rows+ix];
                lp->orig_upbo[lp->rows+ii] -= scale * (fixvalue - lp->orig_lowbo[lp->rows+ix]);
              }
              if(my_infinite(lp, fixvalue))
                status = presolve_setstatus(psdata, UNBOUNDED);
              else if(!presolve_colfix(psdata, ix, fixvalue, TRUE, &iVarFixed))
                status = presolve_setstatus(psdata, INFEASIBLE);
              else
                presolve_colremove(psdata, ix, TRUE);
#ifdef xxParanoia
              if(presolve_rowlengthdebug(psdata) > 0)
                report(lp, SEVERE, "presolve_aggregate: Invalid row count\n");
#endif
              psdata->forceupdate = TRUE;
#endif
            }
          }
          je++;
        }
        jb = je;
      }
      FREE(QSagg);
    }

    /* Clear the non-zero row values ahead of the next row candidate */
    if(ib + 1 < n) {
      ie = mat->col_end[i-1];
      ii = mat->col_end[i];
      for(; ie < ii; ie++)
        colvalues[COL_MAT_ROWNR(ie)] = 0;
    }
  }
Finish:
  FREE(QScand);
  FREE(colvalues);
  FREE(coldel);

  (*nVarsFixed) += iVarFixed;
  (*nSum)       += iVarFixed;

  return( status );
}

STATIC int presolve_makesparser(presolverec *psdata, int *nCoeffChanged, int *nConRemove, int *nVarFixed, int *nSum)
{
  lprec    *lp = psdata->lp;
  MATrec   *mat = lp->matA;
  MYBOOL   chsign;
  int      i, ii, ib, ix, k, n, jb, je, jl, jjb, jje, jjl, jx, jjx, item, itemEQ,
           *nzidx = NULL, status = RUNNING, iObjChanged = 0, iCoeffChanged = 0, iConRemove = 0;
  REAL     test, ratio, value, valueEQ, *valptr;
  LLrec    *EQlist = NULL;
  UNIONTYPE QSORTrec *QS = (UNIONTYPE QSORTrec *) calloc(lp->rows, sizeof(*QS));

  /* Check if we were able to obtain working memory */
  if((QS == NULL) || (psdata->rows->varmap->count == 0) || (psdata->EQmap->count == 0))
    return( status);

  /* Sort rows in 1) increasing order of start index, 2) decreasing length, and
     3) non-equalities (i.e. equalities last) */
  n = 0;
  for(i = firstActiveLink(psdata->rows->varmap); i != 0; i = nextActiveLink(psdata->rows->varmap, i)) {
    k = presolve_rowlength(psdata, i);
    if(k >= 2) {
      item = 0;
      ii = presolve_nextcol(psdata, i, &item);
#ifdef Paranoia
      if((ii < 0) || (item == 0)) {
        report(lp, SEVERE, "presolve_makesparser: Unexpected zero-length row %d\n", i);
        continue;
      }
#endif
      QS[n].int4.intval  = my_chsign(is_constr_type(lp, i, EQ), i);
      QS[n].int4.intpar1 = ROW_MAT_COLNR(ii);
      QS[n].int4.intpar2 = k;
      n++;
    }
  }
  if(n <= 1) {
    FREE(QS);
    return( status );
  }
  QS_execute(QS, n, (findCompare_func *) compSparsity, NULL);

  /* Create associated sorted map of indeces to equality constraints;
     note that we need to have a unit offset for compatibility. */
  allocINT(lp, &nzidx, lp->columns + 1, FALSE);
  createLink(lp->rows, &EQlist, NULL);
  for(ib = 0; ib < n; ib++) {
    i = QS[ib].int4.intval;
    if(i < 0)
      appendLink(EQlist, ib + 1);
  }

  /* Loop over all equality masks */
  for(ix = firstActiveLink(EQlist); ix != 0; ) {

    /* Get row starting and ending positions of the mask */
    ii = abs(QS[ix-1].int4.intval);
    jjb = QS[ix-1].int4.intpar1;
    jje = presolve_lastcol(psdata, ii);
    jje = ROW_MAT_COLNR(jje);
    jjl = QS[ix-1].int4.intpar2;

    /* Scan the OF */
    i = 0;
    chsign = is_chsign(lp, i);
    test = ratio = 0.0;
    itemEQ = 0;
    nzidx[0] = 0;
    while(((jjx = presolve_nextcol(psdata, ii, &itemEQ)) >= 0) && /*(itemEQ > 0) && */
           (fabs(test-ratio) < psdata->epsvalue)) {
      valueEQ = ROW_MAT_VALUE(jjx);
      if(valueEQ == 0)
        continue;
      k = ROW_MAT_COLNR(jjx);
      value = lp->orig_obj[k];
      if(fabs(value) < psdata->epsvalue)
        break;
      if(ratio == 0.0) {
        test = ratio = value / valueEQ;
      }
      else
        test = value / valueEQ;
      /* Store nz index */
      nzidx[++nzidx[0]] = k;
    }

    /* We were successful if the equality was completely traversed; we will
      then zero-out the OF coefficients and update the constant term. */
    if((itemEQ == 0) && (nzidx[0] > 0) && (fabs(test-ratio) < psdata->epsvalue)) {
      for(k = 1; k <= nzidx[0]; k++) {
        /* We should add recovery data for the zero'ed coefficient here */
        jx = nzidx[k];
        value = lp->orig_obj[jx];
        lp->orig_obj[jx] = 0.0;
        /* Update counts */
        value = my_chsign(chsign, value);
        if(value < 0) {
          psdata->rows->negcount[i]--;
          psdata->cols->negcount[jx]--;
        }
        else {
          psdata->rows->plucount[i]--;
          psdata->cols->plucount[jx]--;
        }
        iObjChanged++;
      }
      value = ratio * lp->orig_rhs[ii];
      presolve_adjustrhs(psdata, i, value, psdata->epsvalue);
    }

    /* Scan for compatible constraints that can be masked for sparsity elimination */
    for(ib = 1; ib < ix; ib++) {

      /* Get row starting and ending positions of the target constraint */
      i  = abs(QS[ib-1].int4.intval);
      jb = QS[ib-1].int4.intpar1;
      je = presolve_lastcol(psdata, i);
      je = ROW_MAT_COLNR(je);
      jl = QS[ib-1].int4.intpar2;

      /* Check if there is a window mismatch */
      if((jjb < jb) || (jje > je) || (jjl > jl))
        goto NextEQ;

      /* We have a window match; now check if there is a (scalar) member-by-member
        match as well.  We approach this in the following manner:
          1) Get first (or next) member of active equality
          2) Loop to matching member in the target constraint, but abandon if no match
          3) Set ratio if this is the first match, otherwise compare ratio and abandon
             on mismatch
          4) Go to 1) of there are more elements in the active equality
          5) Proceed to do sparsity elimination if we were successful. */
      chsign = is_chsign(lp, i);
      test = ratio = 0.0;
      itemEQ = 0;
      item = 0;
      nzidx[0] = 0;
      while(((jjx = presolve_nextcol(psdata, ii, &itemEQ)) >= 0) && /*(itemEQ > 0) &&*/
             (fabs(test-ratio) < psdata->epsvalue)) {
        valueEQ = ROW_MAT_VALUE(jjx);
        if(valueEQ == 0)
          continue;
        jx = 0;
        jjx = ROW_MAT_COLNR(jjx);
        for(k = presolve_nextcol(psdata, i, &item);
            (jx < jjx) && (item > 0);
            k = presolve_nextcol(psdata, i, &item)) {
          jx = ROW_MAT_COLNR(k);
          /* Do we have a column index match? */
          if(jx == jjx) {
            value = ROW_MAT_VALUE(k);
            /* Abandon if we have a zero value */
            if(value == 0)
              goto NextEQ;
            if(ratio == 0.0) {
              test = ratio = value / valueEQ;
            }
            else
              test = value / valueEQ;
           /* Store nz index */
            nzidx[++nzidx[0]] = k;
            break;
          }
          /* Give up matching if there is overshooting */
          else if(jx > jjx)
            goto NextEQ;
        }
      }

      /* We were successful if the equality was completely traversed */
      if((itemEQ == 0) && (nzidx[0] > 0) && (fabs(test-ratio) < psdata->epsvalue)) {

        /* Check if we have found parametrically indentical constraints */
        if(presolve_rowlength(psdata, i) == presolve_rowlength(psdata,ii)) {

          value = lp->orig_rhs[i];
          valueEQ = lp->orig_rhs[ii];

          /* Are they both equalities? */
          if(is_constr_type(lp, i, EQ)) {
            /* Determine applicable ratio for the RHS */
            if(fabs(valueEQ) < psdata->epsvalue) {
              if(fabs(value) < psdata->epsvalue)
                test = ratio;
              else
                test = lp->infinite;
            }
            else
              test = value / valueEQ;
            /* Check for infeasibility */
            if(fabs(test-ratio) > psdata->epsvalue) {
              report(lp, NORMAL, "presolve_sparser: Infeasibility of relatively equal constraints %d and %d\n",
                                 i, ii);
              status = presolve_setstatus(psdata, INFEASIBLE);
              goto Finish;
            }
            /* Otherwise we can delete a redundant constraint */
            else {
              removeLink(EQlist, i);
              presolve_rowremove(psdata, i, TRUE);
              MEMCOPY(&QS[ib-1], &QS[ib], n-ib);
              n--;
              iConRemove++;
            }
          }
          /* ... if not, then delete the inequality, since the equality dominates */
          else {
            /* First verify feasibility of the RHS */
            if((value+psdata->epsvalue < valueEQ) ||
               (value-get_rh_range(lp, i)-psdata->epsvalue > valueEQ)) {
              report(lp, NORMAL, "presolve_sparser: Infeasibility of relatively equal RHS values for %d and %d\n",
                                 i, ii);
              status = presolve_setstatus(psdata, INFEASIBLE);
              goto Finish;
            }
            presolve_rowremove(psdata, i, TRUE);
            MEMCOPY(&QS[ib-1], &QS[ib], n-ib);
            n--;
            iConRemove++;
          }
        }

        /* Otherwise zero-out the target constraint coefficients and update the RHS */
        else {
          for(k = 1; k <= nzidx[0]; k++) {
            /* We should add recovery data for the zero'ed coefficient here */
            jjx = nzidx[k];
            jx = ROW_MAT_COLNR(jjx);
            valptr = &ROW_MAT_VALUE(jjx);
            value  = *valptr;
            *valptr = 0.0;
            /* Update counts */
            value = my_chsign(chsign, value);
            if(value < 0) {
              psdata->rows->negcount[i]--;
              psdata->cols->negcount[jx]--;
            }
            else {
              psdata->rows->plucount[i]--;
              psdata->cols->plucount[jx]--;
            }
            iCoeffChanged++;
          }
          value = ratio * lp->orig_rhs[ii];
          presolve_adjustrhs(psdata, i, value, psdata->epsvalue);
        }
      }

    }
    /* Get next equality index */
NextEQ:
    ix = nextActiveLink(EQlist, ix);
  }

Finish:
  FREE(QS);
  freeLink(&EQlist);
  FREE(nzidx);

  /* Let us condense the matrix if we modified the constraint matrix */
  if(iCoeffChanged > 0) {
    mat->row_end_valid = FALSE;
    mat_zerocompact(mat);
    presolve_validate(psdata, TRUE);
#ifdef PresolveForceUpdateMax
    mat_computemax(mat /* , FALSE */);
#endif
    psdata->forceupdate = TRUE;
  }

  (*nConRemove)    += iConRemove;
  (*nCoeffChanged) += iCoeffChanged + iObjChanged;
  (*nSum)          += iCoeffChanged + iObjChanged + iConRemove;

  return( status );
}

STATIC int presolve_SOS1(presolverec *psdata, int *nCoeffChanged, int *nConRemove, int *nVarFixed, int *nSOS, int *nSum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   candelete, SOS_GUBactive = FALSE;
  int      iCoeffChanged = 0, iConRemove = 0, iSOS = 0,
           i,ix,iix, j,jx,jjx, status = RUNNING;
  REAL     Value1;
  MATrec   *mat = lp->matA;

  for(i = lastActiveLink(psdata->rows->varmap); i > 0; ) {
    candelete = FALSE;
    Value1 = get_rh(lp, i);
    jx = get_constr_type(lp, i);
    if((Value1 == 1) && (presolve_rowlength(psdata, i) >= MIN_SOS1LENGTH) &&
       ((SOS_GUBactive && (jx != GE)) || (!SOS_GUBactive && (jx == LE)))) {
      jjx = mat->row_end[i-1];
      iix = mat->row_end[i];
      for(; jjx < iix; jjx++) {
        j = ROW_MAT_COLNR(jjx);
        if(!isActiveLink(psdata->cols->varmap, j))
          continue;
        if(!is_binary(lp, j) || (ROW_MAT_VALUE(jjx) != 1))
          break;
      }
      if(jjx >= iix) {
        char SOSname[16];

        /* Define a new SOS instance */
        ix = SOS_count(lp) + 1;
        sprintf(SOSname, "SOS_%d", ix);
        ix = add_SOS(lp, SOSname, 1, ix, 0, NULL, NULL);
        if(jx == EQ)
          SOS_set_GUB(lp->SOS, ix, TRUE);
        Value1 = 0;
        jjx = mat->row_end[i-1];
        for(; jjx < iix; jjx++) {
          j = ROW_MAT_COLNR(jjx);
          if(!isActiveLink(psdata->cols->varmap, j))
            continue;
          Value1 += 1;
          append_SOSrec(lp->SOS->sos_list[ix-1], 1, &j, &Value1);
        }
        candelete = TRUE;
        iSOS++;
      }
    }

    /* Get next row and do the deletion of the previous, if indicated */
    ix = i;
    i = prevActiveLink(psdata->rows->varmap, i);
    if(candelete) {
      presolve_rowremove(psdata, ix, TRUE);
      iConRemove++;
    }
  }
  if(iSOS)
    report(lp, DETAILED, "presolve_SOS1: Converted %5d constraints to SOS1.\n", iSOS);
  clean_SOSgroup(lp->SOS, (MYBOOL) (iSOS > 0));

  (*nCoeffChanged) += iCoeffChanged;
  (*nConRemove)    += iConRemove;
  (*nSOS)          += iSOS;
  (*nSum)          += iCoeffChanged+iConRemove+iSOS;

  return( status );
}

STATIC int presolve_boundconflict(presolverec *psdata, int baserowno, int colno)
{
  REAL   Value1, Value2;
  lprec  *lp = psdata->lp;
  MATrec *mat = lp->matA;
  int    ix, item = 0,
         status = RUNNING;

  if(baserowno <= 0) do {
    ix = presolve_nextrow(psdata, colno, &item);
    if(ix < 0)
      return( status );
    baserowno = COL_MAT_ROWNR(ix);
  } while(presolve_rowlength(psdata, baserowno) != 1);
  Value1 = get_rh_upper(lp, baserowno),
  Value2 = get_rh_lower(lp, baserowno);

  if(presolve_singletonbounds(psdata, baserowno, colno, &Value2, &Value1, NULL)) {
    int iix;
    item = 0;
    for(ix = presolve_nextrow(psdata, colno, &item);
        ix >= 0; ix = presolve_nextrow(psdata, colno, &item)) {
      iix = COL_MAT_ROWNR(ix);
      if((iix != baserowno) &&
         (presolve_rowlength(psdata, iix) == 1) &&
         !presolve_altsingletonvalid(psdata, iix, colno, Value2, Value1)) {
        status = presolve_setstatus(psdata, INFEASIBLE);
        break;
      }
    }
  }
  else
    status = presolve_setstatus(psdata, INFEASIBLE);
  return( status );
}

STATIC int presolve_columns(presolverec *psdata, int *nCoeffChanged, int *nConRemove, int *nVarFixed, int *nBoundTighten, int *nSum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   candelete, isOFNZ, unbounded,
           probefix = is_presolve(lp, PRESOLVE_PROBEFIX),
           probereduce = is_presolve(lp, PRESOLVE_PROBEREDUCE),
           colfixdual = is_presolve(lp, PRESOLVE_COLFIXDUAL);
  int      iCoeffChanged = 0, iConRemove = 0, iVarFixed = 0, iBoundTighten = 0,
           status = RUNNING, ix, j, countNZ, item;
  REAL     Value1;

  for(j = firstActiveLink(psdata->cols->varmap); (j != 0) && (status == RUNNING); ) {

    /* Don't presolve members SOS'es */
    if(SOS_is_member(lp->SOS, 0, j)) {
      j = nextActiveLink(psdata->cols->varmap, j);
      continue;
    }

    /* Initialize */
    countNZ = presolve_collength(psdata, j);
    isOFNZ  = isnz_origobj(lp, j);
    Value1  = get_lowbo(lp, j);
    unbounded = is_unbounded(lp, j);

    /* Clear unnecessary semicont-definitions */
    if((lp->sc_vars > 0) && (Value1 == 0) && is_semicont(lp, j))
      set_semicont(lp, j, FALSE);

    candelete = FALSE;
    item = 0;
    ix = lp->rows + j;

    /* Check if the variable is unused */
    if((countNZ == 0) && !isOFNZ) {
      if(Value1 != 0)
        report(lp, DETAILED, "presolve_columns: Eliminated unused variable %s\n",
                              get_col_name(lp,j));
      candelete = TRUE;
    }

    /* Check if the variable has a cost, but is not limited by constraints */
    else if((countNZ == 0) && isOFNZ) {
      if(lp->orig_obj[j] < 0)
        Value1 = get_upbo(lp, j);
      if(fabs(Value1) >= lp->infinite) {
        report(lp, DETAILED, "presolve_columns: Unbounded variable %s\n",
                              get_col_name(lp,j));
        status = presolve_setstatus(psdata, UNBOUNDED);
      }
      else {
        /* Fix the value at its best bound */
        report(lp, DETAILED, "presolve_columns: Eliminated trivial variable %s fixed at %g\n",
                              get_col_name(lp,j), Value1);
        candelete = TRUE;
      }
    }

    /* Check if the variable can be eliminated because it is fixed */
    else if(isOrigFixed(lp, ix)) {
      if(countNZ > 0) {
        status = presolve_boundconflict(psdata, -1, j);
        if(status != RUNNING)
          break;
      }
      report(lp, DETAILED, "presolve_columns: Eliminated variable %s fixed at %g\n",
                            get_col_name(lp,j), Value1);
      candelete = TRUE;
    }

#if 0
    /* Merge OF-constraint column doubleton in equality constraint (if it has
      not been captured by the singleton free variable rule above) */
    else if((countNZ == 1) && isOFNZ &&
             ((i = presolve_nextrow(psdata, j, &item)) >= 0) &&
             is_constr_type(lp, i = COL_MAT_ROWNR(i), EQ)) {
      MATrec *mat = lp->matA;

      /* Merge the constraint into the OF */
      Value1 = lp->orig_obj[j] / get_mat(lp, i, j);
      for(jx = mat->row_end[i-1]; jx < mat->row_end[i]; jx++) {
        jjx = ROW_MAT_COLNR(jx);
        lp->orig_obj[jjx] -= Value1 * ROW_MAT_VALUE(jx);
      }
      Value2 = lp->orig_rhs[i];
      presolve_adjustrhs(psdata, 0, Value1 * Value2, 0.0);

      /* Verify feasibility */
      Value2 /= get_mat(lp, i, j);
      if((Value2 < get_lowbo(lp, j)) || (Value2 > get_upbo(lp, j))) {
        status = presolve_setstatus(psdata, INFEASIBLE);
        break;
      }

      /* Do column (and flag row) deletion */
      presolve_rowremove(psdata, i, TRUE);
      psdata->forceupdate = TRUE;
      iConRemove++;
      candelete = TRUE;
    }
#endif
    /* Look for opportunity to fix column based on the dual */
    else if(colfixdual && presolve_colfixdual(psdata, j, &Value1, &status)) {
      if(my_infinite(lp, Value1)) {
        report(lp, DETAILED, "presolve_columns: Unbounded variable %s\n",
                              get_col_name(lp,j));
        status = presolve_setstatus(psdata, UNBOUNDED);
      }
      else {
        /* Fix the value at its best bound */
        report(lp, DETAILED, "presolve_columns: Eliminated dual-zero variable %s fixed at %g\n",
                              get_col_name(lp,j), Value1);
        candelete = TRUE;
      }
    }

    /* Do probing of binary variables to see if we can fix them */
    else if(probefix && is_binary(lp, j) &&
            presolve_probefix01(psdata, j, &Value1)) {
      report(lp, DETAILED, "presolve_columns: Fixed binary variable %s at %g\n",
                            get_col_name(lp,j), Value1);
      candelete = TRUE;
    }
#if 0
    /* Do probing of binary variables to see if we can tighten their coefficients */
    else if(probereduce && is_binary(lp, j) &&
            (ix = presolve_probetighten01(psdata, j) > 0)) {
      report(lp, DETAILED, "presolve_columns: Tightened coefficients for binary variable %s in %d rows\n",
                            get_col_name(lp,j), ix);
      iCoeffChanged += ix;
      psdata->forceupdate = TRUE;
    }
#endif

    /* Perform fixing and deletion, if indicated */
    if(candelete) {

      /* If we have a SOS1 member variable fixed at a non-zero value, then we
        must fix the other member variables at zero and delete the SOS(es) */
      if((Value1 != 0) && SOS_is_member(lp->SOS, 0, j)) {
        ix = iVarFixed;
        if(!presolve_fixSOS1(psdata, j, Value1, &iConRemove, &iVarFixed))
          status = presolve_setstatus(psdata, INFEASIBLE);
        if(iVarFixed > ix)
          psdata->forceupdate = TRUE;
        break;
      }
      else {
        if(!presolve_colfix(psdata, j, Value1, TRUE, &iVarFixed)) {
          status = presolve_setstatus(psdata, INFEASIBLE);
          break;
        }
        j = presolve_colremove(psdata, j, TRUE);
      }
    }
    else
      j = nextActiveLink(psdata->cols->varmap, j);
  }

  /* Remove any "hanging" empty row and columns */
  if(status == RUNNING)
    status = presolve_shrink(psdata, &iConRemove, &iVarFixed);

  (*nCoeffChanged) += iCoeffChanged;
  (*nConRemove)    += iConRemove;
  (*nVarFixed)     += iVarFixed;
  (*nBoundTighten) += iBoundTighten;
  (*nSum)          += iCoeffChanged+iConRemove+iVarFixed+iBoundTighten;

  return( status );
}

STATIC int presolve_freeandslacks(presolverec *psdata, int *nCoeffChanged, int *nConRemove, int *nVarFixed, int *nSum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   isOFNZ, unbounded,
           impliedfree = is_presolve(lp, PRESOLVE_IMPLIEDFREE),
           impliedslack = is_presolve(lp, PRESOLVE_IMPLIEDSLK);
  int      iCoeffChanged = 0, iConRemove = 0, iVarFixed = 0,
           status = RUNNING, i, ix, j, countNZ;
  REAL     coeff_bl, coeff_bu;
  MATrec   *mat = lp->matA;

  if(impliedfree || impliedslack)
  for(j = firstActiveLink(psdata->cols->varmap); j != 0; ) {

    /* Check and initialize */
    if((presolve_collength(psdata, j) != 1) ||
       is_int(lp, j) || is_semicont(lp, j) ||
       !presolve_candeletevar(psdata, j)) {
      j = nextActiveLink(psdata->cols->varmap, j);
      continue;
    }
    ix = 0;
    i = COL_MAT_ROWNR(presolve_nextrow(psdata, j, &ix));
    isOFNZ  = isnz_origobj(lp, j);
    countNZ = presolve_rowlength(psdata, i);
    coeff_bu = get_upbo(lp, j);
    coeff_bl = get_lowbo(lp, j);
    unbounded = my_infinite(lp, coeff_bl) && my_infinite(lp, coeff_bu);
    ix = lp->rows + j;

    /* Eliminate singleton free variable and its associated constraint */
    if(impliedfree && unbounded &&
       presolve_impliedcolfix(psdata, i, j, TRUE)) {
      report(lp, DETAILED, "presolve_freeandslacks: Eliminated free variable %s and row %s\n",
                            get_col_name(lp, j), get_row_name(lp, i));
      presolve_rowremove(psdata, i, TRUE);
      iConRemove++;
      j = presolve_colremove(psdata, j, TRUE);
      iVarFixed++;
    }

    /* Check for implied slack variable in equality constraint */
    else if(impliedslack &&
             (countNZ > 1) &&
             is_constr_type(lp, i, EQ) &&
             presolve_impliedcolfix(psdata, i, j, FALSE)) {
      report(lp, DETAILED, "presolve_freeandslacks: Eliminated implied slack variable %s via row %s\n",
                            get_col_name(lp, j), get_row_name(lp, i));
      psdata->forceupdate = TRUE;
      j = presolve_colremove(psdata, j, TRUE);
      iVarFixed++;
    }

    /* Check for implied (generalized) slack variable in inequality constraint */
    else if(impliedslack && !isOFNZ &&
             my_infinite(lp, coeff_bu) &&                 /* Consider removing this test */
#if 0 /* Force zero-bounded implicit slack  */
             (coeff_bl == 0)) &&
#else
             !my_infinite(lp, coeff_bl) &&
#endif
             (countNZ > 1) &&
             !is_constr_type(lp, i, EQ))  {
      REAL *target,
            ValueA   = COL_MAT_VALUE(presolve_lastrow(psdata, j));
#if 0
      coeff_bu = get_rh_upper(lp, i);
      coeff_bl = get_rh_lower(lp, i);
      if(!presolve_singletonbounds(psdata, i, j, &coeff_bl, &coeff_bu, &ValueA)) {
        status = presolve_setstatus(psdata, INFEASIBLE);
        break;
      }
#endif
      if((coeff_bl != 0) && !my_infinite(lp, coeff_bl) && !my_infinite(lp, coeff_bu))
        coeff_bu -= coeff_bl;

      /* If the coefficient is negative, reduce the lower bound / increase range */
      if(ValueA > 0) {
        target = &lp->orig_upbo[i];
        if(!my_infinite(lp, *target)) {
          if(my_infinite(lp, coeff_bu)) {
            *target = lp->infinite;
            psdata->forceupdate = TRUE;
          }
          else {
            *target += ValueA * coeff_bu;
            *target = presolve_roundrhs(lp, *target, FALSE);
          }
        }
      }
      /* Otherwise see if the upper bound should be changed */
      else {
        target = &lp->orig_rhs[i];
        if(my_infinite(lp, coeff_bu) || my_infinite(lp, *target)) {
          /* Do we suddenly find that the constraint becomes redundant? (e226.mps) */
          if(my_infinite(lp, lp->orig_upbo[i])) {
            presolve_rowremove(psdata, i, TRUE);
            iConRemove++;
          }
          /* Or does the upper bound of a ranged constraint become Inf? */
          else {
            *target -= lp->orig_upbo[i];
            *target = -(*target);
            mat_multrow(mat, i, -1);
            lp->orig_upbo[i] = lp->infinite;
            psdata->forceupdate = TRUE;
          }
        }
        else {
          *target -= ValueA * coeff_bu;
          *target = presolve_roundrhs(lp, *target, FALSE);
        }
      }
      presolve_colfix(psdata, j, coeff_bl, TRUE, &iVarFixed);
      report(lp, DETAILED, "presolve_freeandslacks: Eliminated duplicate slack variable %s via row %s\n",
                            get_col_name(lp, j), get_row_name(lp, i));
      j = presolve_colremove(psdata, j, TRUE);
    }

    /* Go to next column */
    else
      j = nextActiveLink(psdata->cols->varmap, j);
  }

  (*nCoeffChanged) += iCoeffChanged;
  (*nConRemove)    += iConRemove;
  (*nVarFixed)     += iVarFixed;
  (*nSum)          += iCoeffChanged+iConRemove+iVarFixed;

  return( status );
}

STATIC int presolve_preparerows(presolverec *psdata, int *nBoundTighten, int *nSum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   impliedfree = is_presolve(lp, PRESOLVE_IMPLIEDFREE),
           tightenbounds  = is_presolve(lp, PRESOLVE_BOUNDS);
  int      iRangeTighten = 0, iBoundTighten = 0, status = RUNNING, i, j;
  REAL     losum, upsum, lorhs, uprhs, epsvalue = psdata->epsvalue;
  MATrec   *mat = lp->matA;

  for(i = lastActiveLink(psdata->rows->varmap); i > 0; i = prevActiveLink(psdata->rows->varmap, i)) {

   /* First identify any full row infeasibilities */
    j = presolve_rowlengthex(psdata, i);
#ifdef Paranoia
    if(!presolve_testrow(psdata, nextActiveLink(psdata->rows->varmap, i))) {
#else
    if((j > 1) && !psdata->forceupdate && !presolve_rowfeasible(psdata, i, FALSE)) {
#endif
      status = presolve_setstatus(psdata, INFEASIBLE);
      break;
    }

    /* Do bound (LHS) or constraint range (RHS) tightening if we will later identify
      implied free variables (tends to produce degeneracy otherwise) */
    if(impliedfree && (j > 1) && mat_validate(mat)){

      /* Look for opportunity to tighten constraint bounds (and check for feasibility again) */
      presolve_range(lp, i, psdata->rows, &losum, &upsum);
      lorhs = get_rh_lower(lp, i);
      uprhs = get_rh_upper(lp, i);
      if((losum > MIN(upsum, uprhs)+epsvalue) ||
         (upsum < MAX(losum, lorhs)-epsvalue)) {
        report(lp, NORMAL, "presolve_preparerows: Variable bound / constraint value infeasibility in row %s.\n",
                           get_row_name(lp, i));
        status = presolve_setstatus(psdata, INFEASIBLE);
        break;
      }

      if(losum > lorhs+epsvalue) {
        set_rh_lower(lp, i, presolve_roundrhs(lp, losum, TRUE));
        iRangeTighten++;
      }
      if(upsum < uprhs-epsvalue) {
        set_rh_upper(lp, i, presolve_roundrhs(lp, upsum, FALSE));
        iRangeTighten++;
      }
    }

    /* Seek to tighten bounds on individual variables */
    if(tightenbounds && mat_validate(mat)) {
#if 1
      if(j > 1)
        status = presolve_rowtighten(psdata, i, &iBoundTighten, FALSE);
#else
      if((MIP_count(lp) > 0) && (j > 1))
        status = presolve_rowtighten(psdata, i, &iBoundTighten, TRUE);
#endif
    }

    /* Look for opportunity to convert ranged constraint to equality-type */
    if(!is_constr_type(lp, i, EQ) && (get_rh_range(lp, i) < epsvalue)) {
      presolve_setEQ(psdata, i);
      iRangeTighten++;
    }
  }

  psdata->forceupdate |= (MYBOOL) (iBoundTighten > 0);
  (*nBoundTighten) += iBoundTighten+iRangeTighten;
  (*nSum)          += iBoundTighten+iRangeTighten;

  return( status );
}

STATIC int presolve_rows(presolverec *psdata, int *nCoeffChanged, int *nConRemove, int *nVarFixed, int *nBoundTighten, int *nSum)
{
  lprec    *lp = psdata->lp;
  MYBOOL   candelete;
  int      iCoeffChanged = 0, iConRemove = 0, iVarFixed = 0, iBoundTighten = 0,
           status = RUNNING, i,ix, j,jx, item;
  REAL     Value1, Value2, losum, upsum, lorhs, uprhs, epsvalue = psdata->epsvalue;
  MATrec   *mat = lp->matA;

  for(i = lastActiveLink(psdata->rows->varmap); (i > 0) && (status == RUNNING); ) {

    candelete = FALSE;

   /* First identify any full row infeasibilities
      Note: Handle singletons below to ensure that conflicting multiple singleton
            rows with this variable do not provoke notice of infeasibility */
    j = presolve_rowlengthex(psdata, i);
    if((j > 1) &&
       !psdata->forceupdate && !presolve_rowfeasible(psdata, i, FALSE)) {
      status = presolve_setstatus(psdata, INFEASIBLE);
      break;
    }
    presolve_range(lp, i, psdata->rows, &losum, &upsum);
    lorhs = get_rh_lower(lp, i);
    uprhs = get_rh_upper(lp, i);
#ifdef Paranoia
    if((losum>uprhs+epsvalue) || (upsum<lorhs-epsvalue)) {
      status = presolve_setstatus(psdata, INFEASIBLE);
      break;
    }
#endif

    /* Delete empty rows */
    if(j == 0)
      candelete = TRUE;
    else

    /* Convert non-fixed row singletons to bounds */
#if 0  /* Version that deletes bound-fixed columns in presolve_columns() */
    if((j == 1) && (upsum-losum >= -epsvalue)) {
#else  /* Version that deletes bound-fixed columns here */
    if((j == 1) && (uprhs-lorhs >= -epsvalue)) {
#endif
      item = 0;
      jx = presolve_nextcol(psdata, i, &item);
      j = ROW_MAT_COLNR(jx);

      /* Make sure we don't have conflicting other singleton rows with this variable */
      Value1 = lp->infinite;
      Value2 = -Value1;
      if(presolve_collength(psdata, j) > 1)
        status = presolve_boundconflict(psdata, i, j);
      else if(is_constr_type(lp, i, EQ)) {
        Value2 = ROW_MAT_VALUE(jx);
        Value1 = lp->orig_rhs[i] / Value2;
        if(Value2 < 0)
          swapREAL(&losum, &upsum);
        if((Value1 < losum / my_if(my_infinite(lp, losum), my_sign(Value2), Value2) - epsvalue) ||
           (Value1 > upsum / my_if(my_infinite(lp, upsum), my_sign(Value2), Value2) + epsvalue))
          status = presolve_setstatus(psdata, INFEASIBLE);
        Value2 = Value1;
      }

      /* Proceed to fix and remove variable (if it is not a SOS member) */
      if(status == RUNNING) {
        if((fabs(Value2-Value1) < epsvalue) && (fabs(Value2) > epsvalue)) {
          MYBOOL isSOS     = (MYBOOL) (SOS_is_member(lp->SOS, 0, j) != FALSE),
                 deleteSOS = isSOS && presolve_candeletevar(psdata, j);
          if((Value1 != 0) && deleteSOS) {
            if(!presolve_fixSOS1(psdata, j, Value1, &iConRemove, &iVarFixed))
              status = presolve_setstatus(psdata, INFEASIBLE);
              psdata->forceupdate = TRUE;
          }
          else {
            if(!presolve_colfix(psdata, j, Value1, (MYBOOL) !isSOS, NULL))
              status = presolve_setstatus(psdata, INFEASIBLE);
            else if(isSOS && !deleteSOS)
              iBoundTighten++;
            else {
              presolve_colremove(psdata, j, TRUE);
              iVarFixed++;
            }
          }
        }
        else
          status = presolve_colsingleton(psdata, i, j, &iBoundTighten);
      }
      if(status == INFEASIBLE) {
        break;
      }
      if(psdata->forceupdate != AUTOMATIC) {
        /* Store dual recovery information and code for deletion */
        presolve_storeDualUndo(psdata, i, j);
        candelete = TRUE;
      }
    }

    /* Delete non-empty rows and variables that are completely determined at zero */
    else if((j > 0)                            /* Only examine non-empty rows, */
       && (fabs(lp->orig_rhs[i]) < epsvalue)   /* .. and the current RHS is zero, */
       && ((psdata->rows->plucount[i] == 0) ||
           (psdata->rows->negcount[i] == 0))   /* .. and the parameter signs are all equal, */
       && (psdata->rows->pluneg[i] == 0)       /* .. and no (quasi) free variables, */
       && (is_constr_type(lp, i, EQ)
#ifdef FindImpliedEqualities
           || (fabs(lorhs-upsum) < epsvalue)   /* Convert to equalities */
           || (fabs(uprhs-losum) < epsvalue)   /* Convert to equalities */
#endif
          )
          ) {

      /* Delete the columns we can delete */
      status = presolve_rowfixzero(psdata, i, &iVarFixed);

      /* Then delete the row, which is redundant */
      if(status == RUNNING)
        candelete = TRUE;
    }


    /* Check if we have a constraint made redundant through bounds on individual
       variables; such constraints are often referred to as "forcing constraints" */
    else if((losum >= lorhs-epsvalue) &&
             (upsum <= uprhs+epsvalue)) {

      /* Check if we can also fix all the variables */
      if(fabs(losum-upsum) < epsvalue) {
        item = 0;
        jx = presolve_nextcol(psdata, i, &item);
        while((status == RUNNING) && (jx >= 0)) {
          j = ROW_MAT_COLNR(jx);
          Value1 = get_lowbo(lp, j);
          if(presolve_colfix(psdata, j, Value1, TRUE, &iVarFixed)) {
            presolve_colremove(psdata, j, TRUE);
            iVarFixed++;
            jx = presolve_nextcol(psdata, i, &item);
          }
          else
            status = presolve_setstatus(psdata, INFEASIBLE);
        }
      }
      candelete = TRUE;
    }

    /* Get next row and do the deletion of the previous, if indicated */
    ix = i;
    i = prevActiveLink(psdata->rows->varmap, i);
    if(candelete) {
      presolve_rowremove(psdata, ix, TRUE);
      iConRemove++;
    }
  }

  /* Remove any "hanging" empty row and columns */
  if(status == RUNNING)
    status = presolve_shrink(psdata, &iConRemove, &iVarFixed);

  (*nCoeffChanged) += iCoeffChanged;
  (*nConRemove)    += iConRemove;
  (*nVarFixed)     += iVarFixed;
  (*nBoundTighten) += iBoundTighten;
  (*nSum)          += iCoeffChanged+iConRemove+iVarFixed+iBoundTighten;

  return( status );
}

/* Top level presolve routine */
STATIC int presolve(lprec *lp)
{
  int    status = RUNNING,
         i, j = 0, jx = 0, jjx = 0, k, oSum,
         iCoeffChanged = 0, iConRemove = 0, iVarFixed = 0, iBoundTighten = 0, iSOS = 0, iSum = 0,
         nCoeffChanged = 0, nConRemove = 0, nVarFixed = 0, nBoundTighten = 0, nSOS = 0, nSum = 0;
  REAL   Value1, Value2, initrhs0 = lp->orig_rhs[0];
  presolverec *psdata = NULL;
  MATrec *mat = lp->matA;

#if 0
  lp->do_presolve     = PRESOLVE_ROWS;
  report(lp, IMPORTANT, "presolve: Debug override of presolve setting to %d\n", lp->do_presolve);
#endif

 /* Lock the variable mapping arrays and counts ahead of any row/column
    deletion or creation in the course of presolve, solvelp or postsolve */
  if(!lp->varmap_locked)
    varmap_lock(lp);

 /* Check if we have already done presolve */
  mat_validate(mat);
  if(lp->wasPresolved) {
    if(SOS_count(lp) > 0) {
      SOS_member_updatemap(lp->SOS);
      make_SOSchain(lp, (MYBOOL) ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE));
    }
    if((lp->solvecount > 1) && (lp->bb_level < 1) &&
       ((lp->scalemode & SCALE_DYNUPDATE) != 0))
      auto_scale(lp);
    if(!lp->basis_valid) {
      crash_basis(lp);
      report(lp, DETAILED, "presolve: Had to repair broken basis.\n");
    }
    lp->timepresolved = timeNow();
    return(status);
  }

  /* Produce original model statistics (do hoops to produce correct stats if we have SOS'es) */
  i = SOS_count(lp);
  if(i > 0) {
    SOS_member_updatemap(lp->SOS);
    lp->sos_vars = SOS_memberships(lp->SOS, 0);
  }
  REPORT_modelinfo(lp, TRUE, "SUBMITTED");
  report(lp, NORMAL, " \n");
  if(i > 0)
    lp->sos_vars = 0;

  /* Finalize basis indicators; if no basis was created earlier via
     set_basis or crash_basis then simply set the default basis. */
  if(!lp->basis_valid)
    lp->var_basic[0] = AUTOMATIC; /* Flag that we are presolving */

#if 0
write_lp(lp, "test_in.lp");    /* Write to lp-formatted file for debugging */
/*write_mps(lp, "test_in.mps");*/  /* Write to lp-formatted file for debugging */
#endif

  /* Update inf norms and check for potential factorization trouble */
  mat_computemax(mat /*, FALSE */);
#if 0
  Value1 = fabs(lp->negrange);
  if(is_obj_in_basis(lp) && (mat->dynrange < Value1) && vec_computeext(lp->orig_obj, 1, lp->columns, TRUE, &i, &j)) {

    /* Compute relative scale metric */
    Value2 = fabs(lp->orig_obj[j]/lp->orig_obj[i]) / mat->dynrange;
    if(Value2 < 1.0)
      Value2 = 1.0 / Value2;

    /* Determine if we should alert modeler and possibly move the OF out of the coefficient matrix */
    if((Value2 > Value1)           /* Case with extreme scale difference */
#if 1
        || (mat->dynrange == 1.0)  /* Case where we have an all-unit coefficient matrix, possibly totally unimodular */
#endif
      )
      if((lp->simplex_strategy & SIMPLEX_DYNAMIC) > 0) {
        clear_action(&lp->algopt, ALGOPT_OBJINBASIS);
        report(lp, NORMAL, "Moved objective function out of the basis matrix to enhance factorization accuracy.\n");
      }
      else if(mat->dynrange > 1.0)
        report(lp, IMPORTANT, "Warning: Objective/matrix coefficient magnitude differences will cause inaccuracy!\n");
  }
#endif

  /* Do traditional simple presolve */
  yieldformessages(lp);
  if((lp->do_presolve & PRESOLVE_LASTMASKMODE) == PRESOLVE_NONE) {
    mat_checkcounts(mat, NULL, NULL, TRUE);
    i = 0;
  }
  else {

    if(lp->full_solution == NULL)
      allocREAL(lp, &lp->full_solution, lp->sum_alloc+1, TRUE);

    /* Identify infeasible SOS'es prior to any pruning */
    j = 0;
    for(i = 1; i <= SOS_count(lp); i++) {
      k = SOS_infeasible(lp->SOS, i);
      if(k > 0) {
	presolverec psdata;

	psdata.lp = lp;
        report(lp, NORMAL, "presolve: Found SOS %d (type %d) to be range-infeasible on variable %d\n",
                            i, SOS_get_type(lp->SOS, i), k);
        status = presolve_setstatus(&psdata, INFEASIBLE);
        j++;
      }
    }
    if(j > 0)
      goto Finish;

    /* Create and initialize the presolve data structures */
    psdata = presolve_init(lp);

    /* Reentry point for the outermost, computationally expensive presolve loop */
    psdata->outerloops = 0;
    do {
      psdata->outerloops++;
      iCoeffChanged = 0;
      iConRemove    = 0;
      iVarFixed     = 0;
      iBoundTighten = 0;
      iSOS          = 0;
      oSum          = nSum;

      /* Do the middle elimination loop */
      do {
        psdata->middleloops++;
        nSum += iSum;
        iSum = 0;

        /* Accumulate constraint bounds based on bounds on individual variables. */
        j = 0;
        while(presolve_statuscheck(psdata, &status) && psdata->forceupdate) {
          psdata->forceupdate = FALSE;
          /* Update sums, but limit iteration count to avoid possible
            "endless" loops with only marginal bound improvements */
          if(presolve_updatesums(psdata) && (j < MAX_PSBOUNDTIGHTENLOOPS)) {
            /* Do row preparation useful for subsequent column and row presolve operations */
            if((psdata->outerloops == 1) && (psdata->middleloops == 1))
              status = presolve_preparerows(psdata, &iBoundTighten, &iSum);
            nBoundTighten += iBoundTighten;
            iBoundTighten  = 0;
            nSum          += iSum;
            iSum           = 0;
            j++;
            if(status != RUNNING)
              report(lp, NORMAL, "presolve: Break after bound tightening iteration %d.\n", j);
          }
        }
        if(status != RUNNING)
          break;

        /* Do the relatively cheap innermost elimination loop */
        do {

          psdata->innerloops++;
          nSum += iSum;
          iSum = 0;

          /* Eliminate empty rows, convert row singletons to bounds,
            tighten bounds, and remove always satisfied rows */
          if(presolve_statuscheck(psdata, &status) &&
             is_presolve(lp, PRESOLVE_ROWS))
            status = presolve_rows(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iBoundTighten, &iSum);

          /* Eliminate empty or fixed columns (including trivial OF column singletons) */
          if(presolve_statuscheck(psdata, &status) &&
             is_presolve(lp, PRESOLVE_COLS))
            status = presolve_columns(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iBoundTighten, &iSum);

          /* Presolve SOS'es if possible (always do this) */
          if(presolve_statuscheck(psdata, &status))
            status = presolve_redundantSOS(psdata, &iBoundTighten, &iSum);

        } while((status == RUNNING) && (iSum > 0));
        if(status != RUNNING)
          break;

        /* Merge compatible similar rows; loop backwards over every row */
        if(presolve_statuscheck(psdata, &status) &&
           (psdata->outerloops == 1) && (psdata->middleloops <= MAX_PSMERGELOOPS) &&
           is_presolve(lp, PRESOLVE_MERGEROWS))
          status = presolve_mergerows(psdata, &iConRemove, &iSum);

        /* Eliminate dominated rows */
        if(presolve_statuscheck(psdata, &status) &&
           is_presolve(lp, PRESOLVE_ROWDOMINATE))
          presolve_rowdominance(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);

        /* See if we can convert some constraints to SOSes (only SOS1 handled) */
        if(presolve_statuscheck(psdata, &status) && (MIP_count(lp) > 0) &&
           is_presolve(lp, PRESOLVE_SOS))
          status = presolve_SOS1(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSOS, &iSum);

        /* Eliminate dominated columns in set coverage models */
        if(presolve_statuscheck(psdata, &status) && (lp->int_vars > 1) &&
           is_presolve(lp, PRESOLVE_COLDOMINATE))
          presolve_coldominance01(psdata, &iConRemove, &iVarFixed, &iSum);

        /* Aggregate compatible columns */
        if(presolve_statuscheck(psdata, &status) && /*TRUE ||*/
           is_presolve(lp, PRESOLVE_AGGREGATE))
          presolve_aggregate(psdata, &iConRemove, &iVarFixed, &iSum);

        /* Eliminate free variables and implied slacks */
        if(presolve_statuscheck(psdata, &status) &&
/*           !is_presolve(lp, PRESOLVE_ELIMEQ2) && */
           is_presolve(lp, PRESOLVE_IMPLIEDSLK | PRESOLVE_IMPLIEDFREE))
          status = presolve_freeandslacks(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);

      } while((status == RUNNING) && (iSum > 0));
      if(status != RUNNING)
        break;

      /* Check if we can do elimination of rank-deficient equality constraints */
      if(presolve_statuscheck(psdata, &status) && (psdata->EQmap->count > 1) &&
         is_presolve(lp, PRESOLVE_LINDEP)) {
#if 0
        REPORT_mat_mmsave(lp, "A.mtx", NULL, FALSE, "Constraint matrix A");
#endif
        presolve_singularities(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);
      }

      /* Eliminate variable and tighten bounds using 2-element EQs;
        note that this involves modifying the coefficients of A and
        can therefore be a slow operation. */
      if(presolve_statuscheck(psdata, &status) &&
         is_presolve(lp, PRESOLVE_ELIMEQ2)) {
        jjx = 0;
        do {
          jjx += iSum;
          status = presolve_elimeq2(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);
        } while((status == RUNNING) && (iSum > jjx));
        iSum = jjx;

#if 0
        /* Eliminate free variables and implied slacks */
        if(presolve_statuscheck(psdata, &status) &&
           is_presolve(lp, PRESOLVE_IMPLIEDSLK | PRESOLVE_IMPLIEDFREE))
          status = presolve_freeandslacks(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);
#endif
      }

      /* Increase A matrix sparsity by discovering common subsets using EQs */
      if(presolve_statuscheck(psdata, &status) && (psdata->EQmap->count > 0) &&
         is_presolve(lp, PRESOLVE_SPARSER))
        status = presolve_makesparser(psdata, &iCoeffChanged, &iConRemove, &iVarFixed, &iSum);

      /* Do GCD-based coefficient reductions (also does row scaling,
        even if no rhs INT truncations are possible) */
      if(presolve_statuscheck(psdata, &status) && (psdata->INTmap->count > 0) &&
         is_presolve(lp, PRESOLVE_REDUCEGCD))
        if(!presolve_reduceGCD(psdata, &iCoeffChanged, &iBoundTighten, &iSum))
          status = presolve_setstatus(psdata, INFEASIBLE);

      /* Simplify knapsack or set coverage models where OF coefficients are
        duplicated in the constraints.  At the cost of adding helper columns, this
        increases sparsity and facilitates identification of lower and upper bounds. */
      if(presolve_statuscheck(psdata, &status) &&
          is_presolve(lp, PRESOLVE_KNAPSACK)) {
        i = iCoeffChanged;
        status = presolve_knapsack(psdata, &iCoeffChanged);
      }

      /* Remove any "hanging" empty row and columns */
      if(status == RUNNING)
        status = presolve_shrink(psdata, &iConRemove, &iVarFixed);

      nCoeffChanged += iCoeffChanged;
      nConRemove    += iConRemove;
      nVarFixed     += iVarFixed;
      nBoundTighten += iBoundTighten;
      nSOS          += iSOS;
      nSum          += iSum;

      iSum           = iConRemove + iVarFixed + iBoundTighten + iCoeffChanged;
      if(iSum > 0)
        report(lp, NORMAL, "Presolve O:%d -> Reduced rows:%5d, cols:%5d --- changed bnds:%5d, Ab:%5d.\n",
                           psdata->outerloops, iConRemove, iVarFixed, iBoundTighten, iCoeffChanged);

   /* Do the outermost loop again if we were successful in this presolve sequences */
    } while(presolve_statuscheck(psdata, &status) &&
           (psdata->forceupdate || (oSum < nSum)) &&
           (psdata->outerloops < get_presolveloops(lp)) &&
           (psdata->rows->varmap->count+psdata->cols->varmap->count > 0));

   /* Finalize presolve */
#ifdef Paranoia
    i = presolve_debugcheck(lp, psdata->rows->varmap, psdata->cols->varmap);
    if(i > 0)
      report(lp, SEVERE, "presolve: %d internal consistency failure%s\n", i, my_plural_std(i));
    if((SOS_count(lp) > 0) && !presolve_SOScheck(psdata))
      report(lp, SEVERE, "presolve: SOS sparse member mapping problem - part 1\n");
#endif
    /* Perform bound relaxation to reduce chance of degeneracy. */
    if((status == RUNNING) && !is_presolve(lp, PRESOLVE_IMPLIEDFREE))
      jjx = presolve_makefree(psdata);
    else
      jjx = 0;


    /* Finalize the presolve */
    if(!presolve_finalize(psdata))
      report(lp, SEVERE, "presolve: Unable to construct internal data representation\n");

   /* Report summary information */
    i = NORMAL;
    iVarFixed  = lp->presolve_undo->orig_columns - psdata->cols->varmap->count;
    iConRemove = lp->presolve_undo->orig_rows - psdata->rows->varmap->count;
    if(nSum > 0)
      report(lp, i, "PRESOLVE             Elimination loops performed.......... O%d:M%d:I%d\n",
                                  psdata->outerloops, psdata->middleloops, psdata->innerloops);
    if(nVarFixed)
      report(lp, i, "            %8d empty or fixed variables............. %s.\n", nVarFixed, "REMOVED");
    if(nConRemove)
      report(lp, i, "            %8d empty or redundant constraints....... %s.\n", nConRemove, "REMOVED");
    if(nBoundTighten)
      report(lp, i, "            %8d bounds............................... %s.\n", nBoundTighten, "TIGHTENED");
    if(nCoeffChanged)
      report(lp, i, "            %8d matrix coefficients.................. %s.\n", nCoeffChanged, "CHANGED");
    if(jjx > 0)
      report(lp, i, "            %8d variables' final bounds.............. %s.\n", jjx, "RELAXED");
    if(nSOS)
      report(lp, i, "            %8d constraints detected as SOS1......... %s.\n", nSOS, "CONVERTED");

    /* Report optimality or infeasibility */
    if(status == UNBOUNDED)
      report(lp, NORMAL, "%20s Solution status detected............. %s.\n", "", "UNBOUNDED");
    else if(status == INFEASIBLE)
      report(lp, NORMAL, "%20s Solution status detected............. %s.\n", "", "INFEASIBLE");
    else {
      if(psdata->cols->varmap->count == 0)
        Value1 = Value2 = lp->presolve_undo->fixed_rhs[0] -initrhs0;
      else
        presolve_rangeorig(lp, 0, psdata->rows, &Value1, &Value2, -initrhs0);
      if((fabs(Value1 - Value2) < psdata->epsvalue) || (fabs(my_reldiff(Value1, Value2)) < psdata->epsvalue)) {
        if((lp->rows == 0) && (lp->columns == 0)) {
          status = PRESOLVED;
	  Value1 = my_chsign(is_maxim(lp), Value1);
          lp->solution[0] = Value1;
          lp->best_solution[0] = Value1;
          lp->full_solution[0] = Value1;
        }
        report(lp, NORMAL, "%20s OPTIMAL solution found............... %-g", "", Value1);
      }
      else if((status == RUNNING) && (i >= NORMAL)) {
        char lonum[20], upnum[20];
        if(my_infinite(lp, Value1))
          sprintf(lonum, "%13s", "-Inf");
        else
          sprintf(lonum, "%+12g", Value1);
        if(my_infinite(lp, Value2))
          sprintf(upnum, "%-13s", "Inf");
        else
          sprintf(upnum, "%+-12g", Value2);
        report(lp, i,    "%20s [ %s < Z < %s ]\n", "", lonum, upnum);
      }

      /* Update values for dual limit and best heuristic values */
      if((MIP_count(lp) > 0) || (get_Lrows(lp) > 0)) {
        if(is_maxim(lp)) {
          SETMAX(lp->bb_heuristicOF, Value1);
          SETMIN(lp->bb_limitOF, Value2);
        }
        else {
          SETMIN(lp->bb_heuristicOF, Value2);
          SETMAX(lp->bb_limitOF, Value1);
        }
      }
    }
    report(lp, NORMAL, " \n");

    /* Clean up (but save counts of constraint types for display later) */
    j = psdata->LTmap->count;
    jx = psdata->EQmap->count;
    jjx = lp->rows - j - jx;
    presolve_free(&psdata);

  }

  /* Signal that we are done presolving */
  if((lp->usermessage != NULL) &&
     ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != 0) && (lp->msgmask & MSG_PRESOLVE))
     lp->usermessage(lp, lp->msghandle, MSG_PRESOLVE);

  /* Create master SOS variable list */
  if(SOS_count(lp) > 0) {
    /*SOS_member_updatemap(lp->SOS); */
    make_SOSchain(lp, (MYBOOL) ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE));
  }

  /* Finalize model not identified as infeasible or unbounded */
  if(status == RUNNING) {

    /* Resolve GUBs */
    if(is_bb_mode(lp, NODE_GUBMODE))
      identify_GUB(lp, TRUE);

#if 0
    /* Mark rows containing hidden identity matrices so that supporting factorization
      engines can use this structural information to boost efficiency */
    if(is_algopt(lp, ALGOPT_COMPACTBPF))
      lp->bfpoptimize = (MYBOOL) (assist_factorization(lp, ROWTYPE_LOGICAL,
                                                       &lp->rowset1, &lp->rowno1) > 0);
#endif

    /* Scale the model based on current settings */
    auto_scale(lp);

    /* Crash the basis, if specified */
    crash_basis(lp);

    /* Produce presolved model statistics */
    if(nConRemove+nVarFixed+nBoundTighten+nVarFixed+nCoeffChanged > 0) {
      REPORT_modelinfo(lp, FALSE, "REDUCED");
      if(nSum > 0) {
        report(lp, NORMAL, "Row-types:   %7d LE,          %7d GE,             %7d EQ.\n",
                           j, jjx, jx);
        report(lp, NORMAL, " \n");
      }
    }
  }

  /* Optionally produce data on constraint classes */
  if(lp->verbose > NORMAL) {
    report(lp, NORMAL, " \n");
    REPORT_constraintinfo(lp, "CONSTRAINT CLASSES");
    report(lp, NORMAL, " \n");
  }

Finish:
  lp->wasPresolved  = TRUE;
  lp->timepresolved = timeNow();

#if 0
/*  write_mps(lp, "test_out.mps"); */ /* Must put here due to variable name mapping */
  write_lp(lp, "test_out.lp");   /* Must put here due to variable name mapping */
#endif
#if 0
  REPORT_debugdump(lp, "testint2.txt", FALSE);
#endif

  return( status );

}

STATIC MYBOOL postsolve(lprec *lp, int status)
{
  /* Verify solution */
  if(lp->lag_status != RUNNING) {
    int itemp;

    if(status == PRESOLVED)
      status = OPTIMAL;

    if((status == OPTIMAL) || (status == SUBOPTIMAL)) {
      itemp = check_solution(lp, lp->columns, lp->best_solution,
                                 lp->orig_upbo, lp->orig_lowbo, lp->epssolution);
      if((itemp != OPTIMAL) && (lp->spx_status == OPTIMAL))
        lp->spx_status = itemp;
      else if((itemp == OPTIMAL) && ((status == SUBOPTIMAL) || (lp->spx_status == PRESOLVED)))
        lp->spx_status = status;
    }
    else if(status != PRESOLVED) {
      report(lp, NORMAL, "lp_solve unsuccessful after %.0f iter and a last best value of %g\n",
             (double) get_total_iter(lp), lp->best_solution[0]);
      if(lp->bb_totalnodes > 0)
        report(lp, NORMAL, "lp_solve explored %.0f nodes before termination\n",
               (double) get_total_nodes(lp));
    }
    else
      lp->spx_status = OPTIMAL;

    /* Only rebuild primal solution here, since the dual is only computed on request */
    presolve_rebuildUndo(lp, TRUE);
  }

  /* Check if we can clear the variable map */
  if(varmap_canunlock(lp))
    lp->varmap_locked = FALSE;
#if 0
  REPORT_mat_mmsave(lp, "basis.mtx", NULL, FALSE);  /* Write the current basis matrix (no OF) */
#endif

  return( TRUE );
}
