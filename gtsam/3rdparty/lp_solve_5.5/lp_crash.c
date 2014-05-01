
/*
   ----------------------------------------------------------------------------------
   Crash management routines in lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h, lp_utils.h, lp_matrix.h

    Release notes:
    v1.0.0  1 April   2004      First version.
    v1.1.0  20 July 2004        Reworked with flexible matrix storage model.

   ----------------------------------------------------------------------------------
*/

#include <string.h>

#include "commonlib.h"
#include "lp_lib.h"
#include "lp_scale.h"
#include "lp_utils.h"
#include "lp_report.h"
#include "lp_matrix.h"
#include "lp_crash.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


MYBOOL crash_basis(lprec *lp)
{
  int     i;
  MATrec  *mat = lp->matA;
  MYBOOL  ok = TRUE;

  /* Initialize basis indicators */
  if(lp->basis_valid)
    lp->var_basic[0] = FALSE;
  else
    default_basis(lp);

  /* Set initial partial pricing blocks */
  if(lp->rowblocks != NULL)
    lp->rowblocks->blocknow = 1;
  if(lp->colblocks != NULL)
    lp->colblocks->blocknow = ((lp->crashmode == CRASH_NONE) || (lp->colblocks->blockcount == 1) ? 1 : 2);

  /* Construct a basis that is in some measure the "most feasible" */
  if((lp->crashmode == CRASH_MOSTFEASIBLE) && mat_validate(mat)) {
    /* The logic here follows Maros */
    LLrec   *rowLL = NULL, *colLL = NULL;
    int     ii, rx, cx, ix, nz;
    REAL    wx, tx, *rowMAX = NULL, *colMAX = NULL;
    int     *rowNZ = NULL, *colNZ = NULL, *rowWT = NULL, *colWT = NULL;
    REAL    *value;
    int     *rownr, *colnr;

    report(lp, NORMAL, "crash_basis: 'Most feasible' basis crashing selected\n");

    /* Tally row and column non-zero counts */
    ok = allocINT(lp,  &rowNZ, lp->rows+1,     TRUE) &&
         allocINT(lp,  &colNZ, lp->columns+1,  TRUE) &&
         allocREAL(lp, &rowMAX, lp->rows+1,    FALSE) &&
         allocREAL(lp, &colMAX, lp->columns+1, FALSE);
    if(!ok)
      goto Finish;

    nz = mat_nonzeros(mat);
    rownr = &COL_MAT_ROWNR(0);
    colnr = &COL_MAT_COLNR(0);
    value = &COL_MAT_VALUE(0);
    for(i = 0; i < nz;
        i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep) {
      rx = *rownr;
      cx = *colnr;
      wx = fabs(*value);
      rowNZ[rx]++;
      colNZ[cx]++;
      if(i == 0) {
        rowMAX[rx] = wx;
        colMAX[cx] = wx;
        colMAX[0]  = wx;
      }
      else {
        SETMAX(rowMAX[rx], wx);
        SETMAX(colMAX[cx], wx);
        SETMAX(colMAX[0],  wx);
      }
    }
    /* Reduce counts for small magnitude to preserve stability */
    rownr = &COL_MAT_ROWNR(0);
    colnr = &COL_MAT_COLNR(0);
    value = &COL_MAT_VALUE(0);
    for(i = 0; i < nz;
        i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep) {
      rx = *rownr;
      cx = *colnr;
      wx = fabs(*value);
#ifdef CRASH_SIMPLESCALE
      if(wx < CRASH_THRESHOLD * colMAX[0]) {
        rowNZ[rx]--;
        colNZ[cx]--;
      }
#else
      if(wx < CRASH_THRESHOLD * rowMAX[rx])
        rowNZ[rx]--;
      if(wx < CRASH_THRESHOLD * colMAX[cx])
        colNZ[cx]--;
#endif
    }

    /* Set up priority tables */
    ok = allocINT(lp, &rowWT, lp->rows+1, TRUE);
    createLink(lp->rows,    &rowLL, NULL);
    ok &= (rowLL != NULL);
    if(!ok)
      goto Finish;
    for(i = 1; i <= lp->rows; i++) {
      if(get_constr_type(lp, i)==EQ)
        ii = 3;
      else if(lp->upbo[i] < lp->infinite)
        ii = 2;
      else if(fabs(lp->rhs[i]) < lp->infinite)
        ii = 1;
      else
        ii = 0;
      rowWT[i] = ii;
      if(ii > 0)
        appendLink(rowLL, i);
    }
    ok = allocINT(lp, &colWT, lp->columns+1, TRUE);
    createLink(lp->columns, &colLL, NULL);
    ok &= (colLL != NULL);
    if(!ok)
      goto Finish;
    for(i = 1; i <= lp->columns; i++) {
      ix = lp->rows+i;
      if(is_unbounded(lp, i))
        ii = 3;
      else if(lp->upbo[ix] >= lp->infinite)
        ii = 2;
      else if(fabs(lp->upbo[ix]-lp->lowbo[ix]) > lp->epsmachine)
        ii = 1;
      else
        ii = 0;
      colWT[i] = ii;
      if(ii > 0)
        appendLink(colLL, i);
    }

    /* Loop over all basis variables */
    for(i = 1; i <= lp->rows; i++) {

      /* Select row */
      rx = 0;
      wx = -lp->infinite;
      for(ii = firstActiveLink(rowLL); ii > 0; ii = nextActiveLink(rowLL, ii)) {
        tx = rowWT[ii] - CRASH_SPACER*rowNZ[ii];
        if(tx > wx) {
          rx = ii;
          wx = tx;
        }
      }
      if(rx == 0)
        break;
      removeLink(rowLL, rx);

      /* Select column */
      cx = 0;
      wx = -lp->infinite;
      for(ii = mat->row_end[rx-1]; ii < mat->row_end[rx]; ii++) {

        /* Update NZ column counts for row selected above */
        tx = fabs(ROW_MAT_VALUE(ii));
        ix = ROW_MAT_COLNR(ii);
#ifdef CRASH_SIMPLESCALE
        if(tx >= CRASH_THRESHOLD * colMAX[0])
#else
        if(tx >= CRASH_THRESHOLD * colMAX[ix])
#endif
          colNZ[ix]--;
        if(!isActiveLink(colLL, ix) || (tx < CRASH_THRESHOLD * rowMAX[rx]))
          continue;

        /* Now do the test for best pivot */
        tx = my_sign(lp->orig_obj[ix]) - my_sign(ROW_MAT_VALUE(ii));
        tx = colWT[ix] + CRASH_WEIGHT*tx - CRASH_SPACER*colNZ[ix];
        if(tx > wx) {
          cx = ix;
          wx = tx;
        }
      }
      if(cx == 0)
        break;
      removeLink(colLL, cx);

      /* Update row NZ counts */
      ii = mat->col_end[cx-1];
      rownr = &COL_MAT_ROWNR(ii);
      value = &COL_MAT_VALUE(ii);
      for(; ii < mat->col_end[cx];
          ii++, rownr += matRowColStep, value += matValueStep) {
        wx = fabs(*value);
        ix = *rownr;
#ifdef CRASH_SIMPLESCALE
        if(wx >= CRASH_THRESHOLD * colMAX[0])
#else
        if(wx >= CRASH_THRESHOLD * rowMAX[ix])
#endif
          rowNZ[ix]--;
      }

      /* Set new basis variable */
      set_basisvar(lp, rx, lp->rows+cx);
    }

    /* Clean up */
Finish:
    FREE(rowNZ);
    FREE(colNZ);
    FREE(rowMAX);
    FREE(colMAX);
    FREE(rowWT);
    FREE(colWT);
    freeLink(&rowLL);
    freeLink(&colLL);
  }

  /* Construct a basis that is in some measure the "least degenerate" */
  else if((lp->crashmode == CRASH_LEASTDEGENERATE) && mat_validate(mat)) {
    /* The logic here follows Maros */
    LLrec   *rowLL = NULL, *colLL = NULL;
    int     ii, rx, cx, ix, nz, *merit = NULL;
    REAL    *value, wx, hold, *rhs = NULL, *eta = NULL;
    int     *rownr, *colnr;

    report(lp, NORMAL, "crash_basis: 'Least degenerate' basis crashing selected\n");

    /* Create temporary arrays */
    ok = allocINT(lp,  &merit, lp->columns + 1, FALSE) &&
         allocREAL(lp, &eta, lp->rows + 1, FALSE) &&
         allocREAL(lp, &rhs, lp->rows + 1, FALSE);
    createLink(lp->columns, &colLL, NULL);
    createLink(lp->rows, &rowLL, NULL);
    ok &= (colLL != NULL) && (rowLL != NULL);
    if(!ok)
      goto FinishLD;
    MEMCOPY(rhs, lp->orig_rhs, lp->rows + 1);
    for(i = 1; i <= lp->columns; i++)
      appendLink(colLL, i);
    for(i = 1; i <= lp->rows; i++)
      appendLink(rowLL, i);

    /* Loop until we have found enough new bases */
    while(colLL->count > 0) {

      /* Tally non-zeros matching in RHS and each active column */
      nz = mat_nonzeros(mat);
      rownr = &COL_MAT_ROWNR(0);
      colnr = &COL_MAT_COLNR(0);
      ii = 0;
      MEMCLEAR(merit, lp->columns + 1);
      for(i = 0; i < nz;
          i++, rownr += matRowColStep, colnr += matRowColStep) {
        rx = *rownr;
        cx = *colnr;
        if(isActiveLink(colLL, cx) && (rhs[rx] != 0)) {
          merit[cx]++;
          ii++;
        }
      }
      if(ii == 0)
        break;

      /* Find maximal match; break ties with column length */
      i = firstActiveLink(colLL);
      cx = i;
      for(i = nextActiveLink(colLL, i); i != 0; i = nextActiveLink(colLL, i)) {
        if(merit[i] >= merit[cx]) {
          if((merit[i] > merit[cx]) || (mat_collength(mat, i) > mat_collength(mat, cx)))
            cx = i;
        }
      }

      /* Determine the best pivot row */
      i = mat->col_end[cx-1];
      nz = mat->col_end[cx];
      rownr = &COL_MAT_ROWNR(i);
      value = &COL_MAT_VALUE(i);
      rx = 0;
      wx = 0;
      MEMCLEAR(eta, lp->rows + 1);
      for(; i < nz;
          i++, rownr += matRowColStep, value += matValueStep) {
        ix = *rownr;
        hold = *value;
        eta[ix] = rhs[ix] / hold;
        hold = fabs(hold);
        if(isActiveLink(rowLL, ix) && (hold > wx)) {
          wx = hold;
          rx = ix;
        }
      }

      /* Set new basis variable */
      if(rx > 0) {

        /* We have to update the rhs vector for the implied transformation
          in order to be able to find the new RHS non-zero pattern */
        for(i = 1; i <= lp->rows; i++)
           rhs[i] -= wx * eta[i];
        rhs[rx] = wx;

        /* Do the exchange */
        set_basisvar(lp, rx, lp->rows+cx);
        removeLink(rowLL, rx);
      }
      removeLink(colLL, cx);

    }

    /* Clean up */
FinishLD:
    FREE(merit);
    FREE(rhs);
    freeLink(&rowLL);
    freeLink(&colLL);

  }
  return( ok );
}

#if 0
MYBOOL __WINAPI guess_basis(lprec *lp, REAL *guessvector, int *basisvector)
{
  MYBOOL status = FALSE;
  REAL   *values = NULL, *violation = NULL,
         *value, error, upB, loB, sortorder = 1.0;
  int    i, n, *rownr, *colnr;
  MATrec *mat = lp->matA;

  if(!mat_validate(lp->matA))
    return( status );

  /* Create helper arrays */
  if(!allocREAL(lp, &values, lp->sum+1, TRUE) ||
     !allocREAL(lp, &violation, lp->sum+1, TRUE))
    goto Finish;

  /* Compute values of slack variables for given guess vector */
  i = 0;
  n = get_nonzeros(lp);
  rownr = &COL_MAT_ROWNR(i);
  colnr = &COL_MAT_COLNR(i);
  value = &COL_MAT_VALUE(i);
  for(; i < n; i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep)
    values[*rownr] += unscaled_mat(lp, my_chsign(is_chsign(lp, *rownr), *value), *rownr, *colnr) *
                      guessvector[*colnr];
  MEMMOVE(values+lp->rows+1, guessvector+1, lp->columns);

  /* Initialize constraint bound violation measures */
  for(i = 1; i <= lp->rows; i++) {
    upB = get_rh_upper(lp, i);
    loB = get_rh_lower(lp, i);
    error = values[i] - upB;
    if(error > lp->epsprimal)
      violation[i] = sortorder*error;
    else {
      error = loB - values[i];
      if(error > lp->epsprimal)
        violation[i] = sortorder*error;
      else if(is_infinite(lp, loB) && is_infinite(lp, upB))
        ;
      else if(is_infinite(lp, upB))
        violation[i] = sortorder*(loB - values[i]);
      else if(is_infinite(lp, loB))
        violation[i] = sortorder*(values[i] - upB);
      else
        violation[i] = - sortorder*MAX(upB - values[i], values[i] - loB);
    }
    basisvector[i] = i;
  }

  /* Initialize user variable bound violation measures */
  for(i = 1; i <= lp->columns; i++) {
    n = lp->rows+i;
    upB = get_upbo(lp, i);
    loB = get_lowbo(lp, i);
    error = guessvector[i] - upB;
    if(error > lp->epsprimal)
      violation[n] = sortorder*error;
    else {
      error = loB - values[n];
      if(error > lp->epsprimal)
        violation[n] = sortorder*error;
      else if(is_infinite(lp, loB) && is_infinite(lp, upB))
        ;
      else if(is_infinite(lp, upB))
        violation[n] = sortorder*(loB - values[n]);
      else if(is_infinite(lp, loB))
        violation[n] = sortorder*(values[n] - upB);
      else
        violation[n] = - sortorder*MAX(upB - values[n], values[n] - loB);
    }
    basisvector[n] = n;
  }

  /* Sort decending by violation; this means that variables with
     the largest violations will be designated as basic */
  sortByREAL(basisvector, violation, lp->sum, 1, FALSE);

  /* Adjust the non-basic indeces for the (proximal) bound state */
  error = lp->epsprimal;
  for(i = lp->rows+1, rownr = basisvector+i; i <= lp->sum; i++, rownr++) {
    if(*rownr <= lp->rows) {
      if(values[*rownr] <= get_rh_lower(lp, *rownr)+error)
        *rownr = -(*rownr);
    }
    else
      if(values[i] <= get_lowbo(lp, (*rownr)-lp->rows)+error)
        *rownr = -(*rownr);
  }

  /* Clean up and return status */
  status = (MYBOOL) (violation[1] == 0);
Finish:
  FREE(values);
  FREE(violation);


  return( status );
}
#endif

#if 0
MYBOOL __WINAPI guess_basis(lprec *lp, REAL *guessvector, int *basisvector)
{
  MYBOOL *isnz, status = FALSE;
  REAL   *values = NULL, *violation = NULL,
         eps = lp->epsprimal,
         *value, error, upB, loB, sortorder = 1.0;
  int    i, j, n, *rownr, *colnr, *slkpos,
         nrows = lp->rows, ncols = lp->columns;
  MATrec *mat = lp->matA;

  if(!mat_validate(mat))
    return( status );

  /* Create helper arrays */
  if(!allocREAL(lp, &values, lp->sum+1, TRUE) ||
     !allocREAL(lp, &violation, lp->sum+1, TRUE))
    goto Finish;

  /* Compute values of slack variables for given guess vector */
  i = 0;
  n = get_nonzeros(lp);
  rownr = &COL_MAT_ROWNR(i);
  colnr = &COL_MAT_COLNR(i);
  value = &COL_MAT_VALUE(i);
  for(; i < n; i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep)
    values[*rownr] += unscaled_mat(lp, my_chsign(is_chsign(lp, *rownr), *value), *rownr, *colnr) *
                      guessvector[*colnr];
  MEMMOVE(values+nrows+1, guessvector+1, ncols);

  /* Initialize constraint bound violation measures (expressed as positive values) */
  for(i = 1; i <= nrows; i++) {
    upB = get_rh_upper(lp, i);
    loB = get_rh_lower(lp, i);
    error = values[i] - upB;
    if(error > eps)
      violation[i] = sortorder*error;
    else {
      error = loB - values[i];
      if(error > eps)
        violation[i] = sortorder*error;
      else if(my_infinite(lp, loB) && my_infinite(lp, upB))
        ;
      else if(my_infinite(lp, upB))
        violation[i] = sortorder*(loB - values[i]);
      else if(my_infinite(lp, loB))
        violation[i] = sortorder*(values[i] - upB);
      else
        violation[i] = -sortorder*MAX(upB - values[i], values[i] - loB);
    }
    basisvector[i] = i;
  }

  /* Initialize user variable bound violation measures (expressed as positive values) */
  for(i = 1; i <= ncols; i++) {
    n = nrows+i;
    upB = get_upbo(lp, i);
    loB = get_lowbo(lp, i);
    error = guessvector[i] - upB;
    if(error > eps)
      violation[n] = sortorder*error;
    else {
      error = loB - values[n];
      if(error > eps)
        violation[n] = sortorder*error;
      else if(my_infinite(lp, loB) && my_infinite(lp, upB))
        ;
      else if(my_infinite(lp, upB))
        violation[n] = sortorder*(loB - values[n]);
      else if(my_infinite(lp, loB))
        violation[n] = sortorder*(values[n] - upB);
      else
        violation[n] = -sortorder*MAX(upB - values[n], values[n] - loB);
    }
    basisvector[n] = n;
  }

  /* Sort decending by violation; this means that variables with
     the largest violations will be designated as basic */
  sortByREAL(basisvector, violation, lp->sum, 1, FALSE);
  error = violation[1];

  /* Adjust the non-basic indeces for the (proximal) bound state */
  for(i = nrows+1, rownr = basisvector+i; i <= lp->sum; i++, rownr++) {
    if(*rownr <= nrows) {
      if(values[*rownr] <= get_rh_lower(lp, *rownr)+eps)
        *rownr = -(*rownr);
    }
    else
      if(values[i] <= get_lowbo(lp, (*rownr)-nrows)+eps)
        *rownr = -(*rownr);
  }

#if 1
  /* Let us check for obvious row singularities and try to fix these;
     First assemble necessary basis statistics... */
  isnz = (MYBOOL *) values;
  MEMCLEAR(isnz, nrows+1);
  slkpos = (int *) violation;
  MEMCLEAR(slkpos, nrows+1);
  for(i = 1; i <= nrows; i++) {
    j = abs(basisvector[i]);
    if(j <= nrows) {
      isnz[j] = TRUE;
      slkpos[j] = i;
    }
    else {
      j-= nrows;
      j = mat->col_end[j-1];
      isnz[COL_MAT_ROWNR(j)] = TRUE;
      /*isnz[COL_MAT_ROWNR(j+1)] = TRUE;*/
    }
  }
  for(; i <= lp->sum; i++) {
    j = abs(basisvector[i]);
    if(j <= nrows)
      slkpos[j] = i;
  }

  /* ...then set the corresponding slacks basic for row rank deficient positions */
  for(j = 1; j <= nrows; j++) {
#ifdef Paranoia
    if(slkpos[j] == 0)
      report(lp, SEVERE, "guess_basis: Internal error");
#endif
    if(!isnz[j]) {
      isnz[j] = TRUE;
      i = slkpos[j];
      swapINT(&basisvector[i], &basisvector[j]);
      basisvector[j] = abs(basisvector[j]);
    }
  }
#endif

  /* Clean up and return status */
  status = (MYBOOL) (error <= eps);
Finish:
  FREE(values);
  FREE(violation);

  return( status );
}
#endif

MYBOOL __WINAPI guess_basis(lprec *lp, REAL *guessvector, int *basisvector)
{
  MYBOOL *isnz, status = FALSE;
  REAL   *values = NULL, *violation = NULL,
         eps = lp->epsprimal,
         *value, error, upB, loB, sortorder = 1.0;
  int    i, j, jj, n, *rownr, *colnr, *slkpos,
         nrows = lp->rows, ncols = lp->columns;
  MATrec *mat = lp->matA;

  if(!mat_validate(mat))
    return( status );

  /* Create helper arrays */
  if(!allocREAL(lp, &values, lp->sum+1, TRUE) ||
     !allocREAL(lp, &violation, lp->sum+1, TRUE))
    goto Finish;

  /* Compute values of slack variables for given guess vector */
  i = 0;
  n = get_nonzeros(lp);
  rownr = &COL_MAT_ROWNR(i);
  colnr = &COL_MAT_COLNR(i);
  value = &COL_MAT_VALUE(i);
  for(; i < n; i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep)
    values[*rownr] += unscaled_mat(lp, my_chsign(is_chsign(lp, *rownr), *value), *rownr, *colnr) *
                      guessvector[*colnr];
  MEMMOVE(values+nrows+1, guessvector+1, ncols);

  /* Initialize constraint bound violation measures (expressed as positive values) */
  for(i = 1; i <= nrows; i++) {
    upB = get_rh_upper(lp, i);
    loB = get_rh_lower(lp, i);
    error = values[i] - upB;
    if(error > -eps)
      violation[i] = sortorder*MAX(0,error);
    else {
      error = loB - values[i];
      if(error > -eps)
        violation[i] = sortorder*MAX(0,error);
      else if(my_infinite(lp, loB) && my_infinite(lp, upB))
        ;
      else if(my_infinite(lp, upB))
        violation[i] = sortorder*(loB - values[i]);
      else if(my_infinite(lp, loB))
        violation[i] = sortorder*(values[i] - upB);
      else
        violation[i] = -sortorder*MAX(upB - values[i], values[i] - loB);
    }
    basisvector[i] = i;
  }

  /* Initialize user variable bound violation measures (expressed as positive values) */
  for(i = 1; i <= ncols; i++) {
    n = nrows+i;
    upB = get_upbo(lp, i);
    loB = get_lowbo(lp, i);
    error = guessvector[i] - upB;
    if(error > -eps)
      violation[n] = sortorder*MAX(0,error);
    else {
      error = loB - values[n];
      if(error > -eps)
        violation[n] = sortorder*MAX(0,error);
      else if(my_infinite(lp, loB) && my_infinite(lp, upB))
        ;
      else if(my_infinite(lp, upB))
        violation[n] = sortorder*(loB - values[n]);
      else if(my_infinite(lp, loB))
        violation[n] = sortorder*(values[n] - upB);
      else
        violation[n] = -sortorder*MAX(upB - values[n], values[n] - loB);
    }
    basisvector[n] = n;
  }

  /* Sort decending by violation; this means that variables with
     the largest violations will be designated as basic */
  sortByREAL(basisvector, violation, lp->sum, 1, FALSE);
  error = violation[1];

  /* Adjust the non-basic indeces for the (proximal) bound state */
  for(i = nrows+1, rownr = basisvector+i; i <= lp->sum; i++, rownr++) {
    if(*rownr <= nrows) {
      values[*rownr] -= lp->orig_rhs[*rownr];
      if(values[*rownr] <= eps)
        *rownr = -(*rownr);
    }
    else
      if(values[i] <= get_lowbo(lp, (*rownr)-nrows)+eps)
        *rownr = -(*rownr);
  }

  /* Let us check for obvious row singularities and try to fix these;
     First assemble necessary basis statistics... */
  isnz = (MYBOOL *) values;
  MEMCLEAR(isnz, nrows+1);
  slkpos = (int *) violation;
  MEMCLEAR(slkpos, nrows+1);
  for(i = 1; i <= nrows; i++) {
    j = abs(basisvector[i]);
    if(j <= nrows) {
      isnz[j] = TRUE;
      slkpos[j] = i;
    }
    else {
      j-= nrows;
      jj = mat->col_end[j-1];
      isnz[COL_MAT_ROWNR(jj)] = TRUE;
/*      if(++jj < mat->col_end[j])
        isnz[COL_MAT_ROWNR(jj)] = TRUE; */
    }
  }
  for(; i <= lp->sum; i++) {
    j = abs(basisvector[i]);
    if(j <= nrows)
      slkpos[j] = i;
  }

  /* ...then set the corresponding slacks basic for row rank deficient positions */
  for(j = 1; j <= nrows; j++) {
#ifdef Paranoia
    if(slkpos[j] == 0)
      report(lp, SEVERE, "guess_basis: Internal error");
#endif
    if(!isnz[j]) {
      isnz[j] = TRUE;
      i = slkpos[j];
      swapINT(&basisvector[i], &basisvector[j]);
      basisvector[j] = abs(basisvector[j]);
    }
  }

  /* Lastly normalize all basic variables to be coded as lower-bounded */
  for(i = 1; i <= nrows; i++)
    basisvector[i] = -abs(basisvector[i]);

  /* Clean up and return status */
  status = (MYBOOL) (error <= eps);
Finish:
  FREE(values);
  FREE(violation);

  return( status );
}
