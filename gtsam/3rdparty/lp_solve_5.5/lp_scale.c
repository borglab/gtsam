
#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_report.h"
#include "lp_scale.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


/*
    Scaling routines for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h, lp_scale.h

    Release notes:
    v5.0.0  1 January 2004      Significantly expanded and repackaged scaling
                                routines.
    v5.0.1  20 February 2004    Modified rounding behaviour in several areas.
    v5.1.0  20 July 2004        Reworked with flexible matrix storage model.
    v5.2.0  20 February 2005    Converted to matrix storage model without the OF.

   ----------------------------------------------------------------------------------
*/

/* First define scaling and unscaling primitives */

REAL scaled_value(lprec *lp, REAL value, int index)
{
  if(fabs(value) < lp->infinite) {
    if(lp->scaling_used)
      if(index > lp->rows)
        value /= lp->scalars[index];
      else
        value *= lp->scalars[index];
  }
  else
    value = my_sign(value)*lp->infinite;
  return(value);
}

REAL unscaled_value(lprec *lp, REAL value, int index)
{
  if(fabs(value) < lp->infinite) {
    if(lp->scaling_used)
      if(index > lp->rows)
        value *= lp->scalars[index];
      else
        value /= lp->scalars[index];
  }
  else
    value = my_sign(value)*lp->infinite;
  return(value);
}

STATIC REAL scaled_mat(lprec *lp, REAL value, int rownr, int colnr)
{
  if(lp->scaling_used)
    value *= lp->scalars[rownr] * lp->scalars[lp->rows + colnr];
  return( value );
}

STATIC REAL unscaled_mat(lprec *lp, REAL value, int rownr, int colnr)
{
  if(lp->scaling_used)
    value /= lp->scalars[rownr] * lp->scalars[lp->rows + colnr];
  return( value );
}

/* Compute the scale factor by the formulae:
      FALSE: SUM (log |Aij|) ^ 2
      TRUE:  SUM (log |Aij| - RowScale[i] - ColScale[j]) ^ 2 */
REAL CurtisReidMeasure(lprec *lp, MYBOOL _Advanced, REAL *FRowScale, REAL *FColScale)
{
  int      i, nz;
  REAL     absvalue, logvalue;
  register REAL result;
  MATrec   *mat = lp->matA;
  REAL     *value;
  int      *rownr, *colnr;

  /* Do OF part */
  result = 0;
  for(i = 1; i <= lp->columns; i++) {
    absvalue = fabs(lp->orig_obj[i]);
    if(absvalue > 0) {
      logvalue = log(absvalue);
      if(_Advanced)
        logvalue -= FRowScale[0] + FColScale[i];
      result += logvalue*logvalue;
    }
  }

  /* Do constraint matrix part */
  mat_validate(mat);
  value = &(COL_MAT_VALUE(0));
  rownr = &(COL_MAT_ROWNR(0));
  colnr = &(COL_MAT_COLNR(0));
  nz = get_nonzeros(lp);
  for(i = 0; i < nz;
      i++, value += matValueStep, rownr += matRowColStep, colnr += matRowColStep) {
    absvalue = fabs(*value);
    if(absvalue > 0) {
      logvalue = log(absvalue);
      if(_Advanced)
        logvalue -= FRowScale[*rownr] + FColScale[*colnr];
      result += logvalue*logvalue;
    }
  }
  return( result );
}

/* Implementation of the Curtis-Reid scaling based on the paper
   "On the Automatic Scaling of Matrices for Gaussian
    Elimination," Journal of the Institute of Mathematics and
    Its Applications (1972) 10, 118-124.

    Solve the system | M   E | (r)   (sigma)
                     |       | ( ) = (     )
                     | E^T N | (c)   ( tau )

    by the conjugate gradient method (clever recurrences).

    E is the matrix A with all elements = 1

    M is diagonal matrix of row    counts (RowCount)
    N is diagonal matrix of column counts (ColCount)

    sigma is the vector of row    logarithm sums (RowSum)
    tau   is the vector of column logarithm sums (ColSum)

    r, c are resulting row and column scalings (RowScale, ColScale) */

int CurtisReidScales(lprec *lp, MYBOOL _Advanced, REAL *FRowScale, REAL *FColScale)
{
  int    i, row, col, ent, nz;
  REAL   *RowScalem2, *ColScalem2,
         *RowSum, *ColSum,
         *residual_even, *residual_odd;
  REAL   sk,   qk,     ek,
         skm1, qkm1,   ekm1,
         qkm2, qkqkm1, ekm2, ekekm1,
         absvalue, logvalue,
         StopTolerance;
  int    *RowCount, *ColCount, colMax;
  int    Result;
  MATrec *mat = lp->matA;
  REAL   *value;
  int    *rownr, *colnr;

  if(CurtisReidMeasure(lp, _Advanced, FRowScale, FColScale)<0.1*get_nonzeros(lp))
  return(0);

  /* Allocate temporary memory and find RowSum and ColSum measures */
  nz = get_nonzeros(lp);
  colMax = lp->columns;

  allocREAL(lp, &RowSum, lp->rows+1, TRUE);
  allocINT(lp,  &RowCount, lp->rows+1, TRUE);
  allocREAL(lp, &residual_odd, lp->rows+1, TRUE);

  allocREAL(lp, &ColSum, colMax+1, TRUE);
  allocINT(lp,  &ColCount, colMax+1, TRUE);
  allocREAL(lp, &residual_even, colMax+1, TRUE);

  allocREAL(lp, &RowScalem2, lp->rows+1, FALSE);
  allocREAL(lp, &ColScalem2, colMax+1, FALSE);

  /* Set origin for row scaling */
  for(i = 1; i <= colMax; i++) {
    absvalue=fabs(lp->orig_obj[i]);
    if(absvalue>0) {
      logvalue = log(absvalue);
      ColSum[i] += logvalue;
      RowSum[0] += logvalue;
      ColCount[i]++;
      RowCount[0]++;
    }
  }

  value = &(COL_MAT_VALUE(0));
  rownr = &(COL_MAT_ROWNR(0));
  colnr = &(COL_MAT_COLNR(0));
  for(i = 0; i < nz;
      i++, value += matValueStep, rownr += matRowColStep, colnr += matRowColStep) {
    absvalue=fabs(*value);
    if(absvalue>0) {
      logvalue = log(absvalue);
      ColSum[*colnr] += logvalue;
      RowSum[*rownr] += logvalue;
      ColCount[*colnr]++;
      RowCount[*rownr]++;
    }
  }

  /* Make sure we dont't have division by zero errors */
  for(row = 0; row <= lp->rows; row++)
    if(RowCount[row] == 0)
      RowCount[row] = 1;
  for(col = 1; col <= colMax; col++)
    if(ColCount[col] == 0)
      ColCount[col] = 1;

  /* Initialize to RowScale = RowCount-1 RowSum
                   ColScale = 0.0
                   residual = ColSum - ET RowCount-1 RowSum */

  StopTolerance= MAX(lp->scalelimit-floor(lp->scalelimit), DEF_SCALINGEPS);
  StopTolerance *= (REAL) nz;
  for(row = 0; row <= lp->rows; row++) {
    FRowScale[row] = RowSum[row] / (REAL) RowCount[row];
    RowScalem2[row] = FRowScale[row];
  }

  /* Compute initial residual */
  for(col = 1; col <= colMax; col++) {
    FColScale[col] = 0;
    ColScalem2[col] = 0;
    residual_even[col] = ColSum[col];

    if(lp->orig_obj[col] != 0)
      residual_even[col] -= RowSum[0] / (REAL) RowCount[0];

    i = mat->col_end[col-1];
    rownr = &(COL_MAT_ROWNR(i));
    ent = mat->col_end[col];
    for(; i < ent;
        i++, rownr += matRowColStep) {
      residual_even[col] -= RowSum[*rownr] / (REAL) RowCount[*rownr];
    }
  }

  /* Compute sk */
  sk = 0;
  skm1 = 0;
  for(col = 1; col <= colMax; col++)
    sk += (residual_even[col]*residual_even[col]) / (REAL) ColCount[col];

  Result = 0;
  qk=1; qkm1=0; qkm2=0;
  ek=0; ekm1=0; ekm2=0;

  while(sk>StopTolerance) {
  /* Given the values of residual and sk, construct
     ColScale (when pass is even)
     RowScale (when pass is odd)  */

    qkqkm1 = qk * qkm1;
    ekekm1 = ek * ekm1;
    if((Result % 2) == 0) { /* pass is even; construct RowScale[pass+1] */
      if(Result != 0) {
        for(row = 0; row <= lp->rows; row++)
          RowScalem2[row] = FRowScale[row];
        if(qkqkm1 != 0) {
          for(row = 0; row <= lp->rows; row++)
            FRowScale[row]*=(1 + ekekm1 / qkqkm1);
          for(row = 0; row<=lp->rows; row++)
            FRowScale[row]+=(residual_odd[row] / (qkqkm1 * (REAL) RowCount[row]) -
                             RowScalem2[row] * ekekm1 / qkqkm1);
        }
      }
    }
    else { /* pass is odd; construct ColScale[pass+1] */
      for(col = 1; col <= colMax; col++)
        ColScalem2[col] = FColScale[col];
      if(qkqkm1 != 0) {
        for(col = 1; col <= colMax; col++)
          FColScale[col] *= (1 + ekekm1 / qkqkm1);
        for(col = 1; col <= colMax; col++)
          FColScale[col] += (residual_even[col] / ((REAL) ColCount[col] * qkqkm1) -
                             ColScalem2[col] * ekekm1 / qkqkm1);
      }
    }

    /* update residual and sk (pass + 1) */
    if((Result % 2) == 0) { /* even */
       /* residual */
      for(row = 0; row <= lp->rows; row++)
        residual_odd[row] *= ek;

      for(i = 1; i <= colMax; i++)
        if(lp->orig_obj[i] != 0)
          residual_odd[0] += (residual_even[i] / (REAL) ColCount[i]);

      rownr = &(COL_MAT_ROWNR(0));
      colnr = &(COL_MAT_COLNR(0));
      for(i = 0; i < nz;
          i++, rownr += matRowColStep, colnr += matRowColStep) {
        residual_odd[*rownr] += (residual_even[*colnr] / (REAL) ColCount[*colnr]);
      }
      for(row = 0; row <= lp->rows; row++)
        residual_odd[row] *= (-1 / qk);

      /* sk */
      skm1 = sk;
      sk = 0;
      for(row = 0; row <= lp->rows; row++)
        sk += (residual_odd[row]*residual_odd[row]) / (REAL) RowCount[row];
    }
    else { /* odd */
      /* residual */
      for(col = 1; col <= colMax; col++)
        residual_even[col] *= ek;

      for(i = 1; i <= colMax; i++)
        if(lp->orig_obj[i] != 0)
          residual_even[i] += (residual_odd[0] / (REAL) RowCount[0]);

      rownr = &(COL_MAT_ROWNR(0));
      colnr = &(COL_MAT_COLNR(0));
      for(i = 0; i < nz;
          i++, rownr += matRowColStep, colnr += matRowColStep) {
        residual_even[*colnr] += (residual_odd[*rownr] / (REAL) RowCount[*rownr]);
      }
      for(col = 1; col <= colMax; col++)
        residual_even[col] *= (-1 / qk);

      /* sk */
      skm1 = sk;
      sk = 0;
      for(col = 1; col <= colMax; col++)
        sk += (residual_even[col]*residual_even[col]) / (REAL) ColCount[col];
    }

    /* Compute ek and qk */
    ekm2=ekm1;
    ekm1=ek;
    ek=qk * sk / skm1;

    qkm2=qkm1;
    qkm1=qk;
    qk=1-ek;

    Result++;
  }

  /* Synchronize the RowScale and ColScale vectors */
  ekekm1 = ek * ekm1;
  if(qkm1 != 0)
  if((Result % 2) == 0) { /* pass is even, compute RowScale */
    for(row = 0; row<=lp->rows; row++)
      FRowScale[row]*=(1.0 + ekekm1 / qkm1);
    for(row = 0; row<=lp->rows; row++)
      FRowScale[row]+=(residual_odd[row] / (qkm1 * (REAL) RowCount[row]) -
                      RowScalem2[row] * ekekm1 / qkm1);
  }
  else { /* pass is odd, compute ColScale */
    for(col=1; col<=colMax; col++)
      FColScale[col]*=(1 + ekekm1 / qkm1);
    for(col=1; col<=colMax; col++)
      FColScale[col]+=(residual_even[col] / ((REAL) ColCount[col] * qkm1) -
                       ColScalem2[col] * ekekm1 / qkm1);
  }

  /* Do validation, if indicated */
  if(FALSE && mat_validate(mat)){
    double check, error;

    /* CHECK: M RowScale + E ColScale = RowSum */
    error = 0;
    for(row = 0; row <= lp->rows; row++) {
      check = (REAL) RowCount[row] * FRowScale[row];
      if(row == 0) {
        for(i = 1; i <= colMax; i++) {
          if(lp->orig_obj[i] != 0)
            check += FColScale[i];
        }
      }
      else {
        i = mat->row_end[row-1];
        ent = mat->row_end[row];
        for(; i < ent; i++) {
          col = ROW_MAT_COLNR(i);
          check += FColScale[col];
        }
      }
      check -= RowSum[row];
      error += check*check;
    }

    /* CHECK: E^T RowScale + N ColScale = ColSum */
    error = 0;
    for(col = 1; col <= colMax; col++) {
      check = (REAL) ColCount[col] * FColScale[col];

      if(lp->orig_obj[col] != 0)
        check += FRowScale[0];

      i = mat->col_end[col-1];
      ent = mat->col_end[col];
      rownr = &(COL_MAT_ROWNR(i));
      for(; i < ent;
          i++, rownr += matRowColStep) {
        check += FRowScale[*rownr];
      }
      check -= ColSum[col];
      error += check*check;
    }
  }

  /* Convert to scaling factors (rounding to nearest power
     of 2 can optionally be done as a separate step later) */
  for(col = 1; col <= colMax; col++) {
    absvalue = exp(-FColScale[col]);
    if(absvalue < MIN_SCALAR) absvalue = MIN_SCALAR;
    if(absvalue > MAX_SCALAR) absvalue = MAX_SCALAR;
    if(!is_int(lp,col) || is_integerscaling(lp))
        FColScale[col] = absvalue;
    else
        FColScale[col] = 1;
  }
  for(row = 0; row <= lp->rows; row++) {
    absvalue = exp(-FRowScale[row]);
    if(absvalue < MIN_SCALAR) absvalue = MIN_SCALAR;
    if(absvalue > MAX_SCALAR) absvalue = MAX_SCALAR;
    FRowScale[row] = absvalue;
  }

 /* free temporary memory */
  FREE(RowSum);
  FREE(ColSum);
  FREE(RowCount);
  FREE(ColCount);
  FREE(residual_even);
  FREE(residual_odd);
  FREE(RowScalem2);
  FREE(ColScalem2);

  return(Result);

}

STATIC MYBOOL scaleCR(lprec *lp, REAL *scaledelta)
{
  REAL *scalechange = NULL;
  int  Result;

  if(!lp->scaling_used) {
    allocREAL(lp, &lp->scalars, lp->sum_alloc + 1, FALSE);
    for(Result = 0; Result <= lp->sum; Result++)
      lp->scalars[Result] = 1;
    lp->scaling_used = TRUE;
  }

  if(scaledelta == NULL)
    allocREAL(lp, &scalechange, lp->sum + 1, FALSE);
  else
    scalechange = scaledelta;

  Result=CurtisReidScales(lp, FALSE, scalechange, &scalechange[lp->rows]);
  if(Result>0) {

    /* Do the scaling*/
    if(scale_updaterows(lp, scalechange, TRUE) ||
       scale_updatecolumns(lp, &scalechange[lp->rows], TRUE))
      lp->scalemode |= SCALE_CURTISREID;

    set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
  }

  if(scaledelta == NULL)
    FREE(scalechange);

  return((MYBOOL) (Result > 0));
}

STATIC MYBOOL transform_for_scale(lprec *lp, REAL *value)
{
  MYBOOL Accept = TRUE;
  *value = fabs(*value);
#ifdef Paranoia
  if(*value < lp->epsmachine) {
    Accept = FALSE;
    report(lp, SEVERE, "transform_for_scale: A zero-valued entry was passed\n");
  }
  else
#endif
  if(is_scalemode(lp, SCALE_LOGARITHMIC))
    *value = log(*value);
  else if(is_scalemode(lp, SCALE_QUADRATIC))
    (*value) *= (*value);
  return( Accept );
}

STATIC void accumulate_for_scale(lprec *lp, REAL *min, REAL *max, REAL value)
{
  if(transform_for_scale(lp, &value))
    if(is_scaletype(lp, SCALE_MEAN)) {
      *max += value;
      *min += 1;
    }
    else {
      SETMAX(*max, value);
      SETMIN(*min, value);
    }
}

STATIC REAL minmax_to_scale(lprec *lp, REAL min, REAL max, int itemcount)
{
  REAL scale;

  /* Initialize according to transformation / weighting model */
  if(is_scalemode(lp, SCALE_LOGARITHMIC))
    scale = 0;
  else
    scale = 1;
  if(itemcount <= 0)
    return(scale);

  /* Compute base scalar according to chosen scaling type */
  if(is_scaletype(lp, SCALE_MEAN)) {
    if(min > 0)
      scale = max / min;
  }
  else if(is_scaletype(lp, SCALE_RANGE))
    scale = (max + min) / 2;
  else if(is_scaletype(lp, SCALE_GEOMETRIC))
    scale = sqrt(min*max);
  else if(is_scaletype(lp, SCALE_EXTREME))
    scale = max;

  /* Compute final scalar according to transformation / weighting model */
  if(is_scalemode(lp, SCALE_LOGARITHMIC))
    scale = exp(-scale);
  else if(is_scalemode(lp, SCALE_QUADRATIC)) {
    if(scale == 0)
      scale = 1;
    else
      scale = 1 / sqrt(scale);
  }
  else {
    if(scale == 0)
      scale = 1;
    else
      scale = 1 / scale;
  }

  /* Make sure we are within acceptable scaling ranges */
  SETMAX(scale, MIN_SCALAR);
  SETMIN(scale, MAX_SCALAR);

  return(scale);
}

STATIC REAL roundPower2(REAL scale)
/* Purpose is to round a number to it nearest power of 2; in a system
   with binary number representation, this avoids rounding errors when
   scale is used to normalize another value */
{
  long int power2;
  MYBOOL   isSmall = FALSE;

  if(scale == 1)
    return( scale );

  /* Obtain the fractional power of 2 */
  if(scale < 2) {
    scale = 2 / scale;
    isSmall = TRUE;
  }
  else
    scale /= 2;
  scale = log(scale)/log(2.0);

  /* Find the desired nearest power of two and compute the associated scalar */
  power2 = (long) ceil(scale-0.5);
  scale = 1 << power2;
  if(isSmall)
    scale = 1.0 / scale;

  return( scale );

}

STATIC MYBOOL scale_updatecolumns(lprec *lp, REAL *scalechange, MYBOOL updateonly)
{
  int i, j;

  /* Verify that the scale change is significant (different from the unit) */
  for(i = lp->columns; i > 0; i--)
    if(fabs(scalechange[i]-1) > lp->epsprimal)
      break;
  if(i <= 0)
    return( FALSE );

 /* Update the pre-existing column scalar */
  if(updateonly)
    for(i = 1, j = lp->rows + 1; j <= lp->sum; i++, j++)
      lp->scalars[j] *= scalechange[i];
  else
    for(i = 1, j = lp->rows + 1; j <= lp->sum; i++, j++)
      lp->scalars[j] = scalechange[i];

  return( TRUE );
}

STATIC MYBOOL scale_updaterows(lprec *lp, REAL *scalechange, MYBOOL updateonly)
{
  int i;

  /* Verify that the scale change is significant (different from the unit) */
  for(i = lp->rows; i >= 0; i--) {
    if(fabs(scalechange[i]-1) > lp->epsprimal)
      break;
  }
  if(i < 0)
    return( FALSE );

 /* Update the pre-existing row scalar */
  if(updateonly)
    for(i = 0; i <= lp->rows; i++)
      lp->scalars[i] *= scalechange[i];
  else
    for(i = 0; i <= lp->rows; i++)
      lp->scalars[i] = scalechange[i];

  return( TRUE );
}

STATIC MYBOOL scale_columns(lprec *lp, REAL *scaledelta)
{
  int     i,j, colMax, nz;
  REAL    *scalechange;
  REAL    *value;
  int     *colnr;
  MATrec  *mat = lp->matA;

  /* Check that columns are in fact targeted */
  if((lp->scalemode & SCALE_ROWSONLY) != 0)
    return( TRUE );

  if(scaledelta == NULL)
    scalechange = &lp->scalars[lp->rows];
  else
    scalechange = &scaledelta[lp->rows];

  colMax = lp->columns;

  /* Scale matrix entries (including any Lagrangean constraints) */
  for(i = 1; i <= lp->columns; i++) {
    lp->orig_obj[i] *= scalechange[i];
  }

  mat_validate(lp->matA);
  nz = get_nonzeros(lp);
  value = &(COL_MAT_VALUE(0));
  colnr = &(COL_MAT_COLNR(0));
  for(i = 0; i < nz;
      i++, value += matValueStep, colnr += matRowColStep) {
    (*value) *= scalechange[*colnr];
  }

  /* Scale variable bounds as well */
  for(i = 1, j = lp->rows + 1; j <= lp->sum; i++, j++) {
    if(lp->orig_lowbo[j] > -lp->infinite)
      lp->orig_lowbo[j] /= scalechange[i];
    if(lp->orig_upbo[j] < lp->infinite)
      lp->orig_upbo[j] /= scalechange[i];
    if(lp->sc_lobound[i] != 0)
      lp->sc_lobound[i] /= scalechange[i];
  }

  lp->columns_scaled = TRUE;
  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);

  return( TRUE );
}

STATIC MYBOOL scale_rows(lprec *lp, REAL *scaledelta)
{
  int     i, j, nz, colMax;
  REAL    *scalechange;
  REAL    *value;
  int     *rownr;
  MATrec  *mat = lp->matA;


  /* Check that rows are in fact targeted */
  if((lp->scalemode & SCALE_COLSONLY) != 0)
    return( TRUE );

  if(scaledelta == NULL)
    scalechange = lp->scalars;
  else
    scalechange = scaledelta;

  colMax = lp->columns;

  /* First row-scale the matrix (including the objective function) */
  for(i = 1; i <= colMax; i++) {
    lp->orig_obj[i] *= scalechange[0];
  }

  nz = get_nonzeros(lp);
  value = &(COL_MAT_VALUE(0));
  rownr = &(COL_MAT_ROWNR(0));
  for(i = 0; i < nz;
      i++, value += matValueStep, rownr += matRowColStep) {
    (*value) *= scalechange[*rownr];
  }

  /* ...and scale the rhs and the row bounds (RANGES in MPS!!) */
  for(i = 0; i <= lp->rows; i++) {
    if(fabs(lp->orig_rhs[i]) < lp->infinite)
      lp->orig_rhs[i] *= scalechange[i];

    j = lp->presolve_undo->var_to_orig[i];
    if(j != 0)
      lp->presolve_undo->fixed_rhs[j] *= scalechange[i];

    if(lp->orig_upbo[i] < lp->infinite)     /* This is the range */
      lp->orig_upbo[i] *= scalechange[i];

    if((lp->orig_lowbo[i] != 0) && (fabs(lp->orig_lowbo[i]) < lp->infinite))
      lp->orig_lowbo[i] *= scalechange[i];
  }

  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);

  return( TRUE );
}

STATIC REAL scale(lprec *lp, REAL *scaledelta)
{
  int     i, j, nz, row_count, nzOF = 0;
  REAL    *row_max, *row_min, *scalechange = NULL, absval;
  REAL    col_max, col_min;
  MYBOOL  rowscaled, colscaled;
  MATrec  *mat = lp->matA;
  REAL    *value;
  int     *rownr;

  if(is_scaletype(lp, SCALE_NONE))
    return(0.0);

  if(!lp->scaling_used) {
    allocREAL(lp, &lp->scalars, lp->sum_alloc + 1, FALSE);
    for(i = 0; i <= lp->sum; i++) {
      lp->scalars[i] = 1;
    }
    lp->scaling_used = TRUE;
  }
#ifdef Paranoia
  else
    for(i = 0; i <= lp->sum; i++) {
      if(lp->scalars[i] == 0)
        report(lp, SEVERE, "scale: Zero-valued scalar found at index %d\n", i);
    }
#endif
  if(scaledelta == NULL)
    allocREAL(lp, &scalechange, lp->sum + 1, FALSE);
  else
    scalechange = scaledelta;

 /* Must initialize due to computation of scaling statistic - KE */
  for(i = 0; i <= lp->sum; i++)
    scalechange[i] = 1;

  row_count = lp->rows;
  allocREAL(lp, &row_max, row_count + 1, TRUE);
  allocREAL(lp, &row_min, row_count + 1, FALSE);

  /* Initialise min and max values of rows */
  for(i = 0; i <= row_count; i++) {
    if(is_scaletype(lp, SCALE_MEAN))
      row_min[i] = 0;             /* Carries the count of elements */
    else
      row_min[i] = lp->infinite;  /* Carries the minimum element */
  }

  /* Calculate row scaling data */
  for(j = 1; j <= lp->columns; j++) {

    absval = lp->orig_obj[j];
    if(absval != 0) {
      absval = scaled_mat(lp, absval, 0, j);
      accumulate_for_scale(lp, &row_min[0], &row_max[0], absval);
      nzOF++;
    }

    i = mat->col_end[j - 1];
    value = &(COL_MAT_VALUE(i));
    rownr = &(COL_MAT_ROWNR(i));
    nz = mat->col_end[j];
    for(; i < nz;
        i++, value += matValueStep, rownr += matRowColStep) {
      absval = scaled_mat(lp, *value, *rownr, j);
      accumulate_for_scale(lp, &row_min[*rownr], &row_max[*rownr], absval);
    }
  }

  /* Calculate scale factors for rows */
  i = 0;
  for(; i <= lp->rows; i++) {
    if(i == 0)
      nz = nzOF;
    else
      nz = mat_rowlength(lp->matA, i);
    absval = minmax_to_scale(lp, row_min[i], row_max[i], nzOF);
    if(absval == 0)
      absval = 1;
    scalechange[i] = absval;
  }

  FREE(row_max);
  FREE(row_min);

  /* Row-scale the matrix (including the objective function and Lagrangean constraints) */
  rowscaled = scale_updaterows(lp, scalechange, TRUE);

  /* Calculate column scales */
  i = 1;
  for(j = 1; j <= lp->columns; j++) {
    if(is_int(lp,j) && !is_integerscaling(lp)) { /* do not scale integer columns */
      scalechange[lp->rows + j] = 1;
    }
    else {
      col_max = 0;
      if(is_scaletype(lp, SCALE_MEAN))
        col_min = 0;
      else
        col_min = lp->infinite;

      absval = lp->orig_obj[j];
      if(absval != 0) {
        absval = scaled_mat(lp, absval, 0, j);
        accumulate_for_scale(lp, &col_min, &col_max, absval);
      }

      i = mat->col_end[j - 1];
      value = &(COL_MAT_VALUE(i));
      rownr = &(COL_MAT_ROWNR(i));
      nz = mat->col_end[j];
      for(; i < nz;
          i++, value += matValueStep, rownr += matRowColStep) {
        absval = scaled_mat(lp, *value, *rownr, j);
        accumulate_for_scale(lp, &col_min, &col_max, absval);
      }
      nz = mat_collength(lp->matA, j);
      if(fabs(lp->orig_obj[j]) > 0)
        nz++;
      scalechange[lp->rows + j] = minmax_to_scale(lp, col_min, col_max, nz);
    }
  }

  /* ... and then column-scale the already row-scaled matrix */
  colscaled = scale_updatecolumns(lp, &scalechange[lp->rows], TRUE);

  /* Create a geometric mean-type measure of the extent of scaling performed; */
  /* ideally, upon successive calls to scale() the value should converge to 0 */
  if(rowscaled || colscaled) {
    col_max = 0;
    for(j = 1; j <= lp->columns; j++)
      col_max += log(scalechange[lp->rows + j]);
    col_max = exp(col_max/lp->columns);

    i = 0;
    col_min = 0;
    for(; i <= lp->rows; i++)
      col_min += log(scalechange[i]);
    col_min = exp(col_min/row_count);
  }
  else {
    col_max = 1;
    col_min = 1;
  }

  if(scaledelta == NULL)
    FREE(scalechange);

  return(1 - sqrt(col_max*col_min));
}

STATIC MYBOOL finalize_scaling(lprec *lp, REAL *scaledelta)
{
  int i;

  /* Check if we should equilibrate */
  if(is_scalemode(lp, SCALE_EQUILIBRATE) && !is_scaletype(lp, SCALE_CURTISREID)) {
    int oldmode;

    oldmode = lp->scalemode;
    lp->scalemode = SCALE_LINEAR + SCALE_EXTREME;
    scale(lp, scaledelta);
    lp->scalemode = oldmode;
  }

  /* Check if we should prevent rounding errors */
  if(is_scalemode(lp, SCALE_POWER2)) {
    REAL *scalars;
    if(scaledelta == NULL)
      scalars = lp->scalars;
    else
      scalars = scaledelta;

    for(i = 0; i <= lp->sum; i++)
      scalars[i] = roundPower2(scalars[i]);
  }

  /* Then transfer the scalars to the model's data */
  return( scale_rows(lp, scaledelta) && scale_columns(lp, scaledelta) );

}

STATIC REAL auto_scale(lprec *lp)
{
  int    n = 1;
  REAL   scalingmetric = 0, *scalenew = NULL;

  if(lp->scaling_used &&
     ((((lp->scalemode & SCALE_DYNUPDATE) == 0)) || (lp->bb_level > 0)))
    return( scalingmetric);

  if(lp->scalemode != SCALE_NONE) {

    /* Allocate array for incremental scaling if appropriate */
    if((lp->solvecount > 1) && (lp->bb_level < 1) &&
       ((lp->scalemode & SCALE_DYNUPDATE) != 0))
      allocREAL(lp, &scalenew, lp->sum + 1, FALSE);

    if(is_scaletype(lp, SCALE_CURTISREID)) {
      scalingmetric = scaleCR(lp, scalenew);
    }
    else {
      REAL scalinglimit, scalingdelta;
      int  count;

      /* Integer value of scalelimit holds the maximum number of iterations; default to 1 */
      count = (int) floor(lp->scalelimit);
      scalinglimit = lp->scalelimit;
      if((count == 0) || (scalinglimit == 0)) {
        if(scalinglimit > 0)
          count = DEF_SCALINGLIMIT;  /* A non-zero convergence has been given, default to max 5 iterations */
        else
          count = 1;
      }
      else
        scalinglimit -= count;

      /* Scale to desired relative convergence or iteration limit */
      n = 0;
      scalingdelta = 1.0;
      scalingmetric = 1.0;
      while((n < count) && (fabs(scalingdelta) > scalinglimit)) {
        n++;
        scalingdelta = scale(lp, scalenew);
        scalingmetric = scalingmetric*(1+scalingdelta);
      }
      scalingmetric -= 1;
    }
  }

  /* Update the inf norm of the elements of the matrix (excluding the OF) */
  mat_computemax(lp->matA);

  /* Check if we really have to do scaling */
  if(lp->scaling_used && (fabs(scalingmetric) >= lp->epsprimal))
    /* Ok, do it */
    finalize_scaling(lp, scalenew);

  else {

    /* Otherwise reset scaling variables */
    if(lp->scalars != NULL) {
      FREE(lp->scalars);
    }
    lp->scaling_used = FALSE;
    lp->columns_scaled = FALSE;
  }
  if(scalenew != NULL)
    FREE(scalenew);

  return(scalingmetric);
}

STATIC void unscale_columns(lprec *lp)
{
  int     i, j, nz;
  MATrec  *mat = lp->matA;
  REAL    *value;
  int     *rownr, *colnr;

  if(!lp->columns_scaled)
    return;

  /* Unscale OF */
  for(j = 1; j <= lp->columns; j++) {
    lp->orig_obj[j] = unscaled_mat(lp, lp->orig_obj[j], 0, j);
  }

  /* Unscale mat */
  mat_validate(mat);
  nz = get_nonzeros(lp);
  value = &(COL_MAT_VALUE(0));
  rownr = &(COL_MAT_ROWNR(0));
  colnr = &(COL_MAT_COLNR(0));
  for(j = 0; j < nz;
      j++, value += matValueStep, rownr += matRowColStep, colnr += matRowColStep) {
    *value = unscaled_mat(lp, *value, *rownr, *colnr);
  }

  /* Unscale bounds as well */
  for(i = lp->rows + 1, j = 1; i <= lp->sum; i++, j++) {
    lp->orig_lowbo[i] = unscaled_value(lp, lp->orig_lowbo[i], i);
    lp->orig_upbo[i]  = unscaled_value(lp, lp->orig_upbo[i], i);
    lp->sc_lobound[j]  = unscaled_value(lp, lp->sc_lobound[j], i);
  }

  for(i = lp->rows + 1; i<= lp->sum; i++)
    lp->scalars[i] = 1;

  lp->columns_scaled = FALSE;
  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
}

void undoscale(lprec *lp)
{
  int     i, j, nz;
  MATrec  *mat = lp->matA;
  REAL    *value;
  int     *rownr, *colnr;

  if(lp->scaling_used) {

    /* Unscale the OF */
    for(j = 1; j <= lp->columns; j++) {
      lp->orig_obj[j] = unscaled_mat(lp, lp->orig_obj[j], 0, j);
    }

    /* Unscale the matrix */
    mat_validate(mat);
    nz = get_nonzeros(lp);
    value = &(COL_MAT_VALUE(0));
    rownr = &(COL_MAT_ROWNR(0));
    colnr = &(COL_MAT_COLNR(0));
    for(j = 0; j < nz;
        j++, value += matValueStep, rownr += matRowColStep, colnr += matRowColStep) {
      *value = unscaled_mat(lp, *value, *rownr, *colnr);
    }

    /* Unscale variable bounds */
    for(i = lp->rows + 1, j = 1; i <= lp->sum; i++, j++) {
      lp->orig_lowbo[i] = unscaled_value(lp, lp->orig_lowbo[i], i);
      lp->orig_upbo[i]  = unscaled_value(lp, lp->orig_upbo[i], i);
      lp->sc_lobound[j]  = unscaled_value(lp, lp->sc_lobound[j], i);
    }

    /* Unscale the rhs, upper and lower bounds... */
    for(i = 0; i <= lp->rows; i++) {
      lp->orig_rhs[i] = unscaled_value(lp, lp->orig_rhs[i], i);
      j = lp->presolve_undo->var_to_orig[i];
      if(j != 0)
        lp->presolve_undo->fixed_rhs[j] = unscaled_value(lp, lp->presolve_undo->fixed_rhs[j], i);
      lp->orig_lowbo[i] = unscaled_value(lp, lp->orig_lowbo[i], i);
      lp->orig_upbo[i] = unscaled_value(lp, lp->orig_upbo[i], i);
    }

    FREE(lp->scalars);
    lp->scaling_used = FALSE;
    lp->columns_scaled = FALSE;

    set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
  }
}

