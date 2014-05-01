/*
    Minimum matrix inverse fill-in modules - interface for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Kjell Eikland 
    Contact:       kjell.eikland@broadpark.no 
    License terms: LGPL.
    
    Requires:      string.h, colamd.h, lp_lib.h

    Release notes:
    v1.0    1 September 2003    Preprocessing routines for minimum fill-in column 
                                ordering for inverse factorization using the open 
                                source COLAMD library.  Suitable for the dense parts
                                of both the product form and LU factorization inverse 
                                methods.
    v1.1    1 July 2004         Renamed from lp_colamdMDO to lp_MDO.                                

   ---------------------------------------------------------------------------------- 
*/

#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "colamd.h"
#include "lp_MDO.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

STATIC MYBOOL includeMDO(MYBOOL *usedpos, int item)
{
/*  Legend:   TRUE            => A basic slack variable already in the basis
              FALSE           => A column free for being pivoted in
              AUTOMATIC+TRUE  => A row-singleton user column pivoted into the basis
              AUTOMATIC+FALSE => A column-singleton user column pivoted into the basis */

  /* Handle case where we are processing all columns */
  if(usedpos == NULL)
    return( TRUE );
    
  else {
  /* Otherwise do the selective case */
    MYBOOL test = usedpos[item];
#if 1
    return( test != TRUE );
#else
    test = test & TRUE;
    return( test == FALSE );
#endif
  }
}

STATIC int prepareMDO(lprec *lp, MYBOOL *usedpos, int *colorder, int *data, int *rowmap)
/* This routine prepares data structures for colamd().  It is called twice, the first
   time to count applicable non-zero elements by column, and the second time to fill in 
   the row indexes of the non-zero values from the first call.  Note that the colamd() 
   row index base is 0 (which suits lp_solve fine). */
{
  int     i, ii, j, k, kk;
  int     nrows = lp->rows+1, ncols = colorder[0];
  int     offset = 0, Bnz = 0, Tnz;
  MYBOOL  dotally = (MYBOOL) (rowmap == NULL);
  MATrec  *mat = lp->matA;
  REAL    hold;
  REAL    *value;
  int     *rownr;

  if(dotally)
    data[0] = 0;

  Tnz = nrows - ncols;
  for(j = 1; j <= ncols; j++) {
    kk = colorder[j];

    /* Process slacks */
    if(kk <= lp->rows) {
      if(includeMDO(usedpos, kk)) {
        if(!dotally)
          data[Bnz] = rowmap[kk]+offset;
        Bnz++;
      }
      Tnz++;
    }
    /* Process user columns */
    else {
      k = kk - lp->rows;
      i = mat->col_end[k-1];
      ii= mat->col_end[k];
      Tnz += ii-i;
#ifdef Paranoia
      if(i >= ii)
        lp->report(lp, SEVERE, "prepareMDO: Encountered empty basic column %d\n", k);
#endif

      /* Detect if we need to do phase 1 adjustments of zero-valued OF variable */
      rownr = &COL_MAT_ROWNR(i);
      value = &COL_MAT_VALUE(i);
      hold = 0;
      if((*rownr > 0) && includeMDO(usedpos, 0) && modifyOF1(lp, kk, &hold, 1.0)) {
        if(!dotally)
          data[Bnz] = offset;
        Bnz++;
      }
      /* Loop over all NZ-variables */
      for(; i < ii; 
          i++, value += matValueStep, rownr += matRowColStep) {
        if(!includeMDO(usedpos, *rownr))
          continue;
        /* See if we need to change phase 1 OF value */
        if(*rownr == 0) {
          hold = *value;
          if(!modifyOF1(lp, kk, &hold, 1.0)) 
            continue;
        }
        /* Tally uneliminated constraint row values */
        if(!dotally)
          data[Bnz] = rowmap[*rownr]+offset;
        Bnz++;
      }
    }
    if(dotally)
      data[j] = Bnz;
  }
  return( Tnz );
}

STATIC MYBOOL verifyMDO(lprec *lp, int *col_end, int *row_nr, int rowmax, int colmax)
{
  int i, j, n, err = 0;

  for(i = 1; i <= colmax; i++) {
    n = 0;
    for(j = col_end[i-1]; (j < col_end[i]) && (err == 0); j++, n++) {
      if(row_nr[j] < 0 || row_nr[j] > rowmax)
        err = 1;
      if(n > 0 && row_nr[j] <= row_nr[j-1])
        err = 2;
      n++;
    }
  }
  if(err != 0)
    lp->report(lp, SEVERE, "verifyMDO: Invalid MDO input structure generated (error %d)\n", err);
  return( (MYBOOL) (err == 0) );
}

void *mdo_calloc(size_t size, size_t count)
{
  return ( calloc(size, count) );
}
void mdo_free(void *mem)
{
  free( mem );
}


int __WINAPI getMDO(lprec *lp, MYBOOL *usedpos, int *colorder, int *size, MYBOOL symmetric)
{
  int    error = FALSE;
  int    nrows = lp->rows+1, ncols = colorder[0];
  int    i, j, kk, n;
  int    *col_end, *row_map = NULL;
  int    Bnz, Blen, *Brows = NULL;
  int    stats[COLAMD_STATS];
  double knobs[COLAMD_KNOBS];

 /* Tally the non-zero counts of the unused columns/rows of the 
    basis matrix and store corresponding "net" starting positions */
  allocINT(lp, &col_end, ncols+1, FALSE);
  n = prepareMDO(lp, usedpos, colorder, col_end, NULL);
  Bnz = col_end[ncols];

 /* Check that we have unused basic columns, otherwise skip analysis */  
  if(ncols == 0 || Bnz == 0) 
    goto Transfer;

 /* Get net number of rows and fill mapper */
  allocINT(lp, &row_map, nrows, FALSE);
  nrows = 0;
  for(i = 0; i <= lp->rows; i++) {
    row_map[i] = i-nrows;
   /* Increment eliminated row counter if necessary */
    if(!includeMDO(usedpos, i)) 
      nrows++;
  }
  nrows = lp->rows+1 - nrows;

 /* Store row indeces of non-zero values in the basic columns */
  Blen = colamd_recommended(Bnz, nrows, ncols);
  allocINT(lp, &Brows, Blen, FALSE);
  prepareMDO(lp, usedpos, colorder, Brows, row_map);
#ifdef Paranoia
  verifyMDO(lp, col_end, Brows, nrows, ncols);
#endif

 /* Compute the MDO */
#if 1
  colamd_set_defaults(knobs);
  knobs [COLAMD_DENSE_ROW] = 0.2+0.2 ;    /* default changed for UMFPACK */
  knobs [COLAMD_DENSE_COL] = knobs [COLAMD_DENSE_ROW];    
  if(symmetric && (nrows == ncols)) {
    MEMCOPY(colorder, Brows, ncols + 1);
    error = !symamd(nrows, colorder, col_end, Brows, knobs, stats, mdo_calloc, mdo_free);
  }
  else
    error = !colamd(nrows, ncols, Blen, Brows, col_end, knobs, stats);
#else
  if(symmetric && (nrows == ncols)) {
    MEMCOPY(colorder, Brows, ncols + 1);
    error = !symamd(nrows, colorder, col_end, Brows, knobs, stats, mdo_calloc, mdo_free);
  }
  else
    error = !colamd(nrows, ncols, Blen, Brows, col_end, (double *) NULL, stats);
#endif

 /* Transfer the estimated optimal ordering, adjusting for index offsets */
Transfer:
  if(error) 
    error = stats[COLAMD_STATUS];
  else {
    MEMCOPY(Brows, colorder, ncols + 1);
    for(j = 0; j < ncols; j++) {
      kk = col_end[j];
      n = Brows[kk+1];
      colorder[j+1] = n;
    }
  }

  /* Free temporary vectors */
  FREE(col_end);
  if(row_map != NULL)
    FREE(row_map);
  if(Brows != NULL)
    FREE(Brows);

  if(size != NULL)
    *size = ncols;
  return( error );
}


