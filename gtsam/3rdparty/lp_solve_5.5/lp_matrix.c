
#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_scale.h"
#include "lp_report.h"
#include "lp_price.h"
#include "lp_pricePSE.h"
#include "lp_matrix.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


/* -------------------------------------------------------------------------
   Basic matrix routines in lp_solve v5.0+
   -------------------------------------------------------------------------
    Author:        Michel Berkelaar (to lp_solve v3.2),
                   Kjell Eikland    (v4.0 and forward)
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h, lp_pricerPSE.h, lp_matrix.h

    Release notes:
    v5.0.0  1 January 2004      First integrated and repackaged version.
    v5.0.1  7 May 2004          Added matrix transpose function.
    v5.1.0  20 July 2004        Reworked with flexible matrix storage model.
    v5.2.0  10 January 2005     Added fast deletion methods.
                                Added data extraction to matrix method.
                                Changed to explicit OF storage mode.

   ------------------------------------------------------------------------- */

STATIC MATrec *mat_create(lprec *lp, int rows, int columns, REAL epsvalue)
{
  MATrec *newmat;

  newmat = (MATrec *) calloc(1, sizeof(*newmat));
  newmat->lp = lp;

  newmat->rows_alloc = 0;
  newmat->columns_alloc = 0;
  newmat->mat_alloc = 0;

  inc_matrow_space(newmat, rows);
  newmat->rows = rows;
  inc_matcol_space(newmat, columns);
  newmat->columns = columns;
  inc_mat_space(newmat, 0);

  newmat->epsvalue = epsvalue;

  return( newmat );
}

STATIC void mat_free(MATrec **matrix)
{
  if((matrix == NULL) || (*matrix == NULL))
    return;

#if MatrixColAccess==CAM_Record
  FREE((*matrix)->col_mat);
#else /*if MatrixColAccess==CAM_Vector*/
  FREE((*matrix)->col_mat_colnr);
  FREE((*matrix)->col_mat_rownr);
  FREE((*matrix)->col_mat_value);
#endif
  FREE((*matrix)->col_end);
  FREE((*matrix)->col_tag);

#if MatrixRowAccess==RAM_Index
  FREE((*matrix)->row_mat);
#elif MatrixColAccess==CAM_Record
  FREE((*matrix)->row_mat);
#else /*if MatrixRowAccess==COL_Vector*/
  FREE((*matrix)->row_mat_colnr);
  FREE((*matrix)->row_mat_rownr);
  FREE((*matrix)->row_mat_value);
#endif
  FREE((*matrix)->row_end);
  FREE((*matrix)->row_tag);

  FREE((*matrix)->colmax);
  FREE((*matrix)->rowmax);

  FREE(*matrix);
}

STATIC MYBOOL mat_memopt(MATrec *mat, int rowextra, int colextra, int nzextra)
{
  MYBOOL status = TRUE;
  int matalloc, colalloc, rowalloc;

  if((mat == NULL) ||
#if 0
     (++rowextra < 1) || (++colextra < 1) || (++nzextra < 1))
#else
     (rowextra < 0) || (colextra < 0) || (nzextra < 0))
#endif
    return( FALSE );

  mat->rows_alloc    = MIN(mat->rows_alloc,    mat->rows + rowextra);
  mat->columns_alloc = MIN(mat->columns_alloc, mat->columns + colextra);
  mat->mat_alloc     = MIN(mat->mat_alloc,     mat->col_end[mat->columns] + nzextra);
#if 0
  rowalloc = mat->rows_alloc;
  colalloc = mat->columns_alloc;
  matalloc = mat->mat_alloc;
#else
  rowalloc = mat->rows_alloc + 1;
  colalloc = mat->columns_alloc + 1;
  matalloc = mat->mat_alloc + 1;
#endif

#if MatrixColAccess==CAM_Record
  mat->col_mat = (MATitem *) realloc(mat->col_mat, matalloc * sizeof(*(mat->col_mat)));
  status &= (mat->col_mat != NULL);
#else /*if MatrixColAccess==CAM_Vector*/
  status &= allocINT(mat->lp,  &(mat->col_mat_colnr), matalloc, AUTOMATIC) &&
            allocINT(mat->lp,  &(mat->col_mat_rownr), matalloc, AUTOMATIC) &&
            allocREAL(mat->lp, &(mat->col_mat_value), matalloc, AUTOMATIC);
#endif
  status &= allocINT(mat->lp, &mat->col_end, colalloc, AUTOMATIC);
  if(mat->col_tag != NULL)
    status &= allocINT(mat->lp, &mat->col_tag, colalloc, AUTOMATIC);

#if MatrixRowAccess==RAM_Index
  status &= allocINT(mat->lp, &(mat->row_mat), matalloc, AUTOMATIC);
#elif MatrixColAccess==CAM_Record
  mat->row_mat = (MATitem *) realloc(mat->row_mat, matalloc * sizeof(*(mat->row_mat)));
  status &= (mat->row_mat != NULL);
#else /*if MatrixRowAccess==COL_Vector*/
  status &= allocINT(mat->lp,  &(mat->row_mat_colnr), matalloc, AUTOMATIC) &&
            allocINT(mat->lp,  &(mat->row_mat_rownr), matalloc, AUTOMATIC) &&
            allocREAL(mat->lp, &(mat->row_mat_value), matalloc, AUTOMATIC);
#endif
  status &= allocINT(mat->lp, &mat->row_end, rowalloc, AUTOMATIC);
  if(mat->row_tag != NULL)
    status &= allocINT(mat->lp, &mat->row_tag, rowalloc, AUTOMATIC);

  if(mat->colmax != NULL)
    status &= allocREAL(mat->lp, &(mat->colmax), colalloc, AUTOMATIC);
  if(mat->rowmax != NULL)
    status &= allocREAL(mat->lp, &(mat->rowmax), rowalloc, AUTOMATIC);

  return( status );
}

STATIC MYBOOL inc_mat_space(MATrec *mat, int mindelta)
{
  int spaceneeded, nz = mat_nonzeros(mat);

  if(mindelta <= 0)
    mindelta = MAX(mat->rows, mat->columns) + 1;
  spaceneeded = DELTA_SIZE(mindelta, nz);
  SETMAX(mindelta, spaceneeded);

  if(mat->mat_alloc == 0)
    spaceneeded = mindelta;
  else
    spaceneeded = nz + mindelta;

  if(spaceneeded >= mat->mat_alloc) {
    /* Let's allocate at least MAT_START_SIZE entries */
    if(mat->mat_alloc < MAT_START_SIZE)
      mat->mat_alloc = MAT_START_SIZE;

    /* Increase the size by RESIZEFACTOR each time it becomes too small */
    while(spaceneeded >= mat->mat_alloc)
      mat->mat_alloc += mat->mat_alloc / RESIZEFACTOR;

#if MatrixColAccess==CAM_Record
    mat->col_mat = (MATitem *) realloc(mat->col_mat, (mat->mat_alloc) * sizeof(*(mat->col_mat)));
#else /*if MatrixColAccess==CAM_Vector*/
    allocINT(mat->lp,  &(mat->col_mat_colnr), mat->mat_alloc, AUTOMATIC);
    allocINT(mat->lp,  &(mat->col_mat_rownr), mat->mat_alloc, AUTOMATIC);
    allocREAL(mat->lp, &(mat->col_mat_value), mat->mat_alloc, AUTOMATIC);
#endif

#if MatrixRowAccess==RAM_Index
    allocINT(mat->lp, &(mat->row_mat), mat->mat_alloc, AUTOMATIC);
#elif MatrixColAccess==CAM_Record
    mat->row_mat = (MATitem *) realloc(mat->row_mat, (mat->mat_alloc) * sizeof(*(mat->row_mat)));
#else /*if MatrixColAccess==CAM_Vector*/
    allocINT(mat->lp,  &(mat->row_mat_colnr), mat->mat_alloc, AUTOMATIC);
    allocINT(mat->lp,  &(mat->row_mat_rownr), mat->mat_alloc, AUTOMATIC);
    allocREAL(mat->lp, &(mat->row_mat_value), mat->mat_alloc, AUTOMATIC);
#endif
  }
  return(TRUE);
}

STATIC MYBOOL inc_matrow_space(MATrec *mat, int deltarows)
{
  int    rowsum, oldrowsalloc;
  MYBOOL status = TRUE;

  /* Adjust lp row structures */
  if(mat->rows+deltarows >= mat->rows_alloc) {

    /* Update memory allocation and sizes */
    oldrowsalloc = mat->rows_alloc;
    deltarows = DELTA_SIZE(deltarows, mat->rows);
    SETMAX(deltarows, DELTAROWALLOC);
    mat->rows_alloc += deltarows;
    rowsum = mat->rows_alloc + 1;

    /* Update row pointers */
    status = allocINT(mat->lp, &mat->row_end, rowsum, AUTOMATIC);
    mat->row_end_valid = FALSE;
  }
  return( status );
}

STATIC MYBOOL inc_matcol_space(MATrec *mat, int deltacols)
{
  int    i, colsum, oldcolsalloc;
  MYBOOL status = TRUE;

  /* Adjust lp column structures */
  if(mat->columns+deltacols >= mat->columns_alloc) {

    /* Update memory allocation and sizes */
    oldcolsalloc = mat->columns_alloc;
    deltacols = DELTA_SIZE(deltacols, mat->columns);
    SETMAX(deltacols, DELTACOLALLOC);
    mat->columns_alloc += deltacols;
    colsum = mat->columns_alloc + 1;
    status = allocINT(mat->lp, &mat->col_end, colsum, AUTOMATIC);

    /* Update column pointers */
    if(oldcolsalloc == 0)
      mat->col_end[0] = 0;
    for(i = MIN(oldcolsalloc, mat->columns) + 1; i < colsum; i++)
      mat->col_end[i] = mat->col_end[i-1];
    mat->row_end_valid = FALSE;
  }
  return( status );
}

STATIC int mat_collength(MATrec *mat, int colnr)
{
  return( mat->col_end[colnr] - mat->col_end[colnr-1] );
}

STATIC int mat_rowlength(MATrec *mat, int rownr)
{
  if(mat_validate(mat)) {
    if(rownr <= 0)
      return( mat->row_end[0] );
    else
      return( mat->row_end[rownr] - mat->row_end[rownr-1] );
  }
  else
    return( 0 );
}

STATIC int mat_nonzeros(MATrec *mat)
{
  return( mat->col_end[mat->columns] );
}

STATIC MYBOOL mat_indexrange(MATrec *mat, int index, MYBOOL isrow, int *startpos, int *endpos)
{
#ifdef Paranoia
  if(isrow && ((index < 0) || (index > mat->rows)))
    return( FALSE );
  else if(!isrow && ((index < 1) || (index > mat->columns)))
    return( FALSE );
#endif

  if(isrow && mat_validate(mat)) {
    if(index == 0)
      *startpos = 0;
    else
      *startpos = mat->row_end[index-1];
    *endpos = mat->row_end[index];
  }
  else {
    *startpos = mat->col_end[index-1];
    *endpos = mat->col_end[index];
  }
  return( TRUE );
}

STATIC int mat_shiftrows(MATrec *mat, int *bbase, int delta, LLrec *varmap)
{
  int     j, k, i, ii, thisrow, *colend, base;
  MYBOOL  preparecompact = FALSE;
  int     *rownr;

  if(delta == 0)
    return( 0 );
  base = abs(*bbase);

  if(delta > 0) {

    /* Insert row by simply incrementing existing row indeces */
    if(base <= mat->rows) {
      k = mat_nonzeros(mat);
      rownr = &COL_MAT_ROWNR(0);
      for(ii = 0; ii < k; ii++, rownr += matRowColStep) {
        if(*rownr >= base)
          *rownr += delta;
      }
    }

    /* Set defaults (actual basis set in separate procedure) */
    for(i = 0; i < delta; i++) {
      ii = base + i;
      mat->row_end[ii] = 0;
    }
  }
  else if(base <= mat->rows) {

    /* Check for preparation of mass-deletion of rows */
    preparecompact = (MYBOOL) (varmap != NULL);
    if(preparecompact) {
      /* Create the offset array */
      int *newrowidx = NULL;
      allocINT(mat->lp, &newrowidx, mat->rows+1, FALSE);
      newrowidx[0] = 0;
      delta = 0;
      for(j = 1; j <= mat->rows; j++) {
        if(isActiveLink(varmap, j)) {
          delta++;
          newrowidx[j] = delta;
        }
        else
          newrowidx[j] = -1;
      }
      k = 0;
      delta = 0;
      base = mat_nonzeros(mat);
      rownr = &COL_MAT_ROWNR(0);
      for(i = 0; i < base; i++, rownr += matRowColStep) {
        thisrow = newrowidx[*rownr];
        if(thisrow < 0) {
          *rownr = -1;
          delta++;
        }
        else
          *rownr = thisrow;
      }
      FREE(newrowidx);
      return(delta);
    }

    /* Check if we should prepare for compacting later
       (this is in order to speed up multiple row deletions) */
    preparecompact = (MYBOOL) (*bbase < 0);
    if(preparecompact)
      *bbase = my_flipsign((*bbase));

    /* First make sure we don't cross the row count border */
    if(base-delta-1 > mat->rows)
      delta = base - mat->rows - 1;

    /* Then scan over all entries shifting and updating rows indeces */
    if(preparecompact) {
      k = 0;
      for(j = 1, colend = mat->col_end + 1;
          j <= mat->columns; j++, colend++) {
        i = k;
        k = *colend;
        rownr = &COL_MAT_ROWNR(i);
        for(; i < k; i++, rownr += matRowColStep) {
          thisrow = *rownr;
          if(thisrow < base)
            continue;
          else if(thisrow >= base-delta)
            *rownr += delta;
          else
            *rownr = -1;
        }
      }
    }
    else {
      k = 0;
      ii = 0;
      for(j = 1, colend = mat->col_end + 1;
          j <= mat->columns; j++, colend++) {
        i = k;
        k = *colend;
        rownr = &COL_MAT_ROWNR(i);
        for(; i < k; i++, rownr += matRowColStep) {
          thisrow = *rownr;
          if(thisrow >= base) {
            if(thisrow >= base-delta)
              *rownr += delta;
            else
              continue;
          }
          if(ii != i) {
            COL_MAT_COPY(ii, i);
          }
          ii++;
        }
        *colend = ii;
      }
    }
  }
  return( 0 );
}

/* Map-based compacting+insertion of matrix elements without changing row and column indeces.
   When mat2 is NULL, a simple compacting of non-deleted rows and columns is done. */
STATIC int mat_mapreplace(MATrec *mat, LLrec *rowmap, LLrec *colmap, MATrec *mat2)
{
  lprec *lp = mat->lp;
  int   i, ib, ie, ii, j, jj, jb, je, nz, *colend, *rownr, *rownr2, *indirect = NULL;
  REAL  *value, *value2;

  /* Check if there is something to insert */
  if((mat2 != NULL) && ((mat2->col_tag == NULL) || (mat2->col_tag[0] <= 0) || (mat_nonzeros(mat2) == 0)))
    return( 0 );

  /* Create map and sort by increasing index in "mat" */
  if(mat2 != NULL) {
    jj = mat2->col_tag[0];
    allocINT(lp, &indirect, jj+1, FALSE);
    indirect[0] = jj;
    for(i = 1; i <= jj; i++)
      indirect[i] = i;
    hpsortex(mat2->col_tag, jj, 1, sizeof(*indirect), FALSE, compareINT, indirect);
  }

  /* Do the compacting */
  mat->row_end_valid = FALSE;
  nz = mat->col_end[mat->columns];
  ie = 0;
  ii = 0;
  if((mat2 == NULL) || (indirect[0] == 0)) {
    je = mat->columns + 1;
    jj = 1;
    jb = 0;
  }
  else {
    je = indirect[0];
    jj = 0;
    do {
      jj++;
      jb = mat2->col_tag[jj];
    } while(jb <= 0);

  }
  for(j = 1, colend = mat->col_end + 1;
      j <= mat->columns; j++, colend++) {
    ib = ie;
    ie = *colend;

    /* Always skip (condense) replacement columns */
    if(j == jb) {
      jj++;
      if(jj <= je)
        jb = mat2->col_tag[jj];
      else
        jb = mat->columns + 1;
    }

    /* Only include active columns */
    else if(isActiveLink(colmap, j)) {
      rownr = &COL_MAT_ROWNR(ib);
      for(; ib < ie; ib++, rownr += matRowColStep) {

        /* Also make sure the row is active */
        if(isActiveLink(rowmap, *rownr)) {
          if(ii != ib) {
            COL_MAT_COPY(ii, ib);
          }
          ii++;
        }
      }
    }
    *colend = ii;
  }
  if(mat2 == NULL)
    goto Finish;

  /* Tally non-zero insertions */
  i = 0;
  for(j = 1; j <= mat2->col_tag[0]; j++) {
    jj = mat2->col_tag[j];
    if((jj > 0) && isActiveLink(colmap, jj)) {
      jj = indirect[j];
      je = mat2->col_end[jj];
      jb = mat2->col_end[jj-1];
      rownr2 = &COL_MAT2_ROWNR(jb);
      for(; jb < je; jb++, rownr2 += matRowColStep) {
        if((*rownr2 > 0) && isActiveLink(rowmap, *rownr2))
          i++;
      }
    }
  }

  /* Make sure we have enough matrix space */
  ii = mat->col_end[mat->columns] + i;
  if(mat->mat_alloc <= ii)
    inc_mat_space(mat, i);

  /* Do shifting and insertion - loop from the end going forward */
  jj = indirect[0];
  jj = mat2->col_tag[jj];
  for(j = mat->columns, colend = mat->col_end + mat->columns, ib = *colend;
      j > 0; j--) {

    /* Update indeces for this loop */
    ie = ib;
    *colend = ii;
    colend--;
    ib = *colend;

    /* Insert new values */
    if(j == jj) {
      /* Only include an active column */
      if(isActiveLink(colmap, j)) {
        jj = indirect[0];
        jj = indirect[jj];
        rownr = &COL_MAT_ROWNR(ii-1);
        value = &COL_MAT_VALUE(ii-1);
        jb = mat2->col_end[jj-1];
        je = mat2->col_end[jj] - 1;
        rownr2 = &COL_MAT2_ROWNR(je);
        value2 = &COL_MAT2_VALUE(je);

        /* Process constraint coefficients */
        for(; je >= jb; je--, rownr2 -= matRowColStep, value2 -= matValueStep) {
          i = *rownr2;
          if(i == 0) {
            i = -1;
            break;
          }
          else if(isActiveLink(rowmap, i)) {
            ii--;
            *rownr = i;
            rownr -= matRowColStep;
            *value = my_chsign(is_chsign(lp, i), *value2);
            value -= matValueStep;
          }
        }

        /* Then handle the objective */
        if(i == -1) {
          lp->orig_obj[j] = my_chsign(is_maxim(lp), *value2);
          rownr2 -= matRowColStep;
          value2 -= matValueStep;
        }
        else
          lp->orig_obj[j] = 0;

      }
      /* Update replacement column index or break if no more candidates */
      jj = --indirect[0];
      if(jj == 0)
        break;
      jj = mat2->col_tag[jj];
      if(jj <= 0)
        break;
    }
    /* Shift existing values down */
    else {
      if(isActiveLink(colmap, j))
      while(ie > ib) {
        ii--;
        ie--;
        if(ie != ii) {
          COL_MAT_COPY(ii, ie);
        }
      }
    }
  }

  /* Return the delta number of non-zero elements */
Finish:
  nz -= mat->col_end[mat->columns];
  FREE(indirect);

  return( nz );
}

/* Routines to compact rows in matrix based on precoded entries */
STATIC int mat_zerocompact(MATrec *mat)
{
  return( mat_rowcompact(mat, TRUE) );
}
STATIC int mat_rowcompact(MATrec *mat, MYBOOL dozeros)
{
  int  i, ie, ii, j, nn, *colend, *rownr;
  REAL *value;

  nn = 0;
  ie = 0;
  ii = 0;
  for(j = 1, colend = mat->col_end + 1;
      j <= mat->columns; j++, colend++) {
    i = ie;
    ie = *colend;
    rownr = &COL_MAT_ROWNR(i);
    value = &COL_MAT_VALUE(i);
    for(; i < ie;
        i++, rownr += matRowColStep, value += matValueStep) {
      if((*rownr < 0) || (dozeros && (fabs(*value) < mat->epsvalue))) {
        nn++;
        continue;
      }
      if(ii != i) {
        COL_MAT_COPY(ii, i);
      }
      ii++;
    }
    *colend = ii;
  }
  return( nn );
}

/* Routines to compact columns and their indeces based on precoded entries */
STATIC int mat_colcompact(MATrec *mat, int prev_rows, int prev_cols)
{
  int             i, ii, j, k, n_del, n_sum, *colend, *newcolend, *colnr, newcolnr;
  MYBOOL          deleted;
  lprec           *lp = mat->lp;
  presolveundorec *lpundo = lp->presolve_undo;


  n_sum = 0;
  k  = 0;
  ii = 0;
  newcolnr = 1;
  for(j = 1, colend = newcolend = mat->col_end + 1;
      j <= prev_cols; j++, colend++) {
    n_del = 0;
    i = k;
    k = *colend;
    for(colnr = &COL_MAT_COLNR(i); i < k;
        i++, colnr += matRowColStep) {
      if(*colnr < 0) {
        n_del++;
        n_sum++;
        continue;
      }
      if(ii < i) {
        COL_MAT_COPY(ii, i);
      }
      if(newcolnr < j) {
        COL_MAT_COLNR(ii) = newcolnr;
      }
      ii++;
    }
    *newcolend = ii;

    deleted = (MYBOOL) (n_del > 0);
#if 1
    /* Do hoops in case there was an empty column */
    deleted |= (MYBOOL) (!lp->wasPresolved && (lpundo->var_to_orig[prev_rows+j] < 0));

#endif
    /* Increment column variables if current column was not deleted */
    if(!deleted) {
      newcolend++;
      newcolnr++;
    }
  }
  return(n_sum);
}

STATIC int mat_shiftcols(MATrec *mat, int *bbase, int delta, LLrec *varmap)
{
  int     i, ii, k, n, base;


  k = 0;
  if(delta == 0)
    return( k );
  base = abs(*bbase);

  if(delta > 0) {
    /* Shift pointers right */
    for(ii = mat->columns; ii > base; ii--) {
      i = ii + delta;
      mat->col_end[i] = mat->col_end[ii];
    }
    /* Set defaults */
    for(i = 0; i < delta; i++) {
      ii = base + i;
      mat->col_end[ii] = mat->col_end[ii-1];
    }
  }
  else {

    /* Check for preparation of mass-deletion of columns */
    MYBOOL preparecompact = (MYBOOL) (varmap != NULL);
    if(preparecompact) {
      /* Create the offset array */
      int j, *colnr, *colend;
      n = 0;
      k = 0;
      base = 0;
      for(j = 1, colend = mat->col_end + 1;
          j <= mat->columns; j++, colend++) {
        i = k;
        k = *colend;
        if(isActiveLink(varmap, j)) {
          base++;
          ii = base;
        }
        else
          ii = -1;
        if(ii < 0)
          n += k - i;
        colnr = &COL_MAT_COLNR(i);
        for(; i < k; i++, colnr += matRowColStep)
          *colnr = ii;
      }
      return(n);
    }

    /* Check if we should prepare for compacting later
       (this is in order to speed up multiple column deletions) */
    preparecompact = (MYBOOL) (*bbase < 0);
    if(preparecompact)
      *bbase = my_flipsign((*bbase));

    /* First make sure we don't cross the column count border */
    if(base-delta-1 > mat->columns)
      delta = base - mat->columns - 1;

    /* Then scan over all entries shifting and updating column indeces */
    if(preparecompact) {
      int *colnr;
      n = 0;
      i = mat->col_end[base-1];
      k = mat->col_end[base-delta-1];
      for(colnr = &COL_MAT_COLNR(i); i < k;
          i++, colnr += matRowColStep) {
        n++;
        *colnr = -1;
      }
      k = n;
    }
    else {
      /* Delete sparse matrix data, if required */
      if(base <= mat->columns) {

        i = mat->col_end[base-1];          /* Beginning of data to be deleted */
        ii = mat->col_end[base-delta-1];   /* Beginning of data to be shifted left */
        n = mat_nonzeros(mat);             /* Total number of non-zeros */
        k = ii-i;                          /* Number of entries to be deleted */
        if((k > 0) && (n > i)) {
          n -= ii;
          COL_MAT_MOVE(i, ii, n);
        }

        /* Update indexes */
        for(i = base; i <= mat->columns + delta; i++) {
          ii = i - delta;
          mat->col_end[i] = mat->col_end[ii] - k;
        }
      }
    }
  }
  return( k );
}

STATIC MATrec *mat_extractmat(MATrec *mat, LLrec *rowmap, LLrec *colmap, MYBOOL negated)
{
  int    *rownr, *colnr, xa, na;
  REAL   *value;
  MATrec *newmat = mat_create(mat->lp, mat->rows, mat->columns, mat->epsvalue);

  /* Initialize */
  na = mat_nonzeros(mat);
  rownr = &COL_MAT_ROWNR(0);
  colnr = &COL_MAT_COLNR(0);
  value = &COL_MAT_VALUE(0);

  /* Loop over the indeces, picking out values in qualifying rows and colums
     (note that the loop could be speeded up for dense matrices by making an
      outer loop for columns and inner loop for rows) */
  for(xa = 0; xa < na; xa++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep) {
    if((isActiveLink(colmap, *colnr) ^ negated) &&
       (isActiveLink(rowmap, *rownr) ^ negated))
      mat_setvalue(newmat, *rownr, *colnr, *value, FALSE);
  }

  /* Return the populated new matrix */
  return( newmat );
}

STATIC MYBOOL mat_setcol(MATrec *mat, int colno, int count, REAL *column, int *rowno, MYBOOL doscale, MYBOOL checkrowmode)
{
  int    i, jj = 0, elmnr, orignr, newnr, firstrow;
  MYBOOL *addto = NULL, isA, isNZ;
  REAL   value, saved = 0;
  lprec  *lp = mat->lp;

  /* Check if we are in row order mode and should add as row instead;
     the matrix will be transposed at a later stage */
  if(checkrowmode && mat->is_roworder)
    return( mat_setrow(mat, colno, count, column, rowno, doscale, FALSE) );

  /* Initialize and validate */
  isA = (MYBOOL) (mat == mat->lp->matA);
  isNZ = (MYBOOL) (rowno != NULL);
  if(!isNZ)
    count = mat->lp->rows;
  else if((count < 0) || (count > mat->rows+((mat->is_roworder) ? 0 : 1)))
    return( FALSE );
  if(isNZ && (count > 0)) {
    if(count > 1)
      sortREALByINT(column, rowno, count, 0, TRUE);
    if((rowno[0] < 0) || (rowno[count-1] > mat->rows))
      return( FALSE );
  }

  /* Capture OF definition in column mode */
  if(isA && !mat->is_roworder) {
    if(isNZ && (rowno[0] == 0)) {
      value = column[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(doscale)
        value = scaled_mat(lp, value, 0, colno);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[colno] = value;
      count--;
      column++;
      rowno++;
    }
    else if(!isNZ && (column[0] != 0)) {
      value = saved = column[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(doscale)
        value = scaled_mat(lp, value, 0, colno);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[colno] = value;
      column[0] = 0;
    }
    else
      lp->orig_obj[colno] = 0;
  }

  /* Optionally tally and map the new non-zero values */
  firstrow = mat->rows + 1;
  if(isNZ) {
    newnr = count;
    if(newnr) {
      firstrow = rowno[0];
      jj = rowno[newnr - 1];
    }
  }
  else {
    newnr = 0;
    if(!allocMYBOOL(lp, &addto, mat->rows + 1, TRUE)) {
      return( FALSE );
    }
    for(i = mat->rows; i >= 0; i--) {
      if(fabs(column[i]) > mat->epsvalue) {
        addto[i] = TRUE;
        firstrow = i;
        newnr++;
      }
    }
  }

  /* Make sure we have enough matrix space */
  if(!inc_mat_space(mat, newnr)) {
    newnr = 0;
    goto Done;
  }

  /* Shift existing column data and adjust position indeces */
  orignr = mat_collength(mat, colno);
  elmnr = newnr - orignr;
  i = mat_nonzeros(mat) - mat->col_end[colno];
  if((elmnr != 0) && (i > 0)) {
    COL_MAT_MOVE(mat->col_end[colno] + elmnr, mat->col_end[colno], i);
  }
  if(elmnr != 0)
    for(i = colno; i <= mat->columns; i++)
      mat->col_end[i] += elmnr;

  /* We are now ready to copy the new data */
  jj = mat->col_end[colno-1];
  if(isNZ) {
    for(i = 0; i < count; jj++, i++) {
      value = column[i];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(mat->is_roworder) {    /* Fix following Ingmar Stein bug report 12.10.2006 */
        if(isA && doscale)
          value = scaled_mat(lp, value, colno, rowno[i]);
        if(isA)
          value = my_chsign(is_chsign(lp, colno), value);
      }
      else {
        if(isA && doscale)
          value = scaled_mat(lp, value, rowno[i], colno);
        if(isA)
          value = my_chsign(is_chsign(lp, rowno[i]), value);
      }
      SET_MAT_ijA(jj, rowno[i], colno, value);
    }
  }
  else {
    for(i = firstrow; i <= mat->rows; i++) {
      if(!addto[i])
        continue;
      value = column[i];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(mat->is_roworder) {    /* Fix following Ingmar Stein bug report 12.10.2006 */
        if(isA && doscale)
          value = scaled_mat(lp, value, colno, i);
        if(isA)
          value = my_chsign(is_chsign(lp, colno), value);
      }
      else {
        if(isA && doscale)
          value = scaled_mat(lp, value, i, colno);
        if(isA)
          value = my_chsign(is_chsign(lp, i), value);
      }
      SET_MAT_ijA(jj, i, colno, value);
      jj++;
    }
  }
  mat->row_end_valid = FALSE;

  /* Finish and return */
Done:
  if(saved != 0)
    column[0] = saved;
  FREE(addto);
  return( TRUE );

}

STATIC MYBOOL mat_mergemat(MATrec *target, MATrec *source, MYBOOL usecolmap)
{
  lprec *lp = target->lp;
  int   i, ix, iy, n, *colmap = NULL;
  REAL  *colvalue = NULL;

  if((target->rows < source->rows) || !allocREAL(lp, &colvalue, target->rows+1, FALSE))
    return( FALSE );

  if(usecolmap) {
    n = source->col_tag[0];
    allocINT(lp, &colmap, n+1, FALSE);
    for(i = 1; i <= n; i++)
      colmap[i] = i;
    hpsortex(source->col_tag, n, 1, sizeof(*colmap), FALSE, compareINT, colmap);
  }
  else
    n = source->columns;
  for(i = 1; i <= n; i++) {
    if(!usecolmap && (mat_collength(source, i) == 0))
      continue;
    if(usecolmap) {
      ix = colmap[i];
      if(ix <= 0)
        continue;
      iy = source->col_tag[i];
      if(iy <= 0)
        continue;
    }
    else
      ix = iy = i;
    mat_expandcolumn(source, ix, colvalue, NULL, FALSE);
    mat_setcol(target, iy, 0, colvalue, NULL, FALSE, FALSE);
  }

  FREE( colvalue );
  FREE( colmap );

  return( TRUE );
}

STATIC int mat_nz_unused(MATrec *mat)
{
  return( mat->mat_alloc - mat->col_end[mat->columns] );
}

STATIC MYBOOL mat_setrow(MATrec *mat, int rowno, int count, REAL *row, int *colno, MYBOOL doscale, MYBOOL checkrowmode)
{
  int    k, kk, i, ii, j, jj = 0, jj_j, elmnr, orignr, newnr, firstcol, rownr, colnr;
  MYBOOL *addto = NULL, isA, isNZ;
  REAL   value, saved = 0;
  lprec  *lp = mat->lp;

  /* Check if we are in row order mode and should add as column instead;
     the matrix will be transposed at a later stage */
  if(checkrowmode && mat->is_roworder)
    return( mat_setcol(mat, rowno, count, row, colno, doscale, FALSE) );

  /* Do initialization and validation */
  if(!mat_validate(mat))
    return( FALSE );
  isA = (MYBOOL) (mat == lp->matA);
  isNZ = (MYBOOL) (colno != NULL);
  if(!isNZ)
    count = mat->columns;
  else if((count < 0) || (count > mat->columns))
    return( FALSE );
  if(isNZ && (count > 0)) {
    if(count > 1)
      sortREALByINT(row, colno, count, 0, TRUE);
    if((colno[0] < 1) || (colno[count-1] > mat->columns))
      return( FALSE );
  }

  /* Capture OF definition in row mode */
  if(isA && mat->is_roworder) {
    lp->orig_obj[rowno] = 0;
    if(isNZ && (colno[0] == 0)) {
      value = row[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(doscale)
        value = scaled_mat(lp, value, 0, rowno);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[rowno] = value;
      count--;
      row++;
      colno++;
    }
    else if(!isNZ && (row[0] != 0)) {
      value = saved = row[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(doscale)
        value = scaled_mat(lp, value, 0, rowno);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[rowno] = value;
      row[0] = 0;
    }
    else
      lp->orig_obj[rowno] = 0;
  }

#if !defined oldmat_setrow
  /* Optionally tally and map the new non-zero values */
  firstcol = mat->columns + 1;
  if(isNZ) {
    newnr = count;
    if(newnr)
      firstcol = colno[0];
  }
  else {
    newnr = 0;
    if(!allocMYBOOL(lp, &addto, mat->columns + 1, TRUE)) {
      return( FALSE );
    }
    for(i = mat->columns; i >= 1; i--) {
      if(fabs(row[i]) > mat->epsvalue) {
        addto[i] = TRUE;
        firstcol = i;
        newnr++;
      }
    }
  }
#else
  /* Optionally tally and map the new non-zero values */
  i  = mat->row_end[rowno-1];
  ii = mat->row_end[rowno] - 1;
  firstcol = mat->columns + 1;
  if(isNZ) {
    /* See if we can do fast in-place replacements of leading items */
    while((i < ii) && (count > 0) && ((colnr = ROW_MAT_COLNR(i)) == *colno)) {
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      if(mat->is_roworder) {
        if(isA && doscale)
          value = scaled_mat(lp, value, colnr, rowno);
        if(isA)
          value = my_chsign(is_chsign(lp, colnr), value);
      }
      else {
        if(isA && doscale)
          value = scaled_mat(lp, value, rowno, colnr);
        if(isA)
          value = my_chsign(is_chsign(lp, rowno), value);
      }
      ROW_MAT_VALUE(i) = value;
      i++;
      count--;
      row++;
      colno++;
    }
    /* Proceed with remaining entries */
    newnr = count;
    if(newnr > 0)
      firstcol = colno[0];
  }
  else {
    newnr = 0;
    kk = mat->columns;
    if(i < ii)
      colnr = ROW_MAT_COLNR(i);
    else
      colnr = 0;
    for(k = 1; k <= kk; k++) {
      if(fabs(row[k]) > mat->epsvalue) {
        /* See if we can do fast in-place replacements of leading items */
        if((addto == NULL) && (i < ii) && (colnr == k)) {
          if(mat->is_roworder) {
            if(isA && doscale)
              value = scaled_mat(lp, value, colnr, rowno);
            if(isA)
              value = my_chsign(is_chsign(lp, colnr), value);
          }
          else {
            if(isA && doscale)
              value = scaled_mat(lp, value, rowno, colnr);
            if(isA)
              value = my_chsign(is_chsign(lp, rowno), value);
          }
          ROW_MAT_VALUE(i) = value;
          i++;
          if(i < ii)
            colnr = ROW_MAT_COLNR(i);
          else
            colnr = 0;
        }
        /* Otherwise update addto-list */
        else {
          if(addto == NULL) {
            if(!allocMYBOOL(lp, &addto, mat->columns + 1, TRUE))
              return( FALSE );
            firstcol = k;
          }
          addto[k] = TRUE;
          newnr++;
        }
      }
    }
  }
  if(newnr == 0)
    return( TRUE );
#endif

  /* Make sure we have enough matrix space */
  /* if(!inc_mat_space(mat, newnr)) { */
  if((mat_nz_unused(mat) <= newnr) && !inc_mat_space(mat, newnr)) {
    newnr = 0;
    goto Done;
  }

  /* Pack initial entries if existing row data has a lower column
     start index than the first index of the new vector */
  orignr = mat_nonzeros(mat);
  k = newnr - mat_rowlength(mat, rowno);
  kk = 0;
  if(rowno == 0)
    ii = 0;
  else
    ii = mat->row_end[rowno-1];
  if((orignr == 0) || (ii >= orignr))
    j = firstcol /* 1 */;
  else
    j = ROW_MAT_COLNR(ii);
  jj = mat->col_end[firstcol - 1];  /* Set the index of the insertion point for the first new value */
  if(jj >= orignr)
    colnr = firstcol;
  else
    colnr = COL_MAT_COLNR(jj);
  if(j < colnr) {
    jj = elmnr = mat->col_end[j-1];
    for( ; j < colnr; j++) {
      /* Shift entries in current column */
      for( ; jj < mat->col_end[j]; jj++) {
        if(COL_MAT_ROWNR(jj) != rowno) {
          COL_MAT_COPY(elmnr, jj);
          elmnr++;
        }
      }
      /* Update next column start index */
      mat->col_end[j] = elmnr;
    }
    jj_j = jj - elmnr;  /* The shrinkage count */
  }
  else {
    jj_j = 0;
    /* Adjust for case where we simply append values - jj is initially the first column item */
    if(mat->col_end[firstcol] == orignr)
      jj = orignr;
  }

  /* Make sure we have sufficient space for any additional entries and move existing data down;
     this ensures that we only have to relocate matrix elements up in the next stage */
  jj_j = MAX(0, newnr - jj_j);
  if(jj_j > 0) {
    if(!inc_mat_space(mat, jj_j)) {
      FREE(addto);
      return( FALSE );
    }
    if(orignr-jj > 0) {
      COL_MAT_MOVE(jj+jj_j, jj, orignr-jj);
    }
    jj += jj_j;
  }

  /* Handle case where the matrix was empty before (or we can simply append) */
  /* if(orignr == 0) { */
  if(mat->col_end[firstcol] == orignr) {
    if(isNZ)
      elmnr = count;
    else
      elmnr = mat->columns;
    jj_j = mat->col_end[firstcol] /* 0 */;
    for(newnr = 0; newnr < elmnr; newnr++) {
      if(isNZ)
        colnr = colno[newnr];
      else
        colnr = newnr + 1;
      /* Update column start position if we have crossed a column */
      while(colnr > firstcol) {
        mat->col_end[firstcol] = jj_j;
        firstcol++;
      }
      if(isNZ || addto[colnr]) {
        if(isNZ)
          value = row[newnr];
        else
          value = row[colnr];
#ifdef DoMatrixRounding
        value = roundToPrecision(value, mat->epsvalue);
#endif
        if(isA && doscale)
          value = scaled_mat(lp, value, rowno, colnr);
        if(isA)
          value = my_chsign(is_chsign(lp, rowno), value);
        SET_MAT_ijA(jj_j, rowno, colnr, value);
        jj_j++;
        /* Update last column start position */
        mat->col_end[firstcol] = jj_j;
        firstcol++;
      }
    }

    /* Make sure we update tail empty column offsets */
    while(firstcol <= mat->columns) {
      mat->col_end[firstcol] = jj_j;
      firstcol++;
    }
    jj_j = 0;
  }

  /* Start from the top of the first non-zero column of the new row */
  elmnr = orignr + jj_j;
  if(jj < elmnr) {
    if(isNZ)
      newnr = 0;
    else
      newnr = firstcol - 1;
    j = jj - mat->col_end[firstcol - 1];
    colnr = firstcol;
    while((jj < elmnr) || (newnr < count)) {

      /* Update column start position if we have crossed a column */
      while(colnr > firstcol) {
        mat->col_end[firstcol] = kk;
        firstcol++;
      }

      /* See if we have a row equal to or greater than the target row */
      jj_j = jj - j;
      if(jj < elmnr) {
        rownr = COL_MAT_ROWNR(jj);
        colnr = COL_MAT_COLNR(jj);
      }
      else {
        rownr = rowno;
        if(!isNZ)                              /* KE added this conditional on 13.9.2006 */
          colnr = firstcol + 1;
        else
          colnr = mat->columns + 1;
      }

      if(isNZ) {
        if(newnr < count)
          kk = colno[newnr];
        else
          kk = mat->columns + 1;
      }
      else
        kk = newnr + 1;

      /* Test if there is an available new item ... */
#if 1  /* PENO fix 27.2.2005 */
      if((isNZ && (kk > colnr)) ||                    /* If this is not the case */
         (!isNZ && ((kk > colnr) || (!addto[kk])))) {
        /* DELETE if there is an existing value */
        if(!isNZ && (kk <= colnr))
#else
      if((isNZ && (kk > colnr)) ||                    /* If this is not the case */
         (!isNZ && !addto[kk])) {
        /* DELETE if there is an existing value */
        if(!isNZ)
#endif
          newnr++;
        if(rownr == rowno) {
          kk = jj_j;
          j++;
          jj++;
          continue;
        }
        /* KEEP otherwise and move entry up */
        if(!isNZ && (colnr > kk)) {
          colnr = kk;
          kk = jj_j;
          continue;
        }
      }
      else if((colnr > kk) ||                         /* Existing column index > new => INSERT */
              ((colnr == kk) && (rownr >= rowno)) ) { /* Same column index, existing row >= target row => INSERT/REPLACE */

        if(isNZ)
          value = row[newnr];
        else
          value = row[newnr+1];
        newnr++;
#ifdef DoMatrixRounding
        value = roundToPrecision(value, mat->epsvalue);
#endif
        if(isA && doscale)
          value = scaled_mat(lp, value, rowno, /* colnr */ kk);
        if(isA)
          value = my_chsign(is_chsign(lp, rowno), value);
        SET_MAT_ijA(jj_j, rowno, kk, value);

        /* Adjust if we have inserted an element */
        if((colnr > kk) || (rownr > rowno)) {
          j--;
          jj--;
        }
        colnr = kk;
        kk = jj_j;
        jj++;
        continue;
      }

      /* Shift the matrix element up by the active difference */
      if(jj_j != jj) {
        COL_MAT_COPY(jj_j, jj);
      }
      kk = jj_j;
      jj++;

    }

    /* Update pending / incomplete column start position */
    while(colnr > firstcol) {
      mat->col_end[firstcol] = kk;
      firstcol++;
    }

    /* Make sure we update tail column offsets */
    jj_j = jj - j;
    while(firstcol <= mat->columns) {
      mat->col_end[firstcol] = jj_j;
      firstcol++;
    }
  }
  mat->row_end_valid = FALSE;

Done:
  if(saved != 0)
    row[0] = saved;
  FREE(addto);
  return( (MYBOOL) (newnr > 0) );

}

STATIC int mat_appendrow(MATrec *mat, int count, REAL *row, int *colno, REAL mult, MYBOOL checkrowmode)
{
  int    i, j, jj = 0, stcol, elmnr, orignr, newnr, firstcol;
  MYBOOL *addto = NULL, isA, isNZ;
  REAL   value, saved = 0;
  lprec  *lp = mat->lp;

  /* Check if we are in row order mode and should add as column instead;
     the matrix will be transposed at a later stage */
  if(checkrowmode && mat->is_roworder)
    return( mat_appendcol(mat, count, row, colno, mult, FALSE) );

  /* Do initialization and validation */
  isA = (MYBOOL) (mat == lp->matA);
  isNZ = (MYBOOL) (colno != NULL);
  if(isNZ && (count > 0)) {
    if(count > 1)
      sortREALByINT(row, colno, count, 0, TRUE);
    if((colno[0] < 1) || (colno[count-1] > mat->columns))
      return( 0 );
  }
  /* else if((row != NULL) && !mat->is_roworder) */
  else if(!isNZ && (row != NULL) && !mat->is_roworder)
    row[0] = 0;

  /* Capture OF definition in row mode */
  if(isA && mat->is_roworder) {
    if(isNZ && (colno[0] == 0)) {
      value = row[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      value = scaled_mat(lp, value, 0, lp->columns);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[lp->columns] = value;
      count--;
      row++;
      colno++;
    }
    else if(!isNZ && (row != NULL) && (row[0] != 0)) {
      value = saved = row[0];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      value = scaled_mat(lp, value, 0, lp->columns);
      value = my_chsign(is_maxim(lp), value);
      lp->orig_obj[lp->columns] = value;
      row[0] = 0;
    }
    else
      lp->orig_obj[lp->columns] = 0;
  }

  /* Optionally tally and map the new non-zero values */
  firstcol = mat->columns + 1;
  if(isNZ) {
    newnr = count;
    if(newnr) {
      firstcol = colno[0];
      jj = colno[newnr - 1];
    }
  }
  else {
    newnr = 0;
    if(row != NULL) {
      if(!allocMYBOOL(lp, &addto, mat->columns + 1, TRUE)) {
        return( newnr );
      }
      for(i = mat->columns; i >= 1; i--) {
        if(fabs(row[i]) > mat->epsvalue) {
          addto[i] = TRUE;
          firstcol = i;
          newnr++;
        }
      }
    }
  }

  /* Make sure we have sufficient space */
  if(!inc_mat_space(mat, newnr)) {
    newnr = 0;
    goto Done;
  }

  /* Insert the non-zero constraint values */
  orignr = mat_nonzeros(mat) - 1;
  elmnr = orignr + newnr;

  for(j = mat->columns; j >= firstcol; j--) {
    stcol = mat->col_end[j] - 1;
    mat->col_end[j] = elmnr + 1;

   /* Add a new non-zero entry */
    if(((isNZ) && (j == jj)) || ((addto != NULL) && (addto[j]))) {
      newnr--;
      if(isNZ) {
        value = row[newnr];
        if(newnr)
          jj = colno[newnr - 1];
        else
          jj = 0;
      }
      else
        value = row[j];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, mat->epsvalue);
#endif
      value *= mult;
      if(isA)
        value = scaled_mat(lp, value, mat->rows, j);
      SET_MAT_ijA(elmnr, mat->rows, j, value);
      elmnr--;
    }

   /* Shift previous column entries down */
    i = stcol - mat->col_end[j-1] + 1;
    if(i > 0) {
      orignr -= i;
      elmnr  -= i;
      COL_MAT_MOVE(elmnr+1, orignr+1, i);
    }
  }

Done:
  if(saved != 0)
    row[0] = saved;
  FREE(addto);

  return( newnr );

}

STATIC int mat_appendcol(MATrec *mat, int count, REAL *column, int *rowno, REAL mult, MYBOOL checkrowmode)
{
  int     i, row, elmnr, lastnr;
  REAL    value;
  MYBOOL  isA, isNZ;
  lprec   *lp = mat->lp;

  /* Check if we are in row order mode and should add as row instead;
     the matrix will be transposed at a later stage */
  if(checkrowmode && mat->is_roworder)
    return( mat_appendrow(mat, count, column, rowno, mult, FALSE) );

  /* Make sure we have enough space */
/*
  if(!inc_mat_space(mat, mat->rows+1))
    return( 0 );
*/
  if(column == NULL)
    i = 0;
  else if(rowno != NULL)
    i = count;
  else {
    int nrows = mat->rows;

    elmnr = 0;
    for(i = 1; i <= nrows; i++)
      if(column[i] != 0)
        elmnr++;
    i = elmnr;
  }
  if((mat_nz_unused(mat) <= i) && !inc_mat_space(mat, i))
    return( 0 );

  /* Do initialization and validation */
  isA = (MYBOOL) (mat == lp->matA);
  isNZ = (MYBOOL) (column == NULL || rowno != NULL);
  if(isNZ && (count > 0)) {
    if(count > 1)
      sortREALByINT(column, rowno, count, 0, TRUE);
    if((rowno[0] < 0))
      return( 0 );
  }
  if(rowno != NULL)
    count--;

  /* Append sparse regular constraint values */
  elmnr = mat->col_end[mat->columns - 1];
  if(column != NULL) {
    row = -1;
    for(i = ((isNZ || !mat->is_roworder) ? 0 : 1); i <= count ; i++) {
      value = column[i];
      if(fabs(value) > mat->epsvalue) {
        if(isNZ) {
          lastnr = row;
          row = rowno[i];
          /* Check if we have come to the Lagrangean constraints */
          if(row > mat->rows)
            break;
          if(row <= lastnr)
            return( -1 );
        }
        else
          row = i;
#ifdef DoMatrixRounding
        value = roundToPrecision(value, mat->epsvalue);
#endif
        if(mat->is_roworder)
          value *= mult;
        else if(isA) {
          value = my_chsign(is_chsign(lp, row), value);
          value = scaled_mat(lp, value, row, mat->columns);
          if(!mat->is_roworder && (row == 0)) {
            lp->orig_obj[mat->columns] = value;
            continue;
          }
        }

       /* Store the item and update counters */
        SET_MAT_ijA(elmnr, row, mat->columns, value);
        elmnr++;
      }
    }

   /* Fill dense Lagrangean constraints */
    if(get_Lrows(lp) > 0)
      mat_appendcol(lp->matL, get_Lrows(lp), column+mat->rows, NULL, mult, checkrowmode);

  }

 /* Set end of data */
  mat->col_end[mat->columns] = elmnr;

  return( mat->col_end[mat->columns] - mat->col_end[mat->columns-1] );
}

STATIC int mat_checkcounts(MATrec *mat, int *rownum, int *colnum, MYBOOL freeonexit)
{
  int i, j, n;
  int *rownr;

  if(rownum == NULL)
    allocINT(mat->lp, &rownum, mat->rows + 1, TRUE);
  if(colnum == NULL)
    allocINT(mat->lp, &colnum, mat->columns + 1, TRUE);

  for(i = 1 ; i <= mat->columns; i++) {
    j = mat->col_end[i - 1];
    n = mat->col_end[i];
    rownr = &COL_MAT_ROWNR(j);
    for(; j < n;
        j++, rownr += matRowColStep) {
      colnum[i]++;
      rownum[*rownr]++;
    }
  }

  n = 0;
  if((mat->lp->do_presolve != PRESOLVE_NONE) &&
     (mat->lp->spx_trace || (mat->lp->verbose > NORMAL))) {
    for(j = 1; j <= mat->columns; j++)
      if(colnum[j] == 0) {
        n++;
        report(mat->lp, FULL, "mat_checkcounts: Variable %s is not used in any constraints\n",
                              get_col_name(mat->lp, j));
      }
    for(i = 0; i <= mat->rows; i++)
      if(rownum[i] == 0) {
        n++;
        report(mat->lp, FULL, "mat_checkcounts: Constraint %s empty\n",
                              get_row_name(mat->lp, i));
      }
  }

  if(freeonexit) {
    FREE(rownum);
    FREE(colnum);
  }

  return( n );

}

STATIC MYBOOL mat_validate(MATrec *mat)
/* Routine to make sure that row mapping arrays are valid */
{
  int     i, j, je, *rownum;
  int     *rownr, *colnr;

  if(!mat->row_end_valid) {

    MEMCLEAR(mat->row_end, mat->rows + 1);
    allocINT(mat->lp, &rownum, mat->rows + 1, TRUE);

    /* First tally row counts and then cumulate them */
    j = mat_nonzeros(mat);
    rownr = &COL_MAT_ROWNR(0);
    for(i = 0; i < j; i++, rownr += matRowColStep)
      mat->row_end[*rownr]++;
    for(i = 1; i <= mat->rows; i++)
      mat->row_end[i] += mat->row_end[i - 1];

    /* Calculate the column index for every non-zero */
    for(i = 1; i <= mat->columns; i++) {
      j = mat->col_end[i - 1];
      je = mat->col_end[i];
      rownr = &COL_MAT_ROWNR(j);
      colnr = &COL_MAT_COLNR(j);
      for(; j < je; j++, rownr += matRowColStep, colnr += matRowColStep) {
#ifdef Paranoia
        if(/*(*colnr < 0) || (*colnr > mat->columns) || (Normally violated in primal phase 1) */
           (*rownr < 0) || (*rownr > mat->rows)) {
          report(mat->lp, SEVERE, "mat_validate: Matrix value storage error row %d [0..%d], column %d [1..%d]\n",
                                  *rownr, mat->rows, *colnr, mat->columns);
          mat->lp->spx_status = UNKNOWNERROR;
          return(FALSE);
        }
#endif
        *colnr = i;
        if(*rownr == 0)
          mat_set_rowmap(mat, rownum[*rownr],
                              *rownr, i, j);
        else
          mat_set_rowmap(mat, mat->row_end[*rownr - 1] + rownum[*rownr],
                              *rownr, i, j);
        rownum[*rownr]++;
      }
    }

    FREE(rownum);
    mat->row_end_valid = TRUE;
  }

  if(mat == mat->lp->matA)
    mat->lp->model_is_valid = TRUE;
  return( TRUE );
}

MYBOOL mat_get_data(lprec *lp, int matindex, MYBOOL isrow, int **rownr, int **colnr, REAL **value)
{
  MATrec *mat = lp->matA;

#if MatrixRowAccess == RAM_Index
  if(isrow)
    matindex = mat->row_mat[matindex];
  if(rownr != NULL)
    *rownr = &COL_MAT_ROWNR(matindex);
  if(colnr != NULL)
    *colnr = &COL_MAT_COLNR(matindex);
  if(value != NULL)
    *value = &COL_MAT_VALUE(matindex);

#else
  if(isrow) {
    if(rownr != NULL)
      *rownr = &ROW_MAT_ROWNR(matindex);
    if(colnr != NULL)
      *colnr = &ROW_MAT_COLNR(matindex);
    if(value != NULL)
      *value = &ROW_MAT_VALUE(matindex);
  }
  else {
    if(rownr != NULL)
      *rownr = &COL_MAT_ROWNR(matindex);
    if(colnr != NULL)
      *colnr = &COL_MAT_COLNR(matindex);
    if(value != NULL)
      *value = &COL_MAT_VALUE(matindex);
  }

#endif

  return( TRUE );
}


MYBOOL mat_set_rowmap(MATrec *mat, int row_mat_index, int rownr, int colnr, int col_mat_index)
{
#if MatrixRowAccess == RAM_Index
  mat->row_mat[row_mat_index] = col_mat_index;

#elif MatrixColAccess==CAM_Record
  mat->row_mat[row_mat_index].rownr = rownr;
  mat->row_mat[row_mat_index].colnr = colnr;
  mat->row_mat[row_mat_index].value = COL_MAT_VALUE(col_mat_index);

#else /* if MatrixColAccess==CAM_Vector */
  mat->row_mat_rownr[row_mat_index] = rownr;
  mat->row_mat_colnr[row_mat_index] = colnr;
  mat->row_mat_value[row_mat_index] = COL_MAT_VALUE(col_mat_index);

#endif

  return( TRUE );
}

/* Implement combined binary/linear sub-search for matrix look-up */
int mat_findelm(MATrec *mat, int row, int column)
{
  int low, high, mid, item;

#if 0
  if(mat->row_end_valid && (row > 0) &&
     (ROW_MAT_COLNR(mat->row_mat[(low = mat->row_end[row-1])]) == column))
    return(low);
#endif

  if((column < 1) || (column > mat->columns)) {
    report(mat->lp, IMPORTANT, "mat_findelm: Column %d out of range\n", column);
    return( -1 );
  }
  if((row < 0) || (row > mat->rows)) {
    report(mat->lp, IMPORTANT, "mat_findelm: Row %d out of range\n", row);
    return( -1 );
  }

  low = mat->col_end[column - 1];
  high = mat->col_end[column] - 1;
  if(low > high)
    return( -2 );

 /* Do binary search logic */
  mid = (low+high) / 2;
  item = COL_MAT_ROWNR(mid);
  while(high - low > LINEARSEARCH) {
    if(item < row) {
      low = mid + 1;
      mid = (low+high) / 2;
      item = COL_MAT_ROWNR(mid);
    }
    else if(item > row) {
      high = mid - 1;
      mid = (low+high) / 2;
      item = COL_MAT_ROWNR(mid);
    }
    else {
      low = mid;
      high = mid;
    }
  }

 /* Do linear scan search logic */
  if((high > low) && (high - low <= LINEARSEARCH)) {
    item = COL_MAT_ROWNR(low);
    while((low < high) && (item < row)) {
      low++;
      item = COL_MAT_ROWNR(low);
    }
    if(item == row)
      high = low;
  }

  if((low == high) && (row == item))
    return( low );
  else
    return( -2 );
}

int mat_findins(MATrec *mat, int row, int column, int *insertpos, MYBOOL validate)
{
  int low, high, mid, item, exitvalue, insvalue;

#if 0
  if(mat->row_end_valid && (row > 0) &&
     (ROW_MAT_COLNR(mat->row_mat[(low = mat->row_end[row-1])]) == column)) {
    insvalue = low;
    exitvalue = low;
    goto Done;
  }
#endif

  insvalue = -1;

  if((column < 1) || (column > mat->columns)) {
    if((column > 0) && !validate) {
      insvalue = mat->col_end[mat->columns];
      exitvalue = -2;
      goto Done;
    }
    report(mat->lp, IMPORTANT, "mat_findins: Column %d out of range\n", column);
    exitvalue = -1;
    goto Done;
  }
  if((row < 0) || (row > mat->rows)) {
    if((row >= 0) && !validate) {
      insvalue = mat->col_end[column];
      exitvalue = -2;
      goto Done;
    }
    report(mat->lp, IMPORTANT, "mat_findins: Row %d out of range\n", row);
    exitvalue = -1;
    goto Done;
  }

  low = mat->col_end[column - 1];
  insvalue = low;
  high = mat->col_end[column] - 1;
  if(low > high) {
    exitvalue = -2;
    goto Done;
  }

 /* Do binary search logic */
  mid = (low+high) / 2;
  item = COL_MAT_ROWNR(mid);
  while(high - low > LINEARSEARCH) {
    if(item < row) {
      low = mid + 1;
      mid = (low+high) / 2;
      item = COL_MAT_ROWNR(mid);
    }
    else if(item > row) {
      high = mid - 1;
      mid = (low+high) / 2;
      item = COL_MAT_ROWNR(mid);
    }
    else {
      low = mid;
      high = mid;
    }
  }

 /* Do linear scan search logic */
  if((high > low) && (high - low <= LINEARSEARCH)) {
    item = COL_MAT_ROWNR(low);
    while((low < high) && (item < row)) {
      low++;
      item = COL_MAT_ROWNR(low);
    }
    if(item == row)
      high = low;
  }

  insvalue = low;
  if((low == high) && (row == item))
    exitvalue = low;
  else {
    if((low < mat->col_end[column]) && (COL_MAT_ROWNR(low) < row))
      insvalue++;
    exitvalue = -2;
  }

Done:
  if(insertpos != NULL)
    (*insertpos) = insvalue;
  return( exitvalue );
}

STATIC REAL mat_getitem(MATrec *mat, int row, int column)
{
  int elmnr;

#ifdef DirectOverrideOF
  if((row == 0) && (mat == mat->lp->matA) && (mat->lp->OF_override != NULL))
    return( mat->lp->OF_override[column] );
  else
#endif
  {
    elmnr = mat_findelm(mat, row, column);
    if(elmnr >= 0)
      return( COL_MAT_VALUE(elmnr) );
    else
      return( 0 );
  }
}

STATIC MYBOOL mat_additem(MATrec *mat, int row, int column, REAL delta)
{
  int elmnr;

#ifdef DirectOverrideOF
  if((row == 0) && (mat == mat->lp->matA) && (mat->lp->OF_override != NULL))
    return( mat->lp->OF_override[column] );
  else
#endif
  {
    elmnr = mat_findelm(mat, row, column);
    if(elmnr >= 0) {
      COL_MAT_VALUE(elmnr) += delta;
      return( TRUE );
    }
    else {
      mat_setitem(mat, row, column, delta);
      return( FALSE );
    }
  }
}

STATIC MYBOOL mat_setitem(MATrec *mat, int row, int column, REAL value)
{
  return( mat_setvalue(mat, row, column, value, FALSE) );
}

STATIC void mat_multrow(MATrec *mat, int row_nr, REAL mult)
{
  int i, k1, k2;

#if 0
  if(row_nr == 0) {
    k2 = mat->col_end[0];
    for(i = 1; i <= mat->columns; i++) {
      k1 = k2;
      k2 = mat->col_end[i];
      if((k1 < k2) && (COL_MAT_ROWNR(k1) == row_nr))
        COL_MAT_VALUE(k1) *= mult;
    }
  }
  else if(mat_validate(mat)) {
    if(row_nr == 0)
      k1 = 0;
    else
#else
  if(mat_validate(mat)) {
    if(row_nr == 0)
      k1 = 0;
    else
#endif
    k1 = mat->row_end[row_nr-1];
    k2 = mat->row_end[row_nr];
    for(i = k1; i < k2; i++)
      ROW_MAT_VALUE(i) *= mult;
  }
}

STATIC void mat_multcol(MATrec *mat, int col_nr, REAL mult, MYBOOL DoObj)
{
  int    i, ie;
  MYBOOL isA;

#ifdef Paranoia
  if((col_nr < 1) || (col_nr > mat->columns)) {
    report(mat->lp, IMPORTANT, "mult_column: Column %d out of range\n", col_nr);
    return;
  }
#endif
  if(mult == 1.0)
    return;

  isA = (MYBOOL) (mat == mat->lp->matA);

  ie = mat->col_end[col_nr];
  for(i = mat->col_end[col_nr - 1]; i < ie; i++)
    COL_MAT_VALUE(i) *= mult;
  if(isA) {
    if(DoObj)
      mat->lp->orig_obj[col_nr] *= mult;
    if(get_Lrows(mat->lp) > 0)
      mat_multcol(mat->lp->matL, col_nr, mult, DoObj);
  }
}

STATIC void mat_multadd(MATrec *mat, REAL *lhsvector, int varnr, REAL mult)
{
  int               colnr;
  register int      ib, ie, *matRownr;
  register REAL     *matValue;

  /* Handle case of a slack variable */
  if(varnr <= mat->lp->rows) {
    lhsvector[varnr] += mult;
    return;
  }

  /* Do operation on the objective */
  if(mat->lp->matA == mat)
    lhsvector[0] += get_OF_active(mat->lp, varnr, mult);

  /* Scan the constraint matrix target columns */
  colnr = varnr - mat->lp->rows;
  ib = mat->col_end[colnr - 1];
  ie = mat->col_end[colnr];
  if(ib < ie) {

    /* Initialize pointers */
    matRownr = &COL_MAT_ROWNR(ib);
    matValue = &COL_MAT_VALUE(ib);

    /* Then loop over all regular rows */
    for(; ib < ie;
        ib++, matValue += matValueStep, matRownr += matRowColStep) {
      lhsvector[*matRownr] += mult * (*matValue);
    }
  }

}

STATIC MYBOOL mat_setvalue(MATrec *mat, int Row, int Column, REAL Value, MYBOOL doscale)
{
  int    elmnr, lastelm, i, RowA = Row, ColumnA = Column;
  MYBOOL isA;

  /* This function is inefficient if used to add new matrix entries in
     other places than at the end of the matrix. OK for replacing existing
     a non-zero value with another non-zero value */
  isA = (MYBOOL) (mat == mat->lp->matA);
  if(mat->is_roworder)
    swapINT(&Row, &Column);

  /* Set small numbers to zero */
  if(fabs(Value) < mat->epsvalue)
    Value = 0;
#ifdef DoMatrixRounding
  else
    Value = roundToPrecision(Value, mat->epsvalue);
#endif

  /* Check if we need to update column space */
  if(Column > mat->columns) {
    if(isA)
      inc_col_space(mat->lp, ColumnA - mat->columns);
    else
      inc_matcol_space(mat, Column - mat->columns);
  }

  /* Find out if we already have such an entry, or return insertion point */
  i = mat_findins(mat, Row, Column, &elmnr, FALSE);
  if(i == -1)
    return(FALSE);

  if(isA)
    set_action(&mat->lp->spx_action, ACTION_REBASE | ACTION_RECOMPUTE | ACTION_REINVERT);

  if(i >= 0) {
    /* there is an existing entry */
    if(fabs(Value) > mat->epsvalue) { /* we replace it by something non-zero */
      if(isA) {
        Value = my_chsign(is_chsign(mat->lp, RowA), Value);
        if(doscale && mat->lp->scaling_used)
          Value = scaled_mat(mat->lp, Value, RowA, ColumnA);
      }
      COL_MAT_VALUE(elmnr) = Value;
    }
    else { /* setting existing non-zero entry to zero. Remove the entry */
      /* This might remove an entire column, or leave just a bound. No
          nice solution for that yet */

      /* Shift up tail end of the matrix */
      lastelm = mat_nonzeros(mat);
#if 0
      for(i = elmnr; i < lastelm ; i++) {
        COL_MAT_COPY(i, i + 1);
      }
#else
      lastelm -= elmnr;
      COL_MAT_MOVE(elmnr, elmnr + 1, lastelm);
#endif
      for(i = Column; i <= mat->columns; i++)
        mat->col_end[i]--;

      mat->row_end_valid = FALSE;
    }
  }
  else if(fabs(Value) > mat->epsvalue) {
    /* no existing entry. make new one only if not nearly zero */
    /* check if more space is needed for matrix */
    if(!inc_mat_space(mat, 1))
      return(FALSE);

    if(Column > mat->columns) {
      i = mat->columns + 1;
      if(isA)
        shift_coldata(mat->lp, i, ColumnA - mat->columns, NULL);
      else
        mat_shiftcols(mat, &i, Column - mat->columns, NULL);
    }

    /* Shift down tail end of the matrix by one */
    lastelm = mat_nonzeros(mat);
#if 1 /* Does compiler optimization work better here? */
    for(i = lastelm; i > elmnr ; i--) {
      COL_MAT_COPY(i, i - 1);
    }
#else
    lastelm -= elmnr - 1;
    COL_MAT_MOVE(elmnr + 1, elmnr, lastelm);
#endif

    /* Set new element */
    if(isA) {
      Value = my_chsign(is_chsign(mat->lp, RowA), Value);
      if(doscale)
        Value = scaled_mat(mat->lp, Value, RowA, ColumnA);
    }
    SET_MAT_ijA(elmnr, Row, Column, Value);

    /* Update column indexes */
    for(i = Column; i <= mat->columns; i++)
      mat->col_end[i]++;

    mat->row_end_valid = FALSE;
  }

  if(isA && (mat->lp->var_is_free != NULL) && (mat->lp->var_is_free[ColumnA] > 0))
    return( mat_setvalue(mat, RowA, mat->lp->var_is_free[ColumnA], -Value, doscale) );
  return(TRUE);
}

STATIC MYBOOL mat_appendvalue(MATrec *mat, int Row, REAL Value)
{
  int *elmnr, Column = mat->columns;

  /* Set small numbers to zero */
  if(fabs(Value) < mat->epsvalue)
    Value = 0;
#ifdef DoMatrixRounding
  else
    Value = roundToPrecision(Value, mat->epsvalue);
#endif

  /* Check if more space is needed for matrix */
  if(!inc_mat_space(mat, 1))
    return(FALSE);

#ifdef Paranoia
  /* Check valid indeces */
  if((Row < 0) || (Row > mat->rows)) {
    report(mat->lp, SEVERE, "mat_appendvalue: Invalid row index %d specified\n", Row);
    return(FALSE);
  }
#endif

  /* Get insertion point and set value */
  elmnr = mat->col_end + Column;
  SET_MAT_ijA((*elmnr), Row, Column, Value);

  /* Update column count */
  (*elmnr)++;
  mat->row_end_valid = FALSE;

  return(TRUE);
}

STATIC MYBOOL mat_equalRows(MATrec *mat, int baserow, int comprow)
{
  MYBOOL status = FALSE;

  if(mat_validate(mat)) {
    int bj1 = 0, ej1, bj2 = 0, ej2;

    /* Get starting and ending positions */
    if(baserow >= 0)
      bj1 = mat->row_end[baserow-1];
    ej1 = mat->row_end[baserow];
    if(comprow >= 0)
      bj2 = mat->row_end[comprow-1];
    ej2 = mat->row_end[comprow];
    /* Fail if row lengths are unequal */
    if((ej1-bj1) != (ej2-bj2))
      return( status );

    /* Compare column index and value, element by element */
    for(; bj1 < ej1; bj1++, bj2++) {
      if(COL_MAT_COLNR(bj1) != COL_MAT_COLNR(bj2))
        break;
#if 1
      if(fabs(get_mat_byindex(mat->lp, bj1, TRUE, FALSE)-get_mat_byindex(mat->lp, bj2, TRUE, FALSE)) > mat->lp->epsprimal)
#else
      if(fabs(COL_MAT_VALUE(bj1)-COL_MAT_VALUE(bj2)) > mat->lp->epsprimal)
#endif
        break;
    }
    status = (MYBOOL) (bj1 == ej1);
  }
  return( status );
}

STATIC int mat_findcolumn(MATrec *mat, int matindex)
{
  int j;

  for(j = 1; j <= mat->columns; j++) {
    if(matindex < mat->col_end[j])
      break;
  }
  return(j);
}

STATIC int mat_expandcolumn(MATrec *mat, int colnr, REAL *column, int *nzlist, MYBOOL signedA)
{
  MYBOOL  isA = (MYBOOL) (mat->lp->matA == mat);
  int     i, ie, j, nzcount = 0;
  REAL    *matValue;
  int     *matRownr;

  signedA &= isA;

  /* Retrieve a column from the user data matrix A */
  MEMCLEAR(column, mat->rows + 1);
  if(isA) {
    column[0] = mat->lp->orig_obj[colnr];
    if(signedA && is_chsign(mat->lp, 0))
      column[0] = -column[0];
  }

  i = mat->col_end[colnr - 1];
  ie = mat->col_end[colnr];
  matRownr = &COL_MAT_ROWNR(i);
  matValue = &COL_MAT_VALUE(i);
  for(; i < ie;
      i++, matRownr += matRowColStep, matValue += matValueStep) {
    j = *matRownr;
    column[j] = *matValue;
    if(signedA && is_chsign(mat->lp, j))
      column[j] = -column[j];
    nzcount++;
    if(nzlist != NULL)
      nzlist[nzcount] = j;
  }
  if(nzlist != NULL)
    nzlist[0] = nzcount;
  return( nzcount );
}

STATIC MYBOOL mat_computemax(MATrec *mat)
{
  int  *rownr = &COL_MAT_ROWNR(0),
       *colnr = &COL_MAT_COLNR(0),
       i = 0, ie = mat->col_end[mat->columns], ez = 0;
  REAL *value = &COL_MAT_VALUE(0), epsmachine = mat->lp->epsmachine, absvalue;

  /* Prepare arrays */
  if(!allocREAL(mat->lp, &mat->colmax, mat->columns_alloc+1, AUTOMATIC) ||
     !allocREAL(mat->lp, &mat->rowmax, mat->rows_alloc+1, AUTOMATIC))
     return( FALSE );
  MEMCLEAR(mat->colmax, mat->columns+1);
  MEMCLEAR(mat->rowmax, mat->rows+1);

  /* Obtain the row and column maxima in one sweep */
  mat->dynrange = mat->lp->infinite;
  for(; i < ie;
      i++, rownr += matRowColStep, colnr += matRowColStep, value += matValueStep) {
    absvalue = fabs(*value);
    SETMAX(mat->colmax[*colnr], absvalue);
    SETMAX(mat->rowmax[*rownr], absvalue);
    SETMIN(mat->dynrange, absvalue);
    if(absvalue < epsmachine)
      ez++;
  }

  /* Lastly, compute the global maximum and get the dynamic range */
  for(i = 1; i <= mat->rows; i++)
    SETMAX(mat->rowmax[0], mat->rowmax[i]);
  mat->infnorm = mat->colmax[0] = mat->rowmax[0];
  if(mat->dynrange == 0) {
    report(mat->lp, SEVERE, "%d matrix contains zero-valued coefficients.\n", ez);
    mat->dynrange = mat->lp->infinite;
  }
  else {
    mat->dynrange = mat->infnorm / mat->dynrange;
    if(ez > 0)
      report(mat->lp, IMPORTANT, "%d matrix coefficients below machine precision were found.\n", ez);
  }

  return( TRUE );
}

STATIC MYBOOL mat_transpose(MATrec *mat)
{
  int     i, j, nz, k;
  MYBOOL  status;

  status = mat_validate(mat);
  if(status) {

    /* Create a column-ordered sparse element list; "column" index must be shifted */
    nz = mat_nonzeros(mat);
    if(nz > 0) {
#if MatrixColAccess==CAM_Record
      MATitem *newmat;
      newmat = (MATitem *) malloc((mat->mat_alloc) * sizeof(*(mat->col_mat)));
      j = mat->row_end[0];
      for(i = nz-1; i >= j ; i--) {
        k = i-j;
        newmat[k] = mat->col_mat[mat->row_mat[i]];
        newmat[k].row_nr = newmat[k].col_nr;
      }
      for(i = j-1; i >= 0 ; i--) {
        k = nz-j+i;
        newmat[k] = mat->col_mat[mat->row_mat[i]];
        newmat[k].row_nr = newmat[k].col_nr;
      }
      swapPTR((void **) &mat->col_mat, (void **) &newmat);
      FREE(newmat);
#else /*if MatrixColAccess==CAM_Vector*/
      REAL *newValue = NULL;
      int  *newRownr = NULL;
      allocREAL(mat->lp, &newValue, mat->mat_alloc, FALSE);
      allocINT(mat->lp, &newRownr, mat->mat_alloc, FALSE);

      j = mat->row_end[0];
      for(i = nz-1; i >= j ; i--) {
        k = i-j;
        newValue[k] = ROW_MAT_VALUE(i);
        newRownr[k] = ROW_MAT_COLNR(i);
      }
      for(i = j-1; i >= 0 ; i--) {
        k = nz-j+i;
        newValue[k] = ROW_MAT_VALUE(i);
        newRownr[k] = ROW_MAT_COLNR(i);
      }

      swapPTR((void **) &mat->col_mat_rownr, (void **) &newRownr);
      swapPTR((void **) &mat->col_mat_value, (void **) &newValue);
      FREE(newValue);
      FREE(newRownr);
#endif
    }

    /* Transfer row start to column start position; must adjust for different offsets */
    if(mat->rows == mat->rows_alloc)
      inc_matcol_space(mat, 1);
    j = mat->row_end[0];
    for(i = mat->rows; i >= 1; i--)
      mat->row_end[i] -= j;
    mat->row_end[mat->rows] = nz;
    swapPTR((void **) &mat->row_end, (void **) &mat->col_end);

    /* Swap arrays of maximum values */
    swapPTR((void **) &mat->rowmax, (void **) &mat->colmax);

    /* Swap array sizes */
    swapINT(&mat->rows, &mat->columns);
    swapINT(&mat->rows_alloc, &mat->columns_alloc);

    /* Finally set current storage mode */
    mat->is_roworder = (MYBOOL) !mat->is_roworder;
    mat->row_end_valid = FALSE;
  }
  return(status);
}


/* ---------------------------------------------------------------------------------- */
/* Change-tracking routines                                                           */
/* ---------------------------------------------------------------------------------- */
STATIC DeltaVrec *createUndoLadder(lprec *lp, int levelitems, int maxlevels)
{
  DeltaVrec *hold;

  hold = (DeltaVrec *) malloc(sizeof(*hold));
  hold->lp = lp;
  hold->activelevel = 0;
  hold->tracker = mat_create(lp, levelitems, 0, 0.0);
  inc_matcol_space(hold->tracker, maxlevels);
  return( hold );
}
STATIC int incrementUndoLadder(DeltaVrec *DV)
{
  DV->activelevel++;
  inc_matcol_space(DV->tracker, 1);
  mat_shiftcols(DV->tracker, &(DV->activelevel), 1, NULL);
  DV->tracker->columns++;
  return(DV->activelevel);
}
STATIC MYBOOL modifyUndoLadder(DeltaVrec *DV, int itemno, REAL target[], REAL newvalue)
{
  MYBOOL status;
  int    varindex = itemno;
  REAL   oldvalue = target[itemno];

#ifndef UseMilpSlacksRCF  /* Check if we should include ranged constraints */
  varindex -= DV->lp->rows;
#endif
  status = mat_appendvalue(DV->tracker, varindex, oldvalue);
  target[itemno] = newvalue;
  return(status);
}
STATIC int countsUndoLadder(DeltaVrec *DV)
{
  if(DV->activelevel > 0)
    return( mat_collength(DV->tracker, DV->activelevel) );
  else
    return( 0 );
}
STATIC int restoreUndoLadder(DeltaVrec *DV, REAL target[])
{
  int iD = 0;

  if(DV->activelevel > 0) {
    MATrec *mat = DV->tracker;
    int    iB = mat->col_end[DV->activelevel-1],
           iE = mat->col_end[DV->activelevel],
           *matRownr = &COL_MAT_ROWNR(iB);
    REAL   *matValue = &COL_MAT_VALUE(iB),
           oldvalue;

    /* Restore the values */
    iD = iE-iB;
    for(; iB < iE; iB++, matValue += matValueStep, matRownr += matRowColStep) {
      oldvalue = *matValue;
#ifdef UseMilpSlacksRCF  /* Check if we should include ranged constraints */
      target[(*matRownr)] = oldvalue;
#else
      target[DV->lp->rows+(*matRownr)] = oldvalue;
#endif
    }

    /* Get rid of the changes */
    mat_shiftcols(DV->tracker, &(DV->activelevel), -1, NULL);
  }

  return(iD);
}
STATIC int decrementUndoLadder(DeltaVrec *DV)
{
  int deleted = 0;

  if(DV->activelevel > 0) {
    deleted = mat_shiftcols(DV->tracker, &(DV->activelevel), -1, NULL);
    DV->activelevel--;
    DV->tracker->columns--;
  }
  return(deleted);
}
STATIC MYBOOL freeUndoLadder(DeltaVrec **DV)
{
  if((DV == NULL) || (*DV == NULL))
    return(FALSE);

  mat_free(&((*DV)->tracker));
  FREE(*DV);
  return(TRUE);
}

STATIC MYBOOL appendUndoPresolve(lprec *lp, MYBOOL isprimal, REAL beta, int colnrDep)
{
  MATrec *mat;

  /* Point to correct undo structure */
  if(isprimal)
    mat = lp->presolve_undo->primalundo->tracker;
  else
    mat = lp->presolve_undo->dualundo->tracker;

  /* Append the data */
  if((colnrDep > 0) && (beta != 0) &&
     (mat != NULL) && (mat->col_tag[0] > 0)) {
    int ix = mat->col_tag[0];
#if 0
    report(lp, NORMAL, "appendUndoPresolve: %s %g * x%d\n",
                       ( beta < 0 ? "-" : "+"), fabs(beta), colnrDep);
#endif

    /* Do normal user variable case */
    if(colnrDep <= lp->columns)
      mat_setvalue(mat, colnrDep, ix, beta, FALSE);

    /* Handle case where a slack variable is referenced */
    else {
      int ipos, jx = mat->col_tag[ix];
      mat_setvalue(mat, jx, ix, beta, FALSE);
      jx = mat_findins(mat, jx, ix, &ipos, FALSE);
      COL_MAT_ROWNR(ipos) = colnrDep;
    }
    return( TRUE );
  }
  else
    return( FALSE );
}
STATIC MYBOOL addUndoPresolve(lprec *lp, MYBOOL isprimal, int colnrElim, REAL alpha, REAL beta, int colnrDep)
{
  int       ix;
  DeltaVrec **DV;
  MATrec    *mat;
  presolveundorec *psdata = lp->presolve_undo;

  /* Point to and initialize undo structure at first call */
  if(isprimal) {
    DV = &(psdata->primalundo);
    if(*DV == NULL) {
      *DV = createUndoLadder(lp, lp->columns+1, lp->columns);
      mat = (*DV)->tracker;
      mat->epsvalue = lp->matA->epsvalue;
      allocINT(lp, &(mat->col_tag), lp->columns+1, FALSE);
      mat->col_tag[0] = 0;
    }
  }
  else {
    DV = &(psdata->dualundo);
    if(*DV == NULL) {
      *DV = createUndoLadder(lp, lp->rows+1, lp->rows);
      mat = (*DV)->tracker;
      mat->epsvalue = lp->matA->epsvalue;
      allocINT(lp, &(mat->col_tag), lp->rows+1, FALSE);
      mat->col_tag[0] = 0;
    }
  }
  mat = (*DV)->tracker;
#if 0
  report(lp, NORMAL, "addUndoPresolve: x%d = %g %s %g * x%d\n",
                     colnrElim, alpha, ( beta < 0 ? "-" : "+"), fabs(beta), colnrDep);
#endif
  /* Add the data */
  ix = mat->col_tag[0] = incrementUndoLadder(*DV);
  mat->col_tag[ix] = colnrElim;
  if(alpha != 0)
    mat_setvalue(mat, 0, ix, alpha, FALSE);
/*    mat_appendvalue(*mat, 0, alpha);*/
  if((colnrDep > 0) && (beta != 0)) {
    if(colnrDep > lp->columns)
      return( appendUndoPresolve(lp, isprimal, beta, colnrDep) );
    else
      mat_setvalue(mat, colnrDep, ix, beta, FALSE);
  }

  return( TRUE );
}



/* ---------------------------------------------------------------------------------- */
/* High level matrix inverse and product routines in lp_solve                         */
/* ---------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------- */
/*    A brief description of the basis inverse and factorization logic in lp_solve    */
/* ---------------------------------------------------------------------------------- */
/*

   In order to better understand the legacy code for operating with the
   basis and its factorization in lp_solve I (KE) will briefly explain
   the conventions and associated matrix algebra.  Note that with lp_solve
   version 5.5, it is also possible to direct lp_solve to use the traditional
   (textbook) format by setting the obj_in_B parameter to FALSE.

   The matrix description of a linear program (as represented by lp_solve) goes
   like this:

           maximize         c'x
           subject to  r <=  Ax <= b
           where       l <=   x <= u

   The matrix A is partitioned into two column sets [B|N], where B is
   a square matrix of "basis" variables containing non-fixed
   variables of the linear program at any given stage and N is the
   submatrix of corresponding non-basic, fixed variables. The
   variables (columns) in N may be fixed at their lower or upper levels.

   Similarly, the c vector is partitioned into the basic and non-basic
   parts [z|n].

   While lp_solve stores the objective vector c in a dense format, and
   the constraint matrix A in a (fairly standard) sparse format, the
   column vectors passed to the factorization routine include the
   objective coefficient at row index 0.  (In versions of lp_solve
   before v5.2, c was actually explicitly stored as the 0-th row of A).
   The expanded matrix may be called the "A~" form and looks like this:

                       A~ = [ c ]
                            [ A ]

   Linear programming involves solving linear equations based on the
   square basis matrix B, which includes is a subset of columns from A~.
   The implications of the common storage of c and A (e.g. A~) vs. the
   inverse / factorization of B for the operations and updates performed
   by the simplex routine therefore needs to be understood.  As a consquence
   of A~, in lp_solve B is stored in an expanded, bordered format using the
   following (non-singular) representation:

                       B~ = [ 1 z ]
                            [ 0 B ]

   Any basis inversion / factorization engine used by lp_solve must therefore
   explicitly represent and handle the implications of this structure for
   associated matrix operations.

   The standard matrix formula for computing the inverse of a bordered
   matrix shows what the inversion of B~ actually produces:

                  Inv(B~) = [ 1 -z*Inv(B) ]
                            [ 0   Inv(B)  ]

   The A~ and B~ representations require awareness by the developer of the side
   effects of the presence of the top row when doing product operations such as
   b'N, btran and ftran.  Note in particular z*Inv(B) in the top row of Inv(B~),
   which is actually the dual solution vector of the given basis.  This fact
   makes a very common update in the simplex algorithm (reduced costs) returnable
   as a vector simply by setting 1 at the top of a vector being pre-multiplied
   with Inv(B~).

   However, if the objective vector (c) is changed, the expanded representation
   requires that B / B~ be refactorized.  Also, when doing FTRAN, BTRAN
   and x'A-type operations, you will patently get the incorrect result
   if you simply copy the operations given in textbooks.  First I'll show the
   results of an FTRAN operation:

                   Bx = a  ==>  x = FTRAN(a)

   In lp_solve, this operation solves:

                   [ 1 z ] [y] = [d]
                   [ 0 B ] [x]   [a]

   Using the Inv(B~) expression earlier, the FTRAN result is therefore:

             [y] = [ 1 -z*Inv(B) ] [d] = [ d - z*Inv(B)*a ]
             [x]   [ 0   Inv(B)  ] [a]   [   Inv(B)*a     ]

   As an example, the value of the dual objective can be returned at the
   0-th index by passing the active RHS vector with 0 at the 0-th position.

   Similarily, doing the left solve - performing the BTRAN calculation:

                   [x y] [ 1 z ] = [d a']
                         [ 0 B ]

   ... will produce the following result in lp_solve:

   [x y] = [d a'] [ 1 -z*Inv(B) ] = [ d | -d*z*Inv(B) + a'*Inv(B) ]
                  [ 0   Inv(B)  ]

   So, if you thought you were simply computing "a'*Inv(B)", look again.
   In order to produce the desired result, you have to set d to 0 before
   the BTRAN operation.  On the other hand, if you set d to 1 and a to 0,
   then you are very conveniently on your way to obtain the reduced costs
   (needs a further matrix premultiplication with non-basic variables).

   Incidentally, the BTRAN with [1 0] that yields [ 1 | -z*Inv(B) ] can
   also be used as a fast way of checking the accuracy of the current
   factorization.

   Equipped with this understanding, I hope that you see that
   the approach in lp_solve is actually pretty convenient.  It also
   becomes easier to extend functionality in lp_solve by drawing on
   formulas and expressions from LP literature that otherwise assume
   the non-bordered syntax and representation.

                                     Kjell Eikland -- November 2003
                                     KE update     -- April 2005
                                     KE update     -- June 2005

*/

STATIC MYBOOL __WINAPI invert(lprec *lp, MYBOOL shiftbounds, MYBOOL final)
{
  MYBOOL *usedpos, resetbasis;
  REAL   test;
  int    k, i, j;
  int    singularities, usercolB;

 /* Make sure the tags are correct */
  if(!mat_validate(lp->matA)) {
    lp->spx_status = INFEASIBLE;
    return(FALSE);
  }

 /* Create the inverse management object at the first call to invert() */
  if(lp->invB == NULL)
    lp->bfp_init(lp, lp->rows, 0, NULL);
  else
    lp->bfp_preparefactorization(lp);
  singularities = 0;

 /* Must save spx_status since it is used to carry information about
    the presence and handling of singular columns in the matrix */
  if(userabort(lp, MSG_INVERT))
    return(FALSE);

#ifdef Paranoia
  if(lp->spx_trace)
    report(lp, DETAILED, "invert: Iter %10g, fact-length %7d, OF " RESULTVALUEMASK ".\n",
                         (double) get_total_iter(lp), lp->bfp_colcount(lp), (double) -lp->rhs[0]);
#endif

 /* Store state of pre-existing basis, and at the same time check if
    the basis is I; in this case take the easy way out */
  if(!allocMYBOOL(lp, &usedpos, lp->sum + 1, TRUE)) {
    lp->bb_break = TRUE;
    return(FALSE);
  }
  usedpos[0] = TRUE;
  usercolB = 0;
  for(i = 1; i <= lp->rows; i++) {
    k = lp->var_basic[i];
    if(k > lp->rows)
      usercolB++;
    usedpos[k] = TRUE;
  }
#ifdef Paranoia
  if(!verify_basis(lp))
    report(lp, SEVERE, "invert: Invalid basis detected (iter %g).\n",
                       (double) get_total_iter(lp));
#endif

 /* Tally matrix nz-counts and check if we should reset basis
    indicators to all slacks */
  resetbasis = (MYBOOL) ((usercolB > 0) && lp->bfp_canresetbasis(lp));
  k = 0;
  for(i = 1; i <= lp->rows; i++) {
    if(lp->var_basic[i] > lp->rows)
      k += mat_collength(lp->matA, lp->var_basic[i] - lp->rows) + (is_OF_nz(lp,lp->var_basic[i] - lp->rows) ? 1 : 0);
    if(resetbasis) {
      j = lp->var_basic[i];
      if(j > lp->rows)
        lp->is_basic[j] = FALSE;
      lp->var_basic[i] = i;
      lp->is_basic[i] = TRUE;
    }
  }

 /* Now do the refactorization */
  singularities = lp->bfp_factorize(lp, usercolB, k, usedpos, final);

 /* Do user reporting */
  if(userabort(lp, MSG_INVERT))
    goto Cleanup;

 /* Finalize factorization/inversion */
  lp->bfp_finishfactorization(lp);

  /* Recompute the RHS ( Ref. lp_solve inverse logic and Chvatal p. 121 ) */
#ifdef DebugInv
  blockWriteLREAL(stdout, "RHS-values pre invert", lp->rhs, 0, lp->rows);
#endif
  recompute_solution(lp, shiftbounds);
  restartPricer(lp, AUTOMATIC);
#ifdef DebugInv
  blockWriteLREAL(stdout, "RHS-values post invert", lp->rhs, 0, lp->rows);
#endif

Cleanup:
  /* Check for numerical instability indicated by frequent refactorizations */
  test = get_refactfrequency(lp, FALSE);
  if(test < MIN_REFACTFREQUENCY) {
    test = get_refactfrequency(lp, TRUE);
    report(lp, NORMAL, "invert: Refactorization frequency %.1g indicates numeric instability.\n",
                       test);
    lp->spx_status = NUMFAILURE;
  }

  FREE(usedpos);
  return((MYBOOL) (singularities <= 0));
} /* invert */


STATIC MYBOOL fimprove(lprec *lp, REAL *pcol, int *nzidx, REAL roundzero)
{
  REAL   *errors, sdp;
  int    j;
  MYBOOL Ok = TRUE;

  allocREAL(lp, &errors, lp->rows + 1, FALSE);
  if(errors == NULL) {
    Ok = FALSE;
    return(Ok);
  }
  MEMCOPY(errors, pcol, lp->rows + 1);
  lp->bfp_ftran_normal(lp, pcol, nzidx);
  prod_Ax(lp, NULL, pcol, NULL, 0.0, -1,
                                errors, NULL, MAT_ROUNDDEFAULT);
  lp->bfp_ftran_normal(lp, errors, NULL);

  sdp = 0;
  for(j = 1; j <= lp->rows; j++)
    if(fabs(errors[j])>sdp)
      sdp = fabs(errors[j]);
  if(sdp > lp->epsmachine) {
    report(lp, DETAILED, "Iterative FTRAN correction metric %g", sdp);
    for(j = 1; j <= lp->rows; j++) {
      pcol[j] += errors[j];
      my_roundzero(pcol[j], roundzero);
    }
  }
  FREE(errors);
  return(Ok);
}

STATIC MYBOOL bimprove(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero)
{
  int    j;
  REAL   *errors, err, maxerr;
  MYBOOL Ok = TRUE;

  allocREAL(lp, &errors, lp->sum + 1, FALSE);
  if(errors == NULL) {
    Ok = FALSE;
    return(Ok);
  }
  MEMCOPY(errors, rhsvector, lp->sum + 1);

  /* Solve Ax=b for x, compute b back */
  lp->bfp_btran_normal(lp, errors, nzidx);
  prod_xA(lp, NULL, errors, NULL, 0.0, 1.0,
                                  errors, NULL,
                                  MAT_ROUNDDEFAULT);

  /* Take difference with ingoing values, while shifting the column values
     to the rows section and zeroing the columns again */
  for(j = 1; j <= lp->rows; j++)
    errors[j] = errors[lp->rows+lp->var_basic[j]] - rhsvector[j];
  for(j = lp->rows; j <= lp->sum; j++)
    errors[j] = 0;

  /* Solve the b errors for the iterative x adjustment */
  lp->bfp_btran_normal(lp, errors, NULL);

  /* Generate the adjustments and compute statistic */
  maxerr = 0;
  for(j = 1; j <= lp->rows; j++) {
    if(lp->var_basic[j]<=lp->rows) continue;
    err = errors[lp->rows+lp->var_basic[j]];
    if(fabs(err)>maxerr)
      maxerr = fabs(err);
  }
  if(maxerr > lp->epsmachine) {
    report(lp, DETAILED, "Iterative BTRAN correction metric %g", maxerr);
    for(j = 1; j <= lp->rows; j++) {
      if(lp->var_basic[j]<=lp->rows) continue;
      rhsvector[j] += errors[lp->rows+lp->var_basic[j]];
      my_roundzero(rhsvector[j], roundzero);
    }
  }
  FREE(errors);
  return(Ok);
}

STATIC void ftran(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero)
{
#if 0
  if(is_action(lp->improve, IMPROVE_SOLUTION) && lp->bfp_pivotcount(lp))
    fimprove(lp, rhsvector, nzidx, roundzero);
  else
#endif
    lp->bfp_ftran_normal(lp, rhsvector, nzidx);
}

STATIC void btran(lprec *lp, REAL *rhsvector, int *nzidx, REAL roundzero)
{
#if 0
  if(is_action(lp->improve, IMPROVE_SOLUTION) && lp->bfp_pivotcount(lp))
    bimprove(lp, rhsvector, nzidx, roundzero);
  else
#endif
    lp->bfp_btran_normal(lp, rhsvector, nzidx);
}

STATIC MYBOOL fsolve(lprec *lp, int varin, REAL *pcol, int *nzidx, REAL roundzero, REAL ofscalar, MYBOOL prepareupdate)
/* Was setpivcol in versions earlier than 4.0.1.8 - KE */
{
  MYBOOL ok = TRUE;

  if(varin > 0)
    obtain_column(lp, varin, pcol, nzidx, NULL);

 /* Solve, adjusted for objective function scalar */
  pcol[0] *= ofscalar;
  if(prepareupdate)
    lp->bfp_ftran_prepare(lp, pcol, nzidx);
  else
    ftran(lp, pcol, nzidx, roundzero);

  return(ok);

} /* fsolve */


STATIC MYBOOL bsolve(lprec *lp, int row_nr, REAL *rhsvector, int *nzidx, REAL roundzero, REAL ofscalar)
{
  MYBOOL ok = TRUE;

  if(row_nr >= 0) /* Note that row_nr == 0 returns the [1, 0...0 ] vector */
    row_nr = obtain_column(lp, row_nr, rhsvector, nzidx, NULL);

  /* Solve, adjusted for objective function scalar */
  rhsvector[0] *= ofscalar;
  btran(lp, rhsvector, nzidx, roundzero);

  return(ok);

} /* bsolve */


/* Vector compression and expansion routines */
STATIC MYBOOL vec_compress(REAL *densevector, int startpos, int endpos, REAL epsilon,
                           REAL *nzvector, int *nzindex)
{
  int n;

  if((densevector == NULL) || (nzindex == NULL) || (startpos > endpos))
    return( FALSE );

  n = 0;
  densevector += startpos;
  while(startpos <= endpos) {
    if(fabs(*densevector) > epsilon) {  /* Apply zero-threshold */
      if(nzvector != NULL)                       /* Only produce index if no nzvector is given */
        nzvector[n] = *densevector;
      n++;
      nzindex[n] = startpos;
    }
    startpos++;
    densevector++;
  }
  nzindex[0] = n;
  return( TRUE );
}

STATIC MYBOOL vec_expand(REAL *nzvector, int *nzindex, REAL *densevector, int startpos, int endpos)
{
  int i, n;

  n = nzindex[0];
  i = nzindex[n];
  densevector += endpos;
  while(endpos >= startpos) {                     /* Loop from behind to allow densevector == nzvector */
    if(endpos == i) {
      n--;
      *densevector = nzvector[n];
      i = nzindex[n];
    }
    else
      *densevector = 0;
    endpos--;
    densevector--;
  }
  return( TRUE );
}


/* ----------------------------------------------------------------------- */
/* Sparse matrix product routines and utility                              */
/* ----------------------------------------------------------------------- */

STATIC MYBOOL get_colIndexA(lprec *lp, int varset, int *colindex, MYBOOL append)
{
  int      i, varnr, P1extraDim, vb, ve, n, nrows = lp->rows, nsum = lp->sum;
  MYBOOL   omitfixed, omitnonfixed;
  REAL     v;

  /* Find what variable range to scan - default is {SCAN_USERVARS} */
  /* First determine the starting position; add from the top, going down */
  P1extraDim = abs(lp->P1extraDim);
  vb = nrows + 1;
  if(varset & SCAN_ARTIFICIALVARS)
    vb = nsum - P1extraDim + 1;
  if(varset & SCAN_USERVARS)
    vb = nrows + 1;
  if(varset & SCAN_SLACKVARS)
    vb = 1;

  /* Then determine the ending position, add from the bottom, going up */
  ve = nsum;
  if(varset & SCAN_SLACKVARS)
    ve = nrows;
  if(varset & SCAN_USERVARS)
    ve = nsum - P1extraDim;
  if(varset & SCAN_ARTIFICIALVARS)
    ve = nsum;

  /* Adjust for partial pricing */
  if(varset & SCAN_PARTIALBLOCK) {
    SETMAX(vb, partial_blockStart(lp, FALSE));
    SETMIN(ve, partial_blockEnd(lp, FALSE));
  }

  /* Determine exclusion columns */
  omitfixed = (MYBOOL) ((varset & OMIT_FIXED) != 0);
  omitnonfixed = (MYBOOL) ((varset & OMIT_NONFIXED) != 0);
  if(omitfixed && omitnonfixed)
    return(FALSE);

  /* Scan the target colums */
  if(append)
    n = colindex[0];
  else
    n = 0;
  for(varnr = vb; varnr <= ve; varnr++) {

    /* Skip gap in the specified column scan range (possibly user variables) */
    if(varnr > nrows) {
      if((varnr <= nsum-P1extraDim) && !(varset & SCAN_USERVARS))
        continue;
#if 1
      /* Skip empty columns */
      if(/*(lp->P1extraVal == 0) &&*/
         (mat_collength(lp->matA, varnr-nrows) == 0))
        continue;
#endif
    }

    /* Find if the variable is in the scope - default is {Ø} */
    i = lp->is_basic[varnr];
    if((varset & USE_BASICVARS) > 0 && (i))
      ;
    else if((varset & USE_NONBASICVARS) > 0 && (!i))
      ;
    else
      continue;

    v = lp->upbo[varnr];
    if((omitfixed && (v == 0)) ||
       (omitnonfixed && (v != 0)))
      continue;

    /* Append to list */
    n++;
    colindex[n] = varnr;
  }
  colindex[0] = n;

  return(TRUE);
}

STATIC int prod_Ax(lprec *lp, int *coltarget, REAL *input, int *nzinput,
                              REAL roundzero, REAL ofscalar,
                              REAL *output, int *nzoutput, int roundmode)
/* prod_Ax is only used in fimprove; note that it is NOT VALIDATED/verified as of 20030801 - KE */
{
  int      j, colnr, ib, ie, vb, ve;
  MYBOOL   localset, localnz = FALSE, isRC;
  MATrec   *mat = lp->matA;
  REAL     sdp;
  REAL     *value;
  int      *rownr;

  /* Find what variable range to scan - default is {SCAN_USERVARS} */
  /* Define default column target if none was provided */
  isRC = (MYBOOL) ((roundmode & MAT_ROUNDRC) != 0);
  localset = (MYBOOL) (coltarget == NULL);
  if(localset) {
    int varset = SCAN_SLACKVARS | SCAN_USERVARS |
                 USE_BASICVARS | OMIT_FIXED;
    if(isRC && is_piv_mode(lp, PRICE_PARTIAL) && !is_piv_mode(lp, PRICE_FORCEFULL))
      varset |= SCAN_PARTIALBLOCK;
    coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->sum+1, sizeof(*coltarget));
    if(!get_colIndexA(lp, varset, coltarget, FALSE)) {
      mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
      return(FALSE);
    }
  }
  localnz = (MYBOOL) (nzinput == NULL);
  if(localnz) {
    nzinput = (int *) mempool_obtainVector(lp->workarrays, lp->rows+1, sizeof(*nzinput));
    vec_compress(input, 0, lp->rows, lp->matA->epsvalue, NULL, nzinput);
  }

  /* Scan the columns */
  vb = 1;
  ve = coltarget[0];
  for(vb = 1; vb <= coltarget[0]; vb++) {
    colnr = coltarget[vb];
    j = lp->is_basic[colnr];

    /* Perform the multiplication */
    sdp = ofscalar*input[j];
    if(colnr <= lp->rows)               /* A slack variable is in the basis */
      output[colnr] += sdp;
    else {                              /* A normal variable is in the basis */
      colnr -= lp->rows;
      ib = mat->col_end[colnr - 1];
      ie = mat->col_end[colnr];
      rownr = &COL_MAT_ROWNR(ib);
      value = &COL_MAT_VALUE(ib);
      for(; ib < ie;
          ib++, rownr += matRowColStep, value += matValueStep) {
        output[*rownr] += (*value)*sdp;
      }
    }
  }
  roundVector(output+1, lp->rows-1, roundzero);

  /* Clean up and return */
  if(localset)
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
  if(localnz)
    mempool_releaseVector(lp->workarrays, (char *) nzinput, FALSE);

  return(TRUE);
}

STATIC int prod_xA(lprec *lp, int *coltarget,
                              REAL *input, int *nzinput, REAL roundzero, REAL ofscalar,
                              REAL *output, int *nzoutput, int roundmode)
/* Note that the dot product xa is stored at the active column index of A, i.e. of a.
   This means that if the basis only contains non-slack variables, output may point to
   the same vector as input, without overwriting the [0..rows] elements. */
{
  int      colnr, rownr, varnr, ib, ie, vb, ve, nrows = lp->rows;
  MYBOOL   localset, localnz = FALSE, includeOF, isRC;
  REALXP   vmax;
  register REALXP v;
  int      inz, *rowin, countNZ = 0;
  MATrec   *mat = lp->matA;
  register REAL     *matValue;
  register int      *matRownr;

  /* Clean output area (only necessary if we are returning the full vector) */
  isRC = (MYBOOL) ((roundmode & MAT_ROUNDRC) != 0);
  if(nzoutput == NULL) {
    if(input == output)
      MEMCLEAR(output+nrows+1, lp->columns);
    else
      MEMCLEAR(output, lp->sum+1);
  }

  /* Find what variable range to scan - default is {SCAN_USERVARS} */
  /* Define default column target if none was provided */
  localset = (MYBOOL) (coltarget == NULL);
  if(localset) {
    int varset = SCAN_SLACKVARS | SCAN_USERVARS |
                 USE_NONBASICVARS | OMIT_FIXED;
    if(isRC && is_piv_mode(lp, PRICE_PARTIAL) && !is_piv_mode(lp, PRICE_FORCEFULL))
      varset |= SCAN_PARTIALBLOCK;
    coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->sum+1, sizeof(*coltarget));
    if(!get_colIndexA(lp, varset, coltarget, FALSE)) {
      mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
      return(FALSE);
    }
  }
/*#define UseLocalNZ*/
#ifdef UseLocalNZ
  localnz = (MYBOOL) (nzinput == NULL);
  if(localnz) {
    nzinput = (int *) mempool_obtainVector(lp->workarrays, nrows+1, sizeof(*nzinput));
    vec_compress(input, 0, nrows, lp->matA->epsvalue, NULL, nzinput);
  }
#endif
  includeOF = (MYBOOL) (((nzinput == NULL) || (nzinput[1] == 0)) &&
                        (input[0] != 0) && lp->obj_in_basis);

  /* Scan the target colums */
  vmax = 0;
  ve = coltarget[0];
  for(vb = 1; vb <= ve; vb++) {

    varnr = coltarget[vb];

    if(varnr <= nrows) {
      v = input[varnr];
    }
    else {
      colnr = varnr - nrows;
      v = 0;
      ib = mat->col_end[colnr - 1];
      ie = mat->col_end[colnr];
      if(ib < ie) {

        /* Do dense input vector version */
#ifdef UseLocalNZ
        if(localnz || (nzinput == NULL)) {
#else
        if(nzinput == NULL) {
#endif
          /* Do the OF */
          if(includeOF)
#ifdef DirectArrayOF
            v += input[0] * lp->obj[colnr] * ofscalar;
#else
            v += input[0] * get_OF_active(lp, varnr, ofscalar);
#endif

          /* Initialize pointers */
          matRownr = &COL_MAT_ROWNR(ib);
          matValue = &COL_MAT_VALUE(ib);

          /* Do extra loop optimization based on target window overlaps */
#ifdef UseLocalNZ
          if((ib < ie)
             && (colnr <= *nzinput)
             && (COL_MAT_ROWNR(ie-1) >= nzinput[colnr])
             && (*matRownr <= nzinput[*nzinput])
             )
#endif
#ifdef NoLoopUnroll
          /* Then loop over all regular rows */
          for(; ib < ie; ib++) {
            v += input[*matRownr] * (*matValue);
            matValue += matValueStep;
            matRownr += matRowColStep;
          }
#else
          /* Prepare for simple loop unrolling */
          if(((ie-ib) % 2) == 1) {
            v += input[*matRownr] * (*matValue);
            ib++;
            matValue += matValueStep;
            matRownr += matRowColStep;
          }

          /* Then loop over remaining pairs of regular rows */
          while(ib < ie) {
            v += input[*matRownr] * (*matValue);
            v += input[*(matRownr+matRowColStep)] * (*(matValue+matValueStep));
            ib += 2;
            matValue += 2*matValueStep;
            matRownr += 2*matRowColStep;
          }
#endif
        }
        /* Do sparse input vector version */
        else {

          /* Do the OF */
          if(includeOF)
#ifdef DirectArrayOF
            v += input[0] * lp->obj[colnr] * ofscalar;
#else
            v += input[0] * get_OF_active(lp, varnr, ofscalar);
#endif

          /* Initialize pointers */
          inz = 1;
          rowin = nzinput+inz;
          matRownr = &COL_MAT_ROWNR(ib);
          matValue = &COL_MAT_VALUE(ib);
          ie--;

          /* Then loop over all non-OF rows */
          while((inz <= *nzinput) && (ib <= ie)) {

           /* Try to synchronize at right */
            while((*rowin > *matRownr) && (ib < ie)) {
              ib++;
              matValue += matValueStep;
              matRownr += matRowColStep;
            }
            /* Try to synchronize at left */
            while((*rowin < *matRownr) && (inz < *nzinput)) {
              inz++;
              rowin++;
            }
            /* Perform dot product operation if there was a match */
            if(*rowin == *matRownr) {
              v += input[*rowin] * (*matValue);
              /* Step forward at left */
              inz++;
              rowin++;
            }
          }
        }
      }
      if((roundmode & MAT_ROUNDABS) != 0) {
        my_roundzero(v, roundzero);
      }
    }

    /* Special handling of small reduced cost values */
    if(!isRC || (my_chsign(lp->is_lower[varnr], v) < 0)) {
      SETMAX(vmax, fabs((REAL) v));
    }
    if(v != 0) {
      countNZ++;
      if(nzoutput != NULL)
        nzoutput[countNZ] = varnr;
    }
    output[varnr] = (REAL) v;
  }

  /* Compute reduced cost if this option is active */
  if(isRC && !lp->obj_in_basis)
    countNZ = get_basisOF(lp, coltarget, output, nzoutput);

  /* Check if we should do relative rounding */
  if((roundmode & MAT_ROUNDREL) != 0) {
    if((roundzero > 0) && (nzoutput != NULL)) {
      ie = 0;
      if(isRC) {
        SETMAX(vmax, MAT_ROUNDRCMIN);  /* Make sure we don't use very small values */
      }
      vmax *= roundzero;
      for(ib = 1; ib <= countNZ;  ib++) {
        rownr = nzoutput[ib];
        if(fabs(output[rownr]) < vmax)
          output[rownr] = 0;
        else {
          ie++;
          nzoutput[ie] = rownr;
        }
      }
      countNZ = ie;
    }
  }

  /* Clean up and return */
  if(localset)
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
  if(localnz)
    mempool_releaseVector(lp->workarrays, (char *) nzinput, FALSE);

  if(nzoutput != NULL)
    *nzoutput = countNZ;
  return(countNZ);
}

STATIC MYBOOL prod_xA2(lprec *lp, int *coltarget,
                                  REAL *prow, REAL proundzero, int *nzprow,
                                  REAL *drow, REAL droundzero, int *nzdrow,
                                  REAL ofscalar, int roundmode)
{
  int      varnr, colnr, ib, ie, vb, ve, nrows = lp->rows;
  MYBOOL   includeOF, isRC;
  REALXP   dmax, pmax;
  register REALXP d, p;
  MATrec   *mat = lp->matA;
  REAL     value;
  register REAL     *matValue;
  register int      *matRownr;
  MYBOOL localset;

  /* Find what variable range to scan - default is {SCAN_USERVARS} */
  /* First determine the starting position; add from the top, going down */
  localset = (MYBOOL) (coltarget == NULL);
  if(localset) {
    int varset = SCAN_SLACKVARS + SCAN_USERVARS + /*SCAN_ALLVARS +*/
                 /*SCAN_PARTIALBLOCK+*/
                 USE_NONBASICVARS+OMIT_FIXED;
    coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->sum+1, sizeof(*coltarget));
    if(!get_colIndexA(lp, varset, coltarget, FALSE)) {
      mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
      return(FALSE);
    }
  }

  /* Initialize variables */
  isRC = (MYBOOL) ((roundmode & MAT_ROUNDRC) != 0);
  pmax = 0;
  dmax = 0;
  if(nzprow != NULL)
    *nzprow = 0;
  if(nzdrow != NULL)
    *nzdrow = 0;
  includeOF = (MYBOOL) (((prow[0] != 0) || (drow[0] != 0)) &&
                        lp->obj_in_basis);

  /* Scan the target colums */
  ve = coltarget[0];
  for(vb = 1; vb <= ve; vb++) {

    varnr = coltarget[vb];

    if(varnr <= nrows) {
      p = prow[varnr];
      d = drow[varnr];
    }
    else {

      colnr = varnr - nrows;

      p = 0;
      d = 0;
      ib = mat->col_end[colnr - 1];
      ie = mat->col_end[colnr];

      if(ib < ie) {

        /* Do the OF */
        if(includeOF) {
#ifdef DirectArrayOF
          value = lp->obj[colnr] * ofscalar;
#else
          value = get_OF_active(lp, varnr, ofscalar);
#endif
          p += prow[0] * value;
          d += drow[0] * value;
        }

        /* Then loop over all regular rows */
        matRownr = &COL_MAT_ROWNR(ib);
        matValue = &COL_MAT_VALUE(ib);
#ifdef NoLoopUnroll
        for( ; ib < ie; ib++) {
          p += prow[*matRownr] * (*matValue);
          d += drow[*matRownr] * (*matValue);
          matValue += matValueStep;
          matRownr += matRowColStep;
        }
#else
        /* Prepare for simple loop unrolling */
        if(((ie-ib) % 2) == 1) {
          p += prow[*matRownr] * (*matValue);
          d += drow[*matRownr] * (*matValue);
          ib++;
          matValue += matValueStep;
          matRownr += matRowColStep;
        }

        /* Then loop over remaining pairs of regular rows */
        while(ib < ie) {
          p += prow[*matRownr] * (*matValue);
          p += prow[*(matRownr+matRowColStep)] * (*(matValue+matValueStep));
          d += drow[*matRownr] * (*matValue);
          d += drow[*(matRownr+matRowColStep)] * (*(matValue+matValueStep));
          ib += 2;
          matValue += 2*matValueStep;
          matRownr += 2*matRowColStep;
        }
#endif

      }
      if((roundmode & MAT_ROUNDABS) != 0) {
        my_roundzero(p, proundzero);
        my_roundzero(d, droundzero);
      }
    }

    SETMAX(pmax, fabs((REAL) p));
    prow[varnr] = (REAL) p;
    if((nzprow != NULL) && (p != 0)) {
      (*nzprow)++;
      nzprow[*nzprow] = varnr;
    }

    /* Special handling of reduced cost rounding */
    if(!isRC || (my_chsign(lp->is_lower[varnr], d) < 0)) {
      SETMAX(dmax, fabs((REAL) d));
    }
    drow[varnr] = (REAL) d;
    if((nzdrow != NULL) && (d != 0)) {
      (*nzdrow)++;
      nzdrow[*nzdrow] = varnr;
    }
  }

  /* Compute reduced cost here if this option is active */
  if((drow != 0) && !lp->obj_in_basis)
    get_basisOF(lp, coltarget, drow, nzdrow);

  /* Check if we should do relative rounding */
  if((roundmode & MAT_ROUNDREL) != 0) {
    if((proundzero > 0) && (nzprow != NULL)) {
      ie = 0;
      pmax *= proundzero;
      for(ib = 1; ib <= *nzprow;  ib++) {
        varnr = nzprow[ib];
        if(fabs(prow[varnr]) < pmax)
          prow[varnr] = 0;
        else {
          ie++;
          nzprow[ie] = varnr;
        }
      }
      *nzprow = ie;
    }
    if((droundzero > 0) && (nzdrow != NULL)) {
      ie = 0;
      if(isRC) {
        SETMAX(dmax, MAT_ROUNDRCMIN);  /* Make sure we don't use very small values */
      }
      dmax *= droundzero;
      for(ib = 1; ib <= *nzdrow;  ib++) {
        varnr = nzdrow[ib];
        if(fabs(drow[varnr]) < dmax)
          drow[varnr] = 0;
        else {
          ie++;
          nzdrow[ie] = varnr;
        }
      }
      *nzdrow = ie;
    }
  }

  /* Clean up and return */
  if(localset)
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
  return( TRUE );
}

STATIC void bsolve_xA2(lprec *lp, int* coltarget,
                                  int row_nr1, REAL *vector1, REAL roundzero1, int *nzvector1,
                                  int row_nr2, REAL *vector2, REAL roundzero2, int *nzvector2, int roundmode)
{
  REAL ofscalar = 1.0;

 /* Clear and initialize first vector */
  if(nzvector1 == NULL)
    MEMCLEAR(vector1, lp->sum + 1);
  else
    MEMCLEAR(vector1, lp->rows + 1);
  vector1[row_nr1] = 1;
/*  workINT[0] = 1;
  workINT[1] = row_nr1; */

  if(vector2 == NULL) {
    lp->bfp_btran_normal(lp, vector1, NULL);
    prod_xA(lp, coltarget, vector1, NULL, roundzero1, ofscalar*0,
                           vector1, nzvector1, roundmode);
  }
  else {

   /* Clear and initialize second vector */
    if(nzvector2 == NULL)
      MEMCLEAR(vector2, lp->sum + 1);
    else
      MEMCLEAR(vector2, lp->rows + 1);
    if(lp->obj_in_basis || (row_nr2 > 0)) {
      vector2[row_nr2] = 1;
/*      workINT[2] = 1;
      workINT[3] = row_nr2; */
    }
    else
      get_basisOF(lp, NULL, vector2, nzvector2);

   /* A double BTRAN equation solver process is implemented "in-line" below in
      order to save time and to implement different rounding for the two */
    lp->bfp_btran_double(lp, vector1, NULL, vector2, NULL);

   /* Multiply solution vectors with matrix values */
    prod_xA2(lp, coltarget, vector1, roundzero1, nzvector1,
                            vector2, roundzero2, nzvector2,
                            ofscalar, roundmode);
  }
}

