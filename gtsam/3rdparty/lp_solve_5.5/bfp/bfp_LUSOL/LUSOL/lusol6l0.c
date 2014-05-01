
/* Create a row-based version of L0.
   This makes it possible to solve L0'x=h (btran) faster for sparse h,
   since we only run down the columns of L0' (rows of LO) for which
   the corresponding entry in h is non-zero. */
MYBOOL LU1L0(LUSOLrec *LUSOL, LUSOLmat **mat, int *inform)
{
  MYBOOL status = FALSE;
  int    K, L, LL, L1, L2, LENL0, NUML0, I;
  int    *lsumr;

  /* Assume success */
  *inform = LUSOL_INFORM_LUSUCCESS;

  /* Check if there is anything worth doing */
  if(mat == NULL)
    return( status );
  if(*mat != NULL)
    LUSOL_matfree(mat);
  NUML0 = LUSOL->luparm[LUSOL_IP_COLCOUNT_L0];
  LENL0 = LUSOL->luparm[LUSOL_IP_NONZEROS_L0];
  if((NUML0 == 0) || (LENL0 == 0) ||
     (LUSOL->luparm[LUSOL_IP_ACCELERATION] == LUSOL_BASEORDER) ||
     ((LUSOL->luparm[LUSOL_IP_ACCELERATION] & LUSOL_ACCELERATE_L0) == 0))
    return( status );

  /* Allocate temporary array */
  lsumr = (int *) LUSOL_CALLOC((LUSOL->m+1), sizeof(*lsumr));
  if(lsumr == NULL) {
    *inform = LUSOL_INFORM_NOMEMLEFT;
    return( status );
  }

  /* Compute non-zero counts by permuted row index (order is unimportant) */
  K = 0;
  L2 = LUSOL->lena;
  L1 = L2-LENL0+1;
  for(L = L1; L <= L2; L++) {
    I = LUSOL->indc[L];
    lsumr[I]++;
    if(lsumr[I] == 1)
      K++;
  }
  LUSOL->luparm[LUSOL_IP_ROWCOUNT_L0] = K;

  /* Check if we should apply "smarts" before proceeding to the row matrix creation */
  if((LUSOL->luparm[LUSOL_IP_ACCELERATION] & LUSOL_AUTOORDER) &&
     ((REAL) LUSOL->luparm[LUSOL_IP_ROWCOUNT_L0] /
#if 0
             LUSOL->luparm[LUSOL_IP_COLCOUNT_L0]
#else
             LUSOL->m
#endif
      > LUSOL->parmlu[LUSOL_RP_SMARTRATIO]))
    goto Finish;

  /* We are Ok to create the new matrix object */
  *mat = LUSOL_matcreate(LUSOL->m, LENL0);
  if(*mat == NULL) {
    *inform = LUSOL_INFORM_NOMEMLEFT;
    goto Finish;
  }

  /* Cumulate row counts to get vector offsets; first row is leftmost
     (stick with Fortran array offset for consistency) */
  (*mat)->lenx[0] = 1;
  for(K = 1; K <= LUSOL->m; K++) {
    (*mat)->lenx[K] = (*mat)->lenx[K-1] + lsumr[K];
    lsumr[K] = (*mat)->lenx[K-1];
  }

  /* Map the matrix into row order by permuted index;
     Note: The first permuted row is located leftmost in the array.
           The column order is irrelevant, since the indeces will
           refer to constant / resolved values of V[] during solve. */
  L2 = LUSOL->lena;
  L1 = L2-LENL0+1;
  for(L = L1; L <= L2; L++) {
    I = LUSOL->indc[L];
    LL = lsumr[I]++;
    (*mat)->a[LL] = LUSOL->a[L];
    (*mat)->indr[LL] = LUSOL->indr[L];
    (*mat)->indc[LL] = I;
  }

  /* Pack row starting positions, and set mapper from original index to packed */
  I = 0;
  for(L = 1; L <= LUSOL->m; L++) {
    K = LUSOL->ip[L];
    if((*mat)->lenx[K] > (*mat)->lenx[K-1]) {
      I++;
      (*mat)->indx[I] = K;
    }
  }

  /* Confirm that everything went well */
  status = TRUE;

  /* Clean up */
Finish:
  FREE(lsumr);
  return( status );
}

/* Solve L0' v = v based on row-based version of L0, constructed by LU1L0 */
void LU6L0T_v(LUSOLrec *LUSOL, LUSOLmat *mat, REAL V[], int NZidx[], int *INFORM)
{
#ifdef DoTraceL0
  REAL TEMP;
#endif
  int  LEN, K, KK, L, L1, NUML0;
  REAL SMALL;
  register REAL VPIV;
#if (defined LUSOLFastSolve) && !(defined DoTraceL0)
  REAL *aptr;
  int  *jptr;
#else
  int  J;
#endif

  NUML0 = LUSOL->luparm[LUSOL_IP_ROWCOUNT_L0];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];

  /* Loop over the nz columns of L0' - from the end, going forward. */
  for(K = NUML0; K > 0; K--) {
    KK = mat->indx[K];
    L  = mat->lenx[KK];
    L1 = mat->lenx[KK-1];
    LEN = L - L1;
    if(LEN == 0)
      continue;
    /* Get value of the corresponding active entry of V[] */
    VPIV = V[KK];
    /* Only process the column of L0' if the value of V[] is non-zero */
    if(fabs(VPIV)>SMALL) {
/*     ***** This loop could be coded specially. */
#if (defined LUSOLFastSolve) && !(defined DoTraceL0)
      L--;
      for(aptr = mat->a+L, jptr = mat->indr+L;
          LEN > 0; LEN--, aptr--, jptr--)
        V[*jptr] += VPIV * (*aptr);
#else
      for(; LEN > 0; LEN--) {
        L--;
        J = mat->indr[L];
#ifndef DoTraceL0
        V[J] += VPIV * mat->a[L];
#else
        TEMP = V[J];
        V[J] += VPIV * mat->a[L];
        printf("V[%3d] = V[%3d] + L[%d,%d]*V[%3d]\n", J, J, KK,J, KK);
        printf("%6g = %6g + %6g*%6g\n", V[J], TEMP, mat->a[L], VPIV);
#endif
      }
#endif
    }
#ifdef SetSmallToZero
    else
      V[KK] = 0;
#endif
  }

}
