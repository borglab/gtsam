
/* Create a column-based version of U.
   This makes it possible to solve Ux=h (ftran) faster for sparse h,
   since we only run down the rows of U (columns of U') for which
   the corresponding entry in h is non-zero. */
MYBOOL LU1U0(LUSOLrec *LUSOL, LUSOLmat **mat, int *inform)
{
  MYBOOL status = FALSE;
  int    K, L, LL, LENU, NUMU, J;
  int    *lsumc;

  /* Assume success */
  *inform = LUSOL_INFORM_LUSUCCESS;

  /* Check if there is anything worth doing */
  if(mat == NULL)
    return( status );
  if(*mat != NULL)
    LUSOL_matfree(mat);
  NUMU = LUSOL->luparm[LUSOL_IP_RANK_U];
  LENU = LUSOL->luparm[LUSOL_IP_NONZEROS_U];
  if((NUMU == 0) || (LENU == 0) ||
     (LUSOL->luparm[LUSOL_IP_ACCELERATION] == LUSOL_BASEORDER) ||
     ((LUSOL->luparm[LUSOL_IP_ACCELERATION] & LUSOL_ACCELERATE_U) == 0))
    return( status );

  /* Allocate temporary array */
  lsumc = (int *) LUSOL_CALLOC((LUSOL->n+1), sizeof(*lsumc));
  if(lsumc == NULL) {
    *inform = LUSOL_INFORM_NOMEMLEFT;
    return( status );
  }

  /* Compute non-zero counts by permuted column index (order is unimportant) */
  for(L = 1; L <= LENU; L++) {
    J = LUSOL->indr[L];
    lsumc[J]++;
  }

  /* Check if we should apply "smarts" before proceeding to the column matrix creation */
  if((LUSOL->luparm[LUSOL_IP_ACCELERATION] & LUSOL_AUTOORDER) &&
     ((REAL) sqrt((REAL) NUMU/LENU) > LUSOL->parmlu[LUSOL_RP_SMARTRATIO]))
    goto Finish;

  /* We are Ok to create the new matrix object */
  *mat = LUSOL_matcreate(LUSOL->n, LENU);
  if(*mat == NULL) {
    *inform = LUSOL_INFORM_NOMEMLEFT;
    goto Finish;
  }

  /* Cumulate row counts to get vector offsets; first column is leftmost
     (stick with Fortran array offset for consistency) */
  (*mat)->lenx[0] = 1;
  for(K = 1; K <= LUSOL->n; K++) {
    (*mat)->lenx[K] = (*mat)->lenx[K-1] + lsumc[K];
    lsumc[K] = (*mat)->lenx[K-1];
  }

  /* Map the matrix into column order by permuted index;
     Note: The first permuted column is located leftmost in the array.
           The row order is irrelevant, since the indeces will
           refer to constant / resolved values of V[] during solve. */
  for(L = 1; L <= LENU; L++) {
    J = LUSOL->indr[L];
    LL = lsumc[J]++;
    (*mat)->a[LL] = LUSOL->a[L];
    (*mat)->indr[LL] = J;
    (*mat)->indc[LL] = LUSOL->indc[L];
  }

  /* Pack column starting positions, and set mapper from original index to packed */
  J = 0;
  for(L = 1; L <= LUSOL->n; L++) {
    K = LUSOL->iq[L];
#if 1  /* Deactivate to produce a full-rank version (implicit unit diagonals) */
    if((*mat)->lenx[K] > (*mat)->lenx[K-1])
#endif
    {
      J++;
     (*mat)->indx[J] = K;
    }
  }

  /* Confirm that everything went well */
  status = TRUE;

  /* Clean up */
Finish:
  FREE(lsumc);
  return( status );
}

/* Solve U w = v based on column-based version of U, constructed by LU1U0 */
void LU6U0_v(LUSOLrec *LUSOL, LUSOLmat *mat, REAL V[], REAL W[], int NZidx[], int *INFORM)
{
#ifdef DoTraceU0
  REAL TEMP;
#endif
  int  LEN, I, K, L, L1, NRANK, NRANK1, KLAST;
  REAL SMALL;
  register REAL T;
#if (defined xxLUSOLFastSolve) && !(defined DoTraceU0)
  REAL *aptr;
  int  *jptr;
#else
  int  J;
#endif

  NRANK = LUSOL->luparm[LUSOL_IP_RANK_U];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  NRANK1 = NRANK+1;
/*      Find the first nonzero in v(1:nrank), counting backwards. */
  for(KLAST = NRANK; KLAST >= 1; KLAST--) {
    I = LUSOL->ip[KLAST];
    if(fabs(V[I])>SMALL)
      break;
  }
  L = LUSOL->n;
#ifdef xxLUSOLFastSolve
  for(K = KLAST+1, jptr = LUSOL->iq+K; K <= L; K++, jptr++)
    W[*jptr] = ZERO;
#else
  for(K = KLAST+1; K <= L; K++) {
    J = LUSOL->iq[K];
    W[J] = ZERO;
  }
#endif
  /* Loop over the nz columns of U - from the right, going left. */
  for(K = NRANK; K > 0; K--) {
    I = mat->indx[K];
    L = mat->lenx[I];
    L1 = mat->lenx[I-1];
    LEN = L - L1;
    T = V[I];
    if(fabs(T)<=SMALL) {
      W[K] = ZERO;
      continue;
    }
    T /= mat->a[L1];  /* Should it be L or L1 ? */
    W[K] = T;
    LEN--;
/*     ***** This loop could be coded specially. */
#ifdef xxLUSOLFastSolve
    L--;
    for(aptr = mat->a+L, jptr = mat->indc+L;
        LEN > 0; LEN--, aptr--, jptr--)
      V[*jptr] -= T * (*aptr);
#else
    for(; LEN > 0; LEN--) {
      L--;
      J = mat->indc[L];
#ifndef DoTraceL0
      V[J] -= T * mat->a[L];
#else
      TEMP = V[J];
      V[J] += T * mat->a[L];
      printf("V[%3d] = V[%3d] + L[%d,%d]*V[%3d]\n", J, J, I,J, I);
      printf("%6g = %6g + %6g*%6g\n", V[J], TEMP, mat->a[L], T);
#endif
    }
#endif
  }
/*      Compute residual for overdetermined systems. */
  T = ZERO;
  for(K = NRANK1; K <= LUSOL->m; K++) {
    I = LUSOL->ip[K];
    T += fabs(V[I]);
  }
/*      Exit. */
  if(T>ZERO)
    *INFORM = LUSOL_INFORM_LUSINGULAR;
  LUSOL->luparm[LUSOL_IP_INFORM]     = *INFORM;
  LUSOL->parmlu[LUSOL_RP_RESIDUAL_U] = (REAL) T;
}
