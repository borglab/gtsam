
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   File  lusol6a
      lu6sol   lu6L     lu6Lt     lu6U     Lu6Ut   lu6LD   lu6chk
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   26 Apr 2002: lu6 routines put into a separate file.
   15 Dec 2002: lu6sol modularized via lu6L, lu6Lt, lu6U, lu6Ut.
                lu6LD implemented to allow solves with LDL' or L|D|L'.
   15 Dec 2002: Current version of lusol6a.f.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* ==================================================================
   lu6chk  looks at the LU factorization  A = L*U.
   If mode = 1, lu6chk is being called by lu1fac.
   (Other modes not yet implemented.)
   ------------------------------------------------------------------
   The important input parameters are

                  lprint = luparm(2)
                  keepLU = luparm(8)
                  Utol1  = parmlu(4)
                  Utol2  = parmlu(5)

   and the significant output parameters are

                  inform = luparm(10)
                  nsing  = luparm(11)
                  jsing  = luparm(12)
                  jumin  = luparm(19)
                  Lmax   = parmlu(11)
                  Umax   = parmlu(12)
                  DUmax  = parmlu(13)
                  DUmin  = parmlu(14)
                  and      w(*).

   Lmax  and Umax  return the largest elements in L and U.
   DUmax and DUmin return the largest and smallest diagonals of U
                   (excluding diagonals that are exactly zero).
   In general, w(j) is set to the maximum absolute element in
   the j-th column of U.  However, if the corresponding diagonal
   of U is small in absolute terms or relative to w(j)
   (as judged by the parameters Utol1, Utol2 respectively),
   then w(j) is changed to - w(j).
   Thus, if w(j) is not positive, the j-th column of A
   appears to be dependent on the other columns of A.
   The number of such columns, and the position of the last one,
   are returned as nsing and jsing.
   Note that nrank is assumed to be set already, and is not altered.
   Typically, nsing will satisfy      nrank + nsing = n,  but if
   Utol1 and Utol2 are rather large,  nsing > n - nrank   may occur.
   If keepLU = 0,
   Lmax  and Umax  are already set by lu1fac.
   The diagonals of U are in the top of A.
   Only Utol1 is used in the singularity test to set w(*).
   inform = 0  if  A  appears to have full column rank  (nsing = 0).
   inform = 1  otherwise  (nsing .gt. 0).
   ------------------------------------------------------------------
   00 Jul 1987: Early version.
   09 May 1988: f77 version.
   11 Mar 2001: Allow for keepLU = 0.
   17 Nov 2001: Briefer output for singular factors.
   05 May 2002: Comma needed in format 1100 (via Kenneth Holmstrom).
   06 May 2002: With keepLU = 0, diags of U are in natural order.
                They were not being extracted correctly.
   23 Apr 2004: TRP can judge singularity better by comparing
                all diagonals to DUmax.
   27 Jun 2004: (PEG) Allow write only if nout .gt. 0.
   ================================================================== */
#ifdef UseOld_LU6CHK_20040510
void LU6CHK(LUSOLrec *LUSOL, int MODE, int LENA2, int *INFORM)
{
  MYBOOL KEEPLU;
  int    I, J, JUMIN, K, L, L1, L2, LENL, LPRINT, NDEFIC, NRANK;
  REAL   AIJ, DIAG, DUMAX, DUMIN, LMAX, UMAX, UTOL1, UTOL2;

  LPRINT = LUSOL->luparm[LUSOL_IP_PRINTLEVEL];
  KEEPLU = (MYBOOL) (LUSOL->luparm[LUSOL_IP_KEEPLU]!=0);
  NRANK = LUSOL->luparm[LUSOL_IP_RANK_U];
  LENL  = LUSOL->luparm[LUSOL_IP_NONZEROS_L];
  UTOL1 = LUSOL->parmlu[LUSOL_RP_SMALLDIAG_U];
  UTOL2 = LUSOL->parmlu[LUSOL_RP_EPSDIAG_U];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  LMAX  = ZERO;
  UMAX  = ZERO;
  LUSOL->luparm[LUSOL_IP_SINGULARITIES]  = 0;
  LUSOL->luparm[LUSOL_IP_SINGULARINDEX]  = 0;
  JUMIN = 0;
  DUMAX = ZERO;
  DUMIN = LUSOL_BIGNUM;

#ifdef LUSOLFastClear
  MEMCLEAR(LUSOL->w+1, LUSOL->n);
#else
  for(I = 1; I <= LUSOL->n; I++)
    LUSOL->w[I] = ZERO;
#endif

  if(KEEPLU) {
/*     --------------------------------------------------------------
        Find  Lmax.
       -------------------------------------------------------------- */
    for(L = (LENA2+1)-LENL; L <= LENA2; L++) {
      SETMAX(LMAX,fabs(LUSOL->a[L]));
    }
/*     --------------------------------------------------------------
        Find Umax and set w(j) = maximum element in j-th column of U.
       -------------------------------------------------------------- */
    for(K = 1; K <= NRANK; K++) {
      I = LUSOL->ip[K];
      L1 = LUSOL->locr[I];
      L2 = (L1+LUSOL->lenr[I])-1;
      for(L = L1; L <= L2; L++) {
        J = LUSOL->indr[L];
        AIJ = fabs(LUSOL->a[L]);
        SETMAX(LUSOL->w[J],AIJ);
        SETMAX(UMAX,AIJ);
      }
    }
/*     --------------------------------------------------------------
        Negate w(j) if the corresponding diagonal of U is
        too small in absolute terms or relative to the other elements
        in the same column of  U.
        Also find DUmax and DUmin, the extreme diagonals of U.
       -------------------------------------------------------------- */
    for(K = 1; K <= LUSOL->n; K++) {
      J = LUSOL->iq[K];
      if(K>NRANK)
        DIAG = ZERO;
      else {
        I = LUSOL->ip[K];
        L1 = LUSOL->locr[I];
        DIAG = fabs(LUSOL->a[L1]);
        SETMAX(DUMAX,DIAG);
        if(DUMIN>DIAG) {
          DUMIN = DIAG;
          JUMIN = J;
        }
      }
      if((DIAG<=UTOL1) || (DIAG<=UTOL2*LUSOL->w[J])) {
        LUSOL_addSingularity(LUSOL, J, INFORM);
        LUSOL->w[J] = -LUSOL->w[J];
      }
    }
    LUSOL->parmlu[LUSOL_RP_MAXMULT_L] = LMAX;
    LUSOL->parmlu[LUSOL_RP_MAXELEM_U] = UMAX;
  }
   else {
/*     --------------------------------------------------------------
        keepLU = 0.
        Only diag(U) is stored.  Set w(*) accordingly.
       -------------------------------------------------------------- */
    for(K = 1; K <= LUSOL->n; K++) {
      J = LUSOL->iq[K];
      if(K>NRANK)
        DIAG = ZERO;
      else {
/* !             diag   = abs( diagU(k) ) ! 06 May 2002: Diags are in natural order */
        DIAG = fabs(LUSOL->diagU[J]);
        LUSOL->w[J] = DIAG;
        SETMAX(DUMAX,DIAG);
        if(DUMIN>DIAG) {
          DUMIN = DIAG;
          JUMIN = J;
        }
      }
      if(DIAG<=UTOL1) {
        LUSOL_addSingularity(LUSOL, J, INFORM);
        LUSOL->w[J] = -LUSOL->w[J];
      }
    }
  }
/*     -----------------------------------------------------------------
        Set output parameters.
       ----------------------------------------------------------------- */
  if(JUMIN==0)
    DUMIN = ZERO;
  LUSOL->luparm[LUSOL_IP_COLINDEX_DUMIN] = JUMIN;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_DIAGU]  = DUMAX;
  LUSOL->parmlu[LUSOL_RP_MINELEM_DIAGU]  = DUMIN;
/*      The matrix has been judged singular. */
  if(LUSOL->luparm[LUSOL_IP_SINGULARITIES]>0) {
    *INFORM = LUSOL_INFORM_LUSINGULAR;
    NDEFIC = LUSOL->n-NRANK;
    if(LPRINT>=LUSOL_MSG_SINGULARITY) {
      LUSOL_report(LUSOL, 0, "Singular(m%cn)  rank:%9d  n-rank:%8d  nsing:%9d\n",
                             relationChar(LUSOL->m, LUSOL->n),NRANK,NDEFIC,
                             LUSOL->luparm[LUSOL_IP_SINGULARITIES]);
    }
  }
/*      Exit. */
  LUSOL->luparm[LUSOL_IP_INFORM] = *INFORM;
}
#else
void LU6CHK(LUSOLrec *LUSOL, int MODE, int LENA2, int *INFORM)
{
  MYBOOL KEEPLU, TRP;
  int    I, J, JUMIN, K, L, L1, L2, LENL, LDIAGU, LPRINT, NDEFIC, NRANK;
  REAL   AIJ, DIAG, DUMAX, DUMIN, LMAX, UMAX, UTOL1, UTOL2;

  LPRINT = LUSOL->luparm[LUSOL_IP_PRINTLEVEL];
  KEEPLU = (MYBOOL) (LUSOL->luparm[LUSOL_IP_KEEPLU] != 0);
  TRP    = (MYBOOL) (LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TRP);
  NRANK  = LUSOL->luparm[LUSOL_IP_RANK_U];
  LENL   = LUSOL->luparm[LUSOL_IP_NONZEROS_L];
  UTOL1  = LUSOL->parmlu[LUSOL_RP_SMALLDIAG_U];
  UTOL2  = LUSOL->parmlu[LUSOL_RP_EPSDIAG_U];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  LMAX   = ZERO;
  UMAX   = ZERO;
  LUSOL->luparm[LUSOL_IP_SINGULARITIES] = 0;
  LUSOL->luparm[LUSOL_IP_SINGULARINDEX] = 0;
  JUMIN  = 0;
  DUMAX  = ZERO;
  DUMIN  = LUSOL_BIGNUM;

#ifdef LUSOLFastClear
  MEMCLEAR(LUSOL->w+1, LUSOL->n);
#else
  for(I = 1; I <= LUSOL->n; I++)
    LUSOL->w[I] = ZERO;
#endif

  if(KEEPLU) {
/*     --------------------------------------------------------------
        Find  Lmax.
       -------------------------------------------------------------- */
    for(L = (LENA2+1)-LENL; L <= LENA2; L++) {
      SETMAX(LMAX,fabs(LUSOL->a[L]));
     }
/*     --------------------------------------------------------------
        Find Umax and set w(j) = maximum element in j-th column of U.
       -------------------------------------------------------------- */
    for(K = 1; K <= NRANK; K++) {
      I = LUSOL->ip[K];
      L1 = LUSOL->locr[I];
      L2 = (L1+LUSOL->lenr[I])-1;
      for(L = L1; L <= L2; L++) {
        J = LUSOL->indr[L];
        AIJ = fabs(LUSOL->a[L]);
        SETMAX(LUSOL->w[J],AIJ);
        SETMAX(UMAX,AIJ);
      }
    }
    LUSOL->parmlu[LUSOL_RP_MAXMULT_L] = LMAX;
    LUSOL->parmlu[LUSOL_RP_MAXELEM_U] = UMAX;
/*     --------------------------------------------------------------
       Find DUmax and DUmin, the extreme diagonals of U.
       -------------------------------------------------------------- */
    for(K = 1; K <= NRANK; K++) {
      J     = LUSOL->iq[K];
      I     = LUSOL->ip[K];
      L1    = LUSOL->locr[I];
      DIAG  = fabs(LUSOL->a[L1]);
      SETMAX( DUMAX, DIAG );
      if(DUMIN > DIAG) {
        DUMIN  = DIAG;
        JUMIN  = J;
      }
    }
  }
  else {
/*     --------------------------------------------------------------
       keepLU = 0.
       Only diag(U) is stored.  Set w(*) accordingly.
       Find DUmax and DUmin, the extreme diagonals of U.
       -------------------------------------------------------------- */
    LDIAGU = LENA2 - LUSOL->n;
    for(K = 1; K <= NRANK; K++) {
      J           = LUSOL->iq[K];
      DIAG        = fabs( LUSOL->a[LDIAGU + J] ); /* are in natural order */
      LUSOL->w[J] = DIAG;
      SETMAX( DUMAX, DIAG );
      if(DUMIN > DIAG) {
        DUMIN = DIAG;
        JUMIN = J;
      }
    }
  }
/*     --------------------------------------------------------------
       Negate w(j) if the corresponding diagonal of U is
       too small in absolute terms or relative to the other elements
       in the same column of  U.

       23 Apr 2004: TRP ensures that diags are NOT small relative to
                    other elements in their own column.
                    Much better, we can compare all diags to DUmax.
      -------------------------------------------------------------- */
  if((MODE == 1) && TRP) {
    SETMAX( UTOL1, UTOL2*DUMAX );
  }

  if(KEEPLU) {
    for(K = 1; K <= LUSOL->n; K++) {
      J = LUSOL->iq[K];
      if(K>NRANK)
        DIAG = ZERO;
      else {
        I = LUSOL->ip[K];
        L1 = LUSOL->locr[I];
        DIAG = fabs(LUSOL->a[L1]);
      }
      if((DIAG<=UTOL1) || (DIAG<=UTOL2*LUSOL->w[J])) {
        LUSOL_addSingularity(LUSOL, J, INFORM);
        LUSOL->w[J] = -LUSOL->w[J];
      }
    }
  }
  else { /* keepLU = FALSE */
    for(K = 1; K <= LUSOL->n; K++) {
      J = LUSOL->iq[K];
      DIAG = LUSOL->w[J];
      if(DIAG<=UTOL1) {
        LUSOL_addSingularity(LUSOL, J, INFORM);
        LUSOL->w[J] = -LUSOL->w[J];
      }
    }
  }
/*     -----------------------------------------------------------------
        Set output parameters.
       ----------------------------------------------------------------- */
  if(JUMIN==0)
    DUMIN = ZERO;
  LUSOL->luparm[LUSOL_IP_COLINDEX_DUMIN] = JUMIN;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_DIAGU]  = DUMAX;
  LUSOL->parmlu[LUSOL_RP_MINELEM_DIAGU]  = DUMIN;
/*      The matrix has been judged singular. */
  if(LUSOL->luparm[LUSOL_IP_SINGULARITIES]>0) {
    *INFORM = LUSOL_INFORM_LUSINGULAR;
    NDEFIC = LUSOL->n-NRANK;
    if((LUSOL->outstream!=NULL) && (LPRINT>=LUSOL_MSG_SINGULARITY)) {
      LUSOL_report(LUSOL, 0, "Singular(m%cn)  rank:%9d  n-rank:%8d  nsing:%9d\n",
                             relationChar(LUSOL->m, LUSOL->n),NRANK,NDEFIC,
                             LUSOL->luparm[LUSOL_IP_SINGULARITIES]);
    }
  }
/*      Exit. */
  LUSOL->luparm[LUSOL_IP_INFORM] = *INFORM;
}
#endif


/* ------------------------------------------------------------------
   Include routines for row-based L0.
   20 Apr 2005 Current version - KE.
   ------------------------------------------------------------------ */
#include "lusol6l0.c"


/* ------------------------------------------------------------------
   lu6L   solves   L v = v(input).
   ------------------------------------------------------------------
   15 Dec 2002: First version derived from lu6sol.
   15 Dec 2002: Current version.
   ------------------------------------------------------------------ */
void LU6L(LUSOLrec *LUSOL, int *INFORM, REAL V[], int NZidx[])
{
  int  JPIV, K, L, L1, LEN, LENL, LENL0, NUML, NUML0;
  REAL SMALL;
  register REAL VPIV;
#ifdef LUSOLFastSolve
  REAL *aptr;
  int  *iptr, *jptr;
#else
  int  I, J;
#endif

  NUML0 = LUSOL->luparm[LUSOL_IP_COLCOUNT_L0];
  LENL0 = LUSOL->luparm[LUSOL_IP_NONZEROS_L0];
  LENL  = LUSOL->luparm[LUSOL_IP_NONZEROS_L];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  L1 = LUSOL->lena+1;
  for(K = 1; K <= NUML0; K++) {
    LEN = LUSOL->lenc[K];
    L = L1;
    L1 -= LEN;
    JPIV = LUSOL->indr[L1];
    VPIV = V[JPIV];
    if(fabs(VPIV)>SMALL) {
/*     ***** This loop could be coded specially. */
#ifdef LUSOLFastSolve
      L--;
      for(aptr = LUSOL->a+L, iptr = LUSOL->indc+L;
          LEN > 0; LEN--, aptr--, iptr--)
        V[*iptr] += (*aptr) * VPIV;
#else
      for(; LEN > 0; LEN--) {
        L--;
        I = LUSOL->indc[L];
        V[I] += LUSOL->a[L]*VPIV;
      }
#endif
    }
#ifdef SetSmallToZero
    else
      V[JPIV] = 0;
#endif
  }
  L = (LUSOL->lena-LENL0)+1;
  NUML = LENL-LENL0;
/*     ***** This loop could be coded specially. */
#ifdef LUSOLFastSolve
  L--;
  for(aptr = LUSOL->a+L, jptr = LUSOL->indr+L, iptr = LUSOL->indc+L;
      NUML > 0; NUML--, aptr--, jptr--, iptr--) {
    if(fabs(V[*jptr])>SMALL)
      V[*iptr] += (*aptr) * V[*jptr];
#ifdef SetSmallToZero
    else
      V[*jptr] = 0;
#endif
  }
#else
  for(; NUML > 0; NUML--) {
    L--;
    J = LUSOL->indr[L];
    if(fabs(V[J])>SMALL) {
      I = LUSOL->indc[L];
      V[I] += LUSOL->a[L]*V[J];
    }
#ifdef SetSmallToZero
    else
      V[J] = 0;
#endif
  }
#endif
/*      Exit. */
  LUSOL->luparm[LUSOL_IP_INFORM] = *INFORM;
}

/* ==================================================================
   lu6LD  assumes lu1fac has computed factors A = LU of a
   symmetric definite or quasi-definite matrix A,
   using Threshold Symmetric Pivoting (TSP),   luparm(6) = 3,
   or    Threshold Diagonal  Pivoting (TDP),   luparm(6) = 4.
   It also assumes that no updates have been performed.
   In such cases,  U = D L', where D = diag(U).
   lu6LDL returns v as follows:

   mode
    1    v  solves   L D v = v(input).
    2    v  solves   L|D|v = v(input).
   ------------------------------------------------------------------
   15 Dec 2002: First version of lu6LD.
   15 Dec 2002: Current version.
   ================================================================== */
void LU6LD(LUSOLrec *LUSOL, int *INFORM, int MODE, REAL V[], int NZidx[])
{
  int  IPIV, K, L, L1, LEN, NUML0;
  REAL DIAG, SMALL;
  register REAL VPIV;
#ifdef LUSOLFastSolve
  REAL *aptr;
  int  *jptr;
#else
  int  J;
#endif

/*      Solve L D v(new) = v  or  L|D|v(new) = v, depending on mode.
        The code for L is the same as in lu6L,
        but when a nonzero entry of v arises, we divide by
        the corresponding entry of D or |D|. */
  NUML0 = LUSOL->luparm[LUSOL_IP_COLCOUNT_L0];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  L1 = LUSOL->lena+1;
  for(K = 1; K <= NUML0; K++) {
    LEN = LUSOL->lenc[K];
    L = L1;
    L1 -= LEN;
    IPIV = LUSOL->indr[L1];
    VPIV = V[IPIV];
    if(fabs(VPIV)>SMALL) {
/*     ***** This loop could be coded specially. */
#ifdef LUSOLFastSolve
      L--;
      for(aptr = LUSOL->a+L, jptr = LUSOL->indc+L;
          LEN > 0; LEN--, aptr--, jptr--)
        V[*jptr] += (*aptr)*VPIV;
#else
      for(; LEN > 0; LEN--) {
        L--;
        J = LUSOL->indc[L];
        V[J] += LUSOL->a[L]*VPIV;
      }
#endif
/*      Find diag = U(ipiv,ipiv) and divide by diag or |diag|. */
      L = LUSOL->locr[IPIV];
      DIAG = LUSOL->a[L];
      if(MODE==2)
        DIAG = fabs(DIAG);
      V[IPIV] = VPIV/DIAG;
    }
#ifdef SetSmallToZero
    else
      V[IPIV] = 0;
#endif
  }
}


/* ==================================================================
   lu6Lt  solves   L'v = v(input).
   ------------------------------------------------------------------
   15 Dec 2002: First version derived from lu6sol.
   15 Dec 2002: Current version.
   ================================================================== */
void LU6LT(LUSOLrec *LUSOL, int *INFORM, REAL V[], int NZidx[])
{
#ifdef DoTraceL0
  REAL    TEMP;
#endif
  int     K, L, L1, L2, LEN, LENL, LENL0, NUML0;
  REAL    SMALL;
  register REALXP SUM;
  register REAL HOLD;
#if (defined LUSOLFastSolve) && !(defined DoTraceL0)
  REAL    *aptr;
  int     *iptr, *jptr;
#else
  int     I, J;
#endif

  NUML0 = LUSOL->luparm[LUSOL_IP_COLCOUNT_L0];
  LENL0 = LUSOL->luparm[LUSOL_IP_NONZEROS_L0];
  LENL  = LUSOL->luparm[LUSOL_IP_NONZEROS_L];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  L1 = (LUSOL->lena-LENL)+1;
  L2 = LUSOL->lena-LENL0;

/*     ***** This loop could be coded specially. */
#if (defined LUSOLFastSolve) && !(defined DoTraceL0)
  for(L = L1, aptr = LUSOL->a+L1, iptr = LUSOL->indr+L1, jptr = LUSOL->indc+L1;
      L <= L2; L++, aptr++, iptr++, jptr++) {
    HOLD = V[*jptr];
    if(fabs(HOLD)>SMALL)
      V[*iptr] += (*aptr)*HOLD;
#ifdef SetSmallToZero
    else
      V[*jptr] = 0;
#endif
  }
#else
  for(L = L1; L <= L2; L++) {
    J = LUSOL->indc[L];
    HOLD = V[J];
    if(fabs(HOLD)>SMALL) {
      I = LUSOL->indr[L];
      V[I] += LUSOL->a[L]*HOLD;
    }
#ifdef SetSmallToZero
    else
      V[J] = 0;
#endif
  }
#endif

  /* Do row-based L0 version, if available */
  if((LUSOL->L0 != NULL) ||
     ((LUSOL->luparm[LUSOL_IP_BTRANCOUNT] == 0) && LU1L0(LUSOL, &(LUSOL->L0), INFORM))) {
    LU6L0T_v(LUSOL, LUSOL->L0, V, NZidx, INFORM);
  }

  /* Alternatively, do the standard column-based L0 version */
  else  {
    /* Perform loop over columns */
    for(K = NUML0; K >= 1; K--) {
      SUM = ZERO;
      LEN = LUSOL->lenc[K];
      L1 = L2+1;
      L2 += LEN;
/*     ***** This loop could be coded specially. */
#if (defined LUSOLFastSolve) && !(defined DoTraceL0)
      for(L = L1, aptr = LUSOL->a+L1, jptr = LUSOL->indc+L1;
          L <= L2; L++, aptr++, jptr++)
        SUM += (*aptr) * V[*jptr];
#else
      for(L = L1; L <= L2; L++) {
        J = LUSOL->indc[L];
#ifndef DoTraceL0
        SUM += LUSOL->a[L]*V[J];
#else
        TEMP = V[LUSOL->indr[L1]] + SUM;
        SUM += LUSOL->a[L]*V[J];
        printf("V[%3d] = V[%3d] + L[%d,%d]*V[%3d]\n", LUSOL->indr[L1], LUSOL->indr[L1], J,LUSOL->indr[L1], J);
        printf("%6g = %6g + %6g*%6g\n", V[LUSOL->indr[L1]] + SUM, TEMP, LUSOL->a[L], V[J]);
#endif
      }
#endif
      V[LUSOL->indr[L1]] += (REAL) SUM;
    }
  }

/*      Exit. */
  LUSOL->luparm[LUSOL_IP_INFORM] = *INFORM;
}

void print_L0(LUSOLrec *LUSOL)
{
  int  I, J, K, L, L1, L2, LEN, LENL0, NUML0;
  REAL *denseL0 = (REAL*) calloc(LUSOL->m+1, (LUSOL->n+1)*sizeof(*denseL0));

  NUML0 = LUSOL->luparm[LUSOL_IP_COLCOUNT_L0];
  LENL0 = LUSOL->luparm[LUSOL_IP_NONZEROS_L0];

  L2 = LUSOL->lena-LENL0;
  for(K = NUML0; K >= 1; K--) {
    LEN = LUSOL->lenc[K];
    L1 = L2+1;
    L2 += LEN;
    for(L = L1; L <= L2; L++) {
      I = LUSOL->indc[L];
      I = LUSOL->ipinv[I]; /* Undo row mapping */
      J = LUSOL->indr[L];
      denseL0[(LUSOL->n+1)*(J-1) + I] = LUSOL->a[L];
    }
  }

  for(I = 1; I <= LUSOL->n; I++) {
    for(J = 1; J <= LUSOL->m; J++)
      fprintf(stdout, "%10g", denseL0[(LUSOL->n+1)*(J-1) + I]);
    fprintf(stdout, "\n");
  }
  LUSOL_FREE(denseL0);
}


/* ------------------------------------------------------------------
   Include routines for column-based U.
   5 Feb 2006 Current version - KE.
   ------------------------------------------------------------------ */
#include "lusol6u.c"


/* ==================================================================
   lu6U   solves   U w = v.          v  is not altered.
   ------------------------------------------------------------------
   15 Dec 2002: First version derived from lu6sol.
   15 Dec 2002: Current version.
   ================================================================== */
void LU6U(LUSOLrec *LUSOL, int *INFORM, REAL V[], REAL W[], int NZidx[])
{
  /* Do column-based U version, if available */
  if((LUSOL->U != NULL) ||
     ((LUSOL->luparm[LUSOL_IP_FTRANCOUNT] == 0) && LU1U0(LUSOL, &(LUSOL->U), INFORM))) {
    LU6U0_v(LUSOL, LUSOL->U, V, W, NZidx, INFORM);
  }
  /* Alternatively, do the standard column-based L0 version */
  else {
    int  I, J, K, KLAST, L, L1, L2, L3, NRANK, NRANK1;
    REAL SMALL;
    register REALXP T;
#ifdef LUSOLFastSolve
    REAL *aptr;
    int  *jptr;
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
#ifdef LUSOLFastSolve
    for(K = KLAST+1, jptr = LUSOL->iq+K; K <= L; K++, jptr++)
      W[*jptr] = ZERO;
#else
    for(K = KLAST+1; K <= L; K++) {
      J = LUSOL->iq[K];
      W[J] = ZERO;
    }
#endif
/*      Do the back-substitution, using rows 1:klast of U. */
    for(K = KLAST; K >= 1; K--) {
      I = LUSOL->ip[K];
      T = V[I];
      L1 = LUSOL->locr[I];
      L2 = L1+1;
      L3 = (L1+LUSOL->lenr[I])-1;
/*     ***** This loop could be coded specially. */
#ifdef LUSOLFastSolve
      for(L = L2, aptr = LUSOL->a+L2, jptr = LUSOL->indr+L2;
          L <= L3; L++, aptr++, jptr++)
        T -= (*aptr) * W[*jptr];
#else
      for(L = L2; L <= L3; L++) {
        J = LUSOL->indr[L];
        T -= LUSOL->a[L]*W[J];
      }
#endif
      J = LUSOL->iq[K];
      if(fabs((REAL) T)<=SMALL)
        T = ZERO;
      else
        T /= LUSOL->a[L1];
      W[J] = (REAL) T;
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
}

/* ==================================================================
   lu6Ut  solves   U'v = w.          w  is destroyed.
   ------------------------------------------------------------------
   15 Dec 2002: First version derived from lu6sol.
   15 Dec 2002: Current version.
   ================================================================== */
void LU6UT(LUSOLrec *LUSOL, int *INFORM, REAL V[], REAL W[], int NZidx[])
{
  int  I, J, K, L, L1, L2, NRANK, NRANK1,
       *ip = LUSOL->ip + 1, *iq = LUSOL->iq + 1;
  REAL SMALL;
  register REAL T;
#ifdef LUSOLFastSolve
  REAL *aptr;
  int  *jptr;
#endif

  NRANK = LUSOL->luparm[LUSOL_IP_RANK_U];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  NRANK1 = NRANK+1;
  L = LUSOL->m;
#ifdef LUSOLFastSolve
  for(K = NRANK1, jptr = LUSOL->ip+K; K <= L; K++, jptr++)
    V[*jptr] = ZERO;
#else
  for(K = NRANK1; K <= L; K++) {
    I = LUSOL->ip[K];
    V[I] = ZERO;
  }
#endif
/*      Do the forward-substitution, skipping columns of U(transpose)
        when the associated element of w(*) is negligible. */
#if 0
  for(K = 1; K <= NRANK; K++) {
    I = LUSOL->ip[K];
    J = LUSOL->iq[K];
#else
  for(K = 1; K <= NRANK; K++, ip++, iq++) {
    I = *ip;
    J = *iq;
#endif
    T = W[J];
    if(fabs(T)<=SMALL) {
      V[I] = ZERO;
      continue;
    }
    L1 = LUSOL->locr[I];
    T /= LUSOL->a[L1];
    V[I] = T;
    L2 = (L1+LUSOL->lenr[I])-1;
    L1++;
/*     ***** This loop could be coded specially. */
#ifdef LUSOLFastSolve
    for(L = L1, aptr = LUSOL->a+L1, jptr = LUSOL->indr+L1;
        L <= L2; L++, aptr++, jptr++)
      W[*jptr] -= T * (*aptr);
#else
    for(L = L1; L <= L2; L++) {
      J = LUSOL->indr[L];
      W[J] -= T*LUSOL->a[L];
    }
#endif
  }
/*      Compute residual for overdetermined systems. */
  T = ZERO;
  for(K = NRANK1; K <= LUSOL->n; K++) {
    J = LUSOL->iq[K];
    T += fabs(W[J]);
  }
/*      Exit. */
  if(T>ZERO)
    *INFORM = LUSOL_INFORM_LUSINGULAR;
  LUSOL->luparm[LUSOL_IP_INFORM]     = *INFORM;
  LUSOL->parmlu[LUSOL_RP_RESIDUAL_U] = T;
}

/* ==================================================================
   lu6sol  uses the factorization  A = L U  as follows:
   ------------------------------------------------------------------
   mode
    1    v  solves   L v = v(input).   w  is not touched.
    2    v  solves   L'v = v(input).   w  is not touched.
    3    w  solves   U w = v.          v  is not altered.
    4    v  solves   U'v = w.          w  is destroyed.
    5    w  solves   A w = v.          v  is altered as in 1.
    6    v  solves   A'v = w.          w  is destroyed.

   If mode = 3,4,5,6, v and w must not be the same arrays.
   If lu1fac has just been used to factorize a symmetric matrix A
   (which must be definite or quasi-definite), the factors A = L U
   may be regarded as A = LDL', where D = diag(U).  In such cases,

   mode
    7    v  solves   A v = L D L'v = v(input).   w  is not touched.
    8    v  solves       L |D| L'v = v(input).   w  is not touched.

   ip(*), iq(*)      hold row and column numbers in pivotal order.
   lenc(k)           is the length of the k-th column of initial L.
   lenr(i)           is the length of the i-th row of U.
   locc(*)           is not used.
   locr(i)           is the start  of the i-th row of U.

   U is assumed to be in upper-trapezoidal form (nrank by n).
   The first entry for each row is the diagonal element
   (according to the permutations  ip, iq).  It is stored at
   location locr(i) in a(*), indr(*).

   On exit, inform = 0 except as follows.
     if(mode = 3,4,5,6 and if U (and hence A) is singular,)
     inform = 1 if there is a nonzero residual in solving the system
     involving U.  parmlu(20) returns the norm of the residual.
   ------------------------------------------------------------------
     July 1987: Early version.
   09 May 1988: f77 version.
   27 Apr 2000: Abolished the dreaded "computed go to".
                But hard to change other "go to"s to "if then else".
   15 Dec 2002: lu6L, lu6Lt, lu6U, lu6Ut added to modularize lu6sol.
   ================================================================== */
void LU6SOL(LUSOLrec *LUSOL, int MODE, REAL V[], REAL W[], int NZidx[], int *INFORM)
{
  if(MODE==LUSOL_SOLVE_Lv_v) {          /* Solve  L v(new) = v. */
    LU6L(LUSOL, INFORM,V, NZidx);
  }
  else if(MODE==LUSOL_SOLVE_Ltv_v) {    /* Solve  L'v(new) = v. */
    LU6LT(LUSOL, INFORM,V, NZidx);
  }
  else if(MODE==LUSOL_SOLVE_Uw_v) {     /* Solve  U w = v. */
    LU6U(LUSOL, INFORM,V,W, NZidx);
  }
  else if(MODE==LUSOL_SOLVE_Utv_w) {    /* Solve  U'v = w. */
    LU6UT(LUSOL, INFORM,V,W, NZidx);
  }
  else if(MODE==LUSOL_SOLVE_Aw_v) {     /* Solve  A w      = v (i.e. FTRAN) */
    LU6L(LUSOL, INFORM,V, NZidx);        /* via     L v(new) = v */
    LU6U(LUSOL, INFORM,V,W, NULL);       /* ... and U w = v(new). */
  }
  else if(MODE==LUSOL_SOLVE_Atv_w) {    /* Solve  A'v = w (i.e. BTRAN) */
    LU6UT(LUSOL, INFORM,V,W, NZidx);     /* via      U'v = w */
    LU6LT(LUSOL, INFORM,V, NULL);        /* ... and  L'v(new) = v. */
  }
  else if(MODE==LUSOL_SOLVE_Av_v) {     /* Solve  LDv(bar) = v */
    LU6LD(LUSOL, INFORM,1,V, NZidx);     /* and    L'v(new) = v(bar). */
    LU6LT(LUSOL, INFORM,V, NULL);
  }
  else if(MODE==LUSOL_SOLVE_LDLtv_v) {  /* Solve  L|D|v(bar) = v */
    LU6LD(LUSOL, INFORM,2,V, NZidx);     /* and    L'v(new) = v(bar). */
    LU6LT(LUSOL, INFORM,V, NULL);
  }
}

