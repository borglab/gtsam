
/* ==================================================================
   lu1DCP factors a dense m x n matrix A by Gaussian elimination,
   using Complete Pivoting (row and column interchanges) for stability.
   This version also uses column interchanges if all elements in a
   pivot column are smaller than (or equal to) "small".  Such columns
   are changed to zero and permuted to the right-hand end.
   As in LINPACK's dgefa, ipvt(!) keeps track of pivot rows.
   Rows of U are interchanged, but we don't have to physically
   permute rows of L.  In contrast, column interchanges are applied
   directly to the columns of both L and U, and to the column
   permutation vector iq(*).
   ------------------------------------------------------------------
   On entry:
      a       Array holding the matrix A to be factored.
      lda     The leading dimension of the array  a.
      m       The number of rows    in  A.
      n       The number of columns in  A.
      small   A drop tolerance.  Must be zero or positive.

   On exit:
      a       An upper triangular matrix and the multipliers
              which were used to obtain it.
              The factorization can be written  A = L*U  where
              L  is a product of permutation and unit lower
              triangular matrices and  U  is upper triangular.
      nsing   Number of singularities detected.
      ipvt    Records the pivot rows.
      iq      A vector to which column interchanges are applied.
   ------------------------------------------------------------------
   01 May 2002: First dense Complete Pivoting, derived from lu1DPP.
   07 May 2002: Another break needed at end of first loop.
   07 May 2002: Current version of lu1DCP.
   ================================================================== */
void LU1DCP(LUSOLrec *LUSOL, REAL DA[], int LDA, int M, int N, REAL SMALL,
            int *NSING, int IPVT[], int IX[])
{

  int       I, J, K, KP1, L, LAST, LENCOL, IMAX, JMAX, JLAST, JNEW;
  REAL      AIJMAX, AJMAX;
  register REAL T;
#ifdef LUSOLFastDenseIndex
  register REAL *DA1, *DA2;
  int IDA1, IDA2;
#else
  register int IDA1, IDA2;
#endif

  *NSING = 0;
  LENCOL = M+1;
  LAST = N;
/*     -----------------------------------------------------------------
        Start of elimination loop.
       ----------------------------------------------------------------- */
  for(K = 1; K <= N; K++) {
    KP1 = K+1;
    LENCOL--;
/*      Find the biggest aij in row imax and column jmax. */
    AIJMAX = ZERO;
    IMAX = K;
    JMAX = K;
    JLAST = LAST;
    for(J = K; J <= JLAST; J++) {
x10:
      L = idamax(LENCOL,DA+DAPOS(K,J)-LUSOL_ARRAYOFFSET,1)+K-1;
      AJMAX = fabs(DA[DAPOS(L,J)]);
      if(AJMAX<=SMALL) {
/*     ========================================================
        Do column interchange, changing old column to zero.
        Reduce  "last"  and try again with same j.
       ======================================================== */
        (*NSING)++;
        JNEW = IX[LAST];
        IX[LAST] = IX[J];
        IX[J] = JNEW;
#ifdef LUSOLFastDenseIndex
        DA1 = DA+DAPOS(0,LAST);
        DA2 = DA+DAPOS(0,J);
        for(I = 1; I <= K-1; I++) {
          DA1++;
          DA2++;
          T = *DA1;
          *DA1 = *DA2;
          *DA2 = T;
#else
        for(I = 1; I <= K-1; I++) {
          IDA1 = DAPOS(I,LAST);
          IDA2 = DAPOS(I,J);
          T = DA[IDA1];
          DA[IDA1] = DA[IDA2];
          DA[IDA2] = T;
#endif
        }
#ifdef LUSOLFastDenseIndex
        for(I = K; I <= M; I++) {
          DA1++;
          DA2++;
          T = *DA1;
          *DA1 = ZERO;
          *DA2 = T;
#else
        for(I = K; I <= M; I++) {
          IDA1 = DAPOS(I,LAST);
          IDA2 = DAPOS(I,J);
          T = DA[IDA1];
          DA[IDA1] = ZERO;
          DA[IDA2] = T;
#endif
        }
        LAST--;
        if(J<=LAST)
          goto x10;
        break;
      }
/*      Check if this column has biggest aij so far. */
      if(AIJMAX<AJMAX) {
        AIJMAX = AJMAX;
        IMAX = L;
        JMAX = J;
      }
      if(J>=LAST)
        break;
    }
    IPVT[K] = IMAX;
    if(JMAX!=K) {
/*     ==========================================================
        Do column interchange (k and jmax).
       ========================================================== */
      JNEW = IX[JMAX];
      IX[JMAX] = IX[K];
      IX[K] = JNEW;
#ifdef LUSOLFastDenseIndex
      DA1 = DA+DAPOS(0,JMAX);
      DA2 = DA+DAPOS(0,K);
      for(I = 1; I <= M; I++) {
        DA1++;
        DA2++;
        T = *DA1;
        *DA1 = *DA2;
        *DA2 = T;
#else
      for(I = 1; I <= M; I++) {
        IDA1 = DAPOS(I,JMAX);
        IDA2 = DAPOS(I,K);
        T = DA[IDA1];
        DA[IDA1] = DA[IDA2];
        DA[IDA2] = T;
#endif
      }
    }
    if(M>K) {
/*     ===========================================================
        Do row interchange if necessary.
       =========================================================== */
      if(IMAX!=K) {
        IDA1 = DAPOS(IMAX,K);
        IDA2 = DAPOS(K,K);
        T = DA[IDA1];
        DA[IDA1] = DA[IDA2];
        DA[IDA2] = T;
      }
/*     ===========================================================
        Compute multipliers.
        Do row elimination with column indexing.
       =========================================================== */
      T = -ONE/DA[DAPOS(K,K)];
      dscal(M-K,T,DA+DAPOS(KP1,K)-LUSOL_ARRAYOFFSET,1);
      for(J = KP1; J <= LAST; J++) {
        IDA1 = DAPOS(IMAX,J);
        T = DA[IDA1];
        if(IMAX!=K) {
          IDA2 = DAPOS(K,J);
          DA[IDA1] = DA[IDA2];
          DA[IDA2] = T;
        }
        daxpy(M-K,T,DA+DAPOS(KP1,K)-LUSOL_ARRAYOFFSET,1,
                    DA+DAPOS(KP1,J)-LUSOL_ARRAYOFFSET,1);
      }
    }
    else
      break;
    if(K>=LAST)
      break;
  }
/*      Set ipvt(*) for singular rows. */
  for(K = LAST+1; K <= M; K++)
    IPVT[K] = K;

}

/* ==================================================================
   lu1DPP factors a dense m x n matrix A by Gaussian elimination,
   using row interchanges for stability, as in dgefa from LINPACK.
   This version also uses column interchanges if all elements in a
   pivot column are smaller than (or equal to) "small".  Such columns
   are changed to zero and permuted to the right-hand end.
   As in LINPACK, ipvt(*) keeps track of pivot rows.
   Rows of U are interchanged, but we don't have to physically
   permute rows of L.  In contrast, column interchanges are applied
   directly to the columns of both L and U, and to the column
   permutation vector iq(*).
   ------------------------------------------------------------------
   On entry:
        a       Array holding the matrix A to be factored.
        lda     The leading dimension of the array  a.
        m       The number of rows    in  A.
        n       The number of columns in  A.
        small   A drop tolerance.  Must be zero or positive.

   On exit:
        a       An upper triangular matrix and the multipliers
                which were used to obtain it.
                The factorization can be written  A = L*U  where
                L  is a product of permutation and unit lower
                triangular matrices and  U  is upper triangular.
        nsing   Number of singularities detected.
        ipvt    Records the pivot rows.
        iq      A vector to which column interchanges are applied.
   ------------------------------------------------------------------
   02 May 1989: First version derived from dgefa
                in LINPACK (version dated 08/14/78).
   05 Feb 1994: Generalized to treat rectangular matrices
                and use column interchanges when necessary.
                ipvt is retained, but column permutations are applied
                directly to iq(*).
   21 Dec 1994: Bug found via example from Steve Dirkse.
                Loop 100 added to set ipvt(*) for singular rows.
   ================================================================== */
void LU1DPP(LUSOLrec *LUSOL, REAL DA[], int LDA, int M, int N, REAL SMALL,
            int *NSING, int IPVT[], int IX[])
{
  int            I, J, K, KP1, L, LAST, LENCOL;
  register REAL T;
#ifdef LUSOLFastDenseIndex
  register REAL *DA1, *DA2;
  int IDA1, IDA2;
#else
  register int IDA1, IDA2;
#endif

  *NSING = 0;
  K = 1;
  LAST = N;
/*      ------------------------------------------------------------------
        Start of elimination loop.
        ------------------------------------------------------------------ */
x10:
  KP1 = K+1;
  LENCOL = (M-K)+1;
/*      Find l, the pivot row. */
  L = (idamax(LENCOL,DA+DAPOS(K,K)-LUSOL_ARRAYOFFSET,1)+K)-1;
  IPVT[K] = L;
  if(fabs(DA[DAPOS(L,K)])<=SMALL) {
/*         ===============================================================
           Do column interchange, changing old pivot column to zero.
           Reduce  "last"  and try again with same k.
           =============================================================== */
    (*NSING)++;
    J = IX[LAST];
    IX[LAST] = IX[K];
    IX[K] = J;
#ifdef LUSOLFastDenseIndex
    DA1 = DA+DAPOS(0,LAST);
    DA2 = DA+DAPOS(0,K);
    for(I = 1; I <= K-1; I++) {
      DA1++;
      DA2++;
      T = *DA1;
      *DA1 = *DA2;
      *DA2 = T;
#else
    for(I = 1; I <= K-1; I++) {
      IDA1 = DAPOS(I,LAST);
      IDA2 = DAPOS(I,K);
      T = DA[IDA1];
      DA[IDA1] = DA[IDA2];
      DA[IDA2] = T;
#endif
    }
#ifdef LUSOLFastDenseIndex
    for(I = K; I <= M; I++) {
      DA1++;
      DA2++;
      T = *DA1;
      *DA1 = ZERO;
      *DA2 = T;
#else
    for(I = K; I <= M; I++) {
      IDA1 = DAPOS(I,LAST);
      IDA2 = DAPOS(I,K);
      T = DA[IDA1];
      DA[IDA1] = ZERO;
      DA[IDA2] = T;
#endif
    }
    LAST = LAST-1;
    if(K<=LAST)
      goto x10;
  }
  else if(M>K) {
/*         ===============================================================
           Do row interchange if necessary.
           =============================================================== */
    if(L!=K) {
      IDA1 = DAPOS(L,K);
      IDA2 = DAPOS(K,K);
      T = DA[IDA1];
      DA[IDA1] = DA[IDA2];
      DA[IDA2] = T;
    }
/*         ===============================================================
           Compute multipliers.
           Do row elimination with column indexing.
           =============================================================== */
    T = -ONE/DA[DAPOS(K,K)];
    dscal(M-K,T,DA+DAPOS(KP1,K)-LUSOL_ARRAYOFFSET,1);
    for(J = KP1; J <= LAST; J++) {
      IDA1 = DAPOS(L,J);
      T = DA[IDA1];
      if(L!=K) {
        IDA2 = DAPOS(K,J);
        DA[IDA1] = DA[IDA2];
        DA[IDA2] = T;
      }
      daxpy(M-K,T,DA+DAPOS(KP1,K)-LUSOL_ARRAYOFFSET,1,
                  DA+DAPOS(KP1,J)-LUSOL_ARRAYOFFSET,1);
    }
    K++;
    if(K<=LAST)
      goto x10;
  }
/*      Set ipvt(*) for singular rows. */
  for(K = LAST+1; K <= M; K++)
    IPVT[K] = K;

}


/* ==================================================================
   lu1pq1  constructs a permutation  iperm  from the array  len.
   ------------------------------------------------------------------
   On entry:
   len(i)  holds the number of nonzeros in the i-th row (say)
           of an m by n matrix.
   num(*)  can be anything (workspace).

   On exit:
   iperm   contains a list of row numbers in the order
           rows of length 0,  rows of length 1,..., rows of length n.
   loc(nz) points to the first row containing  nz  nonzeros,
           nz = 1, n.
   inv(i)  points to the position of row i within iperm(*).
   ================================================================== */
void LU1PQ1(LUSOLrec *LUSOL, int M, int N, int LEN[],
            int IPERM[], int LOC[], int INV[], int NUM[])
{
  int NZEROS, NZ, I, L;

/*      Count the number of rows of each length. */
  NZEROS = 0;
  for(NZ = 1; NZ <= N; NZ++) {
    NUM[NZ] = 0;
    LOC[NZ] = 0;
  }
  for(I = 1; I <= M; I++) {
    NZ = LEN[I];
    if(NZ==0)
      NZEROS++;
    else
      NUM[NZ]++;
  }
/*      Set starting locations for each length. */
  L = NZEROS+1;
  for(NZ = 1; NZ <= N; NZ++) {
    LOC[NZ] = L;
    L += NUM[NZ];
    NUM[NZ] = 0;
  }
/*      Form the list. */
  NZEROS = 0;
  for(I = 1; I <= M; I++) {
    NZ = LEN[I];
    if(NZ==0) {
      NZEROS++;
      IPERM[NZEROS] = I;
    }
    else {
      L = LOC[NZ]+NUM[NZ];
      IPERM[L] = I;
      NUM[NZ]++;
    }
  }
/*      Define the inverse of iperm. */
  for(L = 1; L <= M; L++) {
    I = IPERM[L];
    INV[I] = L;
  }
}

/* ==================================================================
   lu1pq2 frees the space occupied by the pivot row,
   and updates the column permutation iq.
   Also used to free the pivot column and update the row perm ip.
   ------------------------------------------------------------------
   nzpiv   (input)    is the length of the pivot row (or column).
   nzchng  (output)   is the net change in total nonzeros.
   ------------------------------------------------------------------
   14 Apr 1989  First version.
   ================================================================== */
void LU1PQ2(LUSOLrec *LUSOL, int NZPIV, int *NZCHNG,
            int IND[], int LENOLD[], int LENNEW[], int IXLOC[], int IX[], int IXINV[])
{
  int LR, J, NZ, NZNEW, L, NEXT, LNEW, JNEW;

  *NZCHNG = 0;
  for(LR = 1; LR <= NZPIV; LR++) {
    J = IND[LR];
    IND[LR] = 0;
    NZ = LENOLD[LR];
    NZNEW = LENNEW[J];
    if(NZ!=NZNEW) {
      L = IXINV[J];
      *NZCHNG = (*NZCHNG+NZNEW)-NZ;
/*            l above is the position of column j in iq  (so j = iq(l)). */
      if(NZ<NZNEW) {
/*               Column  j  has to move towards the end of  iq. */
x110:
        NEXT = NZ+1;
        LNEW = IXLOC[NEXT]-1;
        if(LNEW!=L) {
          JNEW = IX[LNEW];
          IX[L] = JNEW;
          IXINV[JNEW] = L;
        }
        L = LNEW;
        IXLOC[NEXT] = LNEW;
        NZ = NEXT;
        if(NZ<NZNEW)
          goto x110;
      }
      else {
/*               Column  j  has to move towards the front of  iq. */
x120:
        LNEW = IXLOC[NZ];
        if(LNEW!=L) {
          JNEW = IX[LNEW];
          IX[L] = JNEW;
          IXINV[JNEW] = L;
        }
        L = LNEW;
        IXLOC[NZ] = LNEW+1;
        NZ = NZ-1;
        if(NZ>NZNEW)
          goto x120;
      }
      IX[LNEW] = J;
      IXINV[J] = LNEW;
    }
  }
}

/* ==================================================================
   lu1pq3  looks at the permutation  iperm(*)  and moves any entries
   to the end whose corresponding length  len(*)  is zero.
   ------------------------------------------------------------------
   09 Feb 1994: Added work array iw(*) to improve efficiency.
   ================================================================== */
void LU1PQ3(LUSOLrec *LUSOL, int MN, int LEN[], int IPERM[], int IW[], int *NRANK)
{
  int NZEROS, K, I;

  *NRANK = 0;
  NZEROS = 0;
  for(K = 1; K <= MN; K++) {
    I = IPERM[K];
    if(LEN[I]==0) {
      NZEROS++;
      IW[NZEROS] = I;
    }
    else {
      (*NRANK)++;
      IPERM[*NRANK] = I;
    }
  }
  for(K = 1; K <= NZEROS; K++)
    IPERM[(*NRANK)+K] = IW[K];
}

/* ==================================================================
   lu1rec
   ------------------------------------------------------------------
   On exit:
   ltop         is the length of useful entries in ind(*), a(*).
   ind(ltop+1)  is "i" such that len(i), loc(i) belong to the last
                item in ind(*), a(*).
   ------------------------------------------------------------------
   00 Jun 1983: Original version of lu1rec followed John Reid's
                compression routine in LA05.  It recovered
                space in ind(*) and optionally a(*)
                by eliminating entries with ind(l) = 0.
                The elements of ind(*) could not be negative.
                If len(i) was positive, entry i contained
                that many elements, starting at  loc(i).
                Otherwise, entry i was eliminated.
   23 Mar 2001: Realised we could have len(i) = 0 in rare cases!
                (Mostly during TCP when the pivot row contains
                a column of length 1 that couldn't be a pivot.)
                Revised storage scheme to
                   keep        entries with       ind(l) >  0,
                   squeeze out entries with -n <= ind(l) <= 0,
                and to allow len(i) = 0.
                Empty items are moved to the end of the compressed
                ind(*) and/or a(*) arrays are given one empty space.
                Items with len(i) < 0 are still eliminated.
   27 Mar 2001: Decided to use only ind(l) > 0 and = 0 in lu1fad.
                Still have to keep entries with len(i) = 0.
   ================================================================== */
void LU1REC(LUSOLrec *LUSOL, int N, MYBOOL REALS, int *LTOP,
                             int IND[], int LEN[], int LOC[])
{
  int  NEMPTY, I, LENI, L, LEND, K, KLAST, ILAST, LPRINT;

  NEMPTY = 0;
  for(I = 1; I <= N; I++) {
    LENI = LEN[I];
    if(LENI>0) {
      L = (LOC[I]+LENI)-1;
      LEN[I] = IND[L];
      IND[L] = -(N+I);
    }
    else if(LENI==0)
      NEMPTY++;
  }
  K = 0;
/*      Previous k */
  KLAST = 0;
/*      Last entry moved. */
  ILAST = 0;
  LEND = *LTOP;
  for(L = 1; L <= LEND; L++) {
    I = IND[L];
    if(I>0) {
      K++;
      IND[K] = I;
      if(REALS)
        LUSOL->a[K] = LUSOL->a[L];
    }
    else if(I<-N) {
/*            This is the end of entry  i. */
      I = -(N+I);
      ILAST = I;
      K++;
      IND[K] = LEN[I];
      if(REALS)
        LUSOL->a[K] = LUSOL->a[L];
      LOC[I] = KLAST+1;
      LEN[I] = K-KLAST;
      KLAST = K;
    }
  }
/*      Move any empty items to the end, adding 1 free entry for each. */
  if(NEMPTY>0) {
    for(I = 1; I <= N; I++) {
      if(LEN[I]==0) {
        K++;
        LOC[I] = K;
        IND[K] = 0;
        ILAST = I;
      }
    }
  }
  LPRINT = LUSOL->luparm[LUSOL_IP_PRINTLEVEL];
  if(LPRINT>=LUSOL_MSG_PIVOT)
    LUSOL_report(LUSOL, 0, "lu1rec.  File compressed from %d to %d\n",
                        *LTOP,K,REALS,NEMPTY);
/*      ncp */
  LUSOL->luparm[LUSOL_IP_COMPRESSIONS_LU]++;
/*      Return ilast in ind(ltop + 1). */
  *LTOP = K;
  IND[(*LTOP)+1] = ILAST;
}

/* ==================================================================
   lu1slk  sets w(j) > 0 if column j is a unit vector.
   ------------------------------------------------------------------
   21 Nov 2000: First version.  lu1fad needs it for TCP.
                Note that w(*) is nominally an integer array,
                but the only spare space is the double array w(*).
   ================================================================== */
void LU1SLK(LUSOLrec *LUSOL)
{
  int J, LC1, LQ, LQ1, LQ2;

  for(J = 1; J <= LUSOL->n; J++) {
    LUSOL->w[J] = 0;
  }
  LQ1 = (LUSOL->iqloc ? LUSOL->iqloc[1] : LUSOL->n+1);
/*  LQ1 = LUSOL->iqloc[1];   This is the original version; correction above by Yin Zhang */
  LQ2 = LUSOL->n;
  if(LUSOL->m>1)
    LQ2 = LUSOL->iqloc[2]-1;
  for(LQ = LQ1; LQ <= LQ2; LQ++) {
    J = LUSOL->iq[LQ];
    LC1 = LUSOL->locc[J];
    if(fabs(LUSOL->a[LC1])==1) {
      LUSOL->w[J] = 1;
    }
  }
}

/* ==================================================================
   lu1gau does most of the work for each step of Gaussian elimination.
   A multiple of the pivot column is added to each other column j
   in the pivot row.  The column list is fully updated.
   The row list is updated if there is room, but some fill-ins may
   remain, as indicated by ifill and jfill.
   ------------------------------------------------------------------
   Input:
      ilast    is the row    at the end of the row    list.
      jlast    is the column at the end of the column list.
      lfirst   is the first column to be processed.
      lu + 1   is the corresponding element of U in au(*).
      nfill    keeps track of pending fill-in.
      a(*)     contains the nonzeros for each column j.
      indc(*)  contains the row indices for each column j.
      al(*)    contains the new column of L.  A multiple of it is
               used to modify each column.
      mark(*)  has been set to -1, -2, -3, ... in the rows
               corresponding to nonzero 1, 2, 3, ... of the col of L.
      au(*)    contains the new row of U.  Each nonzero gives the
               required multiple of the column of L.

   Workspace:
      markl(*) marks the nonzeros of L actually used.
               (A different mark, namely j, is used for each column.)

   Output:
      ilast     New last row    in the row    list.
      jlast     New last column in the column list.
      lfirst    = 0 if all columns were completed,
                > 0 otherwise.
      lu        returns the position of the last nonzero of U
                actually used, in case we come back in again.
      nfill     keeps track of the total extra space needed in the
                row file.
      ifill(ll) counts pending fill-in for rows involved in the new
                column of L.
      jfill(lu) marks the first pending fill-in stored in columns
                involved in the new row of U.
   ------------------------------------------------------------------
   16 Apr 1989: First version of lu1gau.
   23 Apr 1989: lfirst, lu, nfill are now input and output
                to allow re-entry if elimination is interrupted.
   23 Mar 2001: Introduced ilast, jlast.
   27 Mar 2001: Allow fill-in "in situ" if there is already room
                up to but NOT INCLUDING the end of the
                row or column file.
                Seems safe way to avoid overwriting empty rows/cols
                at the end.  (May not be needed though, now that we
                have ilast and jlast.)
   ================================================================== */
void LU1GAU(LUSOLrec *LUSOL, int MELIM, int NSPARE,
            REAL SMALL, int LPIVC1, int LPIVC2, int *LFIRST, int LPIVR2,
            int LFREE, int MINFRE, int ILAST, int *JLAST, int *LROW, int *LCOL,
            int *LU, int *NFILL,
            int MARK[],  REAL AL[], int MARKL[], REAL AU[], int IFILL[], int JFILL[])
{
  MYBOOL ATEND;
  int    LR, J, LENJ, NFREE, LC1, LC2, NDONE, NDROP, L, I, LL, K,
         LR1, LAST, LREP, L1, L2, LC, LENI;
  register REAL UJ;
  REAL   AIJ;

  for(LR = *LFIRST; LR <= LPIVR2; LR++) {
    J = LUSOL->indr[LR];
    LENJ = LUSOL->lenc[J];
    NFREE = LFREE - *LCOL;
    if(NFREE<MINFRE)
      goto x900;
/*         ---------------------------------------------------------------
           Inner loop to modify existing nonzeros in column  j.
           Loop 440 performs most of the arithmetic involved in the
           whole LU factorization.
           ndone counts how many multipliers were used.
           ndrop counts how many modified nonzeros are negligibly small.
           --------------------------------------------------------------- */
    (*LU)++;
    UJ = AU[*LU];
    LC1 = LUSOL->locc[J];
    LC2 = (LC1+LENJ)-1;
    ATEND = (MYBOOL) (J==*JLAST);
    NDONE = 0;
    if(LENJ==0)
      goto x500;
    NDROP = 0;
    for(L = LC1; L <= LC2; L++) {
      I = LUSOL->indc[L];
      LL = -MARK[I];
      if(LL>0) {
        NDONE++;
        MARKL[LL] = J;
        LUSOL->a[L] += AL[LL]*UJ;
        if(fabs(LUSOL->a[L])<=SMALL) {
          NDROP++;
        }
      }
    }
/*         ---------------------------------------------------------------
           Remove any negligible modified nonzeros from both
           the column file and the row file.
           --------------------------------------------------------------- */
    if(NDROP==0)
      goto x500;
    K = LC1;
    for(L = LC1; L <= LC2; L++) {
      I = LUSOL->indc[L];
      if(fabs(LUSOL->a[L])<=SMALL)
        goto x460;
      LUSOL->a[K] = LUSOL->a[L];
      LUSOL->indc[K] = I;
      K++;
      continue;
/*            Delete the nonzero from the row file. */
x460:
      LENJ--;
      LUSOL->lenr[I]--;
      LR1 = LUSOL->locr[I];
      LAST = LR1+LUSOL->lenr[I];
      for(LREP = LR1; LREP <= LAST; LREP++) {
        if(LUSOL->indr[LREP]==J)
          break;
      }
      LUSOL->indr[LREP] = LUSOL->indr[LAST];
      LUSOL->indr[LAST] = 0;
      if(I==ILAST)
        (*LROW)--;
    }
/*         Free the deleted elements from the column file. */
#ifdef LUSOLFastClear
    MEMCLEAR(LUSOL->indc+K, LC2-K+1);
#else
    for(L = K; L <= LC2; L++)
      LUSOL->indc[L] = ZERO;
#endif
    if(ATEND)
      *LCOL = K-1;
/*         ---------------------------------------------------------------
           Deal with the fill-in in column j.
           --------------------------------------------------------------- */
x500:
    if(NDONE==MELIM)
      goto x590;
/*         See if column j already has room for the fill-in. */
    if(ATEND)
      goto x540;
    LAST = (LC1+LENJ)-1;
    L1 = LAST+1;
    L2 = (LAST+MELIM)-NDONE;
/*      27 Mar 2001: Be sure it's not at or past end of the col file. */
    if(L2>=*LCOL)
      goto x520;
    for(L = L1; L <= L2; L++) {
      if(LUSOL->indc[L]!=0)
        goto x520;
    }
    goto x540;
/*         We must move column j to the end of the column file.
           First, leave some spare room at the end of the
           current last column. */
x520:
#if 1
    L1 = (*LCOL)+1;
    L2 = (*LCOL)+NSPARE;
    *LCOL = L2;
    for(L = L1; L <= L2; L++) {
#else
    for(L = (*LCOL)+1; L <= (*LCOL)+NSPARE; L++) {
      *LCOL = L;  /* ****** ERROR ???? */
#endif
/*      Spare space is free. */
      LUSOL->indc[L] = 0;
    }
    ATEND = TRUE;
    *JLAST = J;
    L1 = LC1;
    L2 = *LCOL;
    LC1 = L2+1;
    LUSOL->locc[J] = LC1;
    for(L = L1; L <= LAST; L++) {
      L2++;
      LUSOL->a[L2] = LUSOL->a[L];
      LUSOL->indc[L2] = LUSOL->indc[L];
/*      Free space. */
      LUSOL->indc[L] = 0;
    }
    *LCOL = L2;
/*         ---------------------------------------------------------------
           Inner loop for the fill-in in column j.
           This is usually not very expensive.
           --------------------------------------------------------------- */
x540:
    LAST = (LC1+LENJ)-1;
    LL = 0;
    for(LC = LPIVC1; LC <= LPIVC2; LC++) {
      LL++;
      if(MARKL[LL]==J)
        continue;
      AIJ = AL[LL]*UJ;
      if(fabs(AIJ)<=SMALL)
        continue;
      LENJ++;
      LAST++;
      LUSOL->a[LAST] = AIJ;
      I = LUSOL->indc[LC];
      LUSOL->indc[LAST] = I;
      LENI = LUSOL->lenr[I];
/*            Add 1 fill-in to row i if there is already room.
              27 Mar 2001: Be sure it's not at or past the }
                           of the row file. */
      L = LUSOL->locr[I]+LENI;
      if(L>=*LROW)
        goto x550;
      if(LUSOL->indr[L]>0)
        goto x550;
      LUSOL->indr[L] = J;
      LUSOL->lenr[I] = LENI+1;
      continue;
/*            Row i does not have room for the fill-in.
              Increment ifill(ll) to count how often this has
              happened to row i.  Also, add m to the row index
              indc(last) in column j to mark it as a fill-in that is
              still pending.
              If this is the first pending fill-in for row i,
              nfill includes the current length of row i
              (since the whole row has to be moved later).
              If this is the first pending fill-in for column j,
              jfill(lu) records the current length of column j
              (to shorten the search for pending fill-ins later). */
x550:
      if(IFILL[LL]==0)
        (*NFILL) += LENI+NSPARE;
      if(JFILL[*LU]==0)
        JFILL[*LU] = LENJ;
      (*NFILL)++;
      IFILL[LL]++;
      LUSOL->indc[LAST] = LUSOL->m+I;
    }
    if(ATEND)
      *LCOL = LAST;
/*         End loop for column  j.  Store its final length. */
x590:
    LUSOL->lenc[J] = LENJ;
  }
/*      Successful completion. */
  *LFIRST = 0;
  return;
/*      Interruption.  We have to come back in after the
        column file is compressed.  Give lfirst a new value.
        lu and nfill will retain their current values. */
x900:
  *LFIRST = LR;
}

/* ==================================================================
   lu1mar  uses a Markowitz criterion to select a pivot element
   for the next stage of a sparse LU factorization,
   subject to a Threshold Partial Pivoting stability criterion (TPP)
   that bounds the elements of L.
   ------------------------------------------------------------------
   gamma  is "gamma" in the tie-breaking rule TB4 in the LUSOL paper.
   ------------------------------------------------------------------
   Search cols of length nz = 1, then rows of length nz = 1,
   then   cols of length nz = 2, then rows of length nz = 2, etc.
   ------------------------------------------------------------------
   00 Jan 1986  Version documented in LUSOL paper:
                Gill, Murray, Saunders and Wright (1987),
                Maintaining LU factors of a general sparse matrix,
                Linear algebra and its applications 88/89, 239-270.
   02 Feb 1989  Following Suhl and Aittoniemi (1987), the largest
                element in each column is now kept at the start of
                the column, i.e. in position locc(j) of a and indc.
                This should speed up the Markowitz searches.
   26 Apr 1989  Both columns and rows searched during spars1 phase.
                Only columns searched during spars2 phase.
                maxtie replaced by maxcol and maxrow.
   05 Nov 1993  Initializing  "mbest = m * n"  wasn't big enough when
                m = 10, n = 3, and last column had 7 nonzeros.
   09 Feb 1994  Realised that "mbest = maxmn * maxmn" might overflow.
                Changed to    "mbest = maxmn * 1000".
   27 Apr 2000  On large example from Todd Munson,
                that allowed  "if (mbest .le. nz1**2) go to 900"
                to exit before any pivot had been found.
                Introduced kbest = mbest / nz1.
                Most pivots can be rejected with no integer multiply.
                TRUE merit is evaluated only if it's as good as the
                best so far (or better).  There should be no danger
                of integer overflow unless A is incredibly
                large and dense.
   10 Sep 2000  TCP, aijtol added for Threshold Complete Pivoting.
   ================================================================== */
void LU1MAR(LUSOLrec *LUSOL, int MAXMN, MYBOOL TCP, REAL AIJTOL, REAL LTOL,
            int MAXCOL, int MAXROW, int *IBEST, int *JBEST, int *MBEST)
{
  int  KBEST, NCOL, NROW, NZ1, NZ, LQ1, LQ2, LQ, J, LC1, LC2, LC, I, LEN1, MERIT, LP1,
       LP2, LP, LR1, LR2, LR;
  REAL ABEST, LBEST, AMAX, AIJ, CMAX;

  ABEST = ZERO;
  LBEST = ZERO;
  *IBEST = 0;
  *MBEST = -1;
  KBEST = MAXMN+1;
  NCOL = 0;
  NROW = 0;
  NZ1 = 0;
  for(NZ = 1; NZ <= MAXMN; NZ++) {
/*         nz1    = nz - 1
           if (mbest .le. nz1**2) go to 900 */
    if(KBEST<=NZ1)
      goto x900;
    if(*IBEST>0) {
      if(NCOL>=MAXCOL)
        goto x200;
    }
    if(NZ>LUSOL->m)
      goto x200;
/*         ---------------------------------------------------------------
           Search the set of columns of length  nz.
           --------------------------------------------------------------- */
    LQ1 = LUSOL->iqloc[NZ];
    LQ2 = LUSOL->n;
    if(NZ<LUSOL->m)
      LQ2 = LUSOL->iqloc[NZ+1]-1;
    for(LQ = LQ1; LQ <= LQ2; LQ++) {
      NCOL = NCOL+1;
      J = LUSOL->iq[LQ];
      LC1 = LUSOL->locc[J];
      LC2 = LC1+NZ1;
      AMAX = fabs(LUSOL->a[LC1]);
/*            Test all aijs in this column.
              amax is the largest element (the first in the column).
              cmax is the largest multiplier if aij becomes pivot. */
      if(TCP) {
/*      Nothing in whole column */
        if(AMAX<AIJTOL)
          continue;
      }
      for(LC = LC1; LC <= LC2; LC++) {
        I = LUSOL->indc[LC];
        LEN1 = LUSOL->lenr[I]-1;
/*               merit  = nz1 * len1
                 if (merit > mbest) continue; */
        if(LEN1>KBEST)
          continue;
/*               aij  has a promising merit.
                 Apply the stability test.
                 We require  aij  to be sufficiently large compared to
                 all other nonzeros in column  j.  This is equivalent
                 to requiring cmax to be bounded by Ltol. */
        if(LC==LC1) {
/*                  This is the maximum element, amax.
                    Find the biggest element in the rest of the column
                    and hence get cmax.  We know cmax .le. 1, but
                    we still want it exactly in order to break ties.
                    27 Apr 2002: Settle for cmax = 1. */
          AIJ = AMAX;
          CMAX = ONE;
/*                  cmax   = zero
                    for (l = lc1 + 1; l <= lc2; l++)
                       cmax  = max( cmax, abs( a(l) ) );
                    cmax   = cmax / amax; */
        }
        else {
/*                  aij is not the biggest element, so cmax .ge. 1.
                    Bail out if cmax will be too big. */
          AIJ = fabs(LUSOL->a[LC]);
/*      Absolute test for Complete Pivoting */
          if(TCP) {
            if(AIJ<AIJTOL)
              continue;
/*      TPP */
          }
          else {
            if(AIJ*LTOL<AMAX)
              continue;
          }
          CMAX = AMAX/AIJ;
        }
/*               aij  is big enough.  Its maximum multiplier is cmax. */
        MERIT = NZ1*LEN1;
        if(MERIT==*MBEST) {
/*                  Break ties.
                    (Initializing mbest < 0 prevents getting here if
                    nothing has been found yet.)
                    In this version we minimize cmax
                    but if it is already small we maximize the pivot. */
          if(LBEST<=LUSOL->parmlu[LUSOL_RP_GAMMA] &&
             CMAX<=LUSOL->parmlu[LUSOL_RP_GAMMA]) {
            if(ABEST>=AIJ)
              continue;
          }
          else {
            if(LBEST<=CMAX)
              continue;
          }
        }
/*               aij  is the best pivot so far. */
        *IBEST = I;
        *JBEST = J;
        KBEST = LEN1;
        *MBEST = MERIT;
        ABEST = AIJ;
        LBEST = CMAX;
        if(NZ==1)
          goto x900;
      }
/*            Finished with that column. */
      if(*IBEST>0) {
        if(NCOL>=MAXCOL)
          goto x200;
      }
    }
/*         ---------------------------------------------------------------
           Search the set of rows of length  nz.
           --------------------------------------------------------------- */
x200:
/*    if (mbest .le. nz*nz1) go to 900 */
    if(KBEST<=NZ)
      goto x900;
    if(*IBEST>0) {
      if(NROW>=MAXROW)
        goto x290;
    }
    if(NZ>LUSOL->n)
      goto x290;
    LP1 = LUSOL->iploc[NZ];
    LP2 = LUSOL->m;
    if(NZ<LUSOL->n)
      LP2 = LUSOL->iploc[NZ+1]-1;
    for(LP = LP1; LP <= LP2; LP++) {
      NROW++;
      I = LUSOL->ip[LP];
      LR1 = LUSOL->locr[I];
      LR2 = LR1+NZ1;
      for(LR = LR1; LR <= LR2; LR++) {
        J = LUSOL->indr[LR];
        LEN1 = LUSOL->lenc[J]-1;
/*               merit  = nz1 * len1
                 if (merit .gt. mbest) continue */
        if(LEN1>KBEST)
          continue;
/*               aij  has a promising merit.
                 Find where  aij  is in column  j. */
        LC1 = LUSOL->locc[J];
        LC2 = LC1+LEN1;
        AMAX = fabs(LUSOL->a[LC1]);
        for(LC = LC1; LC <= LC2; LC++) {
          if(LUSOL->indc[LC]==I)
            break;
        }
/*               Apply the same stability test as above. */
        AIJ = fabs(LUSOL->a[LC]);
/*      Absolute test for Complete Pivoting */
        if(TCP) {
          if(AIJ<AIJTOL)
            continue;
        }
        if(LC==LC1) {
/*                  This is the maximum element, amax.
                    Find the biggest element in the rest of the column
                    and hence get cmax.  We know cmax .le. 1, but
                    we still want it exactly in order to break ties.
                    27 Apr 2002: Settle for cmax = 1. */
          CMAX = ONE;
/*                  cmax   = zero
                    for(l = lc1 + 1; l <= lc2; l++)
                       cmax  = max( cmax, fabs( a(l) ) )
                    cmax   = cmax / amax */
        }
        else {
/*                  aij is not the biggest element, so cmax .ge. 1.
                    Bail out if cmax will be too big. */
          if(TCP) {
/*      relax */
          }
          else {
            if(AIJ*LTOL<AMAX)
              continue;
          }
          CMAX = AMAX/AIJ;
        }
/*               aij  is big enough.  Its maximum multiplier is cmax. */
        MERIT = NZ1*LEN1;
        if(MERIT==*MBEST) {
/*                  Break ties as before.
                    (Initializing mbest < 0 prevents getting here if
                    nothing has been found yet.) */
          if(LBEST<=LUSOL->parmlu[LUSOL_RP_GAMMA] &&
             CMAX<=LUSOL->parmlu[LUSOL_RP_GAMMA]) {
            if(ABEST>=AIJ)
              continue;
          }
          else {
            if(LBEST<=CMAX)
              continue;
          }
        }
/*               aij  is the best pivot so far. */
        *IBEST = I;
        *JBEST = J;
        *MBEST = MERIT;
        KBEST = LEN1;
        ABEST = AIJ;
        LBEST = CMAX;
        if(NZ==1)
          goto x900;
      }
/*            Finished with that row. */
      if(*IBEST>0) {
        if(NROW>=MAXROW)
          goto x290;
      }
    }
/*         See if it's time to quit. */
x290:
    if(*IBEST>0) {
      if(NROW>=MAXROW && NCOL>=MAXCOL)
        goto x900;
    }
/*         Press on with next nz. */
    NZ1 = NZ;
    if(*IBEST>0)
      KBEST = *MBEST/NZ1;
  }
x900:
;
}

/* ==================================================================
   lu1mCP  uses a Markowitz criterion to select a pivot element
   for the next stage of a sparse LU factorization,
   subject to a Threshold Complete Pivoting stability criterion (TCP)
   that bounds the elements of L and U.
   ------------------------------------------------------------------
   gamma  is "gamma" in the tie-breaking rule TB4 in the LUSOL paper.
   ------------------------------------------------------------------
   09 May 2002: First version of lu1mCP.
                It searches columns only, using the heap that
                holds the largest element in each column.
   09 May 2002: Current version of lu1mCP.
   ================================================================== */
void LU1MCP(LUSOLrec *LUSOL, REAL AIJTOL, int *IBEST, int *JBEST, int *MBEST,
            int HLEN, REAL HA[], int HJ[])
{
  int  J, KHEAP, LC, LC1, LC2, LENJ, MAXCOL, NCOL, NZ1, I, LEN1, MERIT;
  REAL ABEST, AIJ, AMAX, CMAX, LBEST;

/*      ------------------------------------------------------------------
        Search up to maxcol columns stored at the top of the heap.
        The very top column helps initialize mbest.
        ------------------------------------------------------------------ */
  ABEST = ZERO;
  LBEST = ZERO;
  *IBEST = 0;
/*      Column at the top of the heap */
  *JBEST = HJ[1];
  LENJ = LUSOL->lenc[*JBEST];
/*      Bigger than any possible merit */
  *MBEST = LENJ*HLEN;
/*      ??? Big question */
  MAXCOL = 40;
/*      No. of columns searched */
  NCOL = 0;
  for(KHEAP = 1; KHEAP <= HLEN; KHEAP++) {
    AMAX = HA[KHEAP];
    if(AMAX<AIJTOL)
      continue;
    NCOL++;
    J = HJ[KHEAP];
/*         ---------------------------------------------------------------
           This column has at least one entry big enough (the top one).
           Search the column for other possibilities.
           --------------------------------------------------------------- */
    LENJ = LUSOL->lenc[J];
    NZ1 = LENJ-1;
    LC1 = LUSOL->locc[J];
    LC2 = LC1+NZ1;
/* --      amax   = abs( a(lc1) )
           Test all aijs in this column.
           amax is the largest element (the first in the column).
           cmax is the largest multiplier if aij becomes pivot. */
    for(LC = LC1; LC <= LC2; LC++) {
      I = LUSOL->indc[LC];
      LEN1 = LUSOL->lenr[I]-1;
      MERIT = NZ1*LEN1;
      if(MERIT>*MBEST)
        continue;
/*            aij  has a promising merit. */
      if(LC==LC1) {
/*               This is the maximum element, amax.
                 Find the biggest element in the rest of the column
                 and hence get cmax.  We know cmax .le. 1, but
                 we still want it exactly in order to break ties.
                 27 Apr 2002: Settle for cmax = 1. */
        AIJ = AMAX;
        CMAX = ONE;
/*               cmax   = ZERO;
                 for(l = lc1 + 1; l <= lc2; l++)
                    cmax  = max( cmax, abs( a(l) ) )
                 cmax   = cmax / amax; */
      }
      else {
/*               aij is not the biggest element, so cmax .ge. 1.
                 Bail out if cmax will be too big. */
        AIJ = fabs(LUSOL->a[LC]);
        if(AIJ<AIJTOL)
          continue;
        CMAX = AMAX/AIJ;
      }
/*            aij  is big enough.  Its maximum multiplier is cmax. */
      if(MERIT==*MBEST) {
/*               Break ties.
                 (Initializing mbest "too big" prevents getting here if
                 nothing has been found yet.)
                 In this version we minimize cmax
                 but if it is already small we maximize the pivot. */
        if(LBEST<=LUSOL->parmlu[LUSOL_RP_GAMMA] &&
           CMAX<=LUSOL->parmlu[LUSOL_RP_GAMMA]) {
          if(ABEST>=AIJ)
            continue;
        }
        else {
          if(LBEST<=CMAX)
            continue;
        }
      }
/*            aij  is the best pivot so far. */
      *IBEST = I;
      *JBEST = J;
      *MBEST = MERIT;
      ABEST = AIJ;
      LBEST = CMAX;
/*      Col or row of length 1 */
      if(MERIT==0)
        goto x900;
    }
    if(NCOL>=MAXCOL)
      goto x900;
  }
x900:
;
}

/* ==================================================================
   lu1mRP  uses a Markowitz criterion to select a pivot element
   for the next stage of a sparse LU factorization,
   subject to a Threshold Rook Pivoting stability criterion (TRP)
   that bounds the elements of L and U.
   ------------------------------------------------------------------
   11 Jun 2002: First version of lu1mRP derived from lu1mar.
   11 Jun 2002: Current version of lu1mRP.
   ================================================================== */
void LU1MRP(LUSOLrec *LUSOL, int MAXMN, REAL LTOL, int MAXCOL, int MAXROW,
  int *IBEST, int *JBEST, int *MBEST, REAL AMAXR[])
{
  int  I, J, KBEST, LC, LC1, LC2, LEN1, LP, LP1, LP2, LQ, LQ1,
       LQ2, LR, LR1, LR2, MERIT, NCOL, NROW, NZ, NZ1;
  REAL ABEST, AIJ, AMAX, ATOLI, ATOLJ;

/*      ------------------------------------------------------------------
        Search cols of length nz = 1, then rows of length nz = 1,
        then   cols of length nz = 2, then rows of length nz = 2, etc.
        ------------------------------------------------------------------ */
  ABEST = ZERO;
  *IBEST = 0;
  KBEST = MAXMN+1;
  *MBEST = -1;
  NCOL = 0;
  NROW = 0;
  NZ1 = 0;
  for(NZ = 1; NZ <= MAXMN; NZ++) {
/*         nz1    = nz - 1
           if (mbest .le. nz1**2) go to 900 */
    if(KBEST<=NZ1)
      goto x900;
    if(*IBEST>0) {
      if(NCOL>=MAXCOL)
        goto x200;
    }
    if(NZ>LUSOL->m)
      goto x200;
/*         ---------------------------------------------------------------
           Search the set of columns of length  nz.
           --------------------------------------------------------------- */
    LQ1 = LUSOL->iqloc[NZ];
    LQ2 = LUSOL->n;
    if(NZ<LUSOL->m)
      LQ2 = LUSOL->iqloc[NZ+1]-1;
    for(LQ = LQ1; LQ <= LQ2; LQ++) {
      NCOL = NCOL+1;
      J = LUSOL->iq[LQ];
      LC1 = LUSOL->locc[J];
      LC2 = LC1+NZ1;
      AMAX = fabs(LUSOL->a[LC1]);
/*      Min size of pivots in col j */
      ATOLJ = AMAX/LTOL;
/*            Test all aijs in this column. */
      for(LC = LC1; LC <= LC2; LC++) {
        I = LUSOL->indc[LC];
        LEN1 = LUSOL->lenr[I]-1;
/*               merit  = nz1 * len1
                 if (merit .gt. mbest) continue; */
        if(LEN1>KBEST)
          continue;
/*               aij  has a promising merit.
                 Apply the Threshold Rook Pivoting stability test.
                 First we require aij to be sufficiently large
                 compared to other nonzeros in column j.
                 Then  we require aij to be sufficiently large
                 compared to other nonzeros in row    i. */
        AIJ = fabs(LUSOL->a[LC]);
        if(AIJ<ATOLJ)
          continue;
        if(AIJ*LTOL<AMAXR[I])
          continue;
/*               aij  is big enough. */
        MERIT = NZ1*LEN1;
        if(MERIT==*MBEST) {
/*                  Break ties.
                    (Initializing mbest < 0 prevents getting here if
                    nothing has been found yet.) */
          if(ABEST>=AIJ)
            continue;
        }
/*               aij  is the best pivot so far. */
        *IBEST = I;
        *JBEST = J;
        KBEST = LEN1;
        *MBEST = MERIT;
        ABEST = AIJ;
        if(NZ==1)
          goto x900;
      }
/*            Finished with that column. */
      if(*IBEST>0) {
        if(NCOL>=MAXCOL)
          goto x200;
      }
    }
/*         ---------------------------------------------------------------
           Search the set of rows of length  nz.
           --------------------------------------------------------------- */
x200:
/*    if (mbest .le. nz*nz1) go to 900 */
    if(KBEST<=NZ)
      goto x900;
    if(*IBEST>0) {
      if(NROW>=MAXROW)
        goto x290;
    }
    if(NZ>LUSOL->n)
      goto x290;
    LP1 = LUSOL->iploc[NZ];
    LP2 = LUSOL->m;
    if(NZ<LUSOL->n)
      LP2 = LUSOL->iploc[NZ+1]-1;
    for(LP = LP1; LP <= LP2; LP++) {
      NROW = NROW+1;
      I = LUSOL->ip[LP];
      LR1 = LUSOL->locr[I];
      LR2 = LR1+NZ1;
/*      Min size of pivots in row i */
      ATOLI = AMAXR[I]/LTOL;
      for(LR = LR1; LR <= LR2; LR++) {
        J = LUSOL->indr[LR];
        LEN1 = LUSOL->lenc[J]-1;
/*               merit  = nz1 * len1
                 if (merit .gt. mbest) continue; */
        if(LEN1>KBEST)
          continue;
/*               aij  has a promising merit.
                 Find where  aij  is in column j. */
        LC1 = LUSOL->locc[J];
        LC2 = LC1+LEN1;
        AMAX = fabs(LUSOL->a[LC1]);
        for(LC = LC1; LC <= LC2; LC++) {
          if(LUSOL->indc[LC]==I)
            break;
        }
/*               Apply the Threshold Rook Pivoting stability test.
                 First we require aij to be sufficiently large
                 compared to other nonzeros in row    i.
                 Then  we require aij to be sufficiently large
                 compared to other nonzeros in column j. */
        AIJ = fabs(LUSOL->a[LC]);
        if(AIJ<ATOLI)
          continue;
        if(AIJ*LTOL<AMAX)
          continue;
/*               aij  is big enough. */
        MERIT = NZ1*LEN1;
        if(MERIT==*MBEST) {
/*                  Break ties as before.
                    (Initializing mbest < 0 prevents getting here if
                    nothing has been found yet.) */
          if(ABEST>=AIJ)
            continue;
        }
/*               aij  is the best pivot so far. */
        *IBEST = I;
        *JBEST = J;
        KBEST = LEN1;
        *MBEST = MERIT;
        ABEST = AIJ;
        if(NZ==1)
          goto x900;
      }
/*            Finished with that row. */
      if(*IBEST>0) {
        if(NROW>=MAXROW)
          goto x290;
      }
    }
/*         See if it's time to quit. */
x290:
    if(*IBEST>0) {
      if(NROW>=MAXROW && NCOL>=MAXCOL)
        goto x900;
    }
/*         Press on with next nz. */
    NZ1 = NZ;
    if(*IBEST>0)
      KBEST = *MBEST/NZ1;
  }
x900:
;
}

/* ==================================================================
   lu1mSP  is intended for symmetric matrices that are either
   definite or quasi-definite.
   lu1mSP  uses a Markowitz criterion to select a pivot element for
   the next stage of a sparse LU factorization of a symmetric matrix,
   subject to a Threshold Symmetric Pivoting stability criterion
   (TSP) restricted to diagonal elements to preserve symmetry.
   This bounds the elements of L and U and should have rank-revealing
   properties analogous to Threshold Rook Pivoting for unsymmetric
   matrices.
   ------------------------------------------------------------------
   14 Dec 2002: First version of lu1mSP derived from lu1mRP.
                There is no safeguard to ensure that A is symmetric.
   14 Dec 2002: Current version of lu1mSP.
   ================================================================== */
void LU1MSP(LUSOLrec *LUSOL, int MAXMN, REAL LTOL, int MAXCOL,
            int *IBEST, int *JBEST, int *MBEST)
{
  int  I, J, KBEST, LC, LC1, LC2, LQ, LQ1, LQ2, MERIT, NCOL, NZ, NZ1;
  REAL ABEST, AIJ, AMAX, ATOLJ;

/*      ------------------------------------------------------------------
        Search cols of length nz = 1, then cols of length nz = 2, etc.
        ------------------------------------------------------------------ */
  ABEST = ZERO;
  *IBEST = 0;
  *MBEST = -1;
  KBEST = MAXMN+1;
  NCOL = 0;
  NZ1 = 0;
  for(NZ = 1; NZ <= MAXMN; NZ++) {
/*         nz1    = nz - 1
           if (mbest .le. nz1**2) go to 900 */
    if(KBEST<=NZ1)
      goto x900;
    if(*IBEST>0) {
      if(NCOL>=MAXCOL)
        goto x200;
    }
    if(NZ>LUSOL->m)
      goto x200;
/*         ---------------------------------------------------------------
           Search the set of columns of length  nz.
           --------------------------------------------------------------- */
    LQ1 = LUSOL->iqloc[NZ];
    LQ2 = LUSOL->n;
    if(NZ<LUSOL->m)
      LQ2 = LUSOL->iqloc[NZ+1]-1;
    for(LQ = LQ1; LQ <= LQ2; LQ++) {
      NCOL++;
      J = LUSOL->iq[LQ];
      LC1 = LUSOL->locc[J];
      LC2 = LC1+NZ1;
      AMAX = fabs(LUSOL->a[LC1]);
/*      Min size of pivots in col j */
      ATOLJ = AMAX/LTOL;
/*            Test all aijs in this column.
              Ignore everything except the diagonal. */
      for(LC = LC1; LC <= LC2; LC++) {
        I = LUSOL->indc[LC];
/*      Skip off-diagonals. */
        if(I!=J)
          continue;
/*               merit  = nz1 * nz1
                 if (merit .gt. mbest) continue; */
        if(NZ1>KBEST)
          continue;
/*               aij  has a promising merit.
                 Apply the Threshold Partial Pivoting stability test
                 (which is equivalent to Threshold Rook Pivoting for
                 symmetric matrices).
                 We require aij to be sufficiently large
                 compared to other nonzeros in column j. */
        AIJ = fabs(LUSOL->a[LC]);
        if(AIJ<ATOLJ)
          continue;
/*               aij  is big enough. */
        MERIT = NZ1*NZ1;
        if(MERIT==*MBEST) {
/*                  Break ties.
                    (Initializing mbest < 0 prevents getting here if
                    nothing has been found yet.) */
          if(ABEST>=AIJ)
            continue;
        }
/*               aij  is the best pivot so far. */
        *IBEST = I;
        *JBEST = J;
        KBEST = NZ1;
        *MBEST = MERIT;
        ABEST = AIJ;
        if(NZ==1)
          goto x900;
      }
/*            Finished with that column. */
      if(*IBEST>0) {
        if(NCOL>=MAXCOL)
          goto x200;
      }
    }
/*         See if it's time to quit. */
x200:
    if(*IBEST>0) {
      if(NCOL>=MAXCOL)
        goto x900;
    }
/*         Press on with next nz. */
    NZ1 = NZ;
    if(*IBEST>0)
      KBEST = *MBEST/NZ1;
  }
x900:
;
}

/* ==================================================================
   lu1mxc  moves the largest element in each of columns iq(k1:k2)
   to the top of its column.
   If k1 > k2, nothing happens.
   ------------------------------------------------------------------
   06 May 2002: (and earlier)
                All columns k1:k2 must have one or more elements.
   07 May 2002: Allow for empty columns.  The heap routines need to
                find 0.0 as the "largest element".
   29 Nov 2005: Bug fix - avoiding overwriting the next column when
                the current column is empty (i.e. LENJ==0)
                Yin Zhang <yzhang@cs.utexas.edu>
   ================================================================== */
void LU1MXC(LUSOLrec *LUSOL, int K1, int K2, int IX[])
{
  int  I, J, K, L, LC, LENJ;
  REAL AMAX;

  for(K = K1; K <= K2; K++) {
    J = IX[K];
    LC = LUSOL->locc[J];
    LENJ = LUSOL->lenc[J];
    if(LENJ==0)
/*      LUSOL->a[LC] = ZERO;  Removal suggested by Yin Zhang to avoid overwriting next column when current is empty */
      ;
    else {
      L = idamax(LUSOL->lenc[J], LUSOL->a + LC - LUSOL_ARRAYOFFSET,1) + LC - 1;
      if(L>LC) {
        AMAX = LUSOL->a[L];
        LUSOL->a[L] = LUSOL->a[LC];
        LUSOL->a[LC] = AMAX;
        I = LUSOL->indc[L];
        LUSOL->indc[L] = LUSOL->indc[LC];
        LUSOL->indc[LC] = I;
      }
    }
  }
}

/* ==================================================================
   lu1mxr  finds the largest element in each of row ip(k1:k2)
   and stores it in Amaxr(*).  The nonzeros are stored column-wise
   in (a,indc,lenc,locc) and their structure is row-wise
   in (  indr,lenr,locr).
   If k1 > k2, nothing happens.
   ------------------------------------------------------------------
   11 Jun 2002: First version of lu1mxr.
                Allow for empty columns.
   ================================================================== */
void LU1MXR(LUSOLrec *LUSOL, int K1, int K2, int IX[], REAL AMAXR[])
{
#define FastMXR
#ifdef FastMXR
  static int  I, *J, *IC, K, LC, LC1, LC2, LR, LR1, LR2;
  static REAL AMAX;
#else
  int  I, J, K, LC, LC1, LC2, LR, LR1, LR2;
  REAL AMAX;
#endif

  for(K = K1; K <= K2; K++) {
    AMAX = ZERO;
    I = IX[K];
/*      Find largest element in row i. */
    LR1 = LUSOL->locr[I];
    LR2 = (LR1+LUSOL->lenr[I])-1;
#ifdef FastMXR
    for(LR = LR1, J = LUSOL->indr + LR1;
        LR <= LR2; LR++, J++) {
/*      Find where  aij  is in column  j. */
      LC1 = LUSOL->locc[*J];
      LC2 = LC1+LUSOL->lenc[*J];
      for(LC = LC1, IC = LUSOL->indc + LC1;
          LC < LC2; LC++, IC++) {
        if(*IC==I)
          break;
      }
      SETMAX(AMAX,fabs(LUSOL->a[LC]));
    }
#else
    for(LR = LR1; LR <= LR2; LR++) {
      J = LUSOL->indr[LR];
/*      Find where  aij  is in column  j. */
      LC1 = LUSOL->locc[J];
      LC2 = (LC1+LUSOL->lenc[J])-1;
      for(LC = LC1; LC <= LC2; LC++) {
        if(LUSOL->indc[LC]==I)
          break;
      }
      SETMAX(AMAX,fabs(LUSOL->a[LC]));
    }
#endif
    AMAXR[I] = AMAX;
  }
}


/* ==================================================================
   lu1ful computes a dense (full) LU factorization of the
   mleft by nleft matrix that remains to be factored at the
   beginning of the nrowu-th pass through the main loop of lu1fad.
   ------------------------------------------------------------------
   02 May 1989: First version.
   05 Feb 1994: Column interchanges added to lu1DPP.
   08 Feb 1994: ipinv reconstructed, since lu1pq3 may alter ip.
   ================================================================== */
void LU1FUL(LUSOLrec *LUSOL, int LEND, int LU1, MYBOOL TPP,
            int MLEFT, int NLEFT, int NRANK, int NROWU,
            int *LENL, int *LENU, int *NSING,
            MYBOOL KEEPLU, REAL SMALL, REAL D[], int IPVT[])
{
  int  L, I, J, IPBASE, LDBASE, LQ, LC1, LC2, LC, LD, LKK, LKN, LU, K, L1,
       L2, IBEST, JBEST, LA, LL, NROWD, NCOLD;
  REAL AI, AJ;

/*      ------------------------------------------------------------------
        If lu1pq3 moved any empty rows, reset ipinv = inverse of ip.
        ------------------------------------------------------------------ */
  if(NRANK<LUSOL->m) {
    for(L = 1; L <= LUSOL->m; L++) {
      I = LUSOL->ip[L];
      LUSOL->ipinv[I] = L;
    }
  }
/*      ------------------------------------------------------------------
        Copy the remaining matrix into the dense matrix D.
         ------------------------------------------------------------------ */
#ifdef LUSOLFastClear
  MEMCLEAR((D+1), LEND);
#else
/*   dload(LEND, ZERO, D, 1); */
  for(J = 1; J <= LEND; J++)
    D[J] = ZERO;
#endif

  IPBASE = NROWU-1;
  LDBASE = 1-NROWU;
  for(LQ = NROWU; LQ <= LUSOL->n; LQ++) {
    J = LUSOL->iq[LQ];
    LC1 = LUSOL->locc[J];
    LC2 = (LC1+LUSOL->lenc[J])-1;
    for(LC = LC1; LC <= LC2; LC++) {
      I = LUSOL->indc[LC];
      LD = LDBASE+LUSOL->ipinv[I];
      D[LD] = LUSOL->a[LC];
    }
    LDBASE += MLEFT;
  }
/*      ------------------------------------------------------------------
        Call our favorite dense LU factorizer.
        ------------------------------------------------------------------ */
  if(TPP)
    LU1DPP(LUSOL, D,MLEFT,MLEFT,NLEFT,SMALL,NSING,IPVT,LUSOL->iq+NROWU-LUSOL_ARRAYOFFSET);
  else
    LU1DCP(LUSOL, D,MLEFT,MLEFT,NLEFT,SMALL,NSING,IPVT,LUSOL->iq+NROWU-LUSOL_ARRAYOFFSET);

/*      ------------------------------------------------------------------
        Move D to the beginning of A,
        and pack L and U at the top of a, indc, indr.
        In the process, apply the row permutation to ip.
        lkk points to the diagonal of U.
        ------------------------------------------------------------------ */
#ifdef LUSOLFastCopy
  MEMCOPY(LUSOL->a+1,D+1,LEND);
#else
  dcopy(LEND,D,1,LUSOL->a,1);
#endif
#ifdef ClassicdiagU
  LUSOL->diagU = LUSOL->a + (LUSOL->lena-LUSOL->n);
#endif
  LKK = 1;
  LKN = (LEND-MLEFT)+1;
  LU = LU1;
  for(K = 1; K <= MIN(MLEFT,NLEFT); K++) {
    L1 = IPBASE+K;
    L2 = IPBASE+IPVT[K];
    if(L1!=L2) {
      I = LUSOL->ip[L1];
      LUSOL->ip[L1] = LUSOL->ip[L2];
      LUSOL->ip[L2] = I;
    }
    IBEST = LUSOL->ip[L1];
    JBEST = LUSOL->iq[L1];
    if(KEEPLU) {
/*            ===========================================================
              Pack the next column of L.
              =========================================================== */
      LA = LKK;
      LL = LU;
      NROWD = 1;
      for(I = K+1; I <= MLEFT; I++) {
        LA++;
        AI = LUSOL->a[LA];
        if(fabs(AI)>SMALL) {
          NROWD = NROWD+1;
          LL--;
          LUSOL->a[LL] = AI;
          LUSOL->indc[LL] = LUSOL->ip[IPBASE+I];
          LUSOL->indr[LL] = IBEST;
        }
      }
/*            ===========================================================
              Pack the next row of U.
              We go backwards through the row of D
              so the diagonal ends up at the front of the row of  U.
              Beware -- the diagonal may be zero.
              =========================================================== */
      LA = LKN+MLEFT;
      LU = LL;
      NCOLD = 0;
      for(J = NLEFT; J >= K; J--) {
        LA = LA-MLEFT;
        AJ = LUSOL->a[LA];
        if(fabs(AJ)>SMALL || J==K) {
          NCOLD++;
          LU--;
          LUSOL->a[LU] = AJ;
          LUSOL->indr[LU] = LUSOL->iq[IPBASE+J];
        }
      }
      LUSOL->lenr[IBEST] = -NCOLD;
      LUSOL->lenc[JBEST] = -NROWD;
      *LENL = ((*LENL)+NROWD)-1;
      *LENU = (*LENU)+NCOLD;
      LKN++;
    }
    else {
/*            ===========================================================
              Store just the diagonal of U, in natural order.
              =========================================================== */
      LUSOL->diagU[JBEST] = LUSOL->a[LKK];
    }
    LKK += MLEFT+1;
  }
}


/* ==================================================================
   lu1or1  organizes the elements of an  m by n  matrix  A  as
   follows.  On entry, the parallel arrays   a, indc, indr,
   contain  nelem  entries of the form     aij,    i,    j,
   in any order.  nelem  must be positive.
   Entries not larger than the input parameter  small  are treated as
   zero and removed from   a, indc, indr.  The remaining entries are
   defined to be nonzero.  numnz  returns the number of such nonzeros
   and  Amax  returns the magnitude of the largest nonzero.
   The arrays  lenc, lenr  return the number of nonzeros in each
   column and row of  A.
   inform = 0  on exit, except  inform = 1  if any of the indices in
   indc, indr  imply that the element  aij  lies outside the  m by n
   dimensions of  A.
   ------------------------------------------------------------------
   xx Feb 1985: Original version.
   17 Oct 2000: a, indc, indr now have size lena to allow nelem = 0.
   ================================================================== */
void LU1OR1(LUSOLrec *LUSOL, REAL SMALL,
            REAL *AMAX, int *NUMNZ, int *LERR, int *INFORM)
{
  int I, J, L, LDUMMY;

#ifdef LUSOLFastClear
  MEMCLEAR((LUSOL->lenr+1), LUSOL->m);
  MEMCLEAR((LUSOL->lenc+1), LUSOL->n);
#else
  for(I = 1; I <= LUSOL->m; I++)
    LUSOL->lenr[I] = ZERO;
  for(I = 1; I <= LUSOL->n; I++)
    LUSOL->lenc[I] = ZERO;
#endif

  *AMAX = 0;
  *NUMNZ = LUSOL->nelem;
  L = LUSOL->nelem+1;
  for(LDUMMY = 1; LDUMMY <= LUSOL->nelem; LDUMMY++) {
    L--;
    if(fabs(LUSOL->a[L])>SMALL) {
      I = LUSOL->indc[L];
      J = LUSOL->indr[L];
      SETMAX(*AMAX,fabs(LUSOL->a[L]));
      if(I<1 || I>LUSOL->m)
        goto x910;
      if(J<1 || J>LUSOL->n)
        goto x910;
      LUSOL->lenr[I]++;
      LUSOL->lenc[J]++;
    }
    else {
/*            Replace a negligible element by last element.  Since
              we are going backwards, we know the last element is ok. */
      LUSOL->a[L] = LUSOL->a[*NUMNZ];
      LUSOL->indc[L] = LUSOL->indc[*NUMNZ];
      LUSOL->indr[L] = LUSOL->indr[*NUMNZ];
      (*NUMNZ)--;
    }
  }
  *LERR = 0;
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  return;

x910:
  *LERR = L;
  *INFORM = LUSOL_INFORM_LUSINGULAR;
}

/* ==================================================================
   lu1or2  sorts a list of matrix elements  a(i,j)  into column
   order, given  numa  entries  a(i,j),  i,  j  in the parallel
   arrays  a, inum, jnum  respectively.  The matrix is assumed
   to have  n  columns and an arbitrary number of rows.
   On entry,  len(*)  must contain the length of each column.
   On exit,  a(*) and inum(*)  are sorted,  jnum(*) = 0,  and
   loc(j)  points to the start of column j.
   lu1or2  is derived from  mc20ad,  a routine in the Harwell
   Subroutine Library, author J. K. Reid.
   ------------------------------------------------------------------
   xx Feb 1985: Original version.
   17 Oct 2000: a, inum, jnum now have size lena to allow nelem = 0.
   ================================================================== */
void LU1OR2(LUSOLrec *LUSOL)
{
  REAL ACE, ACEP;
  int  L, J, I, JCE, ICE, ICEP, JCEP, JA, JB;

/*      Set  loc(j)  to point to the beginning of column  j. */
  L = 1;
  for(J = 1; J <= LUSOL->n; J++) {
    LUSOL->locc[J] = L;
    L += LUSOL->lenc[J];
  }
/*      Sort the elements into column order.
        The algorithm is an in-place sort and is of order  numa. */
  for(I = 1; I <= LUSOL->nelem; I++) {
/*         Establish the current entry. */
    JCE = LUSOL->indr[I];
    if(JCE==0)
      continue;
    ACE = LUSOL->a[I];
    ICE = LUSOL->indc[I];
    LUSOL->indr[I] = 0;
/*         Chain from current entry. */
    for(J = 1; J <= LUSOL->nelem; J++) {
/*            The current entry is not in the correct position.
              Determine where to store it. */
      L = LUSOL->locc[JCE];
      LUSOL->locc[JCE]++;
/*            Save the contents of that location. */
      ACEP = LUSOL->a[L];
      ICEP = LUSOL->indc[L];
      JCEP = LUSOL->indr[L];
/*            Store current entry. */
      LUSOL->a[L] = ACE;
      LUSOL->indc[L] = ICE;
      LUSOL->indr[L] = 0;
/*            If next current entry needs to be processed,
              copy it into current entry. */
      if(JCEP==0)
        break;
      ACE = ACEP;
      ICE = ICEP;
      JCE = JCEP;
    }
  }
/*      Reset loc(j) to point to the start of column j. */
  JA = 1;
  for(J = 1; J <= LUSOL->n; J++) {
    JB = LUSOL->locc[J];
    LUSOL->locc[J] = JA;
    JA = JB;
  }
}

/* ==================================================================
   lu1or3  looks for duplicate elements in an  m by n  matrix  A
   defined by the column list  indc, lenc, locc.
   iw  is used as a work vector of length  m.
   ------------------------------------------------------------------
   xx Feb 1985: Original version.
   17 Oct 2000: indc, indr now have size lena to allow nelem = 0.
   ================================================================== */
void LU1OR3(LUSOLrec *LUSOL, int *LERR, int *INFORM)
{
  int I, J, L1, L2, L;

#ifdef LUSOLFastClear
  MEMCLEAR((LUSOL->ip+1), LUSOL->m);
#else
  for(I = 1; I <= LUSOL->m; I++)
    LUSOL->ip[I] = ZERO;
#endif

  for(J = 1; J <= LUSOL->n; J++) {
    if(LUSOL->lenc[J]>0) {
      L1 = LUSOL->locc[J];
      L2 = (L1+LUSOL->lenc[J])-1;
      for(L = L1; L <= L2; L++) {
        I = LUSOL->indc[L];
        if(LUSOL->ip[I]==J)
          goto x910;
        LUSOL->ip[I] = J;
      }
    }
  }
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  return;
x910:
  *LERR = L;
  *INFORM = LUSOL_INFORM_LUSINGULAR;
}

/* ==================================================================
   lu1or4 constructs a row list  indr, locr
   from a corresponding column list  indc, locc,
   given the lengths of both columns and rows in  lenc, lenr.
   ------------------------------------------------------------------
   xx Feb 1985: Original version.
   17 Oct 2000: indc, indr now have size lena to allow nelem = 0.
   ================================================================== */
void LU1OR4(LUSOLrec *LUSOL)
{
  int L, I, L2, J, JDUMMY, L1, LR;

/*      Initialize  locr(i)  to point just beyond where the
        last component of row  i  will be stored. */
  L = 1;
  for(I = 1; I <= LUSOL->m; I++) {
    L += LUSOL->lenr[I];
    LUSOL->locr[I] = L;
  }
/*      By processing the columns backwards and decreasing  locr(i)
        each time it is accessed, it will end up pointing to the
        beginning of row  i  as required. */
  L2 = LUSOL->nelem;
  J = LUSOL->n+1;
  for(JDUMMY = 1; JDUMMY <= LUSOL->n; JDUMMY++) {
    J = J-1;
    if(LUSOL->lenc[J]>0) {
      L1 = LUSOL->locc[J];
      for(L = L1; L <= L2; L++) {
        I = LUSOL->indc[L];
        LR = LUSOL->locr[I]-1;
        LUSOL->locr[I] = LR;
        LUSOL->indr[LR] = J;
      }
      L2 = L1-1;
    }
  }
}

/* ==================================================================
   lu1pen deals with pending fill-in in the row file.
   ------------------------------------------------------------------
   ifill(ll) says if a row involved in the new column of L
             has to be updated.  If positive, it is the total
             length of the final updated row.
   jfill(lu) says if a column involved in the new row of U
             contains any pending fill-ins.  If positive, it points
             to the first fill-in in the column that has yet to be
             added to the row file.
   ------------------------------------------------------------------
   16 Apr 1989: First version of lu1pen.
   23 Mar 2001: ilast used and updated.
   ================================================================== */
void LU1PEN(LUSOLrec *LUSOL, int NSPARE, int *ILAST,
            int LPIVC1, int LPIVC2, int LPIVR1, int LPIVR2,
            int *LROW, int IFILL[], int JFILL[])
{
  int  LL, LC, L, I, LR1, LR2, LR, LU, J, LC1, LC2, LAST;

  LL = 0;
  for(LC = LPIVC1; LC <= LPIVC2; LC++) {
    LL++;
    if(IFILL[LL]==0)
      continue;
/*      Another row has pending fill.
        First, add some spare space at the }
        of the current last row. */
#if 1
    LC1 = (*LROW)+1;
    LC2 = (*LROW)+NSPARE;
    *LROW = LC2;
    for(L = LC1; L <= LC2; L++) {
#else
    for(L = (*LROW)+1; L <= (*LROW)+NSPARE; L++) {
      *LROW = L;  /* ******* ERROR ???? */
#endif
      LUSOL->indr[L] = 0;
    }
/*      Now move row i to the end of the row file. */
    I = LUSOL->indc[LC];
    *ILAST = I;
    LR1 = LUSOL->locr[I];
    LR2 = (LR1+LUSOL->lenr[I])-1;
    LUSOL->locr[I] = (*LROW)+1;
    for(LR = LR1; LR <= LR2; LR++) {
      (*LROW)++;
      LUSOL->indr[*LROW] = LUSOL->indr[LR];
      LUSOL->indr[LR] = 0;
    }
    (*LROW) += IFILL[LL];
  }
/*         Scan all columns of  D  and insert the pending fill-in
           into the row file. */
  LU = 1;
  for(LR = LPIVR1; LR <= LPIVR2; LR++) {
    LU++;
    if(JFILL[LU]==0)
      continue;
    J = LUSOL->indr[LR];
    LC1 = (LUSOL->locc[J]+JFILL[LU])-1;
    LC2 = (LUSOL->locc[J]+LUSOL->lenc[J])-1;
    for(LC = LC1; LC <= LC2; LC++) {
      I = LUSOL->indc[LC]-LUSOL->m;
      if(I>0) {
        LUSOL->indc[LC] = I;
        LAST = LUSOL->locr[I]+LUSOL->lenr[I];
        LUSOL->indr[LAST] = J;
        LUSOL->lenr[I]++;
      }
    }
  }
}


/* ==================================================================
   lu1fad  is a driver for the numerical phase of lu1fac.
   At each stage it computes a column of  L  and a row of  U,
   using a Markowitz criterion to select the pivot element,
   subject to a stability criterion that bounds the elements of  L.
   ------------------------------------------------------------------
   Local variables
   ---------------
   lcol   is the length of the column file.  It points to the last
          nonzero in the column list.
   lrow   is the analogous quantity for the row file.
   lfile  is the file length (lcol or lrow) after the most recent
          compression of the column list or row list.
   nrowd  and  ncold  are the number of rows and columns in the
          matrix defined by the pivot column and row.  They are the
          dimensions of the submatrix D being altered at this stage.
   melim  and  nelim  are the number of rows and columns in the
          same matrix D, excluding the pivot column and row.
   mleft  and  nleft  are the number of rows and columns
          still left to be factored.
   nzchng is the increase in nonzeros in the matrix that remains
          to be factored after the current elimination
          (usually negative).
   nzleft is the number of nonzeros still left to be factored.
   nspare is the space we leave at the end of the last row or
          column whenever a row or column is being moved to the }
          of its file.  nspare = 1 or 2 might help reduce the
          number of file compressions when storage is tight.
   The row and column ordering permutes A into the form
                      ------------------------
                       \                     |
                        \         U1         |
                         \                   |
                          --------------------
                          |\
                          | \
                          |  \
          P A Q   =       |   \
                          |    \
                          |     --------------
                          |     |            |
                          |     |            |
                          | L1  |     A2     |
                          |     |            |
                          |     |            |
                          --------------------
   where the block A2 is factored as  A2 = L2 U2.
   The phases of the factorization are as follows.
   Utri   is true when U1 is being determined.
          Any column of length 1 is accepted immediately (if TPP).
   Ltri   is true when L1 is being determined.
          lu1mar exits as soon as an acceptable pivot is found
          in a row of length 1.
   spars1 is true while the density of the (modified) A2 is less
          than the parameter dens1 = parmlu(7) = 0.3 say.
          lu1mar searches maxcol columns and maxrow rows,
          where  maxcol = luparm(3),  maxrow = maxcol - 1.
          lu1mxc is used to keep the biggest element at the top
          of all remaining columns.
   spars2 is true while the density of the modified A2 is less
          than the parameter dens2 = parmlu(8) = 0.6 say.
          lu1mar searches maxcol columns and no rows.
          lu1mxc could fix up only the first maxcol cols (with TPP).
          22 Sep 2000:  For simplicity, lu1mxc fixes all
                        modified cols.
   dense  is true once the density of A2 reaches dens2.
          lu1mar searches only 1 column (the shortest).
          lu1mxc could fix up only the first column (with TPP).
   ------------------------------------------------------------------
   00 Jan 1986  Version documented in LUSOL paper:
                Gill, Murray, Saunders and Wright (1987),
                Maintaining LU factors of a general sparse matrix,
                Linear algebra and its applications 88/89, 239-270.
   02 Feb 1989  Following Suhl and Aittoniemi (1987), the largest
                element in each column is now kept at the start of
                the column, i.e. in position locc(j) of a and indc.
                This should speed up the Markowitz searches.
                To save time on highly triangular matrices, we wait
                until there are no further columns of length 1
                before setting and maintaining that property.
   12 Apr 1989  ipinv and iqinv added (inverses of ip and iq)
                to save searching ip and iq for rows and columns
                altered in each elimination step.  (Used in lu1pq2)
   19 Apr 1989  Code segmented to reduce its size.
                lu1gau does most of the Gaussian elimination work.
                lu1mar does just the Markowitz search.
                lu1mxc moves biggest elements to top of columns.
                lu1pen deals with pending fill-in in the row list.
                lu1pq2 updates the row and column permutations.
   26 Apr 1989  maxtie replaced by maxcol, maxrow in the Markowitz
                search.  maxcol, maxrow change as density increases.
   25 Oct 1993  keepLU implemented.
   07 Feb 1994  Exit main loop early to finish off with a dense LU.
                densLU tells lu1fad whether to do it.
   21 Dec 1994  Bug fixed.  nrank was wrong after the call to lu1ful.
   12 Nov 1999  A parallel version of dcopy gave trouble in lu1ful
                during left-shift of dense matrix D within a(*).
                Fixed this unexpected problem here in lu1fad
                by making sure the first and second D don't overlap.
   13 Sep 2000  TCP (Threshold Complete Pivoting) implemented.
                lu2max added
                (finds aijmax from biggest elems in each col).
                Utri, Ltri and Spars1 phases apply.
                No switch to Dense CP yet.  (Only TPP switches.)
   14 Sep 2000  imax needed to remember row containing aijmax.
   22 Sep 2000  For simplicity, lu1mxc always fixes all modified cols.
                (TPP spars2 used to fix just the first maxcol cols.)
   08 Nov 2000: Speed up search for aijmax.
                Don't need to search all columns if the elimination
                didn't alter the col containing the current aijmax.
   21 Nov 2000: lu1slk implemented for Utri phase with TCP
                to guard against deceptive triangular matrices.
                (Utri used to have aijtol >= 0.9999 to include
                slacks, but this allows other 1s to be accepted.)
                Utri now accepts slacks, but applies normal aijtol
                test to other pivots.
   28 Nov 2000: TCP with empty cols must call lu1mxc and lu2max
                with ( lq1, n, ... ), not just ( 1, n, ... ).
   23 Mar 2001: lu1fad bug with TCP.
                A col of length 1 might not be accepted as a pivot.
                Later it appears in a pivot row and temporarily
                has length 0 (when pivot row is removed
                but before the column is filled in).  If it is the
                last column in storage, the preceding col also thinks
                it is "last".  Trouble arises when the preceding col
                needs fill-in -- it overlaps the real "last" column.
                (Very rarely, same trouble might have happened if
                the drop tolerance caused columns to have length 0.)
                Introduced ilast to record the last row in row file,
                           jlast to record the last col in col file.
                lu1rec returns ilast = indr(lrow + 1)
                            or jlast = indc(lcol + 1).
                (Should be an output parameter, but didn't want to
                alter lu1rec's parameter list.)
                lu1rec also treats empty rows or cols safely.
                (Doesn't eliminate them!)
   26 Apr 2002: Heap routines added for TCP.
                lu2max no longer needed.
                imax, jmax used only for printing.
   01 May 2002: lu1DCP implemented (dense complete pivoting).
                Both TPP and TCP now switch to dense LU
                when density exceeds dens2.
   06 May 2002: In dense mode, store diag(U) in natural order.
   09 May 2002: lu1mCP implemented (Markowitz TCP via heap).
   11 Jun 2002: lu1mRP implemented (Markowitz TRP).
   28 Jun 2002: Fixed call to lu1mxr.
   14 Dec 2002: lu1mSP implemented (Markowitz TSP).
   15 Dec 2002: Both TPP and TSP can grab cols of length 1
                during Utri.
   ================================================================== */
void LU1FAD(LUSOLrec *LUSOL,
#ifdef ClassicHamaxR
            int LENA2, int LENH, REAL HA[], int HJ[], int HK[], REAL AMAXR[],
#endif
            int *INFORM, int *LENL, int *LENU, int *MINLEN,
            int *MERSUM, int *NUTRI, int *NLTRI,
            int *NDENS1, int *NDENS2, int *NRANK,
            REAL *LMAX, REAL *UMAX, REAL *DUMAX, REAL *DUMIN, REAL *AKMAX)
{
  MYBOOL UTRI, LTRI, SPARS1, SPARS2, DENSE, DENSLU, KEEPLU, TCP, TPP, TRP,TSP;
  int    HLEN, HOPS, H, LPIV, LPRINT, MAXCOL, MAXROW, ILAST, JLAST, LFILE, LROW, LCOL,
         MINMN, MAXMN, NZLEFT, NSPARE, LU1, KK, J, LC, MLEFT, NLEFT, NROWU,
         LQ1, LQ2, JBEST, LQ, I, IBEST, MBEST, LEND, NFREE, LD, NCOLD, NROWD,
         MELIM, NELIM, JMAX, IMAX, LL1, LSAVE, LFREE, LIMIT, MINFRE, LPIVR, LPIVR1, LPIVR2,
         L, LPIVC, LPIVC1, LPIVC2, KBEST, LU, LR, LENJ, LC1, LAST, LL, LS,
         LENI, LR1, LFIRST, NFILL, NZCHNG, K, MRANK, NSING;
  REAL   LIJ, LTOL, SMALL, USPACE, DENS1, DENS2, AIJMAX, AIJTOL, AMAX, ABEST, DIAG, V;
#ifdef ClassicHamaxR
  int    LDIAGU;
#else
  int    LENA2 = LUSOL->lena;
#endif

#ifdef UseTimer
  int    eltime, mktime, ntime;
  timer ( "start", 3 );
  ntime = LUSOL->n / 4;
#endif

#ifdef ForceInitialization
  AIJMAX = 0;
  AIJTOL = 0;
  HLEN   = 0;
  JBEST  = 0;
  IBEST  = 0;
  MBEST  = 0;
  LEND   = 0;
  LD     = 0;
#endif

  LPRINT = LUSOL->luparm[LUSOL_IP_PRINTLEVEL];
  MAXCOL = LUSOL->luparm[LUSOL_IP_MARKOWITZ_MAXCOL];
  LPIV   = LUSOL->luparm[LUSOL_IP_PIVOTTYPE];
  KEEPLU = (MYBOOL) (LUSOL->luparm[LUSOL_IP_KEEPLU]!=FALSE);
/*      Threshold Partial   Pivoting (normal). */
  TPP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TPP);
/*      Threshold Rook      Pivoting */
  TRP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TRP);
/*      Threshold Complete  Pivoting. */
  TCP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TCP);
/*      Threshold Symmetric Pivoting. */
  TSP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TSP);
  DENSLU = FALSE;
  MAXROW = MAXCOL-1;
/*      Assume row m is last in the row file. */
  ILAST = LUSOL->m;
/*      Assume col n is last in the col file. */
  JLAST = LUSOL->n;
  LFILE = LUSOL->nelem;
  LROW = LUSOL->nelem;
  LCOL = LUSOL->nelem;
  MINMN = MIN(LUSOL->m,LUSOL->n);
  MAXMN = MAX(LUSOL->m,LUSOL->n);
  NZLEFT = LUSOL->nelem;
  NSPARE = 1;

  if(KEEPLU)
    LU1 = LENA2+1;
  else {
/*         Store only the diagonals of U in the top of memory. */
#ifdef ClassicdiagU
    LDIAGU = LENA2-LUSOL->n;
    LU1 = LDIAGU+1;
    LUSOL->diagU = LUSOL->a+LDIAGU;
#else
    LU1 = LENA2+1;
#endif
  }

  LTOL = LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij];
  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  USPACE = LUSOL->parmlu[LUSOL_RP_COMPSPACE_U];
  DENS1 = LUSOL->parmlu[LUSOL_RP_MARKOWITZ_CONLY];
  DENS2 = LUSOL->parmlu[LUSOL_RP_MARKOWITZ_DENSE];
  UTRI = TRUE;
  LTRI = FALSE;
  SPARS1 = FALSE;
  SPARS2 = FALSE;
  DENSE = FALSE;
/*      Check parameters. */
  SETMAX(LTOL,1.0001E+0);
  SETMIN(DENS1,DENS2);
/*      Initialize output parameters.
        lenL, lenU, minlen, mersum, nUtri, nLtri, ndens1, ndens2, nrank
        are already initialized by lu1fac. */
  *LMAX  = ZERO;
  *UMAX  = ZERO;
  *DUMAX = ZERO;
  *DUMIN = LUSOL_BIGNUM;
  if(LUSOL->nelem==0)
    *DUMIN = ZERO;
  *AKMAX = ZERO;
  HOPS = 0;
/*      More initialization.
        Don't worry yet about lu1mxc. */
  if(TPP || TSP) {
    AIJMAX = ZERO;
    AIJTOL = ZERO;
    HLEN = 1;
/*      TRP or TCP */
  }
  else {
/*      Move biggest element to top of each column.
        Set w(*) to mark slack columns (unit vectors). */
    LU1MXC(LUSOL, 1,LUSOL->n,LUSOL->iq);
    LU1SLK(LUSOL);
  }
  if(TRP)
/*      Find biggest element in each row. */
#ifdef ClassicHamaxR
    LU1MXR(LUSOL, 1,LUSOL->m,LUSOL->ip,AMAXR);
#else
    LU1MXR(LUSOL, 1,LUSOL->m,LUSOL->ip,LUSOL->amaxr);
#endif

  if(TCP) {
/*      Set Ha(1:Hlen) = biggest element in each column,
            Hj(1:Hlen) = corresponding column indices. */
    HLEN = 0;
    for(KK = 1; KK <= LUSOL->n; KK++) {
      HLEN++;
      J = LUSOL->iq[KK];
      LC = LUSOL->locc[J];
#ifdef ClassicHamaxR
      HA[HLEN] = fabs(LUSOL->a[LC]);
      HJ[HLEN] = J;
      HK[J] = HLEN;
#else
      LUSOL->Ha[HLEN] = fabs(LUSOL->a[LC]);
      LUSOL->Hj[HLEN] = J;
      LUSOL->Hk[J] = HLEN;
#endif
    }
/*      Build the heap, creating new Ha, Hj and setting Hk(1:Hlen). */
#ifdef ClassicHamaxR
    HBUILD(HA,HJ,HK,HLEN,&HOPS);
#else
    HBUILD(LUSOL->Ha,LUSOL->Hj,LUSOL->Hk,HLEN,&HOPS);
#endif
  }
/*      ------------------------------------------------------------------
        Start of main loop.
        ------------------------------------------------------------------ */
  MLEFT = LUSOL->m+1;
  NLEFT = LUSOL->n+1;
  for(NROWU = 1; NROWU <= MINMN; NROWU++) {
#ifdef UseTimer
    mktime = (nrowu / ntime) + 4;
    eltime = (nrowu / ntime) + 9;
#endif
    MLEFT--;
    NLEFT--;
/*         Bail out if there are no nonzero rows left. */
    if(LUSOL->iploc[1]>LUSOL->m)
      goto x900;
/*      For TCP, the largest Aij is at the top of the heap. */
   if(TCP) {
/*
              Marvelously easy */
#ifdef ClassicHamaxR
      AIJMAX = HA[1];
#else
      AIJMAX = LUSOL->Ha[1];
#endif
      SETMAX(*AKMAX,AIJMAX);
      AIJTOL = AIJMAX/LTOL;
    }
/*         ===============================================================
           Find a suitable pivot element.
           =============================================================== */
    if(UTRI) {
/*            ------------------------------------------------------------
              So far all columns have had length 1.
              We are still looking for the (backward) triangular part of A
              that forms the first rows and columns of U.
              ------------------------------------------------------------ */
      LQ1 = LUSOL->iqloc[1];
      LQ2 = LUSOL->n;
      if(LUSOL->m>1)
        LQ2 = LUSOL->iqloc[2]-1;
/*      There are more cols of length 1. */
      if(LQ1<=LQ2) {
        if(TPP || TSP) {
/*      Grab the first one. */
          JBEST = LUSOL->iq[LQ1];
/*      Scan all columns of length 1 ... TRP or TCP */
        }
        else {
          JBEST = 0;
          for(LQ = LQ1; LQ <= LQ2; LQ++) {
            J = LUSOL->iq[LQ];
/*      Accept a slack */
            if(LUSOL->w[J]>ZERO) {
              JBEST = J;
              goto x250;
            }
            LC = LUSOL->locc[J];
            AMAX = fabs(LUSOL->a[LC]);
            if(TRP) {
              I = LUSOL->indc[LC];
#ifdef ClassicHamaxR
              AIJTOL = AMAXR[I]/LTOL;
#else
              AIJTOL = LUSOL->amaxr[I]/LTOL;
#endif
            }
            if(AMAX>=AIJTOL) {
              JBEST = J;
              goto x250;
            }
          }
        }
x250:
        if(JBEST>0) {
          LC = LUSOL->locc[JBEST];
          IBEST = LUSOL->indc[LC];
          MBEST = 0;
          goto x300;
        }
      }
/*            This is the end of the U triangle.
              We will not return to this part of the code.
              TPP and TSP call lu1mxc for the first time
              (to move biggest element to top of each column). */
      if(LPRINT>=LUSOL_MSG_PIVOT)
        LUSOL_report(LUSOL, 0, "Utri ended.  spars1 = TRUE\n");
      UTRI = FALSE;
      LTRI = TRUE;
      SPARS1 = TRUE;
      *NUTRI = NROWU-1;
      if(TPP || TSP)
        LU1MXC(LUSOL, LQ1,LUSOL->n,LUSOL->iq);
    }
    if(SPARS1) {
/*            ------------------------------------------------------------
              Perform a Markowitz search.
              Search cols of length 1, then rows of length 1,
              then   cols of length 2, then rows of length 2, etc.
              ------------------------------------------------------------ */
#ifdef UseTimer
        timer ( "start", mktime );
#endif
/*      12 Jun 2002: Next line disables lu1mCP below
              if (TPP) then */
      if(TPP || TCP) {
        LU1MAR(LUSOL, MAXMN,TCP,AIJTOL,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST);
      }
      else if(TRP) {
#ifdef ClassicHamaxR
        LU1MRP(LUSOL, MAXMN,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST,AMAXR);
#else
        LU1MRP(LUSOL, MAXMN,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST,LUSOL->amaxr);
#endif
/*      else if (TCP) {
        lu1mCP( m    , n     , lena  , aijtol,
                      ibest, jbest , mbest ,
                      a    , indc  , indr  ,
                      lenc , lenr  , locc  ,
                      Hlen , Ha    , Hj    ) */
      }
      else if(TSP) {
        LU1MSP(LUSOL, MAXMN,LTOL,MAXCOL,&IBEST,&JBEST,&MBEST);
        if(IBEST==0)
          goto x990;
      }
#ifdef UseTimer
      timer ( "finish", mktime );
#endif
      if(LTRI) {
/*               So far all rows have had length 1.
                 We are still looking for the (forward) triangle of A
                 that forms the first rows and columns of L. */
        if(MBEST>0) {
          LTRI = FALSE;
          *NLTRI = NROWU-1-*NUTRI;
          if(LPRINT>=LUSOL_MSG_PIVOT)
            LUSOL_report(LUSOL, 0, "Ltri ended.\n");
        }
      }
      else {
/*               See if what's left is as dense as dens1. */
        if(NZLEFT>=(DENS1*MLEFT)*NLEFT) {
          SPARS1 = FALSE;
          SPARS2 = TRUE;
          *NDENS1 = NLEFT;
          MAXROW = 0;
          if(LPRINT>=LUSOL_MSG_PIVOT)
            LUSOL_report(LUSOL, 0, "spars1 ended.  spars2 = TRUE\n");
        }
      }
    }
    else if(SPARS2 || DENSE) {
/*            ------------------------------------------------------------
              Perform a restricted Markowitz search,
              looking at only the first maxcol columns.  (maxrow = 0.)
              ------------------------------------------------------------ */
#ifdef UseTimer
      timer ( "start", mktime );
#endif
/*      12 Jun 2002: Next line disables lu1mCP below
              if (TPP) then */
      if(TPP || TCP) {
        LU1MAR(LUSOL, MAXMN,TCP,AIJTOL,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST);
      }
      else if(TRP) {
#ifdef ClassicHamaxR
        LU1MRP(LUSOL, MAXMN,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST,AMAXR);
#else
        LU1MRP(LUSOL, MAXMN,LTOL,MAXCOL,MAXROW,&IBEST,&JBEST,&MBEST,LUSOL->amaxr);
#endif
/*      else if (TCP) {
        lu1mCP( m    , n     , lena  , aijtol,
                      ibest, jbest , mbest ,
                      a    , indc  , indr  ,
                      lenc , lenr  , locc  ,
                      Hlen , Ha    , Hj    ) */
      }
      else if(TSP) {
        LU1MSP(LUSOL, MAXMN,LTOL,MAXCOL,&IBEST,&JBEST,&MBEST);
        if(IBEST==0)
          goto x985;
      }
#ifdef UseTimer
      timer ( "finish", mktime );
#endif
/*            See if what's left is as dense as dens2. */
      if(SPARS2) {
        if(NZLEFT>=(DENS2*MLEFT)*NLEFT) {
          SPARS2 = FALSE;
          DENSE = TRUE;
          *NDENS2 = NLEFT;
          MAXCOL = 1;
          if(LPRINT>=LUSOL_MSG_PIVOT)
            LUSOL_report(LUSOL, 0, "spars2 ended.  dense = TRUE\n");
        }
      }
    }
/*         ---------------------------------------------------------------
           See if we can finish quickly.
           --------------------------------------------------------------- */
    if(DENSE) {
      LEND = MLEFT*NLEFT;
      NFREE = LU1-1;
      if(NFREE>=2*LEND) {
/*               There is room to treat the remaining matrix as
                 a dense matrix D.
                 We may have to compress the column file first.
                 12 Nov 1999: D used to be put at the
                              beginning of free storage (lD = lcol + 1).
                              Now put it at the end     (lD = lu1 - lenD)
                              so the left-shift in lu1ful will not
                              involve overlapping storage
                              (fatal with parallel dcopy).
   */
        DENSLU = TRUE;
        *NDENS2 = NLEFT;
        LD = LU1-LEND;
        if(LCOL>=LD) {
          LU1REC(LUSOL, LUSOL->n,TRUE,&LCOL,
                        LUSOL->indc,LUSOL->lenc,LUSOL->locc);
          LFILE = LCOL;
          JLAST = LUSOL->indc[LCOL+1];
        }
        goto x900;
      }
    }
/*         ===============================================================
           The best  aij  has been found.
           The pivot row  ibest  and the pivot column  jbest
           Define a dense matrix  D  of size  nrowd  by  ncold.
           =============================================================== */
x300:
    NCOLD = LUSOL->lenr[IBEST];
    NROWD = LUSOL->lenc[JBEST];
    MELIM = NROWD-1;
    NELIM = NCOLD-1;
    (*MERSUM) += MBEST;
    (*LENL) += MELIM;
    (*LENU) += NCOLD;
    if(LPRINT>=LUSOL_MSG_PIVOT) {
      if(NROWU==1)
        LUSOL_report(LUSOL, 0, "lu1fad debug:\n");
      if(TPP || TRP || TSP) {
        LUSOL_report(LUSOL, 0, "nrowu:%7d   i,jbest:%7d,%7d   nrowd,ncold:%6d,%6d\n",
                            NROWU, IBEST,JBEST, NROWD,NCOLD);
/*      TCP */
      }
      else {
#ifdef ClassicHamaxR
        JMAX = HJ[1];
#else
        JMAX = LUSOL->Hj[1];
#endif
        IMAX = LUSOL->indc[LUSOL->locc[JMAX]];
        LUSOL_report(LUSOL, 0, "nrowu:%7d   i,jbest:%7d,%7d   nrowd,ncold:%6d,%6d   i,jmax:%7d,%7d   aijmax:%g\n",
                            NROWU, IBEST,JBEST, NROWD,NCOLD, IMAX,JMAX, AIJMAX);
      }
    }
/*         ===============================================================
           Allocate storage for the next column of  L  and next row of  U.
           Initially the top of a, indc, indr are used as follows:
                      ncold       melim       ncold        melim
           a      |...........|...........|ujbest..ujn|li1......lim|
           indc   |...........|  lenr(i)  |  lenc(j)  |  markl(i)  |
           indr   |...........| iqloc(i)  |  jfill(j) |  ifill(i)  |
                 ^           ^             ^           ^            ^
                 lfree   lsave             lu1         ll1          oldlu1
           Later the correct indices are inserted:
           indc   |           |           |           |i1........im|
           indr   |           |           |jbest....jn|ibest..ibest|
           =============================================================== */
    if(!KEEPLU) {
/*            Always point to the top spot.
              Only the current column of L and row of U will
              take up space, overwriting the previous ones. */
#ifdef ClassicHamaxR
      LU1 = LDIAGU+1;
#else
      LU1 = LENA2+1;
#endif
    }
    /* Update (left-shift) pointers to make room for the new data */
    LL1 = LU1-MELIM;
    LU1 = LL1-NCOLD;
    LSAVE = LU1-NROWD;
    LFREE = LSAVE-NCOLD;

    /* Check if we need to allocate more memory, and allocate if necessary */
#if 0  /* Proposal by Michael A. Saunders (logic based on Markowitz' rule) */
    L = NROWD*NCOLD;

    /* Try to avoid future expansions by anticipating further updates - KE extension */
    if(LUSOL->luparm[LUSOL_IP_UPDATELIMIT] > 0)
#if 1
      L *= (int) (log(LUSOL->luparm[LUSOL_IP_UPDATELIMIT]-LUSOL->luparm[LUSOL_IP_UPDATECOUNT]+2.0) + 1);
#else
      L *= (LUSOL->luparm[LUSOL_IP_UPDATELIMIT]-LUSOL->luparm[LUSOL_IP_UPDATECOUNT]) / 2 + 1;
#endif

#else  /* Version by Kjell Eikland (from luparm[LUSOL_IP_MINIMUMLENA] and safety margin) */
    L  = (KEEPLU ? MAX(LROW, LCOL) + 2*(LUSOL->m+LUSOL->n) : 0);
    L *= LUSOL_MULT_nz_a;
    SETMAX(L, NROWD*NCOLD);
#endif

    /* Do the memory expansion */
    if((L > LFREE-LCOL) && LUSOL_expand_a(LUSOL, &L, &LFREE)) {
      LL1   += L;
      LU1   += L;
      LSAVE += L;
#ifdef ClassicdiagU
      LUSOL->diagU += L;
#endif
#ifdef ClassicHamaxR
      HA    += L;
      HJ    += L;
      HK    += L;
      AMAXR += L;
#endif
    }
    LIMIT = (int) (USPACE*LFILE)+LUSOL->m+LUSOL->n+1000;

/*         Make sure the column file has room.
           Also force a compression if its length exceeds a certain limit. */
#ifdef StaticMemAlloc
    MINFRE = NCOLD+MELIM;
#else
    MINFRE = NROWD*NCOLD;
#endif
    NFREE = LFREE-LCOL;
    if(NFREE<MINFRE || LCOL>LIMIT) {
      LU1REC(LUSOL, LUSOL->n,TRUE,&LCOL,
                    LUSOL->indc,LUSOL->lenc,LUSOL->locc);
      LFILE = LCOL;
      JLAST = LUSOL->indc[LCOL+1];
      NFREE = LFREE-LCOL;
      if(NFREE<MINFRE)
        goto x970;
    }
/*         Make sure the row file has room. */
#ifdef StaticMemAlloc
    MINFRE = NCOLD+MELIM;
#else
    MINFRE = NROWD*NCOLD;
#endif
    NFREE = LFREE-LROW;
    if(NFREE<MINFRE || LROW>LIMIT) {
      LU1REC(LUSOL, LUSOL->m,FALSE,&LROW,
                    LUSOL->indr,LUSOL->lenr,LUSOL->locr);
      LFILE = LROW;
      ILAST = LUSOL->indr[LROW+1];
      NFREE = LFREE-LROW;
      if(NFREE<MINFRE)
        goto x970;
    }
/*         ===============================================================
           Move the pivot element to the front of its row
           and to the top of its column.
           =============================================================== */
    LPIVR = LUSOL->locr[IBEST];
    LPIVR1 = LPIVR+1;
    LPIVR2 = LPIVR+NELIM;
    for(L = LPIVR; L <= LPIVR2; L++) {
      if(LUSOL->indr[L]==JBEST)
        break;
    }

    LUSOL->indr[L] = LUSOL->indr[LPIVR];
    LUSOL->indr[LPIVR] = JBEST;
    LPIVC = LUSOL->locc[JBEST];
    LPIVC1 = LPIVC+1;
    LPIVC2 = LPIVC+MELIM;
    for(L = LPIVC; L <= LPIVC2; L++) {
      if(LUSOL->indc[L]==IBEST)
        break;
    }
    LUSOL->indc[L] = LUSOL->indc[LPIVC];
    LUSOL->indc[LPIVC] = IBEST;
    ABEST = LUSOL->a[L];
    LUSOL->a[L] = LUSOL->a[LPIVC];
    LUSOL->a[LPIVC] = ABEST;
    if(!KEEPLU)
/*            Store just the diagonal of U, in natural order.
   !!         a[ldiagU + nrowu] = abest ! This was in pivot order. */
      LUSOL->diagU[JBEST] = ABEST;

/*     ==============================================================
        Delete pivot col from heap.
        Hk tells us where it is in the heap.
       ============================================================== */
    if(TCP) {
#ifdef ClassicHamaxR
      KBEST = HK[JBEST];
      HDELETE(HA,HJ,HK,&HLEN,KBEST,&H);
#else
      KBEST = LUSOL->Hk[JBEST];
      HDELETE(LUSOL->Ha,LUSOL->Hj,LUSOL->Hk,&HLEN,KBEST,&H);
#endif
      HOPS += H;
    }
/*         ===============================================================
           Delete the pivot row from the column file
           and store it as the next row of  U.
           set  indr(lu) = 0     to initialize jfill ptrs on columns of D,
                indc(lu) = lenj  to save the original column lengths.
           =============================================================== */
    LUSOL->a[LU1] = ABEST;
    LUSOL->indr[LU1] = JBEST;
    LUSOL->indc[LU1] = NROWD;
    LU = LU1;
    DIAG = fabs(ABEST);
    SETMAX(*UMAX,DIAG);
    SETMAX(*DUMAX,DIAG);
    SETMIN(*DUMIN,DIAG);
    for(LR = LPIVR1; LR <= LPIVR2; LR++) {
      LU++;
      J = LUSOL->indr[LR];
      LENJ = LUSOL->lenc[J];
      LUSOL->lenc[J] = LENJ-1;
      LC1 = LUSOL->locc[J];
      LAST = LC1+LUSOL->lenc[J];
      for(L = LC1; L <= LAST; L++) {
        if(LUSOL->indc[L]==IBEST)
          break;
      }
      LUSOL->a[LU] = LUSOL->a[L];
      LUSOL->indr[LU] = 0;
      LUSOL->indc[LU] = LENJ;
      SETMAX(*UMAX,fabs(LUSOL->a[LU]));
      LUSOL->a[L] = LUSOL->a[LAST];
      LUSOL->indc[L] = LUSOL->indc[LAST];
/*      Free entry */
      LUSOL->indc[LAST] = 0;
/* ???        if (j .eq. jlast) lcol = lcol - 1 */
    }
/*         ===============================================================
           Delete the pivot column from the row file
           and store the nonzeros of the next column of  L.
           Set  indc(ll) = 0     to initialize markl(*) markers,
                indr(ll) = 0     to initialize ifill(*) row fill-in cntrs,
                indc(ls) = leni  to save the original row lengths,
                indr(ls) = iqloc(i)    to save parts of  iqloc(*),
                iqloc(i) = lsave - ls  to point to the nonzeros of  L
                         = -1, -2, -3, ... in mark(*).
           =============================================================== */
    LUSOL->indc[LSAVE] = NCOLD;
    if(MELIM==0)
      goto x700;
    LL = LL1-1;
    LS = LSAVE;
    ABEST = ONE/ABEST;
    for(LC = LPIVC1; LC <= LPIVC2; LC++) {
      LL++;
      LS++;
      I = LUSOL->indc[LC];
      LENI = LUSOL->lenr[I];
      LUSOL->lenr[I] = LENI-1;
      LR1 = LUSOL->locr[I];
      LAST = LR1+LUSOL->lenr[I];
      for(L = LR1; L <= LAST; L++) {
        if(LUSOL->indr[L]==JBEST)
          break;
      }
      LUSOL->indr[L] = LUSOL->indr[LAST];
/*      Free entry */
      LUSOL->indr[LAST] = 0;
      LUSOL->a[LL] = -LUSOL->a[LC]*ABEST;
      LIJ = fabs(LUSOL->a[LL]);
      SETMAX(*LMAX,LIJ);
      LUSOL->indc[LL] = 0;
      LUSOL->indr[LL] = 0;
      LUSOL->indc[LS] = LENI;
      LUSOL->indr[LS] = LUSOL->iqloc[I];
      LUSOL->iqloc[I] = LSAVE-LS;
    }
/*         ===============================================================
           Do the Gaussian elimination.
           This involves adding a multiple of the pivot column
           to all other columns in the pivot row.
           Sometimes more than one call to lu1gau is needed to allow
           compression of the column file.
           lfirst  says which column the elimination should start with.
           minfre  is a bound on the storage needed for any one column.
           lu      points to off-diagonals of u.
           nfill   keeps track of pending fill-in in the row file.
           =============================================================== */
    if(NELIM==0)
      goto x700;
    LFIRST = LPIVR1;
    MINFRE = MLEFT+NSPARE;
    LU = 1;
    NFILL = 0;

x400:
#ifdef UseTimer
    timer ( "start", eltime );
#endif
    LU1GAU(LUSOL, MELIM,NSPARE,SMALL,LPIVC1,LPIVC2,&LFIRST,LPIVR2,
           LFREE,MINFRE,ILAST,&JLAST,&LROW,&LCOL,&LU,&NFILL,
           LUSOL->iqloc, LUSOL->a+LL1-LUSOL_ARRAYOFFSET,
           LUSOL->indc+LL1-LUSOL_ARRAYOFFSET, LUSOL->a+LU1-LUSOL_ARRAYOFFSET,
           LUSOL->indr+LL1-LUSOL_ARRAYOFFSET, LUSOL->indr+LU1-LUSOL_ARRAYOFFSET);
#ifdef UseTimer
    timer ( "finish", eltime );
#endif
    if(LFIRST>0) {
/*            The elimination was interrupted.
              Compress the column file and try again.
              lfirst, lu and nfill have appropriate new values. */
      LU1REC(LUSOL, LUSOL->n,TRUE,&LCOL,
                    LUSOL->indc,LUSOL->lenc,LUSOL->locc);
      LFILE = LCOL;
      JLAST = LUSOL->indc[LCOL+1];
      LPIVC = LUSOL->locc[JBEST];
      LPIVC1 = LPIVC+1;
      LPIVC2 = LPIVC+MELIM;
      NFREE = LFREE-LCOL;
      if(NFREE<MINFRE) {
          goto x970;
      }
      goto x400;
    }
/*         ===============================================================
           The column file has been fully updated.
           Deal with any pending fill-in in the row file.
           =============================================================== */
    if(NFILL>0) {
/*            Compress the row file if necessary.
              lu1gau has set nfill to be the number of pending fill-ins
              plus the current length of any rows that need to be moved. */
      MINFRE = NFILL;
      NFREE = LFREE-LROW;
      if(NFREE<MINFRE) {
        LU1REC(LUSOL, LUSOL->m,FALSE,&LROW,
                      LUSOL->indr,LUSOL->lenr,LUSOL->locr);
        LFILE = LROW;
        ILAST = LUSOL->indr[LROW+1];
        LPIVR = LUSOL->locr[IBEST];
        LPIVR1 = LPIVR+1;
        LPIVR2 = LPIVR+NELIM;
        NFREE = LFREE-LROW;
        if(NFREE<MINFRE) {
            goto x970;
        }
      }
/*            Move rows that have pending fill-in to end of the row file.
              Then insert the fill-in. */
      LU1PEN(LUSOL, NSPARE,&ILAST,
             LPIVC1,LPIVC2,LPIVR1,LPIVR2,
             &LROW,LUSOL->indr+LL1-LUSOL_ARRAYOFFSET,LUSOL->indr+LU1-LUSOL_ARRAYOFFSET);
    }
/*         ===============================================================
           Restore the saved values of  iqloc.
           Insert the correct indices for the col of L and the row of U.
           =============================================================== */
x700:
    LUSOL->lenr[IBEST] = 0;
    LUSOL->lenc[JBEST] = 0;
    LL = LL1-1;
    LS = LSAVE;
    for(LC = LPIVC1; LC <= LPIVC2; LC++) {
      LL++;
      LS++;
      I = LUSOL->indc[LC];
      LUSOL->iqloc[I] = LUSOL->indr[LS];
      LUSOL->indc[LL] = I;
      LUSOL->indr[LL] = IBEST;
    }
    LU = LU1-1;
    for(LR = LPIVR; LR <= LPIVR2; LR++) {
      LU++;
      LUSOL->indr[LU] = LUSOL->indr[LR];
    }
/*         ===============================================================
           Free the space occupied by the pivot row
           and update the column permutation.
           Then free the space occupied by the pivot column
           and update the row permutation.
           nzchng is found in both calls to lu1pq2, but we use it only
           after the second.
           =============================================================== */
    LU1PQ2(LUSOL, NCOLD, &NZCHNG,
           LUSOL->indr+LPIVR-LUSOL_ARRAYOFFSET,
           LUSOL->indc+LU1-LUSOL_ARRAYOFFSET, LUSOL->lenc,
           LUSOL->iqloc, LUSOL->iq, LUSOL->iqinv);
    LU1PQ2(LUSOL, NROWD, &NZCHNG,
           LUSOL->indc+LPIVC-LUSOL_ARRAYOFFSET,
           LUSOL->indc+LSAVE-LUSOL_ARRAYOFFSET, LUSOL->lenr,
           LUSOL->iploc, LUSOL->ip, LUSOL->ipinv);
    NZLEFT += NZCHNG;

/*         ===============================================================
           lu1mxr resets Amaxr(i) in each modified row i.
           lu1mxc moves the largest aij to the top of each modified col j.
           28 Jun 2002: Note that cols of L have an implicit diag of 1.0,
                        so lu1mxr is called with ll1, not ll1+1, whereas
                           lu1mxc is called with          lu1+1.
           =============================================================== */
    if(UTRI && TPP) {
/*      Relax -- we're not keeping big elements at the top yet. */
    }
    else {
      if(TRP && MELIM>0)
#ifdef ClassicHamaxR
        LU1MXR(LUSOL, LL1,LL,LUSOL->indc,AMAXR);
#else
        LU1MXR(LUSOL, LL1,LL,LUSOL->indc,LUSOL->amaxr);
#endif

      if(NELIM>0) {
        LU1MXC(LUSOL, LU1+1,LU,LUSOL->indr);
/*      Update modified columns in heap */
        if(TCP) {
          for(KK = LU1+1; KK <= LU; KK++) {
            J = LUSOL->indr[KK];
#ifdef ClassicHamaxR
            K = HK[J];
#else
            K = LUSOL->Hk[J];
#endif
/*      Biggest aij in column j */
            V = fabs(LUSOL->a[LUSOL->locc[J]]);
#ifdef ClassicHamaxR
            HCHANGE(HA,HJ,HK,HLEN,K,V,J,&H);
#else
            HCHANGE(LUSOL->Ha,LUSOL->Hj,LUSOL->Hk,HLEN,K,V,J,&H);
#endif
            HOPS += H;
          }
        }
      }
    }
/*         ===============================================================
           Negate lengths of pivot row and column so they will be
           eliminated during compressions.
           =============================================================== */
    LUSOL->lenr[IBEST] = -NCOLD;
    LUSOL->lenc[JBEST] = -NROWD;

/*         Test for fatal bug: row or column lists overwriting L and U. */
    if(LROW>LSAVE || LCOL>LSAVE)
      goto x980;

/*         Reset the file lengths if pivot row or col was at the end. */
    if(IBEST==ILAST)
      LROW = LUSOL->locr[IBEST];

    if(JBEST==JLAST)
      LCOL = LUSOL->locc[JBEST];

  }
/*      ------------------------------------------------------------------
        End of main loop.
        ------------------------------------------------------------------
        ------------------------------------------------------------------
        Normal exit.
        Move empty rows and cols to the end of ip, iq.
        Then finish with a dense LU if necessary.
        ------------------------------------------------------------------ */
x900:
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  LU1PQ3(LUSOL, LUSOL->m,LUSOL->lenr,LUSOL->ip,LUSOL->ipinv,&MRANK);
  LU1PQ3(LUSOL, LUSOL->n,LUSOL->lenc,LUSOL->iq,LUSOL->iqinv,NRANK);
  SETMIN(*NRANK, MRANK);
  if(DENSLU) {
#ifdef UseTimer
    timer ( "start", 17 );
#endif
    LU1FUL(LUSOL, LEND,LU1,TPP,MLEFT,NLEFT,*NRANK,NROWU,LENL,LENU,
           &NSING,KEEPLU,SMALL,LUSOL->a+LD-LUSOL_ARRAYOFFSET,LUSOL->locr);
/* ***     21 Dec 1994: Bug in next line.
   ***     nrank  = nrank - nsing */
    *NRANK = MINMN-NSING;
#ifdef UseTimer
    timer ( "finish", 17 );
#endif
  }
  *MINLEN = (*LENL)+(*LENU)+2*(LUSOL->m+LUSOL->n);
  goto x990;
/*      Not enough space free after a compress.
        Set  minlen  to an estimate of the necessary value of  lena. */
x970:
  *INFORM = LUSOL_INFORM_ANEEDMEM;
  *MINLEN = LENA2+LFILE+2*(LUSOL->m+LUSOL->n);
  goto x990;
/*      Fatal error.  This will never happen!
       (Famous last words.) */
x980:
  *INFORM = LUSOL_INFORM_FATALERR;
  goto x990;
/*      Fatal error with TSP.  Diagonal pivot not found. */
x985:
  *INFORM = LUSOL_INFORM_NOPIVOT;
/*      Exit. */
x990:
#ifdef UseTimer
  timer ( "finish", 3 );
#endif
;
}


/* ==================================================================
   lu1fac computes a factorization A = L*U, where A is a sparse
   matrix with m rows and n columns, P*L*P' is lower triangular
   and P*U*Q is upper triangular for certain permutations P, Q
   (which are returned in the arrays ip, iq).
   Stability is ensured by limiting the size of the elements of L.
   The nonzeros of A are input via the parallel arrays a, indc, indr,
   which should contain nelem entries of the form    aij,    i,    j
   in any order.  There should be no duplicate pairs         i,    j.

   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   +        Beware !!!   The row indices i must be in indc,         +
   +              and the column indices j must be in indr.         +
   +              (Not the other way round!)                        +
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

   It does not matter if some of the entries in a(*) are zero.
   Entries satisfying  abs( a(i) ) .le. parmlu(3)  are ignored.
   Other parameters in luparm and parmlu are described below.
   The matrix A may be singular.  On exit, nsing = luparm(11) gives
   the number of apparent singularities.  This is the number of
   "small" diagonals of the permuted factor U, as judged by
   the input tolerances Utol1 = parmlu(4) and  Utol2 = parmlu(5).
   The diagonal element diagj associated with column j of A is
   "small" if
               abs( diagj ) .le. Utol1
   or
               abs( diagj ) .le. Utol2 * max( uj ),
   where max( uj ) is the maximum element in the j-th column of U.
   The position of such elements is returned in w(*).  In general,
   w(j) = + max( uj ),  but if column j is a singularity,
   w(j) = - max( uj ).  Thus, w(j) .le. 0 if column j appears to be
   dependent on the other columns of A.
   NOTE: lu1fac (like certain other sparse LU packages) does not
   treat dense columns efficiently.  This means it will be slow
   on "arrow matrices" of the form
                A = (x       a)
                    (  x     b)
                    (    x   c)
                    (      x d)
                    (x x x x e)
   if the numerical values in the dense column allow it to be
   chosen LATE in the pivot order.
   With TPP (Threshold Partial Pivoting), the dense column is
   likely to be chosen late.
   With TCP (Threshold Complete Pivoting), if any of a,b,c,d
   is significantly larger than other elements of A, it will
   be chosen as the first pivot and the dense column will be
   eliminated, giving reasonably sparse factors.
   However, if element e is so big that TCP chooses it, the factors
   will become dense.  (It's hard to win on these examples!)
   ------------------------------------------------------------------

   Notes on the array names
   ------------------------
   During the LU factorization, the sparsity pattern of the matrix
   being factored is stored twice: in a column list and a row list.
   The column list is ( a, indc, locc, lenc )
   where
         a(*)    holds the nonzeros,
         indc(*) holds the indices for the column list,
         locc(j) points to the start of column j in a(*) and indc(*),
         lenc(j) is the number of nonzeros in column j.
   The row list is    (    indr, locr, lenr )
   where
         indr(*) holds the indices for the row list,
         locr(i) points to the start of row i in indr(*),
         lenr(i) is the number of nonzeros in row i.
   At all stages of the LU factorization, ip contains a complete
   row permutation.  At the start of stage k,  ip(1), ..., ip(k-1)
   are the first k-1 rows of the final row permutation P.
   The remaining rows are stored in an ordered list
                        ( ip, iploc, ipinv )
   where
         iploc(nz) points to the start in ip(*) of the set of rows
                   that currently contain nz nonzeros,
         ipinv(i)  points to the position of row i in ip(*).
   For example,
         iploc(1) = k   (and this is where rows of length 1 {),
         iploc(2) = k+p  if there are p rows of length 1
                        (and this is where rows of length 2 {).
   Similarly for iq, iqloc, iqinv.
   ---------------------------------------------------------------------
   INPUT PARAMETERS
   m      (not altered) is the number of rows in A.
   n      (not altered) is the number of columns in A.
   nelem  (not altered) is the number of matrix entries given in
          the arrays a, indc, indr.
   lena   (not altered) is the dimension of  a, indc, indr.
          This should be significantly larger than nelem.
          Typically one should have
             lena > max( 2*nelem, 10*m, 10*n, 10000 )
          but some applications may need more.
          On machines with virtual memory it is safe to have
          lena "far bigger than necessary", since not all of the
          arrays will be used.
   a      (overwritten) contains entries   Aij  in   a(1:nelem).
   indc   (overwritten) contains the indices i in indc(1:nelem).
   indr   (overwritten) contains the indices j in indr(1:nelem).
   luparm input parameters:                                Typical value
   luparm( 1) = nout     File number for printed messages.         6
   luparm( 2) = lprint   Print level.                              0
                    <  0 suppresses output.
                    =  0 gives error messages.
                   >= 10 gives statistics about the LU factors.
                   >= 50 gives debug output from lu1fac
                         (the pivot row and column and the
                         no. of rows and columns involved at
                         each elimination step).
   luparm( 3) = maxcol   lu1fac: maximum number of columns         5
                         searched allowed in a Markowitz-type
                         search for the next pivot element.
                         For some of the factorization, the
                         number of rows searched is
                         maxrow = maxcol - 1.
   luparm( 6) = 0    =>  TPP: Threshold Partial   Pivoting.        0
              = 1    =>  TRP: Threshold Rook      Pivoting.
              = 2    =>  TCP: Threshold Complete  Pivoting.
              = 3    =>  TSP: Threshold Symmetric Pivoting.
              = 4    =>  TDP: Threshold Diagonal  Pivoting.
                              (TDP not yet implemented).
                         TRP and TCP are more expensive than TPP but
                         more stable and better at revealing rank.
                         Take care with setting parmlu(1), especially
                         with TCP.
                         NOTE: TSP and TDP are for symmetric matrices
                         that are either definite or quasi-definite.
                         TSP is effectively TRP for symmetric matrices.
                         TDP is effectively TCP for symmetric matrices.
   luparm( 8) = keepLU   lu1fac: keepLU = 1 means the numerical    1
                         factors will be computed if possible.
                         keepLU = 0 means L and U will be discarded
                         but other information such as the row and
                         column permutations will be returned.
                         The latter option requires less storage.
   parmlu input parameters:                                Typical value
   parmlu( 1) = Ltol1    Max Lij allowed during Factor.
                                                   TPP     10.0 or 100.0
                                                   TRP      4.0 or  10.0
                                                   TCP      5.0 or  10.0
                                                   TSP      4.0 or  10.0
                         With TRP and TCP (Rook and Complete Pivoting),
                         values less than 25.0 may be expensive
                         on badly scaled data.  However,
                         values less than 10.0 may be needed
                         to obtain a reliable rank-revealing
                         factorization.
   parmlu( 2) = Ltol2    Max Lij allowed during Updates.            10.0
                         during updates.
   parmlu( 3) = small    Absolute tolerance for       eps**0.8 = 3.0d-13
                         treating reals as zero.
   parmlu( 4) = Utol1    Absolute tol for flagging    eps**0.67= 3.7d-11
                         small diagonals of U.
   parmlu( 5) = Utol2    Relative tol for flagging    eps**0.67= 3.7d-11
                         small diagonals of U.
                         (eps = machine precision)
   parmlu( 6) = Uspace   Factor limiting waste space in  U.      3.0
                         In lu1fac, the row or column lists
                         are compressed if their length
                         exceeds Uspace times the length of
                         either file after the last compression.
   parmlu( 7) = dens1    The density at which the Markowitz      0.3
                         pivot strategy should search maxcol
                         columns and no rows.
                         (Use 0.3 unless you are experimenting
                         with the pivot strategy.)
   parmlu( 8) = dens2    the density at which the Markowitz      0.5
                         strategy should search only 1 column,
                         or (if storage is available)
                         the density at which all remaining
                         rows and columns will be processed
                         by a dense LU code.
                         For example, if dens2 = 0.1 and lena is
                         large enough, a dense LU will be used
                         once more than 10 per cent of the
                         remaining matrix is nonzero.

   OUTPUT PARAMETERS
   a, indc, indr     contain the nonzero entries in the LU factors of A.
          If keepLU = 1, they are in a form suitable for use
          by other parts of the LUSOL package, such as lu6sol.
          U is stored by rows at the start of a, indr.
          L is stored by cols at the end   of a, indc.
          If keepLU = 0, only the diagonals of U are stored, at the
          end of a.
   ip, iq    are the row and column permutations defining the
          pivot order.  For example, row ip(1) and column iq(1)
          defines the first diagonal of U.
   lenc(1:numl0) contains the number of entries in nontrivial
          columns of L (in pivot order).
   lenr(1:m) contains the number of entries in each row of U
          (in original order).
   locc(1:n) = 0 (ready for the LU update routines).
   locr(1:m) points to the beginning of the rows of U in a, indr.
   iploc, iqloc, ipinv, iqinv  are undefined.
   w      indicates singularity as described above.
   inform = 0 if the LU factors were obtained successfully.
          = 1 if U appears to be singular, as judged by lu6chk.
          = 3 if some index pair indc(l), indr(l) lies outside
              the matrix dimensions 1:m , 1:n.
          = 4 if some index pair indc(l), indr(l) duplicates
              another such pair.
          = 7 if the arrays a, indc, indr were not large enough.
              Their length "lena" should be increase to at least
              the value "minlen" given in luparm(13).
          = 8 if there was some other fatal error.  (Shouldn't happen!)
          = 9 if no diagonal pivot could be found with TSP or TDP.
              The matrix must not be sufficiently definite
              or quasi-definite.
   luparm output parameters:
   luparm(10) = inform   Return code from last call to any LU routine.
   luparm(11) = nsing    No. of singularities marked in the
                         output array w(*).
   luparm(12) = jsing    Column index of last singularity.
   luparm(13) = minlen   Minimum recommended value for  lena.
   luparm(14) = maxlen   ?
   luparm(15) = nupdat   No. of updates performed by the lu8 routines.
   luparm(16) = nrank    No. of nonempty rows of U.
   luparm(17) = ndens1   No. of columns remaining when the density of
                         the matrix being factorized reached dens1.
   luparm(18) = ndens2   No. of columns remaining when the density of
                         the matrix being factorized reached dens2.
   luparm(19) = jumin    The column index associated with DUmin.
   luparm(20) = numL0    No. of columns in initial  L.
   luparm(21) = lenL0    Size of initial  L  (no. of nonzeros).
   luparm(22) = lenU0    Size of initial  U.
   luparm(23) = lenL     Size of current  L.
   luparm(24) = lenU     Size of current  U.
   luparm(25) = lrow     Length of row file.
   luparm(26) = ncp      No. of compressions of LU data structures.
   luparm(27) = mersum   lu1fac: sum of Markowitz merit counts.
   luparm(28) = nUtri    lu1fac: triangular rows in U.
   luparm(29) = nLtri    lu1fac: triangular rows in L.
   luparm(30) =
   parmlu output parameters:
   parmlu(10) = Amax     Maximum element in  A.
   parmlu(11) = Lmax     Maximum multiplier in current  L.
   parmlu(12) = Umax     Maximum element in current  U.
   parmlu(13) = DUmax    Maximum diagonal in  U.
   parmlu(14) = DUmin    Minimum diagonal in  U.
   parmlu(15) = Akmax    Maximum element generated at any stage
                         during TCP factorization.
   parmlu(16) = growth   TPP: Umax/Amax    TRP, TCP, TSP: Akmax/Amax
   parmlu(17) =
   parmlu(18) =
   parmlu(19) =
   parmlu(20) = resid    lu6sol: residual after solve with U or U'.
   ...
   parmlu(30) =
   ------------------------------------------------------------------
   00 Jun 1983  Original version.
   00 Jul 1987  nrank  saved in luparm(16).
   12 Apr 1989  ipinv, iqinv added as workspace.
   26 Apr 1989  maxtie replaced by maxcol in Markowitz search.
   16 Mar 1992  jumin  saved in luparm(19).
   10 Jun 1992  lu1fad has to move empty rows and cols to the bottom
                (via lu1pq3) before doing the dense LU.
   12 Jun 1992  Deleted dense LU (lu1ful, lu1vlu).
   25 Oct 1993  keepLU implemented.
   07 Feb 1994  Added new dense LU (lu1ful, lu1den).
   21 Dec 1994  Bugs fixed in lu1fad (nrank) and lu1ful (ipvt).
   08 Aug 1995  Use ip instead of w as parameter to lu1or3 (for F90).
   13 Sep 2000  TPP and TCP options implemented.
   17 Oct 2000  Fixed troubles due to A = empty matrix (Todd Munson).
   01 Dec 2000  Save Lmax, Umax, etc. after both lu1fad and lu6chk.
                lu1fad sets them when keepLU = false.
                lu6chk sets them otherwise, and includes items
                from the dense LU.
   11 Mar 2001  lu6chk now looks at diag(U) when keepLU = false.
   26 Apr 2002  New TCP implementation using heap routines to
                store largest element in each column.
                New workspace arrays Ha, Hj, Hk required.
                For compatibility, borrow space from a, indc, indr
                rather than adding new input parameters.
   01 May 2002  lu1den changed to lu1DPP (dense partial  pivoting).
                lu1DCP implemented       (dense complete pivoting).
                Both TPP and TCP now switch to dense mode and end.
   ================================================================== */
void LU1FAC(LUSOLrec *LUSOL, int *INFORM)
{
  MYBOOL  KEEPLU, TCP, TPP, TRP, TSP;
  int     LPIV, NELEM0, LPRINT, MINLEN, NUML0, LENL, LENU, LROW, MERSUM,
          NUTRI, NLTRI, NDENS1, NDENS2, NRANK, NSING, JSING, JUMIN, NUMNZ, LERR,
          LU, LL, LM, LTOPL, K, I, LENUK, J, LENLK, IDUMMY, LLSAVE, NMOVE, L2, L, NCP, NBUMP;
#ifdef ClassicHamaxR
  int     LENH, LENA2, LOCH, LMAXR;
#endif

  REAL    LMAX, LTOL, SMALL, AMAX, UMAX, DUMAX, DUMIN, AKMAX, DM, DN, DELEM, DENSTY,
          AGRWTH, UGRWTH, GROWTH, CONDU, DINCR, AVGMER;

/*      Free row-based version of L0 (regenerated by LUSOL_btran). */
  if(LUSOL->L0 != NULL)
    LUSOL_matfree(&(LUSOL->L0));

/*      Grab relevant input parameters. */
  NELEM0 = LUSOL->nelem;
  LPRINT = LUSOL->luparm[LUSOL_IP_PRINTLEVEL];
  LPIV   = LUSOL->luparm[LUSOL_IP_PIVOTTYPE];
  KEEPLU = (MYBOOL) (LUSOL->luparm[LUSOL_IP_KEEPLU]!=FALSE);
/*      Limit on size of Lij */
  LTOL   = LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij];
/*      Drop tolerance */
  SMALL  = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  TPP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TPP);
  TRP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TRP);
  TCP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TCP);
  TSP = (MYBOOL) (LPIV==LUSOL_PIVMOD_TSP);
/*      Initialize output parameters. */
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  LERR   = 0;
  MINLEN = LUSOL->nelem + 2*(LUSOL->m+LUSOL->n);
  NUML0  = 0;
  LENL   = 0;
  LENU   = 0;
  LROW   = 0;
  MERSUM = 0;
  NUTRI  = LUSOL->m;
  NLTRI  = 0;
  NDENS1 = 0;
  NDENS2 = 0;
  NRANK  = 0;
  NSING  = 0;
  JSING  = 0;
  JUMIN  = 0;
  AMAX   = ZERO;
  LMAX   = ZERO;
  UMAX   = ZERO;
  DUMAX  = ZERO;
  DUMIN  = ZERO;
  AKMAX  = ZERO;

/*      Float version of dimensions. */
  DM = LUSOL->m;
  DN = LUSOL->n;
  DELEM = LUSOL->nelem;

/*      Initialize workspace parameters. */
  LUSOL->luparm[LUSOL_IP_COMPRESSIONS_LU] = 0;
  if(LUSOL->lena < MINLEN) {
    if(!LUSOL_realloc_a(LUSOL, MINLEN))
      goto x970;
  }

/*      ------------------------------------------------------------------
        Organize the  aij's  in  a, indc, indr.
        lu1or1  deletes small entries, tests for illegal  i,j's,
                and counts the nonzeros in each row and column.
        lu1or2  reorders the elements of  A  by columns.
        lu1or3  uses the column list to test for duplicate entries
                (same indices  i,j).
        lu1or4  constructs a row list from the column list.
        ------------------------------------------------------------------ */
  LU1OR1(LUSOL, SMALL,&AMAX,&NUMNZ,&LERR,INFORM);
  if(LPRINT>=LUSOL_MSG_STATISTICS) {
    DENSTY = (100*DELEM)/(DM*DN);
    LUSOL_report(LUSOL, 0, "m:%6d %c n:%6d  nzcount:%9d  Amax:%g  Density:%g\n",
                           LUSOL->m, relationChar(LUSOL->m, LUSOL->n), LUSOL->n,
                           LUSOL->nelem, AMAX, DENSTY);
  }
  if(*INFORM!=LUSOL_INFORM_LUSUCCESS)
    goto x930;
  LUSOL->nelem = NUMNZ;
  LU1OR2(LUSOL);
  LU1OR3(LUSOL, &LERR,INFORM);
  if(*INFORM!=LUSOL_INFORM_LUSUCCESS)
    goto x940;
  LU1OR4(LUSOL);
/*      ------------------------------------------------------------------
        Set up lists of rows and columns with equal numbers of nonzeros,
        using  indc(*)  as workspace.
        ------------------------------------------------------------------ */
  LU1PQ1(LUSOL, LUSOL->m,LUSOL->n,LUSOL->lenr,
         LUSOL->ip,LUSOL->iploc,LUSOL->ipinv,
         LUSOL->indc+LUSOL->nelem); /* LUSOL_ARRAYOFFSET implied */
  LU1PQ1(LUSOL, LUSOL->n,LUSOL->m,LUSOL->lenc,
         LUSOL->iq,LUSOL->iqloc,LUSOL->iqinv,
         LUSOL->indc+LUSOL->nelem); /* LUSOL_ARRAYOFFSET implied */
/*      ------------------------------------------------------------------
        For TCP, Ha, Hj, Hk are allocated separately, similarly amaxr
        for TRP. Then compute the factorization  A = L*U.
        ------------------------------------------------------------------ */
#ifdef ClassicHamaxR
  if(TPP || TSP) {
    LENH  = 1;
    LENA2 = LUSOL->lena;
    LOCH  = LUSOL->lena;
    LMAXR = 1;
  }
  else if(TRP) {
    LENH  = 1;                     /* Dummy                                */
    LENA2 = LUSOL->lena-LUSOL->m;  /* Reduced length of      a             */
    LOCH  = LUSOL->lena;           /* Dummy                                */
    LMAXR = LENA2+1;               /* Start of Amaxr      in a             */
  }
  else if(TCP) {
    LENH  = LUSOL->n;              /* Length of heap                       */
    LENA2 = LUSOL->lena-LENH;      /* Reduced length of      a, indc, indr */
    LOCH  = LENA2+1;               /* Start of Ha, Hj, Hk in a, indc, indr */
    LMAXR = 1;                     /* Dummy                                */
  }
  LU1FAD(LUSOL,
         LENA2,LENH,
         LUSOL->a+LOCH-LUSOL_ARRAYOFFSET,
         LUSOL->indc+LOCH-LUSOL_ARRAYOFFSET,
         LUSOL->indr+LOCH-LUSOL_ARRAYOFFSET,
         LUSOL->a+LMAXR-LUSOL_ARRAYOFFSET,
         INFORM,&LENL,&LENU,
         &MINLEN,&MERSUM,&NUTRI,&NLTRI,&NDENS1,&NDENS2,
         &NRANK,&LMAX,&UMAX,&DUMAX,&DUMIN,&AKMAX);
#else
  LU1FAD(LUSOL,
         INFORM,&LENL,&LENU,
         &MINLEN,&MERSUM,&NUTRI,&NLTRI,&NDENS1,&NDENS2,
         &NRANK,&LMAX,&UMAX,&DUMAX,&DUMIN,&AKMAX);
#endif
  LUSOL->luparm[LUSOL_IP_RANK_U]     = NRANK;
  LUSOL->luparm[LUSOL_IP_NONZEROS_L] = LENL;
  if(*INFORM==LUSOL_INFORM_ANEEDMEM)
    goto x970;
  if(*INFORM==LUSOL_INFORM_NOPIVOT)
    goto x985;
  if(*INFORM>LUSOL_INFORM_LUSUCCESS)
    goto x980;
  if(KEEPLU) {
/*         ---------------------------------------------------------------
           The LU factors are at the top of  a, indc, indr,
           with the columns of  L  and the rows of  U  in the order
           ( free )   ... ( u3 ) ( l3 ) ( u2 ) ( l2 ) ( u1 ) ( l1 ).
           Starting with ( l1 ) and ( u1 ), move the rows of  U  to the
           left and the columns of  L  to the right, giving
           ( u1 ) ( u2 ) ( u3 ) ...   ( free )   ... ( l3 ) ( l2 ) ( l1 ).
           Also, set  numl0 = the number of nonempty columns of  U.
           --------------------------------------------------------------- */
    LU = 0;
    LL = LUSOL->lena+1;
#ifdef ClassicHamaxR
    LM = LENA2+1;
#else
    LM = LL;
#endif
    LTOPL = LL-LENL-LENU;
    LROW = LENU;
    for(K = 1; K <= NRANK; K++) {
      I = LUSOL->ip[K];
      LENUK = -LUSOL->lenr[I];
      LUSOL->lenr[I] = LENUK;
      J = LUSOL->iq[K];
      LENLK = -LUSOL->lenc[J]-1;
      if(LENLK>0) {
        NUML0++;
        LUSOL->iqloc[NUML0] = LENLK;
      }
      if(LU+LENUK<LTOPL) {
/*               =========================================================
                 There is room to move ( uk ).  Just right-shift ( lk ).
                 ========================================================= */
        for(IDUMMY = 1; IDUMMY <= LENLK; IDUMMY++) {
          LL--;
          LM--;
          LUSOL->a[LL] = LUSOL->a[LM];
          LUSOL->indc[LL] = LUSOL->indc[LM];
          LUSOL->indr[LL] = LUSOL->indr[LM];
        }
      }
      else {
/*               =========================================================
                 There is no room for ( uk ) yet.  We have to
                 right-shift the whole of the remaining LU file.
                 Note that ( lk ) ends up in the correct place.
                 ========================================================= */
        LLSAVE = LL-LENLK;
        NMOVE = LM-LTOPL;
        for(IDUMMY = 1; IDUMMY <= NMOVE; IDUMMY++) {
          LL--;
          LM--;
          LUSOL->a[LL] = LUSOL->a[LM];
          LUSOL->indc[LL] = LUSOL->indc[LM];
          LUSOL->indr[LL] = LUSOL->indr[LM];
        }
        LTOPL = LL;
        LL = LLSAVE;
        LM = LL;
      }
/*            ======================================================
              Left-shift ( uk ).
              ====================================================== */
      LUSOL->locr[I] = LU+1;
      L2 = LM-1;
      LM = LM-LENUK;
      for(L = LM; L <= L2; L++) {
        LU = LU+1;
        LUSOL->a[LU] = LUSOL->a[L];
        LUSOL->indr[LU] = LUSOL->indr[L];
      }
    }
/*         ---------------------------------------------------------------
           Save the lengths of the nonempty columns of  L,
           and initialize  locc(j)  for the LU update routines.
           --------------------------------------------------------------- */
    for(K = 1; K <= NUML0; K++) {
      LUSOL->lenc[K] = LUSOL->iqloc[K];
    }
    for(J = 1; J <= LUSOL->n; J++) {
      LUSOL->locc[J] = 0;
    }
/*         ---------------------------------------------------------------
           Test for singularity.
           lu6chk  sets  nsing, jsing, jumin, Lmax, Umax, DUmax, DUmin
           (including entries from the dense LU).
           inform = 1  if there are singularities (nsing gt 0).
           --------------------------------------------------------------- */
    LU6CHK(LUSOL, 1,LUSOL->lena,INFORM);
    NSING = LUSOL->luparm[LUSOL_IP_SINGULARITIES];
    JSING = LUSOL->luparm[LUSOL_IP_SINGULARINDEX];
    JUMIN = LUSOL->luparm[LUSOL_IP_COLINDEX_DUMIN];
    LMAX  = LUSOL->parmlu[LUSOL_RP_MAXMULT_L];
    UMAX  = LUSOL->parmlu[LUSOL_RP_MAXELEM_U];
    DUMAX = LUSOL->parmlu[LUSOL_RP_MAXELEM_DIAGU];
    DUMIN = LUSOL->parmlu[LUSOL_RP_MINELEM_DIAGU];
  }
  else {
/*         ---------------------------------------------------------------
           keepLU = 0.  L and U were not kept, just the diagonals of U.
           lu1fac will probably be called again soon with keepLU = .true.
           11 Mar 2001: lu6chk revised.  We can call it with keepLU = 0,
                        but we want to keep Lmax, Umax from lu1fad.
           05 May 2002: Allow for TCP with new lu1DCP.  Diag(U) starts
                        below lena2, not lena.  Need lena2 in next line.
           --------------------------------------------------------------- */
#ifdef ClassicHamaxR
    LU6CHK(LUSOL, 1,LENA2,INFORM);
#else
    LU6CHK(LUSOL, 1,LUSOL->lena,INFORM);
#endif
    NSING = LUSOL->luparm[LUSOL_IP_SINGULARITIES];
    JSING = LUSOL->luparm[LUSOL_IP_SINGULARINDEX];
    JUMIN = LUSOL->luparm[LUSOL_IP_COLINDEX_DUMIN];
    DUMAX = LUSOL->parmlu[LUSOL_RP_MAXELEM_DIAGU];
    DUMIN = LUSOL->parmlu[LUSOL_RP_MINELEM_DIAGU];
  }
  goto x990;
/*      ------------
        Error exits.
        ------------ */
x930:
  *INFORM = LUSOL_INFORM_ADIMERR;
  if(LPRINT>=LUSOL_MSG_SINGULARITY)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\nentry  a[%d]  has an illegal row (%d) or column (%d) index\n",
                        LERR,LUSOL->indc[LERR],LUSOL->indr[LERR]);
  goto x990;
x940:
  *INFORM = LUSOL_INFORM_ADUPLICATE;
  if(LPRINT>=LUSOL_MSG_SINGULARITY)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\nentry  a[%d]  is a duplicate with indeces indc=%d, indr=%d\n",
                        LERR,LUSOL->indc[LERR],LUSOL->indr[LERR]);
  goto x990;
x970:
  *INFORM = LUSOL_INFORM_ANEEDMEM;
  if(LPRINT>=LUSOL_MSG_SINGULARITY)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\ninsufficient storage; increase  lena  from %d to at least %d\n",
                        LUSOL->lena, MINLEN);
  goto x990;
x980:
  *INFORM = LUSOL_INFORM_FATALERR;
  if(LPRINT>=LUSOL_MSG_SINGULARITY)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\nfatal bug   (sorry --- this should never happen)\n");
  goto x990;
x985:
  *INFORM = LUSOL_INFORM_NOPIVOT;
  if(LPRINT>=LUSOL_MSG_SINGULARITY)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\nTSP used but diagonal pivot could not be found\n");

/*      Finalize and store output parameters. */
x990:
  LUSOL->nelem = NELEM0;
  LUSOL->luparm[LUSOL_IP_SINGULARITIES]   = NSING;
  LUSOL->luparm[LUSOL_IP_SINGULARINDEX]   = JSING;
  LUSOL->luparm[LUSOL_IP_MINIMUMLENA]     = MINLEN;
  LUSOL->luparm[LUSOL_IP_UPDATECOUNT]     = 0;
  LUSOL->luparm[LUSOL_IP_RANK_U]          = NRANK;
  LUSOL->luparm[LUSOL_IP_COLCOUNT_DENSE1] = NDENS1;
  LUSOL->luparm[LUSOL_IP_COLCOUNT_DENSE2] = NDENS2;
  LUSOL->luparm[LUSOL_IP_COLINDEX_DUMIN]  = JUMIN;
  LUSOL->luparm[LUSOL_IP_COLCOUNT_L0]     = NUML0;
  LUSOL->luparm[LUSOL_IP_ROWCOUNT_L0]     = 0;
  LUSOL->luparm[LUSOL_IP_NONZEROS_L0]     = LENL;
  LUSOL->luparm[LUSOL_IP_NONZEROS_U0]     = LENU;
  LUSOL->luparm[LUSOL_IP_NONZEROS_L]      = LENL;
  LUSOL->luparm[LUSOL_IP_NONZEROS_U]      = LENU;
  LUSOL->luparm[LUSOL_IP_NONZEROS_ROW]    = LROW;
  LUSOL->luparm[LUSOL_IP_MARKOWITZ_MERIT] = MERSUM;
  LUSOL->luparm[LUSOL_IP_TRIANGROWS_U]    = NUTRI;
  LUSOL->luparm[LUSOL_IP_TRIANGROWS_L]    = NLTRI;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_A]       = AMAX;
  LUSOL->parmlu[LUSOL_RP_MAXMULT_L]       = LMAX;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_U]       = UMAX;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_DIAGU]   = DUMAX;
  LUSOL->parmlu[LUSOL_RP_MINELEM_DIAGU]   = DUMIN;
  LUSOL->parmlu[LUSOL_RP_MAXELEM_TCP]     = AKMAX;
  AGRWTH = AKMAX/(AMAX+LUSOL_SMALLNUM);
  UGRWTH = UMAX/(AMAX+LUSOL_SMALLNUM);
  if(TPP)
    GROWTH = UGRWTH;
/*      TRP or TCP or TSP */
  else
    GROWTH = AGRWTH;
  LUSOL->parmlu[LUSOL_RP_GROWTHRATE]      = GROWTH;

  LUSOL->luparm[LUSOL_IP_FTRANCOUNT]      = 0;
  LUSOL->luparm[LUSOL_IP_BTRANCOUNT]      = 0;

/*      ------------------------------------------------------------------
        Set overall status variable.
        ------------------------------------------------------------------ */
  LUSOL->luparm[LUSOL_IP_INFORM]          = *INFORM;
  if(*INFORM == LUSOL_INFORM_NOMEMLEFT)
    LUSOL_report(LUSOL, 0, "lu1fac  error...\ninsufficient memory available\n");

/*      ------------------------------------------------------------------
        Print statistics for the LU factors.
        ------------------------------------------------------------------ */
  NCP   = LUSOL->luparm[LUSOL_IP_COMPRESSIONS_LU];
  CONDU = DUMAX/MAX(DUMIN,LUSOL_SMALLNUM);
  DINCR = (LENL+LENU)-LUSOL->nelem;
  DINCR = (DINCR*100)/MAX(DELEM,ONE);
  AVGMER = MERSUM;
  AVGMER = AVGMER/DM;
  NBUMP = LUSOL->m-NUTRI-NLTRI;
  if(LPRINT>=LUSOL_MSG_STATISTICS) {
    if(TPP) {
      LUSOL_report(LUSOL, 0, "Merit %g %d %d %d %g %d %d %g %g %d %d %d\n",
                          AVGMER,LENL,LENL+LENU,NCP,DINCR,NUTRI,LENU,
                          LTOL,UMAX,UGRWTH,NLTRI,NDENS1,LMAX);
    }
    else {
      LUSOL_report(LUSOL, 0, "Merit %s %g %d %d %d %g %d %d %g %g %d %d %d %g %g\n",
                          LUSOL_pivotLabel(LUSOL),
                          AVGMER,LENL,LENL+LENU,NCP,DINCR,NUTRI,LENU,
                          LTOL,UMAX,UGRWTH,NLTRI,NDENS1,LMAX,AKMAX,AGRWTH);
    }
    LUSOL_report(LUSOL, 0, "bump%9d  dense2%7d  DUmax%g DUmin%g  conDU%g\n",
                          NBUMP,NDENS2,DUMAX,DUMIN,CONDU);
  }
}


