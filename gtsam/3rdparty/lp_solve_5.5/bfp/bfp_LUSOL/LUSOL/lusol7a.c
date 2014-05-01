
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   File  lusol7a
      lu7add   lu7cyc   lu7elm   lu7for   lu7rnk   lu7zap
      Utilities for LUSOL's update routines.
      lu7for is the most important -- the forward sweep.
  01 May 2002: Derived from LUSOL's original lu7a.f file.
  01 May 2002: Current version of lusol7a.f.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

/* ==================================================================
   lu7add  inserts the first nrank elements of the vector v(*)
   as column  jadd  of  U.  We assume that  U  does not yet have any
   entries in this column.
   Elements no larger than  parmlu(3)  are treated as zero.
   klast  will be set so that the last row to be affected
   (in pivotal order) is row  ip(klast).
   ------------------------------------------------------------------
   09 May 1988: First f77 version.
   ================================================================== */
void LU7ADD(LUSOLrec *LUSOL, int JADD, REAL V[], int LENL, int *LENU,
  int *LROW, int NRANK, int *INFORM, int *KLAST, REAL *VNORM)
{
  REAL SMALL;
  int  K, I, LENI, MINFRE, NFREE, LR1, LR2, L;
#ifndef LUSOLFastMove
  int J;
#endif

  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  *VNORM = ZERO;
  *KLAST = 0;
  for(K = 1; K <= NRANK; K++) {
    I = LUSOL->ip[K];
    if(fabs(V[I])<=SMALL)
      continue;
    *KLAST = K;
    (*VNORM) += fabs(V[I]);
    LENI = LUSOL->lenr[I];
/*         Compress row file if necessary. */
    MINFRE = LENI+1;
    NFREE = LUSOL->lena - LENL - *LROW;
    if(NFREE<MINFRE) {
      LU1REC(LUSOL, LUSOL->m, TRUE,LROW,LUSOL->indr,LUSOL->lenr,LUSOL->locr);
      NFREE = LUSOL->lena - LENL - *LROW;
      if(NFREE<MINFRE)
        goto x970;
    }
/*         Move row  i  to the end of the row file,
           unless it is already there.
           No need to move if there is a gap already. */
    if(LENI==0)
      LUSOL->locr[I] = (*LROW) + 1;
    LR1 = LUSOL->locr[I];
    LR2 = (LR1+LENI)-1;
    if(LR2==*LROW)
      goto x150;
    if(LUSOL->indr[LR2+1]==0)
      goto x180;
    LUSOL->locr[I] = (*LROW) + 1;
#ifdef LUSOLFastMove
    L = LR2-LR1+1;
    if(L > 0) {
      LR2 = (*LROW)+1;
      MEMMOVE(LUSOL->a+LR2,    LUSOL->a+LR1, L);
      MEMMOVE(LUSOL->indr+LR2, LUSOL->indr+LR1, L);
      MEMCLEAR(LUSOL->indr+LR1, L);
      *LROW += L;
    }
#else
    for(L = LR1; L <= LR2; L++) {
      (*LROW)++;
      LUSOL->a[*LROW] = LUSOL->a[L];
      J = LUSOL->indr[L];
      LUSOL->indr[L] = 0;
      LUSOL->indr[*LROW] = J;
    }
#endif
x150:
    LR2 = *LROW;
    (*LROW)++;
/*         Add the element of  v. */
x180:
    LR2++;
    LUSOL->a[LR2] = V[I];
    LUSOL->indr[LR2] = JADD;
    LUSOL->lenr[I] = LENI+1;
    (*LENU)++;
  }
/*      Normal exit. */
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  goto x990;
/*      Not enough storage. */
x970:
  *INFORM = LUSOL_INFORM_ANEEDMEM;
x990:
;
}

/* ==================================================================
   lu7cyc performs a cyclic permutation on the row or column ordering
   stored in ip, moving entry kfirst down to klast.
   If kfirst .ge. klast, lu7cyc should not be called.
   Sometimes klast = 0 and nothing should happen.
   ------------------------------------------------------------------
   09 May 1988: First f77 version.
   ================================================================== */
void LU7CYC(LUSOLrec *LUSOL, int KFIRST, int KLAST, int IX[])
{
  if(KFIRST<KLAST) {
    int IFIRST, K;
#ifdef LUSOLFastMove
#if 1
    IFIRST = IX[KFIRST];
    K = KLAST-KFIRST;
    MEMMOVE(IX+KFIRST, IX+KFIRST+1, K);
    IX[KLAST] = IFIRST;
#else
    int *IXK, *IXK1;
    IXK = IX+KFIRST;
    IFIRST = *IXK;
    for(K = KFIRST, IXK1 = IXK+1; K <= KLAST-1; K++, IXK++, IXK1++) {
      *IXK = *IXK1;
    }
    *IXK = IFIRST;
#endif
#else
    IFIRST = IX[KFIRST];
    for(K = KFIRST; K <= KLAST-1; K++) {
      IX[K] = IX[K+1];
    }
    IX[KLAST] = IFIRST;
#endif
  }
}

/* ==================================================================
   lu7elm  eliminates the subdiagonal elements of a vector  v(*),
   where  L*v = y  for some vector y.
   If  jelm > 0,  y  has just become column  jelm  of the matrix  A.
   lu7elm  should not be called unless  m  is greater than  nrank.
   inform = 0 if y contained no subdiagonal nonzeros to eliminate.
   inform = 1 if y contained at least one nontrivial subdiagonal.
   inform = 7 if there is insufficient storage.
   ------------------------------------------------------------------
   09 May 1988: First f77 version.
                No longer calls lu7for at end.  lu8rpc, lu8mod do so.
   ================================================================== */
void LU7ELM(LUSOLrec *LUSOL, int JELM, REAL V[], int *LENL,
            int *LROW, int NRANK, int *INFORM, REAL *DIAG)
{
  REAL VI, VMAX, SMALL;
  int  NRANK1, MINFRE, NFREE, KMAX, L, K, I, LMAX, IMAX, L1, L2;

#ifdef ForceInitialization
  LMAX = 0;
#endif

  SMALL = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  NRANK1 = NRANK+1;
  *DIAG = ZERO;
/*      Compress row file if necessary. */
  MINFRE = LUSOL->m-NRANK;
  NFREE = LUSOL->lena-(*LENL)-(*LROW);
  if(NFREE>=MINFRE)
    goto x100;
  LU1REC(LUSOL, LUSOL->m,TRUE,LROW,LUSOL->indr,LUSOL->lenr,LUSOL->locr);
  NFREE = LUSOL->lena-(*LENL)-(*LROW);
  if(NFREE<MINFRE)
    goto x970;
    
/*      Pack the subdiagonals of  v  into  L,  and find the largest. */
x100:
  VMAX = ZERO;
  KMAX = 0;
  L = (LUSOL->lena-(*LENL))+1;
  for(K = NRANK1; K <= LUSOL->m; K++) {
    I = LUSOL->ip[K];
    VI = fabs(V[I]);
    if(VI<=SMALL)
      continue;
    L--;
    LUSOL->a[L] = V[I];
    LUSOL->indc[L] = I;
    if(VMAX>=VI)
      continue;
    VMAX = VI;
    KMAX = K;
    LMAX = L;
  }
  if(KMAX==0)
    goto x900;
/*      ------------------------------------------------------------------
        Remove  vmax  by overwriting it with the last packed  v(i).
        Then set the multipliers in  L  for the other elements.
        ------------------------------------------------------------------ */
  IMAX = LUSOL->ip[KMAX];
  VMAX = LUSOL->a[LMAX];
  LUSOL->a[LMAX] = LUSOL->a[L];
  LUSOL->indc[LMAX] = LUSOL->indc[L];
  L1 = L+1;
  L2 = LUSOL->lena-(*LENL);
  *LENL = ((*LENL)+L2)-L;
  for(L = L1; L <= L2; L++) {
    LUSOL->a[L] /= -VMAX;
    LUSOL->indr[L] = IMAX;
  }
/*      Move the row containing vmax to pivotal position nrank + 1. */
  LUSOL->ip[KMAX] = LUSOL->ip[NRANK1];
  LUSOL->ip[NRANK1] = IMAX;
  *DIAG = VMAX;
/*      ------------------------------------------------------------------
        If jelm is positive, insert  vmax  into a new row of  U.
        This is now the only subdiagonal element.
        ------------------------------------------------------------------ */
  if(JELM>0) {
    (*LROW)++;
    LUSOL->locr[IMAX] = *LROW;
    LUSOL->lenr[IMAX] = 1;
    LUSOL->a[*LROW] = VMAX;
    LUSOL->indr[*LROW] = JELM;
  }
  *INFORM = LUSOL_INFORM_LUSINGULAR;
  goto x990;
/*      No elements to eliminate. */
x900:
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  goto x990;
/*      Not enough storage. */
x970:
  *INFORM = LUSOL_INFORM_ANEEDMEM;
x990:
;
}

/* ==================================================================
   lu7for  (forward sweep) updates the LU factorization  A = L*U
   when row  iw = ip(klast)  of  U  is eliminated by a forward
   sweep of stabilized row operations, leaving  ip * U * iq  upper
   triangular.
   The row permutation  ip  is updated to preserve stability and/or
   sparsity.  The column permutation  iq  is not altered.
   kfirst  is such that row  ip(kfirst)  is the first row involved
   in eliminating row  iw.  (Hence,  kfirst  marks the first nonzero
   in row  iw  in pivotal order.)  If  kfirst  is unknown it may be
   input as  1.
   klast   is such that row  ip(klast)  is the row being eliminated.
   klast   is not altered.
   lu7for  should be called only if  kfirst .le. klast.
   If  kfirst = klast,  there are no nonzeros to eliminate, but the
   diagonal element of row  ip(klast)  may need to be moved to the
   front of the row.
   ------------------------------------------------------------------
   On entry,  locc(*)  must be zero.

   On exit:
   inform = 0  if row iw has a nonzero diagonal (could be small).
   inform = 1  if row iw has no diagonal.
   inform = 7  if there is not enough storage to finish the update.

   On a successful exit (inform le 1),  locc(*)  will again be zero.
   ------------------------------------------------------------------
      Jan 1985: Final f66 version.
   09 May 1988: First f77 version.
   ================================================================== */
void LU7FOR(LUSOLrec *LUSOL, int KFIRST, int KLAST, int *LENL, int *LENU,
                     int *LROW, int *INFORM, REAL *DIAG)
{
  MYBOOL SWAPPD;
  int    KBEGIN, IW, LENW, LW1, LW2, JFIRST, MINFRE, NFREE, L, J, KSTART, KSTOP, K,
         LFIRST, IV, LENV, LV1, JLAST, LV2, LV3, LV, JV, LW, LDIAG, LIMIT;
  REAL   AMULT, LTOL, USPACE, SMALL, VJ, WJ;

  LTOL   = LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij];
  SMALL  = LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE];
  USPACE = LUSOL->parmlu[LUSOL_RP_COMPSPACE_U];
  KBEGIN = KFIRST;
  SWAPPD = FALSE;

/*      We come back here from below if a row interchange is performed. */
x100:
  IW = LUSOL->ip[KLAST];
  LENW = LUSOL->lenr[IW];
  if(LENW==0)
    goto x910;
  LW1 = LUSOL->locr[IW];
  LW2 = (LW1+LENW)-1;
  JFIRST = LUSOL->iq[KBEGIN];
  if(KBEGIN>=KLAST)
    goto x700;
/*      Make sure there is room at the end of the row file
        in case row  iw  is moved there and fills in completely. */
  MINFRE = LUSOL->n+1;
  NFREE = LUSOL->lena-(*LENL)-(*LROW);
  if(NFREE<MINFRE) {
    LU1REC(LUSOL, LUSOL->m,TRUE,LROW,LUSOL->indr,LUSOL->lenr,LUSOL->locr);
    LW1 = LUSOL->locr[IW];
    LW2 = (LW1+LENW)-1;
    NFREE = LUSOL->lena-(*LENL)-(*LROW);
    if(NFREE<MINFRE)
      goto x970;

  }
/*      Set markers on row  iw. */
  for(L = LW1; L <= LW2; L++) {
    J = LUSOL->indr[L];
    LUSOL->locc[J] = L;
  }
/*      ==================================================================
        Main elimination loop.
        ================================================================== */
  KSTART = KBEGIN;
  KSTOP = MIN(KLAST,LUSOL->n);
  for(K = KSTART; K <= KSTOP; K++) {
    JFIRST = LUSOL->iq[K];
    LFIRST = LUSOL->locc[JFIRST];
    if(LFIRST==0)
      goto x490;
/*         Row  iw  has its first element in column  jfirst. */
    WJ = LUSOL->a[LFIRST];
    if(K==KLAST)
      goto x490;
/*         ---------------------------------------------------------------
           We are about to use the first element of row  iv
                  to eliminate the first element of row  iw.
           However, we may wish to interchange the rows instead,
           to preserve stability and/or sparsity.
           --------------------------------------------------------------- */
    IV = LUSOL->ip[K];
    LENV = LUSOL->lenr[IV];
    LV1 = LUSOL->locr[IV];
    VJ = ZERO;
    if(LENV==0)
      goto x150;
    if(LUSOL->indr[LV1]!=JFIRST)
      goto x150;
    VJ = LUSOL->a[LV1];
    if(SWAPPD)
      goto x200;
    if(LTOL*fabs(WJ)<fabs(VJ))
      goto x200;
    if(LTOL*fabs(VJ)<fabs(WJ))
      goto x150;
    if(LENV<=LENW)
      goto x200;
/*         ---------------------------------------------------------------
           Interchange rows  iv  and  iw.
           --------------------------------------------------------------- */
x150:
    LUSOL->ip[KLAST] = IV;
    LUSOL->ip[K] = IW;
    KBEGIN = K;
    SWAPPD = TRUE;
    goto x600;
/*         ---------------------------------------------------------------
           Delete the eliminated element from row  iw
           by overwriting it with the last element.
           --------------------------------------------------------------- */
x200:
    LUSOL->a[LFIRST] = LUSOL->a[LW2];
    JLAST = LUSOL->indr[LW2];
    LUSOL->indr[LFIRST] = JLAST;
    LUSOL->indr[LW2] = 0;
    LUSOL->locc[JLAST] = LFIRST;
    LUSOL->locc[JFIRST] = 0;
    LENW--;
    (*LENU)--;
    if(*LROW==LW2)
      (*LROW)--;
    LW2 = LW2-1;
/*         ---------------------------------------------------------------
           Form the multiplier and store it in the  L  file.
           --------------------------------------------------------------- */
    if(fabs(WJ)<=SMALL)
      goto x490;
    AMULT = -WJ/VJ;
    L = LUSOL->lena-(*LENL);
    LUSOL->a[L] = AMULT;
    LUSOL->indr[L] = IV;
    LUSOL->indc[L] = IW;
    (*LENL)++;
/*         ---------------------------------------------------------------
           Add the appropriate multiple of row  iv  to row  iw.
           We use two different inner loops.  The first one is for the
           case where row  iw  is not at the end of storage.
           --------------------------------------------------------------- */
    if(LENV==1)
      goto x490;
    LV2 = LV1+1;
    LV3 = (LV1+LENV)-1;
    if(LW2==*LROW)
      goto x400;
/*         ...............................................................
           This inner loop will be interrupted only if
           fill-in occurs enough to bump into the next row.
           ............................................................... */
    for(LV = LV2; LV <= LV3; LV++) {
      JV = LUSOL->indr[LV];
      LW = LUSOL->locc[JV];
      if(LW>0) {
/*               No fill-in. */
        LUSOL->a[LW] += AMULT*LUSOL->a[LV];
        if(fabs(LUSOL->a[LW])<=SMALL) {
/*                  Delete small computed element. */
          LUSOL->a[LW] = LUSOL->a[LW2];
          J = LUSOL->indr[LW2];
          LUSOL->indr[LW] = J;
          LUSOL->indr[LW2] = 0;
          LUSOL->locc[J] = LW;
          LUSOL->locc[JV] = 0;
          (*LENU)--;
          LENW--;
          LW2--;
        }
      }
      else {
/*               Row  iw  doesn't have an element in column  jv  yet
                 so there is a fill-in. */
        if(LUSOL->indr[LW2+1]!=0)
          goto x360;
        (*LENU)++;
        LENW++;
        LW2++;
        LUSOL->a[LW2] = AMULT*LUSOL->a[LV];
        LUSOL->indr[LW2] = JV;
        LUSOL->locc[JV] = LW2;
      }
    }
    goto x490;
/*         Fill-in interrupted the previous loop.
           Move row  iw  to the end of the row file. */
x360:
    LV2 = LV;
    LUSOL->locr[IW] = (*LROW)+1;

#ifdef LUSOLFastMove
    L = LW2-LW1+1;
    if(L > 0) {
      int loci, *locp;
      for(loci = LW1, locp = LUSOL->indr+LW1;
          loci <= LW2; loci++, locp++) {
        (*LROW)++;
        LUSOL->locc[*locp] = *LROW;
      }
      LW2 = (*LROW)-L+1;
      MEMMOVE(LUSOL->a+LW2,    LUSOL->a+LW1, L);
      MEMMOVE(LUSOL->indr+LW2, LUSOL->indr+LW1, L);
      MEMCLEAR(LUSOL->indr+LW1, L);
    }
#else
    for(L = LW1; L <= LW2; L++) {
      (*LROW)++;
      LUSOL->a[*LROW] = LUSOL->a[L];
      J = LUSOL->indr[L];
      LUSOL->indr[L] = 0;
      LUSOL->indr[*LROW] = J;
      LUSOL->locc[J] = *LROW;
    }
#endif
    LW1 = LUSOL->locr[IW];
    LW2 = *LROW;
/*         ...............................................................
           Inner loop with row  iw  at the end of storage.
           ............................................................... */
x400:
    for(LV = LV2; LV <= LV3; LV++) {
      JV = LUSOL->indr[LV];
      LW = LUSOL->locc[JV];
      if(LW>0) {
/*               No fill-in. */
        LUSOL->a[LW] += AMULT*LUSOL->a[LV];
        if(fabs(LUSOL->a[LW])<=SMALL) {
/*                  Delete small computed element. */
          LUSOL->a[LW] = LUSOL->a[LW2];
          J = LUSOL->indr[LW2];
          LUSOL->indr[LW] = J;
          LUSOL->indr[LW2] = 0;
          LUSOL->locc[J] = LW;
          LUSOL->locc[JV] = 0;
          (*LENU)--;
          LENW--;
          LW2--;
        }
      }
      else {
/*               Row  iw  doesn't have an element in column  jv  yet
                 so there is a fill-in. */
        (*LENU)++;
        LENW++;
        LW2++;
        LUSOL->a[LW2] = AMULT*LUSOL->a[LV];
        LUSOL->indr[LW2] = JV;
        LUSOL->locc[JV] = LW2;
      }
    }
    *LROW = LW2;
/*         The  k-th  element of row  iw  has been processed.
           Reset  swappd  before looking at the next element. */
x490:
    SWAPPD = FALSE;
  }
/*      ==================================================================
        End of main elimination loop.
        ==================================================================

        Cancel markers on row  iw. */
x600:
  LUSOL->lenr[IW] = LENW;
  if(LENW==0)
    goto x910;
  for(L = LW1; L <= LW2; L++) {
    J = LUSOL->indr[L];
    LUSOL->locc[J] = 0;
  }
/*      Move the diagonal element to the front of row  iw.
        At this stage,  lenw gt 0  and  klast le n. */
x700:
  for(L = LW1; L <= LW2; L++) {
    LDIAG = L;
    if(LUSOL->indr[L]==JFIRST)
      goto x730;
  }
  goto x910;

x730:
  *DIAG = LUSOL->a[LDIAG];
  LUSOL->a[LDIAG] = LUSOL->a[LW1];
  LUSOL->a[LW1] = *DIAG;
  LUSOL->indr[LDIAG] = LUSOL->indr[LW1];
  LUSOL->indr[LW1] = JFIRST;
/*      If an interchange is needed, repeat from the beginning with the
        new row  iw,  knowing that the opposite interchange cannot occur. */
  if(SWAPPD)
    goto x100;
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  goto x950;
/*      Singular. */
x910:
  *DIAG = ZERO;
  *INFORM = LUSOL_INFORM_LUSINGULAR;
/*      Force a compression if the file for  U  is much longer than the
        no. of nonzeros in  U  (i.e. if  lrow  is much bigger than  lenU).
        This should prevent memory fragmentation when there is far more
        memory than necessary  (i.e. when  lena  is huge). */
x950:
  LIMIT = (int) (USPACE*(*LENU))+LUSOL->m+LUSOL->n+1000;
  if(*LROW>LIMIT)
    LU1REC(LUSOL, LUSOL->m,TRUE,LROW,LUSOL->indr,LUSOL->lenr,LUSOL->locr);
  goto x990;
/*      Not enough storage. */
x970:
  *INFORM = LUSOL_INFORM_ANEEDMEM;
/*      Exit. */
x990:
;
}

/* ==================================================================
   lu7rnk (check rank) assumes U is currently nrank by n
   and determines if row nrank contains an acceptable pivot.
   If not, the row is deleted and nrank is decreased by 1.
   jsing is an input parameter (not altered).  If jsing is positive,
   column jsing has already been judged dependent.  A substitute
   (if any) must be some other column.
   ------------------------------------------------------------------
   -- Jul 1987: First version.
   09 May 1988: First f77 version.
   ================================================================== */
void LU7RNK(LUSOLrec *LUSOL, int JSING, int *LENU,
            int *LROW, int *NRANK, int *INFORM, REAL *DIAG)
{
  REAL UTOL1, UMAX;
  int  IW, LENW, L1, L2, LMAX, L, JMAX, KMAX;

#ifdef ForceInitialization
  L1 = 0;
  L2 = 0;
#endif

  UTOL1 = LUSOL->parmlu[LUSOL_RP_SMALLDIAG_U];
  *DIAG = ZERO;
/*      Find Umax, the largest element in row nrank. */
  IW = LUSOL->ip[*NRANK];
  LENW = LUSOL->lenr[IW];
  if(LENW==0)
    goto x400;
  L1 = LUSOL->locr[IW];
  L2 = (L1+LENW)-1;
  UMAX = ZERO;
  LMAX = L1;
  for(L = L1; L <= L2; L++) {
    if(UMAX<fabs(LUSOL->a[L])) {
      UMAX = fabs(LUSOL->a[L]);
      LMAX = L;
    }
  }
/*      Find which column that guy is in (in pivotal order).
        Interchange him with column nrank, then move him to be
        the new diagonal at the front of row nrank. */
  *DIAG = LUSOL->a[LMAX];
  JMAX = LUSOL->indr[LMAX];
  for(KMAX = *NRANK; KMAX <= LUSOL->n; KMAX++) {
    if(LUSOL->iq[KMAX]==JMAX)
      break;
  }
  LUSOL->iq[KMAX] = LUSOL->iq[*NRANK];
  LUSOL->iq[*NRANK] = JMAX;
  LUSOL->a[LMAX] = LUSOL->a[L1];
  LUSOL->a[L1] = *DIAG;
  LUSOL->indr[LMAX] = LUSOL->indr[L1];
  LUSOL->indr[L1] = JMAX;
/*      See if the new diagonal is big enough. */
  if(UMAX<=UTOL1)
    goto x400;
  if(JMAX==JSING)
    goto x400;
/*      ------------------------------------------------------------------
        The rank stays the same.
        ------------------------------------------------------------------ */
  *INFORM = LUSOL_INFORM_LUSUCCESS;
  return;
/*      ------------------------------------------------------------------
        The rank decreases by one.
        ------------------------------------------------------------------ */
x400:
  *INFORM = LUSOL_INFORM_RANKLOSS;
  (*NRANK)--;
  if(LENW>0) {
/*         Delete row nrank from U. */
    LENU = LENU-LENW;
    LUSOL->lenr[IW] = 0;
    for(L = L1; L <= L2; L++) {
      LUSOL->indr[L] = 0;
    }
    if(L2==*LROW) {
/*            This row was at the end of the data structure.
              We have to reset lrow.
              Preceding rows might already have been deleted, so we
              have to be prepared to go all the way back to 1. */
      for(L = 1; L <= L2; L++) {
        if(LUSOL->indr[*LROW]>0)
          goto x900;
        (*LROW)--;
      }
    }
  }
x900:
;
}

/* ==================================================================
   lu7zap  eliminates all nonzeros in column  jzap  of  U.
   It also sets  kzap  to the position of  jzap  in pivotal order.
   Thus, on exit we have  iq(kzap) = jzap.
   ------------------------------------------------------------------
   -- Jul 1987: nrank added.
   10 May 1988: First f77 version.
   ================================================================== */
void LU7ZAP(LUSOLrec *LUSOL, int JZAP, int *KZAP, int *LENU, int *LROW,
            int NRANK)
{
  int K, I, LENI, LR1, LR2, L;

  for(K = 1; K <= NRANK; K++) {
    I = LUSOL->ip[K];
    LENI = LUSOL->lenr[I];
    if(LENI==0)
      goto x90;
    LR1 = LUSOL->locr[I];
    LR2 = (LR1+LENI)-1;
    for(L = LR1; L <= LR2; L++) {
      if(LUSOL->indr[L]==JZAP)
        goto x60;
    }
    goto x90;
/*         Delete the old element. */
x60:
    LUSOL->a[L] = LUSOL->a[LR2];
    LUSOL->indr[L] = LUSOL->indr[LR2];
    LUSOL->indr[LR2] = 0;
    LUSOL->lenr[I] = LENI-1;
    (*LENU)--;
/*         Stop if we know there are no more rows containing  jzap. */
x90:
    *KZAP = K;
    if(LUSOL->iq[K]==JZAP)
      goto x800;
  }
/*      nrank must be smaller than n because we haven't found kzap yet. */
  L = LUSOL->n;
  for(K = NRANK+1; K <= L; K++) {
    *KZAP = K;
    if(LUSOL->iq[K]==JZAP)
      break;
  }
/*      See if we zapped the last element in the file. */
x800:
  if(*LROW>0) {
    if(LUSOL->indr[*LROW]==0)
      (*LROW)--;
  }

}

