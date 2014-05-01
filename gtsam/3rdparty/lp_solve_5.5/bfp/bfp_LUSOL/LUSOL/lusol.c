
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   LUSOL routines from the Stanford Optimization Laboratory
   The parts included are:
    lusol1      Factor a given matrix A from scratch (lu1fac).
    lusol2      Heap-management routines for lu1fac.
    lusol6      Solve with the current LU factors.
    lusol7      Utilities for all update routines.
    lusol8      Replace a column (Bartels-Golub update).
   ------------------------------------------------------------------
   26 Apr 2002: TCP implemented using heap data structure.
   01 May 2002: lu1DCP implemented.
   07 May 2002: lu1mxc must put 0.0 at top of empty columns.
   09 May 2002: lu1mCP implements Markowitz with cols searched
                in heap order.
                Often faster (searching 20 or 40 cols) but more dense.
   11 Jun 2002: TRP implemented.
                lu1mRP implements Markowitz with Threshold Rook
                Pivoting.
                lu1mxc maintains max col elements  (was lu1max.)
                lu1mxr maintains max row elements.
   12 Jun 2002: lu1mCP seems too slow on big problems (e.g. memplus).
                Disabled it for the moment.  (Use lu1mar + TCP.)
   14 Dec 2002: TSP implemented.
                lu1mSP implements Markowitz with TSP.
   07 Mar 2003: character*1, character*2 changed to f90 form.
                Comments changed from * in column to ! in column 1.
                Comments kept within column 72 to avoid compiler
                warning.
   06 Mar 2004: Translation to C by Kjell Eikland with the addition
                of data wrappers, parametric constants, various
                helper routines, and dynamic memory reallocation.
   26 May 2004: Added LUSOL_IP_UPDATELIMIT parameter and provided
                for dynamic memory expansion based on possible
                forward requirements.
   08 Jul 2004: Revised logic in lu6chk based on new code from
                Michael Saunders.
   01 Dec 2005: Add support for CMEX interface (disable by undef MATLAB)
                Also include various bug fixes (disable by undef YZHANG)
                Yin Zhang <yzhang@cs.utexas.edu>
   01 Jan 2006: Added storage of singular indeces, not only the last.
   ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <float.h>
#include <math.h>
#include "lusol.h"
#include "myblas.h"
#ifdef MATLAB
  #include "mex.h"
#endif

/* LUSOL Object creation and destruction */

void *clean_realloc(void *oldptr, int width, int newsize, int oldsize)
{
  newsize *= width;
  oldsize *= width;
  oldptr = LUSOL_REALLOC(oldptr, newsize);
  if(newsize > oldsize)
/*    MEMCLEAR(oldptr+oldsize, newsize-oldsize); */
    memset((char *)oldptr+oldsize, '\0', newsize-oldsize);
  return(oldptr);
}

MYBOOL LUSOL_realloc_a(LUSOLrec *LUSOL, int newsize)
{
  int oldsize;

  if(newsize < 0)
    newsize = LUSOL->lena + MAX(abs(newsize), LUSOL_MINDELTA_a);

  oldsize = LUSOL->lena;
  LUSOL->lena = newsize;
  if(newsize > 0)
    newsize++;
  if(oldsize > 0)
    oldsize++;

  LUSOL->a    = (REAL *) clean_realloc(LUSOL->a,    sizeof(*(LUSOL->a)),
                                                    newsize, oldsize);
  LUSOL->indc = (int *)  clean_realloc(LUSOL->indc, sizeof(*(LUSOL->indc)),
                                                    newsize, oldsize);
  LUSOL->indr = (int *)  clean_realloc(LUSOL->indr, sizeof(*(LUSOL->indr)),
                                                    newsize, oldsize);
  if((newsize == 0) ||
     ((LUSOL->a != NULL) && (LUSOL->indc != NULL) && (LUSOL->indr != NULL)))
    return( TRUE );
  else
    return( FALSE );
}

MYBOOL LUSOL_expand_a(LUSOLrec *LUSOL, int *delta_lena, int *right_shift)
{
#ifdef StaticMemAlloc
  return( FALSE );
#else
  int LENA, NFREE, LFREE;

  /* Add expansion factor to avoid having to resize too often/too much;
     (exponential formula suggested by Michael A. Saunders) */
  LENA = LUSOL->lena;
  *delta_lena = DELTA_SIZE(*delta_lena, LENA);

  /* Expand it! */
  if((*delta_lena <= 0) || !LUSOL_realloc_a(LUSOL, LENA+(*delta_lena)))
    return( FALSE );

  /* Make sure we return the actual memory increase of a */
  *delta_lena = LUSOL->lena-LENA;

  /* Shift the used memory area to the right */
  LFREE = *right_shift;
  NFREE = LFREE+*delta_lena;
  LENA  -= LFREE-1;
  MEMMOVE(LUSOL->a+NFREE,    LUSOL->a+LFREE,    LENA);
  MEMMOVE(LUSOL->indr+NFREE, LUSOL->indr+LFREE, LENA);
  MEMMOVE(LUSOL->indc+NFREE, LUSOL->indc+LFREE, LENA);

  /* Also return the new starting position for the used memory area of a */
  *right_shift  = NFREE;

  LUSOL->expanded_a++;
  return( TRUE );
#endif
}

MYBOOL LUSOL_realloc_r(LUSOLrec *LUSOL, int newsize)
{
  int oldsize;

  if(newsize < 0)
    newsize = LUSOL->maxm + MAX(abs(newsize), LUSOL_MINDELTA_rc);

  oldsize = LUSOL->maxm;
  LUSOL->maxm = newsize;
  if(newsize > 0)
    newsize++;
  if(oldsize > 0)
    oldsize++;

  LUSOL->lenr  = (int *) clean_realloc(LUSOL->lenr,  sizeof(*(LUSOL->lenr)),
                                                     newsize, oldsize);
  LUSOL->ip    = (int *) clean_realloc(LUSOL->ip,    sizeof(*(LUSOL->ip)),
                                                     newsize, oldsize);
  LUSOL->iqloc = (int *) clean_realloc(LUSOL->iqloc, sizeof(*(LUSOL->iqloc)),
                                                     newsize, oldsize);
  LUSOL->ipinv = (int *) clean_realloc(LUSOL->ipinv, sizeof(*(LUSOL->ipinv)),
                                                     newsize, oldsize);
  LUSOL->locr  = (int *) clean_realloc(LUSOL->locr,  sizeof(*(LUSOL->locr)),
                                                     newsize, oldsize);

  if((newsize == 0) ||
     ((LUSOL->lenr != NULL) &&
      (LUSOL->ip != NULL) && (LUSOL->iqloc != NULL) &&
      (LUSOL->ipinv != NULL) && (LUSOL->locr != NULL))) {

#ifndef ClassicHamaxR
#ifdef AlwaysSeparateHamaxR
    if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TRP)
#endif
    {
      LUSOL->amaxr = (REAL *) clean_realloc(LUSOL->amaxr, sizeof(*(LUSOL->amaxr)),
                                                          newsize, oldsize);
      if((newsize > 0) && (LUSOL->amaxr == NULL))
        return( FALSE );
    }
#endif
    return( TRUE );
  }
  else
    return( FALSE );
}

MYBOOL LUSOL_realloc_c(LUSOLrec *LUSOL, int newsize)
{
  int oldsize;

  if(newsize < 0)
    newsize = LUSOL->maxn + MAX(abs(newsize), LUSOL_MINDELTA_rc);

  oldsize = LUSOL->maxn;
  LUSOL->maxn = newsize;
  if(newsize > 0)
    newsize++;
  if(oldsize > 0)
    oldsize++;

  LUSOL->lenc  = (int *)  clean_realloc(LUSOL->lenc,  sizeof(*(LUSOL->lenc)),
                                                      newsize, oldsize);
  LUSOL->iq    = (int *)  clean_realloc(LUSOL->iq,    sizeof(*(LUSOL->iq)),
                                                      newsize, oldsize);
  LUSOL->iploc = (int *)  clean_realloc(LUSOL->iploc, sizeof(*(LUSOL->iploc)),
                                                      newsize, oldsize);
  LUSOL->iqinv = (int *)  clean_realloc(LUSOL->iqinv, sizeof(*(LUSOL->iqinv)),
                                                      newsize, oldsize);
  LUSOL->locc  = (int *)  clean_realloc(LUSOL->locc,  sizeof(*(LUSOL->locc)),
                                                      newsize, oldsize);
  LUSOL->w     = (REAL *) clean_realloc(LUSOL->w,     sizeof(*(LUSOL->w)),
                                                      newsize, oldsize);
#ifdef LUSOLSafeFastUpdate
  LUSOL->vLU6L = (REAL *) clean_realloc(LUSOL->vLU6L, sizeof(*(LUSOL->vLU6L)),
                                                      newsize, oldsize);
#else
  LUSOL->vLU6L = LUSOL->w;
#endif

  if((newsize == 0) ||
     ((LUSOL->w != NULL) && (LUSOL->lenc != NULL) &&
      (LUSOL->iq != NULL) && (LUSOL->iploc != NULL) &&
      (LUSOL->iqinv != NULL) && (LUSOL->locc != NULL))) {

#ifndef ClassicHamaxR
    if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TCP) {
      LUSOL->Ha = (REAL *) clean_realloc(LUSOL->Ha,   sizeof(*(LUSOL->Ha)),
                                                      newsize, oldsize);
      LUSOL->Hj = (int *)  clean_realloc(LUSOL->Hj,   sizeof(*(LUSOL->Hj)),
                                                      newsize, oldsize);
      LUSOL->Hk = (int *)  clean_realloc(LUSOL->Hk,   sizeof(*(LUSOL->Hk)),
                                                      newsize, oldsize);
      if((newsize > 0) &&
         ((LUSOL->Ha == NULL) || (LUSOL->Hj == NULL) || (LUSOL->Hk == NULL)))
        return( FALSE );
    }
#endif
#ifndef ClassicdiagU
    if(LUSOL->luparm[LUSOL_IP_KEEPLU] == FALSE) {
      LUSOL->diagU = (REAL *) clean_realloc(LUSOL->diagU, sizeof(*(LUSOL->diagU)),
                                                          newsize, oldsize);
      if((newsize > 0) && (LUSOL->diagU == NULL))
        return( FALSE );
    }
#endif

    return( TRUE );
  }
  else
    return( FALSE );
}

LUSOLrec *LUSOL_create(FILE *outstream, int msgfil, int pivotmodel, int updatelimit)
{
  LUSOLrec *newLU;

  newLU = (LUSOLrec *) LUSOL_CALLOC(1, sizeof(*newLU));
  if(newLU == NULL)
    return( newLU );

  newLU->luparm[LUSOL_IP_SCALAR_NZA]       = LUSOL_MULT_nz_a;
  newLU->outstream = outstream;
  newLU->luparm[LUSOL_IP_PRINTUNIT]        = msgfil;
  newLU->luparm[LUSOL_IP_PRINTLEVEL]       = LUSOL_MSG_SINGULARITY;

  LUSOL_setpivotmodel(newLU, pivotmodel, LUSOL_PIVTOL_DEFAULT);

  newLU->parmlu[LUSOL_RP_GAMMA]            = LUSOL_DEFAULT_GAMMA;

  newLU->parmlu[LUSOL_RP_ZEROTOLERANCE]    = 3.0e-13;

  newLU->parmlu[LUSOL_RP_SMALLDIAG_U]      = /*3.7e-11;*/
  newLU->parmlu[LUSOL_RP_EPSDIAG_U]        = 3.7e-11;

  newLU->parmlu[LUSOL_RP_COMPSPACE_U]      = 3.0e+0;

  newLU->luparm[LUSOL_IP_MARKOWITZ_MAXCOL] = 5;
  newLU->parmlu[LUSOL_RP_MARKOWITZ_CONLY]  = 0.3e+0;
  newLU->parmlu[LUSOL_RP_MARKOWITZ_DENSE]  = 0.5e+0;

  newLU->parmlu[LUSOL_RP_SMARTRATIO]       = LUSOL_DEFAULT_SMARTRATIO;
#ifdef ForceRowBasedL0
  newLU->luparm[LUSOL_IP_ACCELERATION]     = LUSOL_BASEORDER;
#endif
  newLU->luparm[LUSOL_IP_KEEPLU]           = TRUE;
  newLU->luparm[LUSOL_IP_UPDATELIMIT]      = updatelimit;

  init_BLAS();

  return( newLU );
}

MYBOOL LUSOL_sizeto(LUSOLrec *LUSOL, int init_r, int init_c, int init_a)
{
  if(init_c == 0)
    LUSOL_FREE(LUSOL->isingular);
  if(LUSOL_realloc_a(LUSOL, init_a) &&
     LUSOL_realloc_r(LUSOL, init_r) &&
     LUSOL_realloc_c(LUSOL, init_c))
    return( TRUE );
  else
    return( FALSE );
}

char *LUSOL_pivotLabel(LUSOLrec *LUSOL)
{
  static /*const*/ char *pivotText[LUSOL_PIVMOD_MAX+1] =
  {"TPP", "TRP", "TCP", "TSP"};
  return(pivotText[LUSOL->luparm[LUSOL_IP_PIVOTTYPE]]);
}

void LUSOL_setpivotmodel(LUSOLrec *LUSOL, int pivotmodel, int initlevel)
{
  REAL newFM, newUM;

  /* Set pivotmodel if specified */
  if(pivotmodel > LUSOL_PIVMOD_NOCHANGE) {
    if((pivotmodel <= LUSOL_PIVMOD_DEFAULT) || (pivotmodel > LUSOL_PIVMOD_MAX))
      pivotmodel = LUSOL_PIVMOD_TPP;
    LUSOL->luparm[LUSOL_IP_PIVOTTYPE]        = pivotmodel;
  }

  /* Check if we need bother about changing tolerances */
  if((initlevel <= LUSOL_PIVTOL_NOCHANGE) || (initlevel > LUSOL_PIVTOL_MAX))
    return;

  /* Set default pivot tolerances
     (note that UPDATEMAX should always be <= FACTORMAX) */
  if(initlevel == LUSOL_PIVTOL_BAGGY) {        /* Extra-loose pivot thresholds */
    newFM = 500.0;
    newUM = newFM / 20;
  }
  else if(initlevel == LUSOL_PIVTOL_LOOSE) {  /* Moderately tight pivot tolerances */
    newFM = 100.0;
    newUM = newFM / 10;
  }
  else if(initlevel == LUSOL_PIVTOL_NORMAL) { /* Standard pivot tolerances */
    newFM = 28.0;
    newUM = newFM / 4;
  }
  else if(initlevel == LUSOL_PIVTOL_SLIM) {   /* Better accuracy pivot tolerances */
    newFM = 10.0;
    newUM = newFM / 2;
  }
  else if(initlevel == LUSOL_PIVTOL_TIGHT) {  /* Enhanced accuracy pivot tolerances */
    newFM = 5.0;
    newUM = newFM / 2;
  }
  else if(initlevel == LUSOL_PIVTOL_SUPER) {  /* Very tight pivot tolerances for extra accuracy */
    newFM = 2.5;
    newUM = 1.99;
  }
  else {                                      /* Extremely tight pivot tolerances for extra accuracy */
    newFM = 1.99;
    newUM = newFM / 1.49;
  }

  /* Set the tolerances */
  LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = newFM;
  LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij] = newUM;
}

MYBOOL LUSOL_tightenpivot(LUSOLrec *LUSOL)
{
  REAL newvalue;

  /* Give up tightening if we are already less than limit and we cannot change strategy */
  if(MIN(LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij],
         LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij]) < 1.1) {
    if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] >= LUSOL_PIVMOD_TRP)
      return( FALSE );
    LUSOL_setpivotmodel(LUSOL, LUSOL->luparm[LUSOL_IP_PIVOTTYPE]+1, LUSOL_PIVTOL_DEFAULT+1);
    return( 2 );
  }

  /* Otherwise tighten according to defined schedule */
#if 0   /* This is Michael Saunder's proposed tightening procedure */
  newvalue = sqrt(LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij]);
  LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = newvalue;
  SETMIN(LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij], newvalue);
#elif 0 /* This is Kjell Eikland's schedule #1 */
  newvalue = sqrt(LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij]);
  LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = newvalue;
  LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij] = 1.0 + (newvalue - 1.0) / 2;
#else   /* This was Kjell Eikland's schedule #2 */
  LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = 1.0 + LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij]/3.0;
  LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij] = 1.0 + LUSOL->parmlu[LUSOL_RP_UPDATEMAX_Lij]/3.0;
#endif
  return( TRUE );
}

MYBOOL LUSOL_addSingularity(LUSOLrec *LUSOL, int singcol, int *inform)
{
  int NSING = LUSOL->luparm[LUSOL_IP_SINGULARITIES],
      ASING = LUSOL->luparm[LUSOL_IP_SINGULARLISTSIZE];

  /* Check if we need to allocated list memory to store multiple singularities */
  if((NSING > 0) && (NSING >= ASING)) {

    /* Increase list in "reasonable" steps */
    ASING += (int) (10.0 * (log10((REAL) LUSOL->m)+1.0));
    LUSOL->isingular = (int *) LUSOL_REALLOC(LUSOL->isingular, sizeof(*LUSOL->isingular)*(ASING+1));
    if(LUSOL->isingular == NULL) {
      LUSOL->luparm[LUSOL_IP_SINGULARLISTSIZE] = 0;
      *inform = LUSOL_INFORM_NOMEMLEFT;
      return( FALSE );
    }
    LUSOL->luparm[LUSOL_IP_SINGULARLISTSIZE] = ASING;

    /* Transfer the first singularity if the list was just created */
    if(NSING == 1)
      LUSOL->isingular[NSING] = LUSOL->luparm[LUSOL_IP_SINGULARINDEX];
  }

  /* Update singularity count and store its index */
  NSING++;
  if(NSING > 1) {
    LUSOL->isingular[0] = NSING;
    LUSOL->isingular[NSING] = singcol;
  }
  LUSOL->luparm[LUSOL_IP_SINGULARITIES] = NSING;

  /* Mimic old logic by keeping the last singularity stored */
  LUSOL->luparm[LUSOL_IP_SINGULARINDEX] = singcol;

  return( TRUE );
}

int LUSOL_getSingularity(LUSOLrec *LUSOL, int singitem)
{
  if((singitem > LUSOL->luparm[LUSOL_IP_SINGULARITIES]) || (singitem < 0))
    singitem = -1;
  else if(singitem == 0)
    singitem = LUSOL->luparm[LUSOL_IP_SINGULARITIES];
  else if(singitem > 1)
    singitem = LUSOL->isingular[singitem];
  else
    singitem = LUSOL->luparm[LUSOL_IP_SINGULARINDEX];
  return( singitem );
}

int LUSOL_findSingularityPosition(LUSOLrec *LUSOL, int singcol)
/* The purpose of this routine is to find the slack row/column in
   user-index that was singular in the last unsuccessful column
   update; zero is returned if the search was unsuccessful.
   By adding a slack at this position this particular singularity
   should disappear.
   (Source: Michael A. Saunders; private communication to KE) */
{
#if 0 /* Michael Saunders version */
  int j;
  for(j = LUSOL->m; j > 0; j--)
    if(LUSOL->iq[j] == singcol)
      break;
  singcol = j;
#else /* Kjell Eikland version (note that iqinv could be invalid in early versions of LUSOL) */
  singcol = LUSOL->iqinv[singcol];
#endif
  return( LUSOL->ip[singcol] );
}

char *LUSOL_informstr(LUSOLrec *LUSOL, int inform)
{
  static char *informText[LUSOL_INFORM_MAX-LUSOL_INFORM_MIN+1] =
  {"LUSOL_RANKLOSS: Lost rank",
   "LUSOL_LUSUCCESS: Success",
   "LUSOL_LUSINGULAR: Singular A",
   "LUSOL_LUUNSTABLE: Unstable factorization",
   "LUSOL_ADIMERR: Row or column count exceeded",
   "LUSOL_ADUPLICATE: Duplicate A matrix entry found",
   "(Undefined message)",
   "(Undefined message)",
   "LUSOL_ANEEDMEM: Insufficient memory for factorization",
   "LUSOL_FATALERR: Fatal internal error",
   "LUSOL_NOPIVOT: Found no suitable pivot",
   "LUSOL_NOMEMLEFT: Could not obtain more memory"};
  if((inform < LUSOL_INFORM_MIN) || (inform > LUSOL_INFORM_MAX))
    inform = LUSOL->luparm[LUSOL_IP_INFORM];
  return(informText[inform-LUSOL_INFORM_MIN]);
}

void LUSOL_clear(LUSOLrec *LUSOL, MYBOOL nzonly)
{
  int len;

  LUSOL->nelem = 0;
  if(!nzonly) {

   /* lena arrays */
    len = LUSOL->lena + LUSOL_ARRAYOFFSET;
    MEMCLEAR(LUSOL->a,    len);
    MEMCLEAR(LUSOL->indc, len);
    MEMCLEAR(LUSOL->indr, len);

   /* maxm arrays */
    len = LUSOL->maxm + LUSOL_ARRAYOFFSET;
    MEMCLEAR(LUSOL->lenr,  len);
    MEMCLEAR(LUSOL->ip,    len);
    MEMCLEAR(LUSOL->iqloc, len);
    MEMCLEAR(LUSOL->ipinv, len);
    MEMCLEAR(LUSOL->locr,  len);

#ifndef ClassicHamaxR
    if((LUSOL->amaxr != NULL)
#ifdef AlwaysSeparateHamaxR
       && (LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TRP)
#endif
      )
      MEMCLEAR(LUSOL->amaxr, len);
#endif

   /* maxn arrays */
    len = LUSOL->maxn + LUSOL_ARRAYOFFSET;
    MEMCLEAR(LUSOL->lenc,  len);
    MEMCLEAR(LUSOL->iq,    len);
    MEMCLEAR(LUSOL->iploc, len);
    MEMCLEAR(LUSOL->iqinv, len);
    MEMCLEAR(LUSOL->locc,  len);
    MEMCLEAR(LUSOL->w,     len);

    if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TCP) {
      MEMCLEAR(LUSOL->Ha,  len);
      MEMCLEAR(LUSOL->Hj,  len);
      MEMCLEAR(LUSOL->Hk,  len);
    }
#ifndef ClassicdiagU
    if(LUSOL->luparm[LUSOL_IP_KEEPLU] == FALSE) {
      MEMCLEAR(LUSOL->diagU, len);
    }
#endif

  }
}

MYBOOL LUSOL_assign(LUSOLrec *LUSOL, int iA[], int jA[], REAL Aij[], int nzcount, MYBOOL istriplet)
{
  int k, m, n, ij, kol;

  /* Adjust the size of the a structure */
  if(nzcount > (LUSOL->lena/LUSOL->luparm[LUSOL_IP_SCALAR_NZA]) &&
     !LUSOL_realloc_a(LUSOL, nzcount*LUSOL->luparm[LUSOL_IP_SCALAR_NZA]))
    return( FALSE );

  m = 0;
  n = 0;
  kol = 1;
  for(k = 1; k <= nzcount; k++) {
    /* First the row indicator */
    ij = iA[k];
    if(ij > m) {
      m = ij;
      if(m > LUSOL->maxm &&
         !LUSOL_realloc_r(LUSOL, -(m / LUSOL_MINDELTA_FACTOR + 1)))
        return( FALSE );
    }
    LUSOL->indc[k] = ij;

    /* Then the column indicator;
       Handle both triplet and column count formats */
    if(istriplet)
      ij = jA[k];
    else {
      if(k >= jA[kol])
        kol++;
      ij = kol;
    }
    if(ij > n) {
      n = ij;
      if(n > LUSOL->maxn &&
         !LUSOL_realloc_c(LUSOL, -(n / LUSOL_MINDELTA_FACTOR + 1)))
        return( FALSE );
    }
    LUSOL->indr[k] = ij;

    /* Lastly the matrix value itself */
    LUSOL->a[k] = Aij[k];
  }
  LUSOL->m = m;
  LUSOL->n = n;
  LUSOL->nelem = nzcount;
  return( TRUE );
}

int LUSOL_loadColumn(LUSOLrec *LUSOL, int iA[], int jA, REAL Aij[], int nzcount, int offset1)
{
  int i, ii, nz, k;

  nz = LUSOL->nelem;
  i = nz + nzcount;
  if(i > (LUSOL->lena/LUSOL->luparm[LUSOL_IP_SCALAR_NZA]) &&
     !LUSOL_realloc_a(LUSOL, i*LUSOL->luparm[LUSOL_IP_SCALAR_NZA]))
  return( -1 );

  k = 0;
  for(ii = 1; ii <= nzcount; ii++) {
    i = ii + offset1;
    if(Aij[i] == 0)
      continue;
    if(iA[i] <= 0 || iA[i] > LUSOL->m ||
       jA <= 0 || jA > LUSOL->n) {
      LUSOL_report(LUSOL, 0, "Variable index outside of set bounds (r:%d/%d, c:%d/%d)\n",
                             iA[i], LUSOL->m, jA, LUSOL->n);
      continue;
    }
    k++;
    nz++;
    LUSOL->a[nz]    = Aij[i];
    LUSOL->indc[nz] = iA[i];
    LUSOL->indr[nz] = jA;
  }
  LUSOL->nelem = nz;
  return( k );
}

void LUSOL_free(LUSOLrec *LUSOL)
{
  LUSOL_realloc_a(LUSOL, 0);
  LUSOL_realloc_r(LUSOL, 0);
  LUSOL_realloc_c(LUSOL, 0);
  if(LUSOL->L0 != NULL)
    LUSOL_matfree(&(LUSOL->L0));
  if(LUSOL->U != NULL)
    LUSOL_matfree(&(LUSOL->U));
  if(!is_nativeBLAS())
    unload_BLAS();
  LUSOL_FREE(LUSOL);
}

void LUSOL_report(LUSOLrec *LUSOL, int msglevel, char *format, ...)
{
  va_list ap;

  va_start(ap, format);
  if(LUSOL == NULL) {
    vfprintf(stderr, format, ap);
  }
  else if(msglevel >= 0  /*LUSOL->luparm[2]*/) {
    if(LUSOL->writelog != NULL) {
      char buff[255];

      vsprintf(buff, format, ap);
      LUSOL->writelog(LUSOL, LUSOL->loghandle, buff);
    }
    if(LUSOL->outstream != NULL) {
      vfprintf(LUSOL->outstream, format, ap);
      fflush(LUSOL->outstream);
    }
  }
  va_end(ap);
}

void LUSOL_timer(LUSOLrec *LUSOL, int timerid, char *text)
{
  LUSOL_report(LUSOL, -1, "TimerID %d at %s - %s\n",
                          timerid, "", text);
}

int LUSOL_factorize(LUSOLrec *LUSOL)
{
  int inform;

  LU1FAC( LUSOL, &inform );
  return( inform );
}

int LUSOL_ftran(LUSOLrec *LUSOL, REAL b[], int NZidx[], MYBOOL prepareupdate)
{
  int  inform;
  REAL *vector;

  if(prepareupdate)
    vector = LUSOL->vLU6L;
  else
    vector = LUSOL->w;

  /* Copy RHS vector, but make adjustment for offset since this
     can create a memory error when the calling program uses
     a 0-base vector offset back to comply with LUSOL. */
  MEMCOPY(vector+1, b+1, LUSOL->n);
  vector[0] = 0;

  LU6SOL(LUSOL, LUSOL_SOLVE_Aw_v, vector, b, NZidx, &inform);
  LUSOL->luparm[LUSOL_IP_FTRANCOUNT]++;

  return(inform);
}


int LUSOL_btran(LUSOLrec *LUSOL, REAL b[], int NZidx[])
{
  int inform;

  /* Copy RHS vector, but make adjustment for offset since this
     can create a memory error when the calling program uses
     a 0-base vector offset back to comply with LUSOL. */
  MEMCOPY(LUSOL->w+1, b+1, LUSOL->m);
  LUSOL->w[0] = 0;

  LU6SOL(LUSOL, LUSOL_SOLVE_Atv_w, b, LUSOL->w, NZidx, &inform);
  LUSOL->luparm[LUSOL_IP_BTRANCOUNT]++;

  return(inform);
}


int LUSOL_replaceColumn(LUSOLrec *LUSOL, int jcol, REAL v[])
{
  int  inform;
  REAL DIAG, VNORM;

  LU8RPC(LUSOL, LUSOL_UPDATE_OLDNONEMPTY, LUSOL_UPDATE_NEWNONEMPTY,
                jcol, v, NULL,
                &inform, &DIAG, &VNORM);

  LUSOL->replaced_c++;
  return( inform );
}

REAL LUSOL_vecdensity(LUSOLrec *LUSOL, REAL V[])
{
  int I, N = 0;

  for(I = 1; I <= LUSOL->m; I++)
    if(fabs(V[I]) > 0)
      N++;
  return( (REAL) N / (REAL) LUSOL->m );
}

char relationChar(REAL left, REAL right)
{
  if(left > right)
    return('>');
  else if(left == right)
    return('=');
  else
    return('<');
}

/* Retrieve the core modules ordered by order of dependency */

#include "lusol2.c"      /* Heap management */
#include "lusol6a.c"     /* Singularity checking and equation solving */
#include "lusol1.c"      /* Factorization and core components */
#include "lusol7a.c"     /* Utility routines for updates */
#include "lusol8a.c"     /* Column update */


void LUSOL_dump(FILE *output, LUSOLrec *LUSOL)
{
  MYBOOL userfile = (MYBOOL) (output != NULL);

  if(!userfile)
    output = fopen("LUSOL.dbg", "w");

  blockWriteREAL(output, "a", LUSOL->a, 1, LUSOL->lena);
  blockWriteINT(output, "indc", LUSOL->indc, 1, LUSOL->lena);
  blockWriteINT(output, "indr", LUSOL->indr, 1, LUSOL->lena);

  blockWriteINT(output, "ip", LUSOL->ip, 1, LUSOL->m);
  blockWriteINT(output, "iq", LUSOL->iq, 1, LUSOL->n);
  blockWriteINT(output, "lenc", LUSOL->lenc, 1, LUSOL->n);
  blockWriteINT(output, "lenr", LUSOL->lenr, 1, LUSOL->m);

  blockWriteINT(output, "locc", LUSOL->locc, 1, LUSOL->n);
  blockWriteINT(output, "locr", LUSOL->locr, 1, LUSOL->m);
  blockWriteINT(output, "iploc", LUSOL->iploc, 1, LUSOL->n);
  blockWriteINT(output, "iqloc", LUSOL->iqloc, 1, LUSOL->m);

  blockWriteINT(output, "ipinv", LUSOL->ipinv, 1, LUSOL->m);
  blockWriteINT(output, "iqinv", LUSOL->iqinv, 1, LUSOL->n);

  if(!userfile)
    fclose(output);
}

LUSOLmat *LUSOL_matcreate(int dim, int nz)
{
  LUSOLmat *newm;

  newm = (LUSOLmat *) LUSOL_CALLOC(1, sizeof(*newm));
  if(newm != NULL) {
    newm->a    = (REAL *) LUSOL_MALLOC((nz+1)*sizeof(REAL));
    newm->lenx = (int *)  LUSOL_MALLOC((dim+1)*sizeof(int));
    newm->indx = (int *)  LUSOL_MALLOC((dim+1)*sizeof(int));
    newm->indr = (int *)  LUSOL_MALLOC((nz+1)*sizeof(int));
    newm->indc = (int *)  LUSOL_MALLOC((nz+1)*sizeof(int));
    if((newm->a == NULL) ||
       (newm->lenx == NULL) || (newm->indx == NULL) ||
       (newm->indr == NULL) || (newm->indc == NULL))
      LUSOL_matfree(&newm);
  }
  return(newm);
}
void LUSOL_matfree(LUSOLmat **mat)
{
  if((mat == NULL) || (*mat == NULL))
    return;
  LUSOL_FREE((*mat)->a);
  LUSOL_FREE((*mat)->indc);
  LUSOL_FREE((*mat)->indr);
  LUSOL_FREE((*mat)->lenx);
  LUSOL_FREE((*mat)->indx);
  LUSOL_FREE(*mat);
}

