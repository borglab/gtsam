
/*  Modularized simplex basis factorization module - w/interface for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lusol.h, lp_lib.h, myblas.h

    Release notes:
    v2.0.0  1 March 2004        First implementation of the LUSOL v2.0 C translation.
    v2.0.1  1 April 2004        Added singularity recovery and fast/reuse update logic.
    v2.0.2  23 May 2004         Moved mustrefact() function into the BFP structure.
    v2.0.3  5 September 2004    Reworked pivot threshold tightening logic and default
                                values.
    v2.1.0  18 June 2005        Made changes to allow for "pure" factorization;
                                i.e. without the objective function included.

   ---------------------------------------------------------------------------------- */

/* Generic include libraries */
#include <stdlib.h>
#include <string.h>
#include "lp_lib.h"

/* Include libraries for this factorization system */
#include "myblas.h"
#include "commonlib.h"
#include "lp_LUSOL.h"
#include "lusol.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

/* Include routines common to factorization engine implementations */
#include "lp_BFP1.c"
#include "lp_BFP2.c"


/* MUST MODIFY */
char * BFP_CALLMODEL bfp_name(void)
{
  return( "LUSOL v2.2.1.0" );
}


/* MUST MODIFY */
MYBOOL BFP_CALLMODEL bfp_resize(lprec *lp, int newsize)
{
  INVrec *lu;

  lu = lp->invB;

  /* Increment dimensionality since we put the objective row at the top */
  newsize = newsize + bfp_rowoffset(lp);
  lu->dimalloc = newsize;

  /* Allocate index tracker arrays, LU matrices and various work vectors */
  if(!allocREAL(lp, &(lu->value), newsize+MATINDEXBASE, AUTOMATIC))
    return( FALSE );

  /* Data specific to the factorization engine */
  if(lu->LUSOL != NULL) {
    if(newsize > 0)
      LUSOL_sizeto(lu->LUSOL, newsize, newsize, 0);
    else {
      LUSOL_free(lu->LUSOL);
      lu->LUSOL = NULL;
    }
  }
  else if(newsize > 0) {
    int  asize;
    REAL bsize;

    lu->LUSOL = LUSOL_create(NULL, 0, LUSOL_PIVMOD_TPP, bfp_pivotmax(lp)*0);

#if 1
    lu->LUSOL->luparm[LUSOL_IP_ACCELERATION]  = LUSOL_AUTOORDER;
    lu->LUSOL->parmlu[LUSOL_RP_SMARTRATIO]    = 0.50;
#endif
#if 0
    lu->timed_refact = DEF_TIMEDREFACT;
#else
    lu->timed_refact = FALSE;
#endif

    /* The following adjustments seem necessary to make the really tough NETLIB
       models perform reliably and still performant (e.g. cycle.mps) */
#if 0
    lu->LUSOL->parmlu[LUSOL_RP_SMALLDIAG_U]   =
    lu->LUSOL->parmlu[LUSOL_RP_EPSDIAG_U]     = lp->epsprimal;
#endif
#if 0
    lu->LUSOL->parmlu[LUSOL_RP_ZEROTOLERANCE] = lp->epsvalue;
#endif

#if 1
    LUSOL_setpivotmodel(lu->LUSOL, LUSOL_PIVMOD_NOCHANGE, LUSOL_PIVTOL_SLIM);
#else
    LUSOL_setpivotmodel(lu->LUSOL, LUSOL_PIVMOD_NOCHANGE, LUSOL_PIVTOL_TIGHT);
#endif

#ifdef LUSOL_UseBLAS
/*    if(fileSearchPath("PATH", "myBLAS.DLL", NULL) && load_BLAS("myBLAS")) */
    if(is_nativeBLAS() && load_BLAS(libnameBLAS))
      lp->report(lp, NORMAL, "Optimized BLAS was successfully loaded for bfp_LUSOL.\n");
#endif

    /* Try to minimize memory allocation if we have a large number of unit columns */
    bsize = (REAL) lp->get_nonzeros(lp);
    if(newsize > lp->columns)
      bsize += newsize;
    else
      bsize = bsize/lp->columns*newsize;
    /* Add a "reasonable" delta to allow for B and associated factorizations
       that are denser than average; this makes reallocations less frequent.
       Values between 1.2 and 1.5 appear to be reasonable. */
    asize = (int) (bsize*MAX_DELTAFILLIN*1.3333);
    if(!LUSOL_sizeto(lu->LUSOL, newsize, newsize, asize))
      return( FALSE );
  }
  lu->dimcount = newsize;
  return( TRUE );
}


/* MUST MODIFY */
void BFP_CALLMODEL bfp_free(lprec *lp)
{
  INVrec *lu;

  lu = lp->invB;
  if(lu == NULL)
    return;

  /* General arrays */
  FREE(lu->opts);
  FREE(lu->value);

  /* Data specific to the factorization engine */
  LUSOL_free(lu->LUSOL);

  FREE(lu);
  lp->invB = NULL;
}


/* MUST MODIFY */
int BFP_CALLMODEL bfp_nonzeros(lprec *lp, MYBOOL maximum)
{
  INVrec *lu;

  lu = lp->invB;
  if(maximum == TRUE)
    return(lu->max_LUsize);
  else if(maximum == AUTOMATIC)
    return(lu->max_Bsize);
  else
    return(lu->LUSOL->luparm[LUSOL_IP_NONZEROS_L0]+lu->LUSOL->luparm[LUSOL_IP_NONZEROS_U0]);
/*    return(lu->LUSOL->luparm[LUSOL_IP_NONZEROS_ROW]); */
}


/* MUST MODIFY (or ignore) */
int BFP_CALLMODEL bfp_memallocated(lprec *lp)
{
  int      mem;
  LUSOLrec *LUSOL = lp->invB->LUSOL;

  mem = sizeof(REAL) * (LUSOL->lena+LUSOL->maxm+LUSOL_RP_LASTITEM);
  mem += sizeof(int) * (2*LUSOL->lena+5*LUSOL->maxm+5*LUSOL->maxn+LUSOL_IP_LASTITEM);
  if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TCP)
    mem += sizeof(REAL) * LUSOL->maxn + 2*sizeof(REAL)*LUSOL->maxn;
  else if(LUSOL->luparm[LUSOL_IP_PIVOTTYPE] == LUSOL_PIVMOD_TRP)
    mem += sizeof(REAL) * LUSOL->maxn;
  if(!LUSOL->luparm[LUSOL_IP_KEEPLU])
    mem += sizeof(REAL) * LUSOL->maxn;
  return( mem );
}


/* MUST MODIFY */
int BFP_CALLMODEL bfp_preparefactorization(lprec *lp)
{
  INVrec *lu = lp->invB;

  /* Finish any outstanding business */
  if(lu->is_dirty == AUTOMATIC)
    lp->bfp_finishfactorization(lp);

  /* Clear or resize the existing LU matrices - specific for the factorization engine */
  LUSOL_clear(lu->LUSOL, TRUE);
  if(lu->dimcount != lp->rows + bfp_rowoffset(lp))
    lp->bfp_resize(lp, lp->rows);

  /* Reset additional indicators */
  lp->bfp_updaterefactstats(lp);
  lu->col_pos = 0;

  return(0);

}


/* LOCAL HELPER ROUTINE - Replace a basis column with corresponding slack */
int bfp_LUSOLsetcolumn(lprec *lp, int posnr, int colnr)
{
  int nz, inform;

  nz = lp->get_lpcolumn(lp, colnr, lp->invB->LUSOL->w + bfp_rowoffset(lp), NULL, NULL);
  inform = LUSOL_replaceColumn(lp->invB->LUSOL, posnr, lp->invB->LUSOL->w);
  return( inform );
}


/* LOCAL HELPER ROUTINE - force the basis to be the identity matrix */
int bfp_LUSOLidentity(lprec *lp, int *rownum)
{
  int    i, nz;
  INVrec *invB = lp->invB;

  /* Reset the factorization engine */
  LUSOL_clear(invB->LUSOL, TRUE);

  /* Add the basis columns */
  lp->invB->set_Bidentity = TRUE;
  for(i = 1; i <= invB->dimcount; i++) {
    nz = lp->get_basiscolumn(lp, i, rownum, invB->value);
    LUSOL_loadColumn(invB->LUSOL, rownum, i, invB->value, nz, 0);
  }
  lp->invB->set_Bidentity = FALSE;

  /* Factorize */
  i = LUSOL_factorize(invB->LUSOL);

  return( i );
}


/* LOCAL HELPER ROUTINE */
int bfp_LUSOLfactorize(lprec *lp, MYBOOL *usedpos, int *rownum, int *singular)
{
  int    i, j, nz, deltarows = bfp_rowoffset(lp);
  INVrec *invB = lp->invB;

  /* Handle normal, presumed nonsingular case */
  if(singular == NULL) {

  /* Optionally do a symbolic minimum degree ordering;
     not that slack variables should not be processed */
/*#define UsePreprocessMDO*/
#ifdef UsePreprocessMDO
    int *mdo;
    mdo = lp->bfp_createMDO(lp, usedpos, lp->rows, TRUE);
    if(mdo != NULL) {
      for(i = 1; i <= lp->rows; i++)
        lp->set_basisvar(lp, i, mdo[i]);
      FREE(mdo);
    }
#endif

    /* Reset the factorization engine */
    LUSOL_clear(invB->LUSOL, TRUE);

    /* Add the basis columns in the original order */
    for(i = 1; i <= invB->dimcount; i++) {
      nz = lp->get_basiscolumn(lp, i, rownum, invB->value);
      LUSOL_loadColumn(invB->LUSOL, rownum, i, invB->value, nz, 0);
      if((i > deltarows) && (lp->var_basic[i-deltarows] > lp->rows))
        lp->invB->user_colcount++;
    }

    /* Factorize */
    i = LUSOL_factorize(invB->LUSOL);
  }

  /* Handle case where a column may be singular */
  else {
    LLrec *map;

    /* Reset the factorization engine */
    i = bfp_LUSOLidentity(lp, rownum);

    /* Build map of available columns */
    nz = createLink(lp->rows, &map, NULL);
    for(i = 1; i <= lp->rows; i++) {
      if(lp->var_basic[i] <= lp->rows)
        removeLink(map, i);
    }

    /* Rebuild the basis, column by column, while skipping slack columns */
    j = firstActiveLink(map);
    for(i = 1; i <= lp->rows; i++) {
      if(lp->var_basic[i] <= lp->rows)
        continue;
      nz = bfp_LUSOLsetcolumn(lp, j+deltarows, lp->var_basic[i]);
      if(nz == LUSOL_INFORM_LUSUCCESS)
        lp->invB->user_colcount++;
      else {
        nz = bfp_LUSOLsetcolumn(lp, j+deltarows, i);
        lp->set_basisvar(lp, i, i);
      }
      j = nextActiveLink(map, j);
    }

    /* Sort the basis list */
    MEMCOPY(rownum, lp->var_basic, lp->rows+1);
    sortByINT(lp->var_basic, rownum, lp->rows, 1, TRUE);

  }

  return( i );
}
/* LOCAL HELPER ROUTINE */
void bfp_LUSOLtighten(lprec *lp)
{
  int infolevel = DETAILED;

  switch(LUSOL_tightenpivot(lp->invB->LUSOL)) {
    case FALSE: lp->report(lp, infolevel, "bfp_factorize: Very hard numerics, but cannot tighten LUSOL thresholds further.\n");
                 break;
    case TRUE:  lp->report(lp, infolevel, "bfp_factorize: Frequent refact pivot count %d at iter %.0f; tightened thresholds.\n",
                                           lp->invB->num_pivots, (REAL) lp->get_total_iter(lp));
                 break;
    default:    lp->report(lp, infolevel, "bfp_factorize: LUSOL switched to %s pivoting model to enhance stability.\n",
                                           LUSOL_pivotLabel(lp->invB->LUSOL));
  }
}

#define is_fixedvar is_fixedvar_ /* resolves a compiler warning/error conflict with lp_lib.h */

static MYBOOL is_fixedvar(lprec *lp, int variable)
{
  if((lp->bb_bounds != NULL && lp->bb_bounds->UBzerobased) || (variable <= lp->rows))
    return( (MYBOOL) (lp->upbo[variable] < lp->epsprimal) );
  else
    return( (MYBOOL) (lp->upbo[variable]-lp->lowbo[variable] < lp->epsprimal) );
} /* is_fixedvar */

/* MUST MODIFY */
int BFP_CALLMODEL bfp_factorize(lprec *lp, int uservars, int Bsize, MYBOOL *usedpos, MYBOOL final)
{
  int      kcol, inform,
           *rownum = NULL,
           singularities = 0,
           dimsize = lp->invB->dimcount;
  LUSOLrec *LUSOL = lp->invB->LUSOL;

 /* Set dimensions and create work array */
  SETMAX(lp->invB->max_Bsize, Bsize+(1+lp->rows-uservars));
  kcol = lp->invB->dimcount;
  LUSOL->m = kcol;
  LUSOL->n = kcol;
  allocINT(lp, &rownum, kcol+1, FALSE);

 /* Check if the refactorization frequency is low;
    tighten pivot thresholds if appropriate */
  inform = lp->bfp_pivotcount(lp);
  if(!final &&                        /* No solution update-based refactorization */
     !lp->invB->force_refact &&       /* No sparsity-based refactorization */
     !lp->is_action(lp->spx_action,
          ACTION_TIMEDREINVERT) &&    /* No optimal time-based refactorization */
     (inform > 5) && (inform < 0.25*lp->bfp_pivotmax(lp)))
    bfp_LUSOLtighten(lp);


 /* Reload B and factorize */
  inform = bfp_LUSOLfactorize(lp, usedpos, rownum, NULL);

 /* Do some checks */
#ifdef Paranoia
  if(uservars != lp->invB->user_colcount) {
    lp->report(lp, SEVERE, "bfp_factorize: User variable count reconciliation failed\n");
    return( singularities );
  }
#endif

  /* Check result and do further remedial action if necessary */
  if(inform != LUSOL_INFORM_LUSUCCESS) {
    int  singularcols,
         replacedcols = 0;
    REAL hold;

    /* Make sure we do not tighten factorization pivot criteria too often, and simply
       accept the substitution of slack columns into the basis */
    if((lp->invB->num_singular+1) % TIGHTENAFTER == 0)
      bfp_LUSOLtighten(lp);

    /* Try to restore a non-singular basis by substituting singular columns with slacks */
    while((inform == LUSOL_INFORM_LUSINGULAR) && (replacedcols < dimsize)) {
      int    iLeave, jLeave, iEnter;
      MYBOOL isfixed;

      singularities++;
      singularcols = LUSOL->luparm[LUSOL_IP_SINGULARITIES];
      hold = (REAL) lp->get_total_iter(lp);
      lp->report(lp, NORMAL, "bfp_factorize: Resolving %d singularit%s at refact %d, iter %.0f\n",
                             singularcols, my_plural_y(singularcols), lp->invB->num_refact, hold);

      /* Find the failing / singular column(s) and make slack substitutions */
      for(kcol = 1; kcol <= singularcols; kcol++) {

        /* Determine leaving and entering columns. */
        iLeave = LUSOL_getSingularity(LUSOL, kcol);        /* This is the singular column as natural index */
        iEnter = iLeave;                                   /* This is the target replacement slack         */
#if 1
        iEnter = LUSOL->iqinv[iEnter];
        iEnter = LUSOL->ip[iEnter];
#endif
        iLeave-= bfp_rowextra(lp);                         /* This is the original B column/basis index    */
        jLeave = lp->var_basic[iLeave];                    /* This is the IA column index in lp_solve      */

        /* Express the slack index in original lp_solve [1..rows] reference and check validity */
 /*       if(B4 != NULL) iEnter = B4->B4_row[iEnter]; v6 FUNCTIONALITY */
        iEnter -=  bfp_rowextra(lp);
        if(lp->is_basic[iEnter]) {
          lp->report(lp, DETAILED, "bfp_factorize: Replacement slack %d is already basic!\n", iEnter);

          /* See if we can find a good alternative slack variable to enter */
          iEnter = 0;
          for(inform = 1; inform <= lp->rows; inform++)
            if(!lp->is_basic[inform]) {
              if((iEnter == 0) || (lp->upbo[inform] > lp->upbo[iEnter])) {
                iEnter = inform;
                if(my_infinite(lp, lp->upbo[iEnter]))
                  break;
              }
            }
          if(iEnter == 0) {
            lp->report(lp, SEVERE, "bfp_factorize: Could not find replacement slack variable!\n");
            break;
          }
        }

        /* We should update bound states for both the entering and leaving variables.
           Note that this may cause (primal or dual) infeasibility, but I assume that
           lp_solve traps this and takes necessary corrective action. */
        isfixed = is_fixedvar(lp, iEnter);
        if(isfixed)
          lp->fixedvars++;
        hold = lp->upbo[jLeave];
        lp->is_lower[jLeave] = isfixed || (fabs(hold)>=lp->infinite) || (lp->rhs[iLeave] < hold);
        lp->is_lower[iEnter] = TRUE;

        /* Do the basis replacement */
        lp->set_basisvar(lp, iLeave, iEnter);

      }

      /* Refactorize with slack substitutions */
      inform = bfp_LUSOLfactorize(lp, NULL, rownum, NULL);
      replacedcols += singularcols;
    }

    /* Check if we had a fundamental problem */
    if(singularities >= dimsize) {
      lp->report(lp, IMPORTANT, "bfp_factorize: LUSOL was unable to recover from a singular basis\n");
      lp->spx_status = NUMFAILURE;
    }
  }

  /* Clean up before returning */
  FREE(rownum);

  /* Update statistics */
  /* SETMAX(lp->invB->max_Bsize, (*Bsize)); */
  lp->invB->num_singular += singularities;    /* The total number of singular updates */

  return( singularities );
}

/* MUST MODIFY */
MYBOOL BFP_CALLMODEL bfp_finishupdate(lprec *lp, MYBOOL changesign)
/* Was addetacol() in versions of lp_solve before 4.0.1.8 - KE */
{
  int      i, k, kcol, deltarows = bfp_rowoffset(lp);
  REAL     DIAG, VNORM;
  INVrec   *lu = lp->invB;
  LUSOLrec *LUSOL = lu->LUSOL;

  if(!lu->is_dirty)
    return( FALSE );
  if(lu->is_dirty != AUTOMATIC)
    lu->is_dirty = FALSE;

  /* Perform the update */
  k = lu->col_pos+deltarows;
  lu->num_pivots++;
  if(lu->col_leave > lu->dimcount-deltarows)
    lu->user_colcount--;
  if(lu->col_enter > lu->dimcount-deltarows)
    lu->user_colcount++;
  kcol = lu->col_pos;
  lu->col_pos = 0;

  /* Do standard update */
#ifdef LUSOLSafeFastUpdate      /* NB! Defined in lusol.h */
  if(TRUE || !changesign) {
    if(changesign) {
      REAL *temp = LUSOL->vLU6L;
      for(i = 1, temp++; i <= lp->rows+deltarows; i++, temp++)
        if(*temp != 0)
          *temp = -(*temp);
    }
    /* Execute the update using data prepared earlier */
    LU8RPC(LUSOL, LUSOL_UPDATE_OLDNONEMPTY, LUSOL_UPDATE_USEPREPARED,
           k, NULL, NULL, &i, &DIAG, &VNORM);
  }
  else
#endif
  {
    /* Retrieve the data for the entering column (base 0) */
    i = lp->get_lpcolumn(lp, lu->col_enter, lu->value+deltarows, NULL, NULL);
    lu->value[0] = 0;
    /* Execute the update */
    LU8RPC(LUSOL, LUSOL_UPDATE_OLDNONEMPTY, LUSOL_UPDATE_NEWNONEMPTY,
           k, lu->value, NULL, &i, &DIAG, &VNORM);
  }

  if(i == LUSOL_INFORM_LUSUCCESS) {

    /* Check if we should refactorize based on accumulation of fill-in */
    DIAG  = LUSOL->luparm[LUSOL_IP_NONZEROS_L]+LUSOL->luparm[LUSOL_IP_NONZEROS_U];
    VNORM = LUSOL->luparm[LUSOL_IP_NONZEROS_L0]+LUSOL->luparm[LUSOL_IP_NONZEROS_U0];
#if 0
    /* This is Michael Saunder's fixed parameter */
    VNORM *= MAX_DELTAFILLIN;
#else
    /* This is Kjell Eikland's dynamic error accumulation measure */
    VNORM *= pow(MAX_DELTAFILLIN, pow((0.5*LUSOL->nelem/VNORM), 0.25));
#endif
    lu->force_refact = (MYBOOL) ((DIAG > VNORM) && (lu->num_pivots > 20));

#if 0
    /* Additional KE logic to reduce maximum pivot count based on the density of B */
    if(!lu->force_refact) {
      VNORM = lp->rows+1;
      VNORM = 1.0 - pow((REAL) LUSOL->nelem/VNORM/VNORM, 0.2);
      lu->force_refact = (MYBOOL) (lu->num_pivots > VNORM*lp->bfp_pivotmax(lp));
    }
#endif
  }

  /* Handle errors */
  else {
/*    int infolevel = NORMAL; */
    int infolevel = DETAILED;
    lp->report(lp, infolevel, "bfp_finishupdate: Failed at iter %.0f, pivot %d;\n%s\n",
                   (REAL) (lp->total_iter+lp->current_iter), lu->num_pivots, LUSOL_informstr(LUSOL, i));
    if(i == LUSOL_INFORM_ANEEDMEM) {       /* To compress used memory and realloc, if necessary */
      lp->invert(lp, INITSOL_USEZERO, FALSE);
      if(i != LUSOL_INFORM_LUSUCCESS)
        lp->report(lp, NORMAL, "bfp_finishupdate: Insufficient memory at iter %.0f;\n%s\n",
                       (REAL) (lp->total_iter+lp->current_iter), LUSOL_informstr(LUSOL, i));
    }
    else if(i == LUSOL_INFORM_RANKLOSS) {  /* To fix rank loss and clear cumulative errors */
#if 0
      /* This is test code to do pivot in slack BEFORE refactorization (pessimistic approach);
        assumes that LUSOL returns correct information about the source of the singularity */
      kcol = LUSOL->luparm[LUSOL_IP_SINGULARINDEX];
#ifdef MAPSINGULARCOLUMN
      kcol = LUSOL_findColumnPosition(LUSOL, kcol);
#endif
      lp->set_basisvar(lp, kcol-deltarows, kcol-deltarows);
#endif
      lp->invert(lp, INITSOL_USEZERO, FALSE);
      i = LUSOL->luparm[LUSOL_IP_INFORM];
      if(i != LUSOL_INFORM_LUSUCCESS)
        lp->report(lp, NORMAL, "bfp_finishupdate: Recovery attempt unsuccessful at iter %.0f;\n%s\n",
                       (REAL) (lp->total_iter+lp->current_iter), LUSOL_informstr(LUSOL, i));
      else
        lp->report(lp, infolevel, "bfp_finishupdate: Correction or recovery was successful.\n");
    }
  }
  return( (MYBOOL) (i == LUSOL_INFORM_LUSUCCESS) );

} /* bfp_finishupdate */


/* MUST MODIFY */
void BFP_CALLMODEL bfp_ftran_normal(lprec *lp, REAL *pcol, int *nzidx)
{
  int    i;
  INVrec *lu;

  lu = lp->invB;

  /* Do the LUSOL ftran */
  i = LUSOL_ftran(lu->LUSOL, pcol-bfp_rowoffset(lp), nzidx, FALSE);
  if(i != LUSOL_INFORM_LUSUCCESS) {
    lu->status = BFP_STATUS_ERROR;
    lp->report(lp, NORMAL, "bfp_ftran_normal: Failed at iter %.0f, pivot %d;\n%s\n",
                   (REAL) (lp->total_iter+lp->current_iter), lu->num_pivots, LUSOL_informstr(lu->LUSOL, i));
  }
}


/* MAY MODIFY */
void BFP_CALLMODEL bfp_ftran_prepare(lprec *lp, REAL *pcol, int *nzidx)
{
  int    i;
  INVrec *lu;

  lu = lp->invB;

  /* Do the LUSOL ftran */
  i = LUSOL_ftran(lu->LUSOL, pcol-bfp_rowoffset(lp), nzidx, TRUE);
  if(i != LUSOL_INFORM_LUSUCCESS) {
    lu->status = BFP_STATUS_ERROR;
    lp->report(lp, NORMAL, "bfp_ftran_prepare: Failed at iter %.0f, pivot %d;\n%s\n",
                   (REAL) (lp->total_iter+lp->current_iter), lu->num_pivots, LUSOL_informstr(lu->LUSOL, i));
  }
}


/* MUST MODIFY */
void BFP_CALLMODEL bfp_btran_normal(lprec *lp, REAL *prow, int *nzidx)
{
  int    i;
  INVrec *lu;

  lu = lp->invB;

  /* Do the LUSOL btran */
  i = LUSOL_btran(lu->LUSOL, prow-bfp_rowoffset(lp), nzidx);
  if(i != LUSOL_INFORM_LUSUCCESS) {
    lu->status = BFP_STATUS_ERROR;
    lp->report(lp, NORMAL, "bfp_btran_normal: Failed at iter %.0f, pivot %d;\n%s\n",
                   (REAL) (lp->total_iter+lp->current_iter), lu->num_pivots, LUSOL_informstr(lu->LUSOL, i));
  }

  /* Check performance data */
#if 0
  if(lu->num_pivots == 1) {
    if(lu->LUSOL->luparm[LUSOL_IP_ACCELERATION] > 0)
      lp->report(lp, NORMAL, "RowL0 R:%10.7f  C:%10.7f  NZ:%10.7f\n",
                             (REAL) lu->LUSOL->luparm[LUSOL_IP_ROWCOUNT_L0] / lu->LUSOL->m,
                             (REAL) lu->LUSOL->luparm[LUSOL_IP_COLCOUNT_L0] / lu->LUSOL->m,
                             (REAL) lu->LUSOL->luparm[LUSOL_IP_NONZEROS_L0] / pow((REAL) lu->LUSOL->m, 2));
    else
      lp->report(lp, NORMAL, "ColL0 C:%10.7f  NZ:%10.7f\n",
                             (REAL) lu->LUSOL->luparm[LUSOL_IP_COLCOUNT_L0] / lu->LUSOL->m,
                             (REAL) lu->LUSOL->luparm[LUSOL_IP_NONZEROS_L0] / pow((REAL) lu->LUSOL->m, 2));
  }
#endif

}

/* MUST MODIFY - Routine to find maximum rank of equality constraints */
int BFP_CALLMODEL bfp_findredundant(lprec *lp, int items, getcolumnex_func cb, int *maprow, int *mapcol)
{
  int       i, j, nz = 0, m = 0, n = 0, *nzrows = NULL;
  REAL      *nzvalues = NULL, *arraymax = NULL;
  LUSOLrec  *LUSOL;

  /* Are we capable of finding redundancy with this BFP? */
  if((maprow == NULL) && (mapcol == NULL))
    return( n );

  /* If so, initialize memory structures */
  if(!allocINT(lp, &nzrows, items, FALSE) ||
     !allocREAL(lp, &nzvalues, items, FALSE))
    return( n );

  /* Compute the number of non-empty columns */
  m = 0;
  for(j = 1; j <= mapcol[0]; j++) {
    n = cb(lp, mapcol[j], NULL, NULL, maprow);
    if(n > 0) {
      m++;
      mapcol[m] = mapcol[j];
      nz += n;
    }
  }
  mapcol[0] = m;

  /* Instantiate a LUSOL object */
  LUSOL = LUSOL_create(NULL, 0, LUSOL_PIVMOD_TRP, 0);
  if((LUSOL == NULL) || !LUSOL_sizeto(LUSOL, items, m, nz*LUSOL_MULT_nz_a))
    goto Finish;

  /* Modify relevant LUSOL parameters */
  LUSOL->m = items;
  LUSOL->n = m;
#if 0
  LUSOL->luparm[LUSOL_IP_KEEPLU]        = FALSE;
  LUSOL->luparm[LUSOL_IP_PIVOTTYPE]     = LUSOL_PIVMOD_TRP;
  LUSOL->parmlu[LUSOL_RP_FACTORMAX_Lij] = 2.0;
#endif

  /* Load the columns into LUSOL */
  for(j = 1; j <= m; j++) {
    n = cb(lp, mapcol[j], nzvalues, nzrows, maprow);
    i = LUSOL_loadColumn(LUSOL, nzrows, j, nzvalues, n, -1);
    if(n != i) {
      lp->report(lp, IMPORTANT, "bfp_findredundant: Error %d while loading column %d with %d nz\n",
                                i, j, n);
      n = 0;
      goto Finish;
    }
  }

  /* Scale rows to prevent numerical problems */
  if((lp->scalemode != SCALE_NONE) && allocREAL(lp, &arraymax, items+1, TRUE)) {
    for(i = 1; i <= nz; i++) {
      SETMAX(arraymax[LUSOL->indc[i]], fabs(LUSOL->a[i]));
    }
    for(i = 1; i <= nz; i++)
      LUSOL->a[i] /= arraymax[LUSOL->indc[i]];
    FREE(arraymax);
  }

  /* Factorize for maximum rank */
  n = 0;
  i = LUSOL_factorize(LUSOL);
  /*  lp->report(lp, NORMAL, "bfp_findredundant: r=%d c=%d - %s\n", items, m, LUSOL_informstr(LUSOL, i));*/
  if((i == LUSOL_INFORM_LUSUCCESS) || (i != LUSOL_INFORM_LUSINGULAR))
    goto Finish;

  /* We have a singular matrix, obtain the indeces of the singular rows */
  for(i = LUSOL->luparm[LUSOL_IP_RANK_U] + 1; i <= items; i++) {
    n++;
    maprow[n] = LUSOL->ip[i];
  }
  maprow[0] = n;

  /* Clean up */
Finish:
  LUSOL_free(LUSOL);
  FREE(nzrows);
  FREE(nzvalues);

  return( n );
}
