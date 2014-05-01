
#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_report.h"
#include "lp_pricePSE.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


/*
    Advanced simplex price scaling modules - w/interface for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Kjell Eikland
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      lp_lib.h

    Release notes:
    v1.0.0  1 September 2003    Implementation of DEVEX and STEEPEST EDGE
                                routines for the primal and dual simplex.
    v1.0.1  1 January 2004      Made initial value of weight of ingoing
                                variable for the standard mode of DEVEX
                                consistent with the initialization at restart;
                                original version could at worst contribute
                                to cycling.
    v1.0.2  23 March 2004       Added floors to Steepest Edge updates and
                                moved tests for tiny update higher. Previous
                                logic can be simulated by disabling the compiler
                                define ApplySteepestEdgeMinimum.
    v1.1.0  1 July 2004         Renamed from lp_pricerPSE to lp_pricePSE in
                                conjuction with the creation of a separate
                                price library.
    v1.2.0  1 March 2005        Changed memory allocation routines to use
                                standard lp_solve functions, improve error handling
                                and return boolean status values.

   ----------------------------------------------------------------------------------
*/

INLINE MYBOOL applyPricer(lprec *lp)
{
  int rule = get_piv_rule(lp);
  return( (MYBOOL) ((rule == PRICER_DEVEX) || (rule == PRICER_STEEPESTEDGE)) );
}


STATIC void simplexPricer(lprec *lp, MYBOOL isdual)
{
  if(lp->edgeVector != NULL)
    lp->edgeVector[0] = (REAL) isdual;
}


STATIC void freePricer(lprec *lp)
{
  FREE(lp->edgeVector);
}


STATIC MYBOOL resizePricer(lprec *lp)
{
  if(!applyPricer(lp))
    return( TRUE );

  /* Reallocate vector for new size */
  if(!allocREAL(lp, &(lp->edgeVector), lp->sum_alloc+1, AUTOMATIC))
    return( FALSE );

  /* Signal that we have not yet initialized the price vector */
  MEMCLEAR(lp->edgeVector, lp->sum_alloc+1);
  lp->edgeVector[0] = -1;
  return( TRUE );
}


STATIC MYBOOL initPricer(lprec *lp)
{
  if(!applyPricer(lp))
    return( FALSE );

  /* Free any pre-existing pricer */
  freePricer(lp);

  /* Allocate vector to fit current problem size */
  return( resizePricer(lp) );
}


STATIC REAL getPricer(lprec *lp, int item, MYBOOL isdual)
{
  REAL value = 1.0;

  if(!applyPricer(lp))
    return( value );

  value = *lp->edgeVector;

  /* Make sure we have a price vector to use */
  if(value < 0) {
#ifdef Paranoia
    report(lp, SEVERE, "getPricer: Called without having being initialized!\n");
#endif
    return( 1.0 );
  }
  /* We may be calling the primal from the dual (and vice-versa) for validation
     of feasibility; ignore calling origin and simply return 1 */
  else if(isdual != value) {
    return( 1.0 );
  }
  /* Do the normal norm retrieval */
  else {

    if(isdual)
      item = lp->var_basic[item];

    value = lp->edgeVector[item];

    if(value == 0) {
      value = 1.0;
      report(lp, SEVERE, "getPricer: Detected a zero-valued price at index %d\n", item);
    }
#ifdef Paranoia
    else if(value < 0)
      report(lp, SEVERE, "getPricer: Invalid %s reduced cost norm %g at index %d\n",
                          my_if(isdual, "dual", "primal"), value, item);
#endif

  /* Return the norm */
    return( sqrt(value) );
  }
}

STATIC MYBOOL restartPricer(lprec *lp, MYBOOL isdual)
{
  REAL   *sEdge = NULL, seNorm, hold;
  int    i, j, m;
  MYBOOL isDEVEX, ok = applyPricer(lp);

  if(!ok)
    return( ok );

  /* Store the active/current pricing type */
  if(isdual == AUTOMATIC)
    isdual = (MYBOOL) lp->edgeVector[0];
  else
    lp->edgeVector[0] = isdual;

  m = lp->rows;

  /* Determine strategy and check if we have strategy fallback for the primal */
  isDEVEX = is_piv_rule(lp, PRICER_DEVEX);
  if(!isDEVEX && !isdual)
    isDEVEX = is_piv_mode(lp, PRICE_PRIMALFALLBACK);

  /* Check if we only need to do the simple DEVEX initialization */
  if(!is_piv_mode(lp, PRICE_TRUENORMINIT)) {
    if(isdual) {
      for(i = 1; i <= m; i++)
        lp->edgeVector[lp->var_basic[i]] = 1.0;
    }
    else {
      for(i = 1; i <= lp->sum; i++)
        if(!lp->is_basic[i])
          lp->edgeVector[i] = 1.0;
    }
    return( ok );
  }

  /* Otherwise do the full Steepest Edge norm initialization */
  ok = allocREAL(lp, &sEdge, m+1, FALSE);
  if(!ok)
    return( ok );

  if(isdual) {

   /* Extract the rows of the basis inverse and compute their squared norms */

    for(i = 1; i <= m; i++) {

      bsolve(lp, i, sEdge, NULL, 0, 0.0);

      /* Compute the edge norm */
      seNorm = 0;
      for(j = 1; j <= m; j++) {
        hold = sEdge[j];
        seNorm += hold*hold;
      }

      j = lp->var_basic[i];
      lp->edgeVector[j] = seNorm;
    }

  }
  else {

   /* Solve a=Bb for b over all non-basic variables and compute their squared norms */

    for(i = 1; i <= lp->sum; i++) {
      if(lp->is_basic[i])
        continue;

      fsolve(lp, i, sEdge, NULL, 0, 0.0, FALSE);

      /* Compute the edge norm */
      seNorm = 1;
      for(j = 1; j <= m; j++) {
        hold = sEdge[j];
        seNorm += hold*hold;
      }

      lp->edgeVector[i] = seNorm;
    }

  }

  FREE(sEdge);

  return( ok );

}


STATIC MYBOOL formWeights(lprec *lp, int colnr, REAL *pcol, REAL **w)
/* This computes Bw = a, where B is the basis and a is a column of A */
{
  MYBOOL ok = allocREAL(lp, w, lp->rows+1, FALSE);

  if(ok) {
    if(pcol == NULL)
      fsolve(lp, colnr, *w, NULL, 0.0, 0.0, FALSE);
    else {
      MEMCOPY(*w, pcol, lp->rows+1);
/*    *w[0] = 0; */ /* Test */
    }
  }
/*
  if(pcol != NULL) {
    REAL cEdge, hold;
    int  i;

    cEdge = 0;
    for(i = 1; i <= m; i++) {
      hold = *w[i]-pcol[i];
      cEdge += hold*hold;
    }
    cEdge /= m;
    cEdge = sqrt(cEdge);
    if(cEdge > lp->epspivot)
      report(lp, SEVERE, "updatePricer: MRS error is %g\n", cEdge);
  }
*/
  return(ok);
}
STATIC void freeWeights(REAL *w)
{
  FREE(w);
}


STATIC MYBOOL updatePricer(lprec *lp, int rownr, int colnr, REAL *pcol, REAL *prow, int *nzprow)
{
  REAL   *vEdge = NULL, cEdge, hold, *newEdge, *w = NULL;
  int    i, m, n, exitcol, errlevel = DETAILED;
  MYBOOL forceRefresh = FALSE, isDual, isDEVEX, ok = FALSE;

  if(!applyPricer(lp))
    return(ok);

  /* Make sure we have something to update */
  hold = lp->edgeVector[0];
  if(hold < 0)
    return(ok);
  isDual = (MYBOOL) (hold > 0);

  /* Do common initializations and computations */
  m = lp->rows;
  n = lp->sum;
  isDEVEX = is_piv_rule(lp, PRICER_DEVEX);
  exitcol = lp->var_basic[rownr];

  /* Solve/copy Bw = a */
#if 0
  ok = formWeights(lp, colnr, NULL, &w);  /* Compute from scratch - Experimental */
#else
  ok = formWeights(lp, colnr, pcol, &w);  /* Use previously computed values */
#endif
  if(!ok)
    return( ok );

  /* Price norms for the dual simplex - the basic columns */
  if(isDual) {
    REAL rw;
    int  targetcol;

    /* Don't need to compute cross-products with DEVEX */
    if(!isDEVEX) {
      ok = allocREAL(lp, &vEdge, m+1, FALSE);
      if(!ok)
        return( ok );

    /* Extract the row of the inverse containing the leaving variable
       and then form the dot products against the other variables, i.e. "Tau" */
#if 0 /* Extract row explicitly */
      bsolve(lp, rownr, vEdge, 0, 0.0);
#else /* Reuse previously extracted row data */
      MEMCOPY(vEdge, prow, m+1);
      vEdge[0] = 0;
#endif
      lp->bfp_ftran_normal(lp, vEdge, NULL);
    }

    /* Update the squared steepest edge norms; first store some constants */
    cEdge = lp->edgeVector[exitcol];
    rw = w[rownr];
    if(fabs(rw) < lp->epspivot) {
      forceRefresh = TRUE;
      goto Finish2;
    }

   /* Deal with the variable entering the basis to become a new leaving candidate */
    hold = 1 / rw;
    lp->edgeVector[colnr] = (hold*hold) * cEdge;

#ifdef Paranoia
    if(lp->edgeVector[colnr] <= lp->epsmachine)
      report(lp, errlevel, "updatePricer: Invalid dual norm %g at entering index %d - iteration %.0f\n",
                           lp->edgeVector[colnr], rownr, (double) (lp->total_iter+lp->current_iter));
#endif

   /* Then loop over all basic variables, but skip the leaving row */
    for(i = 1; i <= m; i++) {
      if(i == rownr)
        continue;
      targetcol = lp->var_basic[i];
      hold = w[i];
      if(hold == 0)
        continue;
      hold /= rw;
      if(fabs(hold) < lp->epsmachine)
        continue;

      newEdge = &(lp->edgeVector[targetcol]);
      *newEdge += (hold*hold) * cEdge;
      if(isDEVEX) {
        if((*newEdge) > DEVEX_RESTARTLIMIT) {
          forceRefresh = TRUE;
          break;
        }
      }
      else {
        *newEdge -= 2*hold*vEdge[i];
#ifdef xxApplySteepestEdgeMinimum
        SETMAX(*newEdge, hold*hold+1); /* Kludge; use the primal lower bound */
#else
        if(*newEdge <= 0) {
          report(lp, errlevel, "updatePricer: Invalid dual norm %g at index %d - iteration %.0f\n",
                                *newEdge, i, (double) (lp->total_iter+lp->current_iter));
          forceRefresh = TRUE;
          break;
        }
#endif
      }
    }


  }
  /* Price norms for the primal simplex - the non-basic columns */
  else {

    REAL *vTemp = NULL, *vAlpha = NULL, cAlpha;
    int  *coltarget;

    ok = allocREAL(lp, &vTemp, m+1, TRUE) &&
         allocREAL(lp, &vAlpha, n+1, TRUE);
    if(!ok)
      return( ok );

    /* Check if we have strategy fallback for the primal */
    if(!isDEVEX)
      isDEVEX = is_piv_mode(lp, PRICE_PRIMALFALLBACK);

    /* Initialize column target array */
    coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->sum+1, sizeof(*coltarget));
    ok = get_colIndexA(lp, SCAN_SLACKVARS+SCAN_USERVARS+USE_NONBASICVARS, coltarget, FALSE);
    if(!ok) {
      mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
      return( ok );
    }

    /* Don't need to compute cross-products with DEVEX */
    if(!isDEVEX) {
      ok = allocREAL(lp, &vEdge, n+1, TRUE);
      if(!ok)
        return( ok );

      /* Compute v and then N'v */
      MEMCOPY(vTemp, w, m+1);
      bsolve(lp, -1, vTemp, NULL, lp->epsmachine*DOUBLEROUND, 0.0);
      vTemp[0] = 0;
      prod_xA(lp, coltarget, vTemp, NULL, lp->epsmachine, 0.0,
                             vEdge, NULL, MAT_ROUNDDEFAULT);
    }

    /* Compute Sigma and then Alpha */
    bsolve(lp, rownr, vTemp, NULL, 0*DOUBLEROUND, 0.0);
    vTemp[0] = 0;
    prod_xA(lp, coltarget, vTemp, NULL, lp->epsmachine, 0.0,
                           vAlpha, NULL, MAT_ROUNDDEFAULT);
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);

    /* Update the squared steepest edge norms; first store some constants */
    cEdge = lp->edgeVector[colnr];
    cAlpha = vAlpha[colnr];
    if(fabs(cAlpha) < lp->epspivot) {
      forceRefresh = TRUE;
      goto Finish1;
    }

    /* Deal with the variable leaving the basis to become a new entry candidate */
    hold = 1 / cAlpha;
    lp->edgeVector[exitcol] = (hold*hold) * cEdge;

#ifdef Paranoia
    if(lp->edgeVector[exitcol] <= lp->epsmachine)
      report(lp, errlevel, "updatePricer: Invalid primal norm %g at leaving index %d - iteration %.0f\n",
                          lp->edgeVector[exitcol], exitcol, (double) (lp->total_iter+lp->current_iter));
#endif

    /* Then loop over all non-basic variables, but skip the entering column */
    for(i = 1; i <= lp->sum; i++) {
      if(lp->is_basic[i] || (i == colnr))
        continue;
      hold = vAlpha[i];
      if(hold == 0)
        continue;
      hold /= cAlpha;
      if(fabs(hold) < lp->epsmachine)
        continue;

      newEdge = &(lp->edgeVector[i]);
      *newEdge += (hold*hold) * cEdge;
      if(isDEVEX) {
        if((*newEdge) > DEVEX_RESTARTLIMIT) {
          forceRefresh = TRUE;
          break;
        }
      }
      else {
        *newEdge -= 2*hold*vEdge[i];
#ifdef ApplySteepestEdgeMinimum
        SETMAX(*newEdge, hold*hold+1);
#else
        if(*newEdge < 0) {
          report(lp, errlevel, "updatePricer: Invalid primal norm %g at index %d - iteration %.0f\n",
                               *newEdge, i, (double) (lp->total_iter+lp->current_iter));
          if(lp->spx_trace)
            report(lp, errlevel, "Error detail: (RelAlpha=%g, vEdge=%g, cEdge=%g)\n", hold, vEdge[i], cEdge);
          forceRefresh = TRUE;
          break;
        }
#endif
      }
    }

Finish1:
    FREE(vAlpha);
    FREE(vTemp);

  }

Finish2:
  FREE(vEdge);
  freeWeights(w);

  if(forceRefresh)
    ok = restartPricer(lp, AUTOMATIC);
  else
    ok = TRUE;

  return( ok );

}


STATIC MYBOOL verifyPricer(lprec *lp)
{
  REAL value;
  int  i, n;
  MYBOOL ok = applyPricer(lp);

  if(!ok)
    return( ok );
  ok = FALSE;

  /* Verify */
  if(lp->edgeVector == NULL)
    return( ok );
  value = *lp->edgeVector;
  if(value < 0)
    return( ok );

  /* Check the primal */
  n = 1;
  if(value == 0) {

    for(n = lp->sum; n > 0; n--) {
      if(lp->is_basic[n])
        continue;
      value = lp->edgeVector[n];
      if(value <= 0)
        break;
    }
  }
  /* Check the dual */
  else {
    for(i = lp->rows; i > 0; i--) {
      n = lp->var_basic[i];
      value = lp->edgeVector[n];
      if(value <= 0)
        break;
    }
  }

  ok = (MYBOOL) (n == 0);
#ifdef Paranoia
  if(!ok)
    report(lp, SEVERE, "verifyPricer: Invalid norm %g at index %d\n",
                       value, n);
#endif
  return( ok );
}

