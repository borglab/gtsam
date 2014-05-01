
/*
    Core optimization drivers for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Michel Berkelaar (to lp_solve v3.2),
                   Kjell Eikland    (v4.0 and forward)
    Contact:
    License terms: LGPL.

    Requires:      lp_lib.h, lp_simplex.h, lp_presolve.h, lp_pricerPSE.h

    Release notes:
    v5.0.0  1 January 2004      New unit applying stacked basis and bounds storage.
    v5.0.1 31 January 2004      Moved B&B routines to separate file and implemented
                                a new runsolver() general purpose call method.
    v5.0.2  1 May 2004          Changed routine names to be more intuitive.
    v5.1.0  10 January 2005     Created modular stalling/cycling functions.
                                Rewrote dualloop() to optimize long dual and
                                also streamlined primloop() correspondingly.
    v5.2.0  20 March 2005       Reimplemented primal phase 1 logic.
                                Made multiple pricing finally work (primal simplex).

   ----------------------------------------------------------------------------------
*/

#include <string.h>
#include "commonlib.h"
#include "lp_lib.h"
#include "lp_BFP.h"
#include "lp_simplex.h"
#include "lp_crash.h"
#include "lp_presolve.h"
#include "lp_price.h"
#include "lp_pricePSE.h"
#include "lp_report.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


STATIC void stallMonitor_update(lprec *lp, REAL newOF)
{
  int newpos;
  OBJmonrec *monitor = lp->monitor;

  if(monitor->countstep < OBJ_STEPS)
    monitor->countstep++;
  else
    monitor->startstep = mod(monitor->startstep + 1, OBJ_STEPS);
  newpos = mod(monitor->startstep + monitor->countstep - 1, OBJ_STEPS);
  monitor->objstep[newpos] = newOF;
  monitor->idxstep[newpos] = monitor->Icount;
  monitor->currentstep = newpos;
}

STATIC MYBOOL stallMonitor_creepingObj(lprec *lp)
{
  OBJmonrec *monitor = lp->monitor;

  if(monitor->countstep > 1) {
    REAL deltaOF = (monitor->objstep[monitor->currentstep] -
                    monitor->objstep[monitor->startstep]) / monitor->countstep;
    deltaOF /= MAX(1, (monitor->idxstep[monitor->currentstep] -
                       monitor->idxstep[monitor->startstep]));
    deltaOF = my_chsign(monitor->isdual, deltaOF);
    return( (MYBOOL) (deltaOF < monitor->epsvalue) );
  }
  else
    return( FALSE );
}

STATIC MYBOOL stallMonitor_shortSteps(lprec *lp)
{
  OBJmonrec *monitor = lp->monitor;

  if(monitor->countstep == OBJ_STEPS) {
    REAL deltaOF = MAX(1, (monitor->idxstep[monitor->currentstep] -
                           monitor->idxstep[monitor->startstep])) / monitor->countstep;
    deltaOF = pow(deltaOF*OBJ_STEPS, 0.66);
    return( (MYBOOL) (deltaOF > monitor->limitstall[TRUE]) );
  }
  else
    return( FALSE );
}

STATIC void stallMonitor_reset(lprec *lp)
{
  OBJmonrec *monitor = lp->monitor;

  monitor->ruleswitches = 0;
  monitor->Ncycle = 0;
  monitor->Mcycle = 0;
  monitor->Icount = 0;
  monitor->startstep = 0;
  monitor->objstep[monitor->startstep] = lp->infinite;
  monitor->idxstep[monitor->startstep] = monitor->Icount;
  monitor->prevobj = 0;
  monitor->countstep = 1;
}

STATIC MYBOOL stallMonitor_create(lprec *lp, MYBOOL isdual, char *funcname)
{
  OBJmonrec *monitor = NULL;
  if(lp->monitor != NULL)
    return( FALSE );

  monitor = (OBJmonrec *) calloc(sizeof(*monitor), 1);
  if(monitor == NULL)
    return( FALSE );

  monitor->lp = lp;
  strcpy(monitor->spxfunc, funcname);
  monitor->isdual = isdual;
  monitor->pivdynamic = is_piv_mode(lp, PRICE_ADAPTIVE);
  monitor->oldpivstrategy = lp->piv_strategy;
  monitor->oldpivrule = get_piv_rule(lp);
  if(MAX_STALLCOUNT <= 1)
    monitor->limitstall[FALSE] = 0;
  else
    monitor->limitstall[FALSE] = MAX(MAX_STALLCOUNT,
                                     (int) pow((REAL) (lp->rows+lp->columns)/2, 0.667));
#if 1
  monitor->limitstall[FALSE] *= 2+2;  /* Expand degeneracy/stalling tolerance range */
#endif
  monitor->limitstall[TRUE] = monitor->limitstall[FALSE];
  if(monitor->oldpivrule == PRICER_DEVEX) /* Increase tolerance since primal Steepest Edge is expensive */
    monitor->limitstall[TRUE] *= 2;
  if(MAX_RULESWITCH <= 0)
    monitor->limitruleswitches = MAXINT32;
  else
    monitor->limitruleswitches = MAX(MAX_RULESWITCH,
                                     lp->rows/MAX_RULESWITCH);
  monitor->epsvalue = lp->epsprimal; /* lp->epsvalue; */
  lp->monitor = monitor;
  stallMonitor_reset(lp);
  lp->suminfeas = lp->infinite;
  return( TRUE );
}

STATIC MYBOOL stallMonitor_check(lprec *lp, int rownr, int colnr, int lastnr,
                                 MYBOOL minit, MYBOOL approved, MYBOOL *forceoutEQ)
{
  OBJmonrec *monitor = lp->monitor;
  MYBOOL    isStalled, isCreeping, acceptance = TRUE;
  int       altrule,
#ifdef Paranoia
         msglevel = NORMAL;
#else
         msglevel = DETAILED;
#endif
  REAL   deltaobj = lp->suminfeas;

  /* Accept unconditionally if this is the first or second call */
  monitor->active = FALSE;
  if(monitor->Icount <= 1) {
    if(monitor->Icount == 1) {
      monitor->prevobj = lp->rhs[0];
      monitor->previnfeas = deltaobj;
    }
    monitor->Icount++;
    return( acceptance );
  }

  /* Define progress as primal objective less sum of (primal/dual) infeasibilities */
  monitor->thisobj = lp->rhs[0];
  monitor->thisinfeas = deltaobj;
  if(lp->spx_trace &&
     (lastnr > 0))
    report(lp, NORMAL, "%s: Objective at iter %10.0f is " RESULTVALUEMASK " (%4d: %4d %s- %4d)\n",
                       monitor->spxfunc,
                       (double) get_total_iter(lp), monitor->thisobj, rownr, lastnr,
                       my_if(minit == ITERATE_MAJORMAJOR, "<","|"), colnr);
  monitor->pivrule = get_piv_rule(lp);

  /* Check if we have a stationary solution at selected tolerance level;
     allow some difference in case we just refactorized the basis. */
  deltaobj = my_reldiff(monitor->thisobj, monitor->prevobj);
  deltaobj = fabs(deltaobj); /* Pre v5.2 version */
  isStalled = (MYBOOL) (deltaobj < monitor->epsvalue);

  /* Also require that we have a measure of infeasibility-stalling */
  if(isStalled) {
    REAL testvalue, refvalue = monitor->epsvalue;
#if 1
    if(monitor->isdual)
      refvalue *= 1000*log10(9.0+lp->rows);
    else
      refvalue *= 1000*log10(9.0+lp->columns);
#else
      refvalue *= 1000*log10(9.0+lp->sum);
#endif
    testvalue = my_reldiff(monitor->thisinfeas, monitor->previnfeas);
    isStalled &= (fabs(testvalue) < refvalue);

    /* Check if we should force "major" pivoting, i.e. no bound flips;
      this is activated when we see the feasibility deteriorate */
/*    if(!isStalled && (testvalue > 0) && (TRUE || is_action(lp->anti_degen, ANTIDEGEN_BOUNDFLIP))) */
#if !defined _PRICE_NOBOUNDFLIP
    if(!isStalled && (testvalue > 0) && is_action(lp->anti_degen, ANTIDEGEN_BOUNDFLIP))
      acceptance = AUTOMATIC;
  }
#else
    if(!isStalled && (testvalue > 0) && !ISMASKSET(lp->piv_strategy, PRICE_NOBOUNDFLIP)) {
      SETMASK(lp->piv_strategy, PRICE_NOBOUNDFLIP);
      acceptance = AUTOMATIC;
    }
  }
  else
    CLEARMASK(lp->piv_strategy, PRICE_NOBOUNDFLIP);
#endif

#if 1
  isCreeping = FALSE;
#else
  isCreeping |= stallMonitor_creepingObj(lp);
/*  isCreeping |= stallMonitor_shortSteps(lp); */
#endif
  if(isStalled || isCreeping) {

    /* Update counters along with specific tolerance for bound flips */
#if 1
    if(minit != ITERATE_MAJORMAJOR) {
      if(++monitor->Mcycle > 2) {
        monitor->Mcycle = 0;
        monitor->Ncycle++;
      }
    }
    else
#endif
      monitor->Ncycle++;

    /* Start to monitor for variable cycling if this is the initial stationarity */
    if(monitor->Ncycle <= 1) {
      monitor->Ccycle = colnr;
      monitor->Rcycle = rownr;
    }

    /* Check if we should change pivoting strategy */
    else if(isCreeping ||                                                 /* We have OF creep */
            (monitor->Ncycle > monitor->limitstall[monitor->isdual]) ||   /* KE empirical value */
            (monitor->Ccycle == rownr) && (monitor->Rcycle == colnr)) {   /* Obvious cycling */

      monitor->active = TRUE;

      /* Try to force out equality slacks to combat degeneracy */
      if((lp->fixedvars > 0) && (*forceoutEQ != TRUE)) {
        *forceoutEQ = TRUE;
        goto Proceed;
      }

      /* Our options are now to select an alternative rule or to do bound perturbation;
         check if these options are available to us or if we must signal failure and break out. */
      approved &= monitor->pivdynamic && (monitor->ruleswitches < monitor->limitruleswitches);
      if(!approved && !is_anti_degen(lp, ANTIDEGEN_STALLING)) {
        lp->spx_status = DEGENERATE;
        report(lp, msglevel, "%s: Stalling at iter %10.0f; no alternative strategy left.\n",
                             monitor->spxfunc, (double) get_total_iter(lp));
        acceptance = FALSE;
        return( acceptance );
      }

      /* See if we can do the appropriate alternative rule. */
      switch (monitor->oldpivrule) {
        case PRICER_FIRSTINDEX:    altrule = PRICER_DEVEX;
                                   break;
        case PRICER_DANTZIG:       altrule = PRICER_DEVEX;
                                   break;
        case PRICER_DEVEX:         altrule = PRICER_STEEPESTEDGE;
                                   break;
        case PRICER_STEEPESTEDGE:  altrule = PRICER_DEVEX;
                                   break;
        default:                   altrule = PRICER_FIRSTINDEX;
      }
      if(approved &&
         (monitor->pivrule != altrule) && (monitor->pivrule == monitor->oldpivrule)) {

        /* Switch rule to combat degeneracy. */
        monitor->ruleswitches++;
        lp->piv_strategy = altrule;
        monitor->Ccycle = 0;
        monitor->Rcycle = 0;
        monitor->Ncycle = 0;
        monitor->Mcycle = 0;
        report(lp, msglevel, "%s: Stalling at iter %10.0f; changed to '%s' rule.\n",
                             monitor->spxfunc, (double) get_total_iter(lp),
                             get_str_piv_rule(get_piv_rule(lp)));
        if((altrule == PRICER_DEVEX) || (altrule == PRICER_STEEPESTEDGE))
          restartPricer(lp, AUTOMATIC);
      }

      /* If not, code for bound relaxation/perturbation */
      else {
        report(lp, msglevel, "%s: Stalling at iter %10.0f; proceed to bound relaxation.\n",
                             monitor->spxfunc, (double) get_total_iter(lp));
        acceptance = FALSE;
        lp->spx_status = DEGENERATE;
        return( acceptance );
      }
    }
  }

  /* Otherwise change back to original selection strategy as soon as possible */
  else {
    if(monitor->pivrule != monitor->oldpivrule) {
      lp->piv_strategy = monitor->oldpivstrategy;
      altrule = monitor->oldpivrule;
      if((altrule == PRICER_DEVEX) || (altrule == PRICER_STEEPESTEDGE))
        restartPricer(lp, AUTOMATIC);
      report(lp, msglevel, "...returned to original pivot selection rule at iter %.0f.\n",
                           (double) get_total_iter(lp));
    }
    stallMonitor_update(lp, monitor->thisobj);
    monitor->Ccycle = 0;
    monitor->Rcycle = 0;
    monitor->Ncycle = 0;
    monitor->Mcycle = 0;
  }

  /* Update objective progress tracker */
Proceed:
  monitor->Icount++;
  if(deltaobj >= monitor->epsvalue)
    monitor->prevobj = monitor->thisobj;
  monitor->previnfeas = monitor->thisinfeas;

  return( acceptance );
}

STATIC void stallMonitor_finish(lprec *lp)
{
  OBJmonrec *monitor = lp->monitor;
  if(monitor == NULL)
    return;
  if(lp->piv_strategy != monitor->oldpivstrategy)
    lp->piv_strategy = monitor->oldpivstrategy;
  FREE(monitor);
  lp->monitor = NULL;
}


STATIC MYBOOL add_artificial(lprec *lp, int forrownr, REAL *nzarray, int *idxarray)
/* This routine is called for each constraint at the start of
   primloop and the primal problem is infeasible. Its
   purpose is to add artificial variables and associated
   objective function values to populate primal phase 1. */
{
  MYBOOL add;

  /* Make sure we don't add unnecessary artificials, i.e. avoid
     cases where the slack variable is enough */
  add = !isBasisVarFeasible(lp, lp->epspivot, forrownr);

  if(add) {
    int    *rownr = NULL, i, bvar, ii;
    REAL   *avalue = NULL, rhscoef, acoef;
    MATrec *mat = lp->matA;

    /* Check the simple case where a slack is basic */
    for(i = 1; i <= lp->rows; i++) {
      if(lp->var_basic[i] == forrownr)
        break;
    }
    acoef = 1;

    /* If not, look for any basic user variable that has a
       non-zero coefficient in the current constraint row */
    if(i > lp->rows) {
      for(i = 1; i <= lp->rows; i++) {
        ii = lp->var_basic[i] - lp->rows;
        if((ii <= 0) || (ii > (lp->columns-lp->P1extraDim)))
          continue;
        ii = mat_findelm(mat, forrownr, ii);
        if(ii >= 0) {
          acoef = COL_MAT_VALUE(ii);
          break;
        }
      }
    }

    /* If no candidate was found above, gamble on using the densest column available */
#if 0
    if(i > lp->rows) {
      int len = 0;
      bvar = 0;
      for(i = 1; i <= lp->rows; i++) {
        ii = lp->var_basic[i] - lp->rows;
        if((ii <= 0) || (ii > (lp->columns-lp->P1extraDim)))
          continue;
        if(mat_collength(mat, ii) > len) {
          len = mat_collength(mat, ii);
          bvar = i;
        }
      }
      i = bvar;
      acoef = 1;
    }
#endif

    bvar = i;

    add = (MYBOOL) (bvar <= lp->rows);
    if(add) {
      rhscoef = lp->rhs[forrownr];

     /* Create temporary sparse array storage */
      if(nzarray == NULL)
        allocREAL(lp, &avalue, 2, FALSE);
      else
        avalue = nzarray;
      if(idxarray == NULL)
        allocINT(lp, &rownr, 2, FALSE);
      else
        rownr = idxarray;

     /* Set the objective coefficient */
      rownr[0]  =  0;
      avalue[0] = my_chsign(is_chsign(lp, 0), 1);

     /* Set the constraint row coefficient */
      rownr[1]  = forrownr;
      avalue[1] = my_chsign(is_chsign(lp, forrownr), my_sign(rhscoef/acoef));

     /* Add the column of artificial variable data to the user data matrix */
      add_columnex(lp, 2, avalue, rownr);

     /* Free the temporary sparse array storage */
      if(idxarray == NULL)
        FREE(rownr);
      if(nzarray == NULL)
        FREE(avalue);

     /* Now set the artificial variable to be basic */
      set_basisvar(lp, bvar, lp->sum);
      lp->P1extraDim++;
    }
    else {
      report(lp, CRITICAL, "add_artificial: Could not find replacement basis variable for row %d\n",
                           forrownr);
      lp->basis_valid = FALSE;
    }

  }

  return(add);

}

STATIC int get_artificialRow(lprec *lp, int colnr)
{
  MATrec *mat = lp->matA;

#ifdef Paranoia
  if((colnr <= lp->columns-abs(lp->P1extraDim)) || (colnr > lp->columns))
    report(lp, SEVERE, "get_artificialRow: Invalid column index %d\n", colnr);
  if(mat->col_end[colnr] - mat->col_end[colnr-1] != 1)
    report(lp, SEVERE, "get_artificialRow: Invalid column non-zero count\n");
#endif

  /* Return the row index of the singleton */
  colnr = mat->col_end[colnr-1];
  colnr = COL_MAT_ROWNR(colnr);
  return( colnr );
}

STATIC int findAnti_artificial(lprec *lp, int colnr)
/* Primal simplex: Find a basic artificial variable to swap
   against the non-basic slack variable, if possible */
{
  int    i, k, rownr = 0, P1extraDim = abs(lp->P1extraDim);

  if((P1extraDim == 0) || (colnr > lp->rows) || !lp->is_basic[colnr])
    return( rownr );

  for(i = 1; i <= lp->rows; i++) {
    k = lp->var_basic[i];
    if((k > lp->sum-P1extraDim) && (lp->rhs[i] == 0)) {
      rownr = get_artificialRow(lp, k-lp->rows);

      /* Should we find the artificial's slack direct "antibody"? */
      if(rownr == colnr)
        break;
      rownr = 0;
    }
  }
  return( rownr );
}

STATIC int findBasicArtificial(lprec *lp, int before)
{
  int i = 0, P1extraDim = abs(lp->P1extraDim);

  if(P1extraDim > 0) {
    if(before > lp->rows || before <= 1)
      i = lp->rows;
    else
      i = before;

    while((i > 0) && (lp->var_basic[i] <= lp->sum-P1extraDim))
      i--;
  }

  return(i);
}

STATIC void eliminate_artificials(lprec *lp, REAL *prow)
{
  int   i, j, colnr, rownr, P1extraDim = abs(lp->P1extraDim);

  for(i = 1; (i <= lp->rows) && (P1extraDim > 0); i++) {
    j = lp->var_basic[i];
    if(j <= lp->sum-P1extraDim)
      continue;
    j -= lp->rows;
    rownr = get_artificialRow(lp, j);
    colnr = find_rowReplacement(lp, rownr, prow, NULL);
#if 0
    performiteration(lp, rownr, colnr, 0.0, TRUE, FALSE, prow, NULL,
                                                          NULL, NULL, NULL);
#else
    set_basisvar(lp, rownr, colnr);
#endif
    del_column(lp, j);
    P1extraDim--;
  }
  lp->P1extraDim = 0;
}

STATIC void clear_artificials(lprec *lp)
{
  int i, j, n, P1extraDim;

  /* Substitute any basic artificial variable for its slack counterpart */
  n = 0;
  P1extraDim = abs(lp->P1extraDim);
  for(i = 1; (i <= lp->rows) && (n < P1extraDim); i++) {
    j = lp->var_basic[i];
    if(j <= lp->sum-P1extraDim)
      continue;
    j = get_artificialRow(lp, j-lp->rows);
    set_basisvar(lp, i, j);
    n++;
  }
#ifdef Paranoia
  if(n != lp->P1extraDim)
    report(lp, SEVERE, "clear_artificials: Unable to clear all basic artificial variables\n");
#endif

  /* Delete any remaining non-basic artificial variables */
  while(P1extraDim > 0) {
    i = lp->sum-lp->rows;
    del_column(lp, i);
    P1extraDim--;
  }
  lp->P1extraDim = 0;
  if(n > 0) {
    set_action(&lp->spx_action, ACTION_REINVERT);
    lp->basis_valid = TRUE;
  }
}


STATIC int primloop(lprec *lp, MYBOOL primalfeasible, REAL primaloffset)
{
  MYBOOL primal = TRUE, bfpfinal = FALSE, changedphase = FALSE, forceoutEQ = AUTOMATIC,
         primalphase1, pricerCanChange, minit, stallaccept, pendingunbounded;
  int    i, j, k, colnr = 0, rownr = 0, lastnr = 0,
         candidatecount = 0, minitcount = 0, ok = TRUE;
  LREAL  theta = 0.0;
  REAL   epsvalue, xviolated, cviolated,
         *prow = NULL, *pcol = NULL,
         *drow = lp->drow;
  int    *workINT = NULL,
         *nzdrow = lp->nzdrow;

  if(lp->spx_trace)
    report(lp, DETAILED, "Entered primal simplex algorithm with feasibility %s\n",
                         my_boolstr(primalfeasible));

 /* Add sufficent number of artificial variables to make the problem feasible
    through the first phase; delete when primal feasibility has been achieved */
  lp->P1extraDim = 0;
  if(!primalfeasible) {
    lp->simplex_mode = SIMPLEX_Phase1_PRIMAL;
#ifdef Paranoia
    if(!verify_basis(lp))
      report(lp, SEVERE, "primloop: No valid basis for artificial variables\n");
#endif
#if 0
    /* First check if we can get away with a single artificial variable */
    if(lp->equalities == 0) {
      i = (int) feasibilityOffset(lp, !primal);
      add_artificial(lp, i, prow, (int *) pcol);
    }
    else
#endif
    /* Otherwise add as many artificial variables as is necessary
       to force primal feasibility. */
      for(i = 1; i <= lp->rows; i++) {
        add_artificial(lp, i, NULL, NULL);
      }

    /* Make sure we update the working objective */
    if(lp->P1extraDim > 0) {
#if 1 /* v5.1 code: Not really necessary since we do not price the artificial
        variables (stored at the end of the column list, they are initially
        basic and are never allowed to enter the basis, once they exit) */
      ok = allocREAL(lp, &(lp->drow), lp->sum+1, AUTOMATIC) &&
           allocINT(lp, &(lp->nzdrow), lp->sum+1, AUTOMATIC);
      if(!ok)
        goto Finish;
      lp->nzdrow[0] = 0;
      drow = lp->drow;
      nzdrow = lp->nzdrow;
#endif
      mat_validate(lp->matA);
      set_OF_p1extra(lp, 0.0);
    }
    if(lp->spx_trace)
      report(lp, DETAILED, "P1extraDim count = %d\n", lp->P1extraDim);

    simplexPricer(lp, (MYBOOL)!primal);
    invert(lp, INITSOL_USEZERO, TRUE);
  }
  else {
    lp->simplex_mode = SIMPLEX_Phase2_PRIMAL;
    restartPricer(lp, (MYBOOL)!primal);
  }

  /* Create work arrays and optionally the multiple pricing structure */
  ok = allocREAL(lp, &(lp->bsolveVal), lp->rows + 1, FALSE) &&
       allocREAL(lp, &prow, lp->sum + 1, TRUE) &&
       allocREAL(lp, &pcol, lp->rows + 1, TRUE);
  if(is_piv_mode(lp, PRICE_MULTIPLE) && (lp->multiblockdiv > 1)) {
    lp->multivars = multi_create(lp, FALSE);
    ok &= (lp->multivars != NULL) &&
          multi_resize(lp->multivars, lp->sum / lp->multiblockdiv, 2, FALSE, TRUE);
  }
  if(!ok)
    goto Finish;

  /* Initialize regular primal simplex algorithm variables */
  lp->spx_status = RUNNING;
  minit = ITERATE_MAJORMAJOR;
  epsvalue = lp->epspivot;
  pendingunbounded = FALSE;

  ok = stallMonitor_create(lp, FALSE, "primloop");
  if(!ok)
    goto Finish;

  lp->rejectpivot[0] = 0;

 /* Iterate while we are successful; exit when the model is infeasible/unbounded,
    or we must terminate due to numeric instability or user-determined reasons */
  while((lp->spx_status == RUNNING) && !userabort(lp, -1)) {

    primalphase1 = (MYBOOL) (lp->P1extraDim > 0);
    clear_action(&lp->spx_action, ACTION_REINVERT | ACTION_ITERATE);

    /* Check if we have stalling (from numerics or degenerate cycling) */
    pricerCanChange = !primalphase1;
    stallaccept = stallMonitor_check(lp, rownr, colnr, lastnr, minit, pricerCanChange, &forceoutEQ);
    if(!stallaccept)
      break;

   /* Find best column to enter the basis */
RetryCol:
#if 0
    if(verify_solution(lp, FALSE, "spx_loop") > 0)
      i = 1; /* This is just a debug trap */
#endif
    if(!changedphase) {
      i = 0;
      do {
        i++;
        colnr = colprim(lp, drow, nzdrow, (MYBOOL) (minit == ITERATE_MINORRETRY), i, &candidatecount, TRUE, &xviolated);
      } while ((colnr == 0) && (i < partial_countBlocks(lp, (MYBOOL) !primal)) &&
                                partial_blockStep(lp, (MYBOOL) !primal));

      /* Handle direct outcomes */
      if(colnr == 0)
        lp->spx_status = OPTIMAL;
      if(lp->rejectpivot[0] > 0)
        minit = ITERATE_MAJORMAJOR;

      /* See if accuracy check during compute_reducedcosts flagged refactorization */
      if(is_action(lp->spx_action, ACTION_REINVERT))
        bfpfinal = TRUE;

    }

    /* Make sure that we do not erroneously conclude that an unbounded model is optimal */
#ifdef primal_UseRejectionList
    if((colnr == 0) && (lp->rejectpivot[0] > 0)) {
      lp->spx_status = UNBOUNDED;
      if((lp->spx_trace && (lp->bb_totalnodes == 0)) ||
         (lp->bb_trace && (lp->bb_totalnodes > 0)))
        report(lp, DETAILED, "The model is primal unbounded.\n");
      colnr = lp->rejectpivot[1];
      rownr = 0;
      lp->rejectpivot[0] = 0;
      ok = FALSE;
      break;
    }
#endif

    /* Check if we found an entering variable (indicating that we are still dual infeasible) */
    if(colnr > 0) {
      changedphase = FALSE;
      fsolve(lp, colnr, pcol, NULL, lp->epsmachine, 1.0, TRUE);  /* Solve entering column for Pi */

      /* Do special anti-degeneracy column selection, if specified */
      if(is_anti_degen(lp, ANTIDEGEN_COLUMNCHECK) && !check_degeneracy(lp, pcol, NULL)) {
        if(lp->rejectpivot[0] < DEF_MAXPIVOTRETRY/3) {
          i = ++lp->rejectpivot[0];
          lp->rejectpivot[i] = colnr;
          report(lp, DETAILED, "Entering column %d found to be non-improving due to degeneracy.\n",
                     colnr);
          minit = ITERATE_MINORRETRY;
          goto RetryCol;
        }
        else {
          lp->rejectpivot[0] = 0;
          report(lp, DETAILED, "Gave up trying to find a strictly improving entering column.\n");
        }
      }

      /* Find the leaving variable that gives the most stringent bound on the entering variable */
      theta = drow[colnr];
      rownr = rowprim(lp, colnr, &theta, pcol, workINT, forceoutEQ, &cviolated);

#ifdef AcceptMarginalAccuracy
      /* Check for marginal accuracy */
      if((rownr > 0) && (xviolated+cviolated < lp->epspivot)) {
        if(lp->bb_trace || (lp->bb_totalnodes == 0))
          report(lp, DETAILED, "primloop: Assuming convergence with reduced accuracy %g.\n",
                               MAX(xviolated, cviolated));
        rownr = 0;
        colnr = 0;
        goto Optimality;
      }
      else
#endif

      /* See if we can do a straight artificial<->slack replacement (when "colnr" is a slack) */
      if((lp->P1extraDim != 0) && (rownr == 0) && (colnr <= lp->rows))
        rownr = findAnti_artificial(lp, colnr);

      if(rownr > 0) {
        pendingunbounded = FALSE;
        lp->rejectpivot[0] = 0;
        set_action(&lp->spx_action, ACTION_ITERATE);
        if(!lp->obj_in_basis)  /* We must manually copy the reduced cost for RHS update */
          pcol[0] = my_chsign(!lp->is_lower[colnr], drow[colnr]);
        lp->bfp_prepareupdate(lp, rownr, colnr, pcol);
      }

      /* We may be unbounded... */
      else {
        /* First make sure that we are not suffering from precision loss */
#ifdef primal_UseRejectionList
        if(lp->rejectpivot[0] < DEF_MAXPIVOTRETRY) {
          lp->spx_status = RUNNING;
          lp->rejectpivot[0]++;
          lp->rejectpivot[lp->rejectpivot[0]] = colnr;
          report(lp, DETAILED, "...trying to recover via another pivot column.\n");
          minit = ITERATE_MINORRETRY;
          goto RetryCol;
        }
        else
#endif
        /* Check that we are not having numerical problems */
        if(!refactRecent(lp) && !pendingunbounded) {
          bfpfinal = TRUE;
          pendingunbounded = TRUE;
          set_action(&lp->spx_action, ACTION_REINVERT);
        }

        /* Conclude that the model is unbounded */
        else {
          lp->spx_status = UNBOUNDED;
          report(lp, DETAILED, "The model is primal unbounded.\n");
          break;
        }
      }
    }

    /* We handle optimality and phase 1 infeasibility ... */
    else {

Optimality:
      /* Handle possible transition from phase 1 to phase 2 */
      if(!primalfeasible || isP1extra(lp)) {

        if(feasiblePhase1(lp, epsvalue)) {
          lp->spx_status = RUNNING;
          if(lp->bb_totalnodes == 0) {
            report(lp, NORMAL, "Found feasibility by primal simplex after  %10.0f iter.\n",
                                (double) get_total_iter(lp));
            if((lp->usermessage != NULL) && (lp->msgmask & MSG_LPFEASIBLE))
              lp->usermessage(lp, lp->msghandle, MSG_LPFEASIBLE);
          }
          changedphase = FALSE;
          primalfeasible = TRUE;
          lp->simplex_mode = SIMPLEX_Phase2_PRIMAL;
          set_OF_p1extra(lp, 0.0);

         /* We can do two things now;
            1) delete the rows belonging to those variables, since they are redundant, OR
            2) drive out the existing artificial variables via pivoting. */
          if(lp->P1extraDim > 0) {

#ifdef Phase1EliminateRedundant
           /* If it is not a MIP model we can try to delete redundant rows */
            if((lp->bb_totalnodes == 0) && (MIP_count(lp) == 0)) {
              while(lp->P1extraDim > 0) {
                i = lp->rows;
                while((i > 0) && (lp->var_basic[i] <= lp->sum-lp->P1extraDim))
                  i--;
#ifdef Paranoia
                if(i <= 0) {
                  report(lp, SEVERE, "primloop: Could not find redundant artificial.\n");
                  break;
                }
#endif
                /* Obtain column and row indeces */
                j = lp->var_basic[i]-lp->rows;
                k = get_artificialRow(lp, j);

                /* Delete row before column due to basis "compensation logic" */
                if(lp->is_basic[k]) {
                  lp->is_basic[lp->rows+j] = FALSE;
                  del_constraint(lp, k);
                }
                else
                  set_basisvar(lp, i, k);
                del_column(lp, j);
                lp->P1extraDim--;
              }
              lp->basis_valid = TRUE;
            }
           /* Otherwise we drive out the artificials by elimination pivoting */
            else
              eliminate_artificials(lp, prow);

#else
            /* Indicate phase 2 with artificial variables by negating P1extraDim */
            lp->P1extraDim = my_flipsign(lp->P1extraDim);
#endif
          }

          /* We must refactorize since the OF changes from phase 1 to phase 2 */
          set_action(&lp->spx_action, ACTION_REINVERT);
          bfpfinal = TRUE;
        }

        /* We are infeasible in phase 1 */
        else {
          lp->spx_status = INFEASIBLE;
          minit = ITERATE_MAJORMAJOR;
          if(lp->spx_trace)
            report(lp, NORMAL, "Model infeasible by primal simplex at iter   %10.0f.\n",
                               (double) get_total_iter(lp));
        }
      }

      /* Handle phase 1 optimality */
      else {
        /* (Do nothing special) */
      }

      /* Check if we are still primal feasible; the default assumes that this check
         is not necessary after the relaxed problem has been solved satisfactorily. */
      if((lp->bb_level <= 1) || (lp->improve & IMPROVE_BBSIMPLEX)) {
        set_action(&lp->piv_strategy, PRICE_FORCEFULL);
          i = rowdual(lp, lp->rhs, FALSE, FALSE, NULL);
        clear_action(&lp->piv_strategy, PRICE_FORCEFULL);
        if(i > 0) {
          lp->spx_status = LOSTFEAS;
          if(lp->total_iter == 0)
            report(lp, DETAILED, "primloop: Lost primal feasibility at iter  %10.0f: will try to recover.\n",
                                 (double) get_total_iter(lp));
        }
      }
    }

    /* Pivot row/col and update the inverse */
    if(is_action(lp->spx_action, ACTION_ITERATE)) {
      lastnr = lp->var_basic[rownr];

      if(refactRecent(lp) == AUTOMATIC)
        minitcount = 0;
      else if(minitcount > MAX_MINITUPDATES) {
        recompute_solution(lp, INITSOL_USEZERO);
        minitcount = 0;
      }
      minit = performiteration(lp, rownr, colnr, theta, primal,
                                                 (MYBOOL) (/*(candidatecount > 1) && */
                                                           (stallaccept != AUTOMATIC)),
                                                 NULL, NULL,
                                                 pcol, NULL, NULL);
      if(minit != ITERATE_MAJORMAJOR)
        minitcount++;

      if((lp->spx_status == USERABORT) || (lp->spx_status == TIMEOUT))
        break;
      else if(minit == ITERATE_MINORMAJOR)
        continue;
#ifdef UsePrimalReducedCostUpdate
      /* Do a fast update of the reduced costs in preparation for the next iteration */
      if(minit == ITERATE_MAJORMAJOR)
        update_reducedcosts(lp, primal, lastnr, colnr, pcol, drow);
#endif

      /* Detect if an auxiliary variable has left the basis and delete it; if
         the non-basic variable only changed bound (a "minor iteration"), the
         basic artificial variable did not leave and there is nothing to do */
      if((minit == ITERATE_MAJORMAJOR) && (lastnr > lp->sum - abs(lp->P1extraDim))) {
#ifdef Paranoia
        if(lp->is_basic[lastnr] || !lp->is_basic[colnr])
          report(lp, SEVERE, "primloop: Invalid basis indicator for variable %d at iter %10.0f.\n",
                              lastnr, (double) get_total_iter(lp));
#endif
        del_column(lp, lastnr-lp->rows);
        if(lp->P1extraDim > 0)
          lp->P1extraDim--;
        else
          lp->P1extraDim++;
        if(lp->P1extraDim == 0) {
          colnr = 0;
          changedphase = TRUE;
          stallMonitor_reset(lp);
        }
      }
    }

    if(lp->spx_status == SWITCH_TO_DUAL)
      ;
    else if(!changedphase && lp->bfp_mustrefactorize(lp)) {
#ifdef ResetMinitOnReinvert
      minit = ITERATE_MAJORMAJOR;
#endif
      if(!invert(lp, INITSOL_USEZERO, bfpfinal))
        lp->spx_status = SINGULAR_BASIS;
      bfpfinal = FALSE;
    }
  }

  /* Remove any remaining artificial variables (feasible or infeasible model) */
  lp->P1extraDim = abs(lp->P1extraDim);
/*  if((lp->P1extraDim > 0) && (lp->spx_status != DEGENERATE)) { */
  if(lp->P1extraDim > 0) {
    clear_artificials(lp);
    if(lp->spx_status != OPTIMAL)
      restore_basis(lp);
    i = invert(lp, INITSOL_USEZERO, TRUE);
  }
#ifdef Paranoia
  if(!verify_basis(lp))
    report(lp, SEVERE, "primloop: Invalid basis detected due to internal error\n");
#endif

  /* Switch to dual phase 1 simplex for MIP models during
     B&B phases, since this is typically far more efficient */
#ifdef ForceDualSimplexInBB
  if((lp->bb_totalnodes == 0) && (MIP_count(lp) > 0) &&
     ((lp->simplex_strategy & SIMPLEX_Phase1_DUAL) == 0)) {
    lp->simplex_strategy &= ~SIMPLEX_Phase1_PRIMAL;
    lp->simplex_strategy += SIMPLEX_Phase1_DUAL;
  }
#endif

Finish:
  stallMonitor_finish(lp);
  multi_free(&(lp->multivars));
  FREE(prow);
  FREE(pcol);
  FREE(lp->bsolveVal);

  return(ok);
} /* primloop */

STATIC int dualloop(lprec *lp, MYBOOL dualfeasible, int dualinfeasibles[], REAL dualoffset)
{
  MYBOOL primal = FALSE, inP1extra, dualphase1 = FALSE, changedphase = TRUE,
         pricerCanChange, minit, stallaccept, longsteps,
         forceoutEQ = FALSE, bfpfinal = FALSE;
  int    i, colnr = 0, rownr = 0, lastnr = 0,
         candidatecount = 0, minitcount = 0,
#ifdef FixInaccurateDualMinit
         minitcolnr = 0,
#endif
         ok = TRUE;
  int    *boundswaps = NULL;
  LREAL  theta = 0.0;
  REAL   epsvalue, xviolated, cviolated,
         *prow = NULL, *pcol = NULL,
         *drow = lp->drow;
  int    *nzprow = NULL, *workINT = NULL,
         *nzdrow = lp->nzdrow;

  if(lp->spx_trace)
    report(lp, DETAILED, "Entered dual simplex algorithm with feasibility %s.\n",
                         my_boolstr(dualfeasible));

  /* Allocate work arrays */
  ok = allocREAL(lp, &prow,   lp->sum + 1,  TRUE) &&
       allocINT (lp, &nzprow, lp->sum + 1,  FALSE) &&
       allocREAL(lp, &pcol,   lp->rows + 1, TRUE);
  if(!ok)
    goto Finish;

  /* Set non-zero P1extraVal value to force dual feasibility when the dual
     simplex is used as a phase 1 algorithm for the primal simplex.
     The value will be reset when primal feasibility has been achieved, or
     a dual non-feasibility has been encountered (no candidate for a first
     leaving variable) */
  inP1extra = (MYBOOL) (dualoffset != 0);
  if(inP1extra) {
    set_OF_p1extra(lp, dualoffset);
    simplexPricer(lp, (MYBOOL)!primal);
    invert(lp, INITSOL_USEZERO, TRUE);
  }
  else
    restartPricer(lp, (MYBOOL)!primal);

  /* Prepare dual long-step structures */
#if 0
  longsteps = TRUE;
#elif 0
  longsteps = (MYBOOL) ((MIP_count(lp) > 0) && (lp->bb_level > 1));
#elif 0
  longsteps = (MYBOOL) ((MIP_count(lp) > 0) && (lp->solutioncount >= 1));
#else
  longsteps = FALSE;
#endif
#ifdef UseLongStepDualPhase1
  longsteps = !dualfeasible && (MYBOOL) (dualinfeasibles != NULL);
#endif

  if(longsteps) {
    lp->longsteps = multi_create(lp, TRUE);
    ok = (lp->longsteps != NULL) &&
         multi_resize(lp->longsteps, MIN(lp->boundedvars+2, 11), 1, TRUE, TRUE);
    if(!ok)
      goto Finish;
#ifdef UseLongStepPruning
    lp->longsteps->objcheck = TRUE;
#endif
    boundswaps = multi_indexSet(lp->longsteps, FALSE);
  }

  /* Do regular dual simplex variable initializations */
  lp->spx_status = RUNNING;
  minit = ITERATE_MAJORMAJOR;
  epsvalue = lp->epspivot;

  ok = stallMonitor_create(lp, TRUE, "dualloop");
  if(!ok)
    goto Finish;

  lp->rejectpivot[0] = 0;
  if(dualfeasible)
    lp->simplex_mode = SIMPLEX_Phase2_DUAL;
  else
    lp->simplex_mode = SIMPLEX_Phase1_DUAL;

  /* Check if we have equality slacks in the basis and we should try to
     drive them out in order to reduce chance of degeneracy in Phase 1.
     forceoutEQ = FALSE :    Only eliminate assured "good" violated
                             equality constraint slacks
                  AUTOMATIC: Seek more elimination of equality constraint
                             slacks (but not as aggressive as the rule
                             used in lp_solve v4.0 and earlier)
                  TRUE:      Force remaining equality slacks out of the
                             basis */
  if(dualphase1 || inP1extra ||
     ((lp->fixedvars > 0) && is_anti_degen(lp, ANTIDEGEN_FIXEDVARS))) {
    forceoutEQ = AUTOMATIC;
  }
#if 1
  if(is_anti_degen(lp, ANTIDEGEN_DYNAMIC) && (bin_count(lp, TRUE)*2 > lp->columns)) {
    switch (forceoutEQ) {
      case FALSE:     forceoutEQ = AUTOMATIC;
                      break;
 /*     case AUTOMATIC: forceoutEQ = TRUE;
                      break;
      default:        forceoutEQ = TRUE; */
    }
  }
#endif

  while((lp->spx_status == RUNNING) && !userabort(lp, -1)) {

    /* Check if we have stalling (from numerics or degenerate cycling) */
    pricerCanChange = !dualphase1 && !inP1extra;
    stallaccept = stallMonitor_check(lp, rownr, colnr, lastnr, minit, pricerCanChange, &forceoutEQ);
    if(!stallaccept)
      break;

    /* Store current LP index for reference at next iteration */
    changedphase = FALSE;

    /* Compute (pure) dual phase1 offsets / reduced costs if appropriate */
    dualphase1 &= (MYBOOL) (lp->simplex_mode == SIMPLEX_Phase1_DUAL);
    if(longsteps && dualphase1 && !inP1extra) {
      obtain_column(lp, dualinfeasibles[1], pcol, NULL, NULL);
      i = 2;
      for(i = 2; i <= dualinfeasibles[0]; i++)
        mat_multadd(lp->matA, pcol, dualinfeasibles[i], 1.0);
      /* Solve (note that solved pcol will be used instead of lp->rhs) */
      ftran(lp, pcol, NULL, lp->epsmachine);
    }

    /* Do minor iterations (non-basic variable bound flips) for as
       long as possible since this is a cheap way of iterating */
#if (defined dual_Phase1PriceEqualities) || (defined dual_UseRejectionList)
RetryRow:
#endif
    if(minit != ITERATE_MINORRETRY) {
      i = 0;
      do {
        i++;
        rownr = rowdual(lp, my_if(dualphase1, pcol, NULL), forceoutEQ, TRUE, &xviolated);
      } while ((rownr == 0) && (i < partial_countBlocks(lp, (MYBOOL) !primal)) &&
                                partial_blockStep(lp, (MYBOOL) !primal));
    }

    /* Make sure that we do not erroneously conclude that an infeasible model is optimal */
#ifdef dual_UseRejectionList
    if((rownr == 0) && (lp->rejectpivot[0] > 0)) {
      lp->spx_status = INFEASIBLE;
      if((lp->spx_trace && (lp->bb_totalnodes == 0)) ||
         (lp->bb_trace && (lp->bb_totalnodes > 0)))
        report(lp, DETAILED, "The model is primal infeasible.\n");
      rownr = lp->rejectpivot[1];
      colnr = 0;
      lp->rejectpivot[0] = 0;
      ok = FALSE;
      break;
    }
#endif

    /* If we found a leaving variable, find a matching entering one */
    clear_action(&lp->spx_action, ACTION_ITERATE);
    if(rownr > 0) {
      colnr = coldual(lp, rownr, prow, nzprow, drow, nzdrow,
                                 (MYBOOL) (dualphase1 && !inP1extra),
                                 (MYBOOL) (minit == ITERATE_MINORRETRY), &candidatecount, &cviolated);
      if(colnr < 0) {
        minit = ITERATE_MAJORMAJOR;
        continue;
      }
#ifdef AcceptMarginalAccuracy
      else if(xviolated+cviolated < lp->epspivot) {
        if(lp->bb_trace || (lp->bb_totalnodes == 0))
          report(lp, DETAILED, "dualloop: Assuming convergence with reduced accuracy %g.\n",
                               MAX(xviolated, cviolated));
        rownr = 0;
        colnr = 0;
      }
#endif
      /* Check if the long-dual found reason to prune the B&B tree */
      if(lp->spx_status == FATHOMED)
        break;
    }
    else
      colnr = 0;

    /* Process primal-infeasible row */
    if(rownr > 0) {

      if(colnr > 0) {
#ifdef Paranoia
        if((rownr > lp->rows) || (colnr > lp->sum)) {
          report(lp, SEVERE, "dualloop: Invalid row %d(%d) and column %d(%d) pair selected at iteration %.0f\n",
                             rownr, lp->rows, colnr-lp->columns, lp->columns, (double) get_total_iter(lp));
          lp->spx_status = UNKNOWNERROR;
          break;
        }
#endif
        fsolve(lp, colnr, pcol, workINT, lp->epsmachine, 1.0, TRUE);

#ifdef FixInaccurateDualMinit
       /* Prevent bound flip-flops during minor iterations; used to detect
          infeasibility after triggering of minor iteration accuracy management */
        if(colnr != minitcolnr)
          minitcolnr = 0;
#endif

       /* Getting division by zero here; catch it and try to recover */
        if(pcol[rownr] == 0) {
          if(lp->spx_trace)
            report(lp, DETAILED, "dualloop: Attempt to divide by zero (pcol[%d])\n", rownr);
          if(!refactRecent(lp)) {
            report(lp, DETAILED, "...trying to recover by refactorizing basis.\n");
            set_action(&lp->spx_action, ACTION_REINVERT);
            bfpfinal = FALSE;
          }
          else {
            if(lp->bb_totalnodes == 0)
              report(lp, DETAILED, "...cannot recover by refactorizing basis.\n");
            lp->spx_status = NUMFAILURE;
            ok = FALSE;
          }
        }
        else {
          set_action(&lp->spx_action, ACTION_ITERATE);
          lp->rejectpivot[0] = 0;
          if(!lp->obj_in_basis)  /* We must manually copy the reduced cost for RHS update */
            pcol[0] = my_chsign(!lp->is_lower[colnr], drow[colnr]);
          theta = lp->bfp_prepareupdate(lp, rownr, colnr, pcol);

         /* Verify numeric accuracy of the basis factorization and change to
            the "theoretically" correct version of the theta */
          if((lp->improve & IMPROVE_THETAGAP) && !refactRecent(lp) &&
             (my_reldiff(fabs(theta), fabs(prow[colnr])) >
              lp->epspivot*10.0*log(2.0+50.0*lp->rows))) {  /* This is my kludge - KE */
            set_action(&lp->spx_action, ACTION_REINVERT);
            bfpfinal = TRUE;
#ifdef IncreasePivotOnReducedAccuracy
            lp->epspivot = MIN(1.0e-4, lp->epspivot*2.0);
#endif
            report(lp, DETAILED, "dualloop: Refactorizing at iter %.0f due to loss of accuracy.\n",
                                 (double) get_total_iter(lp));
          }
          theta = prow[colnr];
          compute_theta(lp, rownr, &theta, !lp->is_lower[colnr], 0, primal);
        }
      }

#ifdef FixInaccurateDualMinit
      /* Force reinvertion and try another row if we did not find a bound-violated leaving column */
      else if(!refactRecent(lp) && (minit != ITERATE_MAJORMAJOR) && (colnr != minitcolnr)) {
        minitcolnr = colnr;
        i = invert(lp, INITSOL_USEZERO, TRUE);
        if((lp->spx_status == USERABORT) || (lp->spx_status == TIMEOUT))
          break;
        else if(!i) {
          lp->spx_status = SINGULAR_BASIS;
          break;
        }
        minit = ITERATE_MAJORMAJOR;
        continue;
      }
#endif

      /* We may be infeasible, have lost dual feasibility, or simply have no valid entering
         variable for the selected row.  The strategy is to refactorize if we suspect numerical
         problems and loss of dual feasibility; this is done if it has been a while since
         refactorization.  If not, first try to select a different row/leaving variable to
         see if a valid entering variable can be found.  Otherwise, determine this model
         as infeasible. */
      else {

        /* As a first option, try to recover from any numerical trouble by refactorizing */
        if(!refactRecent(lp)) {
          set_action(&lp->spx_action, ACTION_REINVERT);
          bfpfinal = TRUE;
        }

#ifdef dual_UseRejectionList
        /* Check for pivot size issues */
        else if(lp->rejectpivot[0] < DEF_MAXPIVOTRETRY) {
          lp->spx_status = RUNNING;
          lp->rejectpivot[0]++;
          lp->rejectpivot[lp->rejectpivot[0]] = rownr;
          if(lp->bb_totalnodes == 0)
            report(lp, DETAILED, "...trying to find another pivot row!\n");
          goto RetryRow;
        }
#endif
        /* Check if we may have lost dual feasibility if we also did phase 1 here */
        else if(dualphase1 && (dualoffset != 0)) {
          lp->spx_status = LOSTFEAS;
          if((lp->spx_trace && (lp->bb_totalnodes == 0)) ||
             (lp->bb_trace && (lp->bb_totalnodes > 0)))
            report(lp, DETAILED, "dualloop: Model lost dual feasibility.\n");
          ok = FALSE;
          break;
        }

        /* Otherwise just determine that we are infeasible */
        else {
          if(lp->spx_status == RUNNING) {
#if 1
            if(xviolated < lp->epspivot) {
              if(lp->bb_trace || (lp->bb_totalnodes == 0))
                report(lp, NORMAL, "The model is primal optimal, but marginally infeasible.\n");
              lp->spx_status = OPTIMAL;
              break;
            }
#endif
            lp->spx_status = INFEASIBLE;
            if((lp->spx_trace && (lp->bb_totalnodes == 0)) ||
               (lp->bb_trace && (lp->bb_totalnodes > 0)))
            report(lp, DETAILED, "The model is primal infeasible.\n");
          }
          ok = FALSE;
          break;
        }
      }
    }

    /* Make sure that we enter the primal simplex with a high quality solution */
    else if(inP1extra && !refactRecent(lp) && is_action(lp->improve, IMPROVE_INVERSE)) {
       set_action(&lp->spx_action, ACTION_REINVERT);
       bfpfinal = TRUE;
    }

    /* High quality solution with no leaving candidates available ... */
    else {

      bfpfinal = TRUE;

#ifdef dual_RemoveBasicFixedVars
      /* See if we should try to eliminate basic fixed variables;
        can be time-consuming for some models */
      if(inP1extra && (colnr == 0) && (lp->fixedvars > 0) && is_anti_degen(lp, ANTIDEGEN_FIXEDVARS)) {
        report(lp, DETAILED, "dualloop: Trying to pivot out %d fixed basic variables at iter %.0f\n",
                             lp->fixedvars, (double) get_total_iter(lp));
        rownr = 0;
        while(lp->fixedvars > 0) {
          rownr = findBasicFixedvar(lp, rownr, TRUE);
          if(rownr == 0) {
            colnr = 0;
            break;
          }
          colnr = find_rowReplacement(lp, rownr, prow, nzprow);
          if(colnr > 0) {
            theta = 0;
            performiteration(lp, rownr, colnr, theta, TRUE, FALSE, prow, NULL,
                                                            NULL, NULL, NULL);
            lp->fixedvars--;
          }
        }
      }
#endif

      /* Check if we are INFEASIBLE for the case that the dual is used
         as phase 1 before the primal simplex phase 2 */
      if(inP1extra && (colnr < 0) && !isPrimalFeasible(lp, lp->epsprimal, NULL, NULL)) {
        if(lp->bb_totalnodes == 0) {
          if(dualfeasible)
            report(lp, DETAILED, "The model is primal infeasible and dual feasible.\n");
          else
            report(lp, DETAILED, "The model is primal infeasible and dual unbounded.\n");
        }
        set_OF_p1extra(lp, 0);
        inP1extra = FALSE;
        set_action(&lp->spx_action, ACTION_REINVERT);
        lp->spx_status = INFEASIBLE;
        lp->simplex_mode = SIMPLEX_UNDEFINED;
        ok = FALSE;
      }

      /* Check if we are FEASIBLE (and possibly also optimal) for the case that the
         dual is used as phase 1 before the primal simplex phase 2 */
      else if(inP1extra) {

        /* Set default action; force an update of the rhs vector, adjusted for
           the new P1extraVal=0 (set here so that usermessage() behaves properly) */
        if(lp->bb_totalnodes == 0) {
          report(lp, NORMAL, "Found feasibility by dual simplex after    %10.0f iter.\n",
                             (double) get_total_iter(lp));
          if((lp->usermessage != NULL) && (lp->msgmask & MSG_LPFEASIBLE))
            lp->usermessage(lp, lp->msghandle, MSG_LPFEASIBLE);
        }
        set_OF_p1extra(lp, 0);
        inP1extra = FALSE;
        set_action(&lp->spx_action, ACTION_REINVERT);

#if 1
        /* Optionally try another dual loop, if so selected by the user */
        if((lp->simplex_strategy & SIMPLEX_DUAL_PRIMAL) && (lp->fixedvars == 0))
          lp->spx_status = SWITCH_TO_PRIMAL;
#endif
        changedphase = TRUE;

      }

      /* We are primal feasible and also optimal if we were in phase 2 */
      else  {

        lp->simplex_mode = SIMPLEX_Phase2_DUAL;

        /* Check if we still have equality slacks stuck in the basis; drive them out? */
        if((lp->fixedvars > 0) && (lp->bb_totalnodes == 0)) {
#ifdef dual_Phase1PriceEqualities
          if(forceoutEQ != TRUE) {
            forceoutEQ = TRUE;
            goto RetryRow;
          }
#endif
#ifdef Paranoia
          report(lp, NORMAL,
#else
          report(lp, DETAILED,
#endif
                    "Found dual solution with %d fixed slack variables left basic.\n",
                    lp->fixedvars);
        }
        /* Check if we are still dual feasible; the default assumes that this check
          is not necessary after the relaxed problem has been solved satisfactorily. */
        colnr = 0;
        if((dualoffset != 0) || (lp->bb_level <= 1) || (lp->improve & IMPROVE_BBSIMPLEX)) {
          set_action(&lp->piv_strategy, PRICE_FORCEFULL);
            colnr = colprim(lp, drow, nzdrow, FALSE, 1, &candidatecount, FALSE, NULL);
          clear_action(&lp->piv_strategy, PRICE_FORCEFULL);
          if((dualoffset == 0) && (colnr > 0)) {
            lp->spx_status = LOSTFEAS;
            if(lp->total_iter == 0)
              report(lp, DETAILED, "Recovering lost dual feasibility at iter %10.0f.\n",
                                   (double) get_total_iter(lp));
            break;
          }
        }

        if(colnr == 0)
          lp->spx_status = OPTIMAL;
        else {
          lp->spx_status = SWITCH_TO_PRIMAL;
          if(lp->total_iter == 0)
            report(lp, DETAILED, "Use primal simplex for finalization at iter  %10.0f.\n",
                                 (double) get_total_iter(lp));
        }
        if((lp->total_iter == 0) && (lp->spx_status == OPTIMAL))
          report(lp, DETAILED, "Optimal solution with dual simplex at iter   %10.0f.\n",
                               (double) get_total_iter(lp));
      }

      /* Determine if we are ready to break out of the loop */
      if(!changedphase)
        break;
    }

    /* Check if we are allowed to iterate on the chosen column and row */
    if(is_action(lp->spx_action, ACTION_ITERATE)) {

      lastnr = lp->var_basic[rownr];
      if(refactRecent(lp) == AUTOMATIC)
        minitcount = 0;
      else if(minitcount > MAX_MINITUPDATES) {
        recompute_solution(lp, INITSOL_USEZERO);
        minitcount = 0;
      }
      minit = performiteration(lp, rownr, colnr, theta, primal,
                                                 (MYBOOL) (/*(candidatecount > 1) && */
                                                           (stallaccept != AUTOMATIC)),
                                                 prow, nzprow,
                                                 pcol, NULL, boundswaps);

      /* Check if we should abandon iterations on finding that there is no
        hope that this branch can improve on the incumbent B&B solution */
      if(!lp->is_strongbranch && (lp->solutioncount >= 1) && !lp->spx_perturbed && !inP1extra &&
          bb_better(lp, OF_WORKING, OF_TEST_WE)) {
        lp->spx_status = FATHOMED;
        ok = FALSE;
        break;
      }

      if(minit != ITERATE_MAJORMAJOR)
        minitcount++;

      /* Update reduced costs for (pure) dual long-step phase 1 */
      if(longsteps && dualphase1 && !inP1extra) {
        dualfeasible = isDualFeasible(lp, lp->epsprimal, NULL, dualinfeasibles, NULL);
        if(dualfeasible) {
          dualphase1 = FALSE;
          changedphase = TRUE;
          lp->simplex_mode = SIMPLEX_Phase2_DUAL;
        }
      }
#ifdef UseDualReducedCostUpdate
      /* Do a fast update of reduced costs in preparation for the next iteration */
      else if(minit == ITERATE_MAJORMAJOR)
        update_reducedcosts(lp, primal, lastnr, colnr, prow, drow);
#endif
      if((minit == ITERATE_MAJORMAJOR) && (lastnr <= lp->rows) && is_fixedvar(lp, lastnr))
        lp->fixedvars--;
    }

    /* Refactorize if required to */
    if(lp->bfp_mustrefactorize(lp)) {
      if(invert(lp, INITSOL_USEZERO, bfpfinal)) {

#if 0
        /* Verify dual feasibility in case we are attempting the extra dual loop */
        if(changedphase && (dualoffset != 0) && !inP1extra && (lp->spx_status != SWITCH_TO_PRIMAL)) {
#if 1
          if(!isDualFeasible(lp, lp->epsdual, &colnr, NULL, NULL)) {
#else
          set_action(&lp->piv_strategy, PRICE_FORCEFULL);
            colnr = colprim(lp, drow, nzdrow, FALSE, 1, &candidatecount, FALSE, NULL);
          clear_action(&lp->piv_strategy, PRICE_FORCEFULL);
          if(colnr > 0) {
#endif
            lp->spx_status = SWITCH_TO_PRIMAL;
            colnr = 0;
          }
        }
#endif

        bfpfinal = FALSE;
#ifdef ResetMinitOnReinvert
        minit = ITERATE_MAJORMAJOR;
#endif
      }
      else
        lp->spx_status = SINGULAR_BASIS;
    }
  }

Finish:
  stallMonitor_finish(lp);
  multi_free(&(lp->longsteps));
  FREE(prow);
  FREE(nzprow);
  FREE(pcol);

  return(ok);
}

STATIC int spx_run(lprec *lp, MYBOOL validInvB)
{
  int    i, j, singular_count, lost_feas_count, *infeasibles = NULL, *boundflip_count;
  MYBOOL primalfeasible, dualfeasible, lost_feas_state, isbb;
  REAL   primaloffset = 0, dualoffset = 0;

  lp->current_iter  = 0;
  lp->current_bswap = 0;
  lp->spx_status    = RUNNING;
  lp->bb_status = lp->spx_status;
  lp->P1extraDim = 0;
  set_OF_p1extra(lp, 0);
  singular_count  = 0;
  lost_feas_count = 0;
  lost_feas_state = FALSE;
  lp->simplex_mode = SIMPLEX_DYNAMIC;

  /* Compute the number of fixed basic and bounded variables (used in long duals) */
  lp->fixedvars = 0;
  lp->boundedvars = 0;
  for(i = 1; i <= lp->rows; i++) {
    j = lp->var_basic[i];
    if((j <= lp->rows) && is_fixedvar(lp, j))
      lp->fixedvars++;
    if((lp->upbo[i] < lp->infinite) && (lp->upbo[i] > lp->epsprimal))
      lp->boundedvars++;
  }
  for(; i <= lp->sum; i++){
    if((lp->upbo[i] < lp->infinite) && (lp->upbo[i] > lp->epsprimal))
      lp->boundedvars++;
  }
#ifdef UseLongStepDualPhase1
  allocINT(lp, &infeasibles, lp->columns + 1, FALSE);
  infeasibles[0] = 0;
#endif

  /* Reinvert for initialization, if necessary */
  isbb = (MYBOOL) ((MIP_count(lp) > 0) && (lp->bb_level > 1));
  if(is_action(lp->spx_action, ACTION_REINVERT)) {
    if(isbb && (lp->bb_bounds->nodessolved == 0))
/*    if(isbb && (lp->bb_basis->pivots == 0)) */
      recompute_solution(lp, INITSOL_SHIFTZERO);
    else {
      i = my_if(is_action(lp->spx_action, ACTION_REBASE), INITSOL_SHIFTZERO, INITSOL_USEZERO);
      invert(lp, (MYBOOL) i, TRUE);
    }
  }
  else if(is_action(lp->spx_action, ACTION_REBASE))
    recompute_solution(lp, INITSOL_SHIFTZERO);

  /* Optionally try to do bound flips to obtain dual feasibility */
  if(is_action(lp->improve, IMPROVE_DUALFEAS) || (lp->rows == 0))
    boundflip_count = &i;
  else
    boundflip_count = NULL;

  /* Loop for as long as is needed */
  while(lp->spx_status == RUNNING) {

    /* Check for dual and primal feasibility */
    dualfeasible   = isbb ||
                     isDualFeasible(lp, lp->epsprimal, boundflip_count, infeasibles, &dualoffset);

    /* Recompute if the dual feasibility check included bound flips */
    if(is_action(lp->spx_action, ACTION_RECOMPUTE))
      recompute_solution(lp, INITSOL_USEZERO);
    primalfeasible = isPrimalFeasible(lp, lp->epsprimal, NULL, &primaloffset);

    if(userabort(lp, -1))
      break;

    if(lp->spx_trace) {
      if(primalfeasible)
        report(lp, NORMAL, "Start at primal feasible basis\n");
      else if(dualfeasible)
        report(lp, NORMAL, "Start at dual feasible basis\n");
      else if(lost_feas_count > 0)
        report(lp, NORMAL, "Continuing at infeasible basis\n");
      else
        report(lp, NORMAL, "Start at infeasible basis\n");
    }

   /* Now do the simplex magic */
    if(((lp->simplex_strategy & SIMPLEX_Phase1_DUAL) == 0) ||
       ((MIP_count(lp) > 0) && (lp->total_iter == 0) &&
        is_presolve(lp, PRESOLVE_REDUCEMIP))) {
      if(!lost_feas_state && primalfeasible && ((lp->simplex_strategy & SIMPLEX_Phase2_DUAL) > 0))
        lp->spx_status = SWITCH_TO_DUAL;
      else
        primloop(lp, primalfeasible, 0.0);
      if(lp->spx_status == SWITCH_TO_DUAL)
        dualloop(lp, TRUE, NULL, 0.0);
    }
    else {
      if(!lost_feas_state && primalfeasible && ((lp->simplex_strategy & SIMPLEX_Phase2_PRIMAL) > 0))
        lp->spx_status = SWITCH_TO_PRIMAL;
      else
        dualloop(lp, dualfeasible, infeasibles, dualoffset);
      if(lp->spx_status == SWITCH_TO_PRIMAL)
        primloop(lp, TRUE, 0.0);
    }

    /* Check for simplex outcomes that always involve breaking out of the loop;
       this includes optimality, unboundedness, pure infeasibility (i.e. not
       loss of feasibility), numerical failure and perturbation-based degeneracy
       handling */
    i = lp->spx_status;
    primalfeasible = (MYBOOL) (i == OPTIMAL);
    if(primalfeasible || (i == UNBOUNDED))
      break;
    else if(((i == INFEASIBLE) && is_anti_degen(lp, ANTIDEGEN_INFEASIBLE)) ||
             ((i == LOSTFEAS)   && is_anti_degen(lp, ANTIDEGEN_LOSTFEAS)) ||
             ((i == NUMFAILURE) && is_anti_degen(lp, ANTIDEGEN_NUMFAILURE)) ||
             ((i == DEGENERATE) && is_anti_degen(lp, ANTIDEGEN_STALLING))) {
      /* Check if we should not loop here, but do perturbations */
      if((lp->bb_level <= 1)   || is_anti_degen(lp, ANTIDEGEN_DURINGBB))
        break;

      /* Assume that accuracy during B&B is high and that infeasibility is "real" */
#ifdef AssumeHighAccuracyInBB
      if((lp->bb_level > 1) && (i == INFEASIBLE))
        break;
#endif
    }

    /* Check for outcomes that may involve trying another simplex loop */
    if(lp->spx_status == SINGULAR_BASIS) {
      lost_feas_state = FALSE;
      singular_count++;
      if(singular_count >= DEF_MAXSINGULARITIES) {
        report(lp, IMPORTANT, "spx_run: Failure due to too many singular bases.\n");
        lp->spx_status = NUMFAILURE;
        break;
      }
      if(lp->spx_trace || (lp->verbose > DETAILED))
        report(lp, NORMAL, "spx_run: Singular basis; attempting to recover.\n");
      lp->spx_status = RUNNING;
      /* Singular pivots are simply skipped by the inversion, leaving a row's
         slack variable in the basis instead of the singular user variable. */
    }
    else {
      lost_feas_state = (MYBOOL) (lp->spx_status == LOSTFEAS);
#if 0
      /* Optionally handle loss of numerical accuracy as loss of feasibility,
         but only attempt a single loop to try to recover from this. */
      lost_feas_state |= (MYBOOL) ((lp->spx_status == NUMFAILURE) && (lost_feas_count < 1));
#endif
      if(lost_feas_state) {
        lost_feas_count++;
        if(lost_feas_count < DEF_MAXSINGULARITIES) {
          report(lp, DETAILED, "spx_run: Recover lost feasibility at iter  %10.0f.\n",
                                (double) get_total_iter(lp));
          lp->spx_status = RUNNING;
        }
        else {
          report(lp, IMPORTANT, "spx_run: Lost feasibility %d times - iter %10.0f and %9.0f nodes.\n",
                                lost_feas_count, (double) get_total_iter(lp), (double) lp->bb_totalnodes);
          lp->spx_status = NUMFAILURE;
        }
      }
    }
  }

  /* Update iteration tallies before returning */
  lp->total_iter   += lp->current_iter;
  lp->current_iter  = 0;
  lp->total_bswap  += lp->current_bswap;
  lp->current_bswap = 0;
  FREE(infeasibles);

  return(lp->spx_status);
} /* spx_run */

lprec *make_lag(lprec *lpserver)
{
  int    i;
  lprec  *hlp;
  MYBOOL ret;
  REAL   *duals;

  /* Create a Lagrangean solver instance */
  hlp = make_lp(0, lpserver->columns);

  if(hlp != NULL) {

    /* First create and core variable data */
    set_sense(hlp, is_maxim(lpserver));
    hlp->lag_bound = lpserver->bb_limitOF;
    for(i = 1; i <= lpserver->columns; i++) {
      set_mat(hlp, 0, i, get_mat(lpserver, 0, i));
      if(is_binary(lpserver, i))
        set_binary(hlp, i, TRUE);
      else {
        set_int(hlp, i, is_int(lpserver, i));
        set_bounds(hlp, i, get_lowbo(lpserver, i), get_upbo(lpserver, i));
      }
    }
    /* Then fill data for the Lagrangean constraints */
    hlp->matL = lpserver->matA;
    inc_lag_space(hlp, lpserver->rows, TRUE);
    ret = get_ptr_sensitivity_rhs(hlp, &duals, NULL, NULL);
    for(i = 1; i <= lpserver->rows; i++) {
      hlp->lag_con_type[i] = get_constr_type(lpserver, i);
      hlp->lag_rhs[i] = lpserver->orig_rhs[i];
      hlp->lambda[i] = (ret) ? duals[i - 1] : 0.0;
    }
  }

  return(hlp);
}

STATIC int heuristics(lprec *lp, int mode)
/* Initialize / bound a MIP problem */
{
  lprec *hlp;
  int   status = PROCFAIL;

  if(lp->bb_level > 1)
    return( status );

  status = RUNNING;
  lp->bb_limitOF = my_chsign(is_maxim(lp), -lp->infinite);
  if(FALSE && (lp->int_vars > 0)) {

    /* 1. Copy the problem into a new relaxed instance, extracting Lagrangean constraints */
    hlp = make_lag(lp);

    /* 2. Run the Lagrangean relaxation */
    status = solve(hlp);

    /* 3. Copy the key results (bound) into the original problem */
    lp->bb_heuristicOF = hlp->best_solution[0];

    /* 4. Delete the helper heuristic */
    hlp->matL = NULL;
    delete_lp(hlp);
  }

  lp->timeheuristic = timeNow();
  return( status );
}

STATIC int lag_solve(lprec *lp, REAL start_bound, int num_iter)
{
  int    i, j, citer, nochange, oldpresolve;
  MYBOOL LagFeas, AnyFeas, Converged, same_basis;
  REAL   *OrigObj, *ModObj, *SubGrad, *BestFeasSol;
  REAL   Zub, Zlb, Znow, Zprev, Zbest, rhsmod, hold;
  REAL   Phi, StepSize = 0.0, SqrsumSubGrad;

  /* Make sure we have something to work with */
  if(lp->spx_status != OPTIMAL) {
    lp->lag_status = NOTRUN;
    return( lp->lag_status );
  }

  /* Allocate iteration arrays */
  if(!allocREAL(lp, &OrigObj, lp->columns + 1, FALSE) ||
     !allocREAL(lp, &ModObj,  lp->columns + 1, TRUE) ||
     !allocREAL(lp, &SubGrad, get_Lrows(lp) + 1, TRUE) ||
     !allocREAL(lp, &BestFeasSol, lp->sum + 1, TRUE)) {
    lp->lag_status = NOMEMORY;
     return( lp->lag_status );
  }
  lp->lag_status = RUNNING;

  /* Prepare for Lagrangean iterations using results from relaxed problem */
  oldpresolve = lp->do_presolve;
  lp->do_presolve = PRESOLVE_NONE;
  push_basis(lp, NULL, NULL, NULL);

  /* Initialize variables (assume minimization problem in overall structure) */
  Zlb      = lp->best_solution[0];
  Zub      = start_bound;
  Zbest    = Zub;
  Znow     = Zlb;
  Zprev    = lp->infinite;
  rhsmod   = 0;

  Phi      = DEF_LAGCONTRACT; /* In the range 0-2.0 to guarantee convergence */
/*  Phi      = 0.15; */
  LagFeas  = FALSE;
  Converged= FALSE;
  AnyFeas  = FALSE;
  citer    = 0;
  nochange = 0;

  /* Initialize reference and solution vectors; don't bother about the
     original OF offset since we are maintaining an offset locally. */

/* #define DirectOverrideOF */

  get_row(lp, 0, OrigObj);
#ifdef DirectOverrideOF
  set_OF_override(lp, ModObj);
#endif
  OrigObj[0] = get_rh(lp, 0);
  for(i = 1 ; i <= get_Lrows(lp); i++)
    lp->lambda[i] = 0;

  /* Iterate to convergence, failure or user-specified termination */
  while((lp->lag_status == RUNNING) && (citer < num_iter)) {

    citer++;

    /* Compute constraint feasibility gaps and associated sum of squares,
       and determine feasibility over the Lagrangean constraints;
       SubGrad is the subgradient, which here is identical to the slack. */
    LagFeas = TRUE;
    Converged = TRUE;
    SqrsumSubGrad = 0;
    for(i = 1; i <= get_Lrows(lp); i++) {
      hold = lp->lag_rhs[i];
      for(j = 1; j <= lp->columns; j++)
        hold -= mat_getitem(lp->matL, i, j) * lp->best_solution[lp->rows + j];
      if(LagFeas) {
        if(lp->lag_con_type[i] == EQ) {
          if(fabs(hold) > lp->epsprimal)
            LagFeas = FALSE;
        }
        else if(hold < -lp->epsprimal)
          LagFeas = FALSE;
      }
      /* Test for convergence and update */
      if(Converged && (fabs(my_reldiff(hold , SubGrad[i])) > lp->lag_accept))
        Converged = FALSE;
      SubGrad[i] = hold;
      SqrsumSubGrad += hold * hold;
    }
    SqrsumSubGrad = sqrt(SqrsumSubGrad);
#if 1
    Converged &= LagFeas;
#endif
    if(Converged)
      break;

    /* Modify step parameters and initialize ahead of next iteration */
    Znow = lp->best_solution[0] - rhsmod;
    if(Znow > Zub) {
      /* Handle exceptional case where we overshoot */
      Phi *= DEF_LAGCONTRACT;
      StepSize *= (Zub-Zlb) / (Znow-Zlb);
    }
    else
#define LagBasisContract
#ifdef LagBasisContract
/*      StepSize = Phi * (Zub - Znow) / SqrsumSubGrad; */
      StepSize = Phi * (2-DEF_LAGCONTRACT) * (Zub - Znow) / SqrsumSubGrad;
#else
      StepSize = Phi * (Zub - Znow) / SqrsumSubGrad;
#endif

    /* Compute the new dual price vector (Lagrangean multipliers, lambda) */
    for(i = 1; i <= get_Lrows(lp); i++) {
      lp->lambda[i] += StepSize * SubGrad[i];
      if((lp->lag_con_type[i] != EQ) && (lp->lambda[i] > 0)) {
        /* Handle case where we overshoot and need to correct (see above) */
        if(Znow < Zub)
          lp->lambda[i] = 0;
      }
    }
/*    normalizeVector(lp->lambda, get_Lrows(lp)); */

    /* Save the current vector if it is better */
    if(LagFeas && (Znow < Zbest)) {

      /* Recompute the objective function value in terms of the original values */
      MEMCOPY(BestFeasSol, lp->best_solution, lp->sum+1);
      hold = OrigObj[0];
      for(i = 1; i <= lp->columns; i++)
        hold += lp->best_solution[lp->rows + i] * OrigObj[i];
      BestFeasSol[0] = hold;
      if(lp->lag_trace)
        report(lp, NORMAL, "lag_solve: Improved feasible solution at iteration %d of %g\n",
                           citer, hold);

      /* Reset variables */
      Zbest = Znow;
      AnyFeas  = TRUE;
      nochange = 0;
    }
    else if(Znow == Zprev) {
      nochange++;
      if(nochange > LAG_SINGULARLIMIT) {
        Phi *= 0.5;
        nochange = 0;
      }
    }
    Zprev = Znow;

    /* Recompute the objective function values for the next iteration */
    for(j = 1; j <= lp->columns; j++) {
      hold = 0;
      for(i = 1; i <= get_Lrows(lp); i++)
        hold += lp->lambda[i] * mat_getitem(lp->matL, i, j);
      ModObj[j] = OrigObj[j] - my_chsign(is_maxim(lp), hold);
#ifndef DirectOverrideOF
      set_mat(lp, 0, j, ModObj[j]);
#endif
    }

    /* Recompute the fixed part of the new objective function */
    rhsmod = my_chsign(is_maxim(lp), get_rh(lp, 0));
    for(i = 1; i <= get_Lrows(lp); i++)
      rhsmod += lp->lambda[i] * lp->lag_rhs[i];

    /* Print trace/debugging information, if specified */
    if(lp->lag_trace) {
      report(lp, IMPORTANT, "Zub: %10g Zlb: %10g Stepsize: %10g Phi: %10g Feas %d\n",
                 (double) Zub, (double) Zlb, (double) StepSize, (double) Phi, LagFeas);
      for(i = 1; i <= get_Lrows(lp); i++)
        report(lp, IMPORTANT, "%3d SubGrad %10g lambda %10g\n",
                   i, (double) SubGrad[i], (double) lp->lambda[i]);
      if(lp->sum < 20)
        print_lp(lp);
    }

    /* Solve the Lagrangean relaxation, handle failures and compute
       the Lagrangean objective value, if successful */
    i = spx_solve(lp);
    if(lp->spx_status == UNBOUNDED) {
      if(lp->lag_trace) {
        report(lp, NORMAL, "lag_solve: Unbounded solution encountered with this OF:\n");
        for(i = 1; i <= lp->columns; i++)
          report(lp, NORMAL, RESULTVALUEMASK " ", (double) ModObj[i]);
      }
      goto Leave;
    }
    else if((lp->spx_status == NUMFAILURE)   || (lp->spx_status == PROCFAIL) ||
            (lp->spx_status == USERABORT) || (lp->spx_status == TIMEOUT) ||
            (lp->spx_status == INFEASIBLE)) {
      lp->lag_status = lp->spx_status;
    }

    /* Compare optimal bases and contract if we have basis stationarity */
#ifdef LagBasisContract
    same_basis = compare_basis(lp);
    if(LagFeas &&
       !same_basis) {
      pop_basis(lp, FALSE);
      push_basis(lp, NULL, NULL, NULL);
      Phi *= DEF_LAGCONTRACT;
    }
    if(lp->lag_trace) {
      report(lp, DETAILED, "lag_solve: Simplex status code %d, same basis %s\n",
                 lp->spx_status, my_boolstr(same_basis));
      print_solution(lp, 1);
    }
#endif
  }

  /* Transfer solution values */
  if(AnyFeas) {
    lp->lag_bound = my_chsign(is_maxim(lp), Zbest);
    for(i = 0; i <= lp->sum; i++)
      lp->solution[i] = BestFeasSol[i];
    transfer_solution(lp, TRUE);
    if(!is_maxim(lp))
      for(i = 1; i <= get_Lrows(lp); i++)
        lp->lambda[i] = my_flipsign(lp->lambda[i]);
  }

  /* Do standard postprocessing */
Leave:

  /* Set status variables and report */
  if(citer >= num_iter) {
    if(AnyFeas)
      lp->lag_status = FEASFOUND;
    else
      lp->lag_status = NOFEASFOUND;
  }
  else
    lp->lag_status = lp->spx_status;
  if(lp->lag_status == OPTIMAL) {
    report(lp, NORMAL, "\nLagrangean convergence achieved in %d iterations\n",  citer);
    i = check_solution(lp, lp->columns,
                       lp->best_solution, lp->orig_upbo, lp->orig_lowbo, lp->epssolution);
  }
  else {
    report(lp, NORMAL, "\nUnsatisfactory convergence achieved over %d Lagrangean iterations.\n",
                       citer);
    if(AnyFeas)
      report(lp, NORMAL, "The best feasible Lagrangean objective function value was %g\n",
                         lp->best_solution[0]);
  }

  /* Restore the original objective function */
#ifdef DirectOverrideOF
  set_OF_override(lp, NULL);
#else
  for(i = 1; i <= lp->columns; i++)
    set_mat(lp, 0, i, OrigObj[i]);
#endif

  /* ... and then free memory */
  FREE(BestFeasSol);
  FREE(SubGrad);
  FREE(OrigObj);
  FREE(ModObj);
  pop_basis(lp, FALSE);

  lp->do_presolve = oldpresolve;

  return( lp->lag_status );
}

STATIC int spx_solve(lprec *lp)
{
  int       status;
  MYBOOL    iprocessed;

  lp->total_iter       = 0;
  lp->total_bswap      = 0;
  lp->perturb_count    = 0;
  lp->bb_maxlevel      = 1;
  lp->bb_totalnodes    = 0;
  lp->bb_improvements  = 0;
  lp->bb_strongbranches= 0;
  lp->is_strongbranch  = FALSE;
  lp->bb_level         = 0;
  lp->bb_solutionlevel = 0;
  lp->best_solution[0] = my_chsign(is_maxim(lp), lp->infinite);
  if(lp->invB != NULL)
    lp->bfp_restart(lp);

  lp->spx_status = presolve(lp);
  if(lp->spx_status == PRESOLVED) {
    status = lp->spx_status;
    goto Reconstruct;
  }
  else if(lp->spx_status != RUNNING)
    goto Leave;

  iprocessed = !lp->wasPreprocessed;
  if(!preprocess(lp) || userabort(lp, -1))
    goto Leave;

  if(mat_validate(lp->matA)) {

    /* Do standard initializations */
    lp->solutioncount = 0;
    lp->real_solution = lp->infinite;
    set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT);
    lp->bb_break = FALSE;

    /* Do the call to the real underlying solver (note that
       run_BB is replaceable with any compatible MIP solver) */
    status = run_BB(lp);

    /* Restore modified problem */
    if(iprocessed)
      postprocess(lp);

    /* Restore data related to presolve (mainly a placeholder as of v5.1) */
Reconstruct:
    if(!postsolve(lp, status))
      report(lp, SEVERE, "spx_solve: Failure during postsolve.\n");

    goto Leave;
  }

  /* If we get here, mat_validate(lp) failed. */
  if(lp->bb_trace || lp->spx_trace)
    report(lp, CRITICAL, "spx_solve: The current LP seems to be invalid\n");
  lp->spx_status = NUMFAILURE;

Leave:
  lp->timeend = timeNow();

  if((lp->lag_status != RUNNING) && (lp->invB != NULL)) {
    int       itemp;
    REAL      test;

    itemp = lp->bfp_nonzeros(lp, TRUE);
    test = 100;
    if(lp->total_iter > 0)
      test *= (REAL) lp->total_bswap/lp->total_iter;
    report(lp, NORMAL, "\n ");
    report(lp, NORMAL, "MEMO: lp_solve version %d.%d.%d.%d for %d bit OS, with %d bit REAL variables.\n",
                        MAJORVERSION, MINORVERSION, RELEASE, BUILD, 8*sizeof(void *), 8*sizeof(REAL));
    report(lp, NORMAL, "      In the total iteration count %.0f, %.0f (%.1f%%) were bound flips.\n",
                        (double) lp->total_iter, (double) lp->total_bswap, test);
    report(lp, NORMAL, "      There were %d refactorizations, %d triggered by time and %d by density.\n",
                        lp->bfp_refactcount(lp, BFP_STAT_REFACT_TOTAL),
                        lp->bfp_refactcount(lp, BFP_STAT_REFACT_TIMED),
                        lp->bfp_refactcount(lp, BFP_STAT_REFACT_DENSE));
    report(lp, NORMAL, "       ... on average %.1f major pivots per refactorization.\n",
                        get_refactfrequency(lp, TRUE));
    report(lp, NORMAL, "      The largest [%s] fact(B) had %d NZ entries, %.1fx largest basis.\n",
                        lp->bfp_name(), itemp, lp->bfp_efficiency(lp));
    if(lp->perturb_count > 0)
      report(lp, NORMAL, "      The bounds were relaxed via perturbations %d times.\n",
                          lp->perturb_count);
    if(MIP_count(lp) > 0) {
      if(lp->bb_solutionlevel > 0)
        report(lp, NORMAL, "      The maximum B&B level was %d, %.1fx MIP order, %d at the optimal solution.\n",
                        lp->bb_maxlevel, (double) lp->bb_maxlevel / (MIP_count(lp)+lp->int_vars), lp->bb_solutionlevel);
      else
        report(lp, NORMAL, "      The maximum B&B level was %d, %.1fx MIP order, with %.0f nodes explored.\n",
                        lp->bb_maxlevel, (double) lp->bb_maxlevel / (MIP_count(lp)+lp->int_vars), (double) get_total_nodes(lp));
      if(GUB_count(lp) > 0)
        report(lp, NORMAL, "      %d general upper-bounded (GUB) structures were employed during B&B.\n",
                         GUB_count(lp));
    }
    report(lp, NORMAL, "      The constraint matrix inf-norm is %g, with a dynamic range of %g.\n",
                        lp->matA->infnorm, lp->matA->dynrange);
    report(lp, NORMAL, "      Time to load data was %.3f seconds, presolve used %.3f seconds,\n",
                        lp->timestart-lp->timecreate, lp->timepresolved-lp->timestart);
    report(lp, NORMAL, "       ... %.3f seconds in simplex solver, in total %.3f seconds.\n",
                        lp->timeend-lp->timepresolved, lp->timeend-lp->timecreate);
  }
  return( lp->spx_status );

} /* spx_solve */

int lin_solve(lprec *lp)
{
  int status = NOTRUN;

  /* Don't do anything in case of an empty model */
  lp->lag_status = NOTRUN;
  /* if(get_nonzeros(lp) == 0) { */
  if(lp->columns == 0) {
    default_basis(lp);
    lp->spx_status = NOTRUN;
    return( /* OPTIMAL */ lp->spx_status);
  }

  /* Otherwise reset selected arrays before solving */
  unset_OF_p1extra(lp);
  free_duals(lp);
  FREE(lp->drow);
  FREE(lp->nzdrow);
  if(lp->bb_cuttype != NULL)
    freecuts_BB(lp);

  /* Reset/initialize timers */
  lp->timestart        = timeNow();
  lp->timeheuristic    = 0;
  lp->timepresolved    = 0;
  lp->timeend          = 0;

  /* Do heuristics ahead of solving the model */
  if(heuristics(lp, AUTOMATIC) != RUNNING)
    return( INFEASIBLE );

  /* Solve the full, prepared model */
  status = spx_solve(lp);
  if((get_Lrows(lp) > 0) && (lp->lag_status == NOTRUN)) {
    if(status == OPTIMAL)
      status = lag_solve(lp, lp->bb_heuristicOF, DEF_LAGMAXITERATIONS);
    else
      report(lp, IMPORTANT, "\nCannot do Lagrangean optimization since root model was not solved.\n");
  }

  /* Reset heuristic in preparation for next run (if any) */
  lp->bb_heuristicOF = my_chsign(is_maxim(lp), lp->infinite);

  /* Check that correct status code is returned */
/*
   peno 26.12.07
   status was not set to SUBOPTIMAL, only lp->spx_status
   Bug occured by a change in 5.5.0.10 when  && (lp->bb_totalnodes > 0) was added
   added status =
   See UnitTest3
*/
  if((lp->spx_status == OPTIMAL) && lp->bb_break && (lp->bb_totalnodes > 0))
    status = lp->spx_status = SUBOPTIMAL;

  return( status );
}
