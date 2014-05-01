
/* Routines located in lp_BFP1.cpp; common for all factorization engines              */
/* Cfr. lp_BFP.h for definitions                                                      */
/* ---------------------------------------------------------------------------------- */
/* Changes:                                                                           */
/* 29 May 2004       Corrected calculation of bfp_efficiency(), which required        */
/*                   modifying the max_Bsize to include slack variables. KE.          */
/* 16 June 2004      Make the symbolic minimum degree ordering routine available      */
/*                   to BFPs as a routine internal to the library. KE                 */
/* 1  July 2004      Change due to change in MDO naming.                              */
/* ---------------------------------------------------------------------------------- */


/* MUST MODIFY */
MYBOOL BFP_CALLMODEL bfp_compatible(lprec *lp, int bfpversion, int lpversion, int sizeofvar)
{
  MYBOOL status = FALSE;

  if((lp != NULL) && (bfpversion == BFPVERSION) && (sizeof(REAL) == sizeofvar)) {
#if 0
    if(lpversion == MAJORVERSION)  /* Forces BFP renewal at lp_solve major version changes */
#endif
      status = TRUE;
  }
  return( status );
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_status(lprec *lp)
{
  return(lp->invB->status);
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_indexbase(lprec *lp)
{
  return( MATINDEXBASE );
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_rowoffset(lprec *lp)
{
  if(lp->obj_in_basis)
    return( 1 );
  else
    return( 0 );
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_pivotmax(lprec *lp)
{
  if(lp->max_pivots > 0)
    return( lp->max_pivots );
  else
    return( DEF_MAXPIVOT );
}

/* DON'T MODIFY */
REAL * BFP_CALLMODEL bfp_pivotvector(lprec *lp)
{
  return( lp->invB->pcol );
}

/* DON'T MODIFY */
REAL BFP_CALLMODEL bfp_efficiency(lprec *lp)
{
  REAL hold;

  hold = lp->bfp_nonzeros(lp, AUTOMATIC);
  if(hold == 0)
    hold = 1 + lp->rows;
  hold = lp->bfp_nonzeros(lp, TRUE)/hold;

  return(hold);
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_pivotcount(lprec *lp)
{
  return(lp->invB->num_pivots);
}


/* DON'T MODIFY */
int BFP_CALLMODEL bfp_refactcount(lprec *lp, int kind)
{
  if(kind == BFP_STAT_REFACT_TOTAL)
    return(lp->invB->num_refact);
  else if(kind == BFP_STAT_REFACT_TIMED)
    return(lp->invB->num_timed_refact);
  else if(kind == BFP_STAT_REFACT_DENSE)
    return(lp->invB->num_dense_refact);
  else
    return( BFP_STAT_ERROR );
}

/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_mustrefactorize(lprec *lp)
{
  MYBOOL test = lp->is_action(lp->spx_action, ACTION_REINVERT | ACTION_TIMEDREINVERT);
  if(!test) {
    REAL   f;
    INVrec *lu = lp->invB;

    if(lu->num_pivots > 0)
      f = (timeNow()-lu->time_refactstart) / (REAL) lu->num_pivots;
    else
      f = 0;

    /* Always refactorize if we are above the set pivot limit */
    if(lu->force_refact ||
       (lu->num_pivots >= lp->bfp_pivotmax(lp)))
      lp->set_action(&lp->spx_action, ACTION_REINVERT);

    /* Check if we should do an optimal time-based refactorization */
    else if(lu->timed_refact && (lu->num_pivots > 1) &&
            (f > MIN_TIMEPIVOT) && (f > lu->time_refactnext)) {
      /* If we have excessive time usage in automatic mode then
         treat as untimed case and update optimal time metric, ... */
      if((lu->timed_refact == AUTOMATIC) &&
         (lu->num_pivots < 0.4*lp->bfp_pivotmax(lp)))
        lu->time_refactnext = f;
      /* ... otherwise set flag for the optimal time-based refactorization */
      else
        lp->set_action(&lp->spx_action, ACTION_TIMEDREINVERT);
    }

    /* Otherwise simply update the optimal time metric */
    else
      lu->time_refactnext = f;
#if 0
    if(lu->num_pivots % 10 == 0)
      lp->report(lp, NORMAL, "bfp pivot %d - start %f - timestat %f",
                             lu->num_pivots, lu->time_refactstart, f);
#endif
  }

  test = lp->is_action(lp->spx_action, ACTION_REINVERT | ACTION_TIMEDREINVERT);
  return(test);
}

/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_isSetI(lprec *lp)
{
  return( (MYBOOL) lp->invB->set_Bidentity );
}

/* DON'T MODIFY */
int *bfp_createMDO(lprec *lp, MYBOOL *usedpos, int count, MYBOOL doMDO)
{
  int *mdo, i, j, kk;

  mdo = (int *) malloc((count + 1)*sizeof(*mdo));
/*  allocINT(lp, &mdo, count + 1, FALSE); */

 /* Fill the mdo[] array with remaining full-pivot basic user variables */
  kk = 0;
  for(j = 1; j <= lp->columns; j++) {
    i = lp->rows + j;
    if(usedpos[i] == TRUE) {
      kk++;
      mdo[kk] = i;
    }
  }
  mdo[0] = kk;
  if(kk == 0)
    goto Process;

 /* Calculate the approximate minimum degree column ordering */
  if(doMDO) {
    i = lp->getMDO(lp, usedpos, mdo, NULL, FALSE);
    if(i != 0) {
      lp->report(lp, CRITICAL, "bfp_createMDO: Internal error %d in minimum degree ordering routine", i);
      FREE(mdo);
    }
  }
Process:
  return( mdo );
}
void BFP_CALLMODEL bfp_updaterefactstats(lprec *lp)
{
  INVrec *lu = lp->invB;

  /* Signal that we are refactorizing */
  lu->is_dirty = AUTOMATIC;

  /* Set time of start of current refactorization cycle */
  lu->time_refactstart = timeNow();
  lu->time_refactnext  = 0;
  lu->user_colcount = 0;

  /* Do the numbers */
  if(lu->force_refact)
    lu->num_dense_refact++;
  else if(lu->timed_refact && lp->is_action(lp->spx_action, ACTION_TIMEDREINVERT))
    lu->num_timed_refact++;
  lu->num_refact++;
}

int BFP_CALLMODEL bfp_rowextra(lprec *lp)
{
  if(lp->is_obj_in_basis(lp))
    return( 1 );
  else
    return( 0 );
}
