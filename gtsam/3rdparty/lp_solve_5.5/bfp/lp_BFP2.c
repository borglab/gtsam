

/* Routines located in lp_BFP2.cpp; optional shared for canned implementations        */
/* Cfr. lp_BFP.h for definitions                                                      */
/* ---------------------------------------------------------------------------------- */


/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_init(lprec *lp, int size, int delta, char *options)
{
  INVrec *lu;

  lp->invB = (INVrec *) calloc(1, sizeof(*(lp->invB)));
  lu = lp->invB;
  if((lu == NULL) || 
     !lp->bfp_resize(lp, size) ||
     !lp->bfp_restart(lp))
    return( FALSE );

  /* Store any passed options */
  if(options != NULL) {
    size_t len = strlen(options);
    lu->opts = (char *) malloc(len + 1);
    strcpy(lu->opts, options);
  }

  /* Prepare for factorization and undo values reset by bfp_preparefactorization */
  lp->bfp_preparefactorization(lp);
  lu->num_refact = 0;

  return( TRUE );
}

/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_restart(lprec *lp)
{
  INVrec *lu;

  lu = lp->invB;
  if(lu == NULL)
    return( FALSE );

  lu->status = BFP_STATUS_SUCCESS;
  lu->max_Bsize = 0;          /* The largest NZ-count of the B matrix            */
  lu->max_colcount = 0;       /* The maximum number of user columns in B         */
  lu->max_LUsize = 0;         /* The largest NZ-count of LU-files generated      */
  lu->num_refact = 0;         /* The number of times the basis has been factored */
  lu->num_timed_refact = 0;
  lu->num_dense_refact = 0;
  lu->num_pivots = 0;         /* The number of pivots since last factorization   */
  lu->pcol = NULL;
  lu->set_Bidentity = FALSE;

  return( TRUE );
}

/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_implicitslack(lprec *lp)
{
  return( FALSE );
}

/* DON'T MODIFY */
int BFP_CALLMODEL bfp_colcount(lprec *lp)
{
  return(lp->invB->user_colcount);
}


/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_canresetbasis(lprec *lp)
{
  return( FALSE );
}


/* DON'T MODIFY */
MYBOOL BFP_CALLMODEL bfp_pivotalloc(lprec *lp, int newsize)
{
  /* Does nothing in the default implementation */
  return( TRUE );
}


/* DON'T MODIFY */
void BFP_CALLMODEL bfp_finishfactorization(lprec *lp)
{
  INVrec *lu;

  lu = lp->invB;

  SETMAX(lu->max_colcount, lp->bfp_colcount(lp));
  SETMAX(lu->max_LUsize, lp->bfp_nonzeros(lp, FALSE));

  /* Signal that we done factorizing/reinverting */
  lu->is_dirty = FALSE;
  lp->clear_action(&lp->spx_action, ACTION_REINVERT | ACTION_TIMEDREINVERT);
  lu->force_refact = FALSE;

  /* Store information about the current inverse */
  lu->num_pivots = 0;

}


/* DON'T MODIFY */
LREAL BFP_CALLMODEL bfp_prepareupdate(lprec *lp, int row_nr, int col_nr, REAL *pcol)
/* Was condensecol() in versions of lp_solve before 4.0.1.8 - KE */
{
  LREAL  pivValue;
  INVrec *lu;

  lu = lp->invB;

  /* Store the incoming pivot value for RHS update purposes */
  lu->col_enter = col_nr;  /* The index of the new data column */
  lu->col_pos   = row_nr;  /* The basis column to be replaced */
  lu->col_leave = lp->var_basic[row_nr];
  if(pcol == NULL)
    pivValue = 0;
  else
    pivValue = pcol[row_nr];
  lu->theta_enter = pivValue;

  /* Save reference to the elimination vector */
  lu->pcol = pcol;

  /* Set completion status; but hold if we are reinverting */
  if(lu->is_dirty != AUTOMATIC)
    lu->is_dirty = TRUE;

  return( pivValue );
}


/* DON'T MODIFY */
REAL BFP_CALLMODEL bfp_pivotRHS(lprec *lp, LREAL theta, REAL *pcol)
/* This function is used to adjust the RHS in bound swap operations as
   well as handling the updating of the RHS for normal basis changes.
   Was rhsmincol(), ie. "rhs minus column" in versions of lp_solve before 4.0.1.8 - KE */
{
  INVrec    *lu;

  lu = lp->invB;

  if(pcol == NULL)
    pcol = lu->pcol;

  if(theta != 0) {
    register int    i, n = lp->rows;
    register LREAL  roundzero = lp->epsvalue;
    register LREAL  *rhs = lp->rhs, rhsmax = 0;

    for(i = 0; i <= n; i++, rhs++, pcol++) {
      (*rhs) -= theta * (*pcol);
      my_roundzero(*rhs, roundzero);
      SETMAX(rhsmax, fabs(*rhs));
    }
    lp->rhsmax = rhsmax;
  }

  if(pcol == lu->pcol)
    return( lu->theta_enter );
  else
    return( 0.0 );
}


/* DON'T MODIFY */
void BFP_CALLMODEL bfp_btran_double(lprec *lp, REAL *prow, int *pnzidx, REAL *drow, int *dnzidx)
{
  if(prow != NULL)
    lp->bfp_btran_normal(lp, prow, pnzidx);
  if(drow != NULL)
    lp->bfp_btran_normal(lp, drow, dnzidx);
}

