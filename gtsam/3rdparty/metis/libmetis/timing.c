/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * timing.c
 *
 * This file contains routines that deal with timing Metis
 *
 * Started 7/24/97
 * George
 *
 * $Id: timing.c 13936 2013-03-30 03:59:09Z karypis $
 *
 */

#include "metislib.h"


/*************************************************************************
* This function clears the timers
**************************************************************************/
void InitTimers(ctrl_t *ctrl)
{
  gk_clearcputimer(ctrl->TotalTmr);
  gk_clearcputimer(ctrl->InitPartTmr);
  gk_clearcputimer(ctrl->MatchTmr);
  gk_clearcputimer(ctrl->ContractTmr);
  gk_clearcputimer(ctrl->CoarsenTmr);
  gk_clearcputimer(ctrl->UncoarsenTmr);
  gk_clearcputimer(ctrl->RefTmr);
  gk_clearcputimer(ctrl->ProjectTmr);
  gk_clearcputimer(ctrl->SplitTmr);
  gk_clearcputimer(ctrl->Aux1Tmr);
  gk_clearcputimer(ctrl->Aux2Tmr);
  gk_clearcputimer(ctrl->Aux3Tmr);
}



/*************************************************************************
* This function prints the various timers
**************************************************************************/
void PrintTimers(ctrl_t *ctrl)
{
  printf("\nTiming Information -------------------------------------------------");
  printf("\n Multilevel: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->TotalTmr));
  printf("\n     Coarsening: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->CoarsenTmr));
  printf("\n            Matching: \t\t\t %7.3"PRREAL"", gk_getcputimer(ctrl->MatchTmr));
  printf("\n            Contract: \t\t\t %7.3"PRREAL"", gk_getcputimer(ctrl->ContractTmr));
  printf("\n     Initial Partition: \t %7.3"PRREAL"", gk_getcputimer(ctrl->InitPartTmr));
  printf("\n     Uncoarsening: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->UncoarsenTmr));
  printf("\n          Refinement: \t\t\t %7.3"PRREAL"", gk_getcputimer(ctrl->RefTmr));
  printf("\n          Projection: \t\t\t %7.3"PRREAL"", gk_getcputimer(ctrl->ProjectTmr));
  printf("\n     Splitting: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->SplitTmr));
/*
  printf("\n       Aux1Tmr: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->Aux1Tmr));
  printf("\n       Aux2Tmr: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->Aux2Tmr));
  printf("\n       Aux3Tmr: \t\t %7.3"PRREAL"", gk_getcputimer(ctrl->Aux3Tmr));
*/
  printf("\n********************************************************************\n");
}



