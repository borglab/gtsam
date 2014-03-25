/*!
\file
\brief Functions for the edge-based FM refinement

\date Started 7/23/97
\author George  
\author Copyright 1997-2011, Regents of the University of Minnesota 
\version\verbatim $Id: fm.c 10187 2011-06-13 13:46:57Z karypis $ \endverbatim
*/

#include "metislib.h"


/*************************************************************************
* This function performs an edge-based FM refinement
**************************************************************************/
void FM_2WayRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter)
{
  if (graph->ncon == 1) 
    FM_2WayCutRefine(ctrl, graph, ntpwgts, niter);
  else
    FM_Mc2WayCutRefine(ctrl, graph, ntpwgts, niter);
}


/*************************************************************************/
/*! This function performs a cut-focused FM refinement */
/*************************************************************************/
void FM_2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter)
{
  idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, limit, tmp;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
  idx_t *moved, *swaps, *perm;
  rpq_t *queues[2];
  idx_t higain, mincut, mindiff, origdiff, initcut, newcut, mincutorder, avgvwgt;
  idx_t tpwgts[2];

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  where  = graph->where;
  id     = graph->id;
  ed     = graph->ed;
  pwgts  = graph->pwgts;
  bndptr = graph->bndptr;
  bndind = graph->bndind;

  moved = iwspacemalloc(ctrl, nvtxs);
  swaps = iwspacemalloc(ctrl, nvtxs);
  perm  = iwspacemalloc(ctrl, nvtxs);

  tpwgts[0] = graph->tvwgt[0]*ntpwgts[0];
  tpwgts[1] = graph->tvwgt[0]-tpwgts[0];
  
  limit   = gk_min(gk_max(0.01*nvtxs, 15), 100);
  avgvwgt = gk_min((pwgts[0]+pwgts[1])/20, 2*(pwgts[0]+pwgts[1])/nvtxs);

  queues[0] = rpqCreate(nvtxs);
  queues[1] = rpqCreate(nvtxs);

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
      Print2WayRefineStats(ctrl, graph, ntpwgts, 0, -2));

  origdiff = iabs(tpwgts[0]-pwgts[0]);
  iset(nvtxs, -1, moved);
  for (pass=0; pass<niter; pass++) { /* Do a number of passes */
    rpqReset(queues[0]);
    rpqReset(queues[1]);

    mincutorder = -1;
    newcut = mincut = initcut = graph->mincut;
    mindiff = iabs(tpwgts[0]-pwgts[0]);

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERT(CheckBnd(graph));

    /* Insert boundary nodes in the priority queues */
    nbnd = graph->nbnd;
    irandArrayPermute(nbnd, perm, nbnd, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = perm[ii];
      ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
      ASSERT(bndptr[bndind[i]] != -1);
      rpqInsert(queues[where[bndind[i]]], bndind[i], ed[bndind[i]]-id[bndind[i]]);
    }

    for (nswaps=0; nswaps<nvtxs; nswaps++) {
      from = (tpwgts[0]-pwgts[0] < tpwgts[1]-pwgts[1] ? 0 : 1);
      to = (from+1)%2;

      if ((higain = rpqGetTop(queues[from])) == -1)
        break;
      ASSERT(bndptr[higain] != -1);

      newcut -= (ed[higain]-id[higain]);
      INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

      if ((newcut < mincut && iabs(tpwgts[0]-pwgts[0]) <= origdiff+avgvwgt) || 
          (newcut == mincut && iabs(tpwgts[0]-pwgts[0]) < mindiff)) {
        mincut  = newcut;
        mindiff = iabs(tpwgts[0]-pwgts[0]);
        mincutorder = nswaps;
      }
      else if (nswaps-mincutorder > limit) { /* We hit the limit, undo last move */
        newcut += (ed[higain]-id[higain]);
        INC_DEC(pwgts[from], pwgts[to], vwgt[higain]);
        break;
      }

      where[higain] = to;
      moved[higain] = nswaps;
      swaps[nswaps] = higain;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
        printf("Moved %6"PRIDX" from %"PRIDX". [%3"PRIDX" %3"PRIDX"] %5"PRIDX" [%4"PRIDX" %4"PRIDX"]\n", higain, from, ed[higain]-id[higain], vwgt[higain], newcut, pwgts[0], pwgts[1]));

      /**************************************************************
      * Update the id[i]/ed[i] values of the affected nodes
      ***************************************************************/
      SWAP(id[higain], ed[higain], tmp);
      if (ed[higain] == 0 && xadj[higain] < xadj[higain+1]) 
        BNDDelete(nbnd, bndind,  bndptr, higain);

      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];

        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
        INC_DEC(id[k], ed[k], kwgt);

        /* Update its boundary information and queue position */
        if (bndptr[k] != -1) { /* If k was a boundary vertex */
          if (ed[k] == 0) { /* Not a boundary vertex any more */
            BNDDelete(nbnd, bndind, bndptr, k);
            if (moved[k] == -1)  /* Remove it if in the queues */
              rpqDelete(queues[where[k]], k);
          }
          else { /* If it has not been moved, update its position in the queue */
            if (moved[k] == -1) 
              rpqUpdate(queues[where[k]], k, ed[k]-id[k]);
          }
        }
        else {
          if (ed[k] > 0) {  /* It will now become a boundary vertex */
            BNDInsert(nbnd, bndind, bndptr, k);
            if (moved[k] == -1) 
              rpqInsert(queues[where[k]], k, ed[k]-id[k]);
          }
        }
      }

    }


    /****************************************************************
    * Roll back computations
    *****************************************************************/
    for (i=0; i<nswaps; i++)
      moved[swaps[i]] = -1;  /* reset moved array */
    for (nswaps--; nswaps>mincutorder; nswaps--) {
      higain = swaps[nswaps];

      to = where[higain] = (where[higain]+1)%2;
      SWAP(id[higain], ed[higain], tmp);
      if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain+1])
        BNDDelete(nbnd, bndind,  bndptr, higain);
      else if (ed[higain] > 0 && bndptr[higain] == -1)
        BNDInsert(nbnd, bndind,  bndptr, higain);

      INC_DEC(pwgts[to], pwgts[(to+1)%2], vwgt[higain]);
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];

        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
        INC_DEC(id[k], ed[k], kwgt);

        if (bndptr[k] != -1 && ed[k] == 0)
          BNDDelete(nbnd, bndind, bndptr, k);
        if (bndptr[k] == -1 && ed[k] > 0)
          BNDInsert(nbnd, bndind, bndptr, k);
      }
    }

    graph->mincut = mincut;
    graph->nbnd   = nbnd;

    IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
        Print2WayRefineStats(ctrl, graph, ntpwgts, 0, mincutorder));

    if (mincutorder <= 0 || mincut == initcut)
      break;
  }

  rpqDestroy(queues[0]);
  rpqDestroy(queues[1]);

  WCOREPOP;
}


/*************************************************************************/
/*! This function performs a cut-focused multi-constraint FM refinement */
/*************************************************************************/
void FM_Mc2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter)
{
  idx_t i, ii, j, k, l, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, pass, 
        me, limit, tmp, cnum;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *pwgts, *where, *id, *ed, 
        *bndptr, *bndind;
  idx_t *moved, *swaps, *perm, *qnum;
  idx_t higain, mincut, initcut, newcut, mincutorder;
  real_t *invtvwgt, *ubfactors, *minbalv, *newbalv;
  real_t origbal, minbal, newbal, rgain, ffactor;
  rpq_t **queues;

  WCOREPUSH;

  nvtxs    = graph->nvtxs;
  ncon     = graph->ncon;
  xadj     = graph->xadj;
  vwgt     = graph->vwgt;
  adjncy   = graph->adjncy;
  adjwgt   = graph->adjwgt;
  invtvwgt = graph->invtvwgt;
  where    = graph->where;
  id       = graph->id;
  ed       = graph->ed;
  pwgts    = graph->pwgts;
  bndptr   = graph->bndptr;
  bndind   = graph->bndind;

  moved     = iwspacemalloc(ctrl, nvtxs);
  swaps     = iwspacemalloc(ctrl, nvtxs);
  perm      = iwspacemalloc(ctrl, nvtxs);
  qnum      = iwspacemalloc(ctrl, nvtxs);
  ubfactors = rwspacemalloc(ctrl, ncon);
  newbalv   = rwspacemalloc(ctrl, ncon);
  minbalv   = rwspacemalloc(ctrl, ncon);

  limit = gk_min(gk_max(0.01*nvtxs, 25), 150);


  /* Determine a fudge factor to allow the refinement routines to get out 
     of tight balancing constraints. */
  ffactor = .5/gk_max(20, nvtxs);

  /* Initialize the queues */
  queues = (rpq_t **)wspacemalloc(ctrl, 2*ncon*sizeof(rpq_t *));
  for (i=0; i<2*ncon; i++) 
    queues[i] = rpqCreate(nvtxs);
  for (i=0; i<nvtxs; i++)
    qnum[i] = iargmax_nrm(ncon, vwgt+i*ncon, invtvwgt);

  /* Determine the unbalance tolerance for each constraint. The tolerance is
     equal to the maximum of the original load imbalance and the user-supplied
     allowed tolerance. The rationale behind this approach is to allow the
     refinement routine to improve the cut, without having to worry about fixing
     load imbalance problems. The load imbalance is addressed by the balancing
     routines. */
  origbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, ubfactors);
  for (i=0; i<ncon; i++) 
    ubfactors[i] = (ubfactors[i] > 0 ? ctrl->ubfactors[i]+ubfactors[i] : ctrl->ubfactors[i]);


  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
      Print2WayRefineStats(ctrl, graph, ntpwgts, origbal, -2));

  iset(nvtxs, -1, moved);
  for (pass=0; pass<niter; pass++) { /* Do a number of passes */
    for (i=0; i<2*ncon; i++)  
      rpqReset(queues[i]);

    mincutorder = -1;
    newcut = mincut = initcut = graph->mincut;

    minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ubfactors, minbalv);

    ASSERT(ComputeCut(graph, where) == graph->mincut);
    ASSERT(CheckBnd(graph));

    /* Insert boundary nodes in the priority queues */
    nbnd = graph->nbnd;
    irandArrayPermute(nbnd, perm, nbnd/5, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = bndind[perm[ii]];
      ASSERT(ed[i] > 0 || id[i] == 0);
      ASSERT(bndptr[i] != -1);
      //rgain = 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1);
      //rgain = (ed[i]-id[i] > 0 ? 1.0*(ed[i]-id[i])/sqrt(vwgt[i*ncon+qnum[i]]+1) : ed[i]-id[i]);
      rgain = ed[i]-id[i];
      rpqInsert(queues[2*qnum[i]+where[i]], i, rgain);
    }

    for (nswaps=0; nswaps<nvtxs; nswaps++) {
      SelectQueue(graph, ctrl->pijbm, ubfactors, queues, &from, &cnum);

      to = (from+1)%2;

      if (from == -1 || (higain = rpqGetTop(queues[2*cnum+from])) == -1)
        break;
      ASSERT(bndptr[higain] != -1);

      newcut -= (ed[higain]-id[higain]);

      iaxpy(ncon,  1, vwgt+higain*ncon, 1, pwgts+to*ncon,   1);
      iaxpy(ncon, -1, vwgt+higain*ncon, 1, pwgts+from*ncon, 1);
      newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ubfactors, newbalv);

      if ((newcut < mincut && newbal <= ffactor) || 
          (newcut == mincut && (newbal < minbal || 
           (newbal == minbal && BetterBalance2Way(ncon, minbalv, newbalv))))) {
        mincut      = newcut;
        minbal      = newbal;
        mincutorder = nswaps;
        rcopy(ncon, newbalv, minbalv);
      }
      else if (nswaps-mincutorder > limit) { /* We hit the limit, undo last move */
        newcut += (ed[higain]-id[higain]);
        iaxpy(ncon,  1, vwgt+higain*ncon, 1, pwgts+from*ncon, 1);
        iaxpy(ncon, -1, vwgt+higain*ncon, 1, pwgts+to*ncon,   1);
        break;
      }

      where[higain] = to;
      moved[higain] = nswaps;
      swaps[nswaps] = higain;

      if (ctrl->dbglvl&METIS_DBG_MOVEINFO) {
        printf("Moved%6"PRIDX" from %"PRIDX"(%"PRIDX") Gain:%5"PRIDX", "
            "Cut:%5"PRIDX", NPwgts:", higain, from, cnum, ed[higain]-id[higain], newcut);
        for (l=0; l<ncon; l++) 
          printf("(%.3"PRREAL" %.3"PRREAL")", pwgts[l]*invtvwgt[l], pwgts[ncon+l]*invtvwgt[l]);
        printf(" %+.3"PRREAL" LB: %.3"PRREAL"(%+.3"PRREAL")\n", 
            minbal, ComputeLoadImbalance(graph, 2, ctrl->pijbm), newbal);
      }


      /**************************************************************
      * Update the id[i]/ed[i] values of the affected nodes
      ***************************************************************/
      SWAP(id[higain], ed[higain], tmp);
      if (ed[higain] == 0 && xadj[higain] < xadj[higain+1]) 
        BNDDelete(nbnd, bndind,  bndptr, higain);

      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];

        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
        INC_DEC(id[k], ed[k], kwgt);

        /* Update its boundary information and queue position */
        if (bndptr[k] != -1) { /* If k was a boundary vertex */
          if (ed[k] == 0) { /* Not a boundary vertex any more */
            BNDDelete(nbnd, bndind, bndptr, k);
            if (moved[k] == -1)  /* Remove it if in the queues */
              rpqDelete(queues[2*qnum[k]+where[k]], k);
          }
          else { /* If it has not been moved, update its position in the queue */
            if (moved[k] == -1) {
              //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
              //rgain = (ed[k]-id[k] > 0 ? 
              //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
              rgain = ed[k]-id[k];
              rpqUpdate(queues[2*qnum[k]+where[k]], k, rgain);
            }
          }
        }
        else {
          if (ed[k] > 0) {  /* It will now become a boundary vertex */
            BNDInsert(nbnd, bndind, bndptr, k);
            if (moved[k] == -1) {
              //rgain = 1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1);
              //rgain = (ed[k]-id[k] > 0 ? 
              //              1.0*(ed[k]-id[k])/sqrt(vwgt[k*ncon+qnum[k]]+1) : ed[k]-id[k]);
              rgain = ed[k]-id[k];
              rpqInsert(queues[2*qnum[k]+where[k]], k, rgain);
            }
          }
        }
      }

    }


    /****************************************************************
    * Roll back computations
    *****************************************************************/
    for (i=0; i<nswaps; i++)
      moved[swaps[i]] = -1;  /* reset moved array */
    for (nswaps--; nswaps>mincutorder; nswaps--) {
      higain = swaps[nswaps];

      to = where[higain] = (where[higain]+1)%2;
      SWAP(id[higain], ed[higain], tmp);
      if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain+1])
        BNDDelete(nbnd, bndind,  bndptr, higain);
      else if (ed[higain] > 0 && bndptr[higain] == -1)
        BNDInsert(nbnd, bndind,  bndptr, higain);

      iaxpy(ncon,  1, vwgt+higain*ncon, 1, pwgts+to*ncon,         1);
      iaxpy(ncon, -1, vwgt+higain*ncon, 1, pwgts+((to+1)%2)*ncon, 1);
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];

        kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
        INC_DEC(id[k], ed[k], kwgt);

        if (bndptr[k] != -1 && ed[k] == 0)
          BNDDelete(nbnd, bndind, bndptr, k);
        if (bndptr[k] == -1 && ed[k] > 0)
          BNDInsert(nbnd, bndind, bndptr, k);
      }
    }

    graph->mincut = mincut;
    graph->nbnd   = nbnd;

    IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
        Print2WayRefineStats(ctrl, graph, ntpwgts, minbal, mincutorder));

    if (mincutorder <= 0 || mincut == initcut)
      break;
  }

  for (i=0; i<2*ncon; i++) 
    rpqDestroy(queues[i]);

  WCOREPOP;
}


/*************************************************************************/
/*! This function selects the partition number and the queue from which
    we will move vertices out. */
/*************************************************************************/ 
void SelectQueue(graph_t *graph, real_t *pijbm, real_t *ubfactors, 
         rpq_t **queues, idx_t *from, idx_t *cnum)
{
  idx_t ncon, i, part;
  real_t max, tmp;

  ncon = graph->ncon;

  *from = -1;
  *cnum = -1;

  /* First determine the side and the queue, irrespective of the presence of nodes. 
     The side & queue is determined based on the most violated balancing constraint. */
  for (max=0.0, part=0; part<2; part++) {
    for (i=0; i<ncon; i++) {
      tmp = graph->pwgts[part*ncon+i]*pijbm[part*ncon+i] - ubfactors[i];
      /* the '=' in the test bellow is to ensure that under tight constraints
         the partition that is at the max is selected */
      if (tmp >= max) { 
        max   = tmp;
        *from = part;
        *cnum = i;
      }
    }
  }


  if (*from != -1) {
    /* in case the desired queue is empty, select a queue from the same side */
    if (rpqLength(queues[2*(*cnum)+(*from)]) == 0) {
      for (i=0; i<ncon; i++) {
        if (rpqLength(queues[2*i+(*from)]) > 0) {
          max   = graph->pwgts[(*from)*ncon+i]*pijbm[(*from)*ncon+i] - ubfactors[i];
          *cnum = i;
          break;
        }
      }

      for (i++; i<ncon; i++) {
        tmp = graph->pwgts[(*from)*ncon+i]*pijbm[(*from)*ncon+i] - ubfactors[i];
        if (tmp > max && rpqLength(queues[2*i+(*from)]) > 0) {
          max   = tmp;
          *cnum = i;
        }
      }
    }

    /*
    printf("Selected1 %"PRIDX"(%"PRIDX") -> %"PRIDX" [%5"PRREAL"]\n", 
        *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max); 
    */
  }
  else {
    /* the partitioning does not violate balancing constraints, in which case select 
       a queue based on cut criteria */
    for (part=0; part<2; part++) {
      for (i=0; i<ncon; i++) {
        if (rpqLength(queues[2*i+part]) > 0 && 
            (*from == -1 || rpqSeeTopKey(queues[2*i+part]) > max)) {
          max   = rpqSeeTopKey(queues[2*i+part]); 
          *from = part;
          *cnum = i;
        }
      }
    }
    /*
    printf("Selected2 %"PRIDX"(%"PRIDX") -> %"PRIDX"\n", 
        *from, *cnum, rpqLength(queues[2*(*cnum)+(*from)]), max); 
    */
  }
}


/*************************************************************************/
/*! Prints statistics about the refinement */
/*************************************************************************/ 
void Print2WayRefineStats(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         real_t deltabal, idx_t mincutorder)
{
  int i;

  if (mincutorder == -2) {
    printf("Parts: ");
    printf("Nv-Nb[%5"PRIDX" %5"PRIDX"] ICut: %6"PRIDX, 
        graph->nvtxs, graph->nbnd, graph->mincut);
    printf(" [");
    for (i=0; i<graph->ncon; i++)
      printf("(%.3"PRREAL" %.3"PRREAL" T:%.3"PRREAL" %.3"PRREAL")", 
          graph->pwgts[i]*graph->invtvwgt[i], 
          graph->pwgts[graph->ncon+i]*graph->invtvwgt[i],
          ntpwgts[i], ntpwgts[graph->ncon+i]);
    printf("] LB: %.3"PRREAL"(%+.3"PRREAL")\n", 
        ComputeLoadImbalance(graph, 2, ctrl->pijbm), deltabal);
  }
  else {
    printf("\tMincut: %6"PRIDX" at %5"PRIDX" NBND %6"PRIDX" NPwgts: [", 
        graph->mincut, mincutorder, graph->nbnd);
    for (i=0; i<graph->ncon; i++)
      printf("(%.3"PRREAL" %.3"PRREAL")", 
          graph->pwgts[i]*graph->invtvwgt[i], graph->pwgts[graph->ncon+i]*graph->invtvwgt[i]);
    printf("] LB: %.3"PRREAL"(%+.3"PRREAL")\n", 
        ComputeLoadImbalance(graph, 2, ctrl->pijbm), deltabal);
  }
}

