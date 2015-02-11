/*!
\file
\brief Functions for the edge-based balancing 

\date Started 7/23/97
\author George  
\author Copyright 1997-2011, Regents of the University of Minnesota 
\version\verbatim $Id: balance.c 10187 2011-06-13 13:46:57Z karypis $ \endverbatim
*/

#include "metislib.h"

/*************************************************************************
* This function is the entry poidx_t of the bisection balancing algorithms.
**************************************************************************/
void Balance2Way(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts)
{
  if (ComputeLoadImbalanceDiff(graph, 2, ctrl->pijbm, ctrl->ubfactors) <= 0) 
    return;

  if (graph->ncon == 1) {
    /* return right away if the balance is OK */
    if (iabs(ntpwgts[0]*graph->tvwgt[0]-graph->pwgts[0]) < 3*graph->tvwgt[0]/graph->nvtxs)
      return;

    if (graph->nbnd > 0)
      Bnd2WayBalance(ctrl, graph, ntpwgts);
    else
      General2WayBalance(ctrl, graph, ntpwgts);
  }
  else {
    McGeneral2WayBalance(ctrl, graph, ntpwgts);
  }
}


/*************************************************************************
* This function balances two partitions by moving boundary nodes
* from the domain that is overweight to the one that is underweight.
**************************************************************************/
void Bnd2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts)
{
  idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, tmp;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
  idx_t *moved, *perm;
  rpq_t *queue;
  idx_t higain, mincut, mindiff;
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
  perm  = iwspacemalloc(ctrl, nvtxs);

  /* Determine from which domain you will be moving data */
  tpwgts[0] = graph->tvwgt[0]*ntpwgts[0];
  tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
  mindiff   = iabs(tpwgts[0]-pwgts[0]);
  from      = (pwgts[0] < tpwgts[0] ? 1 : 0);
  to        = (from+1)%2;

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
     printf("Partitions: [%6"PRIDX" %6"PRIDX"] T[%6"PRIDX" %6"PRIDX"], Nv-Nb[%6"PRIDX" %6"PRIDX"]. ICut: %6"PRIDX" [B]\n",
             pwgts[0], pwgts[1], tpwgts[0], tpwgts[1], graph->nvtxs, graph->nbnd, 
             graph->mincut));

  queue = rpqCreate(nvtxs);

  iset(nvtxs, -1, moved);

  ASSERT(ComputeCut(graph, where) == graph->mincut);
  ASSERT(CheckBnd(graph));

  /* Insert the boundary nodes of the proper partition whose size is OK in the priority queue */
  nbnd = graph->nbnd;
  irandArrayPermute(nbnd, perm, nbnd/5, 1);
  for (ii=0; ii<nbnd; ii++) {
    i = perm[ii];
    ASSERT(ed[bndind[i]] > 0 || id[bndind[i]] == 0);
    ASSERT(bndptr[bndind[i]] != -1);
    if (where[bndind[i]] == from && vwgt[bndind[i]] <= mindiff)
      rpqInsert(queue, bndind[i], ed[bndind[i]]-id[bndind[i]]);
  }

  mincut = graph->mincut;
  for (nswaps=0; nswaps<nvtxs; nswaps++) {
    if ((higain = rpqGetTop(queue)) == -1)
      break;
    ASSERT(bndptr[higain] != -1);

    if (pwgts[to]+vwgt[higain] > tpwgts[to])
      break;

    mincut -= (ed[higain]-id[higain]);
    INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

    where[higain] = to;
    moved[higain] = nswaps;

    IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
      printf("Moved %6"PRIDX" from %"PRIDX". [%3"PRIDX" %3"PRIDX"] %5"PRIDX" [%4"PRIDX" %4"PRIDX"]\n", higain, from, ed[higain]-id[higain], vwgt[higain], mincut, pwgts[0], pwgts[1]));

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
          if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)  /* Remove it if in the queues */
            rpqDelete(queue, k);
        }
        else { /* If it has not been moved, update its position in the queue */
          if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
            rpqUpdate(queue, k, ed[k]-id[k]);
        }
      }
      else {
        if (ed[k] > 0) {  /* It will now become a boundary vertex */
          BNDInsert(nbnd, bndind, bndptr, k);
          if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff) 
            rpqInsert(queue, k, ed[k]-id[k]);
        }
      }
    }
  }

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
    printf("\tMinimum cut: %6"PRIDX", PWGTS: [%6"PRIDX" %6"PRIDX"], NBND: %6"PRIDX"\n", mincut, pwgts[0], pwgts[1], nbnd));

  graph->mincut = mincut;
  graph->nbnd   = nbnd;

  rpqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************
* This function balances two partitions by moving the highest gain 
* (including negative gain) vertices to the other domain.
* It is used only when tha unbalance is due to non contigous
* subdomains. That is, the are no boundary vertices.
* It moves vertices from the domain that is overweight to the one that 
* is underweight.
**************************************************************************/
void General2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts)
{
  idx_t i, ii, j, k, kwgt, nvtxs, nbnd, nswaps, from, to, pass, me, tmp;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *id, *ed, *bndptr, *bndind, *pwgts;
  idx_t *moved, *perm;
  rpq_t *queue;
  idx_t higain, mincut, mindiff;
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
  perm  = iwspacemalloc(ctrl, nvtxs);

  /* Determine from which domain you will be moving data */
  tpwgts[0] = graph->tvwgt[0]*ntpwgts[0];
  tpwgts[1] = graph->tvwgt[0] - tpwgts[0];
  mindiff   = iabs(tpwgts[0]-pwgts[0]);
  from      = (pwgts[0] < tpwgts[0] ? 1 : 0);
  to        = (from+1)%2;

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
     printf("Partitions: [%6"PRIDX" %6"PRIDX"] T[%6"PRIDX" %6"PRIDX"], Nv-Nb[%6"PRIDX" %6"PRIDX"]. ICut: %6"PRIDX" [B]\n",
             pwgts[0], pwgts[1], tpwgts[0], tpwgts[1], graph->nvtxs, graph->nbnd, graph->mincut));

  queue = rpqCreate(nvtxs);

  iset(nvtxs, -1, moved);

  ASSERT(ComputeCut(graph, where) == graph->mincut);
  ASSERT(CheckBnd(graph));

  /* Insert the nodes of the proper partition whose size is OK in the priority queue */
  irandArrayPermute(nvtxs, perm, nvtxs/5, 1);
  for (ii=0; ii<nvtxs; ii++) {
    i = perm[ii];
    if (where[i] == from && vwgt[i] <= mindiff)
      rpqInsert(queue, i, ed[i]-id[i]);
  }

  mincut = graph->mincut;
  nbnd = graph->nbnd;
  for (nswaps=0; nswaps<nvtxs; nswaps++) {
    if ((higain = rpqGetTop(queue)) == -1)
      break;

    if (pwgts[to]+vwgt[higain] > tpwgts[to])
      break;

    mincut -= (ed[higain]-id[higain]);
    INC_DEC(pwgts[to], pwgts[from], vwgt[higain]);

    where[higain] = to;
    moved[higain] = nswaps;

    IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
      printf("Moved %6"PRIDX" from %"PRIDX". [%3"PRIDX" %3"PRIDX"] %5"PRIDX" [%4"PRIDX" %4"PRIDX"]\n", higain, from, ed[higain]-id[higain], vwgt[higain], mincut, pwgts[0], pwgts[1]));

    /**************************************************************
    * Update the id[i]/ed[i] values of the affected nodes
    ***************************************************************/
    SWAP(id[higain], ed[higain], tmp);
    if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain+1]) 
      BNDDelete(nbnd, bndind,  bndptr, higain);
    if (ed[higain] > 0 && bndptr[higain] == -1)
      BNDInsert(nbnd, bndind,  bndptr, higain);

    for (j=xadj[higain]; j<xadj[higain+1]; j++) {
      k = adjncy[j];

      kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
      INC_DEC(id[k], ed[k], kwgt);

      /* Update the queue position */
      if (moved[k] == -1 && where[k] == from && vwgt[k] <= mindiff)
        rpqUpdate(queue, k, ed[k]-id[k]);

      /* Update its boundary information */
      if (ed[k] == 0 && bndptr[k] != -1) 
        BNDDelete(nbnd, bndind, bndptr, k);
      else if (ed[k] > 0 && bndptr[k] == -1)  
        BNDInsert(nbnd, bndind, bndptr, k);
    }
  }

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, 
    printf("\tMinimum cut: %6"PRIDX", PWGTS: [%6"PRIDX" %6"PRIDX"], NBND: %6"PRIDX"\n", mincut, pwgts[0], pwgts[1], nbnd));

  graph->mincut = mincut;
  graph->nbnd   = nbnd;

  rpqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************
* This function performs an edge-based FM refinement
**************************************************************************/
void McGeneral2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts)
{
  idx_t i, ii, j, k, l, kwgt, nvtxs, ncon, nbnd, nswaps, from, to, pass, 
        me, limit, tmp, cnum;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *where, *pwgts, *id, *ed, *bndptr, *bndind;
  idx_t *moved, *swaps, *perm, *qnum, *qsizes;
  idx_t higain, mincut, newcut, mincutorder;
  real_t *invtvwgt, *minbalv, *newbalv, minbal, newbal;
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

  moved   = iwspacemalloc(ctrl, nvtxs);
  swaps   = iwspacemalloc(ctrl, nvtxs);
  perm    = iwspacemalloc(ctrl, nvtxs);
  qnum    = iwspacemalloc(ctrl, nvtxs);
  newbalv = rwspacemalloc(ctrl, ncon);
  minbalv = rwspacemalloc(ctrl, ncon);
  qsizes  = iwspacemalloc(ctrl, 2*ncon);

  limit = gk_min(gk_max(0.01*nvtxs, 15), 100);

  /* Initialize the queues */
  queues = (rpq_t **)wspacemalloc(ctrl, 2*ncon*sizeof(rpq_t *));
  for (i=0; i<2*ncon; i++) {
    queues[i] = rpqCreate(nvtxs);
    qsizes[i] = 0;
  }

  for (i=0; i<nvtxs; i++) {
    qnum[i] = iargmax_nrm(ncon, vwgt+i*ncon, invtvwgt);
    qsizes[2*qnum[i]+where[i]]++;
  }


  /* for the empty queues, move into them vertices from other queues */
  for (from=0; from<2; from++) {
    for (j=0; j<ncon; j++) {
      if (qsizes[2*j+from] == 0) {
        for (i=0; i<nvtxs; i++) {
          if (where[i] != from)
            continue;

          k = iargmax2_nrm(ncon, vwgt+i*ncon, invtvwgt);
          if (k == j && 
              qsizes[2*qnum[i]+from] > qsizes[2*j+from] && 
              vwgt[i*ncon+qnum[i]]*invtvwgt[qnum[i]] < 1.3*vwgt[i*ncon+j]*invtvwgt[j]) {
            qsizes[2*qnum[i]+from]--;
            qsizes[2*j+from]++;
            qnum[i] = j;
          }
        }
      }
    }
  }


  minbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, minbalv);
  ASSERT(minbal > 0.0);

  newcut = mincut = graph->mincut;
  mincutorder = -1;

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
    printf("Parts: [");
    for (l=0; l<ncon; l++)
      printf("(%6"PRIDX" %6"PRIDX" %.3"PRREAL" %.3"PRREAL") ", 
          pwgts[l], pwgts[ncon+l], ntpwgts[l], ntpwgts[ncon+l]);
    printf("] Nv-Nb[%5"PRIDX", %5"PRIDX"]. ICut: %6"PRIDX", LB: %+.3"PRREAL" [B]\n", 
           graph->nvtxs, graph->nbnd, graph->mincut, minbal);
  }

  iset(nvtxs, -1, moved);

  ASSERT(ComputeCut(graph, where) == graph->mincut);
  ASSERT(CheckBnd(graph));

  /* Insert all nodes in the priority queues */
  nbnd = graph->nbnd;
  irandArrayPermute(nvtxs, perm, nvtxs/10, 1);
  for (ii=0; ii<nvtxs; ii++) {
    i = perm[ii];
    rpqInsert(queues[2*qnum[i]+where[i]], i, ed[i]-id[i]);
  }

  for (nswaps=0; nswaps<nvtxs; nswaps++) {
    if (minbal <= 0.0)
      break;

    SelectQueue(graph, ctrl->pijbm, ctrl->ubfactors, queues, &from, &cnum);
    to = (from+1)%2;

    if (from == -1 || (higain = rpqGetTop(queues[2*cnum+from])) == -1)
      break;

    newcut -= (ed[higain]-id[higain]);

    iaxpy(ncon,  1, vwgt+higain*ncon, 1, pwgts+to*ncon,   1);
    iaxpy(ncon, -1, vwgt+higain*ncon, 1, pwgts+from*ncon, 1);
    newbal = ComputeLoadImbalanceDiffVec(graph, 2, ctrl->pijbm, ctrl->ubfactors, newbalv);

    if (newbal < minbal || (newbal == minbal && 
        (newcut < mincut || 
         (newcut == mincut && BetterBalance2Way(ncon, minbalv, newbalv))))) {
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
      printf("Moved %6"PRIDX" from %"PRIDX"(%"PRIDX"). Gain: %5"PRIDX", "
             "Cut: %5"PRIDX", NPwgts: ", higain, from, cnum, ed[higain]-id[higain], newcut);
      for (l=0; l<ncon; l++) 
        printf("(%6"PRIDX", %6"PRIDX") ", pwgts[l], pwgts[ncon+l]);
      printf(", %+.3"PRREAL" LB: %+.3"PRREAL"\n", minbal, newbal);
    }


    /**************************************************************
    * Update the id[i]/ed[i] values of the affected nodes
    ***************************************************************/
    SWAP(id[higain], ed[higain], tmp);
    if (ed[higain] == 0 && bndptr[higain] != -1 && xadj[higain] < xadj[higain+1]) 
      BNDDelete(nbnd, bndind,  bndptr, higain);
    if (ed[higain] > 0 && bndptr[higain] == -1)
      BNDInsert(nbnd, bndind,  bndptr, higain);

    for (j=xadj[higain]; j<xadj[higain+1]; j++) {
      k = adjncy[j];

      kwgt = (to == where[k] ? adjwgt[j] : -adjwgt[j]);
      INC_DEC(id[k], ed[k], kwgt);

      /* Update the queue position */
      if (moved[k] == -1)
        rpqUpdate(queues[2*qnum[k]+where[k]], k, ed[k]-id[k]);

      /* Update its boundary information */
      if (ed[k] == 0 && bndptr[k] != -1) 
        BNDDelete(nbnd, bndind, bndptr, k);
      else if (ed[k] > 0 && bndptr[k] == -1)  
        BNDInsert(nbnd, bndind, bndptr, k);
    }
  }



  /****************************************************************
  * Roll back computations
  *****************************************************************/
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

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
    printf("\tMincut: %6"PRIDX" at %5"PRIDX", NBND: %6"PRIDX", NPwgts: [", 
        mincut, mincutorder, nbnd);
    for (l=0; l<ncon; l++)
      printf("(%6"PRIDX", %6"PRIDX") ", pwgts[l], pwgts[ncon+l]);
    printf("], LB: %.3"PRREAL"\n", ComputeLoadImbalance(graph, 2, ctrl->pijbm));
  }

  graph->mincut = mincut;
  graph->nbnd   = nbnd;


  for (i=0; i<2*ncon; i++) 
    rpqDestroy(queues[i]);

  WCOREPOP;
}

