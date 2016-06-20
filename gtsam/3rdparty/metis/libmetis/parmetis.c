/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * parmetis.c
 *
 * This file contains top level routines that are used by ParMETIS
 *
 * Started 10/14/97
 * George
 *
 * $Id: parmetis.c 10481 2011-07-05 18:01:23Z karypis $
 *
 */

#include "metislib.h"


/*************************************************************************/
/*! This function is the entry point for the node ND code for ParMETIS.
    The difference between this routine and the standard METIS_NodeND are
    the following
    
    - It performs at least log2(npes) levels of nested dissection.
    - It stores the size of the log2(npes) top-level separators in the
      sizes array.
*/
/*************************************************************************/
int METIS_NodeNDP(idx_t nvtxs, idx_t *xadj, idx_t *adjncy, idx_t *vwgt,
           idx_t npes, idx_t *options, idx_t *perm, idx_t *iperm, idx_t *sizes) 
{
  idx_t i, ii, j, l, nnvtxs=0;
  graph_t *graph;
  ctrl_t *ctrl;
  idx_t *cptr, *cind;

  ctrl = SetupCtrl(METIS_OP_OMETIS, options, 1, 3, NULL, NULL);
  if (!ctrl) return METIS_ERROR_INPUT;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, InitTimers(ctrl));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->TotalTmr));

  /* compress the graph; not that compression only happens if not prunning 
     has taken place. */
  if (ctrl->compress) {
    cptr = imalloc(nvtxs+1, "OMETIS: cptr");
    cind = imalloc(nvtxs, "OMETIS: cind");

    graph = CompressGraph(ctrl, nvtxs, xadj, adjncy, vwgt, cptr, cind);
    if (graph == NULL) {
      /* if there was no compression, cleanup the compress flag */
      gk_free((void **)&cptr, &cind, LTERM);
      ctrl->compress = 0;
    }
    else {
      nnvtxs = graph->nvtxs;
    }
  }

  /* if no compression, setup the graph in the normal way. */
  if (ctrl->compress == 0) 
    graph = SetupGraph(ctrl, nvtxs, 1, xadj, adjncy, vwgt, NULL, NULL);


  /* allocate workspace memory */
  AllocateWorkSpace(ctrl, graph);


  /* do the nested dissection ordering  */
  iset(2*npes-1, 0, sizes);
  MlevelNestedDissectionP(ctrl, graph, iperm, graph->nvtxs, npes, 0, sizes);


  /* Uncompress the ordering */
  if (ctrl->compress) { 
    /* construct perm from iperm */
    for (i=0; i<nnvtxs; i++)
      perm[iperm[i]] = i; 
    for (l=ii=0; ii<nnvtxs; ii++) {
      i = perm[ii];
      for (j=cptr[i]; j<cptr[i+1]; j++)
        iperm[cind[j]] = l++;
    }

    gk_free((void **)&cptr, &cind, LTERM);
  }


  for (i=0; i<nvtxs; i++)
    perm[iperm[i]] = i;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->TotalTmr));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, PrintTimers(ctrl));

  /* clean up */
  FreeCtrl(&ctrl);

  return METIS_OK;
}


/*************************************************************************/
/*! This function is similar to MlevelNestedDissection with the difference
    that it also records separator sizes for the top log2(npes) levels */
/**************************************************************************/
void MlevelNestedDissectionP(ctrl_t *ctrl, graph_t *graph, idx_t *order, 
         idx_t lastvtx, idx_t npes, idx_t cpos, idx_t *sizes)
{
  idx_t i, j, nvtxs, nbnd;
  idx_t *label, *bndind;
  graph_t *lgraph, *rgraph;

  nvtxs = graph->nvtxs;

  if (nvtxs == 0) {
    FreeGraph(&graph);
    return;
  }

  MlevelNodeBisectionMultiple(ctrl, graph);

  IFSET(ctrl->dbglvl, METIS_DBG_SEPINFO, 
      printf("Nvtxs: %6"PRIDX", [%6"PRIDX" %6"PRIDX" %6"PRIDX"]\n", 
        graph->nvtxs, graph->pwgts[0], graph->pwgts[1], graph->pwgts[2]));

  if (cpos < npes-1) {
    sizes[2*npes-2-cpos]       = graph->pwgts[2];
    sizes[2*npes-2-(2*cpos+1)] = graph->pwgts[1];
    sizes[2*npes-2-(2*cpos+2)] = graph->pwgts[0];
  }

  /* Order the nodes in the separator */
  nbnd   = graph->nbnd;
  bndind = graph->bndind;
  label  = graph->label;
  for (i=0; i<nbnd; i++) 
    order[label[bndind[i]]] = --lastvtx;

  SplitGraphOrder(ctrl, graph, &lgraph, &rgraph);

  /* Free the memory of the top level graph */
  FreeGraph(&graph);

  if ((lgraph->nvtxs > MMDSWITCH || 2*cpos+2 < npes-1) && lgraph->nedges > 0) 
    MlevelNestedDissectionP(ctrl, lgraph, order, lastvtx-rgraph->nvtxs, npes, 2*cpos+2, sizes);
  else {
    MMDOrder(ctrl, lgraph, order, lastvtx-rgraph->nvtxs); 
    FreeGraph(&lgraph);
  }
  if ((rgraph->nvtxs > MMDSWITCH || 2*cpos+1 < npes-1) && rgraph->nedges > 0) 
    MlevelNestedDissectionP(ctrl, rgraph, order, lastvtx, npes, 2*cpos+1, sizes);
  else {
    MMDOrder(ctrl, rgraph, order, lastvtx); 
    FreeGraph(&rgraph);
  }
}


/*************************************************************************/
/*! This function bisects a graph by computing a vertex separator */
/**************************************************************************/
int METIS_ComputeVertexSeparator(idx_t *nvtxs, idx_t *xadj, idx_t *adjncy, 
           idx_t *vwgt, idx_t *options, idx_t *r_sepsize, idx_t *part) 
{
  idx_t i, j;
  graph_t *graph;
  ctrl_t *ctrl;

  if ((ctrl = SetupCtrl(METIS_OP_OMETIS, options, 1, 3, NULL, NULL)) == NULL)
    return METIS_ERROR_INPUT;

  InitRandom(ctrl->seed);

  graph = SetupGraph(ctrl, *nvtxs, 1, xadj, adjncy, vwgt, NULL, NULL);

  AllocateWorkSpace(ctrl, graph);

  /*============================================================
   * Perform the bisection
   *============================================================*/ 
  ctrl->CoarsenTo = 100;

  MlevelNodeBisectionMultiple(ctrl, graph);

  *r_sepsize = graph->pwgts[2];
  icopy(*nvtxs, graph->where, part);

  FreeGraph(&graph);

  FreeCtrl(&ctrl);

  return METIS_OK;
}


/*************************************************************************/
/*! This function is the entry point of a node-based separator refinement
    of the nodes with an hmarker[] of 0. */
/*************************************************************************/
int METIS_NodeRefine(idx_t nvtxs, idx_t *xadj, idx_t *vwgt, idx_t *adjncy, 
           idx_t *where, idx_t *hmarker, real_t ubfactor)
{
  graph_t *graph;
  ctrl_t *ctrl;

  /* set up the run time parameters */
  ctrl = SetupCtrl(METIS_OP_OMETIS, NULL, 1, 3, NULL, NULL);
  if (!ctrl) return METIS_ERROR_INPUT;

  /* set up the graph */
  graph = SetupGraph(ctrl, nvtxs, 1, xadj, adjncy, vwgt, NULL, NULL);

  /* allocate workspace memory */
  AllocateWorkSpace(ctrl, graph);

  /* set up the memory and the input partition */
  Allocate2WayNodePartitionMemory(ctrl, graph);
  icopy(nvtxs, where, graph->where);

  Compute2WayNodePartitionParams(ctrl, graph);

  FM_2WayNodeRefine1SidedP(ctrl, graph, hmarker, ubfactor, 10); 
  /* FM_2WayNodeRefine2SidedP(ctrl, graph, hmarker, ubfactor, 10); */

  icopy(nvtxs, graph->where, where);

  FreeGraph(&graph);
  FreeCtrl(&ctrl);

  return METIS_OK;
}


/*************************************************************************/
/*! This function performs a node-based 1-sided FM refinement that moves
    only nodes whose hmarker[] == -1. It is used by Parmetis. */
/*************************************************************************/
void FM_2WayNodeRefine1SidedP(ctrl_t *ctrl, graph_t *graph, 
          idx_t *hmarker, real_t ubfactor, idx_t npasses)
{
  idx_t i, ii, j, k, jj, kk, nvtxs, nbnd, nswaps, nmind, nbad, qsize;
  idx_t *xadj, *vwgt, *adjncy, *where, *pwgts, *edegrees, *bndind, *bndptr;
  idx_t *mptr, *mind, *swaps, *inqueue;
  rpq_t *queue; 
  nrinfo_t *rinfo;
  idx_t higain, oldgain, mincut, initcut, mincutorder;	
  idx_t pass, from, to, limit;
  idx_t badmaxpwgt, mindiff, newdiff;

  WCOREPUSH;

  ASSERT(graph->mincut == graph->pwgts[2]);

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;

  bndind = graph->bndind;
  bndptr = graph->bndptr;
  where  = graph->where;
  pwgts  = graph->pwgts;
  rinfo  = graph->nrinfo;

  queue = rpqCreate(nvtxs);
      
  inqueue = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
  swaps   = iwspacemalloc(ctrl, nvtxs);
  mptr    = iwspacemalloc(ctrl, nvtxs+1);
  mind    = iwspacemalloc(ctrl, 2*nvtxs);

  badmaxpwgt = (idx_t)(ubfactor*gk_max(pwgts[0], pwgts[1]));

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE,
    printf("Partitions-N1: [%6"PRIDX" %6"PRIDX"] Nv-Nb[%6"PRIDX" %6"PRIDX"] "
           "MaxPwgt[%6"PRIDX"]. ISep: %6"PRIDX"\n", 
           pwgts[0], pwgts[1], graph->nvtxs, graph->nbnd, badmaxpwgt, 
           graph->mincut));

  to = (pwgts[0] < pwgts[1] ? 1 : 0);
  for (pass=0; pass<npasses; pass++) {
    from = to; 
    to   = (from+1)%2;

    rpqReset(queue);

    mincutorder = -1;
    initcut = mincut = graph->mincut;
    nbnd = graph->nbnd;

    /* use the swaps array in place of the traditional perm array to save memory */
    irandArrayPermute(nbnd, swaps, nbnd, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = bndind[swaps[ii]];
      ASSERT(where[i] == 2);
      if (hmarker[i] == -1 || hmarker[i] == to) {
        rpqInsert(queue, i, vwgt[i]-rinfo[i].edegrees[from]);
        inqueue[i] = pass;
      }
    }
    qsize = rpqLength(queue);

    ASSERT(CheckNodeBnd(graph, nbnd));
    ASSERT(CheckNodePartitionParams(graph));

    limit = nbnd;

    /******************************************************
    * Get into the FM loop
    *******************************************************/
    mptr[0] = nmind = nbad = 0;
    mindiff = abs(pwgts[0]-pwgts[1]);
    for (nswaps=0; nswaps<nvtxs; nswaps++) {
      if ((higain = rpqGetTop(queue)) == -1) 
        break;

      ASSERT(bndptr[higain] != -1);

      /* The following check is to ensure we break out if there is a posibility
         of over-running the mind array.  */
      if (nmind + xadj[higain+1]-xadj[higain] >= 2*nvtxs-1)
        break;

      inqueue[higain] = -1;

      if (pwgts[to]+vwgt[higain] > badmaxpwgt) { /* Skip this vertex */
        if (nbad++ > limit) 
          break; 
        else {
          nswaps--;
          continue;  
        }
      }

      pwgts[2] -= (vwgt[higain]-rinfo[higain].edegrees[from]);

      newdiff = abs(pwgts[to]+vwgt[higain] - (pwgts[from]-rinfo[higain].edegrees[from]));
      if (pwgts[2] < mincut || (pwgts[2] == mincut && newdiff < mindiff)) {
        mincut      = pwgts[2];
        mincutorder = nswaps;
        mindiff     = newdiff;
        nbad        = 0;
      }
      else {
        if (nbad++ > limit) {
          pwgts[2] += (vwgt[higain]-rinfo[higain].edegrees[from]);
          break; /* No further improvement, break out */
        }
      }

      BNDDelete(nbnd, bndind, bndptr, higain);
      pwgts[to] += vwgt[higain];
      where[higain] = to;
      swaps[nswaps] = higain;  


      /**********************************************************
      * Update the degrees of the affected nodes
      ***********************************************************/
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];
        if (where[k] == 2) { /* For the in-separator vertices modify their edegree[to] */
          rinfo[k].edegrees[to] += vwgt[higain];
        }
        else if (where[k] == from) { /* This vertex is pulled into the separator */
          ASSERTP(bndptr[k] == -1, ("%"PRIDX" %"PRIDX" %"PRIDX"\n", k, bndptr[k], where[k]));
          BNDInsert(nbnd, bndind, bndptr, k);

          mind[nmind++] = k;  /* Keep track for rollback */
          where[k]      = 2;
          pwgts[from]  -= vwgt[k];

          edegrees = rinfo[k].edegrees;
          edegrees[0] = edegrees[1] = 0;
          for (jj=xadj[k]; jj<xadj[k+1]; jj++) {
            kk = adjncy[jj];
            if (where[kk] != 2) 
              edegrees[where[kk]] += vwgt[kk];
            else {
              oldgain = vwgt[kk]-rinfo[kk].edegrees[from];
              rinfo[kk].edegrees[from] -= vwgt[k];

              /* Update the gain of this node if it was not skipped */
              if (inqueue[kk] == pass)
                rpqUpdate(queue, kk, oldgain+vwgt[k]); 
            }
          }

          /* Insert the new vertex into the priority queue. Safe due to one-sided moves */
          if (hmarker[k] == -1 || hmarker[k] == to) {
            rpqInsert(queue, k, vwgt[k]-edegrees[from]);
            inqueue[k] = pass;
          }
        }
      }
      mptr[nswaps+1] = nmind;


      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO,
            printf("Moved %6"PRIDX" to %3"PRIDX", Gain: %5"PRIDX" [%5"PRIDX"] \t[%5"PRIDX" %5"PRIDX" %5"PRIDX"] [%3"PRIDX" %2"PRIDX"]\n", 
                   higain, to, (vwgt[higain]-rinfo[higain].edegrees[from]), 
                   vwgt[higain], pwgts[0], pwgts[1], pwgts[2], nswaps, limit));

    }


    /****************************************************************
    * Roll back computation 
    *****************************************************************/
    for (nswaps--; nswaps>mincutorder; nswaps--) {
      higain = swaps[nswaps];

      ASSERT(CheckNodePartitionParams(graph));
      ASSERT(where[higain] == to);

      INC_DEC(pwgts[2], pwgts[to], vwgt[higain]);
      where[higain] = 2;
      BNDInsert(nbnd, bndind, bndptr, higain);

      edegrees = rinfo[higain].edegrees;
      edegrees[0] = edegrees[1] = 0;
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];
        if (where[k] == 2) 
          rinfo[k].edegrees[to] -= vwgt[higain];
        else
          edegrees[where[k]] += vwgt[k];
      }

      /* Push nodes out of the separator */
      for (j=mptr[nswaps]; j<mptr[nswaps+1]; j++) {
        k = mind[j];
        ASSERT(where[k] == 2);
        where[k] = from;
        INC_DEC(pwgts[from], pwgts[2], vwgt[k]);
        BNDDelete(nbnd, bndind, bndptr, k);
        for (jj=xadj[k]; jj<xadj[k+1]; jj++) {
          kk = adjncy[jj];
          if (where[kk] == 2) 
            rinfo[kk].edegrees[from] += vwgt[k];
        }
      }
    }

    ASSERT(mincut == pwgts[2]);

    IFSET(ctrl->dbglvl, METIS_DBG_REFINE,
      printf("\tMinimum sep: %6"PRIDX" at %5"PRIDX", PWGTS: [%6"PRIDX" %6"PRIDX"], NBND: %6"PRIDX", QSIZE: %6"PRIDX"\n", 
          mincut, mincutorder, pwgts[0], pwgts[1], nbnd, qsize));

    graph->mincut = mincut;
    graph->nbnd   = nbnd;

    if (pass%2 == 1 && (mincutorder == -1 || mincut >= initcut))
      break;
  }

  rpqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************/
/*! This function performs a node-based (two-sided) FM refinement that 
    moves only nodes whose hmarker[] == -1. It is used by Parmetis. */
/*************************************************************************/
void FM_2WayNodeRefine2SidedP(ctrl_t *ctrl, graph_t *graph, 
          idx_t *hmarker, real_t ubfactor, idx_t npasses)
{
  idx_t i, ii, j, k, jj, kk, nvtxs, nbnd, nswaps, nmind;
  idx_t *xadj, *vwgt, *adjncy, *where, *pwgts, *edegrees, *bndind, *bndptr;
  idx_t *mptr, *mind, *moved, *swaps;
  rpq_t *queues[2]; 
  nrinfo_t *rinfo;
  idx_t higain, oldgain, mincut, initcut, mincutorder;	
  idx_t pass, to, other, limit;
  idx_t badmaxpwgt, mindiff, newdiff;
  idx_t u[2], g[2];

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;

  bndind = graph->bndind;
  bndptr = graph->bndptr;
  where  = graph->where;
  pwgts  = graph->pwgts;
  rinfo  = graph->nrinfo;

  queues[0] = rpqCreate(nvtxs);
  queues[1] = rpqCreate(nvtxs);

  moved = iwspacemalloc(ctrl, nvtxs);
  swaps = iwspacemalloc(ctrl, nvtxs);
  mptr  = iwspacemalloc(ctrl, nvtxs+1);
  mind  = iwspacemalloc(ctrl, 2*nvtxs);

  IFSET(ctrl->dbglvl, METIS_DBG_REFINE,
    printf("Partitions: [%6"PRIDX" %6"PRIDX"] Nv-Nb[%6"PRIDX" %6"PRIDX"]. ISep: %6"PRIDX"\n", pwgts[0], pwgts[1], graph->nvtxs, graph->nbnd, graph->mincut));

  badmaxpwgt = (idx_t)(ubfactor*gk_max(pwgts[0], pwgts[1]));

  for (pass=0; pass<npasses; pass++) {
    iset(nvtxs, -1, moved);
    rpqReset(queues[0]);
    rpqReset(queues[1]);

    mincutorder = -1;
    initcut = mincut = graph->mincut;
    nbnd = graph->nbnd;

    /* use the swaps array in place of the traditional perm array to save memory */
    irandArrayPermute(nbnd, swaps, nbnd, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = bndind[swaps[ii]];
      ASSERT(where[i] == 2);
      if (hmarker[i] == -1) {
        rpqInsert(queues[0], i, vwgt[i]-rinfo[i].edegrees[1]);
        rpqInsert(queues[1], i, vwgt[i]-rinfo[i].edegrees[0]);
        moved[i] = -5;
      }
      else if (hmarker[i] != 2) {
        rpqInsert(queues[hmarker[i]], i, vwgt[i]-rinfo[i].edegrees[(hmarker[i]+1)%2]);
        moved[i] = -(10+hmarker[i]);
      }
    }

    ASSERT(CheckNodeBnd(graph, nbnd));
    ASSERT(CheckNodePartitionParams(graph));

    limit = nbnd;

    /******************************************************
    * Get into the FM loop
    *******************************************************/
    mptr[0] = nmind = 0;
    mindiff = abs(pwgts[0]-pwgts[1]);
    to = (pwgts[0] < pwgts[1] ? 0 : 1);
    for (nswaps=0; nswaps<nvtxs; nswaps++) {
      u[0] = rpqSeeTopVal(queues[0]);  
      u[1] = rpqSeeTopVal(queues[1]);
      if (u[0] != -1 && u[1] != -1) {
        g[0] = vwgt[u[0]]-rinfo[u[0]].edegrees[1];
        g[1] = vwgt[u[1]]-rinfo[u[1]].edegrees[0];

        to = (g[0] > g[1] ? 0 : (g[0] < g[1] ? 1 : pass%2)); 

        if (pwgts[to]+vwgt[u[to]] > badmaxpwgt) 
          to = (to+1)%2;
      }
      else if (u[0] == -1 && u[1] == -1) {
        break;
      }
      else if (u[0] != -1 && pwgts[0]+vwgt[u[0]] <= badmaxpwgt) {
        to = 0;
      }
      else if (u[1] != -1 && pwgts[1]+vwgt[u[1]] <= badmaxpwgt) {
        to = 1;
      }
      else
        break;

      other = (to+1)%2;

      higain = rpqGetTop(queues[to]);

      /* Delete its matching entry in the other queue */
      if (moved[higain] == -5) 
        rpqDelete(queues[other], higain);

      ASSERT(bndptr[higain] != -1);

      /* The following check is to ensure we break out if there is a posibility
         of over-running the mind array.  */
      if (nmind + xadj[higain+1]-xadj[higain] >= 2*nvtxs-1)
        break;

      pwgts[2] -= (vwgt[higain]-rinfo[higain].edegrees[other]);

      newdiff = abs(pwgts[to]+vwgt[higain] - (pwgts[other]-rinfo[higain].edegrees[other]));
      if (pwgts[2] < mincut || (pwgts[2] == mincut && newdiff < mindiff)) {
        mincut      = pwgts[2];
        mincutorder = nswaps;
        mindiff     = newdiff;
      }
      else {
        if (nswaps - mincutorder > limit) {
          pwgts[2] += (vwgt[higain]-rinfo[higain].edegrees[other]);
          break; /* No further improvement, break out */
        }
      }

      BNDDelete(nbnd, bndind, bndptr, higain);
      pwgts[to] += vwgt[higain];
      where[higain] = to;
      moved[higain] = nswaps;
      swaps[nswaps] = higain;  


      /**********************************************************
      * Update the degrees of the affected nodes
      ***********************************************************/
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];
        if (where[k] == 2) { /* For the in-separator vertices modify their edegree[to] */
          oldgain = vwgt[k]-rinfo[k].edegrees[to];
          rinfo[k].edegrees[to] += vwgt[higain];
          if (moved[k] == -5 || moved[k] == -(10+other)) 
            rpqUpdate(queues[other], k, oldgain-vwgt[higain]);
        }
        else if (where[k] == other) { /* This vertex is pulled into the separator */
          ASSERTP(bndptr[k] == -1, ("%"PRIDX" %"PRIDX" %"PRIDX"\n", k, bndptr[k], where[k]));
          BNDInsert(nbnd, bndind, bndptr, k);

          mind[nmind++] = k;  /* Keep track for rollback */
          where[k] = 2;
          pwgts[other] -= vwgt[k];

          edegrees = rinfo[k].edegrees;
          edegrees[0] = edegrees[1] = 0;
          for (jj=xadj[k]; jj<xadj[k+1]; jj++) {
            kk = adjncy[jj];
            if (where[kk] != 2) 
              edegrees[where[kk]] += vwgt[kk];
            else {
              oldgain = vwgt[kk]-rinfo[kk].edegrees[other];
              rinfo[kk].edegrees[other] -= vwgt[k];
              if (moved[kk] == -5 || moved[kk] == -(10+to))
                rpqUpdate(queues[to], kk, oldgain+vwgt[k]);
            }
          }

          /* Insert the new vertex into the priority queue (if it has not been moved). */
          if (moved[k] == -1 && (hmarker[k] == -1 || hmarker[k] == to)) {
            rpqInsert(queues[to], k, vwgt[k]-edegrees[other]);
            moved[k] = -(10+to);
          }
#ifdef FULLMOVES  /* this does not work as well as the above partial one */
          if (moved[k] == -1) {
            if (hmarker[k] == -1) {
              rpqInsert(queues[0], k, vwgt[k]-edegrees[1]);
              rpqInsert(queues[1], k, vwgt[k]-edegrees[0]);
              moved[k] = -5;
            }
            else if (hmarker[k] != 2) {
              rpqInsert(queues[hmarker[k]], k, vwgt[k]-edegrees[(hmarker[k]+1)%2]);
              moved[k] = -(10+hmarker[k]);
            }
          }
#endif
        }
      }
      mptr[nswaps+1] = nmind;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO,
            printf("Moved %6"PRIDX" to %3"PRIDX", Gain: %5"PRIDX" [%5"PRIDX"] "
                   "[%4"PRIDX" %4"PRIDX"] \t[%5"PRIDX" %5"PRIDX" %5"PRIDX"]\n", 
                   higain, to, g[to], g[other], vwgt[u[to]], vwgt[u[other]], 
                   pwgts[0], pwgts[1], pwgts[2]));

    }


    /****************************************************************
    * Roll back computation 
    *****************************************************************/
    for (nswaps--; nswaps>mincutorder; nswaps--) {
      higain = swaps[nswaps];

      ASSERT(CheckNodePartitionParams(graph));

      to = where[higain];
      other = (to+1)%2;
      INC_DEC(pwgts[2], pwgts[to], vwgt[higain]);
      where[higain] = 2;
      BNDInsert(nbnd, bndind, bndptr, higain);

      edegrees = rinfo[higain].edegrees;
      edegrees[0] = edegrees[1] = 0;
      for (j=xadj[higain]; j<xadj[higain+1]; j++) {
        k = adjncy[j];
        if (where[k] == 2) 
          rinfo[k].edegrees[to] -= vwgt[higain];
        else
          edegrees[where[k]] += vwgt[k];
      }

      /* Push nodes out of the separator */
      for (j=mptr[nswaps]; j<mptr[nswaps+1]; j++) {
        k = mind[j];
        ASSERT(where[k] == 2);
        where[k] = other;
        INC_DEC(pwgts[other], pwgts[2], vwgt[k]);
        BNDDelete(nbnd, bndind, bndptr, k);
        for (jj=xadj[k]; jj<xadj[k+1]; jj++) {
          kk = adjncy[jj];
          if (where[kk] == 2) 
            rinfo[kk].edegrees[other] += vwgt[k];
        }
      }
    }

    ASSERT(mincut == pwgts[2]);

    IFSET(ctrl->dbglvl, METIS_DBG_REFINE,
      printf("\tMinimum sep: %6"PRIDX" at %5"PRIDX", PWGTS: [%6"PRIDX" %6"PRIDX"], NBND: %6"PRIDX"\n", mincut, mincutorder, pwgts[0], pwgts[1], nbnd));

    graph->mincut = mincut;
    graph->nbnd = nbnd;

    if (mincutorder == -1 || mincut >= initcut)
      break;
  }

  rpqDestroy(queues[0]);
  rpqDestroy(queues[1]);

  WCOREPOP;
}

