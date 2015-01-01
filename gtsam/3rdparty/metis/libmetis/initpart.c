/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * initpart.c
 *
 * This file contains code that performs the initial partition of the
 * coarsest graph
 *
 * Started 7/23/97
 * George
 *
 */

#include "metislib.h"

/*************************************************************************/
/*! This function computes the initial bisection of the coarsest graph */
/*************************************************************************/
void Init2WayPartition(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts) 
{
  mdbglvl_et dbglvl;

  ASSERT(graph->tvwgt[0] >= 0);

  dbglvl = ctrl->dbglvl;
  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, ctrl->dbglvl -= METIS_DBG_REFINE);
  IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, ctrl->dbglvl -= METIS_DBG_MOVEINFO);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));

  switch (ctrl->iptype) {
    case METIS_IPTYPE_RANDOM:
      if (graph->ncon == 1)
        RandomBisection(ctrl, graph, ntpwgts, niparts);
      else
        McRandomBisection(ctrl, graph, ntpwgts, niparts);
      break;

    case METIS_IPTYPE_GROW:
      if (graph->nedges == 0)
        if (graph->ncon == 1)
          RandomBisection(ctrl, graph, ntpwgts, niparts);
        else
          McRandomBisection(ctrl, graph, ntpwgts, niparts);
      else
        if (graph->ncon == 1)
          GrowBisection(ctrl, graph, ntpwgts, niparts);
        else
          McGrowBisection(ctrl, graph, ntpwgts, niparts);
      break;

    default:
      gk_errexit(SIGERR, "Unknown initial partition type: %d\n", ctrl->iptype);
  }

  IFSET(ctrl->dbglvl, METIS_DBG_IPART, printf("Initial Cut: %"PRIDX"\n", graph->mincut));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->InitPartTmr));
  ctrl->dbglvl = dbglvl;

}


/*************************************************************************/
/*! This function computes the initial separator of the coarsest graph */
/*************************************************************************/
void InitSeparator(ctrl_t *ctrl, graph_t *graph, idx_t niparts) 
{
  real_t ntpwgts[2] = {0.5, 0.5};
  mdbglvl_et dbglvl;

  dbglvl = ctrl->dbglvl;
  IFSET(ctrl->dbglvl, METIS_DBG_REFINE, ctrl->dbglvl -= METIS_DBG_REFINE);
  IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, ctrl->dbglvl -= METIS_DBG_MOVEINFO);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->InitPartTmr));

  /* this is required for the cut-based part of the refinement */
  Setup2WayBalMultipliers(ctrl, graph, ntpwgts);

  switch (ctrl->iptype) {
    case METIS_IPTYPE_EDGE:
      if (graph->nedges == 0)
        RandomBisection(ctrl, graph, ntpwgts, niparts);
      else
        GrowBisection(ctrl, graph, ntpwgts, niparts);

      Compute2WayPartitionParams(ctrl, graph);
      ConstructSeparator(ctrl, graph);
      break;

    case METIS_IPTYPE_NODE:
      GrowBisectionNode(ctrl, graph, ntpwgts, niparts);
      break;

    default:
      gk_errexit(SIGERR, "Unkown iptype of %"PRIDX"\n", ctrl->iptype);
  }

  IFSET(ctrl->dbglvl, METIS_DBG_IPART, printf("Initial Sep: %"PRIDX"\n", graph->mincut));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->InitPartTmr));

  ctrl->dbglvl = dbglvl;

}


/*************************************************************************/
/*! This function computes a bisection of a graph by randomly assigning
    the vertices followed by a bisection refinement.
    The resulting partition is returned in graph->where.
*/
/*************************************************************************/
void RandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, ii, j, k, nvtxs, pwgts[2], zeromaxpwgt, from, me, 
        bestcut=0, icut, mincut, inbfs;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where;
  idx_t *perm, *bestwhere;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  Allocate2WayPartitionMemory(ctrl, graph);
  where = graph->where;

  bestwhere = iwspacemalloc(ctrl, nvtxs);
  perm      = iwspacemalloc(ctrl, nvtxs);

  zeromaxpwgt = ctrl->ubfactors[0]*graph->tvwgt[0]*ntpwgts[0];

  for (inbfs=0; inbfs<niparts; inbfs++) {
    iset(nvtxs, 1, where);

    if (inbfs > 0) {
      irandArrayPermute(nvtxs, perm, nvtxs/2, 1);
      pwgts[1] = graph->tvwgt[0];
      pwgts[0] = 0;

      for (ii=0; ii<nvtxs; ii++) {
        i = perm[ii];
        if (pwgts[0]+vwgt[i] < zeromaxpwgt) {
          where[i] = 0;
          pwgts[0] += vwgt[i];
          pwgts[1] -= vwgt[i];
          if (pwgts[0] > zeromaxpwgt)
            break;
        }
      }
    }

    /* Do some partition refinement  */
    Compute2WayPartitionParams(ctrl, graph);
    /* printf("IPART: %3"PRIDX" [%5"PRIDX" %5"PRIDX"] [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", graph->nvtxs, pwgts[0], pwgts[1], graph->pwgts[0], graph->pwgts[1], graph->mincut); */

    Balance2Way(ctrl, graph, ntpwgts);
    /* printf("BPART: [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", graph->pwgts[0], graph->pwgts[1], graph->mincut); */

    FM_2WayRefine(ctrl, graph, ntpwgts, 4);
    /* printf("RPART: [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", graph->pwgts[0], graph->pwgts[1], graph->mincut); */

    if (inbfs==0 || bestcut > graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
      if (bestcut == 0)
        break;
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}


/*************************************************************************/
/*! This function takes a graph and produces a bisection by using a region
    growing algorithm. The resulting bisection is refined using FM.
    The resulting partition is returned in graph->where.
*/
/*************************************************************************/
void GrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, j, k, nvtxs, drain, nleft, first, last, 
        pwgts[2], oneminpwgt, onemaxpwgt, 
        from, me, bestcut=0, icut, mincut, inbfs;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where;
  idx_t *queue, *touched, *gain, *bestwhere;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  Allocate2WayPartitionMemory(ctrl, graph);
  where = graph->where;

  bestwhere = iwspacemalloc(ctrl, nvtxs);
  queue     = iwspacemalloc(ctrl, nvtxs);
  touched   = iwspacemalloc(ctrl, nvtxs);

  onemaxpwgt = ctrl->ubfactors[0]*graph->tvwgt[0]*ntpwgts[1];
  oneminpwgt = (1.0/ctrl->ubfactors[0])*graph->tvwgt[0]*ntpwgts[1];

  for (inbfs=0; inbfs<niparts; inbfs++) {
    iset(nvtxs, 1, where);

    iset(nvtxs, 0, touched);

    pwgts[1] = graph->tvwgt[0];
    pwgts[0] = 0;


    queue[0] = irandInRange(nvtxs);
    touched[queue[0]] = 1;
    first = 0; 
    last  = 1;
    nleft = nvtxs-1;
    drain = 0;

    /* Start the BFS from queue to get a partition */
    for (;;) {
      if (first == last) { /* Empty. Disconnected graph! */
        if (nleft == 0 || drain)
          break;

        k = irandInRange(nleft);
        for (i=0; i<nvtxs; i++) {
          if (touched[i] == 0) {
            if (k == 0)
              break;
            else
              k--;
          }
        }

        queue[0]   = i;
        touched[i] = 1;
        first      = 0; 
        last       = 1;
        nleft--;
      }

      i = queue[first++];
      if (pwgts[0] > 0 && pwgts[1]-vwgt[i] < oneminpwgt) {
        drain = 1;
        continue;
      }

      where[i] = 0;
      INC_DEC(pwgts[0], pwgts[1], vwgt[i]);
      if (pwgts[1] <= onemaxpwgt)
        break;

      drain = 0;
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        k = adjncy[j];
        if (touched[k] == 0) {
          queue[last++] = k;
          touched[k] = 1;
          nleft--;
        }
      }
    }

    /* Check to see if we hit any bad limiting cases */
    if (pwgts[1] == 0) 
      where[irandInRange(nvtxs)] = 1;
    if (pwgts[0] == 0) 
      where[irandInRange(nvtxs)] = 0;

    /*************************************************************
    * Do some partition refinement 
    **************************************************************/
    Compute2WayPartitionParams(ctrl, graph);
    /*
    printf("IPART: %3"PRIDX" [%5"PRIDX" %5"PRIDX"] [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", 
        graph->nvtxs, pwgts[0], pwgts[1], graph->pwgts[0], graph->pwgts[1], graph->mincut); 
    */

    Balance2Way(ctrl, graph, ntpwgts);
    /*
    printf("BPART: [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", graph->pwgts[0],
        graph->pwgts[1], graph->mincut); 
    */

    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
    /*
    printf("RPART: [%5"PRIDX" %5"PRIDX"] %5"PRIDX"\n", graph->pwgts[0], 
        graph->pwgts[1], graph->mincut);
    */

    if (inbfs == 0 || bestcut > graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
      if (bestcut == 0)
        break;
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}


/*************************************************************************/
/*! This function takes a multi-constraint graph and computes a bisection 
    by randomly assigning the vertices and then refining it. The resulting
    partition is returned in graph->where.
*/
/**************************************************************************/
void McRandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, ii, j, k, nvtxs, ncon, from, bestcut=0, mincut, inbfs, qnum;
  idx_t *bestwhere, *where, *perm, *counts;
  idx_t *vwgt;

  WCOREPUSH;

  nvtxs = graph->nvtxs;
  ncon  = graph->ncon;
  vwgt  = graph->vwgt;

  Allocate2WayPartitionMemory(ctrl, graph);
  where = graph->where;

  bestwhere = iwspacemalloc(ctrl, nvtxs);
  perm      = iwspacemalloc(ctrl, nvtxs);
  counts    = iwspacemalloc(ctrl, ncon);

  for (inbfs=0; inbfs<2*niparts; inbfs++) {
    irandArrayPermute(nvtxs, perm, nvtxs/2, 1);
    iset(ncon, 0, counts);

    /* partition by spliting the queues randomly */
    for (ii=0; ii<nvtxs; ii++) {
      i        = perm[ii];
      qnum     = iargmax(ncon, vwgt+i*ncon);
      where[i] = (counts[qnum]++)%2;
    }

    Compute2WayPartitionParams(ctrl, graph);

    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
    Balance2Way(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
    Balance2Way(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

    if (inbfs == 0 || bestcut >= graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
      if (bestcut == 0)
        break;
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}


/*************************************************************************/
/*! This function takes a multi-constraint graph and produces a bisection 
    by using a region growing algorithm. The resulting partition is 
    returned in graph->where.
*/
/*************************************************************************/
void McGrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, j, k, nvtxs, ncon, from, bestcut=0, mincut, inbfs;
  idx_t *bestwhere, *where;

  WCOREPUSH;

  nvtxs = graph->nvtxs;

  Allocate2WayPartitionMemory(ctrl, graph);
  where = graph->where;

  bestwhere = iwspacemalloc(ctrl, nvtxs);

  for (inbfs=0; inbfs<2*niparts; inbfs++) {
    iset(nvtxs, 1, where);
    where[irandInRange(nvtxs)] = 0;

    Compute2WayPartitionParams(ctrl, graph);

    Balance2Way(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);
    Balance2Way(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

    if (inbfs == 0 || bestcut >= graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
      if (bestcut == 0)
        break;
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}


/*************************************************************************/
/* This function takes a graph and produces a tri-section into left, right,
   and separator using a region growing algorithm. The resulting separator
   is refined using node FM.
   The resulting partition is returned in graph->where.
*/
/**************************************************************************/
void GrowBisectionNode(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, j, k, nvtxs, drain, nleft, first, last, pwgts[2], oneminpwgt, 
        onemaxpwgt, from, me, bestcut=0, icut, mincut, inbfs;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *where, *bndind;
  idx_t *queue, *touched, *gain, *bestwhere;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  bestwhere = iwspacemalloc(ctrl, nvtxs);
  queue     = iwspacemalloc(ctrl, nvtxs);
  touched   = iwspacemalloc(ctrl, nvtxs);

  onemaxpwgt = ctrl->ubfactors[0]*graph->tvwgt[0]*0.5;
  oneminpwgt = (1.0/ctrl->ubfactors[0])*graph->tvwgt[0]*0.5;


  /* Allocate refinement memory. Allocate sufficient memory for both edge and node */
  graph->pwgts  = imalloc(3, "GrowBisectionNode: pwgts");
  graph->where  = imalloc(nvtxs, "GrowBisectionNode: where");
  graph->bndptr = imalloc(nvtxs, "GrowBisectionNode: bndptr");
  graph->bndind = imalloc(nvtxs, "GrowBisectionNode: bndind");
  graph->id     = imalloc(nvtxs, "GrowBisectionNode: id");
  graph->ed     = imalloc(nvtxs, "GrowBisectionNode: ed");
  graph->nrinfo = (nrinfo_t *)gk_malloc(nvtxs*sizeof(nrinfo_t), "GrowBisectionNode: nrinfo");
  
  where  = graph->where;
  bndind = graph->bndind;

  for (inbfs=0; inbfs<niparts; inbfs++) {
    iset(nvtxs, 1, where);
    iset(nvtxs, 0, touched);

    pwgts[1] = graph->tvwgt[0];
    pwgts[0] = 0;

    queue[0] = irandInRange(nvtxs);
    touched[queue[0]] = 1;
    first = 0; last = 1;
    nleft = nvtxs-1;
    drain = 0;

    /* Start the BFS from queue to get a partition */
    for (;;) {
      if (first == last) { /* Empty. Disconnected graph! */
        if (nleft == 0 || drain)
          break;
  
        k = irandInRange(nleft);
        for (i=0; i<nvtxs; i++) { /* select the kth untouched vertex */
          if (touched[i] == 0) {
            if (k == 0)
              break;
            else
              k--;
          }
        }

        queue[0]   = i;
        touched[i] = 1;
        first      = 0; 
        last       = 1;
        nleft--;
      }

      i = queue[first++];
      if (pwgts[1]-vwgt[i] < oneminpwgt) {
        drain = 1;
        continue;
      }

      where[i] = 0;
      INC_DEC(pwgts[0], pwgts[1], vwgt[i]);
      if (pwgts[1] <= onemaxpwgt)
        break;

      drain = 0;
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        k = adjncy[j];
        if (touched[k] == 0) {
          queue[last++] = k;
          touched[k] = 1;
          nleft--;
        }
      }
    }

    /*************************************************************
    * Do some partition refinement 
    **************************************************************/
    Compute2WayPartitionParams(ctrl, graph);
    Balance2Way(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, 4);

    /* Construct and refine the vertex separator */
    for (i=0; i<graph->nbnd; i++) {
      j = bndind[i];
      if (xadj[j+1]-xadj[j] > 0) /* ignore islands */
        where[j] = 2;
    }

    Compute2WayNodePartitionParams(ctrl, graph); 
    FM_2WayNodeRefine2Sided(ctrl, graph, 1);
    FM_2WayNodeRefine1Sided(ctrl, graph, 4);

    /*
    printf("ISep: [%"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"] %"PRIDX"\n", 
        inbfs, graph->pwgts[0], graph->pwgts[1], graph->pwgts[2], bestcut); 
    */
    
    if (inbfs == 0 || bestcut > graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}


/*************************************************************************/
/* This function takes a graph and produces a tri-section into left, right,
   and separator using a region growing algorithm. The resulting separator
   is refined using node FM.
   The resulting partition is returned in graph->where.
*/
/**************************************************************************/
void GrowBisectionNode2(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         idx_t niparts)
{
  idx_t i, j, k, nvtxs, bestcut=0, mincut, inbfs;
  idx_t *xadj, *where, *bndind, *bestwhere;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;

  /* Allocate refinement memory. Allocate sufficient memory for both edge and node */
  graph->pwgts  = imalloc(3, "GrowBisectionNode: pwgts");
  graph->where  = imalloc(nvtxs, "GrowBisectionNode: where");
  graph->bndptr = imalloc(nvtxs, "GrowBisectionNode: bndptr");
  graph->bndind = imalloc(nvtxs, "GrowBisectionNode: bndind");
  graph->id     = imalloc(nvtxs, "GrowBisectionNode: id");
  graph->ed     = imalloc(nvtxs, "GrowBisectionNode: ed");
  graph->nrinfo = (nrinfo_t *)gk_malloc(nvtxs*sizeof(nrinfo_t), "GrowBisectionNode: nrinfo");
  
  bestwhere = iwspacemalloc(ctrl, nvtxs);

  where  = graph->where;
  bndind = graph->bndind;

  for (inbfs=0; inbfs<niparts; inbfs++) {
    iset(nvtxs, 1, where);
    if (inbfs > 0)
      where[irandInRange(nvtxs)] = 0;

    Compute2WayPartitionParams(ctrl, graph);
    General2WayBalance(ctrl, graph, ntpwgts);
    FM_2WayRefine(ctrl, graph, ntpwgts, ctrl->niter);

    /* Construct and refine the vertex separator */
    for (i=0; i<graph->nbnd; i++) {
      j = bndind[i];
      if (xadj[j+1]-xadj[j] > 0) /* ignore islands */
        where[j] = 2;
    }

    Compute2WayNodePartitionParams(ctrl, graph); 
    FM_2WayNodeRefine2Sided(ctrl, graph, 4);

    /*
    printf("ISep: [%"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"] %"PRIDX"\n", 
        inbfs, graph->pwgts[0], graph->pwgts[1], graph->pwgts[2], bestcut); 
    */

    if (inbfs == 0 || bestcut > graph->mincut) {
      bestcut = graph->mincut;
      icopy(nvtxs, where, bestwhere);
    }
  }

  graph->mincut = bestcut;
  icopy(nvtxs, bestwhere, where);

  WCOREPOP;
}

