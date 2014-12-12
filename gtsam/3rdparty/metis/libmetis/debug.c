/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * debug.c
 *
 * This file contains code that performs self debuging
 *
 * Started 7/24/97
 * George
 *
 */

#include "metislib.h"



/*************************************************************************/
/*! This function computes the total edgecut 
 */
/*************************************************************************/
idx_t ComputeCut(graph_t *graph, idx_t *where)
{
  idx_t i, j, cut;

  if (graph->adjwgt == NULL) {
    for (cut=0, i=0; i<graph->nvtxs; i++) {
      for (j=graph->xadj[i]; j<graph->xadj[i+1]; j++)
        if (where[i] != where[graph->adjncy[j]])
          cut++;
    }
  }
  else {
    for (cut=0, i=0; i<graph->nvtxs; i++) {
      for (j=graph->xadj[i]; j<graph->xadj[i+1]; j++)
        if (where[i] != where[graph->adjncy[j]])
          cut += graph->adjwgt[j];
    }
  }

  return cut/2;
}


/*************************************************************************/
/*! This function computes the total volume 
 */
/*************************************************************************/
idx_t ComputeVolume(graph_t *graph, idx_t *where)
{
  idx_t i, j, k, me, nvtxs, nparts, totalv;
  idx_t *xadj, *adjncy, *vsize, *marker;


  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vsize  = graph->vsize;

  nparts = where[iargmax(nvtxs, where)]+1;
  marker = ismalloc(nparts, -1, "ComputeVolume: marker");

  totalv = 0;

  for (i=0; i<nvtxs; i++) {
    marker[where[i]] = i;
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = where[adjncy[j]];
      if (marker[k] != i) {
        marker[k] = i;
        totalv += (vsize ? vsize[i] : 1);
      }
    }
  }

  gk_free((void **)&marker, LTERM);

  return totalv;
}


/*************************************************************************/
/*! This function computes the cut given the graph and a where vector 
 */
/*************************************************************************/
idx_t ComputeMaxCut(graph_t *graph, idx_t nparts, idx_t *where)
{
  idx_t i, j, maxcut;
  idx_t *cuts;

  cuts = ismalloc(nparts, 0, "ComputeMaxCut: cuts");

  if (graph->adjwgt == NULL) {
    for (i=0; i<graph->nvtxs; i++) {
      for (j=graph->xadj[i]; j<graph->xadj[i+1]; j++)
        if (where[i] != where[graph->adjncy[j]]) 
          cuts[where[i]]++;
    }
  }
  else {
    for (i=0; i<graph->nvtxs; i++) {
      for (j=graph->xadj[i]; j<graph->xadj[i+1]; j++)
        if (where[i] != where[graph->adjncy[j]])
          cuts[where[i]] += graph->adjwgt[j];
    }
  }

  maxcut = cuts[iargmax(nparts, cuts)];

  printf("%zu => %"PRIDX"\n", iargmax(nparts, cuts), maxcut);

  gk_free((void **)&cuts, LTERM);

  return maxcut;
}


/*************************************************************************/
/*! This function checks whether or not the boundary information is correct 
 */
/*************************************************************************/
idx_t CheckBnd(graph_t *graph) 
{
  idx_t i, j, nvtxs, nbnd;
  idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

  nvtxs = graph->nvtxs;
  xadj = graph->xadj;
  adjncy = graph->adjncy;
  where = graph->where;
  bndptr = graph->bndptr;
  bndind = graph->bndind;

  for (nbnd=0, i=0; i<nvtxs; i++) {
    if (xadj[i+1]-xadj[i] == 0)
      nbnd++;   /* Islands are considered to be boundary vertices */

    for (j=xadj[i]; j<xadj[i+1]; j++) {
      if (where[i] != where[adjncy[j]]) {
        nbnd++;
        ASSERT(bndptr[i] != -1);
        ASSERT(bndind[bndptr[i]] == i);
        break;
      }
    }
  }

  ASSERTP(nbnd == graph->nbnd, ("%"PRIDX" %"PRIDX"\n", nbnd, graph->nbnd));

  return 1;
}



/*************************************************************************/
/*! This function checks whether or not the boundary information is correct 
 */
/*************************************************************************/
idx_t CheckBnd2(graph_t *graph) 
{
  idx_t i, j, nvtxs, nbnd, id, ed;
  idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  where  = graph->where;
  bndptr = graph->bndptr;
  bndind = graph->bndind;

  for (nbnd=0, i=0; i<nvtxs; i++) {
    id = ed = 0;
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      if (where[i] != where[adjncy[j]]) 
        ed += graph->adjwgt[j];
      else
        id += graph->adjwgt[j];
    }
    if (ed - id >= 0 && xadj[i] < xadj[i+1]) {
      nbnd++;
      ASSERTP(bndptr[i] != -1, ("%"PRIDX" %"PRIDX" %"PRIDX"\n", i, id, ed));
      ASSERT(bndind[bndptr[i]] == i);
    }
  }

  ASSERTP(nbnd == graph->nbnd, ("%"PRIDX" %"PRIDX"\n", nbnd, graph->nbnd));

  return 1;
}


/*************************************************************************/
/*! This function checks whether or not the boundary information is correct 
 */
/*************************************************************************/
idx_t CheckNodeBnd(graph_t *graph, idx_t onbnd) 
{
  idx_t i, j, nvtxs, nbnd;
  idx_t *xadj, *adjncy, *where, *bndptr, *bndind;

  nvtxs = graph->nvtxs;
  xadj = graph->xadj;
  adjncy = graph->adjncy;
  where = graph->where;
  bndptr = graph->bndptr;
  bndind = graph->bndind;

  for (nbnd=0, i=0; i<nvtxs; i++) {
    if (where[i] == 2) 
      nbnd++;   
  }

  ASSERTP(nbnd == onbnd, ("%"PRIDX" %"PRIDX"\n", nbnd, onbnd));

  for (i=0; i<nvtxs; i++) {
    if (where[i] != 2) {
      ASSERTP(bndptr[i] == -1, ("%"PRIDX" %"PRIDX"\n", i, bndptr[i]));
    }
    else {
      ASSERTP(bndptr[i] != -1, ("%"PRIDX" %"PRIDX"\n", i, bndptr[i]));
    }
  }

  return 1;
}



/*************************************************************************/
/*! This function checks whether or not the rinfo of a vertex is consistent 
 */
/*************************************************************************/
idx_t CheckRInfo(ctrl_t *ctrl, ckrinfo_t *rinfo)
{
  idx_t i, j;
  cnbr_t *nbrs;

  nbrs = ctrl->cnbrpool + rinfo->inbr;

  for (i=0; i<rinfo->nnbrs; i++) {
    for (j=i+1; j<rinfo->nnbrs; j++)
      ASSERTP(nbrs[i].pid != nbrs[j].pid, 
          ("%"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"\n", 
           i, j, nbrs[i].pid, nbrs[j].pid));
  }

  return 1;
}



/*************************************************************************/
/*! This function checks the correctness of the NodeFM data structures 
 */
/*************************************************************************/
idx_t CheckNodePartitionParams(graph_t *graph)
{
  idx_t i, j, k, l, nvtxs, me, other;
  idx_t *xadj, *adjncy, *adjwgt, *vwgt, *where;
  idx_t edegrees[2], pwgts[3];

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  where  = graph->where;

  /*------------------------------------------------------------
  / Compute now the separator external degrees
  /------------------------------------------------------------*/
  pwgts[0] = pwgts[1] = pwgts[2] = 0;
  for (i=0; i<nvtxs; i++) {
    me = where[i];
    pwgts[me] += vwgt[i];

    if (me == 2) { /* If it is on the separator do some computations */
      edegrees[0] = edegrees[1] = 0;

      for (j=xadj[i]; j<xadj[i+1]; j++) {
        other = where[adjncy[j]];
        if (other != 2)
          edegrees[other] += vwgt[adjncy[j]];
      }
      if (edegrees[0] != graph->nrinfo[i].edegrees[0] || 
          edegrees[1] != graph->nrinfo[i].edegrees[1]) {
        printf("Something wrong with edegrees: %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"\n", 
            i, edegrees[0], edegrees[1], 
            graph->nrinfo[i].edegrees[0], graph->nrinfo[i].edegrees[1]);
        return 0;
      }
    }
  }

  if (pwgts[0] != graph->pwgts[0] || 
      pwgts[1] != graph->pwgts[1] || 
      pwgts[2] != graph->pwgts[2]) {
    printf("Something wrong with part-weights: %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"\n", pwgts[0], pwgts[1], pwgts[2], graph->pwgts[0], graph->pwgts[1], graph->pwgts[2]);
    return 0;
  }

  return 1;
}


/*************************************************************************/
/*! This function checks if the separator is indeed a separator 
 */
/*************************************************************************/
idx_t IsSeparable(graph_t *graph)
{
  idx_t i, j, nvtxs, other;
  idx_t *xadj, *adjncy, *where;

  nvtxs = graph->nvtxs;
  xadj = graph->xadj;
  adjncy = graph->adjncy;
  where = graph->where;

  for (i=0; i<nvtxs; i++) {
    if (where[i] == 2)
      continue;
    other = (where[i]+1)%2;
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      ASSERTP(where[adjncy[j]] != other, 
          ("%"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX" %"PRIDX"\n", 
           i, where[i], adjncy[j], where[adjncy[j]], xadj[i+1]-xadj[i], 
           xadj[adjncy[j]+1]-xadj[adjncy[j]]));
    }
  }

  return 1;
}


/*************************************************************************/
/*! This function recomputes the vrinfo fields and checks them against
    those in the graph->vrinfo structure */
/*************************************************************************/
void CheckKWayVolPartitionParams(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, k, kk, l, nvtxs, nbnd, mincut, minvol, me, other, pid;
  idx_t *xadj, *vsize, *adjncy, *pwgts, *where, *bndind, *bndptr;
  vkrinfo_t *rinfo, *myrinfo, *orinfo, tmprinfo;
  vnbr_t *mynbrs, *onbrs, *tmpnbrs;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vsize  = graph->vsize;
  adjncy = graph->adjncy;
  where  = graph->where;
  rinfo  = graph->vkrinfo;

  tmpnbrs = (vnbr_t *)wspacemalloc(ctrl, ctrl->nparts*sizeof(vnbr_t));

  /*------------------------------------------------------------
  / Compute now the iv/ev degrees
  /------------------------------------------------------------*/
  for (i=0; i<nvtxs; i++) {
    me = where[i];

    myrinfo = rinfo+i;
    mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

    for (k=0; k<myrinfo->nnbrs; k++)
      tmpnbrs[k] = mynbrs[k];

    tmprinfo.nnbrs = myrinfo->nnbrs;
    tmprinfo.nid    = myrinfo->nid;
    tmprinfo.ned    = myrinfo->ned;

    myrinfo = &tmprinfo;
    mynbrs  = tmpnbrs;

    for (k=0; k<myrinfo->nnbrs; k++)
      mynbrs[k].gv = 0;

    for (j=xadj[i]; j<xadj[i+1]; j++) {
      ii     = adjncy[j];
      other  = where[ii];
      orinfo = rinfo+ii;
      onbrs  = ctrl->vnbrpool + orinfo->inbr;

      if (me == other) {
        /* Find which domains 'i' is connected and 'ii' is not and update their gain */
        for (k=0; k<myrinfo->nnbrs; k++) {
          pid = mynbrs[k].pid;
          for (kk=0; kk<orinfo->nnbrs; kk++) {
            if (onbrs[kk].pid == pid)
              break;
          }
          if (kk == orinfo->nnbrs) 
            mynbrs[k].gv -= vsize[ii];
        }
      }
      else {
        /* Find the orinfo[me].ed and see if I'm the only connection */
        for (k=0; k<orinfo->nnbrs; k++) {
          if (onbrs[k].pid == me)
            break;
        }

        if (onbrs[k].ned == 1) { /* I'm the only connection of 'ii' in 'me' */
          for (k=0; k<myrinfo->nnbrs; k++) {
            if (mynbrs[k].pid == other) {
              mynbrs[k].gv += vsize[ii];
              break;
            }
          }

          /* Increase the gains for all the common domains between 'i' and 'ii' */
          for (k=0; k<myrinfo->nnbrs; k++) {
            if ((pid = mynbrs[k].pid) == other)
              continue;
            for (kk=0; kk<orinfo->nnbrs; kk++) {
              if (onbrs[kk].pid == pid) {
                mynbrs[k].gv += vsize[ii];
                break;
              }
            }
          }

        }
        else {
          /* Find which domains 'i' is connected and 'ii' is not and update their gain */
          for (k=0; k<myrinfo->nnbrs; k++) {
            if ((pid = mynbrs[k].pid) == other)
              continue;
            for (kk=0; kk<orinfo->nnbrs; kk++) {
              if (onbrs[kk].pid == pid)
                break;
            }
            if (kk == orinfo->nnbrs) 
              mynbrs[k].gv -= vsize[ii];
          }
        }
      }
    }

    myrinfo = rinfo+i;
    mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

    for (k=0; k<myrinfo->nnbrs; k++) {
      pid = mynbrs[k].pid;
      for (kk=0; kk<tmprinfo.nnbrs; kk++) {
        if (tmpnbrs[kk].pid == pid) {
          if (tmpnbrs[kk].gv != mynbrs[k].gv)
            printf("[%8"PRIDX" %8"PRIDX" %8"PRIDX" %+8"PRIDX" %+8"PRIDX"]\n", 
                i, where[i], pid, mynbrs[k].gv, tmpnbrs[kk].gv);
          break;
        }
      }
    }

  }

  WCOREPOP;
}


