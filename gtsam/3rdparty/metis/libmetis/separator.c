/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * separator.c
 *
 * This file contains code for separator extraction
 *
 * Started 8/1/97
 * George
 *
 * $Id: separator.c 10481 2011-07-05 18:01:23Z karypis $
 *
 */

#include "metislib.h"

/*************************************************************************
* This function takes a bisection and constructs a minimum weight vertex 
* separator out of it. It uses the node-based separator refinement for it.
**************************************************************************/
void ConstructSeparator(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, j, k, nvtxs, nbnd;
  idx_t *xadj, *where, *bndind;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  nbnd   = graph->nbnd;
  bndind = graph->bndind;

  where = icopy(nvtxs, graph->where, iwspacemalloc(ctrl, nvtxs));

  /* Put the nodes in the boundary into the separator */
  for (i=0; i<nbnd; i++) {
    j = bndind[i];
    if (xadj[j+1]-xadj[j] > 0)  /* Ignore islands */
      where[j] = 2;
  }

  FreeRData(graph);

  Allocate2WayNodePartitionMemory(ctrl, graph);
  icopy(nvtxs, where, graph->where);

  WCOREPOP;

  ASSERT(IsSeparable(graph));

  Compute2WayNodePartitionParams(ctrl, graph);

  ASSERT(CheckNodePartitionParams(graph));

  FM_2WayNodeRefine2Sided(ctrl, graph, 1); 
  FM_2WayNodeRefine1Sided(ctrl, graph, 4); 

  ASSERT(IsSeparable(graph));

}



/*************************************************************************
* This function takes a bisection and constructs a minimum weight vertex 
* separator out of it. It uses an unweighted minimum-cover algorithm
* followed by node-based separator refinement.
**************************************************************************/
void ConstructMinCoverSeparator(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, jj, k, l, nvtxs, nbnd, bnvtxs[3], bnedges[2], csize;
  idx_t *xadj, *adjncy, *bxadj, *badjncy;
  idx_t *where, *bndind, *bndptr, *vmap, *ivmap, *cover;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  nbnd   = graph->nbnd;
  bndind = graph->bndind;
  bndptr = graph->bndptr;
  where  = graph->where;

  vmap  = iwspacemalloc(ctrl, nvtxs);
  ivmap = iwspacemalloc(ctrl, nbnd);
  cover = iwspacemalloc(ctrl, nbnd);

  if (nbnd > 0) {
    /* Go through the boundary and determine the sizes of the bipartite graph */
    bnvtxs[0] = bnvtxs[1] = bnedges[0] = bnedges[1] = 0;
    for (i=0; i<nbnd; i++) {
      j = bndind[i];
      k = where[j];
      if (xadj[j+1]-xadj[j] > 0) {
        bnvtxs[k]++;
        bnedges[k] += xadj[j+1]-xadj[j];
      }
    }

    bnvtxs[2] = bnvtxs[0]+bnvtxs[1];
    bnvtxs[1] = bnvtxs[0];
    bnvtxs[0] = 0;

    bxadj   = iwspacemalloc(ctrl, bnvtxs[2]+1);
    badjncy = iwspacemalloc(ctrl, bnedges[0]+bnedges[1]+1);

    /* Construct the ivmap and vmap */
    ASSERT(iset(nvtxs, -1, vmap) == vmap);
    for (i=0; i<nbnd; i++) {
      j = bndind[i];
      k = where[j];
      if (xadj[j+1]-xadj[j] > 0) {
        vmap[j] = bnvtxs[k];
        ivmap[bnvtxs[k]++] = j;
      }
    }

    /* OK, go through and put the vertices of each part starting from 0 */
    bnvtxs[1] = bnvtxs[0];
    bnvtxs[0] = 0;
    bxadj[0] = l = 0;
    for (k=0; k<2; k++) {
      for (ii=0; ii<nbnd; ii++) {
        i = bndind[ii];
        if (where[i] == k && xadj[i] < xadj[i+1]) {
          for (j=xadj[i]; j<xadj[i+1]; j++) {
            jj = adjncy[j];
            if (where[jj] != k) {
              ASSERT(bndptr[jj] != -1); 
              ASSERTP(vmap[jj] != -1, ("%"PRIDX" %"PRIDX" %"PRIDX"\n", jj, vmap[jj], graph->bndptr[jj]));
              badjncy[l++] = vmap[jj];
            }
          }
          bxadj[++bnvtxs[k]] = l;
        }
      }
    }

    ASSERT(l <= bnedges[0]+bnedges[1]);

    MinCover(bxadj, badjncy, bnvtxs[0], bnvtxs[1], cover, &csize);

    IFSET(ctrl->dbglvl, METIS_DBG_SEPINFO,
      printf("Nvtxs: %6"PRIDX", [%5"PRIDX" %5"PRIDX"], Cut: %6"PRIDX", SS: [%6"PRIDX" %6"PRIDX"], Cover: %6"PRIDX"\n", nvtxs, graph->pwgts[0], graph->pwgts[1], graph->mincut, bnvtxs[0], bnvtxs[1]-bnvtxs[0], csize));

    for (i=0; i<csize; i++) {
      j = ivmap[cover[i]];
      where[j] = 2;
    }
  }
  else {
    IFSET(ctrl->dbglvl, METIS_DBG_SEPINFO,
      printf("Nvtxs: %6"PRIDX", [%5"PRIDX" %5"PRIDX"], Cut: %6"PRIDX", SS: [%6"PRIDX" %6"PRIDX"], Cover: %6"PRIDX"\n", nvtxs, graph->pwgts[0], graph->pwgts[1], graph->mincut, (idx_t)0, (idx_t)0, (idx_t)0));
  }

  /* Prepare to refine the vertex separator */
  icopy(nvtxs, graph->where, vmap);

  FreeRData(graph);

  Allocate2WayNodePartitionMemory(ctrl, graph);
  icopy(nvtxs, vmap, graph->where);

  WCOREPOP;

  Compute2WayNodePartitionParams(ctrl, graph);

  ASSERT(CheckNodePartitionParams(graph));

  FM_2WayNodeRefine1Sided(ctrl, graph, ctrl->niter); 

  ASSERT(IsSeparable(graph));
}

