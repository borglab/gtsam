/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * stat.c
 *
 * This file computes various statistics
 *
 * Started 7/25/97
 * George
 *
 * $Id: stat.c 9942 2011-05-17 22:09:52Z karypis $
 *
 */

#include "metislib.h"


/*************************************************************************
* This function computes cuts and balance information
**************************************************************************/
void ComputePartitionInfoBipartite(graph_t *graph, idx_t nparts, idx_t *where)
{
  idx_t i, j, k, nvtxs, ncon, mustfree=0;
  idx_t *xadj, *adjncy, *vwgt, *vsize, *adjwgt, *kpwgts, *tmpptr;
  idx_t *padjncy, *padjwgt, *padjcut;

  nvtxs = graph->nvtxs;
  ncon = graph->ncon;
  xadj = graph->xadj;
  adjncy = graph->adjncy;
  vwgt = graph->vwgt;
  vsize = graph->vsize;
  adjwgt = graph->adjwgt;

  if (vwgt == NULL) {
    vwgt = graph->vwgt = ismalloc(nvtxs, 1, "vwgt");
    mustfree = 1;
  }
  if (adjwgt == NULL) {
    adjwgt = graph->adjwgt = ismalloc(xadj[nvtxs], 1, "adjwgt");
    mustfree += 2;
  }

  printf("%"PRIDX"-way Cut: %5"PRIDX", Vol: %5"PRIDX", ", nparts, ComputeCut(graph, where), ComputeVolume(graph, where));

  /* Compute balance information */
  kpwgts = ismalloc(ncon*nparts, 0, "ComputePartitionInfo: kpwgts");

  for (i=0; i<nvtxs; i++) {
    for (j=0; j<ncon; j++) 
      kpwgts[where[i]*ncon+j] += vwgt[i*ncon+j];
  }

  if (ncon == 1) {
    printf("\tBalance: %5.3"PRREAL" out of %5.3"PRREAL"\n", 
            1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1)),
            1.0*nparts*vwgt[iargmax(nvtxs, vwgt)]/(1.0*isum(nparts, kpwgts, 1)));
  }
  else {
    printf("\tBalance:");
    for (j=0; j<ncon; j++) 
      printf(" (%5.3"PRREAL" out of %5.3"PRREAL")", 
            1.0*nparts*kpwgts[ncon*iargmax_strd(nparts, kpwgts+j, ncon)+j]/(1.0*isum(nparts, kpwgts+j, ncon)),
            1.0*nparts*vwgt[ncon*iargmax_strd(nvtxs, vwgt+j, ncon)+j]/(1.0*isum(nparts, kpwgts+j, ncon)));
    printf("\n");
  }


  /* Compute p-adjncy information */
  padjncy = ismalloc(nparts*nparts, 0, "ComputePartitionInfo: padjncy");
  padjwgt = ismalloc(nparts*nparts, 0, "ComputePartitionInfo: padjwgt");
  padjcut = ismalloc(nparts*nparts, 0, "ComputePartitionInfo: padjwgt");

  iset(nparts, 0, kpwgts);
  for (i=0; i<nvtxs; i++) {
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      if (where[i] != where[adjncy[j]]) {
        padjncy[where[i]*nparts+where[adjncy[j]]] = 1;
        padjcut[where[i]*nparts+where[adjncy[j]]] += adjwgt[j];
        if (kpwgts[where[adjncy[j]]] == 0) {
          padjwgt[where[i]*nparts+where[adjncy[j]]] += vsize[i];
          kpwgts[where[adjncy[j]]] = 1;
        }
      }
    }
    for (j=xadj[i]; j<xadj[i+1]; j++) 
      kpwgts[where[adjncy[j]]] = 0;
  }

  for (i=0; i<nparts; i++)
    kpwgts[i] = isum(nparts, padjncy+i*nparts, 1);
  printf("Min/Max/Avg/Bal # of adjacent     subdomains: %5"PRIDX" %5"PRIDX" %5"PRIDX" %7.3"PRREAL"\n",
    kpwgts[iargmin(nparts, kpwgts)], kpwgts[iargmax(nparts, kpwgts)], isum(nparts, kpwgts, 1)/nparts, 
    1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1)));

  for (i=0; i<nparts; i++)
    kpwgts[i] = isum(nparts, padjcut+i*nparts, 1);
  printf("Min/Max/Avg/Bal # of adjacent subdomain cuts: %5"PRIDX" %5"PRIDX" %5"PRIDX" %7.3"PRREAL"\n",
    kpwgts[iargmin(nparts, kpwgts)], kpwgts[iargmax(nparts, kpwgts)], isum(nparts, kpwgts, 1)/nparts, 
    1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1)));

  for (i=0; i<nparts; i++)
    kpwgts[i] = isum(nparts, padjwgt+i*nparts, 1);
  printf("Min/Max/Avg/Bal/Frac # of interface    nodes: %5"PRIDX" %5"PRIDX" %5"PRIDX" %7.3"PRREAL" %7.3"PRREAL"\n",
    kpwgts[iargmin(nparts, kpwgts)], kpwgts[iargmax(nparts, kpwgts)], isum(nparts, kpwgts, 1)/nparts, 
    1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1)), 1.0*isum(nparts, kpwgts, 1)/(1.0*nvtxs));


  if (mustfree == 1 || mustfree == 3) {
    gk_free((void **)&vwgt, LTERM);
    graph->vwgt = NULL;
  }
  if (mustfree == 2 || mustfree == 3) {
    gk_free((void **)&adjwgt, LTERM);
    graph->adjwgt = NULL;
  }

  gk_free((void **)&kpwgts, &padjncy, &padjwgt, &padjcut, LTERM);
}


/*************************************************************************
* This function computes the balance of the partitioning
**************************************************************************/
void ComputePartitionBalance(graph_t *graph, idx_t nparts, idx_t *where, real_t *ubvec)
{
  idx_t i, j, nvtxs, ncon;
  idx_t *kpwgts, *vwgt;
  real_t balance;

  nvtxs = graph->nvtxs;
  ncon = graph->ncon;
  vwgt = graph->vwgt;

  kpwgts = ismalloc(nparts, 0, "ComputePartitionInfo: kpwgts");

  if (vwgt == NULL) {
    for (i=0; i<nvtxs; i++)
      kpwgts[where[i]]++;
    ubvec[0] = 1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*nvtxs);
  }
  else {
    for (j=0; j<ncon; j++) {
      iset(nparts, 0, kpwgts);
      for (i=0; i<graph->nvtxs; i++)
        kpwgts[where[i]] += vwgt[i*ncon+j];

      ubvec[j] = 1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1));
    }
  }

  gk_free((void **)&kpwgts, LTERM);

}


/*************************************************************************
* This function computes the balance of the element partitioning
**************************************************************************/
real_t ComputeElementBalance(idx_t ne, idx_t nparts, idx_t *where)
{
  idx_t i;
  idx_t *kpwgts;
  real_t balance;

  kpwgts = ismalloc(nparts, 0, "ComputeElementBalance: kpwgts");

  for (i=0; i<ne; i++)
    kpwgts[where[i]]++;

  balance = 1.0*nparts*kpwgts[iargmax(nparts, kpwgts)]/(1.0*isum(nparts, kpwgts, 1));

  gk_free((void **)&kpwgts, LTERM);

  return balance;

}


