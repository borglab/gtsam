/*!
\file  gklib.c
\brief Functions for printing various statistics for the computed partitionings
       and orderings.

\date   Started 7/25/1997
\author George  
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version\verbatim $Id: stat.c 10046 2011-06-01 14:13:40Z karypis $ \endverbatim
*/



#include "metisbin.h"


/****************************************************************************/
/*! This function computes various information associated with a partition */
/****************************************************************************/
void ComputePartitionInfo(params_t *params, graph_t *graph, idx_t *where)
{
  idx_t i, ii, j, k, nvtxs, ncon, nparts, tvwgt;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *kpwgts;
  real_t *tpwgts, unbalance;
  idx_t pid, ndom, maxndom, minndom, tndom, *pptr, *pind, *pdom;
  idx_t ncmps, nover, *cptr, *cind, *cpwgts;

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;
  adjwgt = graph->adjwgt;

  nparts = params->nparts;
  tpwgts = params->tpwgts;

  /* Compute objective-related infomration */
  printf(" - Edgecut: %"PRIDX", communication volume: %"PRIDX".\n\n", 
      ComputeCut(graph, where), ComputeVolume(graph, where));


  /* Compute constraint-related information */
  kpwgts = ismalloc(ncon*nparts, 0, "ComputePartitionInfo: kpwgts");

  for (i=0; i<nvtxs; i++) {
    for (j=0; j<ncon; j++) 
      kpwgts[where[i]*ncon+j] += vwgt[i*ncon+j];
  }

  /* Report on balance */
  printf(" - Balance:\n");
  for (j=0; j<ncon; j++) {
    tvwgt = isum(nparts, kpwgts+j, ncon);
    for (k=0, unbalance=1.0*kpwgts[k*ncon+j]/(tpwgts[k*ncon+j]*tvwgt), i=1; i<nparts; i++) {
      if (unbalance < 1.0*kpwgts[i*ncon+j]/(tpwgts[i*ncon+j]*tvwgt)) {
        unbalance = 1.0*kpwgts[i*ncon+j]/(tpwgts[i*ncon+j]*tvwgt);
        k = i;
      }
    }
    printf("     constraint #%"PRIDX":  %5.3"PRREAL" out of %5.3"PRREAL"\n", 
        j, unbalance,
         1.0*nparts*vwgt[ncon*iargmax_strd(nvtxs, vwgt+j, ncon)+j]/
            (1.0*isum(nparts, kpwgts+j, ncon)));
  }
  printf("\n");

  if (ncon == 1) {
    tvwgt = isum(nparts, kpwgts, 1);
    for (k=0, unbalance=kpwgts[k]/(tpwgts[k]*tvwgt), i=1; i<nparts; i++) {
      if (unbalance < kpwgts[i]/(tpwgts[i]*tvwgt)) {
        unbalance = kpwgts[i]/(tpwgts[i]*tvwgt);
        k = i;
      }
    }

    printf(" - Most overweight partition:\n"
           "     pid: %"PRIDX", actual: %"PRIDX", desired: %"PRIDX", ratio: %.2"PRREAL".\n\n",
        k, kpwgts[k], (idx_t)(tvwgt*tpwgts[k]), unbalance);
  }

  gk_free((void **)&kpwgts, LTERM);


  /* Compute subdomain adjacency information */
  pptr = imalloc(nparts+1, "ComputePartitionInfo: pptr");
  pind = imalloc(nvtxs, "ComputePartitionInfo: pind");
  pdom = imalloc(nparts, "ComputePartitionInfo: pdom");

  iarray2csr(nvtxs, nparts, where, pptr, pind);

  maxndom = nparts+1;
  minndom = 0;
  for (tndom=0, pid=0; pid<nparts; pid++) {
    iset(nparts, 0, pdom);
    for (ii=pptr[pid]; ii<pptr[pid+1]; ii++) {
      i = pind[ii];
      for (j=xadj[i]; j<xadj[i+1]; j++)
        pdom[where[adjncy[j]]] += adjwgt[j];
    }
    pdom[pid] = 0;
    for (ndom=0, i=0; i<nparts; i++)
      ndom += (pdom[i] > 0 ? 1 : 0);
    tndom += ndom;
    if (pid == 0 || maxndom < ndom)
      maxndom = ndom;
    if (pid == 0 || minndom > ndom)
      minndom = ndom;
  }

  printf(" - Subdomain connectivity: max: %"PRIDX", min: %"PRIDX", avg: %.2"PRREAL"\n\n",
      maxndom, minndom, 1.0*tndom/nparts);
      
  gk_free((void **)&pptr, &pind, &pdom, LTERM);


  /* Compute subdomain adjacency information */
  cptr   = imalloc(nvtxs+1, "ComputePartitionInfo: cptr");
  cind   = imalloc(nvtxs, "ComputePartitionInfo: cind");
  cpwgts = ismalloc(nparts, 0, "ComputePartitionInfo: cpwgts");

  ncmps = FindPartitionInducedComponents(graph, where, cptr, cind);
  if (ncmps == nparts)
    printf(" - Each partition is contiguous.\n");
  else {
    if (IsConnected(graph, 0)) {
      for (nover=0, i=0; i<ncmps; i++) {
        cpwgts[where[cind[cptr[i]]]]++;
        if (cpwgts[where[cind[cptr[i]]]] == 2)
          nover++;
      }
      printf(" - There are %"PRIDX" non-contiguous partitions.\n"
             "   Total components after removing the cut edges: %"PRIDX",\n"
             "   max components: %"PRIDX" for pid: %"PRIDX".\n",
          nover, ncmps, imax(nparts, cpwgts), (idx_t)iargmax(nparts, cpwgts));
    }
    else {
      printf(" - The original graph had %"PRIDX" connected components and the resulting\n"
             "   partitioning after removing the cut edges has %"PRIDX" components.",
         FindPartitionInducedComponents(graph, NULL, NULL, NULL), ncmps);
    }
  }

  gk_free((void **)&cptr, &cind, &cpwgts, LTERM);
             
}


