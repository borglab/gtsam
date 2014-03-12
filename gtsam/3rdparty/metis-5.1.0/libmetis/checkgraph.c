/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * checkgraph.c
 *
 * This file contains routines related to I/O
 *
 * Started 8/28/94
 * George
 *
 */

#include "metislib.h"



/*************************************************************************/
/*! This function checks if a graph is valid. A valid graph must satisfy 
    the following constraints:
    - It should contain no self-edges.
    - It should be undirected; i.e., (u,v) and (v,u) should be present.
    - The adjacency list should not contain multiple edges to the same
      other vertex.

    \param graph is the graph to be checked, whose numbering starts from 0.
    \param numflag is 0 if error reporting will be done using 0 as the
           numbering, or 1 if the reporting should be done using 1.
    \param verbose is 1 the identified errors will be displayed, or 0, if
           it should run silently.
*/
/*************************************************************************/
int CheckGraph(graph_t *graph, int numflag, int verbose)
{
  idx_t i, j, k, l;
  idx_t nvtxs, err=0;
  idx_t minedge, maxedge, minewgt, maxewgt;
  idx_t *xadj, *adjncy, *adjwgt, *htable;

  numflag = (numflag == 0 ? 0 : 1);  /* make sure that numflag is 0 or 1 */

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  ASSERT(adjwgt != NULL);

  htable = ismalloc(nvtxs, 0, "htable");

  minedge = maxedge = adjncy[0];
  minewgt = maxewgt = adjwgt[0];

  for (i=0; i<nvtxs; i++) {
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];

      minedge = (k < minedge) ? k : minedge;
      maxedge = (k > maxedge) ? k : maxedge;
      minewgt = (adjwgt[j] < minewgt) ? adjwgt[j] : minewgt;
      maxewgt = (adjwgt[j] > maxewgt) ? adjwgt[j] : maxewgt;

      if (i == k) {
        if (verbose)
          printf("Vertex %"PRIDX" contains a self-loop "
                 "(i.e., diagonal entry in the matrix)!\n", i+numflag);
        err++;
      }
      else {
        for (l=xadj[k]; l<xadj[k+1]; l++) {
          if (adjncy[l] == i) {
            if (adjwgt[l] != adjwgt[j]) {
              if (verbose) 
                printf("Edges (u:%"PRIDX" v:%"PRIDX" wgt:%"PRIDX") and "
                       "(v:%"PRIDX" u:%"PRIDX" wgt:%"PRIDX") "
                       "do not have the same weight!\n", 
                       i+numflag, k+numflag, adjwgt[j],
                       k+numflag, i+numflag, adjwgt[l]);
              err++;
            }
            break;
          }
        }
        if (l == xadj[k+1]) {
          if (verbose)
            printf("Missing edge: (%"PRIDX" %"PRIDX")!\n", k+numflag, i+numflag);
          err++;
        }
      }

      if (htable[k] == 0) {
        htable[k]++;
      }
      else {
        if (verbose)
          printf("Edge %"PRIDX" from vertex %"PRIDX" is repeated %"PRIDX" times\n", 
              k+numflag, i+numflag, htable[k]++);
        err++;
      }
    }

    for (j=xadj[i]; j<xadj[i+1]; j++) 
      htable[adjncy[j]] = 0;
  }

 
  if (err > 0 && verbose) { 
    printf("A total of %"PRIDX" errors exist in the input file. "
           "Correct them, and run again!\n", err);
  }

  gk_free((void **)&htable, LTERM);

  return (err == 0 ? 1 : 0);
}


/*************************************************************************/
/*! This function performs a quick check of the weights of the graph */
/*************************************************************************/
int CheckInputGraphWeights(idx_t nvtxs, idx_t ncon, idx_t *xadj, idx_t *adjncy, 
        idx_t *vwgt, idx_t *vsize, idx_t *adjwgt) 
{
  idx_t i;

  if (ncon <= 0) {
    printf("Input Error: ncon must be >= 1.\n");
    return 0;
  }

  if (vwgt) {
    for (i=ncon*nvtxs; i>=0; i--) {
      if (vwgt[i] < 0) {
        printf("Input Error: negative vertex weight(s).\n");
        return 0;
      }
    }
  }
  if (vsize) {
    for (i=nvtxs; i>=0; i--) {
      if (vsize[i] < 0) {
        printf("Input Error: negative vertex sizes(s).\n");
        return 0;
      }
    }
  }
  if (adjwgt) {
    for (i=xadj[nvtxs]-1; i>=0; i--) {
      if (adjwgt[i] < 0) {
        printf("Input Error: non-positive edge weight(s).\n");
        return 0;
      }
    }
  }

  return 1;
}


/*************************************************************************/
/*! This function creates a graph whose topology is consistent with 
    Metis' requirements that:
    - There are no self-edges.
    - It is undirected; i.e., (u,v) and (v,u) should be present and of the
      same weight.
    - The adjacency list should not contain multiple edges to the same
      other vertex.

    Any of the above errors are fixed by performing the following operations:
    - Self-edges are removed.
    - The undirected graph is formed by the union of edges.
    - One of the duplicate edges is selected.

    The routine does not change the provided vertex weights.
*/
/*************************************************************************/
graph_t *FixGraph(graph_t *graph)
{
  idx_t i, j, k, l, nvtxs, nedges;
  idx_t *xadj, *adjncy, *adjwgt;
  idx_t *nxadj, *nadjncy, *nadjwgt;
  graph_t *ngraph;
  uvw_t *edges;


  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  ASSERT(adjwgt != NULL);

  ngraph = CreateGraph();

  ngraph->nvtxs = nvtxs;

  /* deal with vertex weights/sizes */
  ngraph->ncon  = graph->ncon;
  ngraph->vwgt  = icopy(nvtxs*graph->ncon, graph->vwgt, 
                        imalloc(nvtxs*graph->ncon, "FixGraph: vwgt"));

  ngraph->vsize = ismalloc(nvtxs, 1, "FixGraph: vsize");
  if (graph->vsize)
    icopy(nvtxs, graph->vsize, ngraph->vsize);

  /* fix graph by sorting the "superset" of edges */
  edges = (uvw_t *)gk_malloc(sizeof(uvw_t)*2*xadj[nvtxs], "FixGraph: edges");

  for (nedges=0, i=0; i<nvtxs; i++) {
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      /* keep only the upper-trianglular part of the adjacency matrix */
      if (i < adjncy[j]) {
        edges[nedges].u = i;
        edges[nedges].v = adjncy[j];
        edges[nedges].w = adjwgt[j];
        nedges++;
      }
      else if (i > adjncy[j]) {
        edges[nedges].u = adjncy[j];
        edges[nedges].v = i;
        edges[nedges].w = adjwgt[j];
        nedges++;
      }
    }
  }

  uvwsorti(nedges, edges);


  /* keep the unique subset */
  for (k=0, i=1; i<nedges; i++) {
    if (edges[k].v != edges[i].v || edges[k].u != edges[i].u) {
      edges[++k] = edges[i];
    }
  }
  nedges = k+1;

  /* allocate memory for the fixed graph */
  nxadj   = ngraph->xadj   = ismalloc(nvtxs+1, 0, "FixGraph: nxadj");
  nadjncy = ngraph->adjncy = imalloc(2*nedges, "FixGraph: nadjncy");
  nadjwgt = ngraph->adjwgt = imalloc(2*nedges, "FixGraph: nadjwgt");

  /* create the adjacency list of the fixed graph from the upper-triangular
     part of the adjacency matrix */
  for (k=0; k<nedges; k++) {
    nxadj[edges[k].u]++;
    nxadj[edges[k].v]++;
  }
  MAKECSR(i, nvtxs, nxadj);

  for (k=0; k<nedges; k++) {
    nadjncy[nxadj[edges[k].u]] = edges[k].v;
    nadjncy[nxadj[edges[k].v]] = edges[k].u;
    nadjwgt[nxadj[edges[k].u]] = edges[k].w;
    nadjwgt[nxadj[edges[k].v]] = edges[k].w;
    nxadj[edges[k].u]++;
    nxadj[edges[k].v]++;
  }
  SHIFTCSR(i, nvtxs, nxadj);

  gk_free((void **)&edges, LTERM);

  return ngraph;
}

