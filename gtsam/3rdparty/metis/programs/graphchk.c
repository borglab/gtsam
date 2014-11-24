/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * graphchk.c
 *
 * This file checks the validity of a graph
 *
 * Started 8/28/94
 * George
 *
 * $Id: graphchk.c 9982 2011-05-25 17:18:00Z karypis $
 *
 */

#include "metisbin.h"



/*************************************************************************/
/*! Let entry point of the checker */
/*************************************************************************/
int main(int argc, char *argv[])
{
  graph_t *graph, *fgraph;
  char filename[256];
  idx_t wgtflag;
  params_t params;

  if (argc != 2 && argc != 3) {
    printf("Usage: %s <GraphFile> [FixedGraphFile (for storing the fixed graph)]\n", argv[0]);
    exit(0);
  }

  memset((void *)&params, 0, sizeof(params_t));
  params.filename = gk_strdup(argv[1]);
    
  graph = ReadGraph(&params);
  if (graph->nvtxs == 0) {
    printf("Empty graph!\n");
    exit(0);
  }

  printf("**********************************************************************\n");
  printf("%s", METISTITLE);
  printf(" (HEAD: %s, Built on: %s, %s)\n", SVNINFO, __DATE__, __TIME__);
  printf(" size of idx_t: %zubits, real_t: %zubits, idx_t *: %zubits\n",
      8*sizeof(idx_t), 8*sizeof(real_t), 8*sizeof(idx_t *));
  printf("\n");
  printf("Graph Information ---------------------------------------------------\n");
  printf("  Name: %s, #Vertices: %"PRIDX", #Edges: %"PRIDX"\n\n", 
      params.filename, graph->nvtxs, graph->nedges/2);
  printf("Checking Graph... ---------------------------------------------------\n");

  if (CheckGraph(graph, 1, 1)) {
    printf("   The format of the graph is correct!\n");
  }
  else {
    printf("   The format of the graph is incorrect!\n");
    if (argc == 3) {
      fgraph = FixGraph(graph);
      WriteGraph(fgraph, argv[2]);
      FreeGraph(&fgraph);
      printf("   A corrected version was stored at %s\n", argv[2]);
    }
  }

  printf("\n**********************************************************************\n");


  FreeGraph(&graph);
  gk_free((void **)&params.filename, &params.tpwgtsfile, &params.tpwgts, LTERM);
}  


