/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * cmpfillin.c
 *
 * This file takes a graph and a fill-reducing ordering and computes
 * the fillin.
 *
 * Started 9/1/2004
 * George
 *
 * $Id: cmpfillin.c 9982 2011-05-25 17:18:00Z karypis $
 *
 */

#include "metisbin.h"



/*************************************************************************
* Let the game begin
**************************************************************************/
int main(int argc, char *argv[])
{
  idx_t i;
  idx_t *perm, *iperm;
  graph_t *graph;
  params_t params;
  size_t maxlnz, opc;

  if (argc != 3) {
    printf("Usage: %s <GraphFile> <PermFile\n", argv[0]);
    exit(0);
  }
    
  params.filename = gk_strdup(argv[1]);
  graph = ReadGraph(&params);
  if (graph->nvtxs <= 0) {
    printf("Empty graph. Nothing to do.\n");
    exit(0);
  }
  if (graph->ncon != 1) {
    printf("Ordering can only be applied to graphs with one constraint.\n");
    exit(0);
  }


  /* Read the external iperm vector */
  perm  = imalloc(graph->nvtxs, "main: perm");
  iperm = imalloc(graph->nvtxs, "main: iperm");
  ReadPOVector(graph, argv[2], iperm);

  for (i=0; i<graph->nvtxs; i++)
    perm[iperm[i]] = i;

  printf("**********************************************************************\n");
  printf("%s", METISTITLE);
  printf("Graph Information ---------------------------------------------------\n");
  printf("  Name: %s, #Vertices: %"PRIDX", #Edges: %"PRIDX"\n\n", argv[1], 
      graph->nvtxs, graph->nedges/2);
  printf("Fillin... -----------------------------------------------------------\n");

  ComputeFillIn(graph, perm, iperm, &maxlnz, &opc);
  
  printf("  Nonzeros: %6.3le \tOperation Count: %6.3le\n", (double)maxlnz, (double)opc);


  printf("**********************************************************************\n");

  FreeGraph(&graph);
}  


