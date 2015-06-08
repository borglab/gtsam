/*
 * Copyright 1994-2011, Regents of the University of Minnesota
 *
 * ndmetis.c
 *
 * Driver programs for nested disection ordering
 *
 * Started 8/28/94
 * George
 *
 * $Id: ndmetis.c 13900 2013-03-24 15:27:07Z karypis $
 *
 */

#include "metisbin.h"



/*************************************************************************/
/*! Let the game begin! */
/*************************************************************************/
int main(int argc, char *argv[])
{
  idx_t options[METIS_NOPTIONS];
  graph_t *graph;
  idx_t *perm, *iperm;
  params_t *params;
  int status=0;

  params = parse_cmdline(argc, argv);

  gk_startcputimer(params->iotimer);
  graph = ReadGraph(params);
  gk_stopcputimer(params->iotimer);

  /* This is just for internal use to clean up some files
  {
    char fileout[8192];

    gk_free((void **)&graph->vwgt, &graph->adjwgt, &graph->vsize, LTERM);
    sprintf(fileout, "ND/%s", params->filename);
    if (graph->nvtxs > 25000)
      WriteGraph(graph, fileout);
    exit(0);
  }
  */

  /* Check if the graph is contiguous */
  if (graph->ncon != 1) {
    printf("***The input graph contains %"PRIDX" constraints..\n" 
           "***Ordering requires a graph with one constraint.\n", graph->ncon);
    exit(0);
  }

  NDPrintInfo(params, graph);

  perm  = imalloc(graph->nvtxs, "main: perm");
  iperm = imalloc(graph->nvtxs, "main: iperm");

  METIS_SetDefaultOptions(options);
  options[METIS_OPTION_CTYPE]    = params->ctype;
  options[METIS_OPTION_IPTYPE]   = params->iptype;
  options[METIS_OPTION_RTYPE]    = params->rtype;
  options[METIS_OPTION_DBGLVL]   = params->dbglvl;
  options[METIS_OPTION_UFACTOR]  = params->ufactor;
  options[METIS_OPTION_NO2HOP]   = params->no2hop;
  options[METIS_OPTION_COMPRESS] = params->compress;
  options[METIS_OPTION_CCORDER]  = params->ccorder;
  options[METIS_OPTION_SEED]     = params->seed;
  options[METIS_OPTION_NITER]    = params->niter;
  options[METIS_OPTION_NSEPS]    = params->nseps;
  options[METIS_OPTION_PFACTOR]  = params->pfactor;

  gk_malloc_init();
  gk_startcputimer(params->parttimer);

  status = METIS_NodeND(&graph->nvtxs, graph->xadj, graph->adjncy, graph->vwgt, 
               options, perm, iperm);

  gk_stopcputimer(params->parttimer);
  if (gk_GetCurMemoryUsed() != 0)
    printf("***It seems that Metis did not free all of its memory! Report this.\n");
  params->maxmemory = gk_GetMaxMemoryUsed();
  gk_malloc_cleanup(0);


  if (status != METIS_OK) {
    printf("\n***Metis returned with an error.\n");
  }
  else {
    if (!params->nooutput) {
      /* Write the solution */
      gk_startcputimer(params->iotimer);
      WritePermutation(params->filename, iperm, graph->nvtxs); 
      gk_stopcputimer(params->iotimer);
    }

    NDReportResults(params, graph, perm, iperm);
  }

  FreeGraph(&graph);
  gk_free((void **)&perm, &iperm, LTERM);
  gk_free((void **)&params->filename, &params->tpwgtsfile, &params->tpwgts, 
      &params->ubvec, &params, LTERM);

}


/*************************************************************************/
/*! This function prints run parameters */
/*************************************************************************/
void NDPrintInfo(params_t *params, graph_t *graph)
{ 
  printf("******************************************************************************\n");
  printf("%s", METISTITLE);
  printf(" (HEAD: %s, Built on: %s, %s)\n", SVNINFO, __DATE__, __TIME__);
  printf(" size of idx_t: %zubits, real_t: %zubits, idx_t *: %zubits\n", 
      8*sizeof(idx_t), 8*sizeof(real_t), 8*sizeof(idx_t *));
  printf("\n");
  printf("Graph Information -----------------------------------------------------------\n");
  printf(" Name: %s, #Vertices: %"PRIDX", #Edges: %"PRIDX"\n", 
      params->filename, graph->nvtxs, graph->nedges/2);

  printf("\n");
  printf("Options ---------------------------------------------------------------------\n");
  printf(" ctype=%s, rtype=%s, iptype=%s, seed=%"PRIDX", dbglvl=%"PRIDX"\n",
      ctypenames[params->ctype], rtypenames[params->rtype], 
      iptypenames[params->iptype], params->seed, params->dbglvl);

  printf(" ufactor=%.3f, pfactor=%.2f, no2hop=%s, ccorder=%s, compress=%s, , nooutput=%s\n",
      I2RUBFACTOR(params->ufactor), 
      0.1*params->pfactor,
      (params->no2hop   ? "YES" : "NO"), 
      (params->ccorder  ? "YES" : "NO"), 
      (params->compress ? "YES" : "NO"), 
      (params->nooutput ? "YES" : "NO")
      );

  printf(" niter=%"PRIDX", nseps=%"PRIDX"\n", params->niter, params->nseps);

  printf("\n");
  printf("Node-based Nested Dissection ------------------------------------------------\n");
}


/*************************************************************************/
/*! This function does any post-ordering reporting */
/*************************************************************************/
void NDReportResults(params_t *params, graph_t *graph, idx_t *perm, 
         idx_t *iperm)
{ 
  size_t maxlnz, opc;

  gk_startcputimer(params->reporttimer);
  ComputeFillIn(graph, perm, iperm, &maxlnz, &opc);
  printf("  Nonzeros: %6.3le \tOperation Count: %6.3le\n", (double)maxlnz, (double)opc);

  gk_stopcputimer(params->reporttimer);


  printf("\nTiming Information ----------------------------------------------------------\n");
  printf("  I/O:          \t\t %7.3"PRREAL" sec\n", gk_getcputimer(params->iotimer));
  printf("  Ordering:     \t\t %7.3"PRREAL" sec   (METIS time)\n", gk_getcputimer(params->parttimer));
  printf("  Reporting:    \t\t %7.3"PRREAL" sec\n", gk_getcputimer(params->reporttimer));
  printf("\nMemory Information ----------------------------------------------------------\n");
  printf("  Max memory used:\t\t %7.3"PRREAL" MB\n", (real_t)(params->maxmemory/(1024.0*1024.0)));
  printf("******************************************************************************\n");

}
