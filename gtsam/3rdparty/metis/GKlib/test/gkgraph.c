/*!
\file  
\brief A simple frequent itemset discovery program to test GKlib's routines

\date 6/12/2008
\author George
\version \verbatim $Id: gkgraph.c 11408 2012-01-25 15:05:58Z karypis $ \endverbatim
*/

#include <GKlib.h>

/*************************************************************************/
/*! Data structures for the code */
/*************************************************************************/
typedef struct {
  int type;
  int niter;
  float eps;
  float lamda;

  char *infile;
  char *outfile;
} params_t;

/*************************************************************************/
/*! Constants */
/*************************************************************************/
#define CMD_NITER       1
#define CMD_EPS         2
#define CMD_LAMDA       3
#define CMD_TYPE        4
#define CMD_HELP        10


/*************************************************************************/
/*! Local variables */
/*************************************************************************/
static struct gk_option long_options[] = {
  {"type",       1,      0,      CMD_TYPE},
  {"niter",      1,      0,      CMD_NITER},
  {"lamda",      1,      0,      CMD_LAMDA},
  {"eps",        1,      0,      CMD_EPS},
  {"help",       0,      0,      CMD_HELP},
  {0,            0,      0,      0}
};


/*-------------------------------------------------------------------*/
/* Mini help  */
/*-------------------------------------------------------------------*/
static char helpstr[][100] = {
" ",
"Usage: gkgraph [options] <graph-file> [<out-file>]",
" ",
" Required parameters",
"  graph-file",
"     The name of the file storing the graph. The file is in ",
"     Metis' graph format.",
" ",
" Optional parameters",
"  -niter=int",
"     Specifies the maximum number of iterations. [default: 100]",
" ",
"  -lamda=float",
"     Specifies the follow-the-adjacent-links probability. [default: 0.80]",
" ",
"  -eps=float",
"     Specifies the error tollerance. [default: 1e-10]",
" ",
"  -help",
"     Prints this message.",
""
};

static char shorthelpstr[][100] = {
" ",
"   Usage: gkgraph [options] <graph-file> [<out-file>]",
"          use 'gkgraph -help' for a summary of the options.",
""
};
 


/*************************************************************************/
/*! Function prototypes */
/*************************************************************************/
double compute_compactness(params_t *params, gk_graph_t *graph, int32_t *perm);
void reorder_centroid(params_t *params, gk_graph_t *graph, int32_t *perm);
void print_init_info(params_t *params, gk_graph_t *graph);
void print_final_info(params_t *params);
params_t *parse_cmdline(int argc, char *argv[]);


/*************************************************************************/
/*! the entry point */
/**************************************************************************/
int main(int argc, char *argv[])
{
  ssize_t i, j, v;
  params_t *params;
  gk_graph_t *graph, *pgraph;
  int32_t *perm;
 
  /* get command-line options */
  params = parse_cmdline(argc, argv);

  /* read the data */
  graph = gk_graph_Read(params->infile, GK_GRAPH_FMT_METIS, 0, 0, 0);

  /* display some basic stats */
  print_init_info(params, graph);


  /* determine the initial compactness of the graph */
  printf("Initial compactness: %le\n", compute_compactness(params, graph, NULL));

  /* compute the BFS ordering and re-order the graph */
  //for (i=0; i<params->niter; i++) {
  for (i=0; i<1; i++) {
    v = RandomInRange(graph->nvtxs);
    gk_graph_ComputeBFSOrdering(graph, v, &perm, NULL);
    printf("BFS from %8d. Compactness: %le\n", 
        (int) v, compute_compactness(params, graph, perm));

    pgraph = gk_graph_Reorder(graph, perm, NULL);
    gk_graph_Write(pgraph, "bfs.metis", GK_GRAPH_FMT_METIS);
    gk_graph_Free(&pgraph);

    gk_graph_ComputeBestFOrdering(graph, v, params->type, &perm, NULL);
    printf("BestF from %8d. Compactness: %le\n", 
        (int) v, compute_compactness(params, graph, perm));

    pgraph = gk_graph_Reorder(graph, perm, NULL);
    gk_graph_Write(pgraph, "bestf.metis", GK_GRAPH_FMT_METIS);
    gk_graph_Free(&pgraph);

#ifdef XXX
    for (j=0; j<params->niter; j++) {
      reorder_centroid(params, graph, perm);
      printf("\tAfter centroid; Compactness: %le\n", 
          compute_compactness(params, graph, perm));
    }

    pgraph = gk_graph_Reorder(graph, perm, NULL);
    gk_graph_Write(pgraph, "centroid.metis", GK_GRAPH_FMT_METIS);
    gk_graph_Free(&pgraph);
#endif
    gk_free((void **)&perm, LTERM);
  }

  gk_graph_Free(&graph);
  //gk_graph_Free(&pgraph);

  print_final_info(params);
}




/*************************************************************************/
/*! This function computes the compactness of the graph's adjacency list */
/*************************************************************************/
double compute_compactness(params_t *params, gk_graph_t *graph, int32_t *perm)
{
  int i, v, u, nvtxs;
  ssize_t j, *xadj; 
  int32_t *adjncy;
  double compactness=0.0;
  int *freq;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  freq = gk_ismalloc(nvtxs, 0, "compute_compactness: freq");

  for (i=0; i<nvtxs; i++) {
    v = (perm == NULL ? i : perm[i]);
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      u = (perm == NULL ? adjncy[j] : perm[adjncy[j]]);
      compactness += fabs(v-u);
      freq[gk_abs(v-u)]++;
    }
  }

  /*
  for (i=0; i<nvtxs; i++) {
    if (freq[i] > 0) 
      printf("%7d %6d\n", i, freq[i]);
  }
  */
  printf("\tnsmall: %d\n", freq[1]+freq[2]+freq[3]);

  return compactness/xadj[nvtxs];
}


/*************************************************************************/
/*! This function uses a centroid-based approach to refine the ordering */
/*************************************************************************/
void reorder_centroid(params_t *params, gk_graph_t *graph, int32_t *perm)
{
  int i, v, u, nvtxs;
  ssize_t j, *xadj; 
  int32_t *adjncy;
  gk_fkv_t *cand;
  double displacement;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  cand = gk_fkvmalloc(nvtxs, "reorder_centroid: cand");

  for (i=0; i<nvtxs; i++) {
    v = perm[i];
    displacement = 0.0;

    for (j=xadj[i]; j<xadj[i+1]; j++) {
      u = perm[adjncy[j]];
      displacement += u-v;
      //displacement += sign(u-v, sqrt(fabs(u-v)));
    }

    cand[i].val = i;
    cand[i].key = v + displacement*params->lamda/(xadj[i+1]-xadj[i]);
  }

  /* sort them based on the target position in increasing order */
  gk_fkvsorti(nvtxs, cand);


  /* derive the permutation from the ordered list */
  gk_i32set(nvtxs, -1, perm);
  for (i=0; i<nvtxs; i++) {
    if (perm[cand[i].val] != -1)
      errexit("Resetting perm[%d] = %d\n", cand[i].val, perm[cand[i].val]);
    perm[cand[i].val] = i;
  }

  gk_free((void **)&cand, LTERM);
}








/*************************************************************************/
/*! This function prints run parameters */
/*************************************************************************/
void print_init_info(params_t *params, gk_graph_t *graph)
{
  printf("*******************************************************************************\n");
  printf(" gkgraph\n\n");
  printf("Graph Information ----------------------------------------------------------\n");
  printf(" input file=%s, [%d, %zd]\n", 
      params->infile, graph->nvtxs, graph->xadj[graph->nvtxs]);

  printf("\n");
  printf("Options --------------------------------------------------------------------\n");
  printf(" type=%d, niter=%d, lamda=%f, eps=%e\n",
      params->type, params->niter, params->lamda, params->eps);

  printf("\n");
  printf("Working... -----------------------------------------------------------------\n");
}


/*************************************************************************/
/*! This function prints final statistics */
/*************************************************************************/
void print_final_info(params_t *params)
{
  printf("\n");
  printf("Memory Usage Information -----------------------------------------------------\n");
  printf("   Maximum memory used:              %10zd bytes\n", (ssize_t) gk_GetMaxMemoryUsed());
  printf("   Current memory used:              %10zd bytes\n", (ssize_t) gk_GetCurMemoryUsed());
  printf("********************************************************************************\n");
}


/*************************************************************************/
/*! This is the entry point of the command-line argument parser */
/*************************************************************************/
params_t *parse_cmdline(int argc, char *argv[])
{
  int i;
  int c, option_index;
  params_t *params;

  params = (params_t *)gk_malloc(sizeof(params_t), "parse_cmdline: params");

  /* initialize the params data structure */
  params->type      = 1;
  params->niter     = 1;
  params->eps       = 1e-10;
  params->lamda     = 0.20;
  params->infile    = NULL;


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
      case CMD_TYPE:
        if (gk_optarg) params->type = atoi(gk_optarg);
        break;
      case CMD_NITER:
        if (gk_optarg) params->niter = atoi(gk_optarg);
        break;
      case CMD_EPS:
        if (gk_optarg) params->eps = atof(gk_optarg);
        break;
      case CMD_LAMDA:
        if (gk_optarg) params->lamda = atof(gk_optarg);
        break;

      case CMD_HELP:
        for (i=0; strlen(helpstr[i]) > 0; i++)
          printf("%s\n", helpstr[i]);
        exit(0);
        break;
      case '?':
      default:
        printf("Illegal command-line option(s)\nUse %s -help for a summary of the options.\n", argv[0]);
        exit(0);
    }
  }

  if (argc-gk_optind != 1) {
    printf("Unrecognized parameters.");
    for (i=0; strlen(shorthelpstr[i]) > 0; i++)
      printf("%s\n", shorthelpstr[i]);
    exit(0);
  }

  params->infile  = gk_strdup(argv[gk_optind++]);

  if (argc-gk_optind > 0) 
    params->outfile = gk_strdup(argv[gk_optind++]);
  else
    params->outfile   = gk_strdup("gkgraph.out");

  if (!gk_fexists(params->infile))
    errexit("input file %s does not exist.\n", params->infile);

  return params;
}

