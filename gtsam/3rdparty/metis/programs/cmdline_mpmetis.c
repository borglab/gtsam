/*!
\file cmdline_mpmetis.c

\brief Command-line argument parsing for mpmetis

\date 12/24/2008
\author George
\version\verbatim $Id: cmdline_mpmetis.c 13905 2013-03-25 13:21:20Z karypis $\endverbatim
*/

#include "metisbin.h"


/*-------------------------------------------------------------------
 * Command-line options 
 *-------------------------------------------------------------------*/
static struct gk_option long_options[] = {
  {"gtype",          1,      0,      METIS_OPTION_GTYPE},
  {"ptype",          1,      0,      METIS_OPTION_PTYPE},
  {"objtype",        1,      0,      METIS_OPTION_OBJTYPE},

  {"ctype",          1,      0,      METIS_OPTION_CTYPE},
  {"iptype",         1,      0,      METIS_OPTION_IPTYPE},

  {"minconn",        0,      0,      METIS_OPTION_MINCONN},
  {"contig",         0,      0,      METIS_OPTION_CONTIG},

  {"nooutput",       0,      0,      METIS_OPTION_NOOUTPUT},

  {"ufactor",        1,      0,      METIS_OPTION_UFACTOR},
  {"niter",          1,      0,      METIS_OPTION_NITER},
  {"ncuts",          1,      0,      METIS_OPTION_NCUTS},
  {"ncommon",        1,      0,      METIS_OPTION_NCOMMON},

  {"tpwgts",         1,      0,      METIS_OPTION_TPWGTS},

  {"seed",           1,      0,      METIS_OPTION_SEED},

  {"dbglvl",         1,      0,      METIS_OPTION_DBGLVL},

  {"help",           0,      0,      METIS_OPTION_HELP},
  {0,                0,      0,      0}
};



/*-------------------------------------------------------------------
 * Mappings for the various parameter values
 *-------------------------------------------------------------------*/
static gk_StringMap_t gtype_options[] = {
 {"dual",               METIS_GTYPE_DUAL},
 {"nodal",              METIS_GTYPE_NODAL},
 {NULL,                 0}
};

static gk_StringMap_t ptype_options[] = {
 {"rb",                 METIS_PTYPE_RB},
 {"kway",               METIS_PTYPE_KWAY},
 {NULL,                 0}
};

static gk_StringMap_t objtype_options[] = {
 {"cut",                METIS_OBJTYPE_CUT},
 {"vol",                METIS_OBJTYPE_VOL},
 {NULL,                 0}
};

static gk_StringMap_t ctype_options[] = {
 {"rm",                 METIS_CTYPE_RM},
 {"shem",               METIS_CTYPE_SHEM},
 {NULL,                 0}
};

static gk_StringMap_t iptype_options[] = {
 {"grow",               METIS_IPTYPE_GROW},
 {"random",             METIS_IPTYPE_RANDOM},
 {NULL,                 0}
};


/*-------------------------------------------------------------------
 * Mini help
 *-------------------------------------------------------------------*/
static char helpstr[][100] =
{
" ",
"Usage: mpmetis [options] meshfile nparts",
" ",
" Required parameters",
"    meshfile    Stores the mesh to be partitioned.",
"    nparts      The number of partitions to split the mesh.",
" ",
" Optional parameters",
"  -gtype=string",
"     Specifies the graph to be used for computing the partitioning",
"     The possible values are:",
"        dual     - Partition the dual graph of the mesh [default]",
"        nodal    - Partition the nodal graph of the mesh",
" ",
"  -ptype=string",
"     Specifies the scheme to be used for computing the k-way partitioning.",
"     The possible values are:",
"        rb       - Recursive bisectioning",
"        kway     - Direct k-way partitioning [default]",
" ",
"  -ctype=string",
"     Specifies the scheme to be used to match the vertices of the graph",
"     during the coarsening.",
"     The possible values are:",
"        rm       - Random matching",
"        shem     - Sorted heavy-edge matching [default]",
" ",
"  -iptype=string [applies only when -ptype=rb]",
"     Specifies the scheme to be used to compute the initial partitioning",
"     of the graph.",
"     The possible values are:",
"        grow     - Grow a bisection using a greedy strategy [default]",
"        random   - Compute a bisection at random",
" ",
"  -objtype=string [applies only when -ptype=kway]",
"     Specifies the objective that the partitioning routines will optimize.",
"     The possible values are:",
"        cut      - Minimize the edgecut [default]",
"        vol      - Minimize the total communication volume",
" ",
"  -contig [applies only when -ptype=kway]",
"     Specifies that the partitioning routines should try to produce",
"     partitions that are contiguous. Note that if the input graph is not",
"     connected this option is ignored.",
" ",
"  -minconn [applies only when -ptype=kway]",
"     Specifies that the partitioning routines should try to minimize the",
"     maximum degree of the subdomain graph, i.e., the graph in which each",
"     partition is a node, and edges connect subdomains with a shared",
"     interface.",
" ",
"  -tpwgts=filename",
"     Specifies the name of the file that stores the target weights for",
"     each partition. By default, all partitions are assumed to be of ",
"     the same size.",
" ",
"  -ufactor=int",
"     Specifies the maximum allowed load imbalance among the partitions.",
"     A value of x indicates that the allowed load imbalance is 1+x/1000.",
"     For ptype=rb, the load imbalance is measured as the ratio of the ",
"     2*max(left,right)/(left+right), where left and right are the sizes",
"     of the respective partitions at each bisection. ",
"     For ptype=kway, the load imbalance is measured as the ratio of ",
"     max_i(pwgts[i])/avgpwgt, where pwgts[i] is the weight of the ith",
"     partition and avgpwgt is the sum of the total vertex weights divided",
"     by the number of partitions requested.",
"     For ptype=rb, the default value is 1 (i.e., load imbalance of 1.001).",
"     For ptype=kway, the default value is 30 (i.e., load imbalance of 1.03).",
" ",
"  -ncommon=int",
"     Specifies the common number of nodes that two elements must have",
"     in order to put an edge between them in the dual graph. Default is 1.",
" ",
"  -niter=int",
"     Specifies the number of iterations for the refinement algorithms",
"     at each stage of the uncoarsening process. Default is 10.",
" ",
"  -ncuts=int",
"     Specifies the number of different partitionings that it will compute.",
"     The final partitioning is the one that achieves the best edgecut or",
"     communication volume. Default is 1.",
" ",
"  -nooutput",
"     Specifies that no partitioning file should be generated.",
" ",
"  -seed=int",
"     Selects the seed of the random number generator.  ",
" ",
"  -dbglvl=int      ",
"     Selects the dbglvl.  ",
" ",
"  -help",
"     Prints this message.",
""
};

static char shorthelpstr[][100] = {
" ",
"   Usage: mpmetis [options] <filename> <nparts>",
"          use 'mpmetis -help' for a summary of the options.",
""
};
 


/*************************************************************************
* This is the entry point of the command-line argument parser
**************************************************************************/
params_t *parse_cmdline(int argc, char *argv[])
{
  int i, j, k;
  int c, option_index;
  params_t *params;

  params = (params_t *)gk_malloc(sizeof(params_t), "parse_cmdline");
  memset((void *)params, 0, sizeof(params_t));

  /* initialize the params data structure */
  params->gtype         = METIS_GTYPE_DUAL;
  params->ptype         = METIS_PTYPE_KWAY;
  params->objtype       = METIS_OBJTYPE_CUT;
  params->ctype         = METIS_CTYPE_SHEM;
  params->iptype        = METIS_IPTYPE_GROW;
  params->rtype         = -1;

  params->minconn       = 0;
  params->contig        = 0;

  params->nooutput      = 0;
  params->wgtflag       = 3;

  params->ncuts         = 1;
  params->niter         = 10;
  params->ncommon       = 1;

  params->dbglvl        = 0;
  params->balance       = 0;
  params->seed          = -1;
  params->dbglvl        = 0;

  params->tpwgtsfile    = NULL;

  params->filename      = NULL;
  params->nparts        = 1;

  params->ufactor       = -1;

  gk_clearcputimer(params->iotimer);
  gk_clearcputimer(params->parttimer);
  gk_clearcputimer(params->reporttimer);


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
      case METIS_OPTION_GTYPE:
        if (gk_optarg)
          if ((params->gtype = gk_GetStringID(gtype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;
      case METIS_OPTION_PTYPE:
        if (gk_optarg)
          if ((params->ptype = gk_GetStringID(ptype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;
      case METIS_OPTION_OBJTYPE:
        if (gk_optarg)
          if ((params->objtype = gk_GetStringID(objtype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;
      case METIS_OPTION_CTYPE:
        if (gk_optarg)
          if ((params->ctype = gk_GetStringID(ctype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;
      case METIS_OPTION_IPTYPE:
        if (gk_optarg)
          if ((params->iptype = gk_GetStringID(iptype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;

/*
      case METIS_OPTION_RTYPE:
        if (gk_optarg)
          if ((params->rtype = gk_GetStringID(rtype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;
*/

      case METIS_OPTION_CONTIG:
        params->contig = 1;
        break;

      case METIS_OPTION_MINCONN:
        params->minconn = 1;
        break;

      case METIS_OPTION_NOOUTPUT:
        params->nooutput = 1;
        break;

      case METIS_OPTION_BALANCE:
        params->balance = 1;
        break;

      case METIS_OPTION_TPWGTS:
        if (gk_optarg) params->tpwgtsfile = gk_strdup(gk_optarg);
        break;

      case METIS_OPTION_NCUTS:
        if (gk_optarg) params->ncuts = (idx_t)atoi(gk_optarg);
        break;
      case METIS_OPTION_NITER:
        if (gk_optarg) params->niter = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_NCOMMON:
        if (gk_optarg) params->ncommon = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_UFACTOR:
        if (gk_optarg) params->ufactor = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_SEED:
        if (gk_optarg) params->seed = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_DBGLVL:
        if (gk_optarg) params->dbglvl = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_HELP:
        for (i=0; strlen(helpstr[i]) > 0; i++)
          printf("%s\n", helpstr[i]);
        exit(0);
        break;
      case '?':
      default:
        errexit("Illegal command-line option(s)\n"
                "Use %s -help for a summary of the options.\n", argv[0]);
    }
  }

  if (argc-gk_optind != 2) {
    printf("Missing parameters.");
    for (i=0; strlen(shorthelpstr[i]) > 0; i++)
      printf("%s\n", shorthelpstr[i]);
    exit(0);
  }

  params->filename = gk_strdup(argv[gk_optind++]);
  params->nparts   = atoi(argv[gk_optind++]);
    
  if (params->nparts < 2) 
    errexit("The number of partitions should be greater than 1!\n");


  /* Set the ptype-specific defaults */
  if (params->ptype == METIS_PTYPE_RB) {
    params->rtype = METIS_RTYPE_FM;
  }
  if (params->ptype == METIS_PTYPE_KWAY) {
    params->iptype = METIS_IPTYPE_METISRB;
    params->rtype  = METIS_RTYPE_GREEDY;
  }

  /* Check for invalid parameter combination */
  if (params->ptype == METIS_PTYPE_RB) {
    if (params->contig)
      errexit("The -contig option cannot be specified with rb partitioning.\n");
    if (params->minconn)
      errexit("The -minconn option cannot be specified with rb partitioning.\n");
    if (params->objtype == METIS_OBJTYPE_VOL)
      errexit("The -objtype=vol option cannot be specified with rb partitioning.\n");
  }

  return params;
}


