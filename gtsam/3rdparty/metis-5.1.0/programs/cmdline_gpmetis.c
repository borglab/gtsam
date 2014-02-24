/*!
\file cmdline_gpmetis.c
\brief Command-line argument parsing for gpmetis

\date 12/24/2008
\author George
\version\verbatim $Id: cmdline_gpmetis.c 13901 2013-03-24 16:17:03Z karypis $\endverbatim
*/

#include "metisbin.h"


/*-------------------------------------------------------------------
 * Command-line options 
 *-------------------------------------------------------------------*/
static struct gk_option long_options[] = {
  {"ptype",          1,      0,      METIS_OPTION_PTYPE},
  {"objtype",        1,      0,      METIS_OPTION_OBJTYPE}, 

  {"ctype",          1,      0,      METIS_OPTION_CTYPE},
  {"iptype",         1,      0,      METIS_OPTION_IPTYPE},
/*  {"rtype",          1,      0,      METIS_OPTION_RTYPE}, */

/*  {"balanced",       0,      0,      METIS_OPTION_BALANCE}, */

  {"no2hop",         0,      0,      METIS_OPTION_NO2HOP},
  {"minconn",        0,      0,      METIS_OPTION_MINCONN},
  {"contig",         0,      0,      METIS_OPTION_CONTIG},

  {"nooutput",       0,      0,      METIS_OPTION_NOOUTPUT},

  {"ufactor",        1,      0,      METIS_OPTION_UFACTOR},
  {"niter",          1,      0,      METIS_OPTION_NITER},
  {"ncuts",          1,      0,      METIS_OPTION_NCUTS},

  {"tpwgts",         1,      0,      METIS_OPTION_TPWGTS},
  {"ubvec",          1,      0,      METIS_OPTION_UBVEC},

  {"seed",           1,      0,      METIS_OPTION_SEED},

  {"dbglvl",         1,      0,      METIS_OPTION_DBGLVL},

  {"help",           0,      0,      METIS_OPTION_HELP},
  {0,                0,      0,      0}
};



/*-------------------------------------------------------------------
 * Mappings for the various parameter values
 *-------------------------------------------------------------------*/
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

static gk_StringMap_t rtype_options[] = {
 {"fm",                METIS_RTYPE_FM},
 {"greedy",            METIS_RTYPE_GREEDY},
 {NULL,                 0}
};



/*-------------------------------------------------------------------
 * Mini help
 *-------------------------------------------------------------------*/
static char helpstr[][100] =
{
" ",
"Usage: gpmetis [options] graphfile nparts",
" ",
" Required parameters",
"    graphfile   Stores the graph to be partitioned.",
"    nparts      The number of partitions to split the graph.",
" ",
" Optional parameters",
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
"        grow     - Grow a bisection using a greedy scheme [default for ncon=1]",
"        random   - Compute a bisection at random [default for ncon>1]",
" ",
"  -objtype=string [applies only when -ptype=kway]",
"     Specifies the objective that the partitioning routines will optimize.",
"     The possible values are:",
"        cut      - Minimize the edgecut [default]",
"        vol      - Minimize the total communication volume",
" ",
/*
"  -rtype=string",
"     Specifies the scheme to be used for refinement.",
"     The possible values are:",
"        fm       - 2-way FM refinement [default for -ptype=rb]",
"        random   - Random k-way refinement",
"        greedy   - Greedy k-way refinement [default for -ptype=kway]",
" ",
*/
"  -no2hop",
"     Specifies that the coarsening will not perform any 2-hop matchings",
"     when the standard matching fails to sufficiently contract the graph.",
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
"  -ubvec=string",
"     Applies only for multi-constraint partitioning and specifies the per",
"     constraint allowed load imbalance among partitions. The required ",
"     parameter corresponds to a space separated set of floating point",
"     numbers, one for each of the constraints. For example, for three",
"     constraints, the string can be \"1.02 1.2 1.35\" indicating a ",
"     desired maximum load imbalance of 2%, 20%, and 35%, respectively.",
"     The load imbalance is defined in a way similar to ufactor.",
"     If supplied, this parameter takes priority over ufactor.",
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
/*
"  -balance",
"     Specifies that the final partitioning should contain nparts-1 equal",
"     size partitions with the last partition having upto nparts-1 fewer",
"     vertices.",
" ",
*/
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
"   Usage: gpmetis [options] <filename> <nparts>",
"          use 'gpmetis -help' for a summary of the options.",
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
  params->ptype         = METIS_PTYPE_KWAY;
  params->objtype       = METIS_OBJTYPE_CUT;
  params->ctype         = METIS_CTYPE_SHEM;
  params->iptype        = -1;
  params->rtype         = -1;

  params->no2hop        = 0;
  params->minconn       = 0;
  params->contig        = 0;

  params->nooutput      = 0;
  params->wgtflag       = 3;

  params->ncuts         = 1;
  params->niter         = 10;

  params->dbglvl        = 0;
  params->balance       = 0;
  params->seed          = -1;
  params->dbglvl        = 0;

  params->tpwgtsfile    = NULL;

  params->filename      = NULL;
  params->nparts        = 1;

  params->ufactor       = -1;

  params->ubvecstr      = NULL;
  params->ubvec         = NULL;


  gk_clearcputimer(params->iotimer);
  gk_clearcputimer(params->parttimer);
  gk_clearcputimer(params->reporttimer);


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
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

      case METIS_OPTION_NO2HOP:
        params->no2hop = 1;
        break;

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

      case METIS_OPTION_UBVEC:
        if (gk_optarg) params->ubvecstr = gk_strdup(gk_optarg);
        break;

      case METIS_OPTION_NCUTS:
        if (gk_optarg) params->ncuts = (idx_t)atoi(gk_optarg);
        break;
      case METIS_OPTION_NITER:
        if (gk_optarg) params->niter = (idx_t)atoi(gk_optarg);
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
    params->rtype   = METIS_RTYPE_FM;
  }
  if (params->ptype == METIS_PTYPE_KWAY) {
    params->iptype  = METIS_IPTYPE_METISRB;
    params->rtype   = METIS_RTYPE_GREEDY;
  }

  /* Check for invalid parameter combination */
  if (params->ptype == METIS_PTYPE_RB) {
    if (params->contig)
      errexit("***The -contig option cannot be specified with rb partitioning. Will be ignored.\n");
    if (params->minconn)
      errexit("***The -minconn option cannot be specified with rb partitioning. Will be ignored. \n");
    if (params->objtype == METIS_OBJTYPE_VOL)
      errexit("The -objtype=vol option cannot be specified with rb partitioning.\n");
  }

  return params;
}


