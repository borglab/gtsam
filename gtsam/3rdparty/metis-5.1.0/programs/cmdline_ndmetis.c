/*!
\file cmdline_ndmetis.c
\brief Command-line argument parsing for ndmetis

\date 12/24/2008
\author George
\version\verbatim $Id: cmdline_ndmetis.c 13900 2013-03-24 15:27:07Z karypis $\endverbatim
*/

#include "metisbin.h"


/*-------------------------------------------------------------------
 * Command-line options 
 *-------------------------------------------------------------------*/
static struct gk_option long_options[] = {
  {"ctype",          1,      0,      METIS_OPTION_CTYPE},
  {"iptype",         1,      0,      METIS_OPTION_IPTYPE},
  {"rtype",          1,      0,      METIS_OPTION_RTYPE}, 
  {"ufactor",        1,      0,      METIS_OPTION_UFACTOR},
  {"pfactor",        1,      0,      METIS_OPTION_PFACTOR},
  {"nocompress",     0,      0,      METIS_OPTION_COMPRESS},
  {"ccorder",        0,      0,      METIS_OPTION_CCORDER},
  {"no2hop",         0,      0,      METIS_OPTION_NO2HOP},
  {"nooutput",       0,      0,      METIS_OPTION_NOOUTPUT},
  {"niter",          1,      0,      METIS_OPTION_NITER},
  {"nseps",          1,      0,      METIS_OPTION_NSEPS},
  {"seed",           1,      0,      METIS_OPTION_SEED},
  {"dbglvl",         1,      0,      METIS_OPTION_DBGLVL},
  {"help",           0,      0,      METIS_OPTION_HELP},
  {0,                0,      0,      0}
};


static gk_StringMap_t ctype_options[] = {
 {"rm",                 METIS_CTYPE_RM},
 {"shem",               METIS_CTYPE_SHEM},
 {NULL,                 0}
};

static gk_StringMap_t iptype_options[] = {
 {"edge",               METIS_IPTYPE_EDGE},
 {"node",               METIS_IPTYPE_NODE},
 {NULL,                 0}
};

static gk_StringMap_t rtype_options[] = {
 {"2sided",             METIS_RTYPE_SEP2SIDED},
 {"1sided",             METIS_RTYPE_SEP1SIDED},
 {NULL,                 0}
};



/*-------------------------------------------------------------------
 * Mini help
 *-------------------------------------------------------------------*/
static char helpstr[][100] =
{
" ",
"Usage: ndmetis [options] <filename>",
" ",
" Required parameters",
"    filename    Stores the graph to be partitioned.",
" ",
" Optional parameters",
"  -ctype=string",
"     Specifies the scheme to be used to match the vertices of the graph",
"     during the coarsening.",
"     The possible values are:",
"        rm       - Random matching",
"        shem     - Sorted heavy-edge matching [default]",
" ",
"  -iptype=string [applies only when -ptype=rb]",
"     Specifies the scheme to be used to compute the initial bisection",
"     of the graph.",
"     The possible values are:",
"        edge     - Separator from an edge cut",
"        node     - Separator from a greedy node-based strategy [default]",
" ",
"  -rtype=string",
"     Specifies the scheme to be used for refinement.",
"     The possible values are:",
"        1sided   - 1-sided node-based refinement [default]",
"        2sided   - 2-sided node-based refinement",
" ",
"  -ufactor=int",
"     Specifies the maximum allowed load imbalance between the left and",
"     right partitions during each bisection. The load imbalanced is",
"     measured as the ratio of the 2*max(left,right)/(left+right), where",
"     left and right are the sizes of the respective partitions. ",
"     A value of x indicates that the allowed load imbalance is 1+x/1000.",
"     Default is 200, indicating a load imbalance of 1.20.",
" ",
"  -pfactor=int",
"     Specifies the minimum degree of the vertices that will be ordered ",
"     last. If the specified value is x>0, then any vertices with a degree",
"     greater than 0.1*x*(average degree) are removed from the graph, an",
"     ordering of the rest of the vertices is computed, and an overall ",
"     ordering is computed by ordering the removed vertices at the end ",
"     of the overall ordering.",
"     Default value is 0, indicating that no vertices are removed",
" ",
"  -no2hop",
"     Specifies that the coarsening will not perform any 2-hop matchings",
"     when the standard matching fails to sufficiently contract the graph.",
" ",
"  -nocompress",
"     Specifies that the graph should not be compressed by combining",
"     together vertices that have identical adjacency lists.",
" ",
"  -ccorder",
"     Specifies if the connected components of the graph should first be ",
"     identified and ordered separately.",
" ",
"  -niter=int",
"     Specifies the maximum number of iterations for the refinement ",
"     algorithms at each stage of the uncoarsening process. Default is 10.",
" ",
"  -nseps=int",
"     Specifies the number of different separators that it will compute at",
"     each level of the nested dissection. The final separator that is used",
"     is the smallest one. Default is 1.",
" ",
"  -nooutput",
"     Specifies that no ordering file should be generated.",
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
"   Usage: ndmetis [options] <filename>",
"          use 'ndmetis -help' for a summary of the options.",
""
};
 


/*************************************************************************/
/*! This is the entry point of the command-line argument parser */
/*************************************************************************/
params_t *parse_cmdline(int argc, char *argv[])
{
  int i, j, k;
  int c, option_index;
  params_t *params;

  params = (params_t *)gk_malloc(sizeof(params_t), "parse_cmdline");
  memset((void *)params, 0, sizeof(params_t));

  /* initialize the params data structure */
  params->ctype         = METIS_CTYPE_SHEM;
  params->iptype        = METIS_IPTYPE_NODE;
  params->rtype         = METIS_RTYPE_SEP1SIDED;

  params->ufactor       = OMETIS_DEFAULT_UFACTOR;
  params->pfactor       = 0;
  params->compress      = 1;
  params->ccorder       = 0;
  params->no2hop        = 0;

  params->nooutput      = 0;
  params->wgtflag       = 1;

  params->nseps         = 1;
  params->niter         = 10;

  params->seed          = -1;
  params->dbglvl        = 0;

  params->filename      = NULL;
  params->nparts        = 1;


  gk_clearcputimer(params->iotimer);
  gk_clearcputimer(params->parttimer);
  gk_clearcputimer(params->reporttimer);


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
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

      case METIS_OPTION_RTYPE:
        if (gk_optarg)
          if ((params->rtype = gk_GetStringID(rtype_options, gk_optarg)) == -1)
            errexit("Invalid option -%s=%s\n", long_options[option_index].name, gk_optarg);
        break;

      case METIS_OPTION_UFACTOR:
        if (gk_optarg) params->ufactor = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_PFACTOR:
        if (gk_optarg) params->pfactor = (idx_t)atoi(gk_optarg);
        break;

      case METIS_OPTION_COMPRESS:
        params->compress = 0;
        break;

      case METIS_OPTION_CCORDER:
        params->ccorder = 1;
        break;

      case METIS_OPTION_NO2HOP:
        params->no2hop = 1;
        break;

      case METIS_OPTION_NOOUTPUT:
        params->nooutput = 1;
        break;

      case METIS_OPTION_NSEPS:
        if (gk_optarg) params->nseps = (idx_t)atoi(gk_optarg);
        break;
      case METIS_OPTION_NITER:
        if (gk_optarg) params->niter = (idx_t)atoi(gk_optarg);
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

  if (argc-gk_optind != 1) {
    printf("Missing parameters.");
    for (i=0; strlen(shorthelpstr[i]) > 0; i++)
      printf("%s\n", shorthelpstr[i]);
    exit(0);
  }

  params->filename = gk_strdup(argv[gk_optind++]);
    
  return params;
}


