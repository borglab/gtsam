/*!
\file cmdline_m2gmetis.c

\brief Command-line argument parsing for m2gmetis

\date 12/24/2008
\author George
\version\verbatim $Id: cmdline_m2gmetis.c 10046 2011-06-01 14:13:40Z karypis $\endverbatim
*/

#include "metisbin.h"


/*-------------------------------------------------------------------
 * Command-line options 
 *-------------------------------------------------------------------*/
static struct gk_option long_options[] = {
  {"gtype",          1,      0,      METIS_OPTION_GTYPE},
  {"ncommon",        1,      0,      METIS_OPTION_NCOMMON},

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


/*-------------------------------------------------------------------
 * Mini help
 *-------------------------------------------------------------------*/
static char helpstr[][100] =
{
" ",
"Usage: m2gmetis [options] <meshfile> <graphfile>",
" ",
" Required parameters",
"    meshfile    Stores the input mesh.",
"    graphfile   The filename of the output graph.",
" ",
" Optional parameters",
"  -gtype=string",
"     Specifies the graph that will be generated.",
"     The possible values are:",
"        dual     - Generate dual graph of the mesh [default]",
"        nodal    - Generate the nodal graph of the mesh",
" ",
"  -ncommon=int [applies when gtype=dual]",
"     Specifies the common number of nodes that two elements must have",
"     in order to put an edge between them in the dual graph. Default is 1.",
" ",
"  -dbglvl=int      ",
"     Selects the dbglvl.",
" ",
"  -help",
"     Prints this message.",
""
};

static char shorthelpstr[][100] = {
" ",
"   Usage: m2gmetis [options] <meshfile> <graphfile>",
"          use 'm2gmetis -help' for a summary of the options.",
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
  params->gtype     = METIS_GTYPE_DUAL;
  params->ncommon   = 1;
  params->dbglvl    = 0;
  params->filename  = NULL;
  params->outfile   = NULL;


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

      case METIS_OPTION_NCOMMON:
        if (gk_optarg) params->ncommon = (idx_t)atoi(gk_optarg);
        if (params->ncommon < 1) 
           errexit("The -ncommon option should specify a number >= 1.\n");
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
  params->outfile  = gk_strdup(argv[gk_optind++]);
    
  return params;
}


