/*!
\file  
\brief A simple frequent itemset discovery program to test GKlib's routines

\date 6/12/2008
\author George
\version \verbatim $Id: rw.c 11387 2012-01-21 23:36:23Z karypis $ \endverbatim
*/

#include <GKlib.h>

/*************************************************************************/
/*! Data structures for the code */
/*************************************************************************/
typedef struct {
  int niter;
  int ntvs;
  int ppr;
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
#define CMD_PPR         4
#define CMD_NTVS        5
#define CMD_HELP        10


/*************************************************************************/
/*! Local variables */
/*************************************************************************/
static struct gk_option long_options[] = {
  {"niter",      1,      0,      CMD_NITER},
  {"lamda",      1,      0,      CMD_LAMDA},
  {"eps",        1,      0,      CMD_EPS},
  {"ppr",        1,      0,      CMD_PPR},
  {"ntvs",       1,      0,      CMD_NTVS},
  {"help",       0,      0,      CMD_HELP},
  {0,            0,      0,      0}
};


/*-------------------------------------------------------------------*/
/* Mini help  */
/*-------------------------------------------------------------------*/
static char helpstr[][100] = {
" ",
"Usage: rw [options] <graph-file> <out-file>",
" ",
" Required parameters",
"  graph-file",
"     The name of the file storing the transactions. The file is in ",
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
"  -ppr=int",
"     Specifies the source of the personalized PR. [default: -1]",
" ",
"  -ntvs=int",
"     Specifies the number of test-vectors to compute. [default: -1]",
" ",
"  -help",
"     Prints this message.",
""
};

static char shorthelpstr[][100] = {
" ",
"   Usage: rw [options] <graph-file> <out-file>",
"          use 'rw -help' for a summary of the options.",
""
};
 


/*************************************************************************/
/*! Function prototypes */
/*************************************************************************/
void print_init_info(params_t *params, gk_csr_t *mat);
void print_final_info(params_t *params);
params_t *parse_cmdline(int argc, char *argv[]);


/*************************************************************************/
/*! the entry point */
/**************************************************************************/
int main(int argc, char *argv[])
{
  ssize_t i, j, niter;
  params_t *params;
  gk_csr_t *mat;
  FILE *fpout;
 
  /* get command-line options */
  params = parse_cmdline(argc, argv);

  /* read the data */
  mat = gk_csr_Read(params->infile, GK_CSR_FMT_METIS, 1, 1);

  /* display some basic stats */
  print_init_info(params, mat);



  if (params->ntvs != -1) {
    /* compute the pr for different randomly generated restart-distribution vectors */
    float **prs;

    prs = gk_fAllocMatrix(params->ntvs, mat->nrows, 0.0, "main: prs");

    /* generate the random restart vectors */
    for (j=0; j<params->ntvs; j++) {
      for (i=0; i<mat->nrows; i++)
        prs[j][i] = RandomInRange(931);
      gk_fscale(mat->nrows, 1.0/gk_fsum(mat->nrows, prs[j], 1), prs[j], 1);

      niter = gk_rw_PageRank(mat, params->lamda, params->eps, params->niter, prs[j]);
      printf("tvs#: %zd; niters: %zd\n", j, niter);
    }

    /* output the computed pr scores */
    fpout = gk_fopen(params->outfile, "w", "main: outfile");
    for (i=0; i<mat->nrows; i++) {
      for (j=0; j<params->ntvs; j++) 
        fprintf(fpout, "%.4e ", prs[j][i]);
      fprintf(fpout, "\n");
    }
    gk_fclose(fpout);

    gk_fFreeMatrix(&prs, params->ntvs, mat->nrows);
  }
  else if (params->ppr != -1) {
    /* compute the personalized pr from the specified vertex */
    float *pr;

    pr = gk_fsmalloc(mat->nrows, 0.0, "main: pr");

    pr[params->ppr-1] = 1.0;

    niter = gk_rw_PageRank(mat, params->lamda, params->eps, params->niter, pr);
    printf("ppr: %d; niters: %zd\n", params->ppr, niter);

    /* output the computed pr scores */
    fpout = gk_fopen(params->outfile, "w", "main: outfile");
    for (i=0; i<mat->nrows; i++) 
      fprintf(fpout, "%.4e\n", pr[i]);
    gk_fclose(fpout);

    gk_free((void **)&pr, LTERM);
  }
  else {
    /* compute the standard pr */
    int jmax;
    float diff, maxdiff;
    float *pr;

    pr = gk_fsmalloc(mat->nrows, 1.0/mat->nrows, "main: pr");

    niter = gk_rw_PageRank(mat, params->lamda, params->eps, params->niter, pr);
    printf("pr; niters: %zd\n", niter);

    /* output the computed pr scores */
    fpout = gk_fopen(params->outfile, "w", "main: outfile");
    for (i=0; i<mat->nrows; i++) {
      for (jmax=i, maxdiff=0.0, j=mat->rowptr[i]; j<mat->rowptr[i+1]; j++) {
        if ((diff = fabs(pr[i]-pr[mat->rowind[j]])) > maxdiff) {
          maxdiff = diff;
          jmax = mat->rowind[j];
        }
      }
      fprintf(fpout, "%.4e %10zd %.4e %10d\n", pr[i], 
          mat->rowptr[i+1]-mat->rowptr[i], maxdiff, jmax+1);
    }
    gk_fclose(fpout);

    gk_free((void **)&pr, LTERM);
  }

  gk_csr_Free(&mat);

  /* display some final stats */
  print_final_info(params);
}



/*************************************************************************/
/*! This function prints run parameters */
/*************************************************************************/
void print_init_info(params_t *params, gk_csr_t *mat)
{
  printf("*******************************************************************************\n");
  printf(" fis\n\n");
  printf("Matrix Information ---------------------------------------------------------\n");
  printf(" input file=%s, [%d, %d, %zd]\n", 
      params->infile, mat->nrows, mat->ncols, mat->rowptr[mat->nrows]);

  printf("\n");
  printf("Options --------------------------------------------------------------------\n");
  printf(" niter=%d, ntvs=%d, ppr=%d, lamda=%f, eps=%e\n",
      params->niter, params->ntvs, params->ppr, params->lamda, params->eps);

  printf("\n");
  printf("Performing random walks... ----------------------------------------------\n");
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
  params->niter     = 100;
  params->ppr       = -1;
  params->ntvs      = -1;
  params->eps       = 1e-10;
  params->lamda     = 0.80;
  params->infile    = NULL;
  params->outfile   = NULL;


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
      case CMD_NITER:
        if (gk_optarg) params->niter = atoi(gk_optarg);
        break;
      case CMD_NTVS:
        if (gk_optarg) params->ntvs = atoi(gk_optarg);
        break;
      case CMD_PPR:
        if (gk_optarg) params->ppr = atoi(gk_optarg);
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

  if (argc-gk_optind != 2) {
    printf("Unrecognized parameters.");
    for (i=0; strlen(shorthelpstr[i]) > 0; i++)
      printf("%s\n", shorthelpstr[i]);
    exit(0);
  }

  params->infile  = gk_strdup(argv[gk_optind++]);
  params->outfile = gk_strdup(argv[gk_optind++]);

  if (!gk_fexists(params->infile))
    errexit("input file %s does not exist.\n", params->infile);

  if (params->ppr != -1 && params->ntvs != -1)
    errexit("Only one of the -ppr and -ntvs options can be specified.\n");

  return params;
}

