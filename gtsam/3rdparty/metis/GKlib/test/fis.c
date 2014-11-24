/*!
\file  
\brief A simple frequent itemset discovery program to test GKlib's routines

\date 6/12/2008
\author George
\version \verbatim $Id: fis.c 11075 2011-11-11 22:31:52Z karypis $ \endverbatim
*/

#include <GKlib.h>

/*************************************************************************/
/*! Data structures for the code */
/*************************************************************************/
typedef struct {
  ssize_t minlen, maxlen;
  ssize_t minfreq, maxfreq;
  char *filename;
  int silent;
  ssize_t nitemsets;
  char *clabelfile;
  char **clabels;
} params_t;

/*************************************************************************/
/*! Constants */
/*************************************************************************/
#define CMD_MINLEN      1
#define CMD_MAXLEN      2
#define CMD_MINFREQ     3
#define CMD_MAXFREQ     4
#define CMD_SILENT      5
#define CMD_CLABELFILE  6
#define CMD_HELP        10


/*************************************************************************/
/*! Local variables */
/*************************************************************************/
static struct gk_option long_options[] = {
  {"minlen",        1,      0,      CMD_MINLEN},
  {"maxlen",        1,      0,      CMD_MAXLEN},
  {"minfreq",       1,      0,      CMD_MINFREQ},
  {"maxfreq",       1,      0,      CMD_MAXFREQ},
  {"silent",        0,      0,      CMD_SILENT},
  {"clabels",       1,      0,      CMD_CLABELFILE},
  {"help",          0,      0,      CMD_HELP},
  {0,               0,      0,      0}
};


/*-------------------------------------------------------------------*/
/* Mini help  */
/*-------------------------------------------------------------------*/
static char helpstr[][100] = {
" ",
"Usage: fis [options] <mat-file>",
" ",
" Required parameters",
"  mat-file",
"     The name of the file storing the transactions. The file is in ",
"     Cluto's .mat format.",
" ",
" Optional parameters",
"  -minlen=int",
"     Specifies the minimum length of the patterns. [default: 1]",
" ",
"  -maxlen=int",
"     Specifies the maximum length of the patterns. [default: none]",
" ",
"  -minfreq=int",
"     Specifies the minimum frequency of the patterns. [default: 10]",
" ",
"  -maxfreq=int",
"     Specifies the maximum frequency of the patterns. [default: none]",
" ",
"  -silent",
"     Does not print the discovered itemsets.",
" ",
"  -clabels=filename",
"     Specifies the name of the file that stores the column labels.",
" ",
"  -help",
"     Prints this message.",
""
};

static char shorthelpstr[][100] = {
" ",
"   Usage: fis [options] <mat-file>",
"          use 'fis -help' for a summary of the options.",
""
};
 


/*************************************************************************/
/*! Function prototypes */
/*************************************************************************/
void print_init_info(params_t *params, gk_csr_t *mat);
void print_final_info(params_t *params);
params_t *parse_cmdline(int argc, char *argv[]);
void print_an_itemset(void *stateptr, int nitems, int *itemind, 
                      int ntrans, int *tranind);


/*************************************************************************/
/*! the entry point */
/**************************************************************************/
int main(int argc, char *argv[])
{
  ssize_t i;
  char line[8192];
  FILE *fpin;
  params_t *params;
  gk_csr_t *mat;
 
  params = parse_cmdline(argc, argv);
  params->nitemsets = 0;

  /* read the data */
  mat = gk_csr_Read(params->filename, GK_CSR_FMT_CLUTO, 1, 1);
  gk_csr_CreateIndex(mat, GK_CSR_COL);

  /* read the column labels */
  params->clabels = (char **)gk_malloc(mat->ncols*sizeof(char *), "main: clabels");
  if (params->clabelfile == NULL) {
    for (i=0; i<mat->ncols; i++) {
      sprintf(line, "%zd", i);
      params->clabels[i] = gk_strdup(line);
    }
  }
  else {
    fpin = gk_fopen(params->clabelfile, "r", "main: fpin");
    for (i=0; i<mat->ncols; i++) {
      if (fgets(line, 8192, fpin) == NULL)
        errexit("Failed on fgets.\n");
      params->clabels[i] = gk_strdup(gk_strtprune(line, " \n\t"));
    }
    gk_fclose(fpin);
  }


  print_init_info(params, mat);

  gk_find_frequent_itemsets(mat->nrows, mat->rowptr, mat->rowind,
      params->minfreq, params->maxfreq, params->minlen, params->maxlen,
      &print_an_itemset, (void *)params);

  printf("Total itemsets found: %zd\n", params->nitemsets);

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
      params->filename, mat->nrows, mat->ncols, mat->rowptr[mat->nrows]);

  printf("\n");
  printf("Options --------------------------------------------------------------------\n");
  printf(" minlen=%zd, maxlen=%zd, minfeq=%zd, maxfreq=%zd\n",
      params->minlen, params->maxlen, params->minfreq, params->maxfreq);

  printf("\n");
  printf("Finding patterns... -----------------------------------------------------\n");
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
  params->minlen     = 1;
  params->maxlen     = -1;
  params->minfreq    = 10;
  params->maxfreq    = -1;
  params->silent     = 0;
  params->filename   = NULL;
  params->clabelfile = NULL;


  /* Parse the command line arguments  */
  while ((c = gk_getopt_long_only(argc, argv, "", long_options, &option_index)) != -1) {
    switch (c) {
      case CMD_MINLEN:
        if (gk_optarg) params->minlen = atoi(gk_optarg);
        break;
      case CMD_MAXLEN:
        if (gk_optarg) params->maxlen = atoi(gk_optarg);
        break;
      case CMD_MINFREQ:
        if (gk_optarg) params->minfreq = atoi(gk_optarg);
        break;
      case CMD_MAXFREQ:
        if (gk_optarg) params->maxfreq = atoi(gk_optarg);
        break;

      case CMD_SILENT:
        params->silent = 1;
        break;

      case CMD_CLABELFILE:
        if (gk_optarg) params->clabelfile = gk_strdup(gk_optarg);
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

  params->filename = gk_strdup(argv[gk_optind++]);

  if (!gk_fexists(params->filename))
    errexit("input file %s does not exist.\n", params->filename);

  return params;
}



/*************************************************************************/
/*! This is the callback function for the itemset discovery routine */
/*************************************************************************/
void print_an_itemset(void *stateptr, int nitems, int *itemids, int ntrans, 
         int *transids)
{
  ssize_t i;
  params_t *params;

  params = (params_t *)stateptr;
  params->nitemsets++;

  if (!params->silent) {
    printf("%4zd %4d %4d => ", params->nitemsets, nitems, ntrans);
    for (i=0; i<nitems; i++)
      printf(" %s", params->clabels[itemids[i]]);
    printf("\n");
    for (i=0; i<ntrans; i++)
      printf(" %d\n", transids[i]);
    printf("\n");
  }
}
