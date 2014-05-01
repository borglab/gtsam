#include <string.h>
#include <time.h>
#include <signal.h>
#include "lp_lib.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif

#define filetypeLP      1
#define filetypeMPS     2
#define filetypeFREEMPS 3
#define filetypeCPLEX   4
#define filetypeXLI     5

#define FORCED_EXIT 255

int EndOfPgr(int i)
{
#   if defined FORTIFY
      Fortify_LeaveScope();
#   endif
    exit(i);
    return(0);
}

void SIGABRT_func(int sig)
 {
   EndOfPgr(FORCED_EXIT);
 }

void print_help(char *argv[])
{
  printf("Usage of %s version %d.%d.%d.%d:\n", argv[0], MAJORVERSION, MINORVERSION, RELEASE, BUILD);
  printf("%s [options] [[<]input_file]\n", argv[0]);
  printf("List of options:\n");
  printf("-h\t\tprints this message\n");
#if defined PARSER_LP
  printf("-lp\t\tread from LP file (default)\n");
#endif
  printf("-mps\t\tread from MPS file in fixed format\n");
  printf("-fmps\t\tread from MPS file in free format\n");
  printf("-rxli xliname filename\n\t\tread file with xli library\n");
  printf("-rxlidata datafilename\n\t\tdata file name for xli library.\n");
  printf("-rxliopt options\n\t\toptions for xli library.\n");
  printf("-rbas filename\tread basis from filename.\n");
  printf("-rpar filename\tread parameters from filename.\n");
  printf("-rparopt options\n\t\toptions for parameter file:\n");
  printf("\t\t -H headername: header name for parameters. By default 'Default'\n");
  printf("-wlp filename\twrite to LP file\n");
  printf("-wmps filename\twrite to MPS file in fixed format\n");
  printf("-wfmps filename\twrite to MPS file in free format\n");
  printf("-wxli xliname filename\n\t\twrite file with xli library\n");
  printf("-wxliopt options\n\t\toptions for xli library.\n");
  printf("-wxlisol xliname filename\n\t\twrite solution file with xli library\n");
  printf("-wxlisolopt options\n\t\toptions for xli library.\n");
  printf("-wbas filename\twrite basis to filename.\n");
  printf("-wpar filename\twrite parameters to filename.\n");
  printf("-wparopt options\n\t\toptions for parameter file:\n");
  printf("\t\t -H headername: header name for parameters. By default 'Default'\n");
  printf("-wafter\t\tWrite model after solve (useful if presolve used).\n");
  printf("-parse_only\tparse input file but do not solve\n");
  printf("-nonames\tIgnore variables and constraint names\n");
  printf("-norownames\tIgnore constraint names\n");
  printf("-nocolnames\tIgnore variable names\n");
  printf("\n");
  printf("-min\t\tMinimize the lp problem (overrules setting in file)\n");
  printf("-max\t\tMaximize the lp problem (overrules setting in file)\n");
  printf("-r <value>\tspecify max nbr of pivots between a re-inversion of the matrix\n");
  printf("-piv <rule>\tspecify simplex pivot rule\n");
  printf("\t -piv0: Select first\n");
  printf("\t -piv1: Select according to Dantzig\n");
  printf("\t -piv2: Select Devex pricing from Paula Harris (default)\n");
  printf("\t -piv3: Select steepest edge\n");
  printf("These pivot rules can be combined with any of the following:\n");
  printf("-pivf\t\tIn case of Steepest Edge, fall back to DEVEX in primal.\n");
  printf("-pivm\t\tMultiple pricing.\n");
  printf("-piva\t\tTemporarily use First Index if cycling is detected.\n");
  printf("-pivr\t\tAdds a small randomization effect to the selected pricer.\n");
#if defined EnablePartialOptimization
  printf("-pivp\t\tEnable partial pricing.\n");
  printf("-pivpc\t\tEnable partial pricing on columns.\n");
  printf("-pivpr\t\tEnable partial pricing on rows.\n");
#endif
  printf("-pivll\t\tScan entering/leaving columns left rather than right.\n");
  printf("-pivla\t\tScan entering/leaving columns alternatingly left/right.\n");
  printf("-pivh\t\tUse Harris' primal pivot logic rather than the default.\n");
  printf("-pivt\t\tUse true norms for Devex and Steepest Edge initializations.\n");
  printf("-o0\t\tDon't put objective in basis%s.\n", DEF_OBJINBASIS ? "" : " (default)");
  printf("-o1\t\tPut objective in basis%s.\n", DEF_OBJINBASIS ? " (default)" : "");
  printf("-s <mode> <scaleloop>\tuse automatic problem scaling.\n");
  printf("\t -s0: No scaling\n");
  printf("\t -s1: Geometric scaling (default)\n");
  printf("\t -s2: Curtis-reid scaling\n");
  printf("\t -s3: Scale to convergence using largest absolute value\n");
  printf("\t  -s:\n");
  printf("\t -s4: Numerical range-based scaling\n");
  printf("\t -s5: Scale to convergence using logarithmic mean of all values\n");
  printf("\t -s6: Scale based on the simple numerical range\n");
  printf("\t -s7: Scale quadratic\n");
  printf("These scaling rules can be combined with any of the following:\n");
  printf("-sp\t\talso do power scaling.\n");
  printf("-si\t\talso do Integer scaling (default).\n");
  printf("-se\t\talso do equilibration to scale to the -1..1 range (default).\n");
  printf("-presolve\tpresolve problem before start optimizing (rows+columns)\n");
  printf("-presolverow\tpresolve problem before start optimizing (rows only)\n");
  printf("-presolvecol\tpresolve problem before start optimizing (columns only)\n");
  printf("-presolvel\talso eliminate linearly dependent rows\n");
  printf("-presolves\talso convert constraints to SOSes (only SOS1 handled)\n");
  printf("-presolver\tIf the phase 1 solution process finds that a constraint is\n\t\tredundant then this constraint is deleted\n");
  printf("-presolvek\tSimplification of knapsack-type constraints through\n\t\taddition of an extra variable, which also helps bound the OF\n");
  printf("-presolveq\tDirect substitution of one variable in 2-element equality\n\t\tconstraints; this requires changes to the constraint matrix\n");
  printf("-presolvem\tMerge rows\n");
  printf("-presolvefd\tCOLFIXDUAL\n");
  printf("-presolvebnd\tPresolve bounds\n");
  printf("-presolved\tPresolve duals\n");
  printf("-presolvef\tIdentify implied free variables (releasing their expl. bounds)\n");
  printf("-presolveslk\tIMPLIEDSLK\n");
  printf("-presolveg\tReduce (tighten) coef. in integer models based on GCD argument\n");
  printf("-presolveb\tAttempt to fix binary variables at one of their bounds\n");
  printf("-presolvec\tAttempt to reduce coefficients in binary models\n");
  printf("-presolverowd\tIdenfify and delete qualifying constraints that\n\t\tare dominated by others, also fixes variables at a bound\n");
  printf("-presolvecold\tDeletes variables (mainly binary), that are dominated\n\t\tby others (only one can be non-zero)\n");
  printf("-C <mode>\tbasis crash mode\n");
  printf("\t -C0: No crash basis\n");
  printf("\t -C2: Most feasible basis\n");
  printf("\t -C3: Least degenerate basis\n");
  printf("-prim\t\tPrefer the primal simplex for both phases.\n");
  printf("-dual\t\tPrefer the dual simplex for both phases.\n");
  printf("-simplexpp\tSet Phase1 Primal, Phase2 Primal.\n");
  printf("-simplexdp\tSet Phase1 Dual, Phase2 Primal.\n");
  printf("-simplexpd\tSet Phase1 Primal, Phase2 Dual.\n");
  printf("-simplexdd\tSet Phase1 Dual, Phase2 Dual.\n");
  printf("-degen\t\tuse perturbations to reduce degeneracy,\n\t\tcan increase numerical instability\n");
  printf("-degenc\t\tuse column check to reduce degeneracy\n");
  printf("-degend\t\tdynamic check to reduce degeneracy\n");
  printf("-degenf\t\tanti-degen fixedvars\n");
  printf("-degens\t\tanti-degen stalling\n");
  printf("-degenn\t\tanti-degen numfailure\n");
  printf("-degenl\t\tanti-degen lostfeas\n");
  printf("-degeni\t\tanti-degen infeasible\n");
  printf("-degenb\t\tanti-degen B&B\n");
  printf("-degenr\t\tanti-degen Perturbation of the working RHS at refactorization\n");
  printf("-degenp\t\tanti-degen Limit bound flips\n");
  printf("-trej <Trej>\tset minimum pivot value\n");
  printf("-epsd <epsd>\tset minimum tolerance for reduced costs\n");
  printf("-epsb <epsb>\tset minimum tolerance for the RHS\n");
  printf("-epsel <epsel>\tset tolerance for rounding values to zero\n");
  printf("-epsp <epsp>\tset the value that is used as perturbation scalar for\n\t\tdegenerative problems\n");
  printf("-improve <level>\titerative improvement level\n");
  printf("\t -improve0: none\n");
  printf("\t -improve1: Running accuracy measurement of solved equations on Bx=r\n");
  printf("\t -improve2: Improve initial dual feasibility by bound flips (default)\n");
  printf("\t -improve4: Low-cost accuracy monitoring in the dual\n");
  printf("\t -improve8: check for primal/dual feasibility at the node level\n");
  printf("-timeout <sec>\tTimeout after sec seconds when not solution found.\n");
/*
  printf("-timeoutok\tIf timeout, take the best yet found solution.\n");
*/
  printf("-bfp <filename>\tSet basis factorization package.\n");
  printf("\n");
  printf("-noint\t\tIgnore integer restrictions\n");
  printf("-e <number>\tspecifies the tolerance which is used to determine whether a\n\t\tfloating point number is in fact an integer.\n\t\tShould be < 0.5\n");
  printf("-g <number>\n");
  printf("-ga <number>\tspecifies the absolute MIP gap for branch-and-bound.\n\t\tThis specifies the absolute allowed tolerance\n\t\ton the object function. Can result in faster solving times.\n");
  printf("-gr <number>\tspecifies the relative MIP gap for branch-and-bound.\n\t\tThis specifies the relative allowed tolerance\n\t\ton the object function. Can result in faster solving times.\n");
  printf("-f\t\tspecifies that branch-and-bound algorithm stops at first found\n");
  printf("\t\tsolution\n");
  printf("-b <bound>\tspecify a lower bound for the objective function\n\t\tto the program. If close enough, may speed up the\n\t\tcalculations.\n");
  printf("-o <value>\tspecifies that branch-and-bound algorithm stops when objective\n");
  printf("\t\tvalue is better than value\n");
  printf("-c\n");
  printf("-cc\t\tduring branch-and-bound, take the ceiling branch first\n");
  printf("-cf\t\tduring branch-and-bound, take the floor branch first\n");
  printf("-ca\t\tduring branch-and-bound, the algorithm chooses branch\n");
  printf("-depth <limit>\tset branch-and-bound depth limit\n");
  printf("-n <solnr>\tspecify which solution number to return\n");
  printf("-B <rule>\tspecify branch-and-bound rule\n");
  printf("\t -B0: Select Lowest indexed non-integer column (default)\n");
  printf("\t -B1: Selection based on distance from the current bounds\n");
  printf("\t -B2: Selection based on the largest current bound\n");
  printf("\t -B3: Selection based on largest fractional value\n");
  printf("\t -B4: Simple, unweighted pseudo-cost of a variable\n");
  printf("\t -B5: This is an extended pseudo-costing strategy based on minimizing\n\t      the number of integer infeasibilities\n");
  printf("\t -B6: This is an extended pseudo-costing strategy based on maximizing\n\t      the normal pseudo-cost divided by the number of infeasibilities.\n\t      Similar to (the reciprocal of) a cost/benefit ratio\n");
  printf("These branch-and-bound rules can be combined with any of the following:\n");
  printf("-Bw\t\tWeightReverse branch-and-bound\n");
  printf("-Bb\t\tBranchReverse branch-and-bound\n");
  printf("-Bg\t\tGreedy branch-and-bound\n");
  printf("-Bp\t\tPseudoCost branch-and-bound\n");
  printf("-Bf\t\tDepthFirst branch-and-bound\n");
  printf("-Br\t\tRandomize branch-and-bound\n");
  printf("-BG\t\tGubMode branch-and-bound\n");
  printf("-Bd\t\tDynamic branch-and-bound\n");
  printf("-Bs\t\tRestartMode branch-and-bound\n");
  printf("-BB\t\tBreadthFirst branch-and-bound\n");
  printf("-Bo\t\tOrder variables to improve branch-and-bound performance\n");
  printf("-Bc\t\tDo bound tightening during B&B based of reduced cost info\n");
  printf("-Bi\t\tInitialize pseudo-costs by strong branching\n");
  printf("\n");
  printf("-time\t\tPrint CPU time to parse input and to calculate result.\n");
  printf("-v <level>\tverbose mode, gives flow through the program.\n");
  printf("\t\t if level not provided (-v) then -v4 (NORMAL) is taken.\n");
  printf("\t -v0: NEUTRAL\n");
  printf("\t -v1: CRITICAL\n");
  printf("\t -v2: SEVERE\n");
  printf("\t -v3: IMPORTANT (default)\n");
  printf("\t -v4: NORMAL\n");
  printf("\t -v5: DETAILED\n");
  printf("\t -v6: FULL\n");
  printf("-t\t\ttrace pivot selection\n");
  printf("-d\t\tdebug mode, all intermediate results are printed,\n\t\tand the branch-and-bound decisions\n");
  printf("-R\t\treport information while solving the model\n");
  printf("-Db <filename>\tDo a generic readable data dump of key lp_solve model variables\n\t\tbefore solve.\n\t\tPrincipally for run difference and debugging purposes\n");
  printf("-Da <filename>\tDo a generic readable data dump of key lp_solve model variables\n\t\tafter solve.\n\t\tPrincipally for run difference and debugging purposes\n");
  printf("-i\t\tprint all intermediate valid solutions.\n\t\tCan give you useful solutions even if the total run time\n\t\tis too long\n");
  printf("-ia\t\tprint all intermediate (only non-zero values) valid solutions.\n\t\tCan give you useful solutions even if the total run time\n\t\tis too long\n");
  printf("-S <detail>\tPrint solution. If detail omitted, then -S2 is used.\n");
  printf("\t -S0: Print nothing\n");
  printf("\t -S1: Only objective value\n");
  printf("\t -S2: Obj value+variables (default)\n");
  printf("\t -S3: Obj value+variables+constraints\n");
  printf("\t -S4: Obj value+variables+constraints+duals\n");
  printf("\t -S5: Obj value+variables+constraints+duals+lp model\n");
  printf("\t -S6: Obj value+variables+constraints+duals+lp model+scales\n");
  printf("\t -S7: Obj value+variables+constraints+duals+lp model+scales+lp tableau\n");
}

void print_cpu_times(const char *info)
{
  static clock_t last_time = 0;
  clock_t new_time;

  new_time = clock();
  fprintf(stderr, "CPU Time for %s: %gs (%gs total since program start)\n",
	  info, (new_time - last_time) / (double) CLOCKS_PER_SEC,
	  new_time / (double) CLOCKS_PER_SEC);
  last_time = new_time;
}

#if 0
int myabortfunc(lprec *lp, void *aborthandle)
{
  /* printf("%f\n",lp->rhs[0]*(lp->maximise ? 1 : -1)); */
  return(0);
}
#endif

static MYBOOL isNum(char *val)
{
  int ord;
  char *pointer;

  ord = strtol(val, &pointer, 10);
  return(*pointer == 0);
}

static void DoReport(lprec *lp, char *str)
{
  fprintf(stderr, "%s %6.1fsec %8g\n", str, time_elapsed(lp), get_working_objective(lp));
}

static void __WINAPI LPMessageCB(lprec *lp, void *USERHANDLE, int msg)
{
  if(msg==MSG_LPFEASIBLE)
    DoReport(lp, "Feasible solution ");
  else if(msg==MSG_LPOPTIMAL)
    DoReport(lp, "Real solution ");
  else if(msg==MSG_MILPFEASIBLE)
    DoReport(lp, "First MILP    ");
  else if(msg==MSG_MILPBETTER)
    DoReport(lp, "Improved MILP ");
}

void write_model(lprec *lp, char *wlp, char *wmps, char *wfmps, char *wxli, char *wxlisol, char *wxliname, char *wxlioptions)
{
  if(wlp != NULL)
    write_lp(lp, wlp);

  if(wmps != NULL)
    write_mps(lp, wmps);

  if(wfmps != NULL)
    write_freemps(lp, wfmps);

  if((wxliname != NULL) && (wxli != NULL)) {
    if(!set_XLI(lp, wxliname)) {
      fprintf(stderr, "Unable to set XLI library (%s).\n", wxliname);
      EndOfPgr(FORCED_EXIT);
    }
    write_XLI(lp, wxli, wxlioptions, FALSE);
    set_XLI(lp, NULL);
  }

  if((wxliname != NULL) && (wxlisol != NULL)) {
    if(!set_XLI(lp, wxliname)) {
      fprintf(stderr, "Unable to set XLI library (%s).\n", wxliname);
      EndOfPgr(FORCED_EXIT);
    }
    write_XLI(lp, wxlisol, wxlioptions, TRUE);
    set_XLI(lp, NULL);
  }
}

static void or_value(int *value, int orvalue)
{
  if(*value == -1)
    *value = 0;
  *value |= orvalue;
}

static void set_value(int *value, int orvalue)
{
  *value = orvalue;
}

int main(int argc, char *argv[])
{
  lprec *lp = NULL;
  char *filen, *wlp = NULL, *wmps = NULL, *wfmps = NULL;
  int i;
  int verbose = IMPORTANT /* CRITICAL */;
  int debug = -1;
  MYBOOL report = FALSE;
  MYBOOL nonames = FALSE, norownames = FALSE, nocolnames = FALSE;
  MYBOOL write_model_after = FALSE;
  MYBOOL noint = FALSE;
  int print_sol = -1;
  int floor_first = -1;
  MYBOOL do_set_bb_depthlimit = FALSE;
  int bb_depthlimit = 0;
  MYBOOL do_set_solutionlimit = FALSE;
  int solutionlimit = 0;
  MYBOOL break_at_first = FALSE;
  int scaling = 0;
  double scaleloop = 0;
  MYBOOL tracing = FALSE;
  short filetype = filetypeLP;
  int anti_degen1 = -1;
  int anti_degen2 = -1;
  short print_timing = FALSE;
  short parse_only = FALSE;
  int do_presolve = -1;
  short objective = 0;
  short PRINT_SOLUTION = 2;
  int improve = -1;
  int pivoting1 = -1;
  int pivoting2 = -1;
  int bb_rule1 = -1;
  int bb_rule2 = -1;
  int max_num_inv = -1;
  int scalemode1 = -1;
  int scalemode2 = -1;
  int crashmode = -1;
  /* short timeoutok = FALSE; */
  long sectimeout = -1;
  int result;
  MYBOOL preferdual = AUTOMATIC;
  int simplextype = -1;
  MYBOOL do_set_obj_bound = FALSE;
  REAL obj_bound = 0;
  REAL mip_absgap = -1;
  REAL mip_relgap = -1;
  REAL epsperturb = -1;
  REAL epsint = -1;
  REAL epspivot = -1;
  REAL epsd = -1;
  REAL epsb = -1;
  REAL epsel = -1;
  MYBOOL do_set_break_at_value = FALSE;
  REAL break_at_value = 0;
  FILE *fpin = stdin;
  char *bfp = NULL;
  char *rxliname = NULL, *rxli = NULL, *rxlidata = NULL, *rxlioptions = NULL, *wxliname = NULL, *wxlisol = NULL, *wxli = NULL, *wxlioptions = NULL, *wxlisoloptions = NULL;
  char *rbasname = NULL, *wbasname = NULL;
  char *debugdump_before = NULL;
  char *debugdump_after = NULL;
  char *rparname = NULL;
  char *rparoptions = NULL;
  char *wparname = NULL;
  char *wparoptions = NULL;
  char obj_in_basis = -1;
  MYBOOL ok;
# define SCALINGTHRESHOLD 0.03

  /* read command line arguments */

# if defined FORTIFY
   Fortify_EnterScope();
# endif

  for(i = 1; i < argc; i++) {
    ok = FALSE;
    if(strncmp(argv[i], "-v", 2) == 0) {
      if (argv[i][2])
        verbose = atoi(argv[i] + 2);
      else
        verbose = NORMAL;
    }
    else if(strcmp(argv[i], "-d") == 0)
      debug = TRUE;
    else if(strcmp(argv[i], "-R") == 0)
      report = TRUE;
    else if(strcmp(argv[i], "-i") == 0)
      print_sol = TRUE;
    else if(strcmp(argv[i], "-ia") == 0)
      print_sol = AUTOMATIC;
    else if(strcmp(argv[i], "-nonames") == 0)
      nonames = TRUE;
    else if(strcmp(argv[i], "-norownames") == 0)
      norownames = TRUE;
    else if(strcmp(argv[i], "-nocolnames") == 0)
      nocolnames = TRUE;
    else if((strcmp(argv[i], "-c") == 0) || (strcmp(argv[i], "-cc") == 0))
      floor_first = BRANCH_CEILING;
    else if(strcmp(argv[i], "-cf") == 0)
      floor_first = BRANCH_FLOOR;
    else if(strcmp(argv[i], "-ca") == 0)
      floor_first = BRANCH_AUTOMATIC;
    else if((strcmp(argv[i], "-depth") == 0) && (i + 1 < argc)) {
      do_set_bb_depthlimit = TRUE;
      bb_depthlimit = atoi(argv[++i]);
    }
    else if(strcmp(argv[i], "-Bw") == 0)
      or_value(&bb_rule2, NODE_WEIGHTREVERSEMODE);
    else if(strcmp(argv[i], "-Bb") == 0)
      or_value(&bb_rule2, NODE_BRANCHREVERSEMODE);
    else if(strcmp(argv[i], "-Bg") == 0)
      or_value(&bb_rule2, NODE_GREEDYMODE);
    else if(strcmp(argv[i], "-Bp") == 0)
      or_value(&bb_rule2, NODE_PSEUDOCOSTMODE);
    else if(strcmp(argv[i], "-Bf") == 0)
      or_value(&bb_rule2, NODE_DEPTHFIRSTMODE);
    else if(strcmp(argv[i], "-Br") == 0)
      or_value(&bb_rule2, NODE_RANDOMIZEMODE);
    else if(strcmp(argv[i], "-BG") == 0)
      or_value(&bb_rule2, 0 /* NODE_GUBMODE */); /* doesn't work yet */
    else if(strcmp(argv[i], "-Bd") == 0)
      or_value(&bb_rule2, NODE_DYNAMICMODE);
    else if(strcmp(argv[i], "-Bs") == 0)
      or_value(&bb_rule2, NODE_RESTARTMODE);
    else if(strcmp(argv[i], "-BB") == 0)
      or_value(&bb_rule2, NODE_BREADTHFIRSTMODE);
    else if(strcmp(argv[i], "-Bo") == 0)
      or_value(&bb_rule2, NODE_AUTOORDER);
    else if(strcmp(argv[i], "-Bc") == 0)
      or_value(&bb_rule2, NODE_RCOSTFIXING);
    else if(strcmp(argv[i], "-Bi") == 0)
      or_value(&bb_rule2, NODE_STRONGINIT);
    else if(strncmp(argv[i], "-B", 2) == 0) {
      if (argv[i][2])
        set_value(&bb_rule1, atoi(argv[i] + 2));
      else
        set_value(&bb_rule1, NODE_FIRSTSELECT);
    }
    else if((strcmp(argv[i], "-n") == 0) && (i + 1 < argc)) {
      do_set_solutionlimit = TRUE;
      solutionlimit = atoi(argv[++i]);
    }
    else if((strcmp(argv[i], "-b") == 0) && (i + 1 < argc)) {
      obj_bound = atof(argv[++i]);
      do_set_obj_bound = TRUE;
    }
    else if(((strcmp(argv[i], "-g") == 0) || (strcmp(argv[i], "-ga") == 0)) && (i + 1 < argc))
      mip_absgap = atof(argv[++i]);
    else if((strcmp(argv[i], "-gr") == 0) && (i + 1 < argc))
      mip_relgap = atof(argv[++i]);
    else if((strcmp(argv[i], "-e") == 0) && (i + 1 < argc)) {
      epsint = atof(argv[++i]);
      if((epsint <= 0.0) || (epsint >= 0.5)) {
	fprintf(stderr, "Invalid tolerance %g; 0 < epsilon < 0.5\n",
		(double)epsint);
	EndOfPgr(FORCED_EXIT);
      }
    }
    else if((strcmp(argv[i], "-r") == 0) && (i + 1 < argc))
      max_num_inv = atoi(argv[++i]);
    else if((strcmp(argv[i], "-o") == 0) && (i + 1 < argc)) {
      break_at_value = atof(argv[++i]);
      do_set_break_at_value = TRUE;
    }
    else if(strcmp(argv[i], "-f") == 0)
      break_at_first = TRUE;
    else if(strcmp(argv[i], "-timeoutok") == 0)
      /* timeoutok = TRUE */; /* option no longer needed, but still accepted */
    else if(strcmp(argv[i], "-h") == 0) {
      print_help(argv);
      EndOfPgr(EXIT_SUCCESS);
    }
    else if(strcmp(argv[i], "-prim") == 0)
      preferdual = FALSE;
    else if(strcmp(argv[i], "-dual") == 0)
      preferdual = TRUE;
    else if(strcmp(argv[i], "-simplexpp") == 0)
      simplextype = SIMPLEX_PRIMAL_PRIMAL;
    else if(strcmp(argv[i], "-simplexdp") == 0)
      simplextype = SIMPLEX_DUAL_PRIMAL;
    else if(strcmp(argv[i], "-simplexpd") == 0)
      simplextype = SIMPLEX_PRIMAL_DUAL;
    else if(strcmp(argv[i], "-simplexdd") == 0)
      simplextype = SIMPLEX_DUAL_DUAL;
    else if(strcmp(argv[i], "-sp") == 0)
      or_value(&scalemode2, SCALE_POWER2);
    else if(strcmp(argv[i], "-si") == 0)
      or_value(&scalemode2, SCALE_INTEGERS);
    else if(strcmp(argv[i], "-se") == 0)
      or_value(&scalemode2, SCALE_EQUILIBRATE);
    else if(strncmp(argv[i], "-s", 2) == 0) {
      set_value(&scalemode1, SCALE_NONE);
      scaling = SCALE_MEAN;
      if (argv[i][2]) {
        switch (atoi(argv[i] + 2)) {
	case 0:
	  scaling = SCALE_NONE;
	  break;
        case 1:
	  set_value(&scalemode1, SCALE_GEOMETRIC);
	  break;
	case 2:
	  set_value(&scalemode1, SCALE_CURTISREID);
	  break;
	case 3:
	  set_value(&scalemode1, SCALE_EXTREME);
	  break;
        case 4:
	  set_value(&scalemode1, SCALE_MEAN);
	  break;
	case 5:
	  set_value(&scalemode1, SCALE_MEAN | SCALE_LOGARITHMIC);
	  break;
        case 6:
	  set_value(&scalemode1, SCALE_RANGE);
	  break;
	case 7:
	  set_value(&scalemode1, SCALE_MEAN | SCALE_QUADRATIC);
	  break;
        }
      }
      else
        set_value(&scalemode1, SCALE_MEAN);
      if((i + 1 < argc) && (isNum(argv[i + 1])))
	scaleloop = atoi(argv[++i]);
    }
    else if(strncmp(argv[i], "-C", 2) == 0)
      crashmode = atoi(argv[i] + 2);
    else if(strcmp(argv[i], "-t") == 0)
      tracing = TRUE;
    else if(strncmp(argv[i], "-S", 2) == 0) {
      if (argv[i][2])
        PRINT_SOLUTION = (short) atoi(argv[i] + 2);
      else
        PRINT_SOLUTION = 2;
    }
    else if(strncmp(argv[i], "-improve", 8) == 0) {
      if (argv[i][8])
        or_value(&improve, atoi(argv[i] + 8));
    }
    else if(strcmp(argv[i], "-pivll") == 0)
      or_value(&pivoting2, PRICE_LOOPLEFT);
    else if(strcmp(argv[i], "-pivla") == 0)
      or_value(&pivoting2, PRICE_LOOPALTERNATE);
#if defined EnablePartialOptimization
    else if(strcmp(argv[i], "-pivpc") == 0)
      or_value(&pivoting2, PRICE_AUTOPARTIALCOLS);
    else if(strcmp(argv[i], "-pivpr") == 0)
      or_value(&pivoting2, PRICE_AUTOPARTIALROWS);
    else if(strcmp(argv[i], "-pivp") == 0)
      or_value(&pivoting2, PRICE_AUTOPARTIAL);
#endif
    else if(strcmp(argv[i], "-pivf") == 0)
      or_value(&pivoting2, PRICE_PRIMALFALLBACK);
    else if(strcmp(argv[i], "-pivm") == 0)
      or_value(&pivoting2, PRICE_MULTIPLE);
    else if(strcmp(argv[i], "-piva") == 0)
      or_value(&pivoting2, PRICE_ADAPTIVE);
    else if(strcmp(argv[i], "-pivr") == 0)
      or_value(&pivoting2, PRICE_RANDOMIZE);
    else if(strcmp(argv[i], "-pivh") == 0)
      or_value(&pivoting2, PRICE_HARRISTWOPASS);
    else if(strcmp(argv[i], "-pivt") == 0)
      or_value(&pivoting2, PRICE_TRUENORMINIT);
    else if(strncmp(argv[i], "-piv", 4) == 0) {
      if (argv[i][4])
        set_value(&pivoting1, atoi(argv[i] + 4));
      else
	set_value(&pivoting1, PRICER_DEVEX | PRICE_ADAPTIVE);
    }
#if defined PARSER_LP
    else if(strcmp(argv[i],"-lp") == 0)
      filetype = filetypeLP;
#endif
    else if((strcmp(argv[i],"-wlp") == 0) && (i + 1 < argc))
      wlp = argv[++i];
    else if(strcmp(argv[i],"-mps") == 0)
      filetype = filetypeMPS;
    else if(strcmp(argv[i],"-fmps") == 0)
      filetype = filetypeFREEMPS;
    else if((strcmp(argv[i],"-wmps") == 0) && (i + 1 < argc))
      wmps = argv[++i];
    else if((strcmp(argv[i],"-wfmps") == 0) && (i + 1 < argc))
      wfmps = argv[++i];
    else if(strcmp(argv[i],"-wafter") == 0)
      write_model_after = TRUE;
    else if(strcmp(argv[i],"-degen") == 0)
      set_value(&anti_degen1, ANTIDEGEN_DEFAULT);
    else if(strcmp(argv[i],"-degenf") == 0)
      or_value(&anti_degen2, ANTIDEGEN_FIXEDVARS);
    else if(strcmp(argv[i],"-degenc") == 0)
      or_value(&anti_degen2, ANTIDEGEN_COLUMNCHECK);
    else if(strcmp(argv[i],"-degens") == 0)
      or_value(&anti_degen2, ANTIDEGEN_STALLING);
    else if(strcmp(argv[i],"-degenn") == 0)
      or_value(&anti_degen2, ANTIDEGEN_NUMFAILURE);
    else if(strcmp(argv[i],"-degenl") == 0)
      or_value(&anti_degen2, ANTIDEGEN_LOSTFEAS);
    else if(strcmp(argv[i],"-degeni") == 0)
      or_value(&anti_degen2, ANTIDEGEN_INFEASIBLE);
    else if(strcmp(argv[i],"-degend") == 0)
      or_value(&anti_degen2, ANTIDEGEN_DYNAMIC);
    else if(strcmp(argv[i],"-degenb") == 0)
      or_value(&anti_degen2, ANTIDEGEN_DURINGBB);
    else if(strcmp(argv[i],"-degenr") == 0)
      or_value(&anti_degen2, ANTIDEGEN_RHSPERTURB);
    else if(strcmp(argv[i],"-degenp") == 0)
      or_value(&anti_degen2, ANTIDEGEN_BOUNDFLIP);
    else if(strcmp(argv[i],"-time") == 0) {
      if(clock() == -1)
	fprintf(stderr, "CPU times not available on this machine\n");
      else
	print_timing = TRUE;
    }
    else if((strcmp(argv[i],"-bfp") == 0) && (i + 1 < argc))
      bfp = argv[++i];
    else if((strcmp(argv[i],"-rxli") == 0) && (i + 2 < argc)) {
      rxliname = argv[++i];
      rxli = argv[++i];
      fpin = NULL;
      filetype = filetypeXLI;
    }
    else if((strcmp(argv[i],"-rxlidata") == 0) && (i + 1 < argc))
      rxlidata = argv[++i];
    else if((strcmp(argv[i],"-rxliopt") == 0) && (i + 1 < argc))
      rxlioptions = argv[++i];
    else if((strcmp(argv[i],"-wxli") == 0) && (i + 2 < argc)) {
      wxliname = argv[++i];
      wxli = argv[++i];
    }
    else if((strcmp(argv[i],"-wxliopt") == 0) && (i + 1 < argc))
      wxlioptions = argv[++i];
    else if((strcmp(argv[i],"-wxlisol") == 0) && (i + 2 < argc)) {
      wxliname = argv[++i];
      wxlisol = argv[++i];
    }
    else if((strcmp(argv[i],"-wxlisolopt") == 0) && (i + 1 < argc))
      wxlisoloptions = argv[++i];
    else if((strcmp(argv[i],"-rbas") == 0) && (i + 1 < argc))
      rbasname = argv[++i];
    else if((strcmp(argv[i],"-wbas") == 0) && (i + 1 < argc))
      wbasname = argv[++i];
    else if((strcmp(argv[i],"-Db") == 0) && (i + 1 < argc))
      debugdump_before = argv[++i];
    else if((strcmp(argv[i],"-Da") == 0) && (i + 1 < argc))
      debugdump_after = argv[++i];
    else if((strcmp(argv[i],"-timeout") == 0) && (i + 1 < argc))
      sectimeout = atol(argv[++i]);
    else if((strcmp(argv[i],"-trej") == 0) && (i + 1 < argc))
      epspivot = atof(argv[++i]);
    else if((strcmp(argv[i],"-epsp") == 0) && (i + 1 < argc))
      epsperturb = atof(argv[++i]);
    else if((strcmp(argv[i],"-epsd") == 0) && (i + 1 < argc))
      epsd = atof(argv[++i]);
    else if((strcmp(argv[i],"-epsb") == 0) && (i + 1 < argc))
      epsb = atof(argv[++i]);
    else if((strcmp(argv[i],"-epsel") == 0) && (i + 1 < argc))
      epsel = atof(argv[++i]);
    else if(strcmp(argv[i],"-parse_only") == 0)
      parse_only = TRUE;
    else
      ok = TRUE;

    if(!ok)
      ;
    else if(strcmp(argv[i],"-presolverow") == 0)
      or_value(&do_presolve, PRESOLVE_ROWS);
    else if(strcmp(argv[i],"-presolvecol") == 0)
      or_value(&do_presolve, PRESOLVE_COLS);
    else if(strcmp(argv[i],"-presolve") == 0)
      or_value(&do_presolve, PRESOLVE_ROWS | PRESOLVE_COLS);
    else if(strcmp(argv[i],"-presolvel") == 0)
      or_value(&do_presolve, PRESOLVE_LINDEP);
    else if(strcmp(argv[i],"-presolves") == 0)
      or_value(&do_presolve, PRESOLVE_SOS);
    else if(strcmp(argv[i],"-presolver") == 0)
      or_value(&do_presolve, PRESOLVE_REDUCEMIP);
    else if(strcmp(argv[i],"-presolvek") == 0)
      or_value(&do_presolve, PRESOLVE_KNAPSACK);
    else if(strcmp(argv[i],"-presolveq") == 0)
      or_value(&do_presolve, PRESOLVE_ELIMEQ2);
    else if(strcmp(argv[i],"-presolvem") == 0)
      or_value(&do_presolve, PRESOLVE_MERGEROWS);
    else if(strcmp(argv[i],"-presolvefd") == 0)
      or_value(&do_presolve, PRESOLVE_COLFIXDUAL);
    else if(strcmp(argv[i],"-presolvebnd") == 0)
      or_value(&do_presolve, PRESOLVE_BOUNDS);
    else if(strcmp(argv[i],"-presolved") == 0)
      or_value(&do_presolve, PRESOLVE_DUALS);
    else if(strcmp(argv[i],"-presolvef") == 0)
      or_value(&do_presolve, PRESOLVE_IMPLIEDFREE);
    else if(strcmp(argv[i],"-presolveslk") == 0)
      or_value(&do_presolve, PRESOLVE_IMPLIEDSLK);
    else if(strcmp(argv[i],"-presolveg") == 0)
      or_value(&do_presolve, PRESOLVE_REDUCEGCD);
    else if(strcmp(argv[i],"-presolveb") == 0)
      or_value(&do_presolve, PRESOLVE_PROBEFIX);
    else if(strcmp(argv[i],"-presolvec") == 0)
      or_value(&do_presolve, PRESOLVE_PROBEREDUCE);
    else if(strcmp(argv[i],"-presolverowd") == 0)
      or_value(&do_presolve, PRESOLVE_ROWDOMINATE);
    else if(strcmp(argv[i],"-presolvecold") == 0)
      or_value(&do_presolve, PRESOLVE_COLDOMINATE);
    else if(strcmp(argv[i],"-min") == 0)
      objective = -1;
    else if(strcmp(argv[i],"-max") == 0)
      objective =  1;
    else if(strcmp(argv[i],"-noint") == 0)
      noint =  TRUE;
    else if((strcmp(argv[i],"-rpar") == 0) && (i + 1 < argc))
      i++;
    else if((strcmp(argv[i],"-rparopt") == 0) && (i + 1 < argc))
      i++;
    else if((strcmp(argv[i],"-wpar") == 0) && (i + 1 < argc))
      i++;
    else if((strcmp(argv[i],"-wparopt") == 0) && (i + 1 < argc))
      i++;
	else if(strcmp(argv[i],"-o0") == 0)
	  obj_in_basis = FALSE;
	else if(strcmp(argv[i],"-o1") == 0)
	  obj_in_basis = TRUE;
    else if(fpin == stdin) {
      filen = argv[i];
      if(*filen == '<')
        filen++;
      if((fpin = fopen(filen, "r")) == NULL) {
	print_help(argv);
	fprintf(stderr,"\nError, Unable to open input file '%s'\n",
		argv[i]);
	EndOfPgr(FORCED_EXIT);
      }
    }
    else {
      filen = argv[i];
      if(*filen != '>') {
        print_help(argv);
        fprintf(stderr, "\nError, Unrecognized command line argument '%s'\n",
		argv[i]);
        EndOfPgr(FORCED_EXIT);
      }
    }
  }

  signal(SIGABRT,/* (void (*) OF((int))) */ SIGABRT_func);

  switch(filetype) {
#if defined PARSER_LP
  case filetypeLP:
    lp = read_lp(fpin, verbose, NULL);
    break;
#endif
  case filetypeMPS:
    lp = read_mps(fpin, verbose);
    break;
  case filetypeFREEMPS:
    lp = read_freemps(fpin, verbose);
    break;
  case filetypeXLI:
    lp = read_XLI(rxliname, rxli, rxlidata, rxlioptions, verbose);
    break;
  }

  if((fpin != NULL) && (fpin != stdin))
    fclose(fpin);

  if(print_timing)
    print_cpu_times("Parsing input");

  if(lp == NULL) {
    fprintf(stderr, "Unable to read model.\n");
    EndOfPgr(FORCED_EXIT);
  }

  for(i = 1; i < argc; i++) {
    if((strcmp(argv[i],"-rpar") == 0) && (i + 1 < argc)) {
      if(rparname != NULL) {
	if(!read_params(lp, rparname, rparoptions)) {
	  fprintf(stderr, "Unable to read parameter file (%s)\n", rparname);
	  delete_lp(lp);
	  EndOfPgr(FORCED_EXIT);
	}
      }
      rparname = argv[++i];
    }
    else if((strcmp(argv[i],"-rparopt") == 0) && (i + 1 < argc))
      rparoptions = argv[++i];
    else if((strcmp(argv[i],"-wpar") == 0) && (i + 1 < argc))
      wparname = argv[++i];
    else if((strcmp(argv[i],"-wparopt") == 0) && (i + 1 < argc))
      wparoptions = argv[++i];
  }

  if(rparname != NULL)
    if(!read_params(lp, rparname, rparoptions)) {
      fprintf(stderr, "Unable to read parameter file (%s)\n", rparname);
      delete_lp(lp);
      EndOfPgr(FORCED_EXIT);
    }

  if((nonames) || (nocolnames))
    set_use_names(lp, FALSE, FALSE);
  if((nonames) || (norownames))
    set_use_names(lp, TRUE, FALSE);

  if(objective != 0) {
    if(objective == 1)
      set_maxim(lp);
    else
      set_minim(lp);
  }

  if (obj_in_basis != -1)
    set_obj_in_basis(lp, obj_in_basis);

  if(noint) { /* remove integer conditions */
    for(i = get_Ncolumns(lp); i >= 1; i--) {
      if(is_SOS_var(lp, i)) {
        fprintf(stderr, "Unable to remove integer conditions because there is at least one SOS constraint\n");
	delete_lp(lp);
	EndOfPgr(FORCED_EXIT);
      }
      set_semicont(lp, i, FALSE);
      set_int(lp, i, FALSE);
    }
  }

  if(!write_model_after)
    write_model(lp, wlp, wmps, wfmps, wxli, NULL, wxliname, wxlioptions);

  if(parse_only) {
    if(!write_model_after) {
      delete_lp(lp);
      EndOfPgr(0);
    }
    /* else if(!sectimeout) */
      sectimeout = 1;
  }

  if(PRINT_SOLUTION >= 5)
    print_lp(lp);

#if 0
  put_abortfunc(lp,(abortfunc *) myabortfunc, NULL);
#endif

  if(sectimeout > 0)
    set_timeout(lp, sectimeout);
  if(print_sol >= 0)
    set_print_sol(lp, print_sol);
  if(epsint >= 0)
    set_epsint(lp, epsint);
  if(epspivot >= 0)
    set_epspivot(lp, epspivot);
  if(epsperturb >= 0)
    set_epsperturb(lp, epsperturb);
  if(epsd >= 0)
    set_epsd(lp, epsd);
  if(epsb >= 0)
    set_epsb(lp, epsb);
  if(epsel >= 0)
    set_epsel(lp, epsel);
  if(debug >= 0)
    set_debug(lp, (MYBOOL) debug);
  if(floor_first != -1)
    set_bb_floorfirst(lp, floor_first);
  if(do_set_bb_depthlimit)
    set_bb_depthlimit(lp, bb_depthlimit);
  if(do_set_solutionlimit)
    set_solutionlimit(lp, solutionlimit);
  if(tracing)
    set_trace(lp, tracing);
  if(do_set_obj_bound)
    set_obj_bound(lp, obj_bound);
  if(do_set_break_at_value)
    set_break_at_value(lp, break_at_value);
  if(break_at_first)
    set_break_at_first(lp, break_at_first);
  if(mip_absgap >= 0)
    set_mip_gap(lp, TRUE, mip_absgap);
  if(mip_relgap >= 0)
    set_mip_gap(lp, FALSE, mip_relgap);
  if((anti_degen1 != -1) || (anti_degen2 != -1)) {
    if((anti_degen1 == -1) || (anti_degen2 != -1))
      anti_degen1 = 0;
    if(anti_degen2 == -1)
      anti_degen2 = 0;
    set_anti_degen(lp, anti_degen1 | anti_degen2);
  }
  set_presolve(lp, ((do_presolve == -1) ? get_presolve(lp): do_presolve) | ((PRINT_SOLUTION >= 4) ? PRESOLVE_SENSDUALS : 0), get_presolveloops(lp));
  if(improve != -1)
    set_improve(lp, improve);
  if(max_num_inv >= 0)
    set_maxpivot(lp, max_num_inv);
  if(preferdual != AUTOMATIC)
    set_preferdual(lp, preferdual);
  if((pivoting1 != -1) || (pivoting2 != -1)) {
    if(pivoting1 == -1)
      pivoting1 = get_pivoting(lp) & PRICER_LASTOPTION;
    if(pivoting2 == -1)
      pivoting2 = 0;
    set_pivoting(lp, pivoting1 | pivoting2);
  }
  if((scalemode1 != -1) || (scalemode2 != -1)) {
    if(scalemode1 == -1)
      scalemode1 = get_scaling(lp) & SCALE_CURTISREID;
    if(scalemode2 == -1)
      scalemode2 = 0;
    set_scaling(lp, scalemode1 | scalemode2);
  }
  if(crashmode != -1)
    set_basiscrash(lp, crashmode);
  if((bb_rule1 != -1) || (bb_rule2 != -1)) {
    if(bb_rule1 == -1)
      bb_rule1 = get_bb_rule(lp) & NODE_USERSELECT;
    if(bb_rule2 == -1)
      bb_rule2 = 0;
    set_bb_rule(lp, bb_rule1 | bb_rule2);
  }
  if(simplextype != -1)
    set_simplextype(lp, simplextype);
  if(bfp != NULL)
    if(!set_BFP(lp, bfp)) {
      fprintf(stderr, "Unable to set BFP package.\n");
      delete_lp(lp);
      EndOfPgr(FORCED_EXIT);
    }
  if(debugdump_before != NULL)
    print_debugdump(lp, debugdump_before);
  if(report)
    put_msgfunc(lp, LPMessageCB, NULL, MSG_LPFEASIBLE | MSG_LPOPTIMAL | MSG_MILPFEASIBLE | MSG_MILPBETTER | MSG_PERFORMANCE);

  if(scaling) {
    if(scaleloop <= 0)
      scaleloop = 5;
    if(scaleloop - (int) scaleloop < SCALINGTHRESHOLD)
      scaleloop = (int) scaleloop + SCALINGTHRESHOLD;
    set_scalelimit(lp, scaleloop);
  }

  if(rbasname != NULL)
    if(!read_basis(lp, rbasname, NULL)) {
      fprintf(stderr, "Unable to read basis file.\n");
      delete_lp(lp);
      EndOfPgr(FORCED_EXIT);
    }

  result = solve(lp);

  if(wbasname != NULL)
    if(!write_basis(lp, wbasname))
      fprintf(stderr, "Unable to write basis file.\n");

  if(write_model_after)
    write_model(lp, wlp, wmps, wfmps, wxli, NULL, wxliname, wxlioptions);

  write_model(lp, NULL, NULL, NULL, NULL, wxlisol, wxliname, wxlisoloptions);

  if(PRINT_SOLUTION >= 6)
    print_scales(lp);

  if((print_timing) && (!parse_only))
    print_cpu_times("solving");

  if(debugdump_after != NULL)
    print_debugdump(lp, debugdump_after);

  if(wparname != NULL)
    if(!write_params(lp, wparname, wparoptions)) {
      fprintf(stderr, "Unable to write parameter file (%s)\n", wparname);
      delete_lp(lp);
      EndOfPgr(FORCED_EXIT);
    }

  if(parse_only) {
    delete_lp(lp);
    EndOfPgr(0);
  }

/*
  if((timeoutok) && (result == TIMEOUT) && (get_solutioncount(lp) > 0))
    result = OPTIMAL;
*/

  switch(result) {
  case SUBOPTIMAL:
  case PRESOLVED:
  case OPTIMAL:
  case PROCBREAK:
  case FEASFOUND:
    if ((result == SUBOPTIMAL) && (PRINT_SOLUTION >= 1))
      printf("Suboptimal solution\n");

    if (result == PRESOLVED)
      printf("Presolved solution\n");

    if (PRINT_SOLUTION >= 1)
      print_objective(lp);

    if (PRINT_SOLUTION >= 2)
      print_solution(lp, 1);

    if (PRINT_SOLUTION >= 3)
      print_constraints(lp, 1);

    if (PRINT_SOLUTION >= 4)
      print_duals(lp);

    if(tracing)
      fprintf(stderr,
      "Branch & Bound depth: %d\nNodes processed: %.0f\nSimplex pivots: %.0f\nNumber of equal solutions: %d\n",
	      get_max_level(lp), (REAL) get_total_nodes(lp), (REAL) get_total_iter(lp), get_solutioncount(lp));
    break;
  case NOMEMORY:
    if (PRINT_SOLUTION >= 1)
      printf("Out of memory\n");
    break;
  case INFEASIBLE:
    if (PRINT_SOLUTION >= 1)
      printf("This problem is infeasible\n");
    break;
  case UNBOUNDED:
    if (PRINT_SOLUTION >= 1)
      printf("This problem is unbounded\n");
    break;
  case PROCFAIL:
   if (PRINT_SOLUTION >= 1)
      printf("The B&B routine failed\n");
    break;
  case TIMEOUT:
    if (PRINT_SOLUTION >= 1)
      printf("Timeout\n");
    break;
  case USERABORT:
    if (PRINT_SOLUTION >= 1)
      printf("User aborted\n");
    break;
  default:
    if (PRINT_SOLUTION >= 1)
      printf("lp_solve failed\n");
    break;
  }

  if (PRINT_SOLUTION >= 7)
    print_tableau(lp);

  delete_lp(lp);

  EndOfPgr(result);
  return(result);
}
