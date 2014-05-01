
/* ----------------------------------------------------------------------------------
   Main library of routines for lp_solve v5.0+
   ----------------------------------------------------------------------------------
    Author:        Michel Berkelaar (to v3.2)
                   Kjell Eikland    (v4.0 and forward)
    Contact:       kjell.eikland@broadpark.no
    License terms: LGPL.

    Requires:      (see below)

    Release notes:
    v5.0.0  1 January 2004      First integrated and repackaged version.
    v5.0.1  8 May 2004          Cumulative update since initial release;
                                overall functionality scope maintained.
    v5.1.0  20 July 2004        Reworked lp_solve throughout to fit new
                                flexible matrix storage model.

   ---------------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------------- */
/* Main library of routines for lp_solve                                              */
/*----------------------------------------------------------------------------------- */
#include <signal.h>
#include <string.h>
#include <float.h>
#include <math.h>

#if LoadInverseLib == TRUE
  #ifdef WIN32
    #include <windows.h>
  #else
    #include <dlfcn.h>
  #endif
#endif


/* ---------------------------------------------------------------------------------- */
/* Include core and support modules via headers                                       */
/* ---------------------------------------------------------------------------------- */
#include "lp_lib.h"
#include "commonlib.h"
#include "lp_utils.h"
#include "lp_matrix.h"
#include "lp_SOS.h"
#include "lp_Hash.h"
#include "lp_MPS.h"
#include "lp_wlp.h"
#include "lp_presolve.h"
#include "lp_scale.h"
#include "lp_simplex.h"
#include "lp_mipbb.h"
#include "lp_report.h"
#include "lp_MDO.h"

#if INVERSE_ACTIVE==INVERSE_LUMOD
  #include "lp_LUMOD.h"
#elif INVERSE_ACTIVE==INVERSE_LUSOL
  #include "lp_LUSOL.h"
#elif INVERSE_ACTIVE==INVERSE_GLPKLU
  #include "lp_glpkLU.h"
#elif INVERSE_ACTIVE==INVERSE_ETAPFI
  #include "lp_etaPFI.h"
#elif INVERSE_ACTIVE==INVERSE_LEGACY
  #include "lp_etaPFI.h"
#endif

#if libBLAS > 0
  #include "myblas.h"
#endif

#ifdef __BORLANDC__
  #pragma hdrstop
  #pragma package(smart_init)
#endif

/* ---------------------------------------------------------------------------------- */
/* Include selected basis inverse routines and price norm scalars                     */
/* ---------------------------------------------------------------------------------- */

#include "lp_price.h"
#include "lp_pricePSE.h"

#ifdef FORTIFY
# include "lp_fortify.h"
#endif


/* ---------------------------------------------------------------------------------- */
/* Define some globals                                                                */
/* ---------------------------------------------------------------------------------- */
int callcount = 0;

/* Return lp_solve version information */
void __WINAPI lp_solve_version(int *majorversion, int *minorversion, int *release, int *build)
{
  if(majorversion != NULL)
    (*majorversion) = MAJORVERSION;
  if(minorversion != NULL)
    (*minorversion) = MINORVERSION;
  if(release != NULL)
    (*release) = RELEASE;
  if(build != NULL)
    (*build) = BUILD;
}


/* ---------------------------------------------------------------------------------- */
/* Various interaction elements                                                       */
/* ---------------------------------------------------------------------------------- */

MYBOOL __WINAPI userabort(lprec *lp, int message)
{
  static MYBOOL abort;
  static int spx_save;

  spx_save = lp->spx_status;
  lp->spx_status = RUNNING;
  if(yieldformessages(lp) != 0) {
    lp->spx_status = USERABORT;
    if(lp->bb_level > 0)
      lp->bb_break = TRUE;
  }
  if((message > 0) && (lp->usermessage != NULL) && (lp->msgmask & message))
    lp->usermessage(lp, lp->msghandle, message);
  abort = (MYBOOL) (lp->spx_status != RUNNING);
  if(!abort)
    lp->spx_status = spx_save;
  return( abort );
}

STATIC int yieldformessages(lprec *lp)
{
  static double currenttime;

  if((lp->sectimeout > 0) &&
     (((currenttime = timeNow()) -lp->timestart)-(REAL)lp->sectimeout>0))
    lp->spx_status = TIMEOUT;

  if(lp->ctrlc != NULL) {
    int retcode = lp->ctrlc(lp, lp->ctrlchandle);
    /* Check for command to restart the B&B */
    if((retcode == ACTION_RESTART) && (lp->bb_level > 1)) {
      lp->bb_break = AUTOMATIC;
      retcode = 0;
    }
    return(retcode);
  }
  else
    return(0);
}

void __WINAPI set_outputstream(lprec *lp, FILE *stream)
{
  if((lp->outstream != NULL) && (lp->outstream != stdout)) {
    if(lp->streamowned)
      fclose(lp->outstream);
    else
      fflush(lp->outstream);
  }
  if(stream == NULL)
    lp->outstream = stdout;
  else
    lp->outstream = stream;
  lp->streamowned = FALSE;
}

MYBOOL __WINAPI set_outputfile(lprec *lp, char *filename)
{
  MYBOOL ok;
  FILE   *output = stdout;

  ok = (MYBOOL) ((filename == NULL) || (*filename == 0) || ((output = fopen(filename,"w")) != NULL));
  if(ok) {
    set_outputstream(lp, output);
    lp->streamowned = (MYBOOL) ((filename != NULL) && (*filename != 0));
#if 1
    if((filename != NULL) && (*filename == 0))
      lp->outstream = NULL;
#endif
  }
  return(ok);
}

REAL __WINAPI time_elapsed(lprec *lp)
{
  if(lp->timeend > 0)
    return(lp->timeend - lp->timestart);
  else
    return(timeNow() - lp->timestart);
}

void __WINAPI put_bb_nodefunc(lprec *lp, lphandleint_intfunc newnode, void *bbnodehandle)
{
  lp->bb_usenode = newnode;
  lp->bb_nodehandle = bbnodehandle;         /* User-specified "owner process ID" */
}
void __WINAPI put_bb_branchfunc(lprec *lp, lphandleint_intfunc newbranch, void *bbbranchhandle)
{
  lp->bb_usebranch = newbranch;
  lp->bb_branchhandle = bbbranchhandle;     /* User-specified "owner process ID" */
}
void __WINAPI put_abortfunc(lprec *lp, lphandle_intfunc newctrlc, void *ctrlchandle)
{
  lp->ctrlc = newctrlc;
  lp->ctrlchandle = ctrlchandle;            /* User-specified "owner process ID" */
}
void __WINAPI put_logfunc(lprec *lp, lphandlestr_func newlog, void *loghandle)
{
  lp->writelog = newlog;
  lp->loghandle = loghandle;                /* User-specified "owner process ID" */
}
void __WINAPI put_msgfunc(lprec *lp, lphandleint_func newmsg, void *msghandle, int mask)
{
  lp->usermessage = newmsg;
  lp->msghandle = msghandle;                /* User-specified "owner process ID" */
  lp->msgmask = mask;
}


/* ---------------------------------------------------------------------------------- */
/* DLL exported function                                                              */
/* ---------------------------------------------------------------------------------- */
lprec * __WINAPI read_MPS(char *filename, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readfile(&lp, filename, MPSFIXED, verbose))
    return( lp );
  else
    return( NULL );
}
lprec * __WINAPI read_mps(FILE *filename, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readhandle(&lp, filename, MPSFIXED, verbose))
    return( lp );
  else
    return( NULL );
}
#if defined develop
lprec * __WINAPI read_mpsex(void *userhandle, read_modeldata_func read_modeldata, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readex(&lp, userhandle, read_modeldata, MPSFIXED, verbose))
    return( lp );
  else
    return( NULL );
}
#endif
lprec * __WINAPI read_freeMPS(char *filename, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readfile(&lp, filename, MPSFREE, verbose))
    return( lp );
  else
    return( NULL );
}
lprec * __WINAPI read_freemps(FILE *filename, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readhandle(&lp, filename, MPSFREE, verbose))
    return( lp );
  else
    return( NULL );
}
#if defined develop
lprec * __WINAPI read_freempsex(void *userhandle, read_modeldata_func read_modeldata, int verbose)
{
  lprec *lp = NULL;

  if(MPS_readex(&lp, userhandle, read_modeldata, MPSFREE, verbose))
    return( lp );
  else
    return( NULL );
}
#endif
MYBOOL __WINAPI write_mps(lprec *lp, char *filename)
{
  return(MPS_writefile(lp, MPSFIXED, filename));
}
MYBOOL __WINAPI write_MPS(lprec *lp, FILE *output)
{
  return(MPS_writehandle(lp, MPSFIXED, output));
}

MYBOOL __WINAPI write_freemps(lprec *lp, char *filename)
{
  return(MPS_writefile(lp, MPSFREE, filename));
}
MYBOOL __WINAPI write_freeMPS(lprec *lp, FILE *output)
{
  return(MPS_writehandle(lp, MPSFREE, output));
}

MYBOOL __WINAPI write_lp(lprec *lp, char *filename)
{
  return(LP_writefile(lp, filename));
}
MYBOOL __WINAPI write_LP(lprec *lp, FILE *output)
{
  return(LP_writehandle(lp, output));
}
#ifndef PARSER_LP
MYBOOL __WINAPI LP_readhandle(lprec **lp, FILE *filename, int verbose, char *lp_name)
{
  return(FALSE);
}
lprec * __WINAPI read_lp(FILE *filename, int verbose, char *lp_name)
{
  return(NULL);
}
lprec * __WINAPI read_LP(char *filename, int verbose, char *lp_name)
{
  return(NULL);
}
#endif

MYBOOL __WINAPI write_basis(lprec *lp, char *filename)
{
  int typeMPS = MPSFIXED;
  return( MPS_writeBAS(lp, typeMPS, filename) );
}
MYBOOL __WINAPI read_basis(lprec *lp, char *filename, char *info)
{
  int typeMPS = MPSFIXED;

  typeMPS = MPS_readBAS(lp, typeMPS, filename, info);

  /* Code basis */
  if(typeMPS) {
    set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
    lp->basis_valid = TRUE;   /* Do not re-initialize basis on entering Solve */
    lp->var_basic[0] = FALSE; /* Set to signal that this is a non-default basis */
  }
  return( (MYBOOL) typeMPS );
}

/* Write and read lp_solve parameters (placeholders) - see lp_params.c */
void __WINAPI reset_params(lprec *lp)
{
  int mode;

  lp->epsmachine        = DEF_EPSMACHINE;
  lp->epsperturb        = DEF_PERTURB;
  lp->lag_accept        = DEF_LAGACCEPT;
  set_epslevel(lp, EPS_DEFAULT);

  lp->tighten_on_set    = FALSE;
  lp->negrange          = DEF_NEGRANGE;

#if 0
  lp->do_presolve       = PRESOLVE_ROWS | PRESOLVE_COLS | PRESOLVE_MERGEROWS |
                          PRESOLVE_REDUCEGCD |
                          PRESOLVE_ROWDOMINATE;
#else
  lp->do_presolve       = PRESOLVE_NONE;
#endif
  lp->presolveloops     = DEF_MAXPRESOLVELOOPS;

  lp->scalelimit        = DEF_SCALINGLIMIT;
  lp->scalemode         = SCALE_INTEGERS |
#if 0
                          SCALE_POWER2 |
                          SCALE_LOGARITHMIC | SCALE_MEAN;
#else
                          SCALE_LINEAR | SCALE_GEOMETRIC |
                          SCALE_EQUILIBRATE;
#endif

  lp->crashmode         = CRASH_NONE;

  lp->max_pivots        = 0;
  lp->simplex_strategy  = SIMPLEX_DUAL_PRIMAL;
#define PricerDefaultOpt 1
#if PricerDefaultOpt == 1
  mode = PRICER_DEVEX;
#elif PricerDefaultOpt == 2
  mode = PRICER_STEEPESTEDGE;
  mode |= PRICE_TRUENORMINIT;
#else
  mode = PRICER_STEEPESTEDGE | PRICE_PRIMALFALLBACK;
#endif
  mode |= PRICE_ADAPTIVE;
#ifdef EnableRandomizedPricing
  mode |= PRICE_RANDOMIZE;
#endif
  set_pivoting(lp, mode);

  lp->improve           = IMPROVE_DEFAULT;
  lp->anti_degen        = ANTIDEGEN_DEFAULT;

  lp->bb_floorfirst     = BRANCH_AUTOMATIC;
  lp->bb_rule           = NODE_DYNAMICMODE | NODE_GREEDYMODE | NODE_GAPSELECT |
#if 1
                          NODE_PSEUDOCOSTSELECT |
#else
                          NODE_PSEUDOFEASSELECT |
#endif
                          NODE_RCOSTFIXING;
  lp->bb_limitlevel     = DEF_BB_LIMITLEVEL;
  lp->bb_PseudoUpdates  = DEF_PSEUDOCOSTUPDATES;

  lp->bb_heuristicOF    = my_chsign(is_maxim(lp), MAX(DEF_INFINITE, lp->infinite));
  lp->bb_breakOF        = -lp->bb_heuristicOF;

  lp->sectimeout        = 0;
  lp->solutionlimit     = 1;

  set_outputstream(lp, NULL);          /* Set to default output stream */
  lp->verbose           = NORMAL;
  lp->print_sol         = FALSE;       /* Can be FALSE, TRUE, AUTOMATIC (only non-zeros printed) */
  lp->spx_trace         = FALSE;
  lp->lag_trace         = FALSE;
  lp->bb_trace          = FALSE;
}

void __WINAPI unscale(lprec *lp)
{
  undoscale(lp);
}
int __WINAPI solve(lprec *lp)
{
#if defined FPUexception
  catchFPU(_EM_INVALID | _EM_ZERODIVIDE | _EM_OVERFLOW | _EM_UNDERFLOW);
#endif

  if(has_BFP(lp)) {
    lp->solvecount++;
    if(is_add_rowmode(lp))
      set_add_rowmode(lp, FALSE);
    return(lin_solve(lp));
  }
  else
    return( NOBFP );
}
void __WINAPI print_lp(lprec *lp)
{
  REPORT_lp(lp);
}
void __WINAPI print_tableau(lprec *lp)
{
  REPORT_tableau(lp);
}
void __WINAPI print_objective(lprec *lp)
{
  REPORT_objective(lp);
}
void __WINAPI print_solution(lprec *lp, int columns)
{
  REPORT_solution(lp, columns);
}
void __WINAPI print_constraints(lprec *lp, int columns)
{
  REPORT_constraints(lp, columns);
}
void __WINAPI print_duals(lprec *lp)
{
  REPORT_duals(lp);
}
void __WINAPI print_scales(lprec *lp)
{
  REPORT_scales(lp);
}
MYBOOL __WINAPI print_debugdump(lprec *lp, char *filename)
{
  return(REPORT_debugdump(lp, filename, (MYBOOL) (get_total_iter(lp) > 0)));
}
void __WINAPI print_str(lprec *lp, char *str)
{
  report(lp, lp->verbose, "%s", str);
}



/* ---------------------------------------------------------------------------------- */
/* Parameter setting and retrieval functions                                          */
/* ---------------------------------------------------------------------------------- */

void __WINAPI set_timeout(lprec *lp, long sectimeout)
{
  lp->sectimeout = sectimeout;
}

long __WINAPI get_timeout(lprec *lp)
{
  return(lp->sectimeout);
}

void __WINAPI set_verbose(lprec *lp, int verbose)
{
  lp->verbose = verbose;
}

int __WINAPI get_verbose(lprec *lp)
{
  return(lp->verbose);
}

void __WINAPI set_print_sol(lprec *lp, int print_sol)
{
  lp->print_sol = print_sol;
}

int __WINAPI get_print_sol(lprec *lp)
{
  return(lp->print_sol);
}

void __WINAPI set_debug(lprec *lp, MYBOOL debug)
{
  lp->bb_trace = debug;
}

MYBOOL __WINAPI is_debug(lprec *lp)
{
  return(lp->bb_trace);
}

void __WINAPI set_trace(lprec *lp, MYBOOL trace)
{
  lp->spx_trace = trace;
}

MYBOOL __WINAPI is_trace(lprec *lp)
{
  return(lp->spx_trace);
}

void __WINAPI set_anti_degen(lprec *lp, int anti_degen)
{
  lp->anti_degen = anti_degen;
}

int __WINAPI get_anti_degen(lprec *lp)
{
  return(lp->anti_degen);
}

MYBOOL __WINAPI is_anti_degen(lprec *lp, int testmask)
{
  return((MYBOOL) ((lp->anti_degen == testmask) || ((lp->anti_degen & testmask) != 0)));
}

void __WINAPI set_presolve(lprec *lp, int presolvemode, int maxloops)
{
  presolvemode &= ~PRESOLVE_REDUCEMIP; /* disable PRESOLVE_REDUCEMIP since it is very rare that this is effective, and also that it adds code complications and delayed presolve effects that are not captured properly. */
  lp->do_presolve = presolvemode;
  lp->presolveloops = maxloops;
}

int __WINAPI get_presolve(lprec *lp)
{
  return(lp->do_presolve);
}

int __WINAPI get_presolveloops(lprec *lp)
{
  if(lp->presolveloops < 0)
    return(DEF_MAXPRESOLVELOOPS);
  else if(lp->presolveloops == 0)
    return(MAXINT32);
  else
    return(lp->presolveloops);
}

MYBOOL __WINAPI is_presolve(lprec *lp, int testmask)
{
  return((MYBOOL) ((lp->do_presolve == testmask) || ((lp->do_presolve & testmask) != 0)));
}

void __WINAPI set_maxpivot(lprec *lp, int maxpivot)
{
  lp->max_pivots = maxpivot;
}

int __WINAPI get_maxpivot(lprec *lp)
{
  return( lp->bfp_pivotmax(lp) );
}

void __WINAPI set_bb_rule(lprec *lp, int bb_rule)
{
  lp->bb_rule = bb_rule;
}

int __WINAPI get_bb_rule(lprec *lp)
{
  return(lp->bb_rule);
}

INLINE MYBOOL is_bb_rule(lprec *lp, int bb_rule)
{
  return( (MYBOOL) ((lp->bb_rule & NODE_STRATEGYMASK) == bb_rule) );
}

/* INLINE */ MYBOOL is_bb_mode(lprec *lp, int bb_mask)
{
  return( (MYBOOL) ((lp->bb_rule & bb_mask) > 0) );
}

void __WINAPI set_action(int *actionvar, int actionmask)
{
  *actionvar |= actionmask;
}

void __WINAPI clear_action(int *actionvar, int actionmask)
{
  *actionvar &= ~actionmask;
}

MYBOOL __WINAPI is_action(int actionvar, int testmask)
{
  return( (MYBOOL) ((actionvar & testmask) != 0) );
}

void __WINAPI set_bb_depthlimit(lprec *lp, int bb_maxlevel)
{
  lp->bb_limitlevel = bb_maxlevel;
}

int __WINAPI get_bb_depthlimit(lprec *lp)
{
  return(lp->bb_limitlevel);
}

void __WINAPI set_obj_bound(lprec *lp, REAL bb_heuristicOF)
{
  lp->bb_heuristicOF = bb_heuristicOF;
}

REAL __WINAPI get_obj_bound(lprec *lp)
{
  return(lp->bb_heuristicOF);
}

void __WINAPI set_mip_gap(lprec *lp, MYBOOL absolute, REAL mip_gap)
{
  if(absolute)
    lp->mip_absgap = mip_gap;
  else
    lp->mip_relgap = mip_gap;
}

REAL __WINAPI get_mip_gap(lprec *lp, MYBOOL absolute)
{
  if(absolute)
    return(lp->mip_absgap);
  else
    return(lp->mip_relgap);
}

MYBOOL __WINAPI set_var_branch(lprec *lp, int colnr, int branch_mode)
{
  if(colnr > lp->columns || colnr < 1) {
    report(lp, IMPORTANT, "set_var_branch: Column %d out of range\n", colnr);
    return( FALSE );
  }

  if(lp->bb_varbranch == NULL) {
    int i;
    if(branch_mode == BRANCH_DEFAULT)
      return( TRUE );
    allocMYBOOL(lp, &lp->bb_varbranch, lp->columns_alloc, FALSE);
    for(i = 0; i < lp->columns; i++)
      lp->bb_varbranch[i] = BRANCH_DEFAULT;
  }
  lp->bb_varbranch[colnr - 1] = (MYBOOL) branch_mode;
  return( TRUE );
}

int __WINAPI get_var_branch(lprec *lp, int colnr)
{
  if(colnr > lp->columns || colnr < 1) {
    report(lp, IMPORTANT, "get_var_branch: Column %d out of range\n", colnr);
    return(lp->bb_floorfirst);
  }

  if(lp->bb_varbranch == NULL)
    return(lp->bb_floorfirst);
  if(lp->bb_varbranch[colnr - 1] == BRANCH_DEFAULT)
    return(lp->bb_floorfirst);
  else
    return(lp->bb_varbranch[colnr - 1]);
}

static void set_infiniteex(lprec *lp, REAL infinite, MYBOOL init)
{
  int i;

  infinite = fabs(infinite);
  if((init) || is_infinite(lp, lp->bb_heuristicOF))
    lp->bb_heuristicOF = my_chsign(is_maxim(lp), infinite);
  if((init) || is_infinite(lp, lp->bb_breakOF))
    lp->bb_breakOF = my_chsign(is_maxim(lp), -infinite);
  for(i = 0; i <= lp->sum; i++) {
    if((!init) && is_infinite(lp, lp->orig_lowbo[i]))
      lp->orig_lowbo[i] = -infinite;
    if((init) || is_infinite(lp, lp->orig_upbo[i]))
      lp->orig_upbo[i] = infinite;
  }
  lp->infinite = infinite;
}


MYBOOL __WINAPI is_infinite(lprec *lp, REAL value)
{
#if 1
  return( (MYBOOL) (fabs(value) >= lp->infinite) );
#else
  if(fabs(value) >= lp->infinite)
    return( TRUE );
  else
    return( FALSE );
#endif
}

void __WINAPI set_infinite(lprec *lp, REAL infinite)
{
  set_infiniteex(lp, infinite, FALSE);
}

REAL __WINAPI get_infinite(lprec *lp)
{
  return(lp->infinite);
}

void __WINAPI set_epsperturb(lprec *lp, REAL epsperturb)
{
  lp->epsperturb = epsperturb;
}

REAL __WINAPI get_epsperturb(lprec *lp)
{
  return(lp->epsperturb);
}

void __WINAPI set_epspivot(lprec *lp, REAL epspivot)
{
  lp->epspivot = epspivot;
}

REAL __WINAPI get_epspivot(lprec *lp)
{
  return(lp->epspivot);
}

void __WINAPI set_epsint(lprec *lp, REAL epsint)
{
  lp->epsint = epsint;
}

REAL __WINAPI get_epsint(lprec *lp)
{
  return(lp->epsint);
}

void __WINAPI set_epsb(lprec *lp, REAL epsb)
{
  lp->epsprimal = MAX(epsb, lp->epsmachine);
}

REAL __WINAPI get_epsb(lprec *lp)
{
  return(lp->epsprimal);
}

void __WINAPI set_epsd(lprec *lp, REAL epsd)
{
  lp->epsdual = MAX(epsd, lp->epsmachine); /* Mainly used as tolerance for reduced cost */
}

REAL __WINAPI get_epsd(lprec *lp)
{
  return(lp->epsdual);
}

void __WINAPI set_epsel(lprec *lp, REAL epsel)
{
  lp->epsvalue = MAX(epsel, lp->epsmachine);
}

REAL __WINAPI get_epsel(lprec *lp)
{
  return(lp->epsvalue);
}

MYBOOL __WINAPI set_epslevel(lprec *lp, int epslevel)
{
  REAL SPX_RELAX, MIP_RELAX;

  switch(epslevel) {
    case EPS_TIGHT:  SPX_RELAX = 1;
                      MIP_RELAX = 1;
                      break;
    case EPS_MEDIUM: SPX_RELAX = 10;
                      MIP_RELAX = 1;
                      break;
    case EPS_LOOSE:  SPX_RELAX = 100;
                      MIP_RELAX = 10;
                      break;
    case EPS_BAGGY:  SPX_RELAX = 1000;
                      MIP_RELAX = 100;
                      break;
    default:        return( FALSE );
  }
  lp->epsvalue   = SPX_RELAX*DEF_EPSVALUE;
  lp->epsprimal  = SPX_RELAX*DEF_EPSPRIMAL;
  lp->epsdual    = SPX_RELAX*DEF_EPSDUAL;
  lp->epspivot   = SPX_RELAX*DEF_EPSPIVOT;
  lp->epssolution= MIP_RELAX*DEF_EPSSOLUTION;
  lp->epsint     = MIP_RELAX*DEF_EPSINT;
  lp->mip_absgap = MIP_RELAX*DEF_MIP_GAP;
  lp->mip_relgap = MIP_RELAX*DEF_MIP_GAP;

  return( TRUE );
}

void __WINAPI set_scaling(lprec *lp, int scalemode)
{
  lp->scalemode = scalemode;
}

int __WINAPI get_scaling(lprec *lp)
{
  return(lp->scalemode);
}

MYBOOL __WINAPI is_scalemode(lprec *lp, int testmask)
{
  return((MYBOOL) ((lp->scalemode & testmask) != 0));
}

MYBOOL __WINAPI is_scaletype(lprec *lp, int scaletype)
{
  int testtype;

  testtype = lp->scalemode & SCALE_MAXTYPE;
  return((MYBOOL) (scaletype == testtype));
}

void __WINAPI set_scalelimit(lprec *lp, REAL scalelimit)
/* Set the relative scaling convergence criterion for the active scaling mode;
   the integer part specifies the maximum number of iterations (default = 5). */
{
  lp->scalelimit = fabs(scalelimit);
}

REAL __WINAPI get_scalelimit(lprec *lp)
{
  return(lp->scalelimit);
}

MYBOOL __WINAPI is_integerscaling(lprec *lp)
{
  return(is_scalemode(lp, SCALE_INTEGERS));
}

void __WINAPI set_improve(lprec *lp, int improve)
{
  lp->improve = improve;
}

int __WINAPI get_improve(lprec *lp)
{
  return(lp->improve);
}

void __WINAPI set_lag_trace(lprec *lp, MYBOOL lag_trace)
{
  lp->lag_trace = lag_trace;
}

MYBOOL __WINAPI is_lag_trace(lprec *lp)
{
  return(lp->lag_trace);
}

void __WINAPI set_pivoting(lprec *lp, int pivoting)
{
  /* Set new pivoting strategy */
  lp->piv_strategy = pivoting;
  report(lp, DETAILED, "set_pivoting: Pricing strategy set to '%s'\n",
                       get_str_piv_rule(get_piv_rule(lp)));
}

int __WINAPI get_pivoting(lprec *lp)
{
  return( lp->piv_strategy );
}

/* INLINE */ int get_piv_rule(lprec *lp)
{
  return( (lp->piv_strategy | PRICE_STRATEGYMASK) ^ PRICE_STRATEGYMASK );
}

STATIC char *get_str_piv_rule(int rule)
{
  static char *pivotText[PRICER_LASTOPTION+1] =
  {"Bland first index", "Dantzig", "Devex", "Steepest Edge"};

  return( pivotText[rule] );
}

MYBOOL __WINAPI is_piv_rule(lprec *lp, int rule)
{
  return( (MYBOOL) (get_piv_rule(lp) == rule) );
}

MYBOOL __WINAPI is_piv_mode(lprec *lp, int testmask)
{
  return((MYBOOL) (((testmask & PRICE_STRATEGYMASK) != 0) &&
                   ((lp->piv_strategy & testmask) != 0)));
}

void __WINAPI set_break_at_first(lprec *lp, MYBOOL break_at_first)
{
  lp->bb_breakfirst = break_at_first;
}

MYBOOL __WINAPI is_break_at_first(lprec *lp)
{
  return(lp->bb_breakfirst);
}

void __WINAPI set_bb_floorfirst(lprec *lp, int bb_floorfirst)
{
  lp->bb_floorfirst = (MYBOOL) bb_floorfirst;
}

int __WINAPI get_bb_floorfirst(lprec *lp)
{
  return(lp->bb_floorfirst);
}

void __WINAPI set_break_at_value(lprec *lp, REAL break_at_value)
{
  lp->bb_breakOF = break_at_value;
}

REAL __WINAPI get_break_at_value(lprec *lp)
{
  return(lp->bb_breakOF);
}

void __WINAPI set_negrange(lprec *lp, REAL negrange)
{
  if(negrange <= 0)
    lp->negrange = negrange;
  else
    lp->negrange = 0.0;
}

REAL __WINAPI get_negrange(lprec *lp)
{
  return(lp->negrange);
}

int __WINAPI get_max_level(lprec *lp)
{
  return(lp->bb_maxlevel);
}

COUNTER __WINAPI get_total_nodes(lprec *lp)
{
  return(lp->bb_totalnodes);
}

COUNTER __WINAPI get_total_iter(lprec *lp)
{
  return(lp->total_iter + lp->current_iter);
}

REAL __WINAPI get_objective(lprec *lp)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_objective: Not a valid basis\n");
    return(0.0);
  }

  return( lp->best_solution[0] );
}

int __WINAPI get_nonzeros(lprec *lp)
{
  return( mat_nonzeros(lp->matA) );
}

MYBOOL __WINAPI set_mat(lprec *lp, int rownr, int colnr, REAL value)
{
  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "set_mat: Row %d out of range\n", rownr);
    return( FALSE );
  }
  if((colnr < 1) || (colnr > lp->columns)) {
    report(lp, IMPORTANT, "set_mat: Column %d out of range\n", colnr);
    return( FALSE );
  }

#ifdef DoMatrixRounding
  if(rownr == 0)
    value = roundToPrecision(value, lp->matA->epsvalue);
#endif
  value = scaled_mat(lp, value, rownr, colnr);
  if(rownr == 0) {
    lp->orig_obj[colnr] = my_chsign(is_chsign(lp, rownr), value);
    return( TRUE );
  }
  else
    return( mat_setvalue(lp->matA, rownr, colnr, value, FALSE) );
}

REAL __WINAPI get_working_objective(lprec *lp)
{
  REAL value = 0.0;

  if(!lp->basis_valid)
    report(lp, CRITICAL, "get_working_objective: Not a valid basis\n");
  else if((lp->spx_status == RUNNING) && (lp->solutioncount == 0))
    value = my_chsign(!is_maxim(lp), lp->rhs[0]);
  else
    value = lp->solution[0];

  return(value);
}

REAL __WINAPI get_var_primalresult(lprec *lp, int index)
{
  if((index < 0) || (index > lp->presolve_undo->orig_sum)) {
    report(lp, IMPORTANT, "get_var_primalresult: Index %d out of range\n", index);
    return( 0.0 );
  }
  if((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE)
    return( lp->full_solution[index] );
  else
    return( lp->best_solution[index] );
}

REAL __WINAPI get_var_dualresult(lprec *lp, int index)
{
  REAL *duals;

  if((index < 0) || (index > lp->presolve_undo->orig_sum)) {
    report(lp, IMPORTANT, "get_var_dualresult: Index %d out of range\n", index);
    return( 0.0 );
  }

  if(index == 0)
    return( lp->best_solution[0] );

  /* Make sure we actually have dual information available */
  if(!get_ptr_sensitivity_rhs(lp, &duals, NULL, NULL))
    return( 0.0 );
  else
    duals = ((lp->full_duals == NULL) ? lp->duals : lp->full_duals);
  return( duals[index] );
}

MYBOOL __WINAPI get_variables(lprec *lp, REAL *var)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_variables: Not a valid basis\n");
    return(FALSE);
  }

  MEMCOPY(var, lp->best_solution + (1 + lp->rows), lp->columns);
  return(TRUE);
}

MYBOOL __WINAPI get_ptr_variables(lprec *lp, REAL **var)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_ptr_variables: Not a valid basis\n");
    return(FALSE);
  }

  if(var != NULL)
   *var = lp->best_solution + (1 + lp->rows);
  return(TRUE);
}

MYBOOL __WINAPI get_constraints(lprec *lp, REAL *constr)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_constraints: Not a valid basis\n");
    return(FALSE);
  }

  MEMCOPY(constr, lp->best_solution + 1, lp->rows);
  return(TRUE);
}

MYBOOL __WINAPI get_ptr_constraints(lprec *lp, REAL **constr)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_ptr_constraints: Not a valid basis\n");
    return(FALSE);
  }

  if(constr != NULL)
   *constr = lp->best_solution + 1;
  return(TRUE);
}

MYBOOL __WINAPI get_sensitivity_rhs(lprec *lp, REAL *duals, REAL *dualsfrom, REAL *dualstill)
{
  REAL *duals0, *dualsfrom0, *dualstill0;

  if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_sensitivity_rhs: Not a valid basis\n");
    return(FALSE);
  }

  if(!get_ptr_sensitivity_rhs(lp,
                              (duals != NULL) ? &duals0 : NULL,
                              (dualsfrom != NULL) ? &dualsfrom0 : NULL,
                              (dualstill != NULL) ? &dualstill0 : NULL))
    return(FALSE);

  if(duals != NULL)
    MEMCOPY(duals, duals0, lp->sum);
  if(dualsfrom != NULL)
    MEMCOPY(dualsfrom, dualsfrom0, lp->sum);
  if(dualstill != NULL)
    MEMCOPY(dualstill, dualstill0, lp->sum);
  return(TRUE);
}

MYBOOL __WINAPI get_ptr_sensitivity_rhs(lprec *lp, REAL **duals, REAL **dualsfrom, REAL **dualstill)
{
  if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_ptr_sensitivity_rhs: Not a valid basis\n");
    return(FALSE);
  }

  if(duals != NULL) {
    if(lp->duals == NULL) {
      if((MIP_count(lp) > 0) && (lp->bb_totalnodes > 0)) {
        report(lp, CRITICAL, "get_ptr_sensitivity_rhs: Sensitivity unknown\n");
        return(FALSE);
      }
      if(!construct_duals(lp))
        return(FALSE);
    }
    *duals = lp->duals + 1;
  }

  if((dualsfrom != NULL) || (dualstill != NULL)) {
    if((lp->dualsfrom == NULL) || (lp->dualstill == NULL)) {
      if((MIP_count(lp) > 0) && (lp->bb_totalnodes > 0)) {
        report(lp, CRITICAL, "get_ptr_sensitivity_rhs: Sensitivity unknown\n");
        return(FALSE);
      }
      construct_sensitivity_duals(lp);
      if((lp->dualsfrom == NULL) || (lp->dualstill == NULL))
        return(FALSE);
    }
    if(dualsfrom != NULL)
      *dualsfrom = lp->dualsfrom + 1;
    if(dualstill != NULL)
      *dualstill = lp->dualstill + 1;
  }
  return(TRUE);
}

MYBOOL __WINAPI get_sensitivity_objex(lprec *lp, REAL *objfrom, REAL *objtill, REAL *objfromvalue, REAL *objtillvalue)
{
  REAL *objfrom0, *objtill0, *objfromvalue0, *objtillvalue0;

  if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_sensitivity_objex: Not a valid basis\n");
    return(FALSE);
  }

  if(!get_ptr_sensitivity_objex(lp, (objfrom != NULL) ? &objfrom0 : NULL,
                                    (objtill != NULL) ? &objtill0 : NULL,
                                    (objfromvalue != NULL) ? &objfromvalue0 : NULL,
                                    (objtillvalue != NULL) ? &objtillvalue0 : NULL))
    return(FALSE);

  if((objfrom != NULL) && (objfrom0 != NULL))
    MEMCOPY(objfrom, objfrom0, lp->columns);
  if((objtill != NULL) && (objtill0 != NULL))
    MEMCOPY(objtill, objtill0, lp->columns);
  if((objfromvalue != NULL) && (objfromvalue0 != NULL))
    MEMCOPY(objfromvalue, objfromvalue0, lp->columns);
  if((objtillvalue != NULL) && (objtillvalue0 != NULL))
    MEMCOPY(objtillvalue, objtillvalue0, lp->columns);
  return(TRUE);
}

MYBOOL __WINAPI get_sensitivity_obj(lprec *lp, REAL *objfrom, REAL *objtill)
{
  return(get_sensitivity_objex(lp, objfrom, objtill, NULL, NULL));
}

MYBOOL __WINAPI get_ptr_sensitivity_objex(lprec *lp, REAL **objfrom, REAL **objtill, REAL **objfromvalue, REAL **objtillvalue)
{
  if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_ptr_sensitivity_objex: Not a valid basis\n");
    return(FALSE);
  }

  if((objfrom != NULL) || (objtill != NULL)) {
    if((lp->objfrom == NULL) || (lp->objtill == NULL)) {
      if((MIP_count(lp) > 0) && (lp->bb_totalnodes > 0)) {
        report(lp, CRITICAL, "get_ptr_sensitivity_objex: Sensitivity unknown\n");
        return(FALSE);
      }
      construct_sensitivity_obj(lp);
      if((lp->objfrom == NULL) || (lp->objtill == NULL))
        return(FALSE);
    }
    if(objfrom != NULL)
      *objfrom = lp->objfrom + 1;
    if(objtill != NULL)
      *objtill = lp->objtill + 1;
  }

  if((objfromvalue != NULL) /* || (objtillvalue != NULL) */) {
    if((lp->objfromvalue == NULL) /* || (lp->objtillvalue == NULL) */) {
      if((MIP_count(lp) > 0) && (lp->bb_totalnodes > 0)) {
        report(lp, CRITICAL, "get_ptr_sensitivity_objex: Sensitivity unknown\n");
        return(FALSE);
      }
      construct_sensitivity_duals(lp);
      if((lp->objfromvalue == NULL) /* || (lp->objtillvalue == NULL) */)
        return(FALSE);
    }
  }

  if(objfromvalue != NULL)
    *objfromvalue = lp->objfromvalue + 1;

  if(objtillvalue != NULL)
    *objtillvalue = NULL /* lp->objtillvalue + 1 */;

  return(TRUE);
}

MYBOOL __WINAPI get_ptr_sensitivity_obj(lprec *lp, REAL **objfrom, REAL **objtill)
{
  return(get_ptr_sensitivity_objex(lp, objfrom, objtill, NULL, NULL));
}

void __WINAPI set_solutionlimit(lprec *lp, int limit)
{
  lp->solutionlimit = limit;
}
int __WINAPI get_solutionlimit(lprec *lp)
{
  return(lp->solutionlimit);
}
int __WINAPI get_solutioncount(lprec *lp)
{
  return(lp->solutioncount);
}

int __WINAPI get_Nrows(lprec *lp)
{
  return(lp->rows);
}

int __WINAPI get_Norig_rows(lprec *lp)
{
  if(lp->varmap_locked)
    return(lp->presolve_undo->orig_rows);
  else
    return(lp->rows);
}

int __WINAPI get_Lrows(lprec *lp)
{
  if(lp->matL == NULL)
    return( 0 );
  else
    return( lp->matL->rows );
}

int __WINAPI get_Ncolumns(lprec *lp)
{
  return(lp->columns);
}

int __WINAPI get_Norig_columns(lprec *lp)
{
  if(lp->varmap_locked)
    return(lp->presolve_undo->orig_columns);
  else
    return(lp->columns);
}


/* ---------------------------------------------------------------------------------- */
/* Core routines for lp_solve                                                         */
/* ---------------------------------------------------------------------------------- */
int __WINAPI get_status(lprec *lp)
{
  return(lp->spx_status);
}

char * __WINAPI get_statustext(lprec *lp, int statuscode)
{
  if (statuscode == NOBFP)             return("No basis factorization package");
  else if (statuscode == DATAIGNORED)  return("Invalid input data provided");
  else if (statuscode == NOMEMORY)     return("Not enough memory available");
  else if (statuscode == NOTRUN)       return("Model has not been optimized");
  else if (statuscode == OPTIMAL)      return("OPTIMAL solution");
  else if (statuscode == SUBOPTIMAL)   return("SUB-OPTIMAL solution");
  else if (statuscode == INFEASIBLE)   return("Model is primal INFEASIBLE");
  else if (statuscode == UNBOUNDED)    return("Model is primal UNBOUNDED");
  else if (statuscode == RUNNING)      return("lp_solve is currently running");
  else if (statuscode == NUMFAILURE)   return("NUMERIC FAILURE encountered");
  else if (statuscode == DEGENERATE)   return("DEGENERATE situation");
  else if (statuscode == USERABORT)    return("User-requested termination");
  else if (statuscode == TIMEOUT)      return("Termination due to timeout");
  else if (statuscode == PRESOLVED)    return("Model solved by presolve");
  else if (statuscode == PROCFAIL)     return("B&B routine failed");
  else if (statuscode == PROCBREAK)    return("B&B routine terminated");
  else if (statuscode == FEASFOUND)    return("Feasible B&B solution found");
  else if (statuscode == NOFEASFOUND)  return("No feasible B&B solution found");
  else if (statuscode == FATHOMED)     return("Fathomed/pruned branch");
  else                                 return("Undefined internal error");
}

MYBOOL __WINAPI is_obj_in_basis(lprec *lp)
{
  return( lp->obj_in_basis );
}

void __WINAPI set_obj_in_basis(lprec *lp, MYBOOL obj_in_basis)
{
  lp->obj_in_basis = (MYBOOL) (obj_in_basis == TRUE);
}

lprec * __WINAPI make_lp(int rows, int columns)
{
  lprec *lp;

# if defined FORTIFY
   /* Fortify_EnterScope(); */
# endif

  callcount++;
  if(rows < 0 || columns < 0)
    return(NULL);

  lp = (lprec*) calloc(1, sizeof(*lp));
  if(!lp)
    return(NULL);

  set_lp_name(lp, NULL);
  lp->names_used    = FALSE;
  lp->use_row_names = TRUE;
  lp->use_col_names = TRUE;

  /* Do standard initializations ------------------------------------------------------------ */
#if 1
  lp->obj_in_basis  = DEF_OBJINBASIS;
#else
  lp->obj_in_basis  = FALSE;
#endif
  lp->verbose       = NORMAL;
  set_callbacks(lp);
  set_BFP(lp, NULL);
  set_XLI(lp, NULL);
#if libBLAS > 0
  init_BLAS();
#if libBLAS > 1
  if(is_nativeBLAS() && !load_BLAS(libnameBLAS))
    /*report(lp, "make_lp: Could not load external BLAS library '%s'.\n", libnameBLAS)*/;
#endif
#endif

  /* Define the defaults for key user-settable values --------------------------------------- */
  reset_params(lp);

  /* Do other initializations --------------------------------------------------------------- */
  lp->source_is_file    = FALSE;
  lp->model_is_pure     = TRUE;
  lp->model_is_valid    = FALSE;
  lp->spx_status        = NOTRUN;
  lp->lag_status        = NOTRUN;

  lp->workarrays = mempool_create(lp);
  lp->wasPreprocessed   = FALSE;
  lp->wasPresolved      = FALSE;
  presolve_createUndo(lp);

  lp->bb_varactive      = NULL;
  lp->bb_varbranch      = NULL;
  lp->var_priority      = NULL;

  lp->rhsmax            = 0.0;
  lp->bigM              = 0.0;
  lp->bb_deltaOF        = 0.0;

  lp->equalities        = 0;
  lp->fixedvars         = 0;
  lp->int_vars          = 0;
  lp->sc_vars           = 0;

  lp->sos_ints          = 0;
  lp->sos_vars          = 0;
  lp->sos_priority      = NULL;

  lp->rows_alloc        = 0;
  lp->columns_alloc     = 0;
  lp->sum_alloc         = 0;

  lp->rows              = rows;
  lp->columns           = columns;
  lp->sum               = rows + columns;
  varmap_clear(lp);

  lp->matA = mat_create(lp, rows, columns, lp->epsvalue);
  lp->matL = NULL;
  lp->invB = NULL;
  lp->duals = NULL;
  lp->dualsfrom = NULL;
  lp->dualstill = NULL;
  lp->objfromvalue = NULL;
  lp->objfrom = NULL;
  lp->objtill = NULL;

  inc_col_space(lp, columns + 1);
  inc_row_space(lp, rows + 1);

  /* Avoid bound-checker uninitialized variable error */
  lp->orig_lowbo[0] = 0;

  lp->rootbounds = NULL;
  lp->bb_bounds = NULL;
  lp->bb_basis = NULL;

  lp->basis_valid       = FALSE;
  lp->simplex_mode      = SIMPLEX_DYNAMIC;
  lp->scaling_used      = FALSE;
  lp->columns_scaled    = FALSE;
  lp->P1extraDim        = 0;
  lp->P1extraVal        = 0.0;
  lp->bb_strongbranches = 0;
  lp->current_iter      = 0;
  lp->total_iter        = 0;
  lp->current_bswap     = 0;
  lp->total_bswap       = 0;
  lp->solutioncount     = 0;
  lp->solvecount        = 0;

  allocINT(lp, &lp->rejectpivot, DEF_MAXPIVOTRETRY + 1, TRUE);

  set_minim(lp);
  set_infiniteex(lp, DEF_INFINITE, TRUE);

  initPricer(lp);

  /* Call-back routines by KE */
  lp->ctrlc = NULL;
  lp->ctrlchandle = NULL;
  lp->writelog = NULL;
  lp->loghandle = NULL;
  lp->debuginfo = NULL;
  lp->usermessage = NULL;
  lp->msgmask = MSG_NONE;
  lp->msghandle = NULL;

  lp->timecreate = timeNow();
  return(lp);
}

MYBOOL __WINAPI resize_lp(lprec *lp, int rows, int columns)
{
  MYBOOL status = TRUE;

  if(columns > lp->columns)
    status = inc_col_space(lp, columns - lp->columns);
  else
    while(status && (lp->columns > columns)) {
      status = del_column(lp, lp->columns);
    }
  if(status && (rows > lp->rows))
    status = inc_row_space(lp, rows - lp->rows);
  else
    while(status && (lp->rows > rows)) {
      status = del_constraint(lp, lp->rows);
    }
  return( status );
}

void __WINAPI free_lp(lprec **plp)
{
  if(plp != NULL) {
    lprec *lp = *plp;
    if(lp != NULL)
      delete_lp(lp);
    *plp = NULL;
  }
}

void __WINAPI delete_lp(lprec *lp)
{
  if(lp == NULL)
    return;

  FREE(lp->lp_name);
  FREE(lp->ex_status);
  if(lp->names_used) {
    FREE(lp->row_name);
    FREE(lp->col_name);
    free_hash_table(lp->rowname_hashtab);
    free_hash_table(lp->colname_hashtab);
  }

  mat_free(&lp->matA);
  lp->bfp_free(lp);
#if LoadInverseLib == TRUE
  if(lp->hBFP != NULL)
    set_BFP(lp, NULL);
#endif
#if LoadLanguageLib == TRUE
  if(lp->hXLI != NULL)
    set_XLI(lp, NULL);
#endif

  unset_OF_p1extra(lp);
  FREE(lp->orig_obj);
  FREE(lp->orig_rhs);
  FREE(lp->rhs);
  FREE(lp->var_type);
  set_var_weights(lp, NULL);
  FREE(lp->bb_varbranch);
  FREE(lp->sc_lobound);
  FREE(lp->var_is_free);
  FREE(lp->orig_upbo);
  FREE(lp->orig_lowbo);
  FREE(lp->upbo);
  FREE(lp->lowbo);
  FREE(lp->var_basic);
  FREE(lp->is_basic);
  FREE(lp->is_lower);
  if(lp->bb_PseudoCost != NULL) {
/*    report(lp, SEVERE, "delete_lp: The B&B pseudo-cost array was not cleared on delete\n"); */
    free_pseudocost(lp);
  }
  if(lp->bb_bounds != NULL) {
    report(lp, SEVERE, "delete_lp: The stack of B&B levels was not empty (failed at %.0f nodes)\n",
                       (double) lp->bb_totalnodes);
    unload_BB(lp);
  }
  if(lp->bb_basis != NULL) {
/*    report(lp, SEVERE, "delete_lp: The stack of saved bases was not empty on delete\n"); */
    unload_basis(lp, FALSE);
  }

  FREE(lp->rejectpivot);
  partial_freeBlocks(&(lp->rowblocks));
  partial_freeBlocks(&(lp->colblocks));
  multi_free(&(lp->multivars));
  multi_free(&(lp->longsteps));

  FREE(lp->solution);
  FREE(lp->best_solution);
  FREE(lp->full_solution);

  presolve_freeUndo(lp);
  mempool_free(&(lp->workarrays));

  freePricer(lp);

  FREE(lp->drow);
  FREE(lp->nzdrow);

  FREE(lp->duals);
  FREE(lp->full_duals);
  FREE(lp->dualsfrom);
  FREE(lp->dualstill);
  FREE(lp->objfromvalue);
  FREE(lp->objfrom);
  FREE(lp->objtill);
  FREE(lp->row_type);

  if(lp->sos_vars > 0)
    FREE(lp->sos_priority);
  free_SOSgroup(&(lp->SOS));
  free_SOSgroup(&(lp->GUB));
  freecuts_BB(lp);

  if(lp->scaling_used)
    FREE(lp->scalars);
  if(lp->matL != NULL) {
    FREE(lp->lag_rhs);
    FREE(lp->lambda);
    FREE(lp->lag_con_type);
    mat_free(&lp->matL);
  }
  if(lp->streamowned)
    set_outputstream(lp, NULL);

#if libBLAS > 0
  if(!is_nativeBLAS())
    unload_BLAS();
#endif

  FREE(lp);

# if defined FORTIFY
    /* Fortify_LeaveScope(); */
# endif
}

static MYBOOL get_SOS(lprec *lp, int index, char *name, int *sostype, int *priority, int *count, int *sosvars, REAL *weights)
{
  SOSrec *SOS;

  if((index < 1) || (index > SOS_count(lp)))
    return( FALSE );
  SOS = lp->SOS->sos_list[index-1];
  if(name != NULL)
    strcpy(name, SOS->name);
  if(sostype != NULL)
    *sostype = SOS->type;
  if(priority != NULL)
    *priority = SOS->priority;
  if(count != NULL) {
    *count = SOS->size;
    if(sosvars != NULL) {
      int i;
      for(i = 1; i <= *count; i++) {
        sosvars[i-1] = SOS->members[i];
        if(weights != NULL)
          weights[i-1] = SOS->weights[i];
      }
    }
  }
  return( TRUE );
}

/* Make a copy of the existing model using (mostly) high-level
   construction routines to simplify future maintainance. */
lprec* __WINAPI copy_lp(lprec *lp)
{
  int   i, n, *idx = NULL;
  REAL  hold, *val = NULL;
  lprec *newlp = NULL;
  char buf[256];
  int sostype, priority, count, *sosvars;
  REAL *weights;

#if 0
  if(lp->wasPresolved)
    return( newlp );
#endif

  if(!allocINT(lp, &idx, lp->rows+1, FALSE) ||
     !allocREAL(lp, &val, lp->rows+1, FALSE))
    goto Finish;

  /* Create the new object */
  newlp = make_lp(lp->rows, 0);
  resize_lp(newlp, lp->rows, lp->columns);
  set_sense(newlp, is_maxim(lp));
  set_use_names(newlp, FALSE, is_use_names(lp, FALSE));
  set_use_names(newlp, TRUE, is_use_names(lp, TRUE));
  set_lp_name(newlp, get_lp_name(lp));
  /* set_algopt(newlp, get_algopt(lp)); */ /* v6 */
  set_verbose(newlp, get_verbose(lp));

  /* Transfer standard simplex parameters */
  set_epspivot(newlp, get_epspivot(lp));
  set_epsel(newlp, get_epsel(lp));
  set_epsb(newlp, get_epsb(lp));
  set_epsd(newlp, get_epsd(lp));
  set_pivoting(newlp, get_pivoting(lp));
  set_negrange(newlp, lp->negrange);
  set_infinite(newlp, get_infinite(lp));
  set_presolve(newlp, get_presolve(lp), get_presolveloops(lp));
  set_scaling(newlp, get_scaling(lp));
  set_scalelimit(newlp, get_scalelimit(lp));
  set_simplextype(newlp, get_simplextype(lp));
  set_epsperturb(newlp, get_epsperturb(lp));
  set_anti_degen(newlp, get_anti_degen(lp));
  set_improve(newlp, get_improve(lp));
  set_basiscrash(newlp, get_basiscrash(lp));
  set_maxpivot(newlp, get_maxpivot(lp));
  set_timeout(newlp, get_timeout(lp));

  /* Transfer MILP parameters */
  set_epsint(newlp, get_epsint(lp));
  set_bb_rule(newlp, get_bb_rule(lp));
  set_bb_depthlimit(newlp, get_bb_depthlimit(lp));
  set_bb_floorfirst(newlp, get_bb_floorfirst(lp));
  set_mip_gap(newlp, TRUE, get_mip_gap(lp, TRUE));
  set_mip_gap(newlp, FALSE, get_mip_gap(lp, FALSE));
  set_break_at_first(newlp, is_break_at_first(lp));
  set_break_at_value(newlp, get_break_at_value(lp));

  /* Set RHS and range */
  for(i = 0; i <= lp->rows; i++) {
    if(i > 0)
      set_constr_type(newlp, i, get_constr_type(lp, i));
    set_rh(newlp, i, get_rh(lp, i));
    if((i > 0) && ((hold = get_rh_range(lp, i)) < lp->infinite))
      set_rh_range(newlp, i, hold);
	if(lp->names_used && lp->use_row_names && (lp->row_name[i] != NULL) && (lp->row_name[i]->name != NULL))
      set_row_name(newlp, i, get_row_name(lp, i));
  }

  /* Load the constraint matrix and variable definitions */
  for(i = 1; i <= lp->columns; i++) {
    n = get_columnex(lp, i, val, idx);
    add_columnex(newlp, n, val, idx);
    if(is_binary(lp, i))
      set_binary(newlp, i, TRUE);
    else {
      if(is_int(lp, i))
        set_int(newlp, i, TRUE);
      if((hold = get_lowbo(lp, i)) != 0)
        set_lowbo(newlp, i, hold);
      if((hold = get_upbo(lp, i)) < lp->infinite)
        set_upbo(newlp, i, hold);
    }
    if(is_semicont(lp, i))
      set_semicont(newlp, i, TRUE);
	if(lp->names_used && lp->use_col_names && (lp->col_name[i] != NULL) && (lp->col_name[i]->name != NULL))
      set_col_name(newlp, i, get_col_name(lp, i));
  }

  /* copy SOS data */
  for(i = 1; get_SOS(lp, i, buf, &sostype, &priority, &count, NULL, NULL); i++)
    if (count) {
      sosvars = (int *) malloc(count * sizeof(*sosvars));
      weights = (REAL *) malloc(count * sizeof(*weights));
      get_SOS(lp, i, buf, &sostype, &priority, &count, sosvars, weights);
	  add_SOS(newlp, buf, sostype, priority, count, sosvars, weights);
      free(weights);
      free(sosvars);
    }

#if 0
  /* Other parameters set if the source model was previously solved */
  if(lp->solvecount > 0) {
    MEMCOPY(newlp->scalars, lp->scalars, lp->sum+1);
    MEMCOPY(newlp->var_basic, lp->var_basic, lp->rows+1);
    MEMCOPY(newlp->is_basic, lp->is_basic, lp->sum+1);
    MEMCOPY(newlp->is_lower, lp->is_lower, lp->sum+1);
    MEMCOPY(newlp->solution, lp->solution, lp->sum+1);
    if(lp->duals != NULL) {
      allocREAL(newlp, &newlp->duals, newlp->sum_alloc+1, FALSE);
      MEMCOPY(newlp->duals, lp->duals, lp->sum+1);
    }
    newlp->solutioncount = lp->solutioncount;
    newlp->solvecount = lp->solvecount;
  }
#endif

  /* Clean up before returning */
Finish:
  FREE(val);
  FREE(idx);

  return( newlp );
}
MYBOOL __WINAPI dualize_lp(lprec *lp)
{
  int     i, n;
  MATrec  *mat = lp->matA;
  REAL    *item;

  /* Are we allowed to perform the operation? */
  if((MIP_count(lp) > 0) || (lp->solvecount > 0))
    return( FALSE );

  /* Modify sense */
  set_sense(lp, (MYBOOL) !is_maxim(lp));

  /* Transpose matrix and reverse signs */
  n = mat_nonzeros(mat);
  mat_transpose(mat);
  item = &COL_MAT_VALUE(0);
  for(i = 0; i < n; i++, item += matValueStep)
    *item *= -1;

  /* Row-column swap other vectors */
  swapINT(&lp->rows, &lp->columns);
  swapINT(&lp->rows_alloc, &lp->columns_alloc);
  swapREAL(lp->orig_rhs, lp->orig_obj);
  if ((lp->rhs != NULL) && (lp->obj != NULL))
    swapREAL(lp->rhs, lp->obj);

  /* Reallocate storage */
/*
var_type
sc_bound
solution
best_solution
full_solution
duals
*/

  /* Shift variable bounds */
/*
is_basic
orig_upbo
orig_lowbo
scalars
*/

  return( TRUE );
}

/* Optimize memory usage */
STATIC MYBOOL memopt_lp(lprec *lp, int rowextra, int colextra, int nzextra)
{
  MYBOOL status = FALSE;

  if(lp == NULL)
    return( status );

  status = mat_memopt(lp->matA, rowextra, colextra, nzextra) &&
           (++rowextra > 0) && (++colextra > 0) && (++nzextra > 0);

#if 0 /* inc_ routines not well-tested for reduction in size allocation */
  if(status) {
    int colalloc = lp->columns_alloc - MIN(lp->columns_alloc, lp->columns + colextra),
        rowalloc = lp->rows_alloc    - MIN(lp->rows_alloc,    lp->rows + rowextra);

    status = inc_lag_space(lp, rowalloc, FALSE) &&
             inc_row_space(lp, rowalloc) &&
             inc_col_space(lp, colalloc);
  }
#endif

  return( status );
}


/* Utility routine group for constraint and column deletion/insertion
   mapping in relation to the original set of constraints and columns */
STATIC void varmap_lock(lprec *lp)
{
  presolve_fillUndo(lp, lp->rows, lp->columns, TRUE);
  lp->varmap_locked = TRUE;
}
STATIC void varmap_clear(lprec *lp)
{
  presolve_setOrig(lp, 0, 0);
  lp->varmap_locked = FALSE;
}
STATIC MYBOOL varmap_canunlock(lprec *lp)
{
  /* Don't do anything if variables aren't locked yet */
  if(lp->varmap_locked) {
    int i;
    presolveundorec *psundo = lp->presolve_undo;

    /* Check for the obvious */
    if(/*lp->names_used ||
       (psundo->orig_columns != lp->columns) || (psundo->orig_rows != lp->rows)) */
       (psundo->orig_columns > lp->columns) || (psundo->orig_rows > lp->rows))
      return( FALSE );

    /* Check for deletions */
    for(i = psundo->orig_rows + psundo->orig_columns; i > 0; i--)
      if(psundo->orig_to_var[i] == 0)
        return( FALSE );

    /* Check for insertions */
    for(i = lp->sum; i > 0; i--)
      if(psundo->var_to_orig[i] == 0)
        return( FALSE );
  }
  return( TRUE );
}
STATIC void varmap_add(lprec *lp, int base, int delta)
{
  int i, ii;
  presolveundorec *psundo = lp->presolve_undo;

  /* Don't do anything if variables aren't locked yet */
  if(!lp->varmap_locked)
    return;

  /* Set new constraints/columns to have an "undefined" mapping to original
     constraints/columns (assumes that counters have NOT yet been updated) */
  for(i = lp->sum; i >= base; i--) {
    ii = i + delta;
    psundo->var_to_orig[ii] = psundo->var_to_orig[i];
  }

  /* Initialize map of added rows/columns */
  for(i = 0; i < delta; i++) {
    ii = base + i;
    psundo->var_to_orig[ii] = 0;
  }
}

STATIC void varmap_delete(lprec *lp, int base, int delta, LLrec *varmap)
{
  int             i, ii, j;
  MYBOOL          preparecompact;
  presolveundorec *psundo = lp->presolve_undo;

  /* Set the model "dirty" if we are deleting row of constraint */
  lp->model_is_pure  = FALSE;

  /* Don't do anything if
     1) variables aren't locked yet, or
     2) the constraint was added after the variables were locked */
  if(!lp->varmap_locked) {
#if 1
   if(lp->names_used)
     varmap_lock(lp);
   else
#endif
     return;
  }

  /* Do mass deletion via a linked list */
  preparecompact = (MYBOOL) (varmap != NULL);
  if(preparecompact) {
    preparecompact = (MYBOOL) (base > lp->rows);  /* Set TRUE for columns */
    for(j = firstInactiveLink(varmap); j != 0; j = nextInactiveLink(varmap, j)) {
      i = j;
      if(preparecompact) {
#ifdef Paranoia
        if(SOS_is_member(lp->SOS, 0, j))
          report(lp, SEVERE, "varmap_delete: Deleting variable %d, which is in a SOS!\n", j);
#endif
        i += lp->rows;
      }
      ii = psundo->var_to_orig[i];
      if(ii > 0)  /* It was an original variable; reverse sign of index to flag deletion */
        psundo->var_to_orig[i] = -ii;
      else        /* It was a non-original variable; add special code for deletion */
        psundo->var_to_orig[i] = -(psundo->orig_rows+psundo->orig_columns+i);
    }
    return;
  }

  /* Do legacy simplified version if we are doing batch delete operations */
  preparecompact = (MYBOOL) (base < 0);
  if(preparecompact) {
    base = -base;
    if(base > lp->rows)
      base += (psundo->orig_rows - lp->rows);
    for(i = base; i < base-delta; i++) {
      ii = psundo->var_to_orig[i];
      if(ii > 0)  /* It was an original variable; reverse sign of index to flag deletion */
        psundo->var_to_orig[i] = -ii;
      else       /* It was a non-original variable; add special code for deletion */
        psundo->var_to_orig[i] = -(psundo->orig_rows+psundo->orig_columns+i);
    }
    return;
  }

  /* We are deleting an original constraint/column;
     1) clear mapping of original to deleted
     2) shift the deleted variable to original mappings left
     3) decrement all subsequent original-to-current pointers
  */
  for(i = base; i < base-delta; i++) {
    ii = psundo->var_to_orig[i];
    if(ii > 0)
      psundo->orig_to_var[ii] = 0;
  }
  for(i = base; i <= lp->sum+delta; i++) {
    ii = i - delta;
    psundo->var_to_orig[i] = psundo->var_to_orig[ii];
  }

  i = 1;
  j = psundo->orig_rows;
  if(base > lp->rows) {
    i += j;
    j += psundo->orig_columns;
  }
  ii = base-delta;
  for(; i <= j; i++) {
    if(psundo->orig_to_var[i] >= ii)
      psundo->orig_to_var[i] += delta;
  }

}

STATIC MYBOOL varmap_validate(lprec *lp, int varno)
{
  MYBOOL success = TRUE;
  int i, ii, ix, ie,
       n_rows = lp->rows,
       orig_sum = lp->presolve_undo->orig_sum,
       orig_rows = lp->presolve_undo->orig_rows;

  if(varno <= 0) {
    varno = 1;
    ie = orig_sum;
  }
  else
    ie = varno;
  for(i = varno; success && (i <= ie); i++) {
    ix = lp->presolve_undo->orig_to_var[i];
    if((ix > 0) && (i > orig_rows))
      ix += n_rows;

    /* Check for index out of range due to presolve */
    success = (MYBOOL) (ix <= orig_sum);
    if(!success)
      report(lp, SEVERE, "varmap_validate: Invalid new mapping found for variable %d\n",
                           i);
    else if(ix != 0) {
      ii = lp->presolve_undo->var_to_orig[ix];
      if(ix > n_rows)
        ii += orig_rows;
      success = (MYBOOL) (ii == i);
      if(!success)
        report(lp, SEVERE, "varmap_validate: Invalid old mapping found for variable %d (%d)\n",
                           i, ii);
    }
  }
  return( success );
}

STATIC void varmap_compact(lprec *lp, int prev_rows, int prev_cols)
{
  presolveundorec *psundo = lp->presolve_undo;
  int             i, ii, n_sum, n_rows,
                  orig_rows = psundo->orig_rows,
                  prev_sum = prev_rows + prev_cols;

  /* Nothing to do if the model is not "dirty" or the variable map is not locked */
  if(lp->model_is_pure || !lp->varmap_locked)
    return;

  /* We are deleting an original constraint/column;
     1) clear mapping of original to deleted
     2) shift the deleted variable to original mappings left
     3) decrement all subsequent original-to-current pointers
  */
  n_sum = 0;
  n_rows = 0;
  for(i = 1; i <= prev_sum; i++) {
    ii = psundo->var_to_orig[i];

    /* Process variable if it was deleted in the previous round */
    if(ii < 0) {
      ii = -ii;
      /* Update map back if we have an original variable, otherwise just skip */
      if(i <= prev_rows)
        psundo->orig_to_var[ii] = 0;
      else
        psundo->orig_to_var[orig_rows+ii] = 0;
    }
    /* Otherwise shift and update map back */
    else {
      n_sum++;
      /* Shift only if necessary */
      if(n_sum < i)
        psundo->var_to_orig[n_sum] = ii;
      /* Update map back if we have an original variable */
      if(ii > 0) {
        if(i <= prev_rows) {
          psundo->orig_to_var[ii] = n_sum;
          n_rows = n_sum;
        }
        else
          psundo->orig_to_var[orig_rows+ii] = n_sum-n_rows;
      }
    }
  }
#ifdef xxParanoia
  if(!varmap_validate(lp, 0))
    report(lp, SEVERE, "varmap_compact: Internal presolve mapping error at exit\n");
#endif

}

/* Utility group for shifting row and column data */
STATIC MYBOOL shift_rowcoldata(lprec *lp, int base, int delta, LLrec *usedmap, MYBOOL isrow)
/* Note: Assumes that "lp->sum" and "lp->rows" HAVE NOT been updated to the new counts */
{
  int  i, ii;
  REAL lodefault;

  /* Shift data right/down (insert), and set default values in positive delta-gap */
  if(delta > 0) {

    /* Determine if we can take the easy way out */
    MYBOOL easyout = (MYBOOL) ((lp->solvecount == 0) && (base > lp->rows));

    /* Shift the row/column data */

    MEMMOVE(lp->orig_upbo + base + delta, lp->orig_upbo + base, lp->sum - base + 1);
    MEMMOVE(lp->orig_lowbo + base + delta, lp->orig_lowbo + base, lp->sum - base + 1);

    if(!easyout) {
      MEMMOVE(lp->upbo + base + delta, lp->upbo + base, lp->sum - base + 1);
      MEMMOVE(lp->lowbo + base + delta, lp->lowbo + base, lp->sum - base + 1);
      if(lp->model_is_valid) {
        MEMMOVE(lp->solution + base + delta, lp->solution + base, lp->sum - base + 1);
        MEMMOVE(lp->best_solution + base + delta, lp->best_solution + base, lp->sum - base + 1);
      }
      MEMMOVE(lp->is_lower + base + delta, lp->is_lower + base, lp->sum - base + 1);
    }

    /* Deal with scalars; the vector can be NULL */
    if(lp->scalars != NULL) {
      if(!easyout)
        for(ii = lp->sum; ii >= base; ii--) {
          i = ii + delta;
          lp->scalars[i] = lp->scalars[ii];
        }
      for(ii = base; ii < base + delta; ii++)
        lp->scalars[ii] = 1;
    }

    /* Set defaults */
#ifdef SlackInitMinusInf
    if(isrow)
      lodefault = -lp->infinite;
    else
#endif
      lodefault = 0;

    for(i = 0; i < delta; i++) {
      ii = base + i;
      lp->orig_upbo[ii] = lp->infinite;
      lp->orig_lowbo[ii] = lodefault;
      if(!easyout) {
        lp->upbo[ii] = lp->orig_upbo[ii];
        lp->lowbo[ii] = lp->orig_lowbo[ii];
        lp->is_lower[ii] = TRUE;
      }
    }
  }

  /* Shift data left/up (delete) */
  else if(usedmap != NULL) {
    int k, offset = 0;
    if(!isrow)
      offset += lp->rows;
    i = offset + 1;
    for(k = firstActiveLink(usedmap); k != 0;
        i++, k = nextActiveLink(usedmap, k)) {
      ii = k + offset;
      if(ii == i)
        continue;
      lp->upbo[i] = lp->upbo[ii];
      lp->orig_upbo[i] = lp->orig_upbo[ii];
      lp->lowbo[i] = lp->lowbo[ii];
      lp->orig_lowbo[i] = lp->orig_lowbo[ii];
      lp->solution[i] = lp->solution[ii];
      lp->best_solution[i] = lp->best_solution[ii];
      lp->is_lower[i] = lp->is_lower[ii];
      if(lp->scalars != NULL)
        lp->scalars[i] = lp->scalars[ii];
    }
    if(isrow) {
      base = lp->rows + 1;
      MEMMOVE(lp->upbo + i, lp->upbo + base, lp->columns);
      MEMMOVE(lp->orig_upbo + i, lp->orig_upbo + base, lp->columns);
      MEMMOVE(lp->lowbo + i, lp->lowbo + base, lp->columns);
      MEMMOVE(lp->orig_lowbo + i, lp->orig_lowbo + base, lp->columns);
      if(lp->model_is_valid) {
        MEMMOVE(lp->solution + i, lp->solution + base, lp->columns);
        MEMMOVE(lp->best_solution + i, lp->best_solution + base, lp->columns);
      }
      MEMMOVE(lp->is_lower + i, lp->is_lower + base, lp->columns);
      if(lp->scalars != NULL)
        MEMMOVE(lp->scalars + i, lp->scalars + base, lp->columns);
    }
  }

  else if(delta < 0) {

    /* First make sure we don't cross the sum count border */
    if(base-delta-1 > lp->sum)
      delta = base - lp->sum - 1;

    /* Shift the data*/
    for(i = base; i <= lp->sum + delta; i++) {
      ii = i - delta;
      lp->upbo[i] = lp->upbo[ii];
      lp->orig_upbo[i] = lp->orig_upbo[ii];
      lp->lowbo[i] = lp->lowbo[ii];
      lp->orig_lowbo[i] = lp->orig_lowbo[ii];
      lp->solution[i] = lp->solution[ii];
      lp->best_solution[i] = lp->best_solution[ii];
      lp->is_lower[i] = lp->is_lower[ii];
      if(lp->scalars != NULL)
        lp->scalars[i] = lp->scalars[ii];
    }

  }

  lp->sum += delta;

  lp->matA->row_end_valid = FALSE;

  return(TRUE);
}

STATIC MYBOOL shift_basis(lprec *lp, int base, int delta, LLrec *usedmap, MYBOOL isrow)
/* Note: Assumes that "lp->sum" and "lp->rows" HAVE NOT been updated to the new counts */
{
  int i, ii;
  MYBOOL Ok = TRUE;

  /* Don't bother to shift the basis if it is not yet ready */
  if(!is_BasisReady(lp))
    return( Ok );

  /* Basis adjustments due to insertions (after actual row/column insertions) */
  if(delta > 0) {

    /* Determine if the basis becomes invalidated */
    if(isrow)
      set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT);

    /* Shift and fix invalid basis references (increment higher order basic variable index) */
    if(base <= lp->sum)
      MEMMOVE(lp->is_basic + base + delta, lp->is_basic + base, lp->sum - base + 1);

    /* Prevent CPU-expensive basis updating if this is the initial model creation */
    if(!lp->model_is_pure || (lp->solvecount > 0))
      for(i = 1; i <= lp->rows; i++) {
        ii = lp->var_basic[i];
        if(ii >= base)
          lp->var_basic[i] += delta;
      }

    /* Update the basis (shift and extend) */
    for(i = 0; i < delta; i++) {
      ii = base + i;
      lp->is_basic[ii] = isrow;
      if(isrow)
        lp->var_basic[lp->rows+1+i] = ii;
    }

  }
  /* Basis adjustments due to deletions (after actual row/column deletions) */
  else {
    int j,k;

    /* Fix invalid basis references (decrement high basic slack variable indexes),
       but reset the entire basis if a deleted variable is found in the basis */
    k = 0;
    for(i = 1; i <= lp->rows; i++) {
      ii = lp->var_basic[i];
      lp->is_basic[ii] = FALSE;
      if(ii >= base) {
       /* Skip to next basis variable if this one is to be deleted */
        if(ii < base-delta) {
          set_action(&lp->spx_action, ACTION_REBASE);
          continue;
        }
       /* Otherwise, update the index of the basic variable for deleted variables */
        ii += delta;
      }
      k++;
      lp->var_basic[k] = ii;
    }

    /* Set the new basis indicators */
    i = k;
    if(isrow)
      i = MIN(k, lp->rows+delta);
    for(; i > 0; i--) {
      j = lp->var_basic[i];
      lp->is_basic[j] = TRUE;
    }

    /* If a column was deleted from the basis then simply add back a non-basic
       slack variable; do two scans, if necessary to avoid adding equality slacks */
    if(!isrow && (k < lp->rows)) {
      for(j = 0; j <= 1; j++)
      for(i = 1; (i <= lp->rows) && (k < lp->rows); i++)
        if(!lp->is_basic[i]) {
          if(!is_constr_type(lp, i, EQ) || (j == 1)) {
            k++;
            lp->var_basic[k] = i;
            lp->is_basic[i] = TRUE;
          }
        }
      k = 0;
    }

    /* We are left with "k" indexes; if no basis variable was deleted, k=rows and the
       inverse is still valid, if k+delta < 0 we do not have a valid
       basis and must create one (in most usage modes this should not happen,
       unless there is a bug) */
    if(k+delta < 0)
      Ok = FALSE;
    if(isrow || (k != lp->rows))
      set_action(&lp->spx_action, ACTION_REINVERT);

  }
  return(Ok);

}

STATIC MYBOOL shift_rowdata(lprec *lp, int base, int delta, LLrec *usedmap)
/* Note: Assumes that "lp->rows" HAS NOT been updated to the new count */
{
  int i, ii;

  /* Shift sparse matrix row data */
  if(lp->matA->is_roworder)
    mat_shiftcols(lp->matA, &base, delta, usedmap);
  else
    mat_shiftrows(lp->matA, &base, delta, usedmap);

  /* Shift data down (insert row), and set default values in positive delta-gap */
  if(delta > 0) {

    /* Shift row data */
    for(ii = lp->rows; ii >= base; ii--) {
      i = ii + delta;
      lp->orig_rhs[i] = lp->orig_rhs[ii];
      lp->rhs[i] = lp->rhs[ii];
      lp->row_type[i] = lp->row_type[ii];
    }

    /* Set defaults (actual basis set in separate procedure) */
    for(i = 0; i < delta; i++) {
      ii = base + i;
      lp->orig_rhs[ii] = 0;
      lp->rhs[ii] = 0;
      lp->row_type[ii] = ROWTYPE_EMPTY;
    }
  }

  /* Shift data up (delete row) */
  else if(usedmap != NULL) {
    for(i = 1, ii = firstActiveLink(usedmap); ii != 0;
        i++, ii = nextActiveLink(usedmap, ii)) {
      if(i == ii)
        continue;
      lp->orig_rhs[i] = lp->orig_rhs[ii];
      lp->rhs[i] = lp->rhs[ii];
      lp->row_type[i] = lp->row_type[ii];
    }
    delta = i - lp->rows - 1;
  }
  else if(delta < 0) {

    /* First make sure we don't cross the row count border */
    if(base-delta-1 > lp->rows)
      delta = base - lp->rows - 1;

    /* Shift row data (don't shift basis indexes here; done in next step) */
    for(i = base; i <= lp->rows + delta; i++) {
      ii = i - delta;
      lp->orig_rhs[i] = lp->orig_rhs[ii];
      lp->rhs[i] = lp->rhs[ii];
      lp->row_type[i] = lp->row_type[ii];
    }
  }

  shift_basis(lp, base, delta, usedmap, TRUE);
  shift_rowcoldata(lp, base, delta, usedmap, TRUE);
  inc_rows(lp, delta);

  return(TRUE);
}

STATIC MYBOOL shift_coldata(lprec *lp, int base, int delta, LLrec *usedmap)
/* Note: Assumes that "lp->columns" has NOT been updated to the new count */
{
  int i, ii;

  free_duals(lp);

  /* Shift A matrix data */
  if(lp->matA->is_roworder)
    mat_shiftrows(lp->matA, &base, delta, usedmap);
  else
    mat_shiftcols(lp->matA, &base, delta, usedmap);

  /* Shift data right (insert), and set default values in positive delta-gap */
  if(delta > 0) {

    /* Fix variable priority data */
    if((lp->var_priority != NULL) && (base <= lp->columns)) {
      for(i = 0; i < lp->columns; i++)
        if(lp->var_priority[i] >= base)
          lp->var_priority[i] += delta;
    }
    if((lp->sos_priority != NULL) && (base <= lp->columns)) {
      for(i = 0; i < lp->sos_vars; i++)
        if(lp->sos_priority[i] >= base)
          lp->sos_priority[i] += delta;
    }

    /* Fix invalid split variable data */
    if((lp->var_is_free != NULL) && (base <= lp->columns)) {
      for(i = 1; i <= lp->columns; i++)
        if(abs(lp->var_is_free[i]) >= base)
          lp->var_is_free[i] += my_chsign(lp->var_is_free[i] < 0, delta);
    }

    /* Shift column data right */
    for(ii = lp->columns; ii >= base; ii--) {
      i = ii + delta;
      lp->var_type[i] = lp->var_type[ii];
      lp->sc_lobound[i] = lp->sc_lobound[ii];
      lp->orig_obj[i] = lp->orig_obj[ii];
      if(lp->obj != NULL)
        lp->obj[i] = lp->obj[ii];
/*
      if(lp->objfromvalue != NULL)
        lp->objfromvalue[i] = lp->objfromvalue[ii];
      if(lp->objfrom != NULL)
        lp->objfrom[i] = lp->objfrom[ii];
      if(lp->objtill != NULL)
        lp->objtill[i] = lp->objtill[ii];
*/
      if(lp->var_priority != NULL)
        lp->var_priority[i-1] = lp->var_priority[ii-1];
      if(lp->bb_varbranch != NULL)
        lp->bb_varbranch[i-1] = lp->bb_varbranch[ii-1];
      if(lp->var_is_free != NULL)
        lp->var_is_free[i] = lp->var_is_free[ii];
      if(lp->best_solution != NULL)
        lp->best_solution[lp->rows + i] = lp->best_solution[lp->rows + ii];
    }

    /* Set defaults */
    for(i = 0; i < delta; i++) {
      ii = base + i;
      lp->var_type[ii] = ISREAL;
      lp->sc_lobound[ii] = 0;
      lp->orig_obj[ii] = 0;
      if(lp->obj != NULL)
        lp->obj[ii] = 0;
/*
      if(lp->objfromvalue != NULL)
        lp->objfromvalue[ii] = 0;
      if(lp->objfrom != NULL)
        lp->objfrom[ii] = 0;
      if(lp->objtill != NULL)
        lp->objtill[ii] = 0;
*/
      if(lp->var_priority != NULL)
        lp->var_priority[ii-1] = ii;
      if(lp->bb_varbranch != NULL)
        lp->bb_varbranch[ii-1] = BRANCH_DEFAULT;
      if(lp->var_is_free != NULL)
        lp->var_is_free[ii] = 0;
      if(lp->best_solution != NULL)
        lp->best_solution[lp->rows + ii] = 0;
    }
  }

  /* Shift data left (delete) */
  else if(usedmap != NULL) {
    /* Assume there is no need to handle split columns, since we are doing
       this only from presolve, which comes before splitting of columns. */

    /* First update counts */
    if(lp->int_vars + lp->sc_vars > 0)
    for(ii = firstInactiveLink(usedmap); ii != 0; ii = nextInactiveLink(usedmap, ii)) {
      if(is_int(lp, ii)) {
        lp->int_vars--;
        if(SOS_is_member(lp->SOS, 0, ii))
          lp->sos_ints--;
      }
      if(is_semicont(lp, ii))
        lp->sc_vars--;
    }
    /* Shift array members */
    for(i = 1, ii = firstActiveLink(usedmap); ii != 0;
        i++, ii = nextActiveLink(usedmap, ii)) {
      if(i == ii)
        continue;
      lp->var_type[i] = lp->var_type[ii];
      lp->sc_lobound[i] = lp->sc_lobound[ii];
      lp->orig_obj[i] = lp->orig_obj[ii];
      if(lp->obj != NULL)
        lp->obj[i] = lp->obj[ii];
/*
      if(lp->objfromvalue != NULL)
        lp->objfromvalue[i] = lp->objfromvalue[ii];
      if(lp->objfrom != NULL)
        lp->objfrom[i] = lp->objfrom[ii];
      if(lp->objtill != NULL)
        lp->objtill[i] = lp->objtill[ii];
*/
      if(lp->bb_varbranch != NULL)
        lp->bb_varbranch[i-1] = lp->bb_varbranch[ii-1];
      if(lp->var_is_free != NULL)
        lp->var_is_free[i] = lp->var_is_free[ii];
      if(lp->best_solution != NULL)
        lp->best_solution[lp->rows + i] = lp->best_solution[lp->rows + ii];
    }
    /* Shift variable priority data */
    if((lp->var_priority != NULL) || (lp->sos_priority != NULL)) {
      int *colmap = NULL, k;
      allocINT(lp, &colmap, lp->columns + 1, TRUE);
      for(i = 1, ii = 0; i <= lp->columns; i++) {
        if(isActiveLink(usedmap, i)) {
          ii++;
          colmap[i] = ii;
        }
      }
      if(lp->var_priority != NULL) {
        for(i = 0, ii = 0; i < lp->columns; i++) {
          k = colmap[lp->var_priority[i]];
          if(k > 0) {
            lp->var_priority[ii] = k;
            ii++;
          }
        }
      }
      if(lp->sos_priority != NULL) {
        for(i = 0, ii = 0; i < lp->sos_vars; i++) {
          k = colmap[lp->sos_priority[i]];
          if(k > 0) {
            lp->sos_priority[ii] = k;
            ii++;
          }
        }
        lp->sos_vars = ii;
      }
      FREE(colmap);
    }

    delta = i - lp->columns - 1;
  }
  else if(delta < 0) {

    /* Fix invalid split variable data */
    if(lp->var_is_free != NULL) {
      for(i = 1; i <= lp->columns; i++)
        if(abs(lp->var_is_free[i]) >= base)
          lp->var_is_free[i] -= my_chsign(lp->var_is_free[i] < 0, delta);
    }

    /* Shift column data (excluding the basis) */
    for(i = base; i < base-delta; i++) {
      if(is_int(lp, i)) {
        lp->int_vars--;
        if(SOS_is_member(lp->SOS, 0, i))
          lp->sos_ints--;
      }
      if(is_semicont(lp, i))
        lp->sc_vars--;
    }
    for(i = base; i <= lp->columns + delta; i++) {
      ii = i - delta;
      lp->var_type[i] = lp->var_type[ii];
      lp->sc_lobound[i] = lp->sc_lobound[ii];
      lp->orig_obj[i] = lp->orig_obj[ii];
      if(lp->obj != NULL)
        lp->obj[i] = lp->obj[ii];
/*
      if(lp->objfromvalue != NULL)
        lp->objfromvalue[i] = lp->objfromvalue[ii];
      if(lp->objfrom != NULL)
        lp->objfrom[i] = lp->objfrom[ii];
      if(lp->objtill != NULL)
        lp->objtill[i] = lp->objtill[ii];
*/
      if(lp->var_priority != NULL)
        lp->var_priority[i-1] = lp->var_priority[ii-1];
      if(lp->bb_varbranch != NULL)
        lp->bb_varbranch[i-1] = lp->bb_varbranch[ii-1];
      if(lp->var_is_free != NULL)
        lp->var_is_free[i] = lp->var_is_free[ii];
      if(lp->best_solution != NULL)
        lp->best_solution[lp->rows + i] = lp->best_solution[lp->rows + ii];
    }

    /* Fix invalid variable priority data */
    if(lp->var_priority != NULL) {
      for(i = 0, ii = 0; i < lp->columns; i++)
        if(lp->var_priority[i] > base - delta)
          lp->var_priority[ii++] = lp->var_priority[i] + delta;
        else if(lp->var_priority[i] < base)
          lp->var_priority[ii++] = lp->var_priority[i];
    }
    if(lp->sos_priority != NULL) {
      for(i = 0, ii = 0; i < lp->sos_vars; i++) {
        if(lp->sos_priority[i] > base - delta)
          lp->sos_priority[ii++] = lp->sos_priority[i] + delta;
        else if(lp->sos_priority[i] < base)
          lp->sos_priority[ii++] = lp->sos_priority[i];
      }
      lp->sos_vars = ii;
    }

  }

  shift_basis(lp, lp->rows+base, delta, usedmap, FALSE);
  if(SOS_count(lp) > 0)
    SOS_shift_col(lp->SOS, 0, base, delta, usedmap, FALSE);
  shift_rowcoldata(lp, lp->rows+base, delta, usedmap, FALSE);
  inc_columns(lp, delta);

  return( TRUE );
}

/* Utility group for incrementing row and column vector storage space */
STATIC void inc_rows(lprec *lp, int delta)
{
  lp->rows += delta;
  if(lp->matA->is_roworder)
    lp->matA->columns += delta;
  else
    lp->matA->rows += delta;
}

STATIC void inc_columns(lprec *lp, int delta)
{
  lp->columns += delta;
  if(lp->matA->is_roworder)
    lp->matA->rows += delta;
  else
    lp->matA->columns += delta;
  if(get_Lrows(lp) > 0)
    lp->matL->columns += delta;
}

STATIC MYBOOL inc_rowcol_space(lprec *lp, int delta, MYBOOL isrows)
{
  int i, oldrowcolalloc, rowcolsum;

  /* Get rid of dual arrays */
  if(lp->solvecount > 0)
    free_duals(lp);

  /* Set constants */
  oldrowcolalloc = lp->sum_alloc;
  lp->sum_alloc += delta;
  rowcolsum = lp->sum_alloc + 1;

  /* Reallocate lp memory */
  if(!allocREAL(lp, &lp->upbo, rowcolsum, AUTOMATIC) ||
     !allocREAL(lp, &lp->orig_upbo, rowcolsum, AUTOMATIC) ||
     !allocREAL(lp, &lp->lowbo, rowcolsum, AUTOMATIC) ||
     !allocREAL(lp, &lp->orig_lowbo, rowcolsum, AUTOMATIC) ||
     !allocREAL(lp, &lp->solution, rowcolsum, AUTOMATIC) ||
     !allocREAL(lp, &lp->best_solution, rowcolsum, AUTOMATIC) ||
     !allocMYBOOL(lp, &lp->is_basic, rowcolsum, AUTOMATIC) ||
     !allocMYBOOL(lp, &lp->is_lower, rowcolsum, AUTOMATIC) ||
     ((lp->scalars != NULL) && !allocREAL(lp, &lp->scalars, rowcolsum, AUTOMATIC)))
    return( FALSE );

  /* Fill in default values, where appropriate */
  for(i = oldrowcolalloc+1; i < rowcolsum; i++) {
    lp->upbo[i] = lp->infinite;
    lp->orig_upbo[i] = lp->upbo[i];
    lp->lowbo[i] = 0;
    lp->orig_lowbo[i] = lp->lowbo[i];
    lp->is_basic[i] = FALSE;
    lp->is_lower[i] = TRUE;
  }

  /* Deal with scalars; the vector can be NULL and also contains Lagrangean information */
  if(lp->scalars != NULL) {
    for(i = oldrowcolalloc+1; i < rowcolsum; i++)
      lp->scalars[i] = 1;
    if(oldrowcolalloc == 0)
      lp->scalars[0] = 1;
  }

  return( inc_presolve_space(lp, delta, isrows) &&
           resizePricer(lp) );
}

STATIC MYBOOL inc_lag_space(lprec *lp, int deltarows, MYBOOL ignoreMAT)
{
  int newsize;

  if(deltarows > 0) {

    newsize = get_Lrows(lp) + deltarows;

    /* Reallocate arrays */
    if(!allocREAL(lp, &lp->lag_rhs, newsize+1, AUTOMATIC) ||
       !allocREAL(lp, &lp->lambda, newsize+1, AUTOMATIC) ||
       !allocINT(lp, &lp->lag_con_type, newsize+1, AUTOMATIC))
      return( FALSE );

    /* Reallocate the matrix (note that the row scalars are stored at index 0) */
    if(!ignoreMAT) {
      if(lp->matL == NULL)
        lp->matL = mat_create(lp, newsize, lp->columns, lp->epsvalue);
      else
        inc_matrow_space(lp->matL, deltarows);
    }
    lp->matL->rows += deltarows;

  }
  /* Handle column count expansion as special case */
  else if(!ignoreMAT) {
    inc_matcol_space(lp->matL, lp->columns_alloc-lp->matL->columns_alloc+1);
  }


  return( TRUE );
}

STATIC MYBOOL inc_row_space(lprec *lp, int deltarows)
{
  int    i, rowsum, oldrowsalloc;
  MYBOOL ok = TRUE;

  /* Adjust lp row structures */
  i = lp->rows_alloc+deltarows;
  if(lp->matA->is_roworder) {
    i -= lp->matA->columns_alloc;
    SETMIN(i, deltarows);
    if(i > 0)
      inc_matcol_space(lp->matA, i);
    rowsum = lp->matA->columns_alloc;
  }
  else {
#if 0
    if((lp->rows_alloc > 0) && (lp->rows + deltarows > lp->rows_alloc))
      i = deltarows; /* peno 25/12/06 */
    else
#endif
      i -= lp->matA->rows_alloc;
    SETMIN(i, deltarows);
    if(i > 0)
      inc_matrow_space(lp->matA, i);
    rowsum = lp->matA->rows_alloc;
  }
  if(lp->rows+deltarows > lp->rows_alloc) {

    rowsum++;
    oldrowsalloc = lp->rows_alloc;
    lp->rows_alloc = rowsum;
    deltarows = rowsum - oldrowsalloc;
    rowsum++;

    if(!allocREAL(lp, &lp->orig_rhs, rowsum, AUTOMATIC) ||
       !allocLREAL(lp, &lp->rhs, rowsum, AUTOMATIC) ||
       !allocINT(lp, &lp->row_type, rowsum, AUTOMATIC) ||
       !allocINT(lp, &lp->var_basic, rowsum, AUTOMATIC))
      return( FALSE );

    if(oldrowsalloc == 0) {
      lp->var_basic[0] = AUTOMATIC; /*TRUE;*/  /* Indicates default basis */
      lp->orig_rhs[0] = 0;
      lp->row_type[0] = ROWTYPE_OFMIN;
    }
    for(i = oldrowsalloc+1; i < rowsum; i++) {
      lp->orig_rhs[i] = 0;
      lp->rhs[i] = 0;
      lp->row_type[i] = ROWTYPE_EMPTY;
      lp->var_basic[i] = i;
    }

    /* Adjust hash name structures */
    if(lp->names_used && (lp->row_name != NULL)) {

      /* First check the hash table */
      if(lp->rowname_hashtab->size < lp->rows_alloc) {
        hashtable *ht;

        ht = copy_hash_table(lp->rowname_hashtab, lp->row_name, lp->rows_alloc + 1);
        if(ht == NULL) {
          lp->spx_status = NOMEMORY;
          return( FALSE );
        }
        free_hash_table(lp->rowname_hashtab);
        lp->rowname_hashtab = ht;
      }

      /* Then the string storage (i.e. pointer to the item's hash structure) */
      lp->row_name = (hashelem **) realloc(lp->row_name, (rowsum) * sizeof(*lp->row_name));
      if(lp->row_name == NULL) {
        lp->spx_status = NOMEMORY;
        return( FALSE );
      }
      for(i = oldrowsalloc + 1; i < rowsum; i++)
        lp->row_name[i] = NULL;
    }

    ok = inc_rowcol_space(lp, deltarows, TRUE);

  }
  return(ok);
}

STATIC MYBOOL inc_col_space(lprec *lp, int deltacols)
{
  int i,colsum, oldcolsalloc;

  i = lp->columns_alloc+deltacols;
  if(lp->matA->is_roworder) {
    i -= lp->matA->rows_alloc;
    SETMIN(i, deltacols);
    if(i > 0)
      inc_matrow_space(lp->matA, i);
    colsum = lp->matA->rows_alloc;
  }
  else {
    i -= lp->matA->columns_alloc;
    SETMIN(i, deltacols);
    if(i > 0)
      inc_matcol_space(lp->matA, i);
    colsum = lp->matA->columns_alloc;
  }

  if(lp->columns+deltacols >= lp->columns_alloc) {

    colsum++;
    oldcolsalloc = lp->columns_alloc;
    lp->columns_alloc = colsum;
    deltacols = colsum - oldcolsalloc;
    colsum++;

    /* Adjust hash name structures */
    if(lp->names_used && (lp->col_name != NULL)) {

      /* First check the hash table */
      if(lp->colname_hashtab->size < lp->columns_alloc) {
        hashtable *ht;

        ht = copy_hash_table(lp->colname_hashtab, lp->col_name, lp->columns_alloc + 1);
        if(ht != NULL) {
          free_hash_table(lp->colname_hashtab);
          lp->colname_hashtab = ht;
        }
      }

      /* Then the string storage (i.e. pointer to the item's hash structure) */
      lp->col_name = (hashelem **) realloc(lp->col_name, (colsum) * sizeof(*lp->col_name));
      for(i = oldcolsalloc+1; i < colsum; i++)
        lp->col_name[i] = NULL;
    }

    if(!allocREAL(lp, &lp->orig_obj, colsum, AUTOMATIC) ||
       !allocMYBOOL(lp, &lp->var_type, colsum, AUTOMATIC) ||
       !allocREAL(lp, &lp->sc_lobound, colsum, AUTOMATIC) ||
       ((lp->obj != NULL) && !allocREAL(lp, &lp->obj, colsum, AUTOMATIC)) ||
       ((lp->var_priority != NULL) && !allocINT(lp, &lp->var_priority, colsum-1, AUTOMATIC)) ||
       ((lp->var_is_free != NULL) && !allocINT(lp, &lp->var_is_free, colsum, AUTOMATIC)) ||
       ((lp->bb_varbranch != NULL) && !allocMYBOOL(lp, &lp->bb_varbranch, colsum-1, AUTOMATIC)))
      return( FALSE );

    /* Make sure that Lagrangean constraints have the same number of columns */
    if(get_Lrows(lp) > 0)
      inc_lag_space(lp, 0, FALSE);

    /* Update column pointers */
    for(i = MIN(oldcolsalloc, lp->columns) + 1; i < colsum; i++) {
      lp->orig_obj[i] = 0;
      if(lp->obj != NULL)
        lp->obj[i] = 0;
      lp->var_type[i] = ISREAL;
      lp->sc_lobound[i] = 0;
      if(lp->var_priority != NULL)
        lp->var_priority[i-1] = i;
    }

    if(lp->var_is_free != NULL) {
      for(i = oldcolsalloc+1; i < colsum; i++)
        lp->var_is_free[i] = 0;
    }

    if(lp->bb_varbranch != NULL) {
      for(i = oldcolsalloc; i < colsum-1; i++)
        lp->bb_varbranch[i] = BRANCH_DEFAULT;
    }

    inc_rowcol_space(lp, deltacols, FALSE);

  }
  return(TRUE);
}

/* Problem manipulation routines */

MYBOOL __WINAPI set_obj(lprec *lp, int colnr, REAL value)
{
  if(colnr <= 0)
    colnr = set_rh(lp, 0, value);
  else
    colnr = set_mat(lp, 0, colnr, value);
  return((MYBOOL) colnr);
}

MYBOOL __WINAPI set_obj_fnex(lprec *lp, int count, REAL *row, int *colno)
{
  MYBOOL chsgn = is_maxim(lp);
  int    i, ix;
  REAL   value;

  if(row == NULL)
    return( FALSE );

  else if(colno == NULL) {
    if(count <= 0)
      count = lp->columns;
    for(i = 1; i <= count; i++) {
      value = row[i];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, lp->matA->epsvalue);
#endif
      lp->orig_obj[i] = my_chsign(chsgn, scaled_mat(lp, value, 0, i));
    }
  }
  else {
    MEMCLEAR(lp->orig_obj, lp->columns+1);
    for(i = 0; i < count; i++) {
      ix = colno[i];
      value = row[i];
#ifdef DoMatrixRounding
      value = roundToPrecision(value, lp->matA->epsvalue);
#endif
      lp->orig_obj[ix] = my_chsign(chsgn, scaled_mat(lp, value, 0, ix));
    }
  }

  return(TRUE);
}

MYBOOL __WINAPI set_obj_fn(lprec *lp, REAL *row)
{
  return( set_obj_fnex(lp, 0, row, NULL) );
}

MYBOOL __WINAPI str_set_obj_fn(lprec *lp, char *row_string)
{
  int    i;
  MYBOOL ret = TRUE;
  REAL   *arow;
  char   *p, *newp;

  allocREAL(lp, &arow, lp->columns + 1, FALSE);
  p = row_string;
  for(i = 1; i <= lp->columns; i++) {
    arow[i] = (REAL) strtod(p, &newp);
    if(p == newp) {
      report(lp, IMPORTANT, "str_set_obj_fn: Bad string %s\n", p);
      lp->spx_status = DATAIGNORED;
      ret = FALSE;
      break;
    }
    else
      p = newp;
  }
  if(lp->spx_status != DATAIGNORED)
    ret = set_obj_fn(lp, arow);
  FREE(arow);
  return( ret );
}

STATIC MYBOOL append_columns(lprec *lp, int deltacolumns)
{
  if(!inc_col_space(lp, deltacolumns))
    return( FALSE );
  varmap_add(lp, lp->sum+1, deltacolumns);
  shift_coldata(lp, lp->columns+1, deltacolumns, NULL);
  return( TRUE );
}

STATIC MYBOOL append_rows(lprec *lp, int deltarows)
{
  if(!inc_row_space(lp, deltarows))
    return( FALSE );
  varmap_add(lp, lp->rows+1, deltarows);
  shift_rowdata(lp, lp->rows+1, deltarows, NULL);

  return( TRUE );
}

MYBOOL __WINAPI set_add_rowmode(lprec *lp, MYBOOL turnon)
{
  if((lp->solvecount == 0) && (turnon ^ lp->matA->is_roworder))
    return( mat_transpose(lp->matA) );
  else
    return( FALSE );
}

MYBOOL __WINAPI is_add_rowmode(lprec *lp)
{
  return(lp->matA->is_roworder);
}

MYBOOL __WINAPI set_row(lprec *lp, int rownr, REAL *row)
{
  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "set_row: Row %d out of range\n", rownr);
    return( FALSE );
  }
  if(rownr == 0)
    return( set_obj_fn(lp, row) );
  else
    return( mat_setrow(lp->matA, rownr, lp->columns, row, NULL, TRUE, TRUE) );
}

MYBOOL __WINAPI set_rowex(lprec *lp, int rownr, int count, REAL *row, int *colno)
{
  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "set_rowex: Row %d out of range\n", rownr);
    return( FALSE );
  }
  if(rownr == 0)
    return( set_obj_fnex(lp, count, row, colno) );
  else
    return( mat_setrow(lp->matA, rownr, count, row, colno, TRUE, TRUE) );
}

MYBOOL __WINAPI add_constraintex(lprec *lp, int count, REAL *row, int *colno, int constr_type, REAL rh)
{
  int    n;
  MYBOOL status = FALSE;

  if(!(constr_type == LE || constr_type == GE || constr_type == EQ)) {
    report(lp, IMPORTANT, "add_constraintex: Invalid %d constraint type\n", constr_type);
    return( status );
  }

  /* Prepare for a new row */
  if(!append_rows(lp, 1))
    return( status );

  /* Set constraint parameters, fix the slack */
  if((constr_type & ROWTYPE_CONSTRAINT) == EQ) {
    lp->equalities++;
    lp->orig_upbo[lp->rows] = 0;
    lp->upbo[lp->rows] = 0;
  }
  lp->row_type[lp->rows] = constr_type;

  if(is_chsign(lp, lp->rows) && (rh != 0))
    lp->orig_rhs[lp->rows] = -rh;
  else
    lp->orig_rhs[lp->rows] = rh;

  /* Insert the non-zero constraint values */
  if(colno == NULL && row != NULL)
    n = lp->columns;
  else
    n = count;
  mat_appendrow(lp->matA, n, row, colno, my_chsign(is_chsign(lp, lp->rows), 1.0), TRUE);
  if(!lp->varmap_locked)
    presolve_setOrig(lp, lp->rows, lp->columns);

#ifdef Paranoia
  if(lp->matA->is_roworder)
    n = lp->matA->columns;
  else
    n = lp->matA->rows;
  if(lp->rows != n) {
    report(lp, SEVERE, "add_constraintex: Row count mismatch %d vs %d\n",
                       lp->rows, n);
  }
  else if(is_BasisReady(lp) && !verify_basis(lp))
    report(lp, SEVERE, "add_constraintex: Invalid basis detected for row %d\n", lp->rows);
  else
#endif
  status = TRUE;

  return( status );
}

MYBOOL __WINAPI add_constraint(lprec *lp, REAL *row, int constr_type, REAL rh)
{
  return( add_constraintex(lp, 0, row, NULL, constr_type, rh) );
}

MYBOOL __WINAPI str_add_constraint(lprec *lp, char *row_string, int constr_type, REAL rh)
{
  int    i;
  char   *p, *newp;
  REAL   *aRow;
  MYBOOL status = FALSE;

  allocREAL(lp, &aRow, lp->columns + 1, FALSE);
  p = row_string;

  for(i = 1; i <= lp->columns; i++) {
    aRow[i] = (REAL) strtod(p, &newp);
    if(p == newp) {
      report(lp, IMPORTANT, "str_add_constraint: Bad string '%s'\n", p);
      lp->spx_status = DATAIGNORED;
      break;
    }
    else
      p = newp;
  }
  if(lp->spx_status != DATAIGNORED)
    status = add_constraint(lp, aRow, constr_type, rh);
  FREE(aRow);

  return(status);
}

STATIC MYBOOL del_constraintex(lprec *lp, LLrec *rowmap)
{
  int i;

  if(lp->equalities > 0)
  for(i = firstInactiveLink(rowmap); i != 0; i = nextInactiveLink(rowmap, i)) {
    if(is_constr_type(lp, i, EQ)) {
#ifdef Paranoia
      if(lp->equalities == 0)
        report(lp, SEVERE, "del_constraintex: Invalid count of equality constraints\n");
#endif
       lp->equalities--;
    }
  }

  varmap_delete(lp, 1, -1, rowmap);
  shift_rowdata(lp, 1, -1, rowmap);
  if(!lp->varmap_locked) {
    presolve_setOrig(lp, lp->rows, lp->columns);
    if(lp->names_used)
      del_varnameex(lp, lp->row_name, lp->rowname_hashtab, 0, rowmap);
  }

#ifdef Paranoia
  if(is_BasisReady(lp) && !verify_basis(lp))
    report(lp, SEVERE, "del_constraintex: Invalid basis detected\n");
#endif

  return(TRUE);
}
MYBOOL __WINAPI del_constraint(lprec *lp, int rownr)
{
  MYBOOL preparecompact = (MYBOOL) (rownr < 0);

  if(preparecompact)
    rownr = -rownr;
  if((rownr < 1) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "del_constraint: Attempt to delete non-existing constraint %d\n", rownr);
    return(FALSE);
  }
  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "del_constraint: Cannot delete constraint while in row entry mode.\n");
    return(FALSE);
  }

  if(is_constr_type(lp, rownr, EQ) && (lp->equalities > 0))
    lp->equalities--;

  varmap_delete(lp, my_chsign(preparecompact, rownr), -1, NULL);
  shift_rowdata(lp, my_chsign(preparecompact, rownr), -1, NULL);

/*
   peno 04.10.07
   Fixes a problem with del_constraint.
   Constraints names were not shifted and reported variable result was incorrect.
   See UnitTest1, UnitTest2

   min: -2 x3;

   c1: +x2 -x1 <= 10;
   c: 0 x3 <= 0;
   c2: +x3 +x2 +x1 <= 20;

   2 <= x3 <= 3;
   x1 <= 30;

   // del_constraint(lp, 2);

   // See write_LP and print_solution result

   // To fix, commented if(!lp->varmap_locked)

*/
  /* if(!lp->varmap_locked) */
  {
    presolve_setOrig(lp, lp->rows, lp->columns);
    if(lp->names_used)
      del_varnameex(lp, lp->row_name, lp->rowname_hashtab, rownr, NULL);
  }

#ifdef Paranoia
  if(is_BasisReady(lp) && !verify_basis(lp))
    report(lp, SEVERE, "del_constraint: Invalid basis detected at row %d\n", rownr);
#endif

  return(TRUE);
}

MYBOOL __WINAPI add_lag_con(lprec *lp, REAL *row, int con_type, REAL rhs)
{
  int  k;
  REAL sign;

  if(con_type == LE || con_type == EQ)
    sign = 1;
  else if(con_type == GE)
    sign = -1;
  else {
    report(lp, IMPORTANT, "add_lag_con: Constraint type %d not implemented\n", con_type);
    return(FALSE);
  }

  inc_lag_space(lp, 1, FALSE);

  k = get_Lrows(lp);
  lp->lag_rhs[k] = rhs * sign;
  mat_appendrow(lp->matL, lp->columns, row, NULL, sign, TRUE);
  lp->lambda[k] = 0;
  lp->lag_con_type[k] = con_type;

  return(TRUE);
}

MYBOOL __WINAPI str_add_lag_con(lprec *lp, char *row_string, int con_type, REAL rhs)
{
  int    i;
  MYBOOL ret = TRUE;
  REAL   *a_row;
  char   *p, *new_p;

  allocREAL(lp, &a_row, lp->columns + 1, FALSE);
  p = row_string;

  for(i = 1; i <= lp->columns; i++) {
    a_row[i] = (REAL) strtod(p, &new_p);
    if(p == new_p) {
      report(lp, IMPORTANT, "str_add_lag_con: Bad string '%s'\n", p);
      lp->spx_status = DATAIGNORED;
      ret = FALSE;
      break;
    }
    else
      p = new_p;
  }
  if(lp->spx_status != DATAIGNORED)
    ret = add_lag_con(lp, a_row, con_type, rhs);
  FREE(a_row);
  return( ret );
}

/* INLINE */ MYBOOL is_splitvar(lprec *lp, int colnr)
/* Two cases handled by var_is_free:

   1) LB:-Inf / UB:<Inf variables
      No helper column created, sign of var_is_free set negative with index to itself.
   2) LB:-Inf / UB: Inf (free) variables
      Sign of var_is_free set positive with index to new helper column,
      helper column created with negative var_is_free with index to the original column.

   This function helps identify the helper column in 2).
*/
{
   return((MYBOOL) ((lp->var_is_free != NULL) &&
                    (lp->var_is_free[colnr] < 0) && (-lp->var_is_free[colnr] != colnr)));
}

void del_splitvars(lprec *lp)
{
  int j, jj, i;

  if(lp->var_is_free != NULL) {
    for(j = lp->columns; j >= 1; j--)
      if(is_splitvar(lp, j)) {
        /* Check if we need to modify the basis */
        jj = lp->rows+abs(lp->var_is_free[j]);
        i = lp->rows+j;
        if(lp->is_basic[i] && !lp->is_basic[jj]) {
          i = findBasisPos(lp, i, NULL);
          set_basisvar(lp, i, jj);
        }
        /* Delete the helper column */
        del_column(lp, j);
      }
    FREE(lp->var_is_free);
  }
}

MYBOOL __WINAPI set_column(lprec *lp, int colnr, REAL *column)
{
  return( mat_setcol(lp->matA, colnr, lp->rows, column, NULL, TRUE, TRUE) );
}

MYBOOL __WINAPI set_columnex(lprec *lp, int colnr, int count, REAL *column, int *rowno)
{
  return( mat_setcol(lp->matA, colnr, count, column, rowno, TRUE, TRUE) );
}

MYBOOL __WINAPI add_columnex(lprec *lp, int count, REAL *column, int *rowno)
/* This function adds a data column to the current model; three cases handled:

    1: Prepare for column data by setting column = NULL
    2: Dense vector indicated by (rowno == NULL) over 0..count+get_Lrows() elements
    3: Sparse vector set over row vectors rowno, over 0..count-1 elements.

   NB! If the column has only one entry, this should be handled as
       a bound, but this currently is not the case  */
{
  MYBOOL status = FALSE;

 /* Prepare and shift column vectors */
  if(!append_columns(lp, 1))
    return( status );

 /* Append sparse regular constraint values */
  if(mat_appendcol(lp->matA, count, column, rowno, 1.0, TRUE) < 0)
    report(lp, SEVERE, "add_columnex: Data column %d supplied in non-ascending row index order.\n",
                       lp->columns);
  else
#ifdef Paranoia
  if(lp->columns != (lp->matA->is_roworder ? lp->matA->rows : lp->matA->columns)) {
    report(lp, SEVERE, "add_columnex: Column count mismatch %d vs %d\n",
                       lp->columns, (lp->matA->is_roworder ? lp->matA->rows : lp->matA->columns));
  }
  else if(is_BasisReady(lp) && (lp->P1extraDim == 0) && !verify_basis(lp))
    report(lp, SEVERE, "add_columnex: Invalid basis detected for column %d\n",
                       lp->columns);
  else
#endif
    status = TRUE;

  if(!lp->varmap_locked)
    presolve_setOrig(lp, lp->rows, lp->columns);

  return( status );
}

MYBOOL __WINAPI add_column(lprec *lp, REAL *column)
{
  del_splitvars(lp);
  return(add_columnex(lp, lp->rows, column, NULL));
}

MYBOOL __WINAPI str_add_column(lprec *lp, char *col_string)
{
  int  i;
  MYBOOL ret = TRUE;
  REAL *aCol;
  char *p, *newp;

  allocREAL(lp, &aCol, lp->rows + 1, FALSE);
  p = col_string;

  for(i = 0; i <= lp->rows; i++) {
    aCol[i] = (REAL) strtod(p, &newp);
    if(p == newp) {
      report(lp, IMPORTANT, "str_add_column: Bad string '%s'\n", p);
      lp->spx_status = DATAIGNORED;
      ret = FALSE;
      break;
    }
    else
      p = newp;
  }
  if(lp->spx_status != DATAIGNORED)
    ret = add_column(lp, aCol);
  FREE(aCol);
  return( ret );
}

STATIC MYBOOL del_varnameex(lprec *lp, hashelem **namelist, hashtable *ht, int varnr, LLrec *varmap)
{
  int i, n;

  /* First drop hash table entries of the deleted variables */
  if(varmap != NULL)
    i = firstInactiveLink(varmap);
  else
    i = varnr;
  while(i > 0) {
    if((namelist[i] != NULL) &&
       (namelist[i]->name != NULL))
      drophash(namelist[i]->name, namelist, ht);
    if(varmap != NULL)
      i = nextInactiveLink(varmap, i);
    else
      i = 0;
  }

  /* Then compress the name list */
  if(varmap != NULL) {
    i = firstInactiveLink(varmap);
    n = nextActiveLink(varmap, i);
    varnr = i;
  }
  else {
    i = varnr;
    n = i + 1;
  }
  while(n != 0) {
    namelist[i] = namelist[n];
    if((namelist[i] != NULL) && (namelist[i]->index > varnr))
      namelist[i]->index -= n - i;
    i++;
    if(varmap != NULL)
      n = nextActiveLink(varmap, i);
    else
      n = 0;
  }

  return( TRUE );
}
STATIC MYBOOL del_columnex(lprec *lp, LLrec *colmap)
{
  varmap_delete(lp, lp->rows+1, -1, colmap);
  shift_coldata(lp, 1, -1, colmap);
  if(!lp->varmap_locked) {
    presolve_setOrig(lp, lp->rows, lp->columns);
    if(lp->names_used)
      del_varnameex(lp, lp->col_name, lp->colname_hashtab, 0, colmap);
  }
#ifdef Paranoia
  if(is_BasisReady(lp) && (lp->P1extraDim == 0) && !verify_basis(lp))
    report(lp, SEVERE, "del_columnex: Invalid basis detected\n");
#endif

  return(TRUE);
}
MYBOOL __WINAPI del_column(lprec *lp, int colnr)
{
  MYBOOL preparecompact = (MYBOOL) (colnr < 0);

  if(preparecompact)
    colnr = -colnr;
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "del_column: Column %d out of range\n", colnr);
    return(FALSE);
  }
  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "del_column: Cannot delete column while in row entry mode.\n");
    return(FALSE);
  }

  if((lp->var_is_free != NULL) && (lp->var_is_free[colnr] > 0))
    del_column(lp, lp->var_is_free[colnr]); /* delete corresponding split column (is always after this column) */

  varmap_delete(lp, my_chsign(preparecompact, lp->rows+colnr), -1, NULL);
  shift_coldata(lp, my_chsign(preparecompact, colnr), -1, NULL);
  if(!lp->varmap_locked) {
    presolve_setOrig(lp, lp->rows, lp->columns);
    if(lp->names_used)
      del_varnameex(lp, lp->col_name, lp->colname_hashtab, colnr, NULL);
  }
#ifdef Paranoia
  if(is_BasisReady(lp) && (lp->P1extraDim == 0) && !verify_basis(lp))
    report(lp, SEVERE, "del_column: Invalid basis detected at column %d (%d)\n", colnr, lp->columns);
#endif

  return(TRUE);
}

void __WINAPI set_simplextype(lprec *lp, int simplextype)
{
  lp->simplex_strategy = simplextype;
}

int __WINAPI get_simplextype(lprec *lp)
{
  return(lp->simplex_strategy);
}

void __WINAPI set_preferdual(lprec *lp, MYBOOL dodual)
{
  if(dodual & TRUE)
    lp->simplex_strategy = SIMPLEX_DUAL_DUAL;
  else
    lp->simplex_strategy = SIMPLEX_PRIMAL_PRIMAL;
}

void __WINAPI set_bounds_tighter(lprec *lp, MYBOOL tighten)
{
  lp->tighten_on_set = tighten;
}
MYBOOL __WINAPI get_bounds_tighter(lprec *lp)
{
  return(lp->tighten_on_set);
}

MYBOOL __WINAPI set_upbo(lprec *lp, int colnr, REAL value)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_upbo: Column %d out of range\n", colnr);
    return(FALSE);
  }

#ifdef DoBorderRounding
  if(fabs(value) < lp->infinite)
    value = my_avoidtiny(value, lp->matA->epsvalue);
#endif
  value = scaled_value(lp, value, lp->rows + colnr);
  if(lp->tighten_on_set) {
    if(value < lp->orig_lowbo[lp->rows + colnr]) {
      report(lp, IMPORTANT, "set_upbo: Upperbound must be >= lowerbound\n");
      return(FALSE);
    }
    if(value < lp->orig_upbo[lp->rows + colnr]) {
      set_action(&lp->spx_action, ACTION_REBASE);
      lp->orig_upbo[lp->rows + colnr] = value;
    }
  }
  else
  {
    set_action(&lp->spx_action, ACTION_REBASE);
    if(value > lp->infinite)
      value = lp->infinite;
    lp->orig_upbo[lp->rows + colnr] = value;
  }
  return(TRUE);
}

REAL __WINAPI get_upbo(lprec *lp, int colnr)
{
  REAL value;

  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "get_upbo: Column %d out of range\n", colnr);
    return(0);
  }

  value = lp->orig_upbo[lp->rows + colnr];
  value = unscaled_value(lp, value, lp->rows + colnr);
  return(value);
}

MYBOOL __WINAPI set_lowbo(lprec *lp, int colnr, REAL value)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_lowbo: Column %d out of range\n", colnr);
    return(FALSE);
  }

#ifdef DoBorderRounding
  if(fabs(value) < lp->infinite)
    value = my_avoidtiny(value, lp->matA->epsvalue);
#endif
  value = scaled_value(lp, value, lp->rows + colnr);
  if(lp->tighten_on_set) {
    if(value > lp->orig_upbo[lp->rows + colnr]) {
      report(lp, IMPORTANT, "set_lowbo: Upper bound must be >= lower bound\n");
      return(FALSE);
    }
    if((value < 0) || (value > lp->orig_lowbo[lp->rows + colnr])) {
      set_action(&lp->spx_action, ACTION_REBASE);
      lp->orig_lowbo[lp->rows + colnr] = value;
    }
  }
  else
  {
    set_action(&lp->spx_action, ACTION_REBASE);
    if(value < -lp->infinite)
      value = -lp->infinite;
    lp->orig_lowbo[lp->rows + colnr] = value;
  }
  return(TRUE);
}

REAL __WINAPI get_lowbo(lprec *lp, int colnr)
{
  REAL value;

  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "get_lowbo: Column %d out of range\n", colnr);
    return(0);
  }

  value = lp->orig_lowbo[lp->rows + colnr];
  value = unscaled_value(lp, value, lp->rows + colnr);
  return(value);
}

MYBOOL __WINAPI set_bounds(lprec *lp, int colnr, REAL lower, REAL upper)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_bounds: Column %d out of range\n", colnr);
    return(FALSE);
  }
  if(fabs(upper - lower) < lp->epsvalue) {
    if(lower < 0)
      lower = upper;
    else
      upper = lower;
  }
  else if(lower > upper) {
    report(lp, IMPORTANT, "set_bounds: Column %d upper bound must be >= lower bound\n",
                          colnr);
    return( FALSE );
  }

  colnr += lp->rows;

  if(lower < -lp->infinite)
    lower = -lp->infinite;
  else if(lp->scaling_used) {
    lower = scaled_value(lp, lower, colnr);
#ifdef DoBorderRounding
    lower = my_avoidtiny(lower, lp->matA->epsvalue);
#endif
  }

  if(upper > lp->infinite)
    upper = lp->infinite;
  else if(lp->scaling_used) {
    upper = scaled_value(lp, upper, colnr);
#ifdef DoBorderRounding
    upper = my_avoidtiny(upper, lp->matA->epsvalue);
#endif
  }

  lp->orig_lowbo[colnr] = lower;
  lp->orig_upbo[colnr]  = upper;
  set_action(&lp->spx_action, ACTION_REBASE);

  return(TRUE);
}

MYBOOL get_bounds(lprec *lp, int column, REAL *lower, REAL *upper)
{
  if((column > lp->columns) || (column < 1)) {
    report(lp, IMPORTANT, "get_bounds: Column %d out of range", column);
    return(FALSE);
  }

  if(lower != NULL)
    *lower = get_lowbo(lp, column);
  if(upper != NULL)
    *upper = get_upbo(lp, column);

  return(TRUE);
}

MYBOOL __WINAPI set_int(lprec *lp, int colnr, MYBOOL var_type)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_int: Column %d out of range\n", colnr);
    return(FALSE);
  }

  if((lp->var_type[colnr] & ISINTEGER) != 0) {
    lp->int_vars--;
    lp->var_type[colnr] &= ~ISINTEGER;
  }
  if(var_type) {
    lp->var_type[colnr] |= ISINTEGER;
    lp->int_vars++;
    if(lp->columns_scaled && !is_integerscaling(lp))
      unscale_columns(lp);
  }
  return(TRUE);
}

MYBOOL __WINAPI is_int(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_int: Column %d out of range\n", colnr);
    return(FALSE);
  }

  return((lp->var_type[colnr] & ISINTEGER) != 0);
}

MYBOOL __WINAPI is_SOS_var(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_SOS_var: Column %d out of range\n", colnr);
    return(FALSE);
  }

  return((lp->var_type[colnr] & ISSOS) != 0);
}

int __WINAPI add_SOS(lprec *lp, char *name, int sostype, int priority, int count, int *sosvars, REAL *weights)
{
  SOSrec *SOS;
  int    k;

  if((sostype < 1) || (count < 0)) {
    report(lp, IMPORTANT, "add_SOS: Invalid SOS type definition %d\n", sostype);
    return( 0 );
  }

  /* Make sure SOSes of order 3 and higher are properly defined */
  if(sostype > 2) {
    int j;
    for(k = 1; k <= count; k++) {
      j = sosvars[k];
      if(!is_int(lp, j) || !is_semicont(lp, j)) {
        report(lp, IMPORTANT, "add_SOS: SOS3+ members all have to be integer or semi-continuous.\n");
        return( 0 );
      }
    }
  }

  /* Make size in the list to handle another SOS record */
  if(lp->SOS == NULL)
    lp->SOS = create_SOSgroup(lp);

  /* Create and append SOS to list */
  SOS = create_SOSrec(lp->SOS, name, sostype, priority, count, sosvars, weights);
  k = append_SOSgroup(lp->SOS, SOS);

  return(k);
}

STATIC int add_GUB(lprec *lp, char *name, int priority, int count, int *gubvars)
{
  SOSrec *GUB;
  int    k;

#ifdef Paranoia
  if(count < 0) {
    report(lp, IMPORTANT, "add_GUB: Invalid GUB member count %d\n", count);
    return(FALSE);
  }
#endif

  /* Make size in the list to handle another GUB record */
  if(lp->GUB == NULL)
    lp->GUB = create_SOSgroup(lp);

  /* Create and append GUB to list */
  GUB = create_SOSrec(lp->GUB, name, 1, priority, count, gubvars, NULL);
  GUB->isGUB = TRUE;
  k = append_SOSgroup(lp->GUB, GUB);

  return(k);
}

MYBOOL __WINAPI set_binary(lprec *lp, int colnr, MYBOOL must_be_bin)
{
  MYBOOL status = FALSE;

  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_binary: Column %d out of range\n", colnr);
    return( status );
  }

  status = set_int(lp, colnr, must_be_bin);
  if(status && must_be_bin)
    status = set_bounds(lp, colnr, 0, 1);
  return( status );
}

MYBOOL __WINAPI is_binary(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_binary: Column %d out of range\n", colnr);
    return(FALSE);
  }

  return((MYBOOL) (((lp->var_type[colnr] & ISINTEGER) != 0) &&
                    (get_lowbo(lp, colnr) == 0) &&
                    (fabs(get_upbo(lp, colnr) - 1) < lp->epsprimal)));
}

MYBOOL __WINAPI set_unbounded(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_unbounded: Column %d out of range\n", colnr);
    return( FALSE );
  }

  return( set_bounds(lp, colnr, -lp->infinite, lp->infinite) );
}

MYBOOL __WINAPI is_unbounded(lprec *lp, int colnr)
{
  MYBOOL test;

  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_unbounded: Column %d out of range\n", colnr);
    return(FALSE);
  }

  test = is_splitvar(lp, colnr);
  if(!test) {
    colnr += lp->rows;
    test = (MYBOOL) ((lp->orig_lowbo[colnr] <= -lp->infinite) &&
                     (lp->orig_upbo[colnr] >= lp->infinite));
  }
  return( test );
}

MYBOOL __WINAPI is_negative(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_negative: Column %d out of range\n", colnr);
    return( FALSE );
  }

  colnr += lp->rows;
  return( (MYBOOL) ((lp->orig_upbo[colnr] <= 0) &&
                    (lp->orig_lowbo[colnr] < 0)) );
}

MYBOOL __WINAPI set_var_weights(lprec *lp, REAL *weights)
{
  if(lp->var_priority != NULL) {
    FREE(lp->var_priority);
  }
  if(weights != NULL) {
    int n;
    allocINT(lp, &lp->var_priority, lp->columns_alloc, FALSE);
    for(n = 0; n < lp->columns; n++) {
      lp->var_priority[n] = n+1;
    }
    n = sortByREAL(lp->var_priority, weights, lp->columns, 0, FALSE);
  }
  return(TRUE);
}

MYBOOL __WINAPI set_var_priority(lprec *lp)
/* Experimental automatic variable ordering/priority setting */
{
  MYBOOL status = FALSE;

  if(is_bb_mode(lp, NODE_AUTOORDER) &&
     (lp->var_priority == NULL) &&
     (SOS_count(lp) == 0)) {

    REAL *rcost = NULL;
    int  i, j, *colorder = NULL;

    allocINT(lp, &colorder, lp->columns+1, FALSE);

    /* Create an "optimal" B&B variable ordering; this MDO-based routine
       returns column indeces in an increasing order of co-dependency.
       It can be argued that arranging the columns in right-to-left
       MDO order should tend to minimize the consequences of choosing the
       wrong variable by reducing the average B&B depth. */
    colorder[0] = lp->columns;
    for(j = 1; j <= lp->columns; j++)
      colorder[j] = lp->rows+j;
    i = getMDO(lp, NULL, colorder, NULL, FALSE);

    /* Map to variable weight */
    allocREAL(lp, &rcost, lp->columns+1, FALSE);
    for(j = lp->columns; j > 0; j--) {
      i = colorder[j]-lp->rows;
      rcost[i] = -j;
    }

   /* Establish the MIP variable priorities */
    set_var_weights(lp, rcost+1);

    FREE(rcost);
    FREE(colorder);
    status = TRUE;
  }

  return( status );
}

int __WINAPI get_var_priority(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "get_var_priority: Column %d out of range\n", colnr);
    return(FALSE);
  }

  if(lp->var_priority == NULL)
    return(colnr);
  else
    return(lp->var_priority[colnr - 1]);
}

MYBOOL __WINAPI set_semicont(lprec *lp, int colnr, MYBOOL must_be_sc)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_semicont: Column %d out of range\n", colnr);
    return(FALSE);
  }

  if(lp->sc_lobound[colnr] != 0) {
    lp->sc_vars--;
    lp->var_type[colnr] &= ~ISSEMI;
  }
  lp->sc_lobound[colnr] = must_be_sc;
  if(must_be_sc) {
    lp->var_type[colnr] |= ISSEMI;
    lp->sc_vars++;
  }
  return(TRUE);
}

MYBOOL __WINAPI is_semicont(lprec *lp, int colnr)
{
  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "is_semicont: Column %d out of range\n", colnr);
    return(FALSE);
  }

  return((lp->var_type[colnr] & ISSEMI) != 0);
}

MYBOOL __WINAPI set_rh(lprec *lp, int rownr, REAL value)
{
  if((rownr > lp->rows) || (rownr < 0)) {
    report(lp, IMPORTANT, "set_rh: Row %d out of range\n", rownr);
    return(FALSE);
  }

  if(((rownr == 0) && (!is_maxim(lp))) ||
     ((rownr > 0) && is_chsign(lp, rownr)))    /* setting of RHS of OF IS meaningful */
    value = my_flipsign(value);
  if(fabs(value) > lp->infinite) {
    if(value < 0)
      value = -lp->infinite;
    else
      value = lp->infinite;
  }
#ifdef DoBorderRounding
  else
    value = my_avoidtiny(value, lp->matA->epsvalue);
#endif
  value = scaled_value(lp, value, rownr);
  lp->orig_rhs[rownr] = value;
  set_action(&lp->spx_action, ACTION_RECOMPUTE);
  return(TRUE);
}

REAL __WINAPI get_rh(lprec *lp, int rownr)
{
  REAL value;

  if((rownr > lp->rows) || (rownr < 0)) {
    report(lp, IMPORTANT, "get_rh: Row %d out of range", rownr);
    return( 0.0 );
  }

  value = lp->orig_rhs[rownr];
  if (((rownr == 0) && !is_maxim(lp)) ||
      ((rownr > 0) && is_chsign(lp, rownr)))    /* setting of RHS of OF IS meaningful */
    value = my_flipsign(value);
  value = unscaled_value(lp, value, rownr);
  return(value);
}

REAL get_rh_upper(lprec *lp, int rownr)
{
  REAL value, valueR;

  value = lp->orig_rhs[rownr];
  if(is_chsign(lp, rownr)) {
    valueR = lp->orig_upbo[rownr];
    if(is_infinite(lp, valueR))
      return(lp->infinite);
    value = my_flipsign(value);
    value += valueR;
  }
  value = unscaled_value(lp, value, rownr);
  return(value);
}

REAL get_rh_lower(lprec *lp, int rownr)
{
  REAL value, valueR;

  value = lp->orig_rhs[rownr];
  if(is_chsign(lp, rownr))
    value = my_flipsign(value);
  else {
    valueR = lp->orig_upbo[rownr];
    if(is_infinite(lp, valueR))
      return(-lp->infinite);
    value -= valueR;
  }
  value = unscaled_value(lp, value, rownr);
  return(value);
}

MYBOOL set_rh_upper(lprec *lp, int rownr, REAL value)
{
  if(rownr > lp->rows || rownr < 1) {
    report(lp, IMPORTANT, "set_rh_upper: Row %d out of range", rownr);
    return(FALSE);
  }

 /* First scale the value */
  value = scaled_value(lp, value, rownr);

 /* orig_rhs stores the upper bound assuming a < constraint;
    If we have a > constraint, we must adjust the range instead */
  if(is_chsign(lp, rownr)) {
    if(is_infinite(lp, value))
      lp->orig_upbo[rownr] = lp->infinite;
    else {
#ifdef Paranoia
      if(value + lp->orig_rhs[rownr] < 0) {
        report(lp, SEVERE, "set_rh_upper: Invalid negative range in row %d\n",
                           rownr);
        return(FALSE);
      }
#endif
#ifdef DoBorderRounding
      lp->orig_upbo[rownr] = my_avoidtiny(value + lp->orig_rhs[rownr], lp->epsvalue);
#else
      lp->orig_upbo[rownr] = value + lp->orig_rhs[rownr];
#endif
    }
  }
  else {
    /* If there is a constraint range, then this has to be adjusted also */
    if(!is_infinite(lp, lp->orig_upbo[rownr])) {
      lp->orig_upbo[rownr] -= lp->orig_rhs[rownr] - value;
      my_roundzero(lp->orig_upbo[rownr], lp->epsvalue);
      if(lp->orig_upbo[rownr] < 0) {
        report(lp, IMPORTANT, "set_rh_upper: Negative bound set for constraint %d made 0\n", rownr);
        lp->orig_upbo[rownr] = 0;
      }
    }
    lp->orig_rhs[rownr] = value;
  }
  return(TRUE);
}

MYBOOL set_rh_lower(lprec *lp, int rownr, REAL value)
{
  if(rownr > lp->rows || rownr < 1) {
    report(lp, IMPORTANT, "set_rh_lower: Row %d out of range", rownr);
    return(FALSE);
  }

 /* First scale the value */
  value = scaled_value(lp, value, rownr);

 /* orig_rhs stores the upper bound assuming a < constraint;
    If we have a < constraint, we must adjust the range instead */
  if(!is_chsign(lp, rownr)) {
    if(is_infinite(lp, value))
      lp->orig_upbo[rownr] = lp->infinite;
    else {
#ifdef Paranoia
      if(lp->orig_rhs[rownr] - value < 0) {
        report(lp, SEVERE, "set_rh_lower: Invalid negative range in row %d\n",
                           rownr);
        return(FALSE);
      }
#endif
#ifdef DoBorderRounding
      lp->orig_upbo[rownr] = my_avoidtiny(lp->orig_rhs[rownr] - value, lp->epsvalue);
#else
      lp->orig_upbo[rownr] = lp->orig_rhs[rownr] - value;
#endif
    }
  }
  else {
    value = my_flipsign(value);
    /* If there is a constraint range, then this has to be adjusted also */
    if(!is_infinite(lp, lp->orig_upbo[rownr])) {
      lp->orig_upbo[rownr] -= lp->orig_rhs[rownr] - value;
      my_roundzero(lp->orig_upbo[rownr], lp->epsvalue);
      if(lp->orig_upbo[rownr] < 0) {
        report(lp, IMPORTANT, "set_rh_lower: Negative bound set for constraint %d made 0\n", rownr);
        lp->orig_upbo[rownr] = 0;
      }
    }
    lp->orig_rhs[rownr] = value;
  }
  return(TRUE);
}

MYBOOL __WINAPI set_rh_range(lprec *lp, int rownr, REAL deltavalue)
{
  if((rownr > lp->rows) || (rownr < 1)) {
    report(lp, IMPORTANT, "set_rh_range: Row %d out of range", rownr);
    return(FALSE);
  }

  deltavalue = scaled_value(lp, deltavalue, rownr);
  if(deltavalue > lp->infinite)
    deltavalue = lp->infinite;
  else if(deltavalue < -lp->infinite)
    deltavalue = -lp->infinite;
#ifdef DoBorderRounding
  else
    deltavalue = my_avoidtiny(deltavalue, lp->matA->epsvalue);
#endif

  if(fabs(deltavalue) < lp->epsprimal) {
    /* Conversion to EQ */
    set_constr_type(lp, rownr, EQ);
  }
  else if(is_constr_type(lp, rownr, EQ)) {
    /* EQ with a non-zero range */
    if(deltavalue > 0)
      set_constr_type(lp, rownr, GE);
    else
      set_constr_type(lp, rownr, LE);
    lp->orig_upbo[rownr] = fabs(deltavalue);
  }
  else {
    /* Modify GE/LE ranges */
    lp->orig_upbo[rownr] = fabs(deltavalue);
  }

  return(TRUE);
}

REAL __WINAPI get_rh_range(lprec *lp, int rownr)
{
  if((rownr > lp->rows) || (rownr < 0)) {
    report(lp, IMPORTANT, "get_rh_range: row %d out of range\n", rownr);
    return(FALSE);
  }

  if(lp->orig_upbo[rownr] >= lp->infinite)
    return(lp->orig_upbo[rownr]);
  else
    return(unscaled_value(lp, lp->orig_upbo[rownr], rownr));
}

void __WINAPI set_rh_vec(lprec *lp, REAL *rh)
{
  int  i;
  REAL rhi;

  for(i = 1; i <= lp->rows; i++) {
    rhi = rh[i];
#ifdef DoBorderRounding
    rhi = my_avoidtiny(rhi, lp->matA->epsvalue);
#endif
    lp->orig_rhs[i] = my_chsign(is_chsign(lp, i), scaled_value(lp, rhi, i));
  }
  set_action(&lp->spx_action, ACTION_RECOMPUTE);
}

MYBOOL __WINAPI str_set_rh_vec(lprec *lp, char *rh_string)
{
  int  i;
  MYBOOL ret = TRUE;
  REAL *newrh;
  char *p, *newp;

  allocREAL(lp, &newrh, lp->rows + 1, TRUE);
  p = rh_string;

  for(i = 1; i <= lp->rows; i++) {
    newrh[i] = (REAL) strtod(p, &newp);
    if(p == newp) {
      report(lp, IMPORTANT, "str_set_rh_vec: Bad string %s\n", p);
      lp->spx_status = DATAIGNORED;
      ret = FALSE;
      break;
    }
    else
      p = newp;
  }
  if(!(lp->spx_status == DATAIGNORED))
    set_rh_vec(lp, newrh);
  FREE(newrh);
  return( ret );
}

void __WINAPI set_sense(lprec *lp, MYBOOL maximize)
{
  maximize = (MYBOOL) (maximize != FALSE);
  if(is_maxim(lp) != maximize) {
    int i;
    if(is_infinite(lp, lp->bb_heuristicOF))
      lp->bb_heuristicOF = my_chsign(maximize, lp->infinite);
    if(is_infinite(lp, lp->bb_breakOF))
      lp->bb_breakOF = my_chsign(maximize, -lp->infinite);
    lp->orig_rhs[0] = my_flipsign(lp->orig_rhs[0]);
    for(i = 1; i <= lp->columns; i++)
      lp->orig_obj[i] = my_flipsign(lp->orig_obj[i]);
    set_action(&lp->spx_action, ACTION_REINVERT | ACTION_RECOMPUTE);
  }
  if(maximize)
    lp->row_type[0] = ROWTYPE_OFMAX;
  else
    lp->row_type[0] = ROWTYPE_OFMIN;
}

void __WINAPI set_maxim(lprec *lp)
{
  set_sense(lp, TRUE);
}

void __WINAPI set_minim(lprec *lp)
{
  set_sense(lp, FALSE);
}

MYBOOL __WINAPI is_maxim(lprec *lp)
{
  return( (MYBOOL) ((lp->row_type != NULL) &&
                     ((lp->row_type[0] & ROWTYPE_CHSIGN) == ROWTYPE_GE)) );
}

MYBOOL __WINAPI set_constr_type(lprec *lp, int rownr, int con_type)
{
  MYBOOL oldchsign;

  if(rownr > lp->rows+1 || rownr < 1) {
    report(lp, IMPORTANT, "set_constr_type: Row %d out of range\n", rownr);
    return( FALSE );
  }

  /* Prepare for a new row */
  if((rownr > lp->rows) && !append_rows(lp, rownr-lp->rows))
    return( FALSE );

  /* Update the constraint type data */
  if(is_constr_type(lp, rownr, EQ))
    lp->equalities--;

  if((con_type & ROWTYPE_CONSTRAINT) == EQ) {
    lp->equalities++;
    lp->orig_upbo[rownr] = 0;
  }
  else if(((con_type & LE) > 0) || ((con_type & GE) > 0) || (con_type == FR))
    lp->orig_upbo[rownr] = lp->infinite;
  else {
    report(lp, IMPORTANT, "set_constr_type: Constraint type %d not implemented (row %d)\n",
                          con_type, rownr);
    return( FALSE );
  }

  /* Change the signs of the row, if necessary */
  oldchsign = is_chsign(lp, rownr);
  if(con_type == FR)
    lp->row_type[rownr] = LE;
  else
    lp->row_type[rownr] = con_type;
  if(oldchsign != is_chsign(lp, rownr)) {
    MATrec *mat = lp->matA;

    if(mat->is_roworder)
      mat_multcol(mat, rownr, -1, FALSE);
    else
      mat_multrow(mat, rownr, -1);
    if(lp->orig_rhs[rownr] != 0)
      lp->orig_rhs[rownr] *= -1;
    set_action(&lp->spx_action, ACTION_RECOMPUTE);
  }
  if(con_type == FR)
      lp->orig_rhs[rownr] = lp->infinite;

  set_action(&lp->spx_action, ACTION_REINVERT);
  lp->basis_valid = FALSE;

  return( TRUE );
}

/* INLINE */ MYBOOL is_chsign(lprec *lp, int rownr)
{
  return( (MYBOOL) ((lp->row_type[rownr] & ROWTYPE_CONSTRAINT) == ROWTYPE_CHSIGN) );
}

MYBOOL __WINAPI is_constr_type(lprec *lp, int rownr, int mask)
{
  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "is_constr_type: Row %d out of range\n", rownr);
    return( FALSE );
  }
  return( (MYBOOL) ((lp->row_type[rownr] & ROWTYPE_CONSTRAINT) == mask));
}

int __WINAPI get_constr_type(lprec *lp, int rownr)
{
  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "get_constr_type: Row %d out of range\n", rownr);
    return(-1);
  }
  return( lp->row_type[rownr] );
}
REAL __WINAPI get_constr_value(lprec *lp, int rownr, int count, REAL *primsolution, int *nzindex)
{
  int    i;
  REAL   value = 0.0;
  MATrec *mat = lp->matA;

  if((rownr < 0) || (rownr > get_Nrows(lp)))
    return( value );

  /* First do validation and initialization of applicable primal solution */
  if(!mat_validate(mat) || ((primsolution == NULL) && (lp->solvecount == 0)))
    return( value );
  i = get_Ncolumns(lp);
  if((primsolution != NULL) && (nzindex == NULL) &&
     ((count <= 0) || (count > i)))
    count = i;
  if(primsolution == NULL) {
    get_ptr_variables(lp, &primsolution);
    primsolution--;
    nzindex = NULL;
    count = i;
  }

  /* Do objective or constraint, as specified */
  if(rownr == 0) {
    value += get_rh(lp, 0);
    if(nzindex != NULL)
      for(i = 0; i < count; i++)
        value += get_mat(lp, 0, nzindex[i]) * primsolution[i];
    else
      for(i = 1; i <= count; i++)
        value += get_mat(lp, 0, i) * primsolution[i];
  }
  else {
    if(nzindex != NULL) {
      for(i = 0; i < count; i++)
        value += get_mat(lp, rownr, nzindex[i]) * primsolution[i];
    }
    else {
      int j;

      for(i = mat->row_end[rownr-1]; i < mat->row_end[rownr]; i++) {
        j = ROW_MAT_COLNR(i);
        value += unscaled_mat(lp, ROW_MAT_VALUE(i), rownr, j) * primsolution[j];
      }
      value = my_chsign(is_chsign(lp, rownr), value);
    }
  }
  return( value );
}

STATIC char *get_str_constr_class(lprec *lp, int con_class)
{
  switch(con_class) {
    case ROWCLASS_Unknown:     return("Unknown");
    case ROWCLASS_Objective:   return("Objective");
    case ROWCLASS_GeneralREAL: return("General REAL");
    case ROWCLASS_GeneralMIP:  return("General MIP");
    case ROWCLASS_GeneralINT:  return("General INT");
    case ROWCLASS_GeneralBIN:  return("General BIN");
    case ROWCLASS_KnapsackINT: return("Knapsack INT");
    case ROWCLASS_KnapsackBIN: return("Knapsack BIN");
    case ROWCLASS_SetPacking:  return("Set packing");
    case ROWCLASS_SetCover:    return("Set cover");
    case ROWCLASS_GUB:         return("GUB");
    default:                   return("Error");
  }
}

STATIC char *get_str_constr_type(lprec *lp, int con_type)
{
  switch(con_type) {
    case FR: return("FR");
    case LE: return("LE");
    case GE: return("GE");
    case EQ: return("EQ");
    default: return("Error");
  }
}

STATIC int get_constr_class(lprec *lp, int rownr)
{
  int    aBIN = 0, aINT = 0, aREAL = 0,
         xBIN = 0, xINT = 0, xREAL = 0;
  int    j, elmnr, elmend, nelm;
  MYBOOL chsign;
  REAL   a;
  MATrec *mat = lp->matA;

  if((rownr < 1) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "get_constr_class: Row %d out of range\n", rownr);
    return( ROWCLASS_Unknown );
  }
  mat_validate(mat);

  /* Tally counts of constraint variable types and coefficients */
  if(rownr == 0) {
    elmnr = 1;
    elmend = lp->columns;
    nelm = 0;
  }
  else {
    elmnr  = mat->row_end[rownr - 1];
    elmend = mat->row_end[rownr];
    nelm = elmend - elmnr;
  }
  chsign = is_chsign(lp, rownr);
  for(; elmnr < elmend; elmnr++) {
    if(rownr == 0) {
      a = lp->orig_obj[elmnr];
      if(a == 0)
        continue;
      j = elmnr;
    }
    else {
      j = ROW_MAT_COLNR(elmnr);
      a = ROW_MAT_VALUE(elmnr);
    }
    a = unscaled_mat(lp, my_chsign(chsign, a), rownr, j);
    if(is_binary(lp, j))
      xBIN++;
    else if((get_lowbo(lp, j) >= 0) && is_int(lp, j))
      xINT++;
    else
      xREAL++;  /* Includes integer variables with negative lower bound */

    if(fabs(a-1.0) < lp->epsvalue)
      aBIN++;
    else if((a > 0) && (fabs(floor(a+lp->epsvalue)-a) < lp->epsvalue))
      aINT++;
    else
      aREAL++;  /* Includes negative integer-valued coefficients */
  }

  /* Get the constraint type and the RHS */
  if(rownr == 0)
    return( ROWCLASS_Objective );
  j = get_constr_type(lp, rownr);
  a = get_rh(lp, rownr);

  /* Determine the constraint class */
  if((aBIN == nelm) && (xBIN == nelm) && (a >= 1)) {
    if(a > 1)
      j = ROWCLASS_KnapsackBIN;
    else if(j == EQ)
      j = ROWCLASS_GUB;
    else if(j == LE)
      j = ROWCLASS_SetCover;
    else
      j = ROWCLASS_SetPacking;
  }
  else if((aINT == nelm) && (xINT == nelm) && (a >= 1))
    j = ROWCLASS_KnapsackINT;
  else if(xBIN == nelm)
    j = ROWCLASS_GeneralBIN;
  else if(xINT == nelm)
    j = ROWCLASS_GeneralINT;
  else if((xREAL > 0) && (xINT+xBIN > 0))
    j = ROWCLASS_GeneralMIP;
  else
    j = ROWCLASS_GeneralREAL;

  return( j );
}

REAL __WINAPI get_mat(lprec *lp, int rownr, int colnr)
{
  REAL value;
  int  elmnr;

  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "get_mat: Row %d out of range", rownr);
    return(0);
  }
  if((colnr < 1) || (colnr > lp->columns)) {
    report(lp, IMPORTANT, "get_mat: Column %d out of range", colnr);
    return(0);
  }
  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "get_mat: Cannot read a matrix value while in row entry mode.\n");
    return(0);
  }

  if(rownr == 0) {
    value = lp->orig_obj[colnr];
    value = my_chsign(is_chsign(lp, rownr), value);
    value = unscaled_mat(lp, value, rownr, colnr);
  }
  else {
    elmnr = mat_findelm(lp->matA, rownr, colnr);
    if(elmnr >= 0) {
      MATrec *mat = lp->matA;
      value = my_chsign(is_chsign(lp, rownr), COL_MAT_VALUE(elmnr));
      value = unscaled_mat(lp, value, rownr, colnr);
    }
    else
      value = 0;
  }
  return(value);
}

REAL __WINAPI get_mat_byindex(lprec *lp, int matindex, MYBOOL isrow, MYBOOL adjustsign)
/* Note that this function does not adjust for sign-changed GT constraints! */
{
  int  *rownr, *colnr;
  REAL *value, result;

  mat_get_data(lp, matindex, isrow, &rownr, &colnr, &value);
  if(adjustsign)
    result = (*value) * (is_chsign(lp, *rownr) ? -1 : 1);
  else
    result = *value;
  if(lp->scaling_used)
    return( unscaled_mat(lp, result, *rownr, *colnr) );
  else
    return( result );
}

int __WINAPI get_rowex(lprec *lp, int rownr, REAL *row, int *colno)
{
  MYBOOL isnz;
  int    j, countnz = 0;
  REAL   a;

  if((rownr < 0) || (rownr > lp->rows)) {
    report(lp, IMPORTANT, "get_rowex: Row %d out of range\n", rownr);
    return( -1 );
  }
  if(lp->matA->is_roworder) {
    report(lp, IMPORTANT, "get_rowex: Cannot return a matrix row while in row entry mode.\n");
    return( -1 );
  }

  if((rownr == 0) || !mat_validate(lp->matA)) {
    for(j = 1; j <= lp->columns; j++) {
      a = get_mat(lp,rownr,j);
      isnz = (a != 0);
      if(colno == NULL)
        row[j] = a;
      else if(isnz) {
        row[countnz]   = a;
        colno[countnz] = j;
      }
      if(isnz)
        countnz++;
    }
  }
  else {
    MYBOOL chsign;
    int    ie, i;
    MATrec *mat = lp->matA;

    i = mat->row_end[rownr-1];
    ie = mat->row_end[rownr];
    chsign = is_chsign(lp, rownr);
    if(colno == NULL)
      MEMCLEAR(row, lp->columns+1);
    for(; i < ie; i++) {
      j = ROW_MAT_COLNR(i);
      a = get_mat_byindex(lp, i, TRUE, FALSE);
      a = my_chsign(chsign, a);
      if(colno == NULL)
        row[j] = a;
      else {
        row[countnz]   = a;
        colno[countnz] = j;
      }
      countnz++;
    }
  }
  return( countnz );
}

MYBOOL __WINAPI get_row(lprec *lp, int rownr, REAL *row)
{
  return((MYBOOL) (get_rowex(lp, rownr, row, NULL) >= 0) );
}

int __WINAPI get_columnex(lprec *lp, int colnr, REAL *column, int *nzrow)
{
  int    n = 0, i, ii, ie, *rownr;
  REAL   hold, *value;
  MATrec *mat = lp->matA;

  if((colnr > lp->columns) || (colnr < 1)) {
    report(lp, IMPORTANT, "get_columnex: Column %d out of range\n", colnr);
    return( -1 );
  }
  if(mat->is_roworder) {
    report(lp, IMPORTANT, "get_columnex: Cannot return a column while in row entry mode\n");
    return( -1 );
  }

  /* Add the objective function */
  if(nzrow == NULL)
    MEMCLEAR(column, lp->rows + 1);
  hold = get_mat(lp, 0, colnr);
  if(nzrow == NULL) {
    column[n] = hold;
    if(hold != 0)
      n++;
  }
  else if(hold != 0) {
    column[n] = hold;
    nzrow[n] = 0;
    n++;
  }

  i  = lp->matA->col_end[colnr - 1];
  ie = lp->matA->col_end[colnr];
  if(nzrow == NULL)
    n += ie - i;
  rownr = &COL_MAT_ROWNR(i);
  value = &COL_MAT_VALUE(i);
  for(; i < ie;
      i++, rownr += matRowColStep, value += matValueStep) {
    ii = *rownr;

    hold = my_chsign(is_chsign(lp, ii), *value);
    hold = unscaled_mat(lp, hold, ii, colnr);
    if(nzrow == NULL)
      column[ii] = hold;
    else if(hold != 0) {
      column[n] = hold;
      nzrow[n] = ii;
      n++;
    }
  }
  return( n );
}

MYBOOL __WINAPI get_column(lprec *lp, int colnr, REAL *column)
{
  return( (MYBOOL) (get_columnex(lp, colnr, column, NULL) >= 0) );
}

STATIC void set_OF_override(lprec *lp, REAL *ofVector)
/* The purpose of this function is to set, or clear if NULL, the
   ofVector[0..columns] as the active objective function instead of
   the one stored in the A-matrix. See also lag_solve().*/
{
  lp->obj = ofVector;
}

MYBOOL modifyOF1(lprec *lp, int index, REAL *ofValue, REAL mult)
/* Adjust objective function values for primal/dual phase 1, if appropriate */
{
  MYBOOL accept = TRUE;
/*  static MYBOOL accept;
  accept = TRUE;  */

  /* Primal simplex: Set user variables to zero or BigM-scaled */
  if(((lp->simplex_mode & SIMPLEX_Phase1_PRIMAL) != 0) && (abs(lp->P1extraDim) > 0)) {
#ifndef Phase1EliminateRedundant
    if(lp->P1extraDim < 0) {
      if(index > lp->sum + lp->P1extraDim)
        accept = FALSE;
    }
    else
#endif
    if((index <= lp->sum - lp->P1extraDim) || (mult == 0)) {
      if((mult == 0) || (lp->bigM == 0))
        accept = FALSE;
      else
        (*ofValue) /= lp->bigM;
    }
  }

  /* Dual simplex: Subtract P1extraVal from objective function values */
  else if(((lp->simplex_mode & SIMPLEX_Phase1_DUAL) != 0) && (index > lp->rows)) {
#if 1  /* This may help increase sparsity of the (extended) basis matrix;
         Can it introduce degeneracy in some cases? */
    if((lp->P1extraVal != 0) && (lp->orig_obj[index - lp->rows] > 0))
      *ofValue = 0;
    else
#endif
    {
      *ofValue -= lp->P1extraVal;
#if 0
      if(is_action(lp->anti_degen, ANTIDEGEN_RHSPERTURB))
        *ofValue -= rand_uniform(lp, lp->epsperturb);
#endif
    }
  }

  /* Do scaling and test for zero */
  if(accept) {
    (*ofValue) *= mult;
    if(fabs(*ofValue) < lp->epsmachine) {
      (*ofValue) = 0;
      accept = FALSE;
    }
  }
  else
    (*ofValue) = 0;

  return( accept );
}

STATIC void set_OF_p1extra(lprec *lp, REAL p1extra)
{
  int  i;
  REAL *value;

  if(lp->spx_trace)
    report(lp, DETAILED, "set_OF_p1extra: Set dual objective offset to %g at iter %.0f.\n",
                          p1extra, (double) get_total_iter(lp));
  lp->P1extraVal = p1extra;
  if(lp->obj == NULL)
    allocREAL(lp, &lp->obj, lp->columns_alloc+1, TRUE);
  for(i = 1, value = lp->obj+1; i <= lp->columns; i++, value++) {
    *value = lp->orig_obj[i];
    modifyOF1(lp, lp->rows + i, value, 1.0);
  }
}

STATIC void unset_OF_p1extra(lprec *lp)
{
  lp->P1extraVal = 0;
  FREE(lp->obj);
}

REAL __WINAPI get_OF_active(lprec *lp, int varnr, REAL mult)
{
  int  colnr = varnr - lp->rows;
  REAL holdOF = 0;

#ifdef Paranoia
  if((colnr <= 0) || (colnr > lp->columns)) {
    report(lp, SEVERE, "get_OF_active: Invalid column index %d supplied\n", colnr);
  }
  else
#endif
  if(lp->obj == NULL) {
    if(colnr > 0)
      holdOF = lp->orig_obj[colnr];
    modifyOF1(lp, varnr, &holdOF, mult);
  }
  else if(colnr > 0)
    holdOF = lp->obj[colnr] * mult;

  return( holdOF );
}

STATIC MYBOOL is_OF_nz(lprec *lp, int colnr)
{
  return( (MYBOOL) (lp->orig_obj[colnr] != 0) );
}

STATIC int singleton_column(lprec *lp, int row_nr, REAL *column, int *nzlist, REAL value, int *maxabs)
{
  int nz = 1;

  if(nzlist == NULL) {
    MEMCLEAR(column, lp->rows + 1);
    column[row_nr] = value;
  }
  else {
    column[nz] = value;
    nzlist[nz] = row_nr;
  }

  if(maxabs != NULL)
    *maxabs = row_nr;
  return( nz );
}

STATIC int expand_column(lprec *lp, int col_nr, REAL *column, int *nzlist, REAL mult, int *maxabs)
{
  int     i, ie, j, maxidx, nzcount;
  REAL    value, maxval;
  MATrec  *mat = lp->matA;
  REAL    *matValue;
  int     *matRownr;

  /* Retrieve a column from the user data matrix A */
  maxval = 0;
  maxidx = -1;
  if(nzlist == NULL) {
    MEMCLEAR(column, lp->rows + 1);
    i  = mat->col_end[col_nr - 1];
    ie = mat->col_end[col_nr];
    matRownr = &COL_MAT_ROWNR(i);
    matValue = &COL_MAT_VALUE(i);
    nzcount = i;
    for(; i < ie;
        i++, matRownr += matRowColStep, matValue += matValueStep) {
      j = *matRownr;
      value = *matValue;
      if(j > 0) {
        value *= mult;
        if(fabs(value) > maxval) {
          maxval = fabs(value);
          maxidx = j;
        }
      }
      column[j] = value;
    }
    nzcount = i - nzcount;

    /* Get the objective as row 0, optionally adjusting the objective for phase 1 */
    if(lp->obj_in_basis) {
      column[0] = get_OF_active(lp, lp->rows+col_nr, mult);
      if(column[0] != 0)
        nzcount++;
    }
  }
  else {
    nzcount = 0;

    /* Get the objective as row 0, optionally adjusting the objective for phase 1 */
    if(lp->obj_in_basis) {
      value = get_OF_active(lp, lp->rows+col_nr, mult);
      if(value != 0) {
        nzcount++;
        nzlist[nzcount] = 0;
        column[nzcount] = value;
      }
    }

    /* Loop over the non-zero column entries */
    i  = mat->col_end[col_nr - 1];
    ie = mat->col_end[col_nr];
    matRownr = &COL_MAT_ROWNR(i);
    matValue = &COL_MAT_VALUE(i);
    for(; i < ie;
        i++, matRownr += matRowColStep, matValue += matValueStep) {
      j = *matRownr;
      value = (*matValue) * mult;
      nzcount++;
      nzlist[nzcount] = j;
      column[nzcount] = value;
      if(fabs(value) > maxval) {
        maxval = fabs(value);
        maxidx = nzcount;
      }
    }
  }

  if(maxabs != NULL)
    *maxabs = maxidx;
  return( nzcount );
}


/* Retrieve a column vector from the data matrix [1..rows, rows+1..rows+columns];
   needs __WINAPI call model since it may be called from BFPs */
int __WINAPI obtain_column(lprec *lp, int varin, REAL *pcol, int *nzlist, int *maxabs)
{
  REAL value = my_chsign(lp->is_lower[varin], -1);
  if(varin > lp->rows) {
    varin -= lp->rows;
    varin = expand_column(lp, varin, pcol, nzlist, value, maxabs);
  }
  else if(lp->obj_in_basis || (varin > 0))
    varin = singleton_column(lp, varin, pcol, nzlist, value, maxabs);
  else
    varin = get_basisOF(lp, NULL, pcol, nzlist);

  return(varin);
}

/* GENERAL INVARIANT CALLBACK FUNCTIONS */
MYBOOL set_callbacks(lprec *lp)
{
  /* Assign API functions to lp structure (mainly for XLIs) */
  lp->add_column              = add_column;
  lp->add_columnex            = add_columnex;
  lp->add_constraint          = add_constraint;
  lp->add_constraintex        = add_constraintex;
  lp->add_lag_con             = add_lag_con;
  lp->add_SOS                 = add_SOS;
  lp->column_in_lp            = column_in_lp;
  lp->copy_lp                 = copy_lp;
  lp->default_basis           = default_basis;
  lp->del_column              = del_column;
  lp->del_constraint          = del_constraint;
  lp->delete_lp               = delete_lp;
  lp->dualize_lp              = dualize_lp;
  lp->free_lp                 = free_lp;
  lp->get_anti_degen          = get_anti_degen;
  lp->get_basis               = get_basis;
  lp->get_basiscrash          = get_basiscrash;
  lp->get_bb_depthlimit       = get_bb_depthlimit;
  lp->get_bb_floorfirst       = get_bb_floorfirst;
  lp->get_bb_rule             = get_bb_rule;
  lp->get_bounds_tighter      = get_bounds_tighter;
  lp->get_break_at_value      = get_break_at_value;
  lp->get_col_name            = get_col_name;
  lp->get_columnex            = get_columnex;
  lp->get_constr_type         = get_constr_type;
  lp->get_constr_value        = get_constr_value;
  lp->get_constraints         = get_constraints;
  lp->get_dual_solution       = get_dual_solution;
  lp->get_epsb                = get_epsb;
  lp->get_epsd                = get_epsd;
  lp->get_epsel               = get_epsel;
  lp->get_epsint              = get_epsint;
  lp->get_epsperturb          = get_epsperturb;
  lp->get_epspivot            = get_epspivot;
  lp->get_improve             = get_improve;
  lp->get_infinite            = get_infinite;
  lp->get_lambda              = get_lambda;
  lp->get_lowbo               = get_lowbo;
  lp->get_lp_index            = get_lp_index;
  lp->get_lp_name             = get_lp_name;
  lp->get_Lrows               = get_Lrows;
  lp->get_mat                 = get_mat;
  lp->get_mat_byindex         = get_mat_byindex;
  lp->get_max_level           = get_max_level;
  lp->get_maxpivot            = get_maxpivot;
  lp->get_mip_gap             = get_mip_gap;
  lp->get_multiprice          = get_multiprice;
  lp->get_nameindex           = get_nameindex;
  lp->get_Ncolumns            = get_Ncolumns;
  lp->get_negrange            = get_negrange;
  lp->get_nonzeros            = get_nonzeros;
  lp->get_Norig_columns       = get_Norig_columns;
  lp->get_Norig_rows          = get_Norig_rows;
  lp->get_Nrows               = get_Nrows;
  lp->get_obj_bound           = get_obj_bound;
  lp->get_objective           = get_objective;
  lp->get_orig_index          = get_orig_index;
  lp->get_origcol_name        = get_origcol_name;
  lp->get_origrow_name        = get_origrow_name;
  lp->get_partialprice        = get_partialprice;
  lp->get_pivoting            = get_pivoting;
  lp->get_presolve            = get_presolve;
  lp->get_presolveloops       = get_presolveloops;
  lp->get_primal_solution     = get_primal_solution;
  lp->get_print_sol           = get_print_sol;
  lp->get_pseudocosts         = get_pseudocosts;
  lp->get_ptr_constraints     = get_ptr_constraints;
  lp->get_ptr_dual_solution   = get_ptr_dual_solution;
  lp->get_ptr_lambda          = get_ptr_lambda;
  lp->get_ptr_primal_solution = get_ptr_primal_solution;
  lp->get_ptr_sensitivity_obj = get_ptr_sensitivity_obj;
  lp->get_ptr_sensitivity_objex = get_ptr_sensitivity_objex;
  lp->get_ptr_sensitivity_rhs = get_ptr_sensitivity_rhs;
  lp->get_ptr_variables       = get_ptr_variables;
  lp->get_rh                  = get_rh;
  lp->get_rh_range            = get_rh_range;
  lp->get_row                 = get_row;
  lp->get_rowex               = get_rowex;
  lp->get_row_name            = get_row_name;
  lp->get_scalelimit          = get_scalelimit;
  lp->get_scaling             = get_scaling;
  lp->get_sensitivity_obj     = get_sensitivity_obj;
  lp->get_sensitivity_objex   = get_sensitivity_objex;
  lp->get_sensitivity_rhs     = get_sensitivity_rhs;
  lp->get_simplextype         = get_simplextype;
  lp->get_solutioncount       = get_solutioncount;
  lp->get_solutionlimit       = get_solutionlimit;
  lp->get_status              = get_status;
  lp->get_statustext          = get_statustext;
  lp->get_timeout             = get_timeout;
  lp->get_total_iter          = get_total_iter;
  lp->get_total_nodes         = get_total_nodes;
  lp->get_upbo                = get_upbo;
  lp->get_var_branch          = get_var_branch;
  lp->get_var_dualresult      = get_var_dualresult;
  lp->get_var_primalresult    = get_var_primalresult;
  lp->get_var_priority        = get_var_priority;
  lp->get_variables           = get_variables;
  lp->get_verbose             = get_verbose;
  lp->get_working_objective   = get_working_objective;
  lp->has_BFP                 = has_BFP;
  lp->has_XLI                 = has_XLI;
  lp->is_add_rowmode          = is_add_rowmode;
  lp->is_anti_degen           = is_anti_degen;
  lp->is_binary               = is_binary;
  lp->is_break_at_first       = is_break_at_first;
  lp->is_constr_type          = is_constr_type;
  lp->is_debug                = is_debug;
  lp->is_feasible             = is_feasible;
  lp->is_unbounded            = is_unbounded;
  lp->is_infinite             = is_infinite;
  lp->is_int                  = is_int;
  lp->is_integerscaling       = is_integerscaling;
  lp->is_lag_trace            = is_lag_trace;
  lp->is_maxim                = is_maxim;
  lp->is_nativeBFP            = is_nativeBFP;
  lp->is_nativeXLI            = is_nativeXLI;
  lp->is_negative             = is_negative;
  lp->is_obj_in_basis         = is_obj_in_basis;
  lp->is_piv_mode             = is_piv_mode;
  lp->is_piv_rule             = is_piv_rule;
  lp->is_presolve             = is_presolve;
  lp->is_scalemode            = is_scalemode;
  lp->is_scaletype            = is_scaletype;
  lp->is_semicont             = is_semicont;
  lp->is_SOS_var              = is_SOS_var;
  lp->is_trace                = is_trace;
  lp->lp_solve_version        = lp_solve_version;
  lp->make_lp                 = make_lp;
  lp->print_constraints       = print_constraints;
  lp->print_debugdump         = print_debugdump;
  lp->print_duals             = print_duals;
  lp->print_lp                = print_lp;
  lp->print_objective         = print_objective;
  lp->print_scales            = print_scales;
  lp->print_solution          = print_solution;
  lp->print_str               = print_str;
  lp->print_tableau           = print_tableau;
  lp->put_abortfunc           = put_abortfunc;
  lp->put_bb_nodefunc         = put_bb_nodefunc;
  lp->put_bb_branchfunc       = put_bb_branchfunc;
  lp->put_logfunc             = put_logfunc;
  lp->put_msgfunc             = put_msgfunc;
  lp->read_LPhandle           = LP_readhandle;
  lp->read_MPShandle          = MPS_readhandle;
  lp->read_XLI                = read_XLI;
  lp->read_basis              = read_basis;
  lp->reset_basis             = reset_basis;
  lp->read_params             = read_params;
  lp->reset_params            = reset_params;
  lp->resize_lp               = resize_lp;
  lp->set_action              = set_action;
  lp->set_add_rowmode         = set_add_rowmode;
  lp->set_anti_degen          = set_anti_degen;
  lp->set_basisvar            = set_basisvar;
  lp->set_basis               = set_basis;
  lp->set_basiscrash          = set_basiscrash;
  lp->set_bb_depthlimit       = set_bb_depthlimit;
  lp->set_bb_floorfirst       = set_bb_floorfirst;
  lp->set_bb_rule             = set_bb_rule;
  lp->set_BFP                 = set_BFP;
  lp->set_binary              = set_binary;
  lp->set_bounds              = set_bounds;
  lp->set_bounds_tighter      = set_bounds_tighter;
  lp->set_break_at_first      = set_break_at_first;
  lp->set_break_at_value      = set_break_at_value;
  lp->set_col_name            = set_col_name;
  lp->set_constr_type         = set_constr_type;
  lp->set_debug               = set_debug;
  lp->set_epsb                = set_epsb;
  lp->set_epsd                = set_epsd;
  lp->set_epsel               = set_epsel;
  lp->set_epsint              = set_epsint;
  lp->set_epslevel            = set_epslevel;
  lp->set_epsperturb          = set_epsperturb;
  lp->set_epspivot            = set_epspivot;
  lp->set_unbounded           = set_unbounded;
  lp->set_improve             = set_improve;
  lp->set_infinite            = set_infinite;
  lp->set_int                 = set_int;
  lp->set_lag_trace           = set_lag_trace;
  lp->set_lowbo               = set_lowbo;
  lp->set_lp_name             = set_lp_name;
  lp->set_mat                 = set_mat;
  lp->set_maxim               = set_maxim;
  lp->set_maxpivot            = set_maxpivot;
  lp->set_minim               = set_minim;
  lp->set_mip_gap             = set_mip_gap;
  lp->set_multiprice          = set_multiprice;
  lp->set_negrange            = set_negrange;
  lp->set_obj                 = set_obj;
  lp->set_obj_bound           = set_obj_bound;
  lp->set_obj_fn              = set_obj_fn;
  lp->set_obj_fnex            = set_obj_fnex;
  lp->set_obj_in_basis        = set_obj_in_basis;
  lp->set_outputfile          = set_outputfile;
  lp->set_outputstream        = set_outputstream;
  lp->set_partialprice        = set_partialprice;
  lp->set_pivoting            = set_pivoting;
  lp->set_preferdual          = set_preferdual;
  lp->set_presolve            = set_presolve;
  lp->set_print_sol           = set_print_sol;
  lp->set_pseudocosts         = set_pseudocosts;
  lp->set_rh                  = set_rh;
  lp->set_rh_range            = set_rh_range;
  lp->set_rh_vec              = set_rh_vec;
  lp->set_row                 = set_row;
  lp->set_rowex               = set_rowex;
  lp->set_row_name            = set_row_name;
  lp->set_scalelimit          = set_scalelimit;
  lp->set_scaling             = set_scaling;
  lp->set_semicont            = set_semicont;
  lp->set_sense               = set_sense;
  lp->set_simplextype         = set_simplextype;
  lp->set_solutionlimit       = set_solutionlimit;
  lp->set_timeout             = set_timeout;
  lp->set_trace               = set_trace;
  lp->set_upbo                = set_upbo;
  lp->set_var_branch          = set_var_branch;
  lp->set_var_weights         = set_var_weights;
  lp->set_verbose             = set_verbose;
  lp->set_XLI                 = set_XLI;
  lp->solve                   = solve;
  lp->str_add_column          = str_add_column;
  lp->str_add_constraint      = str_add_constraint;
  lp->str_add_lag_con         = str_add_lag_con;
  lp->str_set_obj_fn          = str_set_obj_fn;
  lp->str_set_rh_vec          = str_set_rh_vec;
  lp->time_elapsed            = time_elapsed;
  lp->unscale                 = unscale;
  lp->write_lp                = write_lp;
  lp->write_LP                = write_LP;
  lp->write_mps               = write_mps;
  lp->write_freemps           = write_freemps;
  lp->write_MPS               = write_MPS;
  lp->write_freeMPS           = write_freeMPS;
  lp->write_XLI               = write_XLI;
  lp->write_basis             = write_basis;
  lp->write_params            = write_params;

  /* Utility functions (mainly for BFPs) */
  lp->userabort               = userabort;
  lp->report                  = report;
  lp->explain                 = explain;
  lp->set_basisvar            = set_basisvar;
  lp->get_lpcolumn            = obtain_column;
  lp->get_basiscolumn         = get_basiscolumn;
  lp->get_OF_active           = get_OF_active;
  lp->getMDO                  = getMDO;
  lp->invert                  = invert;
  lp->set_action              = set_action;
  lp->clear_action            = clear_action;
  lp->is_action               = is_action;

  return( TRUE );
}

/* SUPPORT FUNCTION FOR BASIS FACTORIZATION PACKAGES */
MYBOOL __WINAPI has_BFP(lprec *lp)
{
  return( is_nativeBFP(lp)
#if LoadInverseLib == TRUE
       || (MYBOOL) (lp->hBFP != NULL)
#endif
        );
}

MYBOOL __WINAPI is_nativeBFP(lprec *lp)
{
#ifdef ExcludeNativeInverse
  return( FALSE );
#elif LoadInverseLib == TRUE
  return( (MYBOOL) (lp->hBFP == NULL) );
#else
  return( TRUE );
#endif
}

MYBOOL __WINAPI set_BFP(lprec *lp, char *filename)
/* (Re)mapping of basis factorization variant methods is done here */
{
  int result = LIB_LOADED;

  /* Release the BFP and basis if we are active */
  if(lp->invB != NULL)
    bfp_free(lp);

#if LoadInverseLib == TRUE
  if(lp->hBFP != NULL) {
  #ifdef WIN32
    FreeLibrary(lp->hBFP);
  #else
    dlclose(lp->hBFP);
  #endif
    lp->hBFP = NULL;
  }
#endif

  if(filename == NULL) {
    if(!is_nativeBFP(lp))
      return( FALSE );
#ifndef ExcludeNativeInverse
    lp->bfp_name = bfp_name;
    lp->bfp_compatible = bfp_compatible;
    lp->bfp_free = bfp_free;
    lp->bfp_resize = bfp_resize;
    lp->bfp_nonzeros = bfp_nonzeros;
    lp->bfp_memallocated = bfp_memallocated;
    lp->bfp_restart = bfp_restart;
    lp->bfp_mustrefactorize = bfp_mustrefactorize;
    lp->bfp_preparefactorization = bfp_preparefactorization;
    lp->bfp_factorize = bfp_factorize;
    lp->bfp_finishupdate = bfp_finishupdate;
    lp->bfp_ftran_normal = bfp_ftran_normal;
    lp->bfp_ftran_prepare = bfp_ftran_prepare;
    lp->bfp_btran_normal = bfp_btran_normal;
    lp->bfp_status = bfp_status;
    lp->bfp_implicitslack = bfp_implicitslack;
    lp->bfp_indexbase = bfp_indexbase;
    lp->bfp_rowoffset = bfp_rowoffset;
    lp->bfp_pivotmax = bfp_pivotmax;
    lp->bfp_init = bfp_init;
    lp->bfp_pivotalloc = bfp_pivotalloc;
    lp->bfp_colcount = bfp_colcount;
    lp->bfp_canresetbasis = bfp_canresetbasis;
    lp->bfp_finishfactorization = bfp_finishfactorization;
    lp->bfp_updaterefactstats = bfp_updaterefactstats;
    lp->bfp_prepareupdate = bfp_prepareupdate;
    lp->bfp_pivotRHS = bfp_pivotRHS;
    lp->bfp_btran_double = bfp_btran_double;
    lp->bfp_efficiency = bfp_efficiency;
    lp->bfp_pivotvector = bfp_pivotvector;
    lp->bfp_pivotcount = bfp_pivotcount;
    lp->bfp_refactcount = bfp_refactcount;
    lp->bfp_isSetI = bfp_isSetI;
    lp->bfp_findredundant = bfp_findredundant;
#endif
  }
  else {
#if LoadInverseLib == TRUE
  #ifdef WIN32
   /* Get a handle to the Windows DLL module. */
    lp->hBFP = LoadLibrary(filename);

   /* If the handle is valid, try to get the function addresses. */
    if(lp->hBFP != NULL) {
      lp->bfp_compatible           = (BFPbool_lpintintint *)
                                      GetProcAddress(lp->hBFP, "bfp_compatible");
      if(lp->bfp_compatible == NULL)
        result = LIB_NOINFO;
      else if(lp->bfp_compatible(lp, BFPVERSION, MAJORVERSION, sizeof(REAL))) {

      lp->bfp_name                 = (BFPchar *)
                                      GetProcAddress(lp->hBFP, "bfp_name");
      lp->bfp_free                 = (BFP_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_free");
      lp->bfp_resize               = (BFPbool_lpint *)
                                      GetProcAddress(lp->hBFP, "bfp_resize");
      lp->bfp_nonzeros             = (BFPint_lpbool *)
                                      GetProcAddress(lp->hBFP, "bfp_nonzeros");
      lp->bfp_memallocated         = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_memallocated");
      lp->bfp_restart              = (BFPbool_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_restart");
      lp->bfp_mustrefactorize      = (BFPbool_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_mustrefactorize");
      lp->bfp_preparefactorization = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_preparefactorization");
      lp->bfp_factorize            = (BFPint_lpintintboolbool *)
                                      GetProcAddress(lp->hBFP, "bfp_factorize");
      lp->bfp_finishupdate         = (BFPbool_lpbool *)
                                      GetProcAddress(lp->hBFP, "bfp_finishupdate");
      lp->bfp_ftran_normal         = (BFP_lprealint *)
                                      GetProcAddress(lp->hBFP, "bfp_ftran_normal");
      lp->bfp_ftran_prepare        = (BFP_lprealint *)
                                      GetProcAddress(lp->hBFP, "bfp_ftran_prepare");
      lp->bfp_btran_normal         = (BFP_lprealint *)
                                      GetProcAddress(lp->hBFP, "bfp_btran_normal");
      lp->bfp_status               = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_status");
      lp->bfp_implicitslack        = (BFPbool_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_implicitslack");
      lp->bfp_indexbase            = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_indexbase");
      lp->bfp_rowoffset            = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_rowoffset");
      lp->bfp_pivotmax             = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_pivotmax");
      lp->bfp_init                 = (BFPbool_lpintintchar *)
                                      GetProcAddress(lp->hBFP, "bfp_init");
      lp->bfp_pivotalloc           = (BFPbool_lpint *)
                                      GetProcAddress(lp->hBFP, "bfp_pivotalloc");
      lp->bfp_colcount             = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_colcount");
      lp->bfp_canresetbasis        = (BFPbool_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_canresetbasis");
      lp->bfp_finishfactorization  = (BFP_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_finishfactorization");
      lp->bfp_updaterefactstats    = (BFP_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_updaterefactstats");
      lp->bfp_prepareupdate        = (BFPlreal_lpintintreal *)
                                      GetProcAddress(lp->hBFP, "bfp_prepareupdate");
      lp->bfp_pivotRHS             = (BFPreal_lplrealreal *)
                                      GetProcAddress(lp->hBFP, "bfp_pivotRHS");
      lp->bfp_btran_double         = (BFP_lprealintrealint *)
                                      GetProcAddress(lp->hBFP, "bfp_btran_double");
      lp->bfp_efficiency           = (BFPreal_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_efficiency");
      lp->bfp_pivotvector          = (BFPrealp_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_pivotvector");
      lp->bfp_pivotcount           = (BFPint_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_pivotcount");
      lp->bfp_refactcount          = (BFPint_lpint *)
                                      GetProcAddress(lp->hBFP, "bfp_refactcount");
      lp->bfp_isSetI               = (BFPbool_lp *)
                                      GetProcAddress(lp->hBFP, "bfp_isSetI");
      lp->bfp_findredundant        = (BFPint_lpintrealcbintint *)
                                      GetProcAddress(lp->hBFP, "bfp_findredundant");
      }
      else
        result = LIB_VERINVALID;
    }
  #else
   /* First standardize UNIX .SO library name format. */
    char bfpname[260], *ptr;

    strcpy(bfpname, filename);
    if((ptr = strrchr(filename, '/')) == NULL)
      ptr = filename;
    else
      ptr++;
    bfpname[(int) (ptr - filename)] = 0;
    if(strncmp(ptr, "lib", 3))
      strcat(bfpname, "lib");
    strcat(bfpname, ptr);
    if(strcmp(bfpname + strlen(bfpname) - 3, ".so"))
      strcat(bfpname, ".so");

   /* Get a handle to the module. */
    lp->hBFP = dlopen(bfpname, RTLD_LAZY);

   /* If the handle is valid, try to get the function addresses. */
    if(lp->hBFP != NULL) {
      lp->bfp_compatible           = (BFPbool_lpintintint *)
                                      dlsym(lp->hBFP, "bfp_compatible");
      if(lp->bfp_compatible == NULL)
        result = LIB_NOINFO;
      else if(lp->bfp_compatible(lp, BFPVERSION, MAJORVERSION, sizeof(REAL))) {

      lp->bfp_name                 = (BFPchar *)
                                      dlsym(lp->hBFP, "bfp_name");
      lp->bfp_free                 = (BFP_lp *)
                                      dlsym(lp->hBFP, "bfp_free");
      lp->bfp_resize               = (BFPbool_lpint *)
                                      dlsym(lp->hBFP, "bfp_resize");
      lp->bfp_nonzeros             = (BFPint_lpbool *)
                                      dlsym(lp->hBFP, "bfp_nonzeros");
      lp->bfp_memallocated         = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_memallocated");
      lp->bfp_restart              = (BFPbool_lp *)
                                      dlsym(lp->hBFP, "bfp_restart");
      lp->bfp_mustrefactorize      = (BFPbool_lp *)
                                      dlsym(lp->hBFP, "bfp_mustrefactorize");
      lp->bfp_preparefactorization = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_preparefactorization");
      lp->bfp_factorize            = (BFPint_lpintintboolbool *)
                                      dlsym(lp->hBFP, "bfp_factorize");
      lp->bfp_finishupdate         = (BFPbool_lpbool *)
                                      dlsym(lp->hBFP, "bfp_finishupdate");
      lp->bfp_ftran_normal         = (BFP_lprealint *)
                                      dlsym(lp->hBFP, "bfp_ftran_normal");
      lp->bfp_ftran_prepare        = (BFP_lprealint *)
                                      dlsym(lp->hBFP, "bfp_ftran_prepare");
      lp->bfp_btran_normal         = (BFP_lprealint *)
                                      dlsym(lp->hBFP, "bfp_btran_normal");
      lp->bfp_status               = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_status");
      lp->bfp_implicitslack        = (BFPbool_lp *)
                                      dlsym(lp->hBFP, "bfp_implicitslack");
      lp->bfp_indexbase            = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_indexbase");
      lp->bfp_rowoffset            = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_rowoffset");
      lp->bfp_pivotmax             = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_pivotmax");
      lp->bfp_init                 = (BFPbool_lpintintchar *)
                                      dlsym(lp->hBFP, "bfp_init");
      lp->bfp_pivotalloc           = (BFPbool_lpint *)
                                      dlsym(lp->hBFP, "bfp_pivotalloc");
      lp->bfp_colcount             = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_colcount");
      lp->bfp_canresetbasis        = (BFPbool_lp *)
                                      dlsym(lp->hBFP, "bfp_canresetbasis");
      lp->bfp_finishfactorization  = (BFP_lp *)
                                      dlsym(lp->hBFP, "bfp_finishfactorization");
      lp->bfp_updaterefactstats    = (BFP_lp *)
                                      dlsym(lp->hBFP, "bfp_updaterefactstats");
      lp->bfp_prepareupdate        = (BFPlreal_lpintintreal *)
                                      dlsym(lp->hBFP, "bfp_prepareupdate");
      lp->bfp_pivotRHS             = (BFPreal_lplrealreal *)
                                      dlsym(lp->hBFP, "bfp_pivotRHS");
      lp->bfp_btran_double         = (BFP_lprealintrealint *)
                                      dlsym(lp->hBFP, "bfp_btran_double");
      lp->bfp_efficiency           = (BFPreal_lp *)
                                      dlsym(lp->hBFP, "bfp_efficiency");
      lp->bfp_pivotvector          = (BFPrealp_lp *)
                                      dlsym(lp->hBFP, "bfp_pivotvector");
      lp->bfp_pivotcount           = (BFPint_lp *)
                                      dlsym(lp->hBFP, "bfp_pivotcount");
      lp->bfp_refactcount          = (BFPint_lpint *)
                                      dlsym(lp->hBFP, "bfp_refactcount");
      lp->bfp_isSetI               = (BFPbool_lp *)
                                      dlsym(lp->hBFP, "bfp_isSetI");
      lp->bfp_findredundant        = (BFPint_lpintrealcbintint *)
                                      dlsym(lp->hBFP, "bfp_findredundant");
      }
      else
        result = LIB_VERINVALID;
    }
  #endif
    else
      result = LIB_NOTFOUND;
#endif
    /* Do validation */
    if((result != LIB_LOADED) ||
       ((lp->bfp_name == NULL) ||
        (lp->bfp_compatible == NULL) ||
        (lp->bfp_free == NULL) ||
        (lp->bfp_resize == NULL) ||
        (lp->bfp_nonzeros == NULL) ||
        (lp->bfp_memallocated == NULL) ||
        (lp->bfp_restart == NULL) ||
        (lp->bfp_mustrefactorize == NULL) ||
        (lp->bfp_preparefactorization == NULL) ||
        (lp->bfp_factorize == NULL) ||
        (lp->bfp_finishupdate == NULL) ||
        (lp->bfp_ftran_normal == NULL) ||
        (lp->bfp_ftran_prepare == NULL) ||
        (lp->bfp_btran_normal == NULL) ||
        (lp->bfp_status == NULL) ||
        (lp->bfp_implicitslack == NULL) ||
        (lp->bfp_indexbase == NULL) ||
        (lp->bfp_rowoffset == NULL) ||
        (lp->bfp_pivotmax == NULL) ||
        (lp->bfp_init == NULL) ||
        (lp->bfp_pivotalloc == NULL) ||
        (lp->bfp_colcount == NULL) ||
        (lp->bfp_canresetbasis == NULL) ||
        (lp->bfp_finishfactorization == NULL) ||
        (lp->bfp_updaterefactstats == NULL) ||
        (lp->bfp_prepareupdate == NULL) ||
        (lp->bfp_pivotRHS == NULL) ||
        (lp->bfp_btran_double == NULL) ||
        (lp->bfp_efficiency == NULL) ||
        (lp->bfp_pivotvector == NULL) ||
        (lp->bfp_pivotcount == NULL) ||
        (lp->bfp_refactcount == NULL) ||
        (lp->bfp_isSetI == NULL) ||
        (lp->bfp_findredundant == NULL)
       )) {
      set_BFP(lp, NULL);
      if(result == LIB_LOADED)
        result = LIB_NOFUNCTION;
    }
  }
  if(filename != NULL) {
    char info[LIB_STR_MAXLEN+1];
    switch(result) {
      case LIB_NOTFOUND:   strcpy(info, LIB_STR_NOTFOUND);
                           break;
      case LIB_NOINFO:     strcpy(info, LIB_STR_NOINFO);
                           break;
      case LIB_NOFUNCTION: strcpy(info, LIB_STR_NOFUNCTION);
                           break;
      case LIB_VERINVALID: strcpy(info, LIB_STR_VERINVALID);
                           break;
      default:             strcpy(info, LIB_STR_LOADED);
    }
    report(lp, IMPORTANT, "set_BFP: %s '%s'\n",
                          info, filename);
  }
  return( (MYBOOL) (result == LIB_LOADED));
}


/* External language interface routines */
/* DON'T MODIFY */
lprec * __WINAPI read_XLI(char *xliname, char *modelname, char *dataname, char *options, int verbose)
{
  lprec *lp;

  lp = make_lp(0, 0);
  if(lp != NULL) {
    lp->source_is_file = TRUE;
    lp->verbose = verbose;
    if(!set_XLI(lp, xliname)) {
      free_lp(&lp);
      printf("read_XLI: No valid XLI package selected or available.\n");
    }
    else {
      if(!lp->xli_readmodel(lp, modelname, (dataname != NULL) && (*dataname != 0) ? dataname : NULL, options, verbose))
        free_lp(&lp);
    }
  }
  return( lp );
}

MYBOOL __WINAPI write_XLI(lprec *lp, char *filename, char *options, MYBOOL results)
{
  return( has_XLI(lp) && mat_validate(lp->matA) && lp->xli_writemodel(lp, filename, options, results) );
}

MYBOOL __WINAPI has_XLI(lprec *lp)
{
  return( is_nativeXLI(lp)
#if LoadLanguageLib == TRUE
       || (MYBOOL) (lp->hXLI != NULL)
#endif
        );
}

MYBOOL __WINAPI is_nativeXLI(lprec *lp)
{
#ifdef ExcludeNativeLanguage
  return( FALSE );
#elif LoadLanguageLib == TRUE
  return( (MYBOOL) (lp->hXLI == NULL) );
#else
  return( TRUE );
#endif
}

MYBOOL __WINAPI set_XLI(lprec *lp, char *filename)
/* (Re)mapping of external language interface variant methods is done here */
{
  int result = LIB_LOADED;

#if LoadLanguageLib == TRUE
  if(lp->hXLI != NULL) {
  #ifdef WIN32
    FreeLibrary(lp->hXLI);
  #else
    dlclose(lp->hXLI);
  #endif
    lp->hXLI = NULL;
  }
#endif

  if(filename == NULL) {
    if(!is_nativeXLI(lp))
      return( FALSE );
#ifndef ExcludeNativeLanguage
    lp->xli_name = xli_name;
    lp->xli_compatible = xli_compatible;
    lp->xli_readmodel = xli_readmodel;
    lp->xli_writemodel = xli_writemodel;
#endif
  }
  else {
#if LoadLanguageLib == TRUE
  #ifdef WIN32
   /* Get a handle to the Windows DLL module. */
    lp->hXLI = LoadLibrary(filename);

   /* If the handle is valid, try to get the function addresses. */
    if(lp->hXLI != NULL) {
      lp->xli_compatible           = (XLIbool_lpintintint *)
                                      GetProcAddress(lp->hXLI, "xli_compatible");
      if(lp->xli_compatible == NULL)
        result = LIB_NOINFO;
      else if(lp->xli_compatible(lp, XLIVERSION, MAJORVERSION, sizeof(REAL))) {

        lp->xli_name                 = (XLIchar *)
                                        GetProcAddress(lp->hXLI, "xli_name");
        lp->xli_readmodel            = (XLIbool_lpcharcharcharint *)
                                        GetProcAddress(lp->hXLI, "xli_readmodel");
        lp->xli_writemodel           = (XLIbool_lpcharcharbool *)
                                        GetProcAddress(lp->hXLI, "xli_writemodel");
      }
      else
        result = LIB_VERINVALID;
    }
  #else
   /* First standardize UNIX .SO library name format. */
    char xliname[260], *ptr;

    strcpy(xliname, filename);
    if((ptr = strrchr(filename, '/')) == NULL)
      ptr = filename;
    else
      ptr++;
    xliname[(int) (ptr - filename)] = 0;
    if(strncmp(ptr, "lib", 3))
      strcat(xliname, "lib");
    strcat(xliname, ptr);
    if(strcmp(xliname + strlen(xliname) - 3, ".so"))
      strcat(xliname, ".so");

   /* Get a handle to the module. */
    lp->hXLI = dlopen(xliname, RTLD_LAZY);

   /* If the handle is valid, try to get the function addresses. */
    if(lp->hXLI != NULL) {
      lp->xli_compatible           = (XLIbool_lpintintint *)
                                      dlsym(lp->hXLI, "xli_compatible");
      if(lp->xli_compatible == NULL)
        result = LIB_NOINFO;
      else if(lp->xli_compatible(lp, XLIVERSION, MAJORVERSION, sizeof(REAL))) {

        lp->xli_name                 = (XLIchar *)
                                        dlsym(lp->hXLI, "xli_name");
        lp->xli_readmodel            = (XLIbool_lpcharcharcharint *)
                                        dlsym(lp->hXLI, "xli_readmodel");
        lp->xli_writemodel           = (XLIbool_lpcharcharbool *)
                                        dlsym(lp->hXLI, "xli_writemodel");
      }
      else
        result = LIB_VERINVALID;
    }
  #endif
    else
      result = LIB_NOTFOUND;
#endif
    /* Do validation */
    if((result != LIB_LOADED) ||
       ((lp->xli_name == NULL) ||
        (lp->xli_compatible == NULL) ||
        (lp->xli_readmodel == NULL) ||
        (lp->xli_writemodel == NULL)
       )) {
      set_XLI(lp, NULL);
      if(result == LIB_LOADED)
        result = LIB_NOFUNCTION;
    }
  }
  if(filename != NULL) {
    char info[LIB_STR_MAXLEN+1];
    switch(result) {
      case LIB_NOTFOUND:   strcpy(info, LIB_STR_NOTFOUND);
                           break;
      case LIB_NOINFO:     strcpy(info, LIB_STR_NOINFO);
                           break;
      case LIB_NOFUNCTION: strcpy(info, LIB_STR_NOFUNCTION);
                           break;
      case LIB_VERINVALID: strcpy(info, LIB_STR_VERINVALID);
                           break;
      default:             strcpy(info, LIB_STR_LOADED);
    }
    report(lp, IMPORTANT, "set_XLI: %s '%s'\n",
                          info, filename);
  }
  return( (MYBOOL) (result == LIB_LOADED));
}


STATIC int get_basisOF(lprec *lp, int coltarget[], REAL crow[], int colno[])
/* Fill vector of basic OF values or subtract incoming values from these.
   This function is called twice during reduced cost updates when the basis
   does not contain the basic OF vector as the top row.  The colno[] array
   is filled with the count of non-zero values and the index to those. */
{
  int            i, n = lp->rows, nz = 0;
  REAL           *obj = lp->obj;
  register REAL epsvalue = lp->epsvalue;

  /* Compute offset over the specified objective indeces (step 2) */
  if(coltarget != NULL) {
    register int  ix, m = coltarget[0];
    register REAL value;

    for(i = 1, coltarget++; i <= m; i++, coltarget++) {
      ix = *coltarget;
      /* Finalize the computation of the reduced costs, based on the format that
         duals are computed as negatives, ref description for step 1 above */
      value = crow[ix];
      if(ix > n)
        value += obj[ix - n];
/*      if(value != 0) { */
      if(fabs(value) > epsvalue) {
        nz++;
        if(colno != NULL)
          colno[nz] = ix;
      }
	  else
	    value = 0.0;
      crow[ix] = value;
    }
  }

  /* Get the basic objective function values (step 1) */
  else {
    register int *basvar = lp->var_basic;

    for(i = 1, crow++, basvar++; i <= n;
         i++, crow++, basvar++) {
      /* Load the objective value of the active basic variable; note that we
         change the sign of the value to maintain computational compatibility with
         the calculation of duals using in-basis storage of the basic OF values */
      if(*basvar <= n)
        *crow = 0;
      else
        *crow = -obj[(*basvar) - n];
      if((*crow) != 0) {
/*      if(fabs(*crow) > epsvalue) { */
        nz++;
        if(colno != NULL)
          colno[nz] = i;
      }
    }
  }
  if(colno != NULL)
    colno[0] = nz;
  return( nz );
}

int __WINAPI get_basiscolumn(lprec *lp, int j, int rn[], double bj[])
/* This routine returns sparse vectors for all basis
   columns, including the OF dummy (index 0) and slack columns.
   NOTE that the index usage is nonstandard for lp_solve, since
   the array offset is 1, not 0. */
{
  int k = lp->bfp_rowoffset(lp),
      matbase = lp->bfp_indexbase(lp);

  /* Do target index adjustment (etaPFI with matbase==0 is special case) */
  if(matbase > 0)
    matbase += k - 1;

 /* Convert index of slack and user columns */
  j -= k;
  if((j > 0) && !lp->bfp_isSetI(lp))
    j = lp->var_basic[j];

 /* Process OF dummy and slack columns (always at lower bound) */
  if(j <= lp->rows) {
    rn[1] = j + matbase;
    bj[1] = 1.0;
    k = 1;
  }
 /* Process user columns (negated if at lower bound) */
  else {
    k = obtain_column(lp, j, bj, rn, NULL);
    if(matbase != 0)
      for(j = 1; j <= k; j++)
        rn[j] += matbase;
  }

  return( k );
}

MYBOOL __WINAPI get_primal_solution(lprec *lp, REAL *pv)
{
  if(lp->spx_status == OPTIMAL)
    ;
  else if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_primal_solution: Not a valid basis");
    return(FALSE);
  }

  MEMCOPY(pv, lp->best_solution, lp->sum + 1);
  return(TRUE);
}

MYBOOL __WINAPI get_ptr_primal_solution(lprec *lp, REAL **pv)
{
  *pv = lp->best_solution;
  return(TRUE);
}

MYBOOL __WINAPI get_dual_solution(lprec *lp, REAL *rc)
{
  REAL *duals;
  MYBOOL ret;

  if(!lp->basis_valid) {
    report(lp, CRITICAL, "get_dual_solution: Not a valid basis");
    return(FALSE);
  }

  ret = get_ptr_sensitivity_rhs(lp, &duals, NULL, NULL);

  if(ret)
    MEMCOPY(rc, duals - 1, lp->sum + 1);
  return(ret);
}

MYBOOL __WINAPI get_ptr_dual_solution(lprec *lp, REAL **rc)
{
  MYBOOL ret = lp->basis_valid;

  /* Just return availability of dual information if rc is NULL */
  if(rc == NULL)
    return( ret && ((MIP_count(lp) == 0) || (lp->bb_totalnodes > 0)) );

  if(!ret) {
    report(lp, CRITICAL, "get_ptr_dual_solution: Not a valid basis");
    return(ret);
  }

  /* Otherwise, get the pointer to the dual information (and optionally produce it) */
  ret = get_ptr_sensitivity_rhs(lp, rc, NULL, NULL);
  if(ret)
    (*rc)--;

  return(ret);
}

MYBOOL __WINAPI get_lambda(lprec *lp, REAL *lambda)
{
  if(!lp->basis_valid || (get_Lrows(lp) == 0)) {
    report(lp, CRITICAL, "get_lambda: Not a valid basis");
    return(FALSE);
  }

  MEMCOPY(lambda, lp->lambda+1, get_Lrows(lp));
  return(TRUE);
}

MYBOOL __WINAPI get_ptr_lambda(lprec *lp, REAL **lambda)
{
  *lambda = lp->lambda;
  return(TRUE);
}

int __WINAPI get_orig_index(lprec *lp, int lp_index)
{
  if(lp->varmap_locked)
    return(lp->presolve_undo->var_to_orig[lp_index]);
  else if(lp_index <= lp->presolve_undo->orig_rows)
    return(lp_index);
  else
    return(lp_index-lp->presolve_undo->orig_rows);
}
int __WINAPI get_lp_index(lprec *lp, int orig_index)
{
  if(lp->varmap_locked)
    return(lp->presolve_undo->orig_to_var[orig_index]);
  else if(orig_index <= lp->presolve_undo->orig_rows)
    return(orig_index);
  else
    return(orig_index-lp->presolve_undo->orig_rows);
}

MYBOOL __WINAPI is_feasible(lprec *lp, REAL *values, REAL threshold)
/* Recommend to use threshold = lp->epspivot */
{
  int     i, j, elmnr, ie;
  REAL    *this_rhs, dist;
  REAL    *value;
  int     *rownr;
  MATrec  *mat = lp->matA;

  for(i = lp->rows + 1; i <= lp->sum; i++) {
    if(values[i - lp->rows] < unscaled_value(lp, lp->orig_lowbo[i], i)
       || values[i - lp->rows] > unscaled_value(lp, lp->orig_upbo[i], i)) {
      if(!((lp->sc_lobound[i - lp->rows]>0) && (values[i - lp->rows]==0)))
        return(FALSE);
    }
  }

  this_rhs = (REAL *) mempool_obtainVector(lp->workarrays, lp->rows+1, sizeof(*this_rhs));
/*  allocREAL(lp, &this_rhs, lp->rows + 1, TRUE); */
  for(j = 1; j <= lp->columns; j++) {
    elmnr = mat->col_end[j - 1];
    ie = mat->col_end[j];
    rownr = &COL_MAT_ROWNR(elmnr);
    value = &COL_MAT_VALUE(elmnr);
    for(; elmnr < ie; elmnr++, rownr += matRowColStep, value += matValueStep) {
      this_rhs[*rownr] += unscaled_mat(lp, *value, *rownr, j);
    }
  }
  for(i = 1; i <= lp->rows; i++) {
    dist = lp->orig_rhs[i] - this_rhs[i];
    my_roundzero(dist, threshold);
    if((lp->orig_upbo[i] == 0 && dist != 0) ||( dist < 0)) {
      FREE(this_rhs);
      return(FALSE);
    }
  }
  mempool_releaseVector(lp->workarrays, (char *) this_rhs, FALSE);
/*  FREE(this_rhs); */
  return(TRUE);
}

int __WINAPI column_in_lp(lprec *lp, REAL *testcolumn)
{
  int    i, j, je, colnr = 0;
  int    nz, ident = 1;
  MATrec *mat = lp->matA;
  int    *matRownr;
  REAL   value, *matValue;

  for(nz = 0, i = 1; i <= lp->rows; i++)
    if(fabs(testcolumn[i]) > lp->epsvalue) nz++;

  for(i = 1; (i <= lp->columns) && (ident); i++) {
    ident = nz;
    value = fabs(get_mat(lp, 0, i)-testcolumn[0]);
    if(value > lp->epsvalue)
      continue;
    j = mat->col_end[i - 1];
    je = mat->col_end[i];
    matRownr = &COL_MAT_ROWNR(j);
    matValue = &COL_MAT_VALUE(j);
    for(; (j < je) && (ident >= 0);
        j++, ident--, matRownr += matRowColStep, matValue += matValueStep) {
      value = *matValue;
      if(is_chsign(lp, *matRownr))
        value = my_flipsign(value);
      value = unscaled_mat(lp, value, *matRownr, i);
      value -= testcolumn[*matRownr];
      if(fabs(value) > lp->epsvalue)
        break;
    }
    if(ident == 0)
      colnr = i;
  }
  return( colnr );
}

MYBOOL __WINAPI set_lp_name(lprec *lp, char *name)
{
  if (name == NULL) {
    FREE(lp->lp_name);
    lp->lp_name = NULL;
  }
  else {
    allocCHAR(lp, &lp->lp_name, (int) (strlen(name) + 1), AUTOMATIC);
    strcpy(lp->lp_name, name);
  }
  return(TRUE);
}

char * __WINAPI get_lp_name(lprec *lp)
{
  return((lp->lp_name != NULL) ? lp->lp_name : (char *) "");
}

STATIC MYBOOL init_rowcol_names(lprec *lp)
{
  if(!lp->names_used) {
    lp->row_name = (hashelem **) calloc(lp->rows_alloc + 1, sizeof(*lp->row_name));
    lp->col_name = (hashelem **) calloc(lp->columns_alloc + 1, sizeof(*lp->col_name));
    lp->rowname_hashtab = create_hash_table(lp->rows_alloc + 1, 0);
    lp->colname_hashtab = create_hash_table(lp->columns_alloc + 1, 1);
    lp->names_used = TRUE;
  }
  return(TRUE);
}

MYBOOL rename_var(lprec *lp, int varindex, char *new_name, hashelem **list, hashtable **ht)
{
  hashelem *hp;
  MYBOOL   newitem;

  hp = list[varindex];
  newitem = (MYBOOL) (hp == NULL);
  if(newitem)
    hp = puthash(new_name, varindex, list, *ht);
  else if((strlen(hp->name) != strlen(new_name)) ||
          (strcmp(hp->name, new_name) != 0)) {
    hashtable *newht, *oldht;

    allocCHAR(lp, &hp->name, (int) (strlen(new_name) + 1), AUTOMATIC);
    strcpy(hp->name, new_name);
    oldht = *ht;
    newht = copy_hash_table(oldht, list, oldht->size);
    *ht = newht;
    free_hash_table(oldht);
  }
  return(newitem);
}

MYBOOL __WINAPI is_use_names(lprec *lp, MYBOOL isrow)
{
  if(isrow)
    return( lp->use_row_names );
  else
    return( lp->use_col_names );
}

void __WINAPI set_use_names(lprec *lp, MYBOOL isrow, MYBOOL use_names)
{
  if(isrow)
    lp->use_row_names = use_names;
  else
    lp->use_col_names = use_names;
}

int __WINAPI get_nameindex(lprec *lp, char *varname, MYBOOL isrow)
{
  if(isrow)
    return( find_row(lp, varname, FALSE) );
  else
    return( find_var(lp, varname, FALSE) );
}

MYBOOL __WINAPI set_row_name(lprec *lp, int rownr, char *new_name)
{
  if((rownr < 0) || (rownr > lp->rows+1)) {
    report(lp, IMPORTANT, "set_row_name: Row %d out of range", rownr);
    return(FALSE);
  }

  /* Prepare for a new row */
  if((rownr > lp->rows) && !append_rows(lp, rownr-lp->rows))
    return( FALSE );
  if(!lp->names_used) {
    if(!init_rowcol_names(lp))
      return(FALSE);
  }
  rename_var(lp, rownr, new_name, lp->row_name, &lp->rowname_hashtab);

  return(TRUE);
}

char * __WINAPI get_row_name(lprec *lp, int rownr)
{
  if((rownr < 0) || (rownr > lp->rows+1)) {
    report(lp, IMPORTANT, "get_row_name: Row %d out of range", rownr);
    return(NULL);
  }

  if((lp->presolve_undo->var_to_orig != NULL) && lp->wasPresolved) {
    if(lp->presolve_undo->var_to_orig[rownr] == 0)
      rownr = -rownr;
    else
      rownr = lp->presolve_undo->var_to_orig[rownr];
  }
  return( get_origrow_name(lp, rownr) );
}

char * __WINAPI get_origrow_name(lprec *lp, int rownr)
{
  MYBOOL newrow;
  static char name[50];
  char   *ptr;

  newrow = (MYBOOL) (rownr < 0);
  rownr = abs(rownr);
#ifdef Paranoia
  if(((lp->presolve_undo->var_to_orig == NULL) && newrow) ||
     (rownr > MAX(lp->rows, lp->presolve_undo->orig_rows))) {
    report(lp, IMPORTANT, "get_origrow_name: Row %d out of range", rownr);
    return(NULL);
  }
#endif

  if(lp->names_used && lp->use_row_names && (lp->row_name[rownr] != NULL) &&
                            (lp->row_name[rownr]->name != NULL)) {
#ifdef Paranoia
    if(lp->row_name[rownr]->index != rownr)
      report(lp, SEVERE, "get_origrow_name: Inconsistent row ordinal %d vs %d\n",
                         rownr, lp->row_name[rownr]->index);
#endif
    ptr = lp->row_name[rownr]->name;
  }
  else {
    if(newrow)
      sprintf(name, ROWNAMEMASK2, rownr);
    else
      sprintf(name, ROWNAMEMASK, rownr);
    ptr = name;
  }
  return(ptr);
}

MYBOOL __WINAPI set_col_name(lprec *lp, int colnr, char *new_name)
{
  if((colnr > lp->columns+1) || (colnr < 1)) {
    report(lp, IMPORTANT, "set_col_name: Column %d out of range", colnr);
  }

  if((colnr > lp->columns) && !append_columns(lp, colnr-lp->columns))
    return(FALSE);

  if(!lp->names_used)
    init_rowcol_names(lp);
  rename_var(lp, colnr, new_name, lp->col_name, &lp->colname_hashtab);

  return(TRUE);
}

char * __WINAPI get_col_name(lprec *lp, int colnr)
{
  if((colnr > lp->columns+1) || (colnr < 1)) {
    report(lp, IMPORTANT, "get_col_name: Column %d out of range", colnr);
    return(NULL);
  }

  if((lp->presolve_undo->var_to_orig != NULL) && lp->wasPresolved) {
    if(lp->presolve_undo->var_to_orig[lp->rows + colnr] == 0)
      colnr = -colnr;
    else
      colnr = lp->presolve_undo->var_to_orig[lp->rows + colnr];
  }
  return( get_origcol_name(lp, colnr) );
}

char * __WINAPI get_origcol_name(lprec *lp, int colnr)
{
  MYBOOL newcol;
  char   *ptr;
  static char name[50];

  newcol = (MYBOOL) (colnr < 0);
  colnr = abs(colnr);
#ifdef Paranoia
  if(((lp->presolve_undo->var_to_orig == NULL) && newcol) ||
     (colnr > MAX(lp->columns, lp->presolve_undo->orig_columns))) {
    report(lp, IMPORTANT, "get_origcol_name: Column %d out of range", colnr);
    return(NULL);
  }
#endif

  if(lp->names_used && lp->use_col_names && (lp->col_name[colnr] != NULL) && (lp->col_name[colnr]->name != NULL)) {
#ifdef Paranoia
    if(lp->col_name[colnr]->index != colnr)
      report(lp, SEVERE, "get_origcol_name: Inconsistent column ordinal %d vs %d\n",
                         colnr, lp->col_name[colnr]->index);
#endif
    ptr = lp->col_name[colnr]->name;
  }
  else {
    if(newcol)
      sprintf((char *) name, COLNAMEMASK2, colnr);
    else
      sprintf((char *) name, COLNAMEMASK, colnr);
    ptr = name;
  }
  return(ptr);
}

STATIC int MIP_count(lprec *lp)
{
  return( lp->int_vars+lp->sc_vars+SOS_count(lp) );
}
STATIC int bin_count(lprec *lp, MYBOOL working)
{
  int i, n = 0;
  if(working) {
    for(i = lp->rows+1; i <= lp->sum; i++)
      if(fabs(unscaled_value(lp, lp->upbo[i], i) - 1) < lp->epsvalue)
        n++;
  }
  else {
    for(i = 1; i <= lp->columns; i++)
      if((fabs(get_upbo(lp, i) - 1) < lp->epsvalue) &&
         (fabs(get_lowbo(lp, i) - 0) < lp->epsvalue))
        n++;
  }
  return( n );
}
STATIC int SOS_count(lprec *lp)
{
  if(lp->SOS == NULL)
    return( 0 );
  else
    return( lp->SOS->sos_count );
}
STATIC int GUB_count(lprec *lp)
{
  if(lp->GUB == NULL)
    return( 0 );
  else
    return( lp->GUB->sos_count );
}

STATIC REAL compute_violation(lprec *lp, int row_nr)
/* Returns the bound violation of a given basic variable; the return
   value is negative if it is below is lower bound, it is positive
   if it is greater than the upper bound, and zero otherwise. */
{
  REAL value, test;

  value  = lp->rhs[row_nr];
  row_nr = lp->var_basic[row_nr];
  test = value - my_lowbound(lp->lowbo[row_nr]);
  my_roundzero(test, lp->epsprimal);
  if(test > 0) {
    test = value - lp->upbo[row_nr];
    my_roundzero(test, lp->epsprimal);
    if(test < 0)
      test = 0;
  }
  return( test );
}

STATIC REAL feasibilityOffset(lprec *lp, MYBOOL isdual)
{
  int    i, j;
  REAL   f, Extra;

  Extra = 0;
  if(isdual) {
   /* This section computes a OF offset to ensure that the dual phase 1 is
      feasible.  It is used to compute a primal feasible base that can be
      passed to the primal simplex in phase 2. */
#if 0

   /* This is the legacy (v3.2-) P1extraVal logic that sets Extra to be the
      smallest negative reduced cost. Note that the reduced costs are the
      values of the dual slacks, which are [0..Inf> for feasibility.
      If we have negative reduced costs for bounded non-basic variables, we
      can simply switch the bound to obtain feasibility and possibly avoid
      having to set Extra. */
    if(!isDualFeasible(lp, lp->epsprimal, NULL, NULL, &f)
      Extra = f;

#else
  /* Find the most negative of the objective coefficients. We will subtract this
     value from every element of the objective row, making it non-negative and
     the problem therefore dual feasible. */
    for(i = 1; i <= lp->columns; i++) {
      f = lp->orig_obj[i];
      if(f < Extra)
        Extra = f;
    }
#endif
  }

  else {
  /* Set Extra to be the index of the most negative of the net RHS coefficients;
     this approach can be used in the primal phase 1 followed by the dual phase 2
     and when there are no ranged constraints.  When there are ranged constraints,
     additional artificial variables must be introduced. */
    Extra = 0;
    j = 0;
    Extra = lp->infinite;
    for(i = 1; i <= lp->rows; i++) {
      f = lp->rhs[i];
      if(f < Extra) {
        Extra = f;
        j = i;
      }
    }
    Extra = j;
  }

  return(Extra);

}

STATIC REAL compute_dualslacks(lprec *lp, int target, REAL **dvalues, int **nzdvalues, MYBOOL dosum)
/* Note that this function is similar to the compute_reducedcosts function in lp_price.c */
{
  int    i, varnr,
         *coltarget, **nzduals, *nzvtemp = NULL;
  REAL   d, g = 0, **duals, *vtemp = NULL;
  MYBOOL localREAL = (MYBOOL) (dvalues == NULL),
         localINT  = (MYBOOL) (nzdvalues == NULL);

  if(is_action(lp->spx_action, ACTION_REBASE) ||
     is_action(lp->spx_action, ACTION_REINVERT) || !lp->basis_valid)
    return( g );

  /* Initialize */
  if(!localREAL) {
    duals = dvalues;
    nzduals = nzdvalues;
  }
  else {
    duals = &vtemp;
    nzduals = &nzvtemp;
  }
  if(localINT || (*nzduals == NULL))
    allocINT(lp, nzduals, lp->columns + 1, AUTOMATIC);
  if(localREAL || (*duals == NULL))
    allocREAL(lp, duals, lp->sum + 1, AUTOMATIC);
  if(target == 0)
    target = SCAN_ALLVARS+ USE_NONBASICVARS;

  /* Define variable target list and compute the reduced costs */
  coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->columns+1, sizeof(*coltarget));
  if(!get_colIndexA(lp, target, coltarget, FALSE)) {
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
    return(FALSE);
  }
  bsolve(lp, 0, *duals, NULL, lp->epsmachine*DOUBLEROUND, 1.0);
  prod_xA(lp, coltarget, *duals, NULL, lp->epsmachine, 1.0,
                         *duals, *nzduals, MAT_ROUNDDEFAULT | MAT_ROUNDRC);
  mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);

  /* Compute sum or maximum infeasibility as specified */
  for(i = 1; i <= (*nzduals)[0]; i++) {
    varnr = (*nzduals)[i];
    d = my_chsign(!lp->is_lower[varnr], (*duals)[varnr]);
    if(d < 0) {
      if(dosum)
        g += -d;         /* Compute sum as a positive number */
      else {
        SETMIN(g, d);    /* Compute gap as a negative number */
      }
    }
  }

  /* Clean up */
  if(localREAL)
    FREE(*duals);
  if(localINT)
    FREE(*nzduals);

  return( g );
}

STATIC REAL compute_feasibilitygap(lprec *lp, MYBOOL isdual, MYBOOL dosum)
{
  REAL f = 0;

  /* This computes the primal feasibility gap (for use with the dual simplex phase 1) */
  if(isdual) {
    int  i;
    REAL g;

    for(i = 1; i <= lp->rows; i++) {
      if(lp->rhs[i] < 0)
        g = lp->rhs[i];
      else if(lp->rhs[i] > lp->upbo[lp->var_basic[i]])
        g = lp->rhs[i] - lp->upbo[lp->var_basic[i]];
      else
        g = 0;
      if(dosum)
        f += g;
      else {
        SETMAX(f, g);
      }
    }
  }
  /* This computes the dual feasibility gap (for use with the primal simplex phase 1) */
  else
    f = compute_dualslacks(lp, SCAN_USERVARS+USE_ALLVARS, NULL, NULL, dosum);

  return( f );
}

/* Find the smallest fractional value in a given row of the OF/constraint matrix */
STATIC int row_decimals(lprec *lp, int rownr, MYBOOL intsonly, REAL *intscalar)
{
  int basi, i, j, ncols = lp->columns;
  REAL f, /* g, */ epsvalue = lp->epsprimal;

  basi = 0;
  for(j = 1; j <= ncols; j++) {
    if(intsonly && !is_int(lp, j)) {
      if(intsonly == TRUE)
        break;
      else
        continue;
    }
    f = fabs(get_mat(lp, rownr, j));
    /* f = fmod(f, 1); */
    f -= floor (f + epsvalue);
/*
    if(f <= epsvalue)
      continue;
    g = f;
*/
    for(i = 0; (i <= MAX_FRACSCALE) && (/* g */ f > epsvalue); i++) {
      f *= 10;
      /* g = fmod(f, 1); */
      f -= floor (f + epsvalue);
    }
    if(i > MAX_FRACSCALE)
      /* i = MAX_FRACSCALE */ break;
    SETMAX(basi, i);
  }
  if(j > ncols)
    *intscalar = pow(10.0, basi);
  else {
    basi = -1;
    *intscalar = 1;
  }
  return( basi );
}

STATIC int row_intstats(lprec *lp, int rownr, int pivcolnr, int *maxndec,
                        int *plucount, int *intcount, int *intval, REAL *valGCD, REAL *pivcolval)
{
  int    jb, je, jj, nn = 0, multA, multB, intGCD = 0;
  REAL   rowval, inthold, intfrac;
  MATrec *mat = lp->matA;

  /* Do we have a valid matrix? */
  if(mat_validate(mat)) {

    /* Get smallest fractional row value */
    *maxndec = row_decimals(lp, rownr, AUTOMATIC, &intfrac);

    /* Get OF row starting and ending positions, as well as the first column index */
    if(rownr == 0) {
      jb = 1;
      je = lp->columns+1;
    }
    else {
      jb = mat->row_end[rownr-1];
      je = mat->row_end[rownr];
    }
    nn = je - jb;
    *pivcolval = 1.0;
    *plucount = 0;
    *intcount = 0;
    *intval   = 0;
    for(; jb < je; jb++) {

      if(rownr == 0) {
        if(lp->orig_obj[jb] == 0) {
          nn--;
          continue;
        }
        jj = jb;
      }
      else
        jj = ROW_MAT_COLNR(jb);

      /* Pick up the value of the pivot column and continue */
      if(jj == pivcolnr) {
        if(rownr == 0)
          *pivcolval = unscaled_mat(lp, lp->orig_obj[jb], 0, jb);
        else
          *pivcolval = get_mat_byindex(lp, jb, TRUE, FALSE);
        continue;
      }
      if(!is_int(lp, jj))
        continue;

      /* Update the count of integer columns */
      (*intcount)++;

      /* Update the count of positive parameter values */
      if(rownr == 0)
        rowval = unscaled_mat(lp, lp->orig_obj[jb], 0, jb);
      else
        rowval = get_mat_byindex(lp, jb, TRUE, FALSE);
      if(rowval > 0)
        (*plucount)++;

      /* Check if the parameter value is integer and update the row's GCD */
      rowval = fabs(rowval) * intfrac;
      rowval += rowval*lp->epsmachine;
      rowval = modf(rowval, &inthold);
      if(rowval < lp->epsprimal) {
        (*intval)++;
        if(*intval == 1)
          intGCD = (int) inthold;
        else
          intGCD = gcd(intGCD, (LLONG) inthold, &multA, &multB);
      }
    }
    *valGCD = intGCD;
    *valGCD /= intfrac;
  }

  return(nn);
}

REAL MIP_stepOF(lprec *lp)
/* This function tries to find a non-zero minimum improvement
   if the OF contains all integer variables (logic only applies if we are
   looking for a single solution, not possibly several equal-valued ones).
*/
{
  MYBOOL OFgcd;
  int    colnr, rownr, n, ib, ie, maxndec,
         pluscount, intcount, intval;
  REAL   value, valOF, divOF, valGCD;
  MATrec *mat = lp->matA;

  value = 0;
  if((lp->int_vars > 0) && (lp->solutionlimit == 1) && mat_validate(mat)) {

    /* Get statistics for integer OF variables and compute base stepsize */
    n = row_intstats(lp, 0, -1, &maxndec, &pluscount, &intcount, &intval, &valGCD, &divOF);
    if((n == 0) || (maxndec < 0))
      return( value );
    OFgcd = (MYBOOL) (intval > 0);
    if(OFgcd)
      value = valGCD;

    /* Check non-ints in the OF to see if we can get more info */
    if(n - intcount > 0) {
      int nrv = 0;

      /* See if we have equality constraints */
      ie = lp->rows;
      for(ib = 1; ib <= ie; ib++) {
        if(is_constr_type(lp, ib, EQ))
          break;
      }

      /* If so, there may be a chance to find an improved stepsize */
      if(ib < ie)
      for(colnr = 1; colnr <= lp->columns; colnr++) {

        /* Go directly to the next variable if this is an integer or
          there is no row candidate to explore for hidden bounds for
          real-valued variables (limit scan to one row!) */
        if(is_int(lp, colnr))
          continue;
        nrv++;
        /* Scan equality constraints */
        ib = mat->col_end[colnr-1];
        ie = mat->col_end[colnr];
        while(ib < ie) {
          if(is_constr_type(lp, (rownr = COL_MAT_ROWNR(ib)), EQ)) {

            /* Get "child" row statistics, but break out if we don't
              find enough information, i.e. no integers with coefficients of proper type */
            n = row_intstats(lp, rownr, colnr, &maxndec, &pluscount, &intcount, &intval, &valGCD, &divOF);
            if((intval < n - 1) || (maxndec < 0)) {
              value = 0;
              break;
            }

            /* We can update */
            valOF = unscaled_mat(lp, lp->orig_obj[colnr], 0, colnr);
            valOF = fabs( valOF * (valGCD / divOF) );
            if(OFgcd) {
              SETMIN(value, valOF);
            }
            else {
              OFgcd = TRUE;
              value = valOF;
            }
          }
          ib++;
        }

        /* No point in continuing scan if we failed in current column */
        if(value == 0)
          break;
      }

      /* Check if we found information for any real-valued variable;
         if not, then we must set the iprovement delta to 0 */
      if(nrv == 0)
        value = 0;
    }
  }
  return( value );
}

STATIC MYBOOL isPrimalSimplex(lprec *lp)
{
  return((MYBOOL) (((lp->simplex_mode & SIMPLEX_Phase1_PRIMAL) != 0) ||
                   ((lp->simplex_mode & SIMPLEX_Phase2_PRIMAL) != 0)));
}

STATIC MYBOOL isPhase1(lprec *lp)
{
  return((MYBOOL) (((lp->simplex_mode & SIMPLEX_Phase1_PRIMAL) != 0) ||
                   ((lp->simplex_mode & SIMPLEX_Phase1_DUAL) != 0)));
}

STATIC MYBOOL isP1extra(lprec *lp)
{
  return((MYBOOL) ((lp->P1extraDim > 0) || (lp->P1extraVal != 0)));
}

STATIC MYBOOL feasiblePhase1(lprec *lp, REAL epsvalue)
{
  REAL   gap;
  MYBOOL test;

  gap = fabs(lp->rhs[0] - lp->orig_rhs[0]);
  test = (MYBOOL) (gap < epsvalue);
  return( test) ;
}

STATIC MYBOOL isDegenerateBasis(lprec *lp, int basisvar)
{
  int varindex;

  varindex = lp->var_basic[basisvar];
  if((fabs(lp->rhs[basisvar]) < lp->epsprimal) ||
     (fabs(lp->upbo[varindex]-lp->rhs[basisvar]) < lp->epsprimal))
    return( TRUE );
  else
    return( FALSE );
}

STATIC int findBasicFixedvar(lprec *lp, int afternr, MYBOOL slacksonly)
{
  int varnr, delta = 1;

  if(afternr < 0) {
    delta = -1;
    afternr = -afternr;
  }
  afternr += delta;
  if((afternr < 1) || (afternr > lp->rows))
    return( 0 );

  for(; (afternr > 0) && (afternr <= lp->rows); afternr += delta) {
    varnr = lp->var_basic[afternr];
    if(((varnr <= lp->rows) && is_constr_type(lp, varnr, EQ)) ||
       (!slacksonly && (varnr > lp->rows) && is_fixedvar(lp, varnr)))
      break;
  }

  if(afternr > lp->rows)
    afternr = 0;

  return( afternr );
}

STATIC MYBOOL isBasisVarFeasible(lprec *lp, REAL tol, int basis_row)
{
  int    col;
  REAL   x;
  MYBOOL Ok = TRUE;
  MYBOOL doSC = FALSE;

  col = lp->var_basic[basis_row];
  x = lp->rhs[basis_row];         /* The current solution of basic variables stored here! */
  if((x < -tol) || (x > lp->upbo[col]+tol))
    Ok = FALSE;
  else if(doSC && (col > lp->rows) && (fabs(lp->sc_lobound[col - lp->rows]) > 0)) {
    if((x > tol) && (x < fabs(lp->sc_lobound[col - lp->rows])-tol))
      Ok = FALSE;
  }
  return( Ok );
}
STATIC MYBOOL isPrimalFeasible(lprec *lp, REAL tol, int infeasibles[], REAL *feasibilitygap)
{
  int    i;
  MYBOOL feasible = TRUE;

  /* This is a short-hand call to rowdual() to check for primal infeasibility */

#if 0
  /* Traditional indexing style */
  for(i = 1; i <= lp->rows; i++) {
    feasible = isBasisVarFeasible(lp, tol, i);
#else
  /* Fast array pointer style */
  LREAL *rhsptr;
  int  *idxptr;

  if(infeasibles != NULL)
    infeasibles[0] = 0;
  for(i = 1, rhsptr = lp->rhs+1, idxptr = lp->var_basic+1;
      (i <= lp->rows); i++, rhsptr++, idxptr++) {
    feasible = TRUE;
/*    if(((*rhsptr) < lp->lowbo[*idxptr]-tol) || ((*rhsptr) > lp->upbo[*idxptr]+tol)) */
    if(((*rhsptr) < -tol) || ((*rhsptr) > lp->upbo[*idxptr]+tol))
      feasible = FALSE;
#endif
    if(!feasible) {
      if(infeasibles == NULL)
        break;
      infeasibles[0]++;
      infeasibles[infeasibles[0]] = i;
    }
  }

  /* Compute feasibility gap (could actually do this calculation above) */
  if(feasibilitygap != NULL) {
    if(feasible)
      *feasibilitygap = 0.0;
    else
      *feasibilitygap = feasibilityOffset(lp, FALSE);
  }

  return(feasible);
}

STATIC MYBOOL isDualFeasible(lprec *lp, REAL tol, int *boundflipcount, int infeasibles[], REAL *feasibilitygap)
{
  int    i, varnr,
         n = 0,  /* Number of infeasible duals corrected with bound-swaps */
         m = 0,
         target = SCAN_ALLVARS+USE_NONBASICVARS;
  REAL   f = 0;
  MYBOOL feasible, islower;


  /* The reduced costs are the values of the dual slacks, which
     are [0..Inf> for feasibility.  If we have negative reduced costs
     for bounded non-basic variables, we can simply switch the bound
     of bounded variables to obtain dual feasibility and possibly avoid
     having to use dual simplex phase 1. */
  if((infeasibles != NULL) || (boundflipcount != NULL)) {
    int  *nzdcol = NULL;
    REAL d, *dcol = NULL;

    f = compute_dualslacks(lp, target, &dcol, &nzdcol, FALSE);
    if(nzdcol != NULL)
    for(i = 1; i <= nzdcol[0]; i++) {
      varnr = nzdcol[i];
      islower = lp->is_lower[varnr];
      d = my_chsign(!islower, dcol[varnr]);

      /* Don't bother with uninteresting non-basic variables */
      if((d > -tol) ||                /* Positive reduced costs with a tolerance */
         my_unbounded(lp, varnr) ||   /* Free variables cannot change bound */
         is_fixedvar(lp, varnr))      /* Equality slack or a fixed variable ("type 3") */
        continue;

      /* Check if we have non-flippable bounds, i.e. an unbounded variable
         (types 2+4), or bounded variables (type 3), and if the counter is NULL. */
      if( (boundflipcount == NULL) ||
          ((lp->bb_level <= 1) && (my_rangebo(lp, varnr) > fabs(lp->negrange))) ||
          (islower && my_infinite(lp, lp->upbo[varnr])) ||
          (!islower && my_infinite(lp, my_lowbo(lp, varnr))) ) {
        m++;
        if(infeasibles != NULL)
          infeasibles[m] = varnr;
      }
      /* Only do bound flips if the user-provided counter is non-NULL */
      else {
        lp->is_lower[varnr] = !islower;
        n++;
      }
    }
    if(infeasibles != NULL)
      infeasibles[0] = m;
    FREE(dcol);
    FREE(nzdcol);
    if(n > 0) {
      set_action(&lp->spx_action, ACTION_RECOMPUTE);
      if(m == 0)
        f = 0;
    }
  }
  else
    f = compute_dualslacks(lp, target, NULL, NULL, FALSE);
/*    f = feasibilityOffset(lp, TRUE); */  /* Safe legacy mode */

  /* Do an extra scan to see if there are bounded variables in the OF not present in any constraint;
     Most typically, presolve fixes such cases, so this is rarely encountered. */

  varnr = lp->rows + 1;
  for(i = 1; i <= lp->columns; i++, varnr++) {
    islower = lp->is_lower[varnr];
    if((my_chsign(islower, lp->orig_obj[i]) > 0) && (mat_collength(lp->matA, i) == 0) && !SOS_is_member(lp->SOS, 0, i)) {
      lp->is_lower[varnr] = !islower;
      if((islower && my_infinite(lp, lp->upbo[varnr])) ||
         (!islower && my_infinite(lp, my_lowbo(lp, varnr)))) {
        lp->spx_status = UNBOUNDED;
        break;
      }
      n++;
    }
  }

  /* Return status */

  if(boundflipcount != NULL)
    *boundflipcount = n;
  if(feasibilitygap != NULL) {
    my_roundzero(f, tol);
    *feasibilitygap = f;
  }
  feasible = (MYBOOL) ((f == 0) && (m == 0));

  return(feasible);
}

void __WINAPI default_basis(lprec *lp)
{
  int i;

  /* Set the slack variables to be basic; note that the is_basic[] array
     is a helper array filled in presolve() to match var_basic[]. */
  for(i = 1; i <= lp->rows; i++) {
    lp->var_basic[i] = i;
    lp->is_basic[i] = TRUE;
    lp->is_lower[i] = TRUE;
  }
  lp->var_basic[0] = TRUE; /* Set to signal that this is the default basis */

  /* Set user variables at their lower bound, including the
     dummy slack for the objective "constraint" */
  for(; i <= lp->sum; i++) {
    lp->is_basic[i] = FALSE;
    lp->is_lower[i] = TRUE;
  }
  lp->is_lower[0] = TRUE;

  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
  lp->basis_valid = TRUE;  /* Do not re-initialize basis on entering Solve */
}

int __WINAPI get_basiscrash(lprec *lp)
{
  return(lp->crashmode);
}

void __WINAPI set_basiscrash(lprec *lp, int mode)
{
  lp->crashmode = mode;
}

MYBOOL __WINAPI set_basis(lprec *lp, int *bascolumn, MYBOOL nonbasic)   /* Added by KE */
{
  int    i,s,k,n;

  /* Make sure we are consistent */
  if(lp->wasPresolved && ((lp->rows != lp->presolve_undo->orig_rows) ||
                          (lp->columns != lp->presolve_undo->orig_columns)))
    return( FALSE );

 /* Initialize (lp->is_basic is set in preprocess); Note that as of v5 and before
    it is an lp_solve convention that basic variables are at their lower bounds!
    This routine provides for the a possible future case that basic variables
    can be upper-bounded. */
  lp->is_lower[0] = TRUE;
  for(i = 1; i <= lp->sum; i++) {
    lp->is_lower[i] = TRUE;
    lp->is_basic[i] = FALSE;
  }
  for(i = 1; i <= lp->rows; i++)
    lp->var_basic[i] = FALSE;

 /* Set basic and optionally non-basic variables;
    negative index means at lower bound, positive at upper bound */
  if(nonbasic)
    n = lp->sum;
  else
    n = lp->rows;
  for(i = 1; i <= n; i++) {
    s = bascolumn[i];
    k = abs(s);
    if(k <= 0 || k > lp->sum)
      return( FALSE );
    if(i <= lp->rows) {
      lp->var_basic[i] = k;
      lp->is_basic[k] = TRUE;
    }
    else     /* Remove this test if basic variables can be upper-bounded */
    if(s > 0)
      lp->is_lower[k] = FALSE;
  }
  if(!verify_basis(lp))
    return( FALSE );

 /* Invalidate basis */
  set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT | ACTION_RECOMPUTE);
  lp->basis_valid = TRUE;   /* Do not re-initialize basis on entering Solve */
  lp->var_basic[0] = FALSE; /* Set to signal that this is a non-default basis */

  return( TRUE );
}

void __WINAPI reset_basis(lprec *lp)
{
  lp->basis_valid = FALSE;   /* Causes reinversion at next opportunity */
}

MYBOOL __WINAPI get_basis(lprec *lp, int *bascolumn, MYBOOL nonbasic)
{
  int    k, i;

  if(!lp->basis_valid ||
     (lp->rows != lp->presolve_undo->orig_rows) ||
     (lp->columns != lp->presolve_undo->orig_columns))
    return( FALSE );

  *bascolumn = 0;

  /* First save basic variable indexes */
  for(i = 1; i <= lp->rows; i++) {
    k = lp->var_basic[i];
    bascolumn[i] = my_chsign(lp->is_lower[k], k);
  }

  /* Then optionally save non-basic variable indeces */
  if(nonbasic) {
    for(k = 1; (k <= lp->sum) && (i <= lp->sum); k++) {
      if(lp->is_basic[k])
        continue;
      bascolumn[i] = my_chsign(lp->is_lower[k], k);
      i++;
    }
  }
  return( TRUE );
}

STATIC MYBOOL is_BasisReady(lprec *lp)
{
  return( (MYBOOL) (lp->var_basic[0] != AUTOMATIC) );
}

STATIC MYBOOL is_slackbasis(lprec *lp)
{
  int n = 0, err = 0;
  if(lp->basis_valid) {
    int i, k;
    MYBOOL *used = NULL;

    allocMYBOOL(lp, &used, lp->rows+1, TRUE);
    for(i = 1; i <= lp->rows; i++) {
      k = lp->var_basic[i];
      if(k <= lp->rows) {
        if(used[k])
          err++;
        else
          used[k] = TRUE;
        n++;
      }
    }
    FREE(used);
    if(err > 0)
      report(lp, SEVERE, "is_slackbasis: %d inconsistencies found in slack basis\n", err);
  }
  return( (MYBOOL) (n == lp->rows) );
}

STATIC MYBOOL verify_basis(lprec *lp)
{
  int    i, ii, k = 0;
  MYBOOL result = FALSE;

  for(i = 1; i <= lp->rows; i++) {
    ii = lp->var_basic[i];
    if((ii < 1) || (ii > lp->sum) || !lp->is_basic[ii]) {
      k = i;
      ii = 0;
      goto Done;
    }
  }

  ii = lp->rows;
  for(i = 1; i <= lp->sum; i++) {
    if(lp->is_basic[i])
      ii--;
  }
  result = (MYBOOL) (ii == 0);

Done:
#if 0  /* For testing */
  if(!result)
    ii = 0;
#endif
  return(result);
}

int __WINAPI set_basisvar(lprec *lp, int basisPos, int enteringCol)
{
  int leavingCol;

  leavingCol = lp->var_basic[basisPos];

#ifdef Paranoia
  if((basisPos < 1) || (basisPos > lp->rows))
    report(lp, SEVERE, "set_basisvar: Invalid leaving basis position %d specified at iter %.0f\n",
                       basisPos, (double) get_total_iter(lp));
  if((leavingCol < 1) || (leavingCol > lp->sum))
    report(lp, SEVERE, "set_basisvar: Invalid leaving column %d referenced at iter %.0f\n",
                       leavingCol, (double) get_total_iter(lp));
  if((enteringCol < 1) || (enteringCol > lp->sum))
    report(lp, SEVERE, "set_basisvar: Invalid entering column %d specified at iter %.0f\n",
                       enteringCol, (double) get_total_iter(lp));
#endif

#ifdef ParanoiaXY
  if(!lp->is_basic[leavingCol])
    report(lp, IMPORTANT, "set_basisvar: Leaving variable %d is not basic at iter %.0f\n",
                           leavingCol, (double) get_total_iter(lp));
  if(enteringCol > lp->rows && lp->is_basic[enteringCol])
    report(lp, IMPORTANT, "set_basisvar: Entering variable %d is already basic at iter %.0f\n",
                           enteringCol, (double) get_total_iter(lp));
#endif

  lp->var_basic[0]          = FALSE;       /* Set to signal that this is a non-default basis */
  lp->var_basic[basisPos]   = enteringCol;
  lp->is_basic[leavingCol]  = FALSE;
  lp->is_basic[enteringCol] = TRUE;
  if(lp->bb_basis != NULL)
    lp->bb_basis->pivots++;

  return(leavingCol);
}

/* Bounds updating and unloading routines; requires that the
   current values for upbo and lowbo are in the original base. */
STATIC int perturb_bounds(lprec *lp, BBrec *perturbed, MYBOOL doRows, MYBOOL doCols, MYBOOL includeFIXED)
{
  int  i, ii, n = 0;
  REAL new_lb, new_ub, *upbo, *lowbo;

  if(perturbed == NULL)
    return( n );

 /* Map reference bounds to previous state, i.e. cumulate
    perturbations in case of persistent problems */
  upbo  = perturbed->upbo;
  lowbo = perturbed->lowbo;

 /* Set appropriate target variable range */
  i = 1;
  ii = lp->rows;
  if(!doRows)
    i += ii;
  if(!doCols)
    ii = lp->sum;

 /* Perturb (expand) finite variable bounds randomly */
  for(; i <= ii; i++) {

    /* Don't perturb regular slack variables */
    if((i <= lp->rows) && (lowbo[i] == 0) && (upbo[i] >= lp->infinite))
      continue;

    new_lb = lowbo[i];
    new_ub = upbo[i];

    /* Don't perturb fixed variables if not specified */
    if(!includeFIXED && (new_ub == new_lb))
      continue;

    /* Lower bound for variables (consider implementing RHS here w/contentmode== AUTOMATIC) */
    if((i > lp->rows) && (new_lb < lp->infinite)) {
      new_lb = rand_uniform(lp, RANDSCALE) + 1;
      new_lb *= lp->epsperturb;
      lowbo[i] -= new_lb;
      n++;
    }

    /* Upper bound */
    if(new_ub < lp->infinite) {
      new_ub = rand_uniform(lp, RANDSCALE) + 1;
      new_ub *= lp->epsperturb;
      upbo[i] += new_ub;
      n++;
    }
  }

 /* Make sure we start from scratch */
  set_action(&lp->spx_action, ACTION_REBASE);

  return( n );
}

STATIC MYBOOL impose_bounds(lprec *lp, REAL *upbo, REAL *lowbo)
/* Explicitly set working bounds to given vectors without pushing or popping */
{
  MYBOOL ok;

  ok = (MYBOOL) ((upbo != NULL) || (lowbo != NULL));
  if(ok) {
    if((upbo != NULL) && (upbo != lp->upbo))
      MEMCOPY(lp->upbo,  upbo,  lp->sum + 1);
    if((lowbo != NULL) && (lowbo != lp->lowbo))
      MEMCOPY(lp->lowbo, lowbo, lp->sum + 1);
    if(lp->bb_bounds != NULL)
      lp->bb_bounds->UBzerobased = FALSE;
    set_action(&lp->spx_action, ACTION_REBASE);
  }
  set_action(&lp->spx_action, ACTION_RECOMPUTE);
  return( ok );
}

STATIC MYBOOL validate_bounds(lprec *lp, REAL *upbo, REAL *lowbo)
/* Check if all bounds are Explicitly set working bounds to given vectors without pushing or popping */
{
  MYBOOL ok;
  int    i;

  ok = (MYBOOL) ((upbo != NULL) || (lowbo != NULL));
  if(ok) {
    for(i = 1; i <= lp->sum; i++)
      if((lowbo[i] > upbo[i]) || (lowbo[i] < lp->orig_lowbo[i]) || (upbo[i] > lp->orig_upbo[i]))
        break;
    ok = (MYBOOL) (i > lp->sum);
  }
  return( ok );
}

STATIC int unload_BB(lprec *lp)
{
  int levelsunloaded = 0;

  if(lp->bb_bounds != NULL)
    while(pop_BB(lp->bb_bounds))
      levelsunloaded++;
  return( levelsunloaded );
}


#define LowerStorageModel 1
#define BasisStorageModel 1
STATIC basisrec *push_basis(lprec *lp, int *basisvar, MYBOOL *isbasic, MYBOOL *islower)
/* Save the ingoing basis and push it onto the stack */
{
  int sum = lp->sum + 1;
  basisrec *newbasis = NULL;

  newbasis = (basisrec *) calloc(sizeof(*newbasis), 1);
  if((newbasis != NULL) &&
#if LowerStorageModel == 0
    allocMYBOOL(lp, &newbasis->is_lower,  sum,  FALSE) &&
#else
    allocMYBOOL(lp, &newbasis->is_lower,  (sum + 8) / 8,  TRUE) &&
#endif
#if BasisStorageModel == 0
    allocMYBOOL(lp, &newbasis->is_basic,  sum,  FALSE) &&
#endif
    allocINT(lp,    &newbasis->var_basic, lp->rows + 1, FALSE)) {

    if(islower == NULL)
      islower = lp->is_lower;
    if(isbasic == NULL)
      isbasic = lp->is_basic;
    if(basisvar == NULL)
      basisvar = lp->var_basic;

#if LowerStorageModel == 0
    MEMCOPY(newbasis->is_lower,  islower,  sum);
#else
    for(sum = 1; sum <= lp->sum; sum++)
      if(islower[sum])
        set_biton(newbasis->is_lower, sum);
#endif
#if BasisStorageModel == 0
    MEMCOPY(newbasis->is_basic,  isbasic,  lp->sum + 1);
#endif
    MEMCOPY(newbasis->var_basic, basisvar, lp->rows + 1);

    newbasis->previous = lp->bb_basis;
    if(lp->bb_basis == NULL)
      newbasis->level = 0;
    else
      newbasis->level = lp->bb_basis->level + 1;
    newbasis->pivots = 0;

    lp->bb_basis = newbasis;
  }
  return( newbasis );
}

STATIC MYBOOL compare_basis(lprec *lp)
/* Compares the last pushed basis with the currently active basis */
{
  int i, j;
  MYBOOL same_basis = TRUE;

  if(lp->bb_basis == NULL)
    return( FALSE );

  /* Loop over basis variables until a mismatch (order can be different) */
  i = 1;
  while(same_basis && (i <= lp->rows)) {
    j = 1;
    while(same_basis && (j <= lp->rows)) {
      same_basis = (MYBOOL) (lp->bb_basis->var_basic[i] != lp->var_basic[j]);
      j++;
    }
    same_basis = !same_basis;
    i++;
  }
  /* Loop over bound status indicators until a mismatch */
  i = 1;
  while(same_basis && (i <= lp->sum)) {
    same_basis = (lp->bb_basis->is_lower[i] && lp->is_lower[i]);
    i++;
  }

  return( same_basis );
}

STATIC MYBOOL restore_basis(lprec *lp)
/* Restore values from the previously pushed / saved basis without popping it */
{
  MYBOOL ok;
  int    i;

  ok = (MYBOOL) (lp->bb_basis != NULL);
  if(ok) {
    MEMCOPY(lp->var_basic, lp->bb_basis->var_basic, lp->rows + 1);
#if BasisStorageModel == 0
    MEMCOPY(lp->is_basic,  lp->bb_basis->is_basic,  lp->sum + 1);
#else
    MEMCLEAR(lp->is_basic, lp->sum + 1);
    for(i = 1; i <= lp->rows; i++)
      lp->is_basic[lp->var_basic[i]] = TRUE;
#endif
#if LowerStorageModel == 0
    MEMCOPY(lp->is_lower,  lp->bb_basis->is_lower,  lp->sum + 1);
#else
    for(i = 1; i <= lp->sum; i++)
      lp->is_lower[i] = is_biton(lp->bb_basis->is_lower, i);
#endif
    set_action(&lp->spx_action, ACTION_REBASE | ACTION_REINVERT);
  }
  return( ok );
}

STATIC MYBOOL pop_basis(lprec *lp, MYBOOL restore)
/* Pop / free, and optionally restore the previously "pushed" / saved basis */
{
  MYBOOL ok;
  basisrec *oldbasis;

  ok = (MYBOOL) (lp->bb_basis != NULL);
  if(ok) {
    oldbasis = lp->bb_basis;
    if(oldbasis != NULL) {
      lp->bb_basis = oldbasis->previous;
      FREE(oldbasis->var_basic);
#if BasisStorageModel == 0
      FREE(oldbasis->is_basic);
#endif
      FREE(oldbasis->is_lower);
      FREE(oldbasis);
    }
    if(restore && (lp->bb_basis != NULL))
      restore_basis(lp);
  }
  return( ok );
}

STATIC int unload_basis(lprec *lp, MYBOOL restorelast)
{
  int levelsunloaded = 0;

  if(lp->bb_basis != NULL)
    while(pop_basis(lp, restorelast))
      levelsunloaded++;
  return( levelsunloaded );
}


STATIC REAL scaled_floor(lprec *lp, int colnr, REAL value, REAL epsscale)
{
  value = floor(value);
  if(value != 0)
  if(lp->columns_scaled && is_integerscaling(lp)) {
    value = scaled_value(lp, value, colnr);
    if(epsscale != 0)
      value += epsscale*lp->epsmachine;
/*      value += epsscale*lp->epsprimal; */
/*    value = restoreINT(value, lp->epsint); */
  }
  return(value);
}

STATIC REAL scaled_ceil(lprec *lp, int colnr, REAL value, REAL epsscale)
{
  value = ceil(value);
  if(value != 0)
  if(lp->columns_scaled && is_integerscaling(lp)) {
    value = scaled_value(lp, value, colnr);
    if(epsscale != 0)
      value -= epsscale*lp->epsmachine;
/*      value -= epsscale*lp->epsprimal; */
/*    value = restoreINT(value, lp->epsint); */
  }
  return(value);
}

/* Branch and bound variable selection functions */

STATIC MYBOOL is_sc_violated(lprec *lp, int column)
{
  int  varno;
  REAL tmpreal;

  varno = lp->rows+column;
  tmpreal = unscaled_value(lp, lp->sc_lobound[column], varno);
  return( (MYBOOL) ((tmpreal > 0) &&                    /* it is an (inactive) SC variable...    */
                    (lp->solution[varno] < tmpreal) &&  /* ...and the NZ lower bound is violated */
                    (lp->solution[varno] > 0)) );       /* ...and the Z lowerbound is violated   */
}
STATIC int find_sc_bbvar(lprec *lp, int *count)
{
  int    i, ii, n, bestvar;
  int    firstsc, lastsc;
  REAL   hold, holdINT, bestval, OFval, randval, scval;
  MYBOOL reversemode, greedymode, randomizemode,
         pseudocostmode, pseudocostsel;

  bestvar = 0;
  if((lp->sc_vars == 0) || (*count > 0))
    return(bestvar);

  reversemode    = is_bb_mode(lp, NODE_WEIGHTREVERSEMODE);
  greedymode     = is_bb_mode(lp, NODE_GREEDYMODE);
  randomizemode  = is_bb_mode(lp, NODE_RANDOMIZEMODE);
  pseudocostmode = is_bb_mode(lp, NODE_PSEUDOCOSTMODE);
  pseudocostsel  = is_bb_rule(lp, NODE_PSEUDOCOSTSELECT) ||
                   is_bb_rule(lp, NODE_PSEUDONONINTSELECT) ||
                   is_bb_rule(lp, NODE_PSEUDORATIOSELECT);

  bestvar = 0;
  bestval = -lp->infinite;
  hold    = 0;
  randval = 1;
  firstsc = 0;
  lastsc  = lp->columns;

  for(n = 1; n <= lp->columns; n++) {
    ii = get_var_priority(lp, n);
    i = lp->rows + ii;
    if(!lp->bb_varactive[ii] && is_sc_violated(lp, ii) && !SOS_is_marked(lp->SOS, 0, ii)) {

      /* Do tallies */
      (*count)++;
      lastsc = i;
      if(firstsc <= 0)
        firstsc = i;
      scval = get_pseudorange(lp->bb_PseudoCost, ii, BB_SC);

      /* Select default pricing/weighting mode */
      if(pseudocostmode)
        OFval = get_pseudonodecost(lp->bb_PseudoCost, ii, BB_SC, lp->solution[i]);
      else
        OFval = my_chsign(is_maxim(lp), get_mat(lp, 0, ii));

      if(randomizemode)
        randval = exp(rand_uniform(lp, 1.0));

      /* Find the maximum pseudo-cost of a variable (don't apply pseudocostmode here) */
      if(pseudocostsel) {
        if(pseudocostmode)
          hold = OFval;
        else
          hold = get_pseudonodecost(lp->bb_PseudoCost, ii, BB_SC, lp->solution[i]);
        hold *= randval;
        if(greedymode) {
          if(pseudocostmode) /* Override! */
            OFval = my_chsign(is_maxim(lp), get_mat(lp, 0, ii));
          hold *= OFval;
        }
        hold = my_chsign(reversemode, hold);
      }
      else
      /* Find the variable with the largest sc gap (closest to the sc mean) */
      if(is_bb_rule(lp, NODE_FRACTIONSELECT)) {
        hold = modf(lp->solution[i]/scval, &holdINT);
        holdINT = hold-1;
        if(fabs(holdINT) > hold)
          hold = holdINT;
        if(greedymode)
          hold *= OFval;
        hold = my_chsign(reversemode, hold)*scval*randval;
      }
      else
      /* Do first or last violated sc index selection (default) */
      /* if(is_bb_rule(lp, NODE_FIRSTSELECT)) */
      {
        if(reversemode)
          continue;
        else {
          bestvar = i;
          break;
        }
      }

      /* Select better, check for ties, and split by proximity to 0.5*sc_lobound */
      if(hold > bestval) {
        if( (bestvar == 0) ||
            (hold > bestval+lp->epsprimal) ||
            (fabs(modf(lp->solution[i]/scval, &holdINT) - 0.5) <
             fabs(modf(lp->solution[bestvar]/
                       get_pseudorange(lp->bb_PseudoCost, bestvar-lp->rows, BB_SC), &holdINT) - 0.5)) ) {
          bestval = hold;
          bestvar = i;
        }
      }
    }
  }

  if(is_bb_rule(lp, NODE_FIRSTSELECT) && reversemode)
    bestvar = lastsc;

  return(bestvar);
}

STATIC int find_sos_bbvar(lprec *lp, int *count, MYBOOL intsos)
{
  int k, i, j, var;

  var = 0;
  if((lp->SOS == NULL) || (*count > 0))
    return(var);

  /* Check if the SOS'es happen to already be satisified */
  i = SOS_is_satisfied(lp->SOS, 0, lp->solution);
  if((i == SOS_COMPLETE) || (i == SOS_INCOMPLETE))
    return(-1);

  /* Otherwise identify a SOS variable to enter B&B */
  for(k = 0; k < lp->sos_vars; k++) {
    i = lp->sos_priority[k];
#ifdef Paranoia
    if((i < 1) || (i > lp->columns))
      report(lp, SEVERE, "find_sos_bbvar: Invalid SOS variable map %d at %d\n",
                         i, k);
#endif
    j = lp->rows + i;
    if(!SOS_is_marked(lp->SOS, 0, i) && !SOS_is_full(lp->SOS, 0, i, FALSE)) {
/*    if(!SOS_is_marked(lp->SOS, 0, i) && !SOS_is_full(lp->SOS, 0, i, TRUE)) { */
      if(!intsos || is_int(lp, i)) {
        (*count)++;
        if(var == 0) {
          var = j;
          break;
        }
      }
    }
  }
#ifdef Paranoia
  if((var > 0) && !SOS_is_member(lp->SOS, 0, var-lp->rows))
     report(lp, SEVERE, "find_sos_bbvar: Found variable %d, which is not a SOS!\n", var);
#endif
  return(var);
}

STATIC int find_int_bbvar(lprec *lp, int *count, BBrec *BB, MYBOOL *isfeasible)
{
  int    i, ii, n, k, bestvar, depthmax, *nonint = NULL;
  REAL   hold, holdINT, bestval, OFval, randval,
         *lowbo = BB->lowbo, *upbo = BB->upbo;
  MYBOOL reversemode, greedymode, depthfirstmode, breadthfirstmode,
         randomizemode, rcostmode,
         pseudocostmode, pseudocostsel, pseudostrong, isINT, valINT;

  if((lp->int_vars == 0) || (*count > 0))
    return( 0 );
  if(lp->bb_usenode != NULL) {
    i = lp->bb_usenode(lp, lp->bb_nodehandle, BB_INT);
    if(i >= 0) {
      if(i > 0)
        (*count)++;
      return( i );
    }
  }

  reversemode    = is_bb_mode(lp, NODE_WEIGHTREVERSEMODE);
  greedymode     = is_bb_mode(lp, NODE_GREEDYMODE);
  randomizemode  = is_bb_mode(lp, NODE_RANDOMIZEMODE);
  depthfirstmode = is_bb_mode(lp, NODE_DEPTHFIRSTMODE);
  breadthfirstmode = is_bb_mode(lp, NODE_BREADTHFIRSTMODE) &&
                     (MYBOOL) (lp->bb_level <= lp->int_vars);
  rcostmode      = (MYBOOL) (BB->lp->solutioncount > 0) && is_bb_mode(lp, NODE_RCOSTFIXING);
  pseudocostmode = is_bb_mode(lp, NODE_PSEUDOCOSTMODE);
  pseudocostsel  = is_bb_rule(lp, NODE_PSEUDOCOSTSELECT) ||
                   is_bb_rule(lp, NODE_PSEUDONONINTSELECT) ||
                   is_bb_rule(lp, NODE_PSEUDORATIOSELECT);
  pseudostrong   = FALSE &&
                   pseudocostsel && !rcostmode && is_bb_mode(lp, NODE_STRONGINIT);

  /* Fill list of non-ints */
  allocINT(lp, &nonint, lp->columns + 1, FALSE);
  n = 0;
  depthmax = -1;
  if(isfeasible != NULL)
    *isfeasible = TRUE;
  BB->lastrcf = 0;
  for(k = 1; (k <= lp->columns); k++) {
    ii = get_var_priority(lp, k);
    isINT = is_int(lp,ii);
    i  = lp->rows + ii;

    /* Tally reduced cost fixing opportunities for ranged non-basic nonINTs */
    if(!isINT) {
#ifdef UseMilpExpandedRCF
      if(rcostmode) {
        bestvar = rcfbound_BB(BB, i, isINT, NULL, isfeasible);
        if(bestvar != FR)
          BB->lastrcf++;
      }
#endif
    }
    else {

      valINT = solution_is_int(lp, i, FALSE);

      /* Skip already fixed variables */
      if(lowbo[i] == upbo[i]) {

        /* Check for validity */
#ifdef Paranoia
        if(!valINT) {
          report(lp, IMPORTANT,
                 "find_int_bbvar: INT var %d was fixed at %d, but computed as %g at node %.0f\n",
                  ii, (int) lowbo[i], lp->solution[i], (double) lp->bb_totalnodes);
          lp->bb_break = TRUE;
          lp->spx_status = UNKNOWNERROR;
          bestvar = 0;
          goto Done;
        }
#endif
      }

      /* The variable has not yet been fixed */
      else {

        /* Tally reduced cost fixing opportunities (also when the
           variables are integer-valued at the current relaxation) */
        if(rcostmode) {
          bestvar = rcfbound_BB(BB, i, isINT, NULL, isfeasible);
          if(bestvar != FR)
            BB->lastrcf++;
        }
        else
          bestvar = FR;

        /* Only qualify variable as branching node if it is non-integer and
           it will not be subsequently fixed via reduced cost fixing logic */
        if(!valINT && (bestvar >= FR)) {

          n++;
          nonint[n] = ii;
          SETMAX(depthmax, lp->bb_varactive[ii]);
        }
      }

    }
  }

#ifdef UseMilpSlacksRCF
  /* Optionally also tally slacks */
  if(rcostmode) {
    for(i = 1; (i <= lp->rows) && (BB->lastrcf == 0); i++) {
      /* Skip already fixed slacks (equalities) */
      if(lowbo[i] < upbo[i]) {
        bestvar = rcfbound_BB(BB, i, FALSE, NULL, isfeasible);
        if(bestvar != FR)
          BB->lastrcf++;
      }
    }
  }
#endif
  nonint[0] = n;
  *count    = n;
  bestvar   = 0;
  if(n == 0)     /* No non-integers found */
    goto Done;

  bestval  = -lp->infinite;
  hold     = 0;
  randval  = 1;

  /* Sort non-ints by depth in case we have breadthfirst or depthfirst modes */
  if((lp->bb_level > 1) && (depthmax > 0) && (depthfirstmode || breadthfirstmode)) {
    int *depths = NULL;

    /* Fill attribute array and make sure ordinal order breaks ties during sort */
    allocINT(lp, &depths, n + 1, FALSE);
    for(i = 1; i <= n; i++)
      depths[i] = (depthfirstmode ? n+1-i : i) + (n+1)*lp->bb_varactive[nonint[i]];
    hpsortex(depths, n, 1, sizeof(*nonint), depthfirstmode, compareINT, nonint);
    FREE(depths);
  }

  /* Do simple firstselect handling */
  if(is_bb_rule(lp, NODE_FIRSTSELECT)) {
    if(reversemode)
      bestvar = lp->rows + nonint[nonint[0]];
    else
      bestvar = lp->rows + nonint[1];
  }

  else for(n = 1; n <= nonint[0]; n++) {
    ii = nonint[n];
    i = lp->rows + ii;

    /* Do the naive detection */
    if(n == 1)
      bestvar = i;

    /* Should we do a "strong" pseudo-cost initialization or an incremental update? */
    if(pseudostrong &&
       (MAX(lp->bb_PseudoCost->LOcost[ii].rownr,
            lp->bb_PseudoCost->UPcost[ii].rownr) < lp->bb_PseudoCost->updatelimit) &&
       (MAX(lp->bb_PseudoCost->LOcost[ii].colnr,
            lp->bb_PseudoCost->UPcost[ii].colnr) < 5*lp->bb_PseudoCost->updatelimit)) {
      strongbranch_BB(lp, BB, ii, BB_INT, nonint[0]);
    }

    /* Select default pricing/weighting mode */
    if(pseudocostmode)
      OFval = get_pseudonodecost(lp->bb_PseudoCost, ii, BB_INT, lp->solution[i]);
    else
      OFval = my_chsign(is_maxim(lp), get_mat(lp, 0, ii));

    if(randomizemode)
      randval = exp(rand_uniform(lp, 1.0));

    /* Find the maximum pseudo-cost of a variable (don't apply pseudocostmode here) */
    if(pseudocostsel) {
      if(pseudocostmode)
        hold = OFval;
      else
        hold = get_pseudonodecost(lp->bb_PseudoCost, ii, BB_INT, lp->solution[i]);
      hold *= randval;
      if(greedymode) {
        if(pseudocostmode) /* Override! */
          OFval = my_chsign(is_maxim(lp), get_mat(lp, 0, ii));
        hold *= OFval;
      }
      hold = my_chsign(reversemode, hold);
    }
    else
    /* Find the variable with the largest gap to its bounds (distance from being fixed) */
    if(is_bb_rule(lp, NODE_GAPSELECT)) {
      hold = lp->solution[i];
      holdINT = hold-unscaled_value(lp, upbo[i], i);
      hold -= unscaled_value(lp, lowbo[i], i);
      if(fabs(holdINT) > hold)
        hold = holdINT;
      if(greedymode)
        hold *= OFval;
      hold = my_chsign(reversemode, hold)*randval;
    }
    else
    /* Find the variable with the largest integer gap (closest to 0.5) */
    if(is_bb_rule(lp, NODE_FRACTIONSELECT)) {
      hold = modf(lp->solution[i], &holdINT);
      holdINT = hold-1;
      if(fabs(holdINT) > hold)
        hold = holdINT;
      if(greedymode)
        hold *= OFval;
      hold = my_chsign(reversemode, hold)*randval;
    }
    else
    /* Find the "range", most flexible variable */
    if(is_bb_rule(lp, NODE_RANGESELECT)) {
      hold = unscaled_value(lp, upbo[i]-lowbo[i], i);
      if(greedymode)
        hold *= OFval;
      hold = my_chsign(reversemode, hold)*randval;
    }

    /* Select better, check for ties, and split by proximity to 0.5 */
    if(hold > bestval) {
      if( (hold > bestval+lp->epsprimal) ||
          (fabs(modf(lp->solution[i], &holdINT) - 0.5) <
           fabs(modf(lp->solution[bestvar], &holdINT) - 0.5)) ) {
        bestval = hold;
        bestvar = i;
      }
    }
  }

Done:
  FREE(nonint);
  return(bestvar);
}

STATIC BBPSrec *init_pseudocost(lprec *lp, int pseudotype)
{
  int     i;
  REAL    PSinitUP, PSinitLO;
  BBPSrec *newitem;
  MYBOOL  isPSCount;

  /* Allocate memory */
  newitem = (BBPSrec*) malloc(sizeof(*newitem));
  newitem->lp = lp;
  newitem->LOcost = (MATitem*) malloc((lp->columns+1) * sizeof(*newitem->LOcost));
  newitem->UPcost = (MATitem*) malloc((lp->columns+1) * sizeof(*newitem->UPcost));
  newitem->secondary = NULL;

  /* Initialize with OF values */
  newitem->pseodotype = (pseudotype & NODE_STRATEGYMASK);
  isPSCount = ((pseudotype & NODE_PSEUDONONINTSELECT) != 0);
  for(i = 1; i <= lp->columns; i++) {
    newitem->LOcost[i].rownr = 1; /* Actual updates */
    newitem->LOcost[i].colnr = 1; /* Attempted updates */
    newitem->UPcost[i].rownr = 1;
    newitem->UPcost[i].colnr = 1;

    /* Initialize with the plain OF value as conventional usage suggests, or
       override in case of pseudo-nonint count strategy */
    PSinitUP = my_chsign(is_maxim(lp), get_mat(lp, 0, i));
    PSinitLO = -PSinitUP;
    if(isPSCount) {
      /* Set default assumed reduction in the number of non-ints by choosing this variable;
         KE changed from 0 on 30 June 2004 and made two-sided selectable.  Note that the
         typical value range is <0..1>, with a positive bias for an "a priori" assumed
         fast-converging (low "MIP-complexity") model. Very hard models may require
         negative initialized values for one or both. */
      PSinitUP = 0.1*0;
#if 0
      PSinitUP = my_chsign(PSinitUP < 0, PSinitUP);
      PSinitLO = -PSinitUP;
#else
      PSinitLO = PSinitUP;
#endif
    }
    newitem->UPcost[i].value = PSinitUP;
    newitem->LOcost[i].value = PSinitLO;
  }
  newitem->updatelimit     = lp->bb_PseudoUpdates;
  newitem->updatesfinished = 0;
  newitem->restartlimit    = DEF_PSEUDOCOSTRESTART;

  /* Let the user get an opportunity to initialize pseudocosts */
  if(userabort(lp, MSG_INITPSEUDOCOST))
    lp->spx_status = USERABORT;

  return( newitem );
}

STATIC MYBOOL free_pseudoclass(BBPSrec **PseudoClass)
{
  BBPSrec *target = *PseudoClass;

  FREE(target->LOcost);
  FREE(target->UPcost);
  target = target->secondary;
  FREE(*PseudoClass);
  *PseudoClass = target;

  return( (MYBOOL) (target != NULL) );
}

STATIC void free_pseudocost(lprec *lp)
{
  if((lp != NULL) && (lp->bb_PseudoCost != NULL)) {
    while(free_pseudoclass(&(lp->bb_PseudoCost)) );
  }
}

MYBOOL __WINAPI set_pseudocosts(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit)
{
  int i;

  if((lp->bb_PseudoCost == NULL) || ((clower == NULL) && (cupper == NULL)))
    return(FALSE);
  for(i = 1; i <= lp->columns; i++) {
    if(clower != NULL)
      lp->bb_PseudoCost->LOcost[i].value = clower[i];
    if(cupper != NULL)
      lp->bb_PseudoCost->UPcost[i].value = cupper[i];
  }
  if(updatelimit != NULL)
    lp->bb_PseudoCost->updatelimit = *updatelimit;
  return(TRUE);
}

MYBOOL __WINAPI get_pseudocosts(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit)
{
  int i;

  if((lp->bb_PseudoCost == NULL) || ((clower == NULL) && (cupper == NULL)))
    return(FALSE);
  for(i = 1; i <= lp->columns; i++) {
    if(clower != NULL)
      clower[i] = lp->bb_PseudoCost->LOcost[i].value;
    if(cupper != NULL)
      cupper[i] = lp->bb_PseudoCost->UPcost[i].value;
  }
  if(updatelimit != NULL)
    *updatelimit = lp->bb_PseudoCost->updatelimit;
  return(TRUE);
}

STATIC REAL get_pseudorange(BBPSrec *pc, int mipvar, int varcode)
{
  if(varcode == BB_SC)
    return( unscaled_value(pc->lp, pc->lp->sc_lobound[mipvar], pc->lp->rows+mipvar) );
  else
    return( 1.0 );
}

STATIC void update_pseudocost(BBPSrec *pc, int mipvar, int varcode, MYBOOL capupper, REAL varsol)
{
  REAL     OFsol, uplim;
  MATitem  *PS;
  MYBOOL   nonIntSelect = is_bb_rule(pc->lp, NODE_PSEUDONONINTSELECT);

  /* Establish input values;
     Note: The pseudocosts are normalized to the 0-1 range! */
  uplim = get_pseudorange(pc, mipvar, varcode);
  varsol = modf(varsol/uplim, &OFsol);

  /* Set reference value according to pseudocost mode */
  if(nonIntSelect)
    OFsol = pc->lp->bb_bounds->lastvarcus;    /* The count of MIP infeasibilities */
  else
    OFsol = pc->lp->solution[0];              /* The problem's objective function value */

  if(_isnan(varsol)) {
    pc->lp->bb_parentOF = OFsol;
    return;
  }

  /* Point to the applicable (lower or upper) bound and increment attempted update count */
  if(capupper) {
    PS = &pc->LOcost[mipvar];
  }
  else {
    PS = &pc->UPcost[mipvar];
    varsol = 1-varsol;
  }
  PS->colnr++;

  /* Make adjustment to divisor if we are using the ratio pseudo-cost approach */
  if(is_bb_rule(pc->lp, NODE_PSEUDORATIOSELECT))
    varsol *= capupper;

  /* Compute the update (consider weighting in favor of most recent) */
  mipvar = pc->updatelimit;
  if(((mipvar <= 0) || (PS->rownr < mipvar)) &&
     (fabs(varsol) > pc->lp->epspivot)) {
    /* We are interested in the change in the MIP measure (contribution to increase
       or decrease, as the case may be) and not its last value alone. */
    PS->value = PS->value*PS->rownr + (pc->lp->bb_parentOF-OFsol) / (varsol*uplim);
    PS->rownr++;
    PS->value /= PS->rownr;
    /* Check if we have enough information to restart */
    if(PS->rownr == mipvar) {
      pc->updatesfinished++;
      if(is_bb_mode(pc->lp, NODE_RESTARTMODE) &&
        (pc->updatesfinished/(2.0*pc->lp->int_vars) >
         pc->restartlimit)) {
        pc->lp->bb_break = AUTOMATIC;
        pc->restartlimit *= 2.681;  /* KE: Who can figure this one out? */
        if(pc->restartlimit > 1)
          pc->lp->bb_rule -= NODE_RESTARTMODE;
        report(pc->lp, NORMAL, "update_pseudocost: Restarting with updated pseudocosts\n");
      }
    }
  }
  pc->lp->bb_parentOF = OFsol;
}

STATIC REAL get_pseudobranchcost(BBPSrec *pc, int mipvar, MYBOOL dofloor)
{
  if(dofloor)
    return( pc->LOcost[mipvar].value );
  else
    return( pc->UPcost[mipvar].value );
}

STATIC REAL get_pseudonodecost(BBPSrec *pc, int mipvar, int vartype, REAL varsol)
{
  REAL hold, uplim;

  uplim = get_pseudorange(pc, mipvar, vartype);
  varsol = modf(varsol/uplim, &hold);
  if(_isnan(varsol))
    varsol = 0;

  hold = pc->LOcost[mipvar].value*varsol +
         pc->UPcost[mipvar].value*(1-varsol);

  return( hold*uplim );
}

STATIC int compute_theta(lprec *lp, int rownr, LREAL *theta, int isupbound, REAL HarrisScalar, MYBOOL primal)
/* The purpose of this routine is to compute the non-basic bound state / value of
   the leaving variable. Note that the incoming theta is "d" in Chvatal-terminology */
{
  int             colnr = lp->var_basic[rownr];
  register LREAL x     = lp->rhs[rownr];
  REAL            lb    = 0,  /* Put lower bound here when the fully bounded version is implemented */
                  ub    = lp->upbo[colnr],
                  eps   = lp->epsprimal;  /* Primal feasibility tolerance */

  /* Compute theta for the primal simplex */
  HarrisScalar *= eps;
  if(primal) {

    if(*theta > 0)
      x -= lb - HarrisScalar;   /* A positive number */
    else if(ub < lp->infinite)
      x -= ub + HarrisScalar;   /* A negative number */
    else {
      *theta = -lp->infinite;
      return( colnr );
    }
  }
  /* Compute theta for the dual simplex */
  else {

    if(isupbound)
      *theta = -(*theta);

    /* Current value is below or equal to its lower bound */
    if(x < lb+eps)
      x -= lb - HarrisScalar;

    /* Current value is above or equal to its upper bound */
    else if(x > ub-eps) {
      if(ub >= lp->infinite) {
        *theta = lp->infinite * my_sign(*theta);
        return( colnr );
      }
      else
        x -= ub + HarrisScalar;
    }
  }
  my_roundzero(x, lp->epsmachine);
  *theta = x / *theta;

#ifdef EnforcePositiveTheta
  /* Check if we have negative theta due to rounding or an internal error */
  if(*theta < 0) {
    if(primal && (ub == lb))
      lp->rhs[rownr] = lb;
    else
#ifdef Paranoia
    if(*theta < -eps) {
      report(lp, DETAILED, "compute_theta: Negative theta (%g) not allowed in base-0 version of lp_solve\n",
                            *theta);
    }
#endif
    *theta = 0;
  }
#endif

  return( colnr );
}

STATIC MYBOOL check_degeneracy(lprec *lp, REAL *pcol, int *degencount)
/* Check if the entering column Pi=Inv(B)*a is likely to produce improvement;
   (cfr. Istvan Maros: CTOTSM p. 233) */
{
  int  i, ndegen;
  REAL *rhs, sdegen, epsmargin = lp->epsprimal;

  sdegen = 0;
  ndegen = 0;
  rhs    = lp->rhs;
  for(i = 1; i <= lp->rows; i++) {
    rhs++;
    pcol++;
    if(fabs(*rhs) < epsmargin) {
      sdegen += *pcol;
      ndegen++;
    }
    else if(fabs((*rhs)-lp->upbo[lp->var_basic[i]]) < epsmargin) {
      sdegen -= *pcol;
      ndegen++;
    }
  }
  if(degencount != NULL)
    *degencount = ndegen;
/*  sdegen += epsmargin*ndegen; */
  return( (MYBOOL) (sdegen <= 0) );
}

STATIC MYBOOL performiteration(lprec *lp, int rownr, int varin, LREAL theta, MYBOOL primal, MYBOOL allowminit,
                               REAL *prow, int *nzprow, REAL *pcol, int *nzpcol, int *boundswaps)
{
  static int    varout;
  static REAL   pivot, epsmargin, leavingValue, leavingUB, enteringUB;
  static MYBOOL leavingToUB, enteringFromUB, enteringIsFixed, leavingIsFixed;
  MYBOOL *islower = &(lp->is_lower[varin]);
  MYBOOL minitNow = FALSE, minitStatus = ITERATE_MAJORMAJOR;
  LREAL  deltatheta = theta;

  if(userabort(lp, MSG_ITERATION))
    return( minitNow );

#ifdef Paranoia
  if(rownr > lp->rows) {
    if (lp->spx_trace)
      report(lp, IMPORTANT, "performiteration: Numeric instability encountered!\n");
    lp->spx_status = NUMFAILURE;
    return( FALSE );
  }
#endif
  varout = lp->var_basic[rownr];
#ifdef Paranoia
  if(!lp->is_lower[varout])
    report(lp, SEVERE, "performiteration: Leaving variable %d was at its upper bound at iter %.0f\n",
                        varout, (double) get_total_iter(lp));
#endif

  /* Theta is the largest change possible (strictest constraint) for the entering
     variable (Theta is Chvatal's "t", ref. Linear Programming, pages 124 and 156) */
  lp->current_iter++;

  /* Test if it is possible to do a cheap "minor iteration"; i.e. set entering
     variable to its opposite bound, without entering the basis - which is
     obviously not possible for fixed variables! */
  epsmargin = lp->epsprimal;
  enteringFromUB = !(*islower);
  enteringUB = lp->upbo[varin];
  leavingUB  = lp->upbo[varout];
  enteringIsFixed = (MYBOOL) (fabs(enteringUB) < epsmargin);
  leavingIsFixed  = (MYBOOL) (fabs(leavingUB) < epsmargin);
#if defined _PRICE_NOBOUNDFLIP
  allowminit     &= !ISMASKSET(lp->piv_strategy, PRICE_NOBOUNDFLIP);
#endif
#ifdef Paranoia
  if(enteringUB < 0)
    report(lp, SEVERE, "performiteration: Negative range for entering variable %d at iter %.0f\n",
                        varin, (double) get_total_iter(lp));
  if(leavingUB < 0)
    report(lp, SEVERE, "performiteration: Negative range for leaving variable %d at iter %.0f\n",
                        varout, (double) get_total_iter(lp));
#endif

  /* Handle batch bound swaps with the dual long-step algorithm;
     Loop over specified bound swaps; update RHS and Theta for bound swaps */
  if((boundswaps != NULL) && (boundswaps[0] > 0)) {

    int   i, boundvar;
    REAL  *hold;

    /* Allocate and initialize accumulation array */
    allocREAL(lp, &hold, lp->rows + 1, TRUE);

    /* Accumulate effective bound swaps and update flag */
    for(i = 1; i <= boundswaps[0]; i++) {
      boundvar = boundswaps[i];
      deltatheta = my_chsign(!lp->is_lower[boundvar], lp->upbo[boundvar]);
      mat_multadd(lp->matA, hold, boundvar, deltatheta);
      lp->is_lower[boundvar] = !lp->is_lower[boundvar];
    }
    lp->current_bswap += boundswaps[0];
    lp->current_iter  += boundswaps[0];

    /* Solve for bound flip update vector (note that this does not
       overwrite the stored update vector for the entering variable) */
    ftran(lp, hold, NULL, lp->epsmachine);
    if(!lp->obj_in_basis)
      hold[0] = 0; /* The correct reduced cost goes here (adjusted for bound state) ****** */

    /* Update the RHS / basic variable values and set revised thetas */
    pivot = lp->bfp_pivotRHS(lp, 1, hold);
    deltatheta = multi_enteringtheta(lp->longsteps);
    theta = deltatheta;

    FREE(hold);
  }

  /* Otherwise to traditional check for single bound swap */
  else if(allowminit &&
           !enteringIsFixed) {

/*    pivot = epsmargin; */
    pivot = lp->epsdual;
/* #define v51mode */ /* Enable this for v5.1 operation mode */
#ifdef v51mode
    if(((lp->simplex_mode & SIMPLEX_Phase1_DUAL) == 0) ||
       !is_constr_type(lp, rownr, EQ))                      /* *** DEBUG CODE KE */
#endif
    if(enteringUB - theta < -pivot) {

#ifndef v51mode
      if(fabs(enteringUB - theta) < pivot)
        minitStatus = ITERATE_MINORMAJOR;
      else
#endif
        minitStatus = ITERATE_MINORRETRY;
      minitNow    = (MYBOOL) (minitStatus != ITERATE_MAJORMAJOR);
    }
  }

  /* Process for traditional style single minor iteration */
  if(minitNow) {

   /* Set the new values (note that theta is set to always be positive) */
    theta = MIN(fabs(theta), enteringUB);

    /* Update the RHS / variable values and do bound-swap */
    pivot = lp->bfp_pivotRHS(lp, theta, NULL);
    *islower = !(*islower);

    lp->current_bswap++;

  }

  /* Process for major iteration */
  else {

    /* Update the active pricer for the current pivot */
    updatePricer(lp, rownr, varin, lp->bfp_pivotvector(lp), prow, nzprow);

    /* Update the current basic variable values */
    pivot = lp->bfp_pivotRHS(lp, theta, NULL);

    /* See if the leaving variable goes directly to its upper bound. */
    leavingValue = lp->rhs[rownr];
    leavingToUB = (MYBOOL) (leavingValue > 0.5*leavingUB);
    lp->is_lower[varout] = leavingIsFixed || !leavingToUB;

    /* Set the value of the entering varible (theta always set to be positive) */
    if(enteringFromUB) {
      lp->rhs[rownr] = enteringUB - deltatheta;
      *islower = TRUE;
    }
    else
      lp->rhs[rownr] = deltatheta;
    my_roundzero(lp->rhs[rownr], epsmargin);

   /* Update basis indeces */
    varout = set_basisvar(lp, rownr, varin);

   /* Finalize the update in preparation for next major iteration */
    lp->bfp_finishupdate(lp, enteringFromUB);

  }

  /* Show pivot tracking information, if specified */
  if((lp->verbose > NORMAL) && (MIP_count(lp) == 0) &&
     ((lp->current_iter % MAX(2, lp->rows / 10)) == 0))
    report(lp, NORMAL, "Objective value " RESULTVALUEMASK " at iter %10.0f.\n",
                       lp->rhs[0], (double) get_total_iter(lp));

#if 0
  if(verify_solution(lp, FALSE, my_if(minitNow, "MINOR", "MAJOR")) >= 0) {
    if(minitNow)
      pivot = get_obj_active(lp, varin);
    else
      pivot = get_obj_active(lp, varout);
  }
#endif
#if 0
  if((lp->longsteps != NULL) && (boundswaps[0] > 0) && lp->longsteps->objcheck &&
    ((pivot = fabs(my_reldiff(lp->rhs[0], lp->longsteps->obj_last))) > lp->epssolution)) {
    report(lp, IMPORTANT, "performiteration: Objective value gap %8.6f found at iter %6.0f (%d bound flips, %d)\n",
                          pivot, (double) get_total_iter(lp), boundswaps[0], enteringFromUB);
  }
#endif

  if(lp->spx_trace) {
    if(minitNow)
      report(lp, NORMAL, "I:%5.0f - minor - %5d ignored,          %5d flips  from %s with THETA=%g and OBJ=%g\n",
                         (double) get_total_iter(lp), varout, varin, (enteringFromUB ? "UPPER" : "LOWER"), theta, lp->rhs[0]);
    else
      report(lp, NORMAL, "I:%5.0f - MAJOR - %5d leaves to %s,  %5d enters from %s with THETA=%g and OBJ=%g\n",
                         (double) get_total_iter(lp), varout, (leavingToUB    ? "UPPER" : "LOWER"),
                                           varin,  (enteringFromUB ? "UPPER" : "LOWER"), theta, lp->rhs[0]);
    if(minitNow) {
      if(!lp->is_lower[varin])
        report(lp, DETAILED,
        "performiteration: Variable %d changed to its lower bound at iter %.0f (from %g)\n",
        varin, (double) get_total_iter(lp), enteringUB);
      else
        report(lp, DETAILED,
        "performiteration: Variable %d changed to its upper bound at iter %.0f (to %g)\n",
        varin, (double) get_total_iter(lp), enteringUB);
    }
    else
      report(lp, NORMAL,
          "performiteration: Variable %d entered basis at iter %.0f at " RESULTVALUEMASK "\n",
          varin, (double) get_total_iter(lp), lp->rhs[rownr]);
    if(!primal) {
      pivot = compute_feasibilitygap(lp, (MYBOOL)!primal, TRUE);
      report(lp, NORMAL, "performiteration: Feasibility gap at iter %.0f is " RESULTVALUEMASK "\n",
                         (double) get_total_iter(lp), pivot);
    }
    else
      report(lp, NORMAL,
          "performiteration: Current objective function value at iter %.0f is " RESULTVALUEMASK "\n",
          (double) get_total_iter(lp), lp->rhs[0]);
  }

  return( minitStatus );

} /* performiteration */

STATIC REAL get_refactfrequency(lprec *lp, MYBOOL final)
{
  COUNTER iters;
  int     refacts;

  /* Get numerator and divisor information */
  iters   = (lp->total_iter+lp->current_iter) - (lp->total_bswap+lp->current_bswap);
  refacts = lp->bfp_refactcount(lp, BFP_STAT_REFACT_TOTAL);

  /* Return frequency for different cases:
      1) Actual frequency in case final statistic is desired
      2) Dummy if we are in a B&B process
      3) Frequency with added initialization offsets which
         are diluted in course of the solution process */
  if(final)
    return( (REAL) (iters) / MAX(1,refacts) );
  else if(lp->bb_totalnodes > 0)
    return( (REAL) lp->bfp_pivotmax(lp) );
  else
    return( (REAL) (lp->bfp_pivotmax(lp)+iters) / (1+refacts) );
}

#if 0
/* INLINE */ MYBOOL is_fixedvar(lprec *lp, int variable)
{
  if((lp->bb_bounds != NULL && lp->bb_bounds->UBzerobased) || (variable <= lp->rows))
    return( (MYBOOL) (lp->upbo[variable] < lp->epsprimal) );
  else
    return( (MYBOOL) (lp->upbo[variable]-lp->lowbo[variable] < lp->epsprimal) );
} /* is_fixedvar */
#else
MYBOOL is_fixedvar(lprec *lp, int varnr)
{
  if(lp->bb_bounds == NULL) {
    if(varnr <= lp->rows)
      return( (MYBOOL) (lp->orig_upbo[varnr] < lp->epsmachine) );
    else
      return( (MYBOOL) (lp->orig_upbo[varnr]-lp->orig_lowbo[varnr] < lp->epsmachine) );
  }
  else if((varnr <= lp->rows) || (lp->bb_bounds->UBzerobased == TRUE))
    return( (MYBOOL) (lp->upbo[varnr] < lp->epsvalue) );
  else
    return( (MYBOOL) (lp->upbo[varnr]-lp->lowbo[varnr] < lp->epsvalue) );
}
#endif

STATIC MYBOOL solution_is_int(lprec *lp, int index, MYBOOL checkfixed)
{
#if 1
  return( (MYBOOL) (isINT(lp, lp->solution[index]) && (!checkfixed || is_fixedvar(lp, index))) );
#else
  if(isINT(lp, lp->solution[index])) {
    if(checkfixed)
      return(is_fixedvar(lp, index));
    else
      return(TRUE);
  }
  return(FALSE);
#endif
} /* solution_is_int */


MYBOOL __WINAPI set_multiprice(lprec *lp, int multiblockdiv)
{
  /* See if we are resetting multiply priced column structures */
  if(multiblockdiv != lp->multiblockdiv) {
    if(multiblockdiv < 1)
      multiblockdiv = 1;
    lp->multiblockdiv = multiblockdiv;
    multi_free(&(lp->multivars));
  }
  return( TRUE );
}

int __WINAPI get_multiprice(lprec *lp, MYBOOL getabssize)
{
  if((lp->multivars == NULL) || (lp->multivars->used == 0))
    return( 0 );
  if(getabssize)
    return( lp->multivars->size );
  else
    return( lp->multiblockdiv );
}

MYBOOL __WINAPI set_partialprice(lprec *lp, int blockcount, int *blockstart, MYBOOL isrow)
{
  int        ne, i, items;
  partialrec **blockdata;

  /* Determine partial target (rows or columns) */
  if(isrow)
    blockdata = &(lp->rowblocks);
  else
    blockdata = &(lp->colblocks);

  /* See if we are resetting partial blocks */
  ne = 0;
  items = IF(isrow, lp->rows, lp->columns);
  if(blockcount == 1)
    partial_freeBlocks(blockdata);

  /* Set a default block count if this was not specified */
  else if(blockcount <= 0) {
    blockstart = NULL;
    if(items < DEF_PARTIALBLOCKS*DEF_PARTIALBLOCKS)
      blockcount = items / DEF_PARTIALBLOCKS + 1;
    else
      blockcount = DEF_PARTIALBLOCKS;
    ne = items / blockcount;
    if(ne * blockcount < items)
      ne++;
  }

  /* Fill partial block arrays;
     Note: These will be modified during preprocess to reflect
           presolved columns and the handling of slack variables. */
  if(blockcount > 1) {
    MYBOOL     isNew = (MYBOOL) (*blockdata == NULL);

    /* Provide for extra block with slack variables in the column mode */
    i = 0;
    if(!isrow)
      i++;

    /* (Re)-allocate memory */
    if(isNew)
      *blockdata = partial_createBlocks(lp, isrow);
    allocINT(lp, &((*blockdata)->blockend), blockcount+i+1, AUTOMATIC);
    allocINT(lp, &((*blockdata)->blockpos), blockcount+i+1, AUTOMATIC);

    /* Copy the user-provided block start positions */
    if(blockstart != NULL) {
      MEMCOPY((*blockdata)->blockend+i, blockstart, blockcount+i+1);
      if(!isrow) {
        blockcount++;
        (*blockdata)->blockend[0] = 1;
        for(i = 1; i < blockcount; i++)
          (*blockdata)->blockend[i] += lp->rows;
      }
    }

    /* Fill the block ending positions if they were not specified */
    else {
      (*blockdata)->blockend[0] = 1;
      (*blockdata)->blockpos[0] = 1;
      if(ne == 0) {
        ne = items / blockcount;
        /* Increase the block size if we have a fractional value */
        while(ne * blockcount < items)
          ne++;
      }
      i = 1;
      if(!isrow) {
        (*blockdata)->blockend[i] = (*blockdata)->blockend[i-1]+lp->rows;
        blockcount++;
        i++;
        items += lp->rows;
      }
      for(; i < blockcount; i++)
        (*blockdata)->blockend[i] = (*blockdata)->blockend[i-1]+ne;

      /* Let the last block handle the "residual" */
      (*blockdata)->blockend[blockcount] = items+1;
    }

    /* Fill starting positions (used in multiple partial pricing) */
    for(i = 1; i <= blockcount; i++)
      (*blockdata)->blockpos[i] = (*blockdata)->blockend[i-1];

  }

  /* Update block count */
  (*blockdata)->blockcount = blockcount;


  return( TRUE );
} /* set_partialprice */

void __WINAPI get_partialprice(lprec *lp, int *blockcount, int *blockstart, MYBOOL isrow)
{
  partialrec *blockdata;

  /* Determine partial target (rows or columns) */
  if(isrow)
    blockdata = lp->rowblocks;
  else
    blockdata = lp->colblocks;

  *blockcount = partial_countBlocks(lp, isrow);
  if((blockdata != NULL) && (blockstart != NULL)) {
    int i = 0, k = *blockcount;
    if(!isrow)
      i++;
    MEMCOPY(blockstart, blockdata->blockend + i, k - i);
    if(!isrow) {
      k -= i;
      for(i = 0; i < k; i++)
        blockstart[i] -= lp->rows;
    }
  }
}


/* Solution-related functions */
STATIC MYBOOL bb_better(lprec *lp, int target, int mode)
/* Must handle four modes (logic assumes Min!):
      -----|--.--|----->
   1  ++++++-----------  LHS exclusive test point is better
   2  +++++++++--------  LHS inclusive
   3  ++++++-----++++++  LHS+RHS exclusive
   4  --------+++++++++  RHS inclusive
   5  -----------++++++  RHS exclusive
*/
{
  REAL   epsvalue, offset = lp->epsprimal,
         refvalue = lp->infinite, testvalue = lp->solution[0];
  MYBOOL ismax = is_maxim(lp),
         relgap = is_action(mode, OF_TEST_RELGAP),
         fcast  = is_action(target, OF_PROJECTED),
         delta  = is_action(target, OF_DELTA);

  if(relgap) {
    epsvalue = lp->mip_relgap;
    clear_action(&mode, OF_TEST_RELGAP);
  }
  else
    epsvalue = lp->mip_absgap;

  if(delta)
    clear_action(&target, OF_DELTA);
  if(fcast)
    clear_action(&target, OF_PROJECTED);
#ifdef Paranoia
  if((mode < OF_TEST_BT) || (mode > OF_TEST_WT))
    report(lp, SEVERE, "bb_better: Passed invalid mode '%d'\n", mode);
#endif

  switch(target) {
    case OF_RELAXED:   refvalue = lp->real_solution;
                       break;
    case OF_INCUMBENT: refvalue = lp->best_solution[0];
                       break;
    case OF_WORKING:  refvalue = my_chsign(!ismax, lp->bb_workOF /* unscaled_value(lp, lp->bb_workOF, 0) */ );
                       if(fcast)
                         testvalue = my_chsign(!ismax, lp->longsteps->obj_last) - epsvalue;
                       else
                         testvalue = my_chsign(!ismax, lp->rhs[0] /* unscaled_value(lp, lp->rhs[0], 0) */);
                       break;
    case OF_USERBREAK: refvalue = lp->bb_breakOF;
                       break;
    case OF_HEURISTIC: refvalue = lp->bb_heuristicOF;
                       break;
    case OF_DUALLIMIT: refvalue = lp->bb_limitOF;
                       break;
    default         :  report(lp, SEVERE, "bb_better: Passed invalid test target '%d'\n", target);
                       return( FALSE );
  }

  /* Adjust the test value for the desired acceptability window */
  if(delta) {
    SETMAX(epsvalue, lp->bb_deltaOF - epsvalue);
  }
  else
    epsvalue = my_chsign(target >= OF_USERBREAK, epsvalue); /* *** This seems Ok, but should be verified */
  testvalue += my_chsign(ismax, epsvalue);

  /* Compute the raw test value */
  if(relgap)
    testvalue = my_reldiff(testvalue, refvalue);
  else
    testvalue -= refvalue;

  /* Make test value adjustment based on the selected option */
  if(mode == OF_TEST_NE)
    relgap = (MYBOOL) (fabs(testvalue) >= offset);
  else {
    testvalue = my_chsign(mode > OF_TEST_NE, testvalue);
    testvalue = my_chsign(ismax, testvalue);
    relgap = (MYBOOL) (testvalue < offset);
  }
  return( relgap );
}

STATIC void construct_solution(lprec *lp, REAL *target)
{
  int     i, j, basi;
  REAL    f, epsvalue = lp->epsprimal;
  REAL    *solution;
  REAL    *value;
  int     *rownr;
  MATrec  *mat = lp->matA;

  if(target == NULL)
    solution = lp->solution;
  else
    solution = target;

  /* Initialize OF and slack variables. */
  for(i = 0; i <= lp->rows; i++) {
#ifdef LegacySlackDefinition
    if(i == 0)
      f = unscaled_value(lp, -lp->orig_rhs[i], i);
    else {
      j = lp->presolve_undo->var_to_orig[i];
      if(j > 0) {
        f = lp->presolve_undo->fixed_rhs[j];
        f = unscaled_value(lp, f, i);
      }
      else
        f = 0;
    }
#else
    f = lp->orig_rhs[i];
    if((i > 0) && !lp->is_basic[i] && !lp->is_lower[i])
#ifdef SlackInitMinusInf
      f -= my_chsign(is_chsign(lp, i), fabs(lp->upbo[i]));
#else
      f -= my_chsign(is_chsign(lp, i), fabs(lp->lowbo[i] + lp->upbo[i]));
#endif
    f = unscaled_value(lp, -f, i);
#endif
    solution[i] = f;
  }

  /* Initialize user variables to their lower bounds. */
  for(i = lp->rows+1; i <= lp->sum; i++)
    solution[i] = lp->lowbo[i];

  /* Add values of user basic variables. */
  for(i = 1; i <= lp->rows; i++) {
    basi = lp->var_basic[i];
    if(basi > lp->rows) {
      solution[basi] += lp->rhs[i];
    }
  }

  /* 1. Adjust non-basic variables at their upper bounds,
     2. Unscale all user variables,
     3. Optionally do precision management. */
  for(i = lp->rows + 1; i <= lp->sum; i++) {
    if(!lp->is_basic[i] && !lp->is_lower[i])
      solution[i] += lp->upbo[i];
    solution[i] = unscaled_value(lp, solution[i], i);
#ifdef xImproveSolutionPrecision
    if(is_int(lp, i-lp->rows))
      solution[i] = restoreINT(solution[i], lp->epsint);
    else
      solution[i] = restoreINT(solution[i], lp->epsprimal);
#endif
  }

  /* Compute the OF and slack values "in extentio" */
  for(j = 1; j <= lp->columns; j++) {
    f = solution[lp->rows + j];
    if(f != 0) {
      solution[0] += f * unscaled_mat(lp, lp->orig_obj[j], 0, j);
      i = mat->col_end[j-1];
      basi = mat->col_end[j];
      rownr = &COL_MAT_ROWNR(i);
      value = &COL_MAT_VALUE(i);
      for(; i < basi;
          i++, rownr += matRowColStep, value += matValueStep)
        solution[*rownr] += f * unscaled_mat(lp, *value, *rownr, j);
    }
  }

  /* Do slack precision management and sign reversal if necessary */
  for(i = 0; i <= lp->rows; i++) {
#ifdef ImproveSolutionPrecision
    my_roundzero(solution[i], epsvalue);
#endif
    if(is_chsign(lp, i))
      solution[i] = my_flipsign(solution[i]);
  }

 /* Record the best real-valued solution and compute a simple MIP solution limit */
  if(target == NULL) {
    if(is_infinite(lp, lp->real_solution)) {
      lp->bb_workOF = lp->rhs[0];
      lp->real_solution = solution[0];
      if(is_infinite(lp, lp->bb_limitOF))
        lp->bb_limitOF = lp->real_solution;
      else {
        if(is_maxim(lp)) {
          SETMIN(lp->bb_limitOF, lp->real_solution);
        }
        else {
          SETMAX(lp->bb_limitOF, lp->real_solution);
        }
      }

      /* Do MIP-related tests and computations */
      if((lp->int_vars > 0) && mat_validate(lp->matA) && !lp->wasPresolved) {
        REAL fixedOF = unscaled_value(lp, lp->orig_rhs[0], 0);

        /* Check if we have an all-integer OF */
        basi = lp->columns;
        for(j = 1; j <= basi; j++) {
          f = fabs(get_mat(lp, 0, j)) + lp->epsint/2;
          f = fmod(f, 1);
          if(!is_int(lp, j) || (f > lp->epsint))
            break;
        }

        /* If so, we can round up the fractional OF */
        if(j > basi) {
          f = my_chsign(is_maxim(lp), lp->real_solution) + fixedOF;
          f = floor(f+(1-epsvalue));
          lp->bb_limitOF = my_chsign(is_maxim(lp), f - fixedOF);
        }
      }
      /* Check that a user limit on the OF is feasible */
      if((lp->int_vars > 0) &&
         (my_chsign(is_maxim(lp), my_reldiff(lp->best_solution[0],lp->bb_limitOF)) < -epsvalue)) {
        lp->spx_status = INFEASIBLE;
        lp->bb_break = TRUE;
      }
    }
  }

} /* construct_solution */

STATIC int check_solution(lprec *lp, int  lastcolumn, REAL *solution,
                          REAL *upbo, REAL *lowbo, REAL tolerance)
{
/*#define UseMaxValueInCheck*/
  MYBOOL isSC;
  REAL   test, value, hold, diff, maxdiff = 0.0, maxerr = 0.0, *matValue,
#ifdef UseMaxValueInCheck
         *maxvalue = NULL,
#else
         *plusum = NULL, *negsum = NULL;
#endif
  int    i,j,n, errlevel = IMPORTANT, errlimit = 10, *matRownr, *matColnr;
  MATrec *mat = lp->matA;

  report(lp, NORMAL, " \n");
  if(MIP_count(lp) > 0)
    report(lp, NORMAL, "%s solution  " RESULTVALUEMASK " after %10.0f iter, %9.0f nodes (gap %.1f%%).\n",
                       my_if(lp->bb_break && bb_better(lp, OF_RELAXED, OF_TEST_NE), "Subopt.", "Optimal"),
                       solution[0], (double) lp->total_iter, (double) lp->bb_totalnodes,
                       100.0*fabs(my_reldiff(lp->solution[0], lp->bb_limitOF)));
  else
    report(lp, NORMAL, "Optimal solution  " RESULTVALUEMASK " after %10.0f iter.\n",
                       solution[0], (double) lp->total_iter);

 /* Find the signed sums and the largest absolute product in the matrix (exclude the OF for speed) */
#ifdef UseMaxValueInCheck
  allocREAL(lp, &maxvalue, lp->rows + 1, FALSE);
  for(i = 0; i <= lp->rows; i++)
    maxvalue[i] = fabs(get_rh(lp, i));
#else
  allocREAL(lp, &plusum, lp->rows + 1, TRUE);
  allocREAL(lp, &negsum, lp->rows + 1, TRUE);
#endif
  n = get_nonzeros(lp);
  matRownr = &COL_MAT_ROWNR(0);
  matColnr = &COL_MAT_COLNR(0);
  matValue = &COL_MAT_VALUE(0);
  for(i = 0; i < n; i++, matRownr += matRowColStep,
                         matColnr += matRowColStep,
                         matValue += matValueStep) {
    test = unscaled_mat(lp, *matValue, *matRownr, *matColnr);
    test *= solution[lp->rows + (*matColnr)];
#ifdef UseMaxValueInCheck
    test = fabs(test);
    if(test > maxvalue[*matRownr])
      maxvalue[*matRownr] = test;
#else
    if(test > 0)
      plusum[*matRownr] += test;
    else
      negsum[*matRownr] += test;
#endif
  }


 /* Check if solution values are within the bounds; allowing a margin for numeric errors */
  n = 0;
  for(i = lp->rows + 1; i <= lp->rows+lastcolumn; i++) {

    value = solution[i];

    /* Check for case where we are testing an intermediate solution
       (variables shifted to the origin) */
    if(lowbo == NULL)
      test = 0;
    else
      test = unscaled_value(lp, lowbo[i], i);

    isSC = is_semicont(lp, i - lp->rows);
    diff = my_reldiff(value, test);
    if(diff < 0) {
      if(isSC && (value < test/2))
        test = 0;
      SETMAX(maxerr, fabs(value-test));
      SETMAX(maxdiff, fabs(diff));
    }
    if((diff < -tolerance) && !isSC)  {
      if(n < errlimit)
      report(lp, errlevel,
        "check_solution: Variable   %s = " RESULTVALUEMASK " is below its lower bound " RESULTVALUEMASK "\n",
         get_col_name(lp, i-lp->rows), value, test);
      n++;
    }

    test = unscaled_value(lp, upbo[i], i);
    diff = my_reldiff(value, test);
    if(diff > 0) {
      SETMAX(maxerr, fabs(value-test));
      SETMAX(maxdiff, fabs(diff));
    }
    if(diff > tolerance) {
      if(n < errlimit)
      report(lp, errlevel,
         "check_solution: Variable   %s = " RESULTVALUEMASK " is above its upper bound " RESULTVALUEMASK "\n",
         get_col_name(lp, i-lp->rows), value, test);
      n++;
    }
  }

 /* Check if constraint values are within the bounds; allowing a margin for numeric errors */
  for(i = 1; i <= lp->rows; i++) {

    test = lp->orig_rhs[i];
    if(is_infinite(lp, test))
      continue;

#ifdef LegacySlackDefinition
    j = lp->presolve_undo->var_to_orig[i];
    if(j != 0) {
      if(is_infinite(lp, lp->presolve_undo->fixed_rhs[j]))
        continue;
      test += lp->presolve_undo->fixed_rhs[j];
    }
#endif

    if(is_chsign(lp, i)) {
      test = my_flipsign(test);
      test += fabs(upbo[i]);
    }
    value = solution[i];
    test = unscaled_value(lp, test, i);
#ifndef LegacySlackDefinition
    value += test;
#endif
/*    diff = my_reldiff(value, test); */
#ifdef UseMaxValueInCheck
    hold = maxvalue[i];
#else
    hold = plusum[i] - negsum[i];
#endif
    if(hold < lp->epsvalue)
      hold = 1;
    diff = my_reldiff((value+1)/hold, (test+1)/hold);
    if(diff > 0) {
      SETMAX(maxerr, fabs(value-test));
      SETMAX(maxdiff, fabs(diff));
    }
    if(diff > tolerance) {
      if(n < errlimit)
      report(lp, errlevel,
        "check_solution: Constraint %s = " RESULTVALUEMASK " is above its %s " RESULTVALUEMASK "\n",
        get_row_name(lp, i), value,
        (is_constr_type(lp, i, EQ) ? "equality of" : "upper bound"), test);
      n++;
    }

    test = lp->orig_rhs[i];
#ifdef LegacySlackDefinition
    j = lp->presolve_undo->var_to_orig[i];
    if(j != 0) {
      if(is_infinite(lp, lp->presolve_undo->fixed_rhs[j]))
        continue;
      test += lp->presolve_undo->fixed_rhs[j];
    }
#endif

    value = solution[i];
    if(is_chsign(lp, i))
      test = my_flipsign(test);
    else {
      if(is_infinite(lp, upbo[i]))
        continue;
      test -= fabs(upbo[i]);
#ifndef LegacySlackDefinition
      value = fabs(upbo[i]) - value;
#endif
    }
    test = unscaled_value(lp, test, i);
#ifndef LegacySlackDefinition
    value += test;
#endif
/*    diff = my_reldiff(value, test); */
#ifdef UseMaxValueInCheck
    hold = maxvalue[i];
#else
    hold = plusum[i] - negsum[i];
#endif
    if(hold < lp->epsvalue)
      hold = 1;
    diff = my_reldiff((value+1)/hold, (test+1)/hold);
    if(diff < 0) {
      SETMAX(maxerr, fabs(value-test));
      SETMAX(maxdiff, fabs(diff));
    }
    if(diff < -tolerance) {
      if(n < errlimit)
      report(lp, errlevel,
        "check_solution: Constraint %s = " RESULTVALUEMASK " is below its %s " RESULTVALUEMASK "\n",
        get_row_name(lp, i), value,
        (is_constr_type(lp, i, EQ) ? "equality of" : "lower bound"), test);
      n++;
    }
  }

#ifdef UseMaxValueInCheck
  FREE(maxvalue);
#else
  FREE(plusum);
  FREE(negsum);
#endif

  if(n > 0) {
    report(lp, IMPORTANT, "\nSeriously low accuracy found ||*|| = %g (rel. error %g)\n",
               maxerr, maxdiff);
    return(NUMFAILURE);
  }
  else {
    if(maxerr > 1.0e-7)
      report(lp, NORMAL, "\nMarginal numeric accuracy ||*|| = %g (rel. error %g)\n",
                 maxerr, maxdiff);
    else if(maxerr > 1.0e-9)
      report(lp, NORMAL, "\nReasonable numeric accuracy ||*|| = %g (rel. error %g)\n",
                 maxerr, maxdiff);
    else if(maxerr > 1.0e11)
      report(lp, NORMAL, "\nVery good numeric accuracy ||*|| = %g\n", maxerr);
    else
      report(lp, NORMAL, "\nExcellent numeric accuracy ||*|| = %g\n", maxerr);

    return(OPTIMAL);
  }

} /* check_solution */

STATIC void transfer_solution_var(lprec *lp, int uservar)
{
  if(lp->varmap_locked && (MYBOOL) ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE)) {
    uservar += lp->rows;
    lp->full_solution[lp->presolve_undo->orig_rows +
                      lp->presolve_undo->var_to_orig[uservar]] = lp->best_solution[uservar];
  }
}
STATIC void transfer_solution(lprec *lp, MYBOOL dofinal)
{
  int i, ii;

  MEMCOPY(lp->best_solution, lp->solution, lp->sum + 1);

  /* Round integer solution values to actual integers */
  if(is_integerscaling(lp) && (lp->int_vars > 0))
    for(i = 1; i <= lp->columns; i++) {
      if(is_int(lp, i)) {
        ii = lp->rows + i;
        lp->best_solution[ii] = floor(lp->best_solution[ii] + 0.5);
      }
    }

  /* Transfer to full solution vector in the case of presolved eliminations */
  if(dofinal && lp->varmap_locked &&
     (MYBOOL) ((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE)) {
    presolveundorec *psundo = lp->presolve_undo;

    lp->full_solution[0] = lp->best_solution[0];
    for(i = 1; i <= lp->rows; i++) {
      ii = psundo->var_to_orig[i];
#ifdef Paranoia
      if((ii < 0) || (ii > lp->presolve_undo->orig_rows))
        report(lp, SEVERE, "transfer_solution: Invalid mapping of row index %d to original index '%d'\n",
                            i, ii);
#endif
      lp->full_solution[ii] = lp->best_solution[i];
    }
    for(i = 1; i <= lp->columns; i++) {
      ii = psundo->var_to_orig[lp->rows+i];
#ifdef Paranoia
      if((ii < 0) || (ii > lp->presolve_undo->orig_columns))
        report(lp, SEVERE, "transfer_solution: Invalid mapping of column index %d to original index '%d'\n",
                            i, ii);
#endif
      lp->full_solution[psundo->orig_rows+ii] = lp->best_solution[lp->rows+i];
    }
  }

}

STATIC MYBOOL construct_duals(lprec *lp)
{
  int  i, n, *coltarget;
  REAL scale0, value, dualOF;

  if(lp->duals != NULL)
    free_duals(lp);

  if(is_action(lp->spx_action, ACTION_REBASE) ||
     is_action(lp->spx_action, ACTION_REINVERT) || (!lp->basis_valid) ||
     !allocREAL(lp, &(lp->duals), lp->sum + 1, AUTOMATIC))
    return(FALSE);

  /* Initialize */
  coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->columns+1, sizeof(*coltarget));
  if(!get_colIndexA(lp, SCAN_USERVARS+USE_NONBASICVARS, coltarget, FALSE)) {
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
    return(FALSE);
  }
  bsolve(lp, 0, lp->duals, NULL, lp->epsmachine*DOUBLEROUND, 1.0);
  prod_xA(lp, coltarget, lp->duals, NULL, lp->epsmachine, 1.0,
                         lp->duals, NULL, MAT_ROUNDDEFAULT | MAT_ROUNDRC);
  mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);


  /* The (Lagrangean) dual values are the reduced costs of the primal slacks;
     when the slack is at its upper bound, change the sign. */
  n = lp->rows;
  for(i = 1; i <= n; i++) {
    if(lp->is_basic[i])
      lp->duals[i] = 0;
    /* Added a test if variable is different from 0 because sometime you get -0 and this
       is different from 0 on for example INTEL processors (ie 0 != -0 on INTEL !) PN */
    else if((is_chsign(lp, 0) == is_chsign(lp, i)) && lp->duals[i])
      lp->duals[i] = my_flipsign(lp->duals[i]);
  }
  if(is_maxim(lp)) {
    n = lp->sum;
    for(i = lp->rows + 1; i <= n; i++)
      lp->duals[i] = my_flipsign(lp->duals[i]);
  }

  /* If we presolved, then reconstruct the duals */
  n = lp->presolve_undo->orig_sum;
  if(((lp->do_presolve & PRESOLVE_LASTMASKMODE) != PRESOLVE_NONE) &&
      allocREAL(lp, &(lp->full_duals), n + 1, TRUE)) {
    int ix, ii = lp->presolve_undo->orig_rows;

    n = lp->sum;
    for(ix = 1; ix <= n; ix++) {
      i = lp->presolve_undo->var_to_orig[ix];
      if(ix > lp->rows)
        i += ii;
#ifdef Paranoia
      /* Check for index out of range due to presolve */
      if(i > lp->presolve_undo->orig_sum)
        report(lp, SEVERE, "construct_duals: Invalid presolve variable mapping found\n");
#endif
      lp->full_duals[i] = lp->duals[ix];
    }
    presolve_rebuildUndo(lp, FALSE);
  }

  /* Calculate the dual OF and do scaling adjustments to the duals */
  if(lp->scaling_used)
    scale0 = lp->scalars[0];
  else
    scale0 = 1;
  dualOF = my_chsign(is_maxim(lp), lp->orig_rhs[0]) / scale0;
  for(i = 1; i <= lp->sum; i++) {
    value = scaled_value(lp, lp->duals[i] / scale0, i);
    my_roundzero(value, lp->epsprimal);
    lp->duals[i] = value;
    if(i <= lp->rows)
      dualOF += value * lp->solution[i];
  }

#if 0
  /* See if we can make use of the dual OF;
     note that we do not currently adjust properly for presolve */
  if(lp->rows == lp->presolve_undo->orig_rows)
  if(MIP_count(lp) > 0) {
    if(is_maxim(lp)) {
      SETMIN(lp->bb_limitOF, dualOF);
    }
    else {
      SETMAX(lp->bb_limitOF, dualOF);
    }
  }
  else if(fabs(my_reldiff(dualOF, lp->solution[0])) > lp->epssolution)
    report(lp, IMPORTANT, "calculate_duals: Check for possible suboptimal solution!\n");
#endif

  return(TRUE);
} /* construct_duals */

/* Calculate sensitivity duals */
STATIC MYBOOL construct_sensitivity_duals(lprec *lp)
{
  int  k,varnr, ok = TRUE;
  int  *workINT = NULL;
  REAL *pcol,a,infinite,epsvalue,from,till,objfromvalue;

  /* one column of the matrix */
  FREE(lp->objfromvalue);
  FREE(lp->dualsfrom);
  FREE(lp->dualstill);
  if(!allocREAL(lp, &pcol, lp->rows + 1, TRUE) ||
     !allocREAL(lp, &lp->objfromvalue, lp->columns + 1, AUTOMATIC) ||
     !allocREAL(lp, &lp->dualsfrom, lp->sum + 1, AUTOMATIC) ||
     !allocREAL(lp, &lp->dualstill, lp->sum + 1, AUTOMATIC)) {
    FREE(pcol);
    FREE(lp->objfromvalue);
    FREE(lp->dualsfrom);
    FREE(lp->dualstill);
    ok = FALSE;
  }
  else {
    infinite=lp->infinite;
    epsvalue=lp->epsmachine;
    for(varnr=1; varnr<=lp->sum; varnr++) {
      from=infinite;
      till=infinite;
      objfromvalue=infinite;
      if (!lp->is_basic[varnr]) {
        if (!fsolve(lp, varnr, pcol, workINT, epsvalue, 1.0, FALSE)) {  /* construct one column of the tableau */
          ok = FALSE;
          break;
        }
        /* Search for the rows(s) which first result in further iterations */
        for (k=1; k<=lp->rows; k++) {
          if (fabs(pcol[k])>epsvalue) {
            a = unscaled_value(lp, lp->rhs[k]/pcol[k], varnr);
            if((varnr > lp->rows) && (fabs(lp->solution[varnr]) <= epsvalue) && (a < objfromvalue) && (a >= lp->lowbo[varnr]))
              objfromvalue = a;
            if ((a<=0.0) && (pcol[k]<0.0) && (-a<from)) from=my_flipsign(a);
            if ((a>=0.0) && (pcol[k]>0.0) && ( a<till)) till= a;
            if (lp->upbo[lp->var_basic[k]] < infinite) {
              a = (REAL) ((lp->rhs[k]-lp->upbo[lp->var_basic[k]])/pcol[k]);
              a = unscaled_value(lp, a, varnr);
              if((varnr > lp->rows) && (fabs(lp->solution[varnr]) <= epsvalue) && (a < objfromvalue) && (a >= lp->lowbo[varnr]))
                objfromvalue = a;
              if ((a<=0.0) && (pcol[k]>0.0) && (-a<from)) from=my_flipsign(a);
              if ((a>=0.0) && (pcol[k]<0.0) && ( a<till)) till= a;
            }
          }
        }

        if (!lp->is_lower[varnr]) {
          a=from;
          from=till;
          till=a;
        }
        if ((varnr<=lp->rows) && (!is_chsign(lp, varnr))) {
          a=from;
          from=till;
          till=a;
        }
      }

      if (from!=infinite)
        lp->dualsfrom[varnr]=lp->solution[varnr]-from;
      else
        lp->dualsfrom[varnr]=-infinite;
      if (till!=infinite)
        lp->dualstill[varnr]=lp->solution[varnr]+till;
      else
        lp->dualstill[varnr]=infinite;

      if (varnr > lp->rows) {
        if (objfromvalue != infinite) {
          if (!lp->is_lower[varnr])
            objfromvalue = lp->upbo[varnr] - objfromvalue;
          if ((lp->upbo[varnr] < infinite) && (objfromvalue > lp->upbo[varnr]))
            objfromvalue = lp->upbo[varnr];
          objfromvalue += lp->lowbo[varnr];
        }
        else
          objfromvalue = -infinite;
        lp->objfromvalue[varnr - lp->rows] = objfromvalue;
      }

    }
    FREE(pcol);
  }
  return((MYBOOL) ok);
} /* construct_sensitivity_duals */

/* Calculate sensitivity objective function */
STATIC MYBOOL construct_sensitivity_obj(lprec *lp)
{
  int  i, l, varnr, row_nr, ok = TRUE;
  REAL *OrigObj = NULL, *drow = NULL, *prow = NULL,
       sign, a, min1, min2, infinite, epsvalue, from, till;

  /* objective function */
  FREE(lp->objfrom);
  FREE(lp->objtill);
  if(!allocREAL(lp, &drow, lp->sum + 1, TRUE) ||
     !allocREAL(lp, &OrigObj, lp->columns + 1, FALSE) ||
     !allocREAL(lp, &prow, lp->sum + 1, TRUE) ||
     !allocREAL(lp, &lp->objfrom, lp->columns + 1, AUTOMATIC) ||
     !allocREAL(lp, &lp->objtill, lp->columns + 1, AUTOMATIC)) {
Abandon:
    FREE(drow);
    FREE(OrigObj);
    FREE(prow);
    FREE(lp->objfrom);
    FREE(lp->objtill);
    ok = FALSE;
  }
  else {
    int *coltarget;

    infinite=lp->infinite;
    epsvalue=lp->epsmachine;

    coltarget = (int *) mempool_obtainVector(lp->workarrays, lp->columns+1, sizeof(*coltarget));
    if(!get_colIndexA(lp, SCAN_USERVARS+USE_NONBASICVARS, coltarget, FALSE)) {
      mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
      goto Abandon;
    }
    bsolve(lp, 0, drow, NULL, epsvalue*DOUBLEROUND, 1.0);
    prod_xA(lp, coltarget, drow, NULL, epsvalue, 1.0,
                           drow, NULL, MAT_ROUNDDEFAULT | MAT_ROUNDRC);

    /* original (unscaled) objective function */
    get_row(lp, 0, OrigObj);
    for(i = 1; i <= lp->columns; i++) {
      from=-infinite;
      till= infinite;
      varnr = lp->rows + i;
      if(!lp->is_basic[varnr]) {
      /* only the coeff of the objective function of column i changes. */
        a = unscaled_mat(lp, drow[varnr], 0, i);
        if(is_maxim(lp))
          a = -a;
        if (lp->upbo[varnr] == 0.0)
          /* ignore, because this case doesn't results in further iterations */ ;
        else if((lp->is_lower[varnr] != 0) == (is_maxim(lp) == FALSE))
          from = OrigObj[i] - a; /* less than this value gives further iterations */
        else
          till = OrigObj[i] - a; /* bigger than this value gives further iterations */
      }
      else {
      /* all the coeff of the objective function change. Search the minimal change needed for further iterations */
        for(row_nr=1;
            (row_nr<=lp->rows) && (lp->var_basic[row_nr]!=varnr); row_nr++)
          /* Search on which row the variable exists in the basis */ ;
        if(row_nr<=lp->rows) {       /* safety test; should always be found ... */
          /* Construct one row of the tableau */
          bsolve(lp, row_nr, prow, NULL, epsvalue*DOUBLEROUND, 1.0);
          prod_xA(lp, coltarget, prow, NULL, epsvalue, 1.0,
                                 prow, NULL, MAT_ROUNDDEFAULT);
          /* sign = my_chsign(is_chsign(lp, row_nr), -1); */
          sign = my_chsign(lp->is_lower[row_nr], -1);
          min1=infinite;
          min2=infinite;
          for(l=1; l<=lp->sum; l++)   /* search for the column(s) which first results in further iterations */
            if ((!lp->is_basic[l]) && (lp->upbo[l]>0.0) &&
                (fabs(prow[l])>epsvalue) && (drow[l]*(lp->is_lower[l] ? -1 : 1)<epsvalue)) {
              a = unscaled_mat(lp, fabs(drow[l] / prow[l]), 0, i);
              if(prow[l]*sign*(lp->is_lower[l] ? 1 : -1) < 0.0) {
                if(a < min1)
                  min1 = a;
              }
              else {
                if(a < min2)
                  min2 = a;
              }
            }
          if ((lp->is_lower[varnr] == 0) == (is_maxim(lp) == FALSE)) {
            a = min1;
            min1 = min2;
            min2 = a;
          }
          if (min1<infinite)
            from = OrigObj[i]-min1;
          if (min2<infinite)
            till = OrigObj[i]+min2;
          a = lp->solution[varnr];
          if (is_maxim(lp)) {
            if (a - lp->lowbo[varnr] < epsvalue)
              from = -infinite; /* if variable is at lower bound then decrementing objective coefficient will not result in extra iterations because it would only extra decrease the value, but since it is at its lower bound ... */
            else if (lp->lowbo[varnr] + lp->upbo[varnr] - a < epsvalue)
              till = infinite;  /* if variable is at upper bound then incrementing objective coefficient will not result in extra iterations because it would only extra increase the value, but since it is at its upper bound ... */
          }
          else {
            if (a - lp->lowbo[varnr] < epsvalue)
              till = infinite;  /* if variable is at lower bound then incrementing objective coefficient will not result in extra iterations because it would only extra decrease the value, but since it is at its lower bound ... */
            else if (lp->lowbo[varnr] + lp->upbo[varnr] - a < epsvalue)
              from = -infinite; /* if variable is at upper bound then decrementing objective coefficient will not result in extra iterations because it would only extra increase the value, but since it is at its upper bound ... */
          }
        }
      }
      lp->objfrom[i]=from;
      lp->objtill[i]=till;
    }
    mempool_releaseVector(lp->workarrays, (char *) coltarget, FALSE);
  }
  FREE(prow);
  FREE(OrigObj);
  FREE(drow);

  return((MYBOOL) ok);
} /* construct_sensitivity_obj */

STATIC MYBOOL refactRecent(lprec *lp)
{
  int pivcount = lp->bfp_pivotcount(lp);
  if(pivcount == 0)
    return( AUTOMATIC );
  else if (pivcount < 2*DEF_MAXPIVOTRETRY)
    return( TRUE );
  else
    return( FALSE );
}

STATIC MYBOOL check_if_less(lprec *lp, REAL x, REAL y, int variable)
{
  if(y < x-scaled_value(lp, lp->epsint, variable)) {
    if(lp->bb_trace)
      report(lp, NORMAL, "check_if_less: Invalid new bound %g should be < %g for %s\n",
                         x, y, get_col_name(lp, variable));
    return(FALSE);
  }
  else
    return(TRUE);
}

/* Various basis utility routines */

STATIC int findNonBasicSlack(lprec *lp, MYBOOL *is_basic)
{
  int i;

  for(i = lp->rows; i > 0; i--)
    if(!is_basic[i])
      break;
  return( i );
}

STATIC int findBasisPos(lprec *lp, int notint, int *var_basic)
{
  int i;

  if(var_basic == NULL)
    var_basic = lp->var_basic;
  for(i = lp->rows; i > 0; i--)
    if(var_basic[i] == notint)
      break;
  return( i );
}

STATIC void replaceBasisVar(lprec *lp, int rownr, int var, int *var_basic, MYBOOL *is_basic)
{
  int out;

  out = var_basic[rownr];
  var_basic[rownr] = var;
  is_basic[out] = FALSE;
  is_basic[var] = TRUE;
}

STATIC void free_duals(lprec *lp)
{
  FREE(lp->duals);
  FREE(lp->full_duals);
  FREE(lp->dualsfrom);
  FREE(lp->dualstill);
  FREE(lp->objfromvalue);
  FREE(lp->objfrom);
  FREE(lp->objtill);
}

/* Transform RHS by adjusting for the bound state of variables;
   optionally rebase upper bound, and account for this in later calls */
STATIC void initialize_solution(lprec *lp, MYBOOL shiftbounds)
{
  int     i, k1, k2, *matRownr, colnr;
  LREAL   theta;
  REAL    value, *matValue, loB, upB;
  MATrec  *mat = lp->matA;

  /* Set bounding status indicators */
  if(lp->bb_bounds != NULL) {
    if(shiftbounds == INITSOL_SHIFTZERO) {
      if(lp->bb_bounds->UBzerobased)
        report(lp, SEVERE, "initialize_solution: The upper bounds are already zero-based at refactorization %d\n",
                           lp->bfp_refactcount(lp, BFP_STAT_REFACT_TOTAL));
      lp->bb_bounds->UBzerobased = TRUE;
    }
    else if(!lp->bb_bounds->UBzerobased)
        report(lp, SEVERE, "initialize_solution: The upper bounds are not zero-based at refactorization %d\n",
                           lp->bfp_refactcount(lp, BFP_STAT_REFACT_TOTAL));
  }

  /* Initialize the working RHS/basic variable solution vector */
  i = is_action(lp->anti_degen, ANTIDEGEN_RHSPERTURB) && (lp->monitor != NULL) && lp->monitor->active;
  if(sizeof(*lp->rhs) == sizeof(*lp->orig_rhs) && !i) {
    MEMCOPY(lp->rhs, lp->orig_rhs, lp->rows+1);
  }
  else if(i) {
    lp->rhs[0] = lp->orig_rhs[0];
    for(i = 1; i <= lp->rows; i++) {
      if(is_constr_type(lp, i, EQ))
        theta = rand_uniform(lp, lp->epsvalue);
      else {
        theta = rand_uniform(lp, lp->epsperturb);
/*        if(lp->orig_upbo[i] < lp->infinite)
          lp->orig_upbo[i] += theta; */
      }
      lp->rhs[i] = lp->orig_rhs[i] + theta;
    }
  }
  else
    for(i = 0; i <= lp->rows; i++)
      lp->rhs[i] = lp->orig_rhs[i];

/* Adjust active RHS for variables at their active upper/lower bounds */
  for(i = 1; i <= lp->sum; i++) {

    upB = lp->upbo[i];
    loB = lp->lowbo[i];

    /* Shift to "ranged" upper bound, tantamount to defining zero-based variables */
    if(shiftbounds == INITSOL_SHIFTZERO) {
      if((loB > -lp->infinite) && (upB < lp->infinite))
        lp->upbo[i] -= loB;
      if(lp->upbo[i] < 0)
        report(lp, SEVERE, "initialize_solution: Invalid rebounding; variable %d at refact %d, iter %.0f\n",
                           i, lp->bfp_refactcount(lp, BFP_STAT_REFACT_TOTAL), (double) get_total_iter(lp));
    }

    /* Use "ranged" upper bounds */
    else if(shiftbounds == INITSOL_USEZERO) {
      if((loB > -lp->infinite) && (upB < lp->infinite))
        upB += loB;
    }

    /* Shift upper bound back to original value */
    else if(shiftbounds == INITSOL_ORIGINAL) {
      if((loB > -lp->infinite) && (upB < lp->infinite)) {
        lp->upbo[i] += loB;
        upB += loB;
      }
      continue;
    }
    else
      report(lp, SEVERE, "initialize_solution: Invalid option value '%d'\n",
                         shiftbounds);

    /* Set the applicable adjustment */
    if(lp->is_lower[i])
      theta = loB;
    else
      theta = upB;


    /* Check if we need to pass through the matrix;
       remember that basis variables are always lower-bounded */
    if(theta == 0)
      continue;

    /* Do user and artificial variables */
    if(i > lp->rows) {

      /* Get starting and ending indeces in the NZ vector */
      colnr = i - lp->rows;
      k1 = mat->col_end[colnr - 1];
      k2 = mat->col_end[colnr];
      matRownr = &COL_MAT_ROWNR(k1);
      matValue = &COL_MAT_VALUE(k1);

      /* Get the objective as row 0, optionally adjusting the objective for phase 1 */
      value = get_OF_active(lp, i, theta);
      lp->rhs[0] -= value;

      /* Do the normal case */
      for(; k1 < k2;
          k1++, matRownr += matRowColStep, matValue += matValueStep) {
        lp->rhs[*matRownr] -= theta * (*matValue);
      }
    }

    /* Do slack variables (constraint "bounds")*/
    else {
      lp->rhs[i] -= theta;
    }

  }

  /* Do final pass to get the maximum value */
  i = idamax(lp->rows /* +1 */, lp->rhs, 1);
  lp->rhsmax = fabs(lp->rhs[i]);

  if(shiftbounds == INITSOL_SHIFTZERO)
    clear_action(&lp->spx_action, ACTION_REBASE);

}

/* This routine recomputes the basic variables using the full inverse */
STATIC void recompute_solution(lprec *lp, MYBOOL shiftbounds)
{
  /* Compute RHS = b - A(n)*x(n) */
  initialize_solution(lp, shiftbounds);

  /* Compute x(b) = Inv(B)*RHS (Ref. lp_solve inverse logic and Chvatal p. 121) */
  lp->bfp_ftran_normal(lp, lp->rhs, NULL);
  if(!lp->obj_in_basis) {
    int i, ib, n = lp->rows;
    for(i = 1; i <= n; i++) {
      ib = lp->var_basic[i];
      if(ib > n)
        lp->rhs[0] -= get_OF_active(lp, ib, lp->rhs[i]);
    }
  }

 /* Round the values (should not be greater than the factor used in bfp_pivotRHS) */
  roundVector(lp->rhs, lp->rows, lp->epsvalue);

  clear_action(&lp->spx_action, ACTION_RECOMPUTE);
}

/* This routine compares an existing basic solution to a recomputed one;
   Note that the routine must provide for the possibility that the order of the
   basis variables can be changed by the inversion engine. */
STATIC int verify_solution(lprec *lp, MYBOOL reinvert, char *info)
{
  int  i, ii, n, *oldmap, *newmap, *refmap = NULL;
  REAL *oldrhs, err, errmax;

  allocINT(lp, &oldmap, lp->rows+1, FALSE);
  allocINT(lp, &newmap, lp->rows+1, FALSE);
  allocREAL(lp, &oldrhs, lp->rows+1, FALSE);

  /* Get sorted mapping of the old basis */
  for(i = 0; i <= lp->rows; i++)
    oldmap[i] = i;
  if(reinvert) {
    allocINT(lp, &refmap, lp->rows+1, FALSE);
    MEMCOPY(refmap, lp->var_basic, lp->rows+1);
    sortByINT(oldmap, refmap, lp->rows, 1, TRUE);
  }

  /* Save old and calculate the new RHS vector */
  MEMCOPY(oldrhs, lp->rhs, lp->rows+1);
  if(reinvert)
    invert(lp, INITSOL_USEZERO, FALSE);
  else
    recompute_solution(lp, INITSOL_USEZERO);

  /* Get sorted mapping of the new basis */
  for(i = 0; i <= lp->rows; i++)
    newmap[i] = i;
  if(reinvert) {
    MEMCOPY(refmap, lp->var_basic, lp->rows+1);
    sortByINT(newmap, refmap, lp->rows, 1, TRUE);
  }

  /* Identify any gap */
  errmax = 0;
  ii = -1;
  n = 0;
  for(i = lp->rows; i > 0; i--) {
    err = fabs(my_reldiff(oldrhs[oldmap[i]], lp->rhs[newmap[i]]));
    if(err > lp->epsprimal) {
      n++;
      if(err > errmax) {
        ii = i;
        errmax = err;
      }
    }
  }
  err = fabs(my_reldiff(oldrhs[i], lp->rhs[i]));
  if(err < lp->epspivot) {
    i--;
    err = 0;
  }
  else {
    n++;
    if(ii < 0) {
      ii = 0;
      errmax = err;
    }
  }
  if(n > 0) {
    report(lp, IMPORTANT, "verify_solution: Iter %.0f %s - %d errors; OF %g, Max @row %d %g\n",
                           (double) get_total_iter(lp), my_if(info == NULL, "", info), n, err, newmap[ii], errmax);
  }
  /* Copy old results back (not possible for inversion) */
  if(!reinvert)
    MEMCOPY(lp->rhs, oldrhs, lp->rows+1);

  FREE(oldmap);
  FREE(newmap);
  FREE(oldrhs);
  if(reinvert)
    FREE(refmap);

  return( ii );

}

/* Preprocessing and postprocessing functions */
STATIC int identify_GUB(lprec *lp, MYBOOL mark)
{
  int    i, j, jb, je, k, knint, srh;
  REAL   rh, mv, tv, bv;
  MATrec *mat = lp->matA;

  if((lp->equalities == 0) || !mat_validate(mat))
    return( 0 );

  k = 0;
  for(i = 1; i <= lp->rows; i++) {

    /* Check if it is an equality constraint */
    if(!is_constr_type(lp, i, EQ))
      continue;

    rh = get_rh(lp, i);
    srh = my_sign(rh);
    knint = 0;
    je = mat->row_end[i];
    for(jb = mat->row_end[i-1]; jb < je; jb++) {
      j = ROW_MAT_COLNR(jb);

      /* Check for validity of the equation elements */
      if(!is_int(lp, j))
        knint++;
      if(knint > 1)
        break;

      mv = get_mat_byindex(lp, jb, TRUE, FALSE);
      if(fabs(my_reldiff(mv, rh)) > lp->epsprimal)
        break;

      tv = mv*get_upbo(lp, j);
      bv = get_lowbo(lp, j);
#if 0 /* Requires 1 as upper bound */
      if((fabs(my_reldiff(tv, rh)) > lp->epsprimal) || (bv != 0))
#else /* Can handle any upper bound >= 1 */
      if((srh*(tv-rh) < -lp->epsprimal) || (bv != 0))
#endif
        break;
    }

    /* Update GUB count and optionally mark the GUB */
    if(jb == je) {
      k++;
      if(mark == TRUE)
        lp->row_type[i] |= ROWTYPE_GUB;
      else if(mark == AUTOMATIC)
        break;
    }

  }
  return( k );
}

STATIC int prepare_GUB(lprec *lp)
{
  int    i, j, jb, je, k, *members = NULL;
  REAL   rh;
  char   GUBname[16];
  MATrec *mat = lp->matA;

  if((lp->equalities == 0) ||
     !allocINT(lp, &members, lp->columns+1, TRUE) ||
     !mat_validate(mat))
    return( 0 );

  for(i = 1; i <= lp->rows; i++) {

    /* Check if it has been marked as a GUB */
    if(!(lp->row_type[i] & ROWTYPE_GUB))
      continue;

    /* Pick up the GUB column indeces */
    k = 0;
    je = mat->row_end[i];
    for(jb = mat->row_end[i-1], k = 0; jb < je; jb++) {
      members[k] = ROW_MAT_COLNR(jb);
      k++;
    }

    /* Add the GUB */
    j = GUB_count(lp) + 1;
    sprintf(GUBname, "GUB_%d", i);
    add_GUB(lp, GUBname, j, k, members);

    /* Unmark the GUBs */
    clear_action(&(lp->row_type[i]), ROWTYPE_GUB);

    /* Standardize coefficients to 1 if necessary */
    rh = get_rh(lp, i);
    if(fabs(my_reldiff(rh, 1)) > lp->epsprimal) {
      set_rh(lp, i, 1);
      for(jb = mat->row_end[i-1]; jb < je; jb++) {
        j = ROW_MAT_COLNR(jb);
        set_mat(lp, i,j, 1);
      }
    }

  }
  FREE(members);
  return(GUB_count(lp));
}

/* Pre- and post processing functions, i.a. splitting free variables */
STATIC MYBOOL pre_MIPOBJ(lprec *lp)
{
#ifdef MIPboundWithOF
  if(MIP_count(lp) > 0) {
    int i = 1;
    while((i <= lp->rows) && !mat_equalRows(lp->matA, 0, i) && !is_constr_type(lp, i, EQ))
      i++;
    if(i <= lp->rows)
      lp->constraintOF = i;
  }
#endif
  lp->bb_deltaOF = MIP_stepOF(lp);
  return( TRUE );
}
STATIC MYBOOL post_MIPOBJ(lprec *lp)
{
#ifdef MIPboundWithOF
/*
  if(lp->constraintOF) {
    del_constraint(lp, lp->rows);
    if(is_BasisReady(lp) && !verify_basis(lp))
      return( FALSE );
  }
*/
#endif
  return( TRUE );
}

int preprocess(lprec *lp)
{
  int    i, j, k, ok = TRUE, *new_index = NULL;
  REAL   hold, *new_column = NULL;
  MYBOOL scaled, primal1, primal2;

 /* do not process if already preprocessed */
  if(lp->wasPreprocessed)
    return( ok );

  /* Write model statistics and optionally initialize partial pricing structures */
  if(lp->lag_status != RUNNING) {
    MYBOOL doPP;

    /* Extract the user-specified simplex strategy choices */
    primal1 = (MYBOOL) (lp->simplex_strategy & SIMPLEX_Phase1_PRIMAL);
    primal2 = (MYBOOL) (lp->simplex_strategy & SIMPLEX_Phase2_PRIMAL);

    /* Initialize partial pricing structures */
    doPP = is_piv_mode(lp, PRICE_PARTIAL | PRICE_AUTOPARTIAL);
/*    doPP &= (MYBOOL) (lp->columns / 2 > lp->rows); */
    if(doPP) {
      i = partial_findBlocks(lp, FALSE, FALSE);
      if(i < 4)
        i = (int) (5 * log((REAL) lp->columns / lp->rows));
      report(lp, NORMAL, "The model is %s to have %d column blocks/stages.\n",
                         (i > 1 ? "estimated" : "set"), i);
      set_partialprice(lp, i, NULL, FALSE);
    }
/*    doPP &= (MYBOOL) (lp->rows / 4 > lp->columns); */
    if(doPP) {
      i = partial_findBlocks(lp, FALSE, TRUE);
      if(i < 4)
        i = (int) (5 * log((REAL) lp->rows / lp->columns));
      report(lp, NORMAL, "The model is %s to have %d row blocks/stages.\n",
                         (i > 1 ? "estimated" : "set"), i);
      set_partialprice(lp, i, NULL, TRUE);
    }

    /* Check for presence of valid pricing blocks if partial pricing
      is defined, but not autopartial is not set */
    if(!doPP && is_piv_mode(lp, PRICE_PARTIAL)) {
      if((lp->rowblocks == NULL) || (lp->colblocks == NULL)) {
        report(lp, IMPORTANT, "Ignoring partial pricing, since block structures are not defined.\n");
        clear_action(&lp->piv_strategy, PRICE_PARTIAL);
      }
    }

    /* Initialize multiple pricing block divisor */
#if 0
    if(primal1 || primal2)
      lp->piv_strategy |= PRICE_MULTIPLE | PRICE_AUTOMULTIPLE;
#endif
    if(is_piv_mode(lp, PRICE_MULTIPLE) && (primal1 || primal2)) {
      doPP = is_piv_mode(lp, PRICE_AUTOMULTIPLE);
      if(doPP) {
        i = (int) (2.5*log((REAL) lp->sum));
        SETMAX( i, 1);
        set_multiprice(lp, i);
      }
      if(lp->multiblockdiv > 1)
      report(lp, NORMAL, "Using %d-candidate primal simplex multiple pricing block.\n",
                          lp->columns / lp->multiblockdiv);
    }
    else
      set_multiprice(lp, 1);

    report(lp, NORMAL, "Using %s simplex for phase 1 and %s simplex for phase 2.\n",
                       my_if(primal1, "PRIMAL", "DUAL"), my_if(primal2, "PRIMAL", "DUAL"));
    i = get_piv_rule(lp);
    if((i == PRICER_STEEPESTEDGE) && is_piv_mode(lp, PRICE_PRIMALFALLBACK))
      report(lp, NORMAL, "The pricing strategy is set to '%s' for the dual and '%s' for the primal.\n",
                       get_str_piv_rule(i), get_str_piv_rule(i-1));
    else
      report(lp, NORMAL, "The primal and dual simplex pricing strategy set to '%s'.\n",
                       get_str_piv_rule(i));

    report(lp, NORMAL, " \n");
  }

  /* Compute a minimum step improvement step requirement */
  pre_MIPOBJ(lp);

 /* First create extra columns for FR variables or flip MI variables */
  for (j = 1; j <= lp->columns; j++) {

#ifdef Paranoia
    if((lp->rows != lp->matA->rows) || (lp->columns != lp->matA->columns))
      report(lp, SEVERE, "preprocess: Inconsistent variable counts found\n");
#endif

   /* First handle sign-flipping of variables:
       1) ... with a finite upper bound and a negative Inf-bound (since basis variables are lower-bounded)
       2) ... with bound assymetry within negrange limits (for stability reasons) */
    i = lp->rows + j;
    hold = lp->orig_upbo[i];
/*
    if((hold <= 0) || (!is_infinite(lp, lp->negrange) &&
                       (hold < -lp->negrange) &&
                       (lp->orig_lowbo[i] <= lp->negrange)) ) {
*/
#define fullybounded FALSE
    if( ((hold < lp->infinite) && my_infinite(lp, lp->orig_lowbo[i])) ||
        (!fullybounded && !my_infinite(lp, lp->negrange) &&
         (hold < -lp->negrange) && (lp->orig_lowbo[i] <= lp->negrange)) ) {
      /* Delete split sibling variable if one existed from before */
      if((lp->var_is_free != NULL) && (lp->var_is_free[j] > 0))
        del_column(lp, lp->var_is_free[j]);
      /* Negate the column / flip to the positive range */
      mat_multcol(lp->matA, j, -1, TRUE);
      if(lp->var_is_free == NULL) {
        if(!allocINT(lp, &lp->var_is_free, MAX(lp->columns, lp->columns_alloc) + 1, TRUE))
          return(FALSE);
      }
      lp->var_is_free[j] = -j; /* Indicator UB and LB are switched, with no helper variable added */
      lp->orig_upbo[i] = my_flipsign(lp->orig_lowbo[i]);
      lp->orig_lowbo[i] = my_flipsign(hold);
      /* Check for presence of negative ranged SC variable */
      if(lp->sc_lobound[j] > 0) {
        lp->sc_lobound[j] = lp->orig_lowbo[i];
        lp->orig_lowbo[i] = 0;
      }
    }
   /* Then deal with -+, full-range/FREE variables by creating a helper variable */
    else if((lp->orig_lowbo[i] <= lp->negrange) && (hold >= -lp->negrange)) {
      if(lp->var_is_free == NULL) {
        if(!allocINT(lp, &lp->var_is_free, MAX(lp->columns,lp->columns_alloc) + 1, TRUE))
          return(FALSE);
      }
      if(lp->var_is_free[j] <= 0) { /* If this variable wasn't split yet ... */
        if(SOS_is_member(lp->SOS, 0, i - lp->rows)) {   /* Added */
          report(lp, IMPORTANT, "preprocess: Converted negative bound for SOS variable %d to zero",
                                i - lp->rows);
          lp->orig_lowbo[i] = 0;
          continue;
        }
        if(new_column == NULL) {
          if(!allocREAL(lp, &new_column, lp->rows + 1, FALSE) ||
             !allocINT(lp, &new_index, lp->rows + 1, FALSE)) {
            ok = FALSE;
            break;
          }
        }
       /* Avoid precision loss by turning off unscaling and rescaling */
       /* in get_column and add_column operations; also make sure that */
       /* full scaling information is preserved */
        scaled = lp->scaling_used;
        lp->scaling_used = FALSE;
        k = get_columnex(lp, j, new_column, new_index);
        if(!add_columnex(lp, k, new_column, new_index)) {
          ok = FALSE;
          break;
        }
        mat_multcol(lp->matA, lp->columns, -1, TRUE);
        if(scaled)
          lp->scalars[lp->rows+lp->columns] = lp->scalars[i];
        lp->scaling_used = (MYBOOL) scaled;
        /* Only create name if we are not clearing a pre-used item, since this
           variable could have been deleted by presolve but the name is required
           for solution reconstruction. */
        if(lp->names_used && (lp->col_name[j] == NULL)) {
          char fieldn[50];

          sprintf(fieldn, "__AntiBodyOf(%d)__", j);
          if(!set_col_name(lp, lp->columns, fieldn)) {
/*          if (!set_col_name(lp, lp->columns, get_col_name(lp, j))) { */
            ok = FALSE;
            break;
          }
        }
        /* Set (positive) index to the original column's split / helper and back */
        lp->var_is_free[j] = lp->columns;
      }
      lp->orig_upbo[lp->rows + lp->var_is_free[j]] = my_flipsign(lp->orig_lowbo[i]);
      lp->orig_lowbo[i] = 0;

      /* Negative index indicates x is split var and -var_is_free[x] is index of orig var */
      lp->var_is_free[lp->var_is_free[j]] = -j;
      lp->var_type[lp->var_is_free[j]] = lp->var_type[j];
    }
   /* Check for positive ranged SC variables */
    else if(lp->sc_lobound[j] > 0) {
      lp->sc_lobound[j] = lp->orig_lowbo[i];
      lp->orig_lowbo[i] = 0;
    }

   /* Tally integer variables in SOS'es */
    if(SOS_is_member(lp->SOS, 0, j) && is_int(lp, j))
      lp->sos_ints++;
  }

  FREE(new_column);
  FREE(new_index);

  /* Fill lists of GUB constraints, if appropriate */
  if((MIP_count(lp) > 0) && is_bb_mode(lp, NODE_GUBMODE) && (identify_GUB(lp, AUTOMATIC) > 0))
    prepare_GUB(lp);

  /* (Re)allocate reduced cost arrays */
  ok = allocREAL(lp, &(lp->drow), lp->sum+1, AUTOMATIC) &&
       allocINT(lp, &(lp->nzdrow), lp->sum+1, AUTOMATIC);
  if(ok)
    lp->nzdrow[0] = 0;

  /* Minimize memory usage */
  memopt_lp(lp, 0, 0, 0);

  lp->wasPreprocessed = TRUE;

  return(ok);
}

void postprocess(lprec *lp)
{
  int i,ii,j;
  REAL hold;

 /* Check if the problem actually was preprocessed */
  if(!lp->wasPreprocessed)
    return;

 /* Must compute duals here in case we have free variables; note that in
    this case sensitivity analysis is not possible unless done here */
  if((MIP_count(lp) == 0) &&
     (is_presolve(lp, PRESOLVE_DUALS) || (lp->var_is_free != NULL)))
    construct_duals(lp);
  if(is_presolve(lp, PRESOLVE_SENSDUALS)) {
    if(!construct_sensitivity_duals(lp) || !construct_sensitivity_obj(lp))
      report(lp, IMPORTANT, "postprocess: Unable to allocate working memory for duals.\n");
  }

 /* Loop over all columns */
  for (j = 1; j <= lp->columns; j++) {
    i = lp->rows + j;
   /* Reconstruct strictly negative values */
    if((lp->var_is_free != NULL) && (lp->var_is_free[j] < 0)) {
      /* Check if we have the simple case where the UP and LB are negated and switched */
      if(-lp->var_is_free[j] == j) {
        mat_multcol(lp->matA, j, -1, TRUE);
        hold = lp->orig_upbo[i];
        lp->orig_upbo[i] = my_flipsign(lp->orig_lowbo[i]);
        lp->orig_lowbo[i] = my_flipsign(hold);
        lp->best_solution[i] = my_flipsign(lp->best_solution[i]);
        transfer_solution_var(lp, j);

        /* hold = lp->objfrom[j];
        lp->objfrom[j] = my_flipsign(lp->objtill[j]);
        lp->objtill[j] = my_flipsign(hold); */ /* under investigation <peno> */

        /* lp->duals[i] = my_flipsign(lp->duals[i]);
        hold = lp->dualsfrom[i];
        lp->dualsfrom[i] = my_flipsign(lp->dualstill[i]);
        lp->dualstill[i] = my_flipsign(hold); */ /* under investigation <peno> */
       /* Bound switch undone, so clear the status */
        lp->var_is_free[j] = 0;
       /* Adjust negative ranged SC */
        if(lp->sc_lobound[j] > 0)
          lp->orig_lowbo[lp->rows + j] = -lp->sc_lobound[j];
      }
      /* Ignore the split / helper columns (will be deleted later) */
    }
   /* Condense values of extra columns of quasi-free variables split in two */
    else if((lp->var_is_free != NULL) && (lp->var_is_free[j] > 0)) {
      ii = lp->var_is_free[j]; /* Index of the split helper var */
      /* if(lp->objfrom[j] == -lp->infinite)
        lp->objfrom[j] = -lp->objtill[ii];
      lp->objtill[ii] = lp->infinite;
      if(lp->objtill[j] == lp->infinite)
        lp->objtill[j] = my_flipsign(lp->objfrom[ii]);
      lp->objfrom[ii] = -lp->infinite; */ /* under investigation <peno> */

      ii += lp->rows;
      lp->best_solution[i] -= lp->best_solution[ii]; /* join the solution again */
      transfer_solution_var(lp, j);
      lp->best_solution[ii] = 0;

      /* if(lp->duals[i] == 0)
        lp->duals[i] = my_flipsign(lp->duals[ii]);
      lp->duals[ii] = 0;
      if(lp->dualsfrom[i] == -lp->infinite)
        lp->dualsfrom[i] = my_flipsign(lp->dualstill[ii]);
      lp->dualstill[ii] = lp->infinite;
      if(lp->dualstill[i] == lp->infinite)
        lp->dualstill[i] = my_flipsign(lp->dualsfrom[ii]);
      lp->dualsfrom[ii] = -lp->infinite; */ /* under investigation <peno> */

      /* Reset to original bound */
      lp->orig_lowbo[i] = my_flipsign(lp->orig_upbo[ii]);
    }
   /* Adjust for semi-continuous variables */
    else if(lp->sc_lobound[j] > 0) {
      lp->orig_lowbo[i] = lp->sc_lobound[j];
    }
  }

  /* Remove any split column helper variables */
  del_splitvars(lp);
  post_MIPOBJ(lp);

  /* Do extended reporting, if specified */
  if(lp->verbose > NORMAL) {
    REPORT_extended(lp);

  }

  lp->wasPreprocessed = FALSE;
}

