
#ifndef HEADER_lp_lib
#define HEADER_lp_lib

/* --------------------------------------------------------------------------

  This is the main library header file for the lp_solve v5.0 release

  Starting at version 3.0, LP_Solve is released under the LGPL license.
  For full information, see the enclosed file LGPL.txt.

  Original developer:   Michel Berkelaar  -  michel@ics.ele.tue.nl
  Most changes 1.5-2.0: Jeroen Dirks      -  jeroend@tor.numetrix.com
  Changes 3.2-4.0:      Kjell Eikland     -  kjell.eikland@broadpark.no
                        (Simplex code, SOS, SC, code optimization)
                        Peter Notebaert   -  lpsolve@peno.be
                        (Sensitivity analysis, documentation)
  Changes 5.0+:         Kjell Eikland     -  kjell.eikland@broadpark.no
                        (BFP, XLI, simplex, B&B, code modularization)
                        Peter Notebaert   -  lpsolve@peno.be
                        (Sensitivity analysis, New lp parser, LINDO (XLI)
                        parser, VB/.NET interface, documentation)

  Release notes:

  Version 4.0 enhances version 3.2 in terms of internal program/simplex
  architecture, call level interfaces, data layout, features and contains
  several bug fixes.  There is now complete support for semi-continuous
  variables and SOS constructions.  In the process, a complete API
  was added. The MPS parser has been amended to support this.
  Sensitivity analysis and variouse bug fixes was provided by Peter
  Notebaert in 4.0 sub-releases.  Peter also wrote a complete
  documentation of the API and contributed a VB interface, both of which
  significantly enhanced the accessibility of lp_solve.

  Version 5.0 is a major rewrite and code cleanup.  The main additions that
  drove forward this cleanup were the modular inversion logic with optimal
  column ordering, addition of primal phase 1 and dual phase 2 logic for
  full flexibility in the selection of primal and dual simplex modes,
  DEVEX and steepest edge pivot selection, along with dynamic cycling
  detection and prevention.  This cleanup made it possible to harmonize the
  internal rounding principles, contributing to increased numerical stability.

  Version 5.1 rearranges the matrix storage model by enabling both legacy
  element record-based storage and split vector storage.  In addition the
  lprec structure is optimized and additional routines are added, mainly for
  sparse vector additions and enhanced XLI functionality.  Support for XML-
  based models was added on the basis of the LPFML schema via xli_LPFML.

  Version 5.2 removes the objective function from the constraint matrix,
  adds a number of presolve options and speed them up.  Degeneracy handling
  is significantly improved. Support for XLI_ZIMPL was added.
  Multiple and partial pricing has been enhanced and activated.

  -------------------------------------------------------------------------- */
/* Define user program feature option switches                               */
/* ------------------------------------------------------------------------- */

#if !defined _WINDOWS && !defined _WIN32 && !defined WIN32
# define _isnan(x) FALSE
#endif

#define SETMASK(variable, mask)     variable |= mask
#define CLEARMASK(variable, mask)   variable &= ~(mask)
#define TOGGLEMASK(variable, mask)  variable ^= mask
#define ISMASKSET(variable, mask)   (MYBOOL) (((variable) & (mask)) != 0)

/* Utility/system settings                                                   */
/* ------------------------------------------------------------------------- */
/*#define INTEGERTIME */                    /* Set use of lower-resolution timer */


/* New v5.0+ simplex/optimization features and settings                      */
/* ------------------------------------------------------------------------- */
/*#define NoRowScaleOF */               /* Optionally skip row-scaling of the OF */
#define DoMatrixRounding                  /* Round A matrix elements to precision */
#define DoBorderRounding            /* Round RHS, bounds and ranges to precision */
#define Phase1EliminateRedundant        /* Remove rows of redundant artificials  */
#define FixViolatedOptimal
#define ImproveSolutionPrecision                 /* Round optimal solution values */
/*#define IncreasePivotOnReducedAccuracy */  /* Increase epspivot on instability */
/*#define FixInaccurateDualMinit */     /* Reinvert on inaccuracy in dual minits */
/*#define EnforcePositiveTheta */        /* Ensure that the theta range is valid */
#define ResetMinitOnReinvert
/*#define UsePrimalReducedCostUpdate */                            /* Not tested */
/*#define UseDualReducedCostUpdate */      /* Seems Ok, but slower than expected */
/*#ifdef UseLegacyExtrad */                     /* Use v3.2- style Extrad method */
#define UseMilpExpandedRCF         /* Non-ints in reduced cost bound tightening */
/*#define UseMilpSlacksRCF */  /* Slacks in reduced cost bound tightening (degen
                                  prone); requires !SlackInitMinusInf */
#define LegacySlackDefinition      /* Slack as the "value of the constraint" */


/* Development features (change at own risk)                                 */
/* ------------------------------------------------------------------------- */
/*#define MIPboundWithOF */ /* Enable to detect OF constraint for use during B&B */
/*#define SlackInitMinusInf */        /* Slacks have 0 LB if this is not defined */
#define FULLYBOUNDEDSIMPLEX FALSE     /* WARNING: Activate at your own risk! */


/* Specify use of the basic linear algebra subroutine library                */
/* ------------------------------------------------------------------------- */
#define libBLAS                  2        /* 0: No, 1: Internal, 2: External */
#define libnameBLAS        "myBLAS"


/* Active inverse logic (default is optimized original etaPFI)               */
/* ------------------------------------------------------------------------- */
#if !defined LoadInverseLib
# define LoadInverseLib TRUE          /* Enable alternate inverse libraries */
#endif
/*#define ExcludeNativeInverse     */   /* Disable INVERSE_ACTIVE inverse engine */

#define DEF_OBJINBASIS        TRUE  /* Additional rows inserted at the top (1 => OF) */

#define INVERSE_NONE            -1
#define INVERSE_LEGACY           0
#define INVERSE_ETAPFI           1
#define INVERSE_LUMOD            2
#define INVERSE_LUSOL            3
#define INVERSE_GLPKLU           4

#ifndef RoleIsExternalInvEngine            /* Defined in inverse DLL drivers */
  #ifdef ExcludeNativeInverse
    #define INVERSE_ACTIVE       INVERSE_NONE       /* Disable native engine */
  #else
    #define INVERSE_ACTIVE       INVERSE_LEGACY      /* User or DLL-selected */
  #endif
#endif


/* Active external language interface logic (default is none)                */
/* ------------------------------------------------------------------------- */
#if !defined LoadLanguageLib
# define LoadLanguageLib TRUE         /* Enable alternate language libraries */
#endif
#define ExcludeNativeLanguage                 /* Disable LANGUAGE_ACTIVE XLI */

#define LANGUAGE_NONE           -1
#define LANGUAGE_LEGACYLP        0
#define LANGUAGE_CPLEXLP         1
#define LANGUAGE_MPSX            2
#define LANGUAGE_LPFML           3
#define LANGUAGE_MATHPROG        4
#define LANGUAGE_AMPL            5
#define LANGUAGE_GAMS            6
#define LANGUAGE_ZIMPL           7
#define LANGUAGE_S               8
#define LANGUAGE_R               9
#define LANGUAGE_MATLAB         10
#define LANGUAGE_OMATRIX        11
#define LANGUAGE_SCILAB         12
#define LANGUAGE_OCTAVE         13
#define LANGUAGE_EMPS           14

#ifndef RoleIsExternalLanguageEngine      /* Defined in XLI driver libraries */
  #ifdef ExcludeNativeLanguage
    #define LANGUAGE_ACTIVE       LANGUAGE_NONE     /* Disable native engine */
  #else
    #define LANGUAGE_ACTIVE       LANGUAGE_CPLEXLP   /* User or DLL-selected */
  #endif
#endif


/* Default parameters and tolerances                                         */
/* ------------------------------------------------------------------------- */
#define OriginalPARAM           0
#define ProductionPARAM         1
#define ChvatalPARAM            2
#define LoosePARAM              3
#if 1
  #define ActivePARAM           ProductionPARAM
#else
  #define ActivePARAM           LoosePARAM
#endif


/* Miscellaneous settings                                                    */
/* ------------------------------------------------------------------------- */
#ifndef Paranoia
  #ifdef _DEBUG
    #define Paranoia
  #endif
#endif


/* Program version data                                                      */
/* ------------------------------------------------------------------------- */
#define MAJORVERSION             5
#define MINORVERSION             5
#define RELEASE                  0
#define BUILD                   11
#define BFPVERSION              12       /* Checked against bfp_compatible() */
#define XLIVERSION              12       /* Checked against xli_compatible() */
/* Note that both BFPVERSION and XLIVERSION typically have to be incremented
   in the case that the lprec structure changes.                             */


/* Include/header files                                                      */
/* ------------------------------------------------------------------------- */
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

#include "lp_types.h"
#include "lp_utils.h"

#if (LoadInverseLib == TRUE) || (LoadLanguageLib == TRUE)
  #ifdef WIN32
    #include <windows.h>
  #else
    #include <dlfcn.h>
  #endif
#endif

#ifndef BFP_CALLMODEL
  #ifdef WIN32
    #define BFP_CALLMODEL __stdcall   /* "Standard" call model */
  #else
    #define BFP_CALLMODEL
  #endif
#endif
#ifndef XLI_CALLMODEL
  #define XLI_CALLMODEL BFP_CALLMODEL
#endif

#define REGISTER        register      /* Speed up certain operations */


/* Definition of program constrants                                          */
/* ------------------------------------------------------------------------- */
#define SIMPLEX_UNDEFINED        0
#define SIMPLEX_Phase1_PRIMAL    1
#define SIMPLEX_Phase1_DUAL      2
#define SIMPLEX_Phase2_PRIMAL    4
#define SIMPLEX_Phase2_DUAL      8
#define SIMPLEX_DYNAMIC         16
#define SIMPLEX_AUTODUALIZE     32

#define SIMPLEX_PRIMAL_PRIMAL   (SIMPLEX_Phase1_PRIMAL + SIMPLEX_Phase2_PRIMAL)
#define SIMPLEX_DUAL_PRIMAL     (SIMPLEX_Phase1_DUAL   + SIMPLEX_Phase2_PRIMAL)
#define SIMPLEX_PRIMAL_DUAL     (SIMPLEX_Phase1_PRIMAL + SIMPLEX_Phase2_DUAL)
#define SIMPLEX_DUAL_DUAL       (SIMPLEX_Phase1_DUAL   + SIMPLEX_Phase2_DUAL)
#define SIMPLEX_DEFAULT         (SIMPLEX_DUAL_PRIMAL)

/* Variable codes (internal) */
#define ISREAL                   0
#define ISINTEGER                1
#define ISSEMI                   2
#define ISSOS                    4
#define ISSOSTEMPINT             8
#define ISGUB                   16

/* Presolve defines */
#define PRESOLVE_NONE            0
#define PRESOLVE_ROWS            1
#define PRESOLVE_COLS            2
#define PRESOLVE_LINDEP          4
#define PRESOLVE_AGGREGATE       8  /* Not implemented */
#define PRESOLVE_SPARSER        16  /* Not implemented */
#define PRESOLVE_SOS            32
#define PRESOLVE_REDUCEMIP      64
#define PRESOLVE_KNAPSACK      128  /* Implementation not tested completely */
#define PRESOLVE_ELIMEQ2       256
#define PRESOLVE_IMPLIEDFREE   512
#define PRESOLVE_REDUCEGCD    1024
#define PRESOLVE_PROBEFIX     2048
#define PRESOLVE_PROBEREDUCE  4096
#define PRESOLVE_ROWDOMINATE  8192
#define PRESOLVE_COLDOMINATE 16384  /* Reduced functionality, should be expanded */
#define PRESOLVE_MERGEROWS   32768
#define PRESOLVE_IMPLIEDSLK  65536
#define PRESOLVE_COLFIXDUAL 131072
#define PRESOLVE_BOUNDS     262144
#define PRESOLVE_LASTMASKMODE    (PRESOLVE_DUALS - 1)
#define PRESOLVE_DUALS      524288
#define PRESOLVE_SENSDUALS 1048576

/* Basis crash options */
#define CRASH_NONE               0
#define CRASH_NONBASICBOUNDS     1
#define CRASH_MOSTFEASIBLE       2
#define CRASH_LEASTDEGENERATE    3

/* Solution recomputation options (internal) */
#define INITSOL_SHIFTZERO        0
#define INITSOL_USEZERO          1
#define INITSOL_ORIGINAL         2

/* Strategy codes to avoid or recover from degenerate pivots,
   infeasibility or numeric errors via randomized bound relaxation */
#define ANTIDEGEN_NONE           0
#define ANTIDEGEN_FIXEDVARS      1
#define ANTIDEGEN_COLUMNCHECK    2
#define ANTIDEGEN_STALLING       4
#define ANTIDEGEN_NUMFAILURE     8
#define ANTIDEGEN_LOSTFEAS      16
#define ANTIDEGEN_INFEASIBLE    32
#define ANTIDEGEN_DYNAMIC       64
#define ANTIDEGEN_DURINGBB     128
#define ANTIDEGEN_RHSPERTURB   256
#define ANTIDEGEN_BOUNDFLIP    512
#define ANTIDEGEN_DEFAULT        (ANTIDEGEN_FIXEDVARS | ANTIDEGEN_STALLING /* | ANTIDEGEN_INFEASIBLE */)

/* REPORT defines */
#define NEUTRAL                  0
#define CRITICAL                 1
#define SEVERE                   2
#define IMPORTANT                3
#define NORMAL                   4
#define DETAILED                 5
#define FULL                     6

/* MESSAGE defines */
#define MSG_NONE                 0
#define MSG_PRESOLVE             1
#define MSG_ITERATION            2
#define MSG_INVERT               4
#define MSG_LPFEASIBLE           8
#define MSG_LPOPTIMAL           16
#define MSG_LPEQUAL             32
#define MSG_LPBETTER            64
#define MSG_MILPFEASIBLE       128
#define MSG_MILPEQUAL          256
#define MSG_MILPBETTER         512
#define MSG_MILPSTRATEGY      1024
#define MSG_MILPOPTIMAL       2048
#define MSG_PERFORMANCE       4096
#define MSG_INITPSEUDOCOST    8192

/* MPS file types */
#define MPSFIXED                 1
#define MPSFREE                  2

/* MPS defines (internal) */
#define MPSUNDEF                -4
#define MPSNAME                 -3
#define MPSOBJSENSE             -2
#define MPSOBJNAME              -1
#define MPSROWS                  0
#define MPSCOLUMNS               1
#define MPSRHS                   2
#define MPSBOUNDS                3
#define MPSRANGES                4
#define MPSSOS                   5

#define MPSVARMASK          "%-8s"
#define MPSVALUEMASK        "%12g"

/* Constraint type codes  (internal) */
#define ROWTYPE_EMPTY            0
#define ROWTYPE_LE               1
#define ROWTYPE_GE               2
#define ROWTYPE_EQ               3
#define ROWTYPE_CONSTRAINT       ROWTYPE_EQ  /* This is the mask for modes */
#define ROWTYPE_OF               4
#define ROWTYPE_INACTIVE         8
#define ROWTYPE_RELAX           16
#define ROWTYPE_GUB             32
#define ROWTYPE_OFMAX            (ROWTYPE_OF + ROWTYPE_GE)
#define ROWTYPE_OFMIN            (ROWTYPE_OF + ROWTYPE_LE)
#define ROWTYPE_CHSIGN           ROWTYPE_GE

/* Public constraint codes */
#define FR                       ROWTYPE_EMPTY
#define LE                       ROWTYPE_LE
#define GE                       ROWTYPE_GE
#define EQ                       ROWTYPE_EQ
#define OF                       ROWTYPE_OF

/* MIP constraint classes */
#define ROWCLASS_Unknown         0   /* Undefined/unknown */
#define ROWCLASS_Objective       1   /* The objective function */
#define ROWCLASS_GeneralREAL     2   /* General real-values constraint */
#define ROWCLASS_GeneralMIP      3   /* General mixed integer/binary and real valued constraint */
#define ROWCLASS_GeneralINT      4   /* General integer-only constraint */
#define ROWCLASS_GeneralBIN      5   /* General binary-only constraint */
#define ROWCLASS_KnapsackINT     6   /* Sum of positive integer times integer variables <= positive integer */
#define ROWCLASS_KnapsackBIN     7   /* Sum of positive integer times binary variables <= positive integer */
#define ROWCLASS_SetPacking      8   /* Sum of binary variables >= 1 */
#define ROWCLASS_SetCover        9   /* Sum of binary variables <= 1 */
#define ROWCLASS_GUB            10   /* Sum of binary variables = 1  */
#define ROWCLASS_MAX             ROWCLASS_GUB

/* Column subsets (internal) */
#define SCAN_USERVARS            1
#define SCAN_SLACKVARS           2
#define SCAN_ARTIFICIALVARS      4
#define SCAN_PARTIALBLOCK        8
#define USE_BASICVARS           16
#define USE_NONBASICVARS        32
#define SCAN_NORMALVARS         (SCAN_USERVARS + SCAN_ARTIFICIALVARS)
#define SCAN_ALLVARS            (SCAN_SLACKVARS + SCAN_USERVARS + SCAN_ARTIFICIALVARS)
#define USE_ALLVARS             (USE_BASICVARS + USE_NONBASICVARS)
#define OMIT_FIXED              64
#define OMIT_NONFIXED          128

/* Improvement defines */
#define IMPROVE_NONE             0
#define IMPROVE_SOLUTION         1
#define IMPROVE_DUALFEAS         2
#define IMPROVE_THETAGAP         4
#define IMPROVE_BBSIMPLEX        8
#define IMPROVE_DEFAULT          (IMPROVE_DUALFEAS + IMPROVE_THETAGAP)
#define IMPROVE_INVERSE          (IMPROVE_SOLUTION + IMPROVE_THETAGAP)

/* Scaling types */
#define SCALE_NONE               0
#define SCALE_EXTREME            1
#define SCALE_RANGE              2
#define SCALE_MEAN               3
#define SCALE_GEOMETRIC          4
#define SCALE_FUTURE1            5
#define SCALE_FUTURE2            6
#define SCALE_CURTISREID         7   /* Override to Curtis-Reid "optimal" scaling */

/* Alternative scaling weights */
#define SCALE_LINEAR             0
#define SCALE_QUADRATIC          8
#define SCALE_LOGARITHMIC       16
#define SCALE_USERWEIGHT        31
#define SCALE_MAXTYPE            (SCALE_QUADRATIC-1)

/* Scaling modes */
#define SCALE_POWER2            32   /* As is or rounded to power of 2 */
#define SCALE_EQUILIBRATE       64   /* Make sure that no scaled number is above 1 */
#define SCALE_INTEGERS         128   /* Apply to integer columns/variables */
#define SCALE_DYNUPDATE        256   /* Apply incrementally every solve() */
#define SCALE_ROWSONLY         512   /* Override any scaling to only scale the rows */
#define SCALE_COLSONLY        1024   /* Override any scaling to only scale the rows */

/* Standard defines for typical scaling models (no Lagrangeans) */
#define SCALEMODEL_EQUILIBRATED  (SCALE_LINEAR+SCALE_EXTREME+SCALE_INTEGERS)
#define SCALEMODEL_GEOMETRIC     (SCALE_LINEAR+SCALE_GEOMETRIC+SCALE_INTEGERS)
#define SCALEMODEL_ARITHMETIC    (SCALE_LINEAR+SCALE_MEAN+SCALE_INTEGERS)
#define SCALEMODEL_DYNAMIC       (SCALEMODEL_GEOMETRIC+SCALE_EQUILIBRATE)
#define SCALEMODEL_CURTISREID    (SCALE_CURTISREID+SCALE_INTEGERS+SCALE_POWER2)

/* Iteration status and strategies (internal) */
#define ITERATE_MAJORMAJOR       0
#define ITERATE_MINORMAJOR       1
#define ITERATE_MINORRETRY       2

/* Pricing methods */
#define PRICER_FIRSTINDEX        0
#define PRICER_DANTZIG           1
#define PRICER_DEVEX             2
#define PRICER_STEEPESTEDGE      3
#define PRICER_LASTOPTION        PRICER_STEEPESTEDGE

/* Additional settings for pricers (internal) */
#define PRICER_RANDFACT        0.1
#define DEVEX_RESTARTLIMIT 1.0e+09    /* Reset the norms if any value exceeds this limit */
#define DEVEX_MINVALUE       0.000    /* Minimum weight [0..1] for entering variable, consider 0.01 */

/* Pricing strategies */
#define PRICE_PRIMALFALLBACK     4    /* In case of Steepest Edge, fall back to DEVEX in primal */
#define PRICE_MULTIPLE           8    /* Enable multiple pricing (primal simplex) */
#define PRICE_PARTIAL           16    /* Enable partial pricing */
#define PRICE_ADAPTIVE          32    /* Temporarily use alternative strategy if cycling is detected */
#define PRICE_HYBRID            64    /* NOT IMPLEMENTED */
#define PRICE_RANDOMIZE        128    /* Adds a small randomization effect to the selected pricer */
#define PRICE_AUTOPARTIAL      256    /* Detect and use data on the block structure of the model (primal) */
#define PRICE_AUTOMULTIPLE     512    /* Automatically select multiple pricing (primal simplex) */
#define PRICE_LOOPLEFT        1024    /* Scan entering/leaving columns left rather than right */
#define PRICE_LOOPALTERNATE   2048    /* Scan entering/leaving columns alternatingly left/right */
#define PRICE_HARRISTWOPASS   4096    /* Use Harris' primal pivot logic rather than the default */
#define PRICE_FORCEFULL       8192    /* Non-user option to force full pricing */
#define PRICE_TRUENORMINIT   16384    /* Use true norms for Devex and Steepest Edge initializations */

/*#define _PRICE_NOBOUNDFLIP*/
#if defined _PRICE_NOBOUNDFLIP
#define PRICE_NOBOUNDFLIP    65536    /* Disallow automatic bound-flip during pivot */
#endif

#define PRICE_STRATEGYMASK       (PRICE_PRIMALFALLBACK + \
                                  PRICE_MULTIPLE + PRICE_PARTIAL + \
                                  PRICE_ADAPTIVE + PRICE_HYBRID + \
                                  PRICE_RANDOMIZE + PRICE_AUTOPARTIAL + PRICE_AUTOMULTIPLE + \
                                  PRICE_LOOPLEFT + PRICE_LOOPALTERNATE + \
                                  PRICE_HARRISTWOPASS + \
                                  PRICE_FORCEFULL + PRICE_TRUENORMINIT)

/* B&B active variable codes (internal) */
#define BB_REAL                  0
#define BB_INT                   1
#define BB_SC                    2
#define BB_SOS                   3
#define BB_GUB                   4

/* B&B strategies */
#define NODE_FIRSTSELECT         0
#define NODE_GAPSELECT           1
#define NODE_RANGESELECT         2
#define NODE_FRACTIONSELECT      3
#define NODE_PSEUDOCOSTSELECT    4
#define NODE_PSEUDONONINTSELECT  5    /* Kjell Eikland #1 - Minimize B&B depth */
#define NODE_PSEUDOFEASSELECT   (NODE_PSEUDONONINTSELECT+NODE_WEIGHTREVERSEMODE)
#define NODE_PSEUDORATIOSELECT   6    /* Kjell Eikland #2 - Minimize a "cost/benefit" ratio */
#define NODE_USERSELECT          7
#define NODE_STRATEGYMASK        (NODE_WEIGHTREVERSEMODE-1) /* Mask for B&B strategies */
#define NODE_WEIGHTREVERSEMODE   8
#define NODE_BRANCHREVERSEMODE  16
#define NODE_GREEDYMODE         32
#define NODE_PSEUDOCOSTMODE     64
#define NODE_DEPTHFIRSTMODE    128
#define NODE_RANDOMIZEMODE     256
#define NODE_GUBMODE           512
#define NODE_DYNAMICMODE      1024
#define NODE_RESTARTMODE      2048
#define NODE_BREADTHFIRSTMODE 4096
#define NODE_AUTOORDER        8192
#define NODE_RCOSTFIXING     16384
#define NODE_STRONGINIT      32768

#define BRANCH_CEILING           0
#define BRANCH_FLOOR             1
#define BRANCH_AUTOMATIC         2
#define BRANCH_DEFAULT           3

/* Action constants for simplex and B&B (internal) */
#define ACTION_NONE              0
#define ACTION_ACTIVE            1
#define ACTION_REBASE            2
#define ACTION_RECOMPUTE         4
#define ACTION_REPRICE           8
#define ACTION_REINVERT         16
#define ACTION_TIMEDREINVERT    32
#define ACTION_ITERATE          64
#define ACTION_RESTART         255

/* Solver status values */
#define UNKNOWNERROR            -5
#define DATAIGNORED             -4
#define NOBFP                   -3
#define NOMEMORY                -2
#define NOTRUN                  -1
#define OPTIMAL                  0
#define SUBOPTIMAL               1
#define INFEASIBLE               2
#define UNBOUNDED                3
#define DEGENERATE               4
#define NUMFAILURE               5
#define USERABORT                6
#define TIMEOUT                  7
#define RUNNING                  8
#define PRESOLVED                9

/* Branch & Bound and Lagrangean extra status values (internal) */
#define PROCFAIL                10
#define PROCBREAK               11
#define FEASFOUND               12
#define NOFEASFOUND             13
#define FATHOMED                14

/* Status values internal to the solver (internal) */
#define SWITCH_TO_PRIMAL        20
#define SWITCH_TO_DUAL          21
#define SINGULAR_BASIS          22
#define LOSTFEAS                23
#define MATRIXERROR             24

/* Objective testing options for "bb_better" (internal) */
#define OF_RELAXED               0
#define OF_INCUMBENT             1
#define OF_WORKING               2
#define OF_USERBREAK             3
#define OF_HEURISTIC             4
#define OF_DUALLIMIT             5
#define OF_DELTA                 8  /* Mode */
#define OF_PROJECTED            16  /* Mode - future, not active */

#define OF_TEST_BT               1
#define OF_TEST_BE               2
#define OF_TEST_NE               3
#define OF_TEST_WE               4
#define OF_TEST_WT               5
#define OF_TEST_RELGAP           8  /* Mode */


/* Name list and sparse matrix storage parameters (internal) */
#define MAT_START_SIZE       10000
#define DELTACOLALLOC          100
#define DELTAROWALLOC          100
#define RESIZEFACTOR             4  /* Fractional increase in selected memory allocations */

/* Default solver parameters and tolerances (internal) */
#define DEF_PARTIALBLOCKS       10  /* The default number of blocks for partial pricing */
#define DEF_MAXRELAX             7  /* Maximum number of non-BB relaxations in MILP */
#define DEF_MAXPIVOTRETRY       10  /* Maximum number of times to retry a div-0 situation */
#define DEF_MAXSINGULARITIES    10  /* Maximum number of singularities in refactorization */
#define MAX_MINITUPDATES        60  /* Maximum number of bound swaps between refactorizations
                                       without recomputing the whole vector - contain errors */
#define MIN_REFACTFREQUENCY      5  /* Refactorization frequency indicating an inherent
                                       numerical instability of the basis */
#define LAG_SINGULARLIMIT        5  /* Number of times the objective does not change
                                       before it is assumed that the Lagrangean constraints
                                       are non-binding, and therefore impossible to converge;
                                       upper iteration limit is divided by this threshold */
#define MIN_TIMEPIVOT      5.0e-02  /* Minimum time per pivot for reinversion optimization
                                       purposes; use active monitoring only if a pivot
                                       takes more than MINTIMEPIVOT seconds.  5.0e-2 is
                                       roughly suitable for a 1GHz system.  */
#define MAX_STALLCOUNT          12  /* The absolute upper limit to the number of stalling or
                                       cycling iterations before switching rule */
#define MAX_RULESWITCH           5  /* The maximum number of times to try an alternate pricing rule
                                       to recover from stalling; set negative for no limit. */
#define DEF_TIMEDREFACT  AUTOMATIC  /* Default for timed refactorization in BFPs;
                                       can be FALSE, TRUE or AUTOMATIC (dynamic) */

#define DEF_SCALINGLIMIT         5  /* The default maximum number of scaling iterations */

#define DEF_NEGRANGE      -1.0e+06  /* Downward limit for expanded variable range before the
                                       variable is split into positive and negative components */
#define DEF_BB_LIMITLEVEL      -50  /* Relative B&B limit to protect against very deep,
                                       memory-consuming trees */

#define MAX_FRACSCALE            6  /* The maximum decimal scan range for simulated integers */
#define RANDSCALE              100  /* Randomization scaling range */
#define DOUBLEROUND        0.0e-02  /* Extra rounding scalar used in btran/ftran calculations; the
                                       rationale for 0.0 is that prod_xA() uses rounding as well */
#define DEF_EPSMACHINE    2.22e-16  /* Machine relative precision (doubles) */
#define MIN_STABLEPIVOT        5.0  /* Minimum pivot magnitude assumed to be numerically stable */


/* Precision macros                                                                       */
/* -------------------------------------------------------------------------------------- */
#define PREC_REDUCEDCOST        lp->epsvalue
#define PREC_IMPROVEGAP         lp->epsdual
#define PREC_SUBSTFEASGAP       lp->epsprimal
#if 1
  #define PREC_BASICSOLUTION    lp->epsvalue  /* Zero-rounding of RHS/basic solution vector */
#else
  #define PREC_BASICSOLUTION    lp->epsmachine  /* Zero-rounding of RHS/basic solution vector */
#endif
#define LIMIT_ABS_REL         10.0  /* Limit for testing using relative metric */


/* Parameters constants for short-cut setting of tolerances                           */
/* -------------------------------------------------------------------------------------- */
#define EPS_TIGHT                0
#define EPS_MEDIUM               1
#define EPS_LOOSE                2
#define EPS_BAGGY                3
#define EPS_DEFAULT              EPS_TIGHT


#if ActivePARAM==ProductionPARAM    /* PARAMETER SET FOR PRODUCTION                       */
/* -------------------------------------------------------------------------------------- */
#define DEF_INFINITE       1.0e+30  /* Limit for dynamic range */
#define DEF_EPSVALUE       1.0e-12  /* High accuracy and feasibility preserving tolerance */
#define DEF_EPSPRIMAL      1.0e-10  /* For rounding primal/RHS values to 0 */
#define DEF_EPSDUAL        1.0e-09  /* For rounding reduced costs to 0 */
#define DEF_EPSPIVOT       2.0e-07  /* Pivot reject threshold */
#define DEF_PERTURB        1.0e-05  /* Perturbation scalar for degenerate problems;
                                       must at least be RANDSCALE greater than EPSPRIMAL */
#define DEF_EPSSOLUTION    1.0e-05  /* Margin of error for solution bounds */
#define DEF_EPSINT         1.0e-07  /* Accuracy for considering a float value as integer */

#elif ActivePARAM==OriginalPARAM    /* PARAMETER SET FOR LEGACY VERSIONS                  */
/* -------------------------------------------------------------------------------------- */
#define DEF_INFINITE       1.0e+24  /* Limit for dynamic range */
#define DEF_EPSVALUE       1.0e-08  /* High accuracy and feasibility preserving tolerance */
#define DEF_EPSPRIMAL     5.01e-07  /* For rounding primal/RHS values to 0, infeasibility */
#define DEF_EPSDUAL        1.0e-06  /* For rounding reduced costs to 0 */
#define DEF_EPSPIVOT       1.0e-04  /* Pivot reject threshold */
#define DEF_PERTURB        1.0e-05  /* Perturbation scalar for degenerate problems;
                                       must at least be RANDSCALE greater than EPSPRIMAL */
#define DEF_EPSSOLUTION    1.0e-02  /* Margin of error for solution bounds */
#define DEF_EPSINT         1.0e-03  /* Accuracy for considering a float value as integer */

#elif ActivePARAM==ChvatalPARAM     /* PARAMETER SET EXAMPLES FROM Vacek Chvatal          */
/* -------------------------------------------------------------------------------------- */
#define DEF_INFINITE       1.0e+30  /* Limit for dynamic range */
#define DEF_EPSVALUE       1.0e-10  /* High accuracy and feasibility preserving tolerance */
#define DEF_EPSPRIMAL       10e-07  /* For rounding primal/RHS values to 0 */
#define DEF_EPSDUAL         10e-05  /* For rounding reduced costs to 0 */
#define DEF_EPSPIVOT        10e-05  /* Pivot reject threshold */
#define DEF_PERTURB         10e-03  /* Perturbation scalar for degenerate problems;
                                       must at least be RANDSCALE greater than EPSPRIMAL */
#define DEF_EPSSOLUTION    1.0e-05  /* Margin of error for solution bounds */
#define DEF_EPSINT         5.0e-03  /* Accuracy for considering a float value as integer */

#elif ActivePARAM==LoosePARAM       /* PARAMETER SET FOR LOOSE TOLERANCES                 */
/* -------------------------------------------------------------------------------------- */
#define DEF_INFINITE       1.0e+30  /* Limit for dynamic range */
#define DEF_EPSVALUE       1.0e-10  /* High accuracy and feasibility preserving tolerance */
#define DEF_EPSPRIMAL     5.01e-08  /* For rounding primal/RHS values to 0 */
#define DEF_EPSDUAL        1.0e-07  /* For rounding reduced costs to 0 */
#define DEF_EPSPIVOT       1.0e-05  /* Pivot reject threshold */
#define DEF_PERTURB        1.0e-05  /* Perturbation scalar for degenerate problems;
                                       must at least be RANDSCALE greater than EPSPRIMAL */
#define DEF_EPSSOLUTION    1.0e-05  /* Margin of error for solution bounds */
#define DEF_EPSINT         1.0e-04  /* Accuracy for considering a float value as integer */

#endif


#define DEF_MIP_GAP        1.0e-11  /* The default absolute and relative MIP gap */
#define SCALEDINTFIXRANGE      1.6  /* Epsilon range multiplier < 2 for collapsing bounds to fix */

#define MIN_SCALAR         1.0e-10  /* Smallest allowed scaling adjustment */
#define MAX_SCALAR         1.0e+10  /* Largest allowed scaling adjustment */
#define DEF_SCALINGEPS     1.0e-02  /* Relative scaling convergence criterion for auto_scale */

#define DEF_LAGACCEPT      1.0e-03  /* Default Lagrangean convergence acceptance criterion */
#define DEF_LAGCONTRACT       0.90  /* The contraction parameter for Lagrangean iterations */
#define DEF_LAGMAXITERATIONS   100  /* The maximum number of Lagrangean iterations */

#define DEF_PSEUDOCOSTUPDATES    7  /* The default number of times pseudo-costs are recalculated;
                                       experiments indicate that costs tend to stabilize */
#define DEF_PSEUDOCOSTRESTART 0.15  /* The fraction of price updates required for B&B restart
                                       when the mode is NODE_RESTARTMODE */
#define DEF_MAXPRESOLVELOOPS     0  /* Upper limit to the number of loops during presolve,
                                       <= 0 for no limit. */


/* Hashing prototypes and function headers                                   */
/* ------------------------------------------------------------------------- */
#include "lp_Hash.h"


/* Sparse matrix prototypes                                                  */
/* ------------------------------------------------------------------------- */
#include "lp_matrix.h"


/* Basis storage (mainly for B&B) */
typedef struct _basisrec
{
  int       level;
  int       *var_basic;
  MYBOOL    *is_basic;
  MYBOOL    *is_lower;
  int       pivots;
  struct   _basisrec *previous;
} basisrec;

/* Presolve undo data storage */
typedef struct _presolveundorec
{
  lprec     *lp;
  int       orig_rows;
  int       orig_columns;
  int       orig_sum;
  int       *var_to_orig;       /* sum_alloc+1 : Mapping of variables from solution to
                                   best_solution to account for removed variables and
                                   rows during presolve; a non-positive value indicates
                                   that the constraint or variable was removed */
  int       *orig_to_var;       /* sum_alloc+1 : Mapping from original variable index to
                                   current / working index number */
  REAL      *fixed_rhs;         /* rows_alloc+1 : Storage of values of presolved fixed colums */
  REAL      *fixed_obj;         /* columns_alloc+1: Storage of values of presolved fixed rows */
  DeltaVrec *deletedA;          /* A matrix of eliminated data from matA */
  DeltaVrec *primalundo;        /* Affine translation vectors for eliminated primal variables */
  DeltaVrec *dualundo;          /* Affine translation vectors for eliminated dual variables */
  MYBOOL    OFcolsdeleted;
} presolveundorec;

/* Pseudo-cost arrays used during B&B */
typedef struct _BBPSrec
{
  lprec     *lp;
  int       pseodotype;
  int       updatelimit;
  int       updatesfinished;
  REAL      restartlimit;
  MATitem   *UPcost;
  MATitem   *LOcost;
  struct   _BBPSrec *secondary;
} BBPSrec;

#include "lp_mipbb.h"


/* Partial pricing block data */
typedef struct _partialrec {
  lprec     *lp;
  int       blockcount;         /* ## The number of logical blocks or stages in the model */
  int       blocknow;           /* The currently active block */
  int       *blockend;          /* Array of column indeces giving the start of each block */
  int       *blockpos;          /* Array of column indeces giving the start scan position */
  MYBOOL    isrow;
} partialrec;


/* Specially Ordered Sets (SOS) prototypes and settings                      */
/* ------------------------------------------------------------------------- */
/* SOS storage structure (LINEARSEARCH is typically in the 0-10 range)       */
#ifndef LINEARSEARCH
#define LINEARSEARCH 0
#endif

#include "lp_SOS.h"


/* Prototypes for user call-back functions                                   */
/* ------------------------------------------------------------------------- */
typedef int    (__WINAPI lphandle_intfunc)(lprec *lp, void *userhandle);
typedef void   (__WINAPI lphandlestr_func)(lprec *lp, void *userhandle, char *buf);
typedef void   (__WINAPI lphandleint_func)(lprec *lp, void *userhandle, int message);
typedef int    (__WINAPI lphandleint_intfunc)(lprec *lp, void *userhandle, int message);


/* API typedef definitions                                                   */
/* ------------------------------------------------------------------------- */
typedef MYBOOL (__WINAPI add_column_func)(lprec *lp, REAL *column);
typedef MYBOOL (__WINAPI add_columnex_func)(lprec *lp, int count, REAL *column, int *rowno);
typedef MYBOOL (__WINAPI add_constraint_func)(lprec *lp, REAL *row, int constr_type, REAL rh);
typedef MYBOOL (__WINAPI add_constraintex_func)(lprec *lp, int count, REAL *row, int *colno, int constr_type, REAL rh);
typedef MYBOOL (__WINAPI add_lag_con_func)(lprec *lp, REAL *row, int con_type, REAL rhs);
typedef int (__WINAPI add_SOS_func)(lprec *lp, char *name, int sostype, int priority, int count, int *sosvars, REAL *weights);
typedef int (__WINAPI column_in_lp_func)(lprec *lp, REAL *column);
typedef lprec * (__WINAPI copy_lp_func)(lprec *lp);
typedef void (__WINAPI default_basis_func)(lprec *lp);
typedef MYBOOL (__WINAPI del_column_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI del_constraint_func)(lprec *lp, int rownr);
typedef void (__WINAPI delete_lp_func)(lprec *lp);
typedef MYBOOL (__WINAPI dualize_lp_func)(lprec *lp);
typedef void (__WINAPI free_lp_func)(lprec **plp);
typedef int (__WINAPI get_anti_degen_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_basis_func)(lprec *lp, int *bascolumn, MYBOOL nonbasic);
typedef int (__WINAPI get_basiscrash_func)(lprec *lp);
typedef int (__WINAPI get_bb_depthlimit_func)(lprec *lp);
typedef int (__WINAPI get_bb_floorfirst_func)(lprec *lp);
typedef int (__WINAPI get_bb_rule_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_bounds_tighter_func)(lprec *lp);
typedef REAL (__WINAPI get_break_at_value_func)(lprec *lp);
typedef char * (__WINAPI get_col_name_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI get_column_func)(lprec *lp, int colnr, REAL *column);
typedef int (__WINAPI get_columnex_func)(lprec *lp, int colnr, REAL *column, int *nzrow);
typedef int (__WINAPI get_constr_type_func)(lprec *lp, int rownr);
typedef REAL (__WINAPI get_constr_value_func)(lprec *lp, int rownr, int count, REAL *primsolution, int *nzindex);
typedef MYBOOL (__WINAPI get_constraints_func)(lprec *lp, REAL *constr);
typedef MYBOOL (__WINAPI get_dual_solution_func)(lprec *lp, REAL *rc);
typedef REAL (__WINAPI get_epsb_func)(lprec *lp);
typedef REAL (__WINAPI get_epsd_func)(lprec *lp);
typedef REAL (__WINAPI get_epsel_func)(lprec *lp);
typedef REAL (__WINAPI get_epsint_func)(lprec *lp);
typedef REAL (__WINAPI get_epsperturb_func)(lprec *lp);
typedef REAL (__WINAPI get_epspivot_func)(lprec *lp);
typedef int (__WINAPI get_improve_func)(lprec *lp);
typedef REAL (__WINAPI get_infinite_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_lambda_func)(lprec *lp, REAL *lambda);
typedef REAL (__WINAPI get_lowbo_func)(lprec *lp, int colnr);
typedef int (__WINAPI get_lp_index_func)(lprec *lp, int orig_index);
typedef char * (__WINAPI get_lp_name_func)(lprec *lp);
typedef int (__WINAPI get_Lrows_func)(lprec *lp);
typedef REAL (__WINAPI get_mat_func)(lprec *lp, int rownr, int colnr);
typedef REAL (__WINAPI get_mat_byindex_func)(lprec *lp, int matindex, MYBOOL isrow, MYBOOL adjustsign);
typedef int (__WINAPI get_max_level_func)(lprec *lp);
typedef int (__WINAPI get_maxpivot_func)(lprec *lp);
typedef REAL (__WINAPI get_mip_gap_func)(lprec *lp, MYBOOL absolute);
typedef int (__WINAPI get_multiprice_func)(lprec *lp, MYBOOL getabssize);
typedef MYBOOL (__WINAPI is_use_names_func)(lprec *lp, MYBOOL isrow);
typedef void (__WINAPI set_use_names_func)(lprec *lp, MYBOOL isrow, MYBOOL use_names);
typedef int (__WINAPI get_nameindex_func)(lprec *lp, char *varname, MYBOOL isrow);
typedef int (__WINAPI get_Ncolumns_func)(lprec *lp);
typedef REAL (__WINAPI get_negrange_func)(lprec *lp);
typedef int (__WINAPI get_nz_func)(lprec *lp);
typedef int (__WINAPI get_Norig_columns_func)(lprec *lp);
typedef int (__WINAPI get_Norig_rows_func)(lprec *lp);
typedef int (__WINAPI get_Nrows_func)(lprec *lp);
typedef REAL (__WINAPI get_obj_bound_func)(lprec *lp);
typedef REAL (__WINAPI get_objective_func)(lprec *lp);
typedef int (__WINAPI get_orig_index_func)(lprec *lp, int lp_index);
typedef char * (__WINAPI get_origcol_name_func)(lprec *lp, int colnr);
typedef char * (__WINAPI get_origrow_name_func)(lprec *lp, int rownr);
typedef void (__WINAPI get_partialprice_func)(lprec *lp, int *blockcount, int *blockstart, MYBOOL isrow);
typedef int (__WINAPI get_pivoting_func)(lprec *lp);
typedef int (__WINAPI get_presolve_func)(lprec *lp);
typedef int (__WINAPI get_presolveloops_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_primal_solution_func)(lprec *lp, REAL *pv);
typedef int (__WINAPI get_print_sol_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_pseudocosts_func)(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit);
typedef MYBOOL (__WINAPI get_ptr_constraints_func)(lprec *lp, REAL **constr);
typedef MYBOOL (__WINAPI get_ptr_dual_solution_func)(lprec *lp, REAL **rc);
typedef MYBOOL (__WINAPI get_ptr_lambda_func)(lprec *lp, REAL **lambda);
typedef MYBOOL (__WINAPI get_ptr_primal_solution_func)(lprec *lp, REAL **pv);
typedef MYBOOL (__WINAPI get_ptr_sensitivity_obj_func)(lprec *lp, REAL **objfrom, REAL **objtill);
typedef MYBOOL (__WINAPI get_ptr_sensitivity_objex_func)(lprec *lp, REAL **objfrom, REAL **objtill, REAL **objfromvalue, REAL **objtillvalue);
typedef MYBOOL (__WINAPI get_ptr_sensitivity_rhs_func)(lprec *lp, REAL **duals, REAL **dualsfrom, REAL **dualstill);
typedef MYBOOL (__WINAPI get_ptr_variables_func)(lprec *lp, REAL **var);
typedef REAL (__WINAPI get_rh_func)(lprec *lp, int rownr);
typedef REAL (__WINAPI get_rh_range_func)(lprec *lp, int rownr);
typedef int (__WINAPI get_rowex_func)(lprec *lp, int rownr, REAL *row, int *colno);
typedef MYBOOL (__WINAPI get_row_func)(lprec *lp, int rownr, REAL *row);
typedef char * (__WINAPI get_row_name_func)(lprec *lp, int rownr);
typedef REAL (__WINAPI get_scalelimit_func)(lprec *lp);
typedef int (__WINAPI get_scaling_func)(lprec *lp);
typedef MYBOOL (__WINAPI get_sensitivity_obj_func)(lprec *lp, REAL *objfrom, REAL *objtill);
typedef MYBOOL (__WINAPI get_sensitivity_objex_func)(lprec *lp, REAL *objfrom, REAL *objtill, REAL *objfromvalue, REAL *objtillvalue);
typedef MYBOOL (__WINAPI get_sensitivity_rhs_func)(lprec *lp, REAL *duals, REAL *dualsfrom, REAL *dualstill);
typedef int (__WINAPI get_simplextype_func)(lprec *lp);
typedef int (__WINAPI get_solutioncount_func)(lprec *lp);
typedef int (__WINAPI get_solutionlimit_func)(lprec *lp);
typedef int (__WINAPI get_status_func)(lprec *lp);
typedef char * (__WINAPI get_statustext_func)(lprec *lp, int statuscode);
typedef long (__WINAPI get_timeout_func)(lprec *lp);
typedef COUNTER (__WINAPI get_total_iter_func)(lprec *lp);
typedef COUNTER (__WINAPI get_total_nodes_func)(lprec *lp);
typedef REAL (__WINAPI get_upbo_func)(lprec *lp, int colnr);
typedef int (__WINAPI get_var_branch_func)(lprec *lp, int colnr);
typedef REAL (__WINAPI get_var_dualresult_func)(lprec *lp, int index);
typedef REAL (__WINAPI get_var_primalresult_func)(lprec *lp, int index);
typedef int (__WINAPI get_var_priority_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI get_variables_func)(lprec *lp, REAL *var);
typedef int (__WINAPI get_verbose_func)(lprec *lp);
typedef MYBOOL (__WINAPI guess_basis_func)(lprec *lp, REAL *guessvector, int *basisvector);
typedef REAL (__WINAPI get_working_objective_func)(lprec *lp);
typedef MYBOOL (__WINAPI has_BFP_func)(lprec *lp);
typedef MYBOOL (__WINAPI has_XLI_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_add_rowmode_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_anti_degen_func)(lprec *lp, int testmask);
typedef MYBOOL (__WINAPI is_binary_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI is_break_at_first_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_constr_type_func)(lprec *lp, int rownr, int mask);
typedef MYBOOL (__WINAPI is_debug_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_feasible_func)(lprec *lp, REAL *values, REAL threshold);
typedef MYBOOL (__WINAPI is_unbounded_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI is_infinite_func)(lprec *lp, REAL value);
typedef MYBOOL (__WINAPI is_int_func)(lprec *lp, int column);
typedef MYBOOL (__WINAPI is_integerscaling_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_lag_trace_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_maxim_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_nativeBFP_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_nativeXLI_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_negative_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI is_obj_in_basis_func)(lprec *lp);
typedef MYBOOL (__WINAPI is_piv_mode_func)(lprec *lp, int testmask);
typedef MYBOOL (__WINAPI is_piv_rule_func)(lprec *lp, int rule);
typedef MYBOOL (__WINAPI is_presolve_func)(lprec *lp, int testmask);
typedef MYBOOL (__WINAPI is_scalemode_func)(lprec *lp, int testmask);
typedef MYBOOL (__WINAPI is_scaletype_func)(lprec *lp, int scaletype);
typedef MYBOOL (__WINAPI is_semicont_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI is_SOS_var_func)(lprec *lp, int colnr);
typedef MYBOOL (__WINAPI is_trace_func)(lprec *lp);
typedef void (__WINAPI lp_solve_version_func)(int *majorversion, int *minorversion, int *release, int *build);
typedef lprec * (__WINAPI make_lp_func)(int rows, int columns);
typedef void (__WINAPI print_constraints_func)(lprec *lp, int columns);
typedef MYBOOL (__WINAPI print_debugdump_func)(lprec *lp, char *filename);
typedef void (__WINAPI print_duals_func)(lprec *lp);
typedef void (__WINAPI print_lp_func)(lprec *lp);
typedef void (__WINAPI print_objective_func)(lprec *lp);
typedef void (__WINAPI print_scales_func)(lprec *lp);
typedef void (__WINAPI print_solution_func)(lprec *lp, int columns);
typedef void (__WINAPI print_str_func)(lprec *lp, char *str);
typedef void (__WINAPI print_tableau_func)(lprec *lp);
typedef void (__WINAPI put_abortfunc_func)(lprec *lp, lphandle_intfunc newctrlc, void *ctrlchandle);
typedef void (__WINAPI put_bb_nodefunc_func)(lprec *lp, lphandleint_intfunc newnode, void *bbnodehandle);
typedef void (__WINAPI put_bb_branchfunc_func)(lprec *lp, lphandleint_intfunc newbranch, void *bbbranchhandle);
typedef void (__WINAPI put_logfunc_func)(lprec *lp, lphandlestr_func newlog, void *loghandle);
typedef void (__WINAPI put_msgfunc_func)(lprec *lp, lphandleint_func newmsg, void *msghandle, int mask);
typedef MYBOOL (__WINAPI read_LPhandle_func)(lprec **lp, FILE *filehandle, int verbose, char *lp_name);
typedef MYBOOL (__WINAPI read_MPShandle_func)(lprec **lp, FILE *filehandle, int typeMPS, int verbose);
typedef lprec * (__WINAPI read_XLI_func)(char *xliname, char *modelname, char *dataname, char *options, int verbose);
typedef MYBOOL (__WINAPI read_basis_func)(lprec *lp, char *filename, char *info);
typedef void (__WINAPI reset_basis_func)(lprec *lp);
typedef MYBOOL (__WINAPI read_params_func)(lprec *lp, char *filename, char *options);
typedef void (__WINAPI reset_params_func)(lprec *lp);
typedef MYBOOL (__WINAPI resize_lp_func)(lprec *lp, int rows, int columns);
typedef MYBOOL (__WINAPI set_add_rowmode_func)(lprec *lp, MYBOOL turnon);
typedef void (__WINAPI set_anti_degen_func)(lprec *lp, int anti_degen);
typedef int  (__WINAPI set_basisvar_func)(lprec *lp, int basisPos, int enteringCol);
typedef MYBOOL (__WINAPI set_basis_func)(lprec *lp, int *bascolumn, MYBOOL nonbasic);
typedef void (__WINAPI set_basiscrash_func)(lprec *lp, int mode);
typedef void (__WINAPI set_bb_depthlimit_func)(lprec *lp, int bb_maxlevel);
typedef void (__WINAPI set_bb_floorfirst_func)(lprec *lp, int bb_floorfirst);
typedef void (__WINAPI set_bb_rule_func)(lprec *lp, int bb_rule);
typedef MYBOOL (__WINAPI set_BFP_func)(lprec *lp, char *filename);
typedef MYBOOL (__WINAPI set_binary_func)(lprec *lp, int colnr, MYBOOL must_be_bin);
typedef MYBOOL (__WINAPI set_bounds_func)(lprec *lp, int colnr, REAL lower, REAL upper);
typedef void (__WINAPI set_bounds_tighter_func)(lprec *lp, MYBOOL tighten);
typedef void (__WINAPI set_break_at_first_func)(lprec *lp, MYBOOL break_at_first);
typedef void (__WINAPI set_break_at_value_func)(lprec *lp, REAL break_at_value);
typedef MYBOOL (__WINAPI set_column_func)(lprec *lp, int colnr, REAL *column);
typedef MYBOOL (__WINAPI set_columnex_func)(lprec *lp, int colnr, int count, REAL *column, int *rowno);
typedef MYBOOL (__WINAPI set_col_name_func)(lprec *lp, int colnr, char *new_name);
typedef MYBOOL (__WINAPI set_constr_type_func)(lprec *lp, int rownr, int con_type);
typedef void (__WINAPI set_debug_func)(lprec *lp, MYBOOL debug);
typedef void (__WINAPI set_epsb_func)(lprec *lp, REAL epsb);
typedef void (__WINAPI set_epsd_func)(lprec *lp, REAL epsd);
typedef void (__WINAPI set_epsel_func)(lprec *lp, REAL epsel);
typedef void (__WINAPI set_epsint_func)(lprec *lp, REAL epsint);
typedef MYBOOL (__WINAPI set_epslevel_func)(lprec *lp, int epslevel);
typedef void (__WINAPI set_epsperturb_func)(lprec *lp, REAL epsperturb);
typedef void (__WINAPI set_epspivot_func)(lprec *lp, REAL epspivot);
typedef MYBOOL (__WINAPI set_unbounded_func)(lprec *lp, int colnr);
typedef void (__WINAPI set_improve_func)(lprec *lp, int improve);
typedef void (__WINAPI set_infinite_func)(lprec *lp, REAL infinite);
typedef MYBOOL (__WINAPI set_int_func)(lprec *lp, int colnr, MYBOOL must_be_int);
typedef void (__WINAPI set_lag_trace_func)(lprec *lp, MYBOOL lag_trace);
typedef MYBOOL (__WINAPI set_lowbo_func)(lprec *lp, int colnr, REAL value);
typedef MYBOOL (__WINAPI set_lp_name_func)(lprec *lp, char *lpname);
typedef MYBOOL (__WINAPI set_mat_func)(lprec *lp, int row, int column, REAL value);
typedef void (__WINAPI set_maxim_func)(lprec *lp);
typedef void (__WINAPI set_maxpivot_func)(lprec *lp, int max_num_inv);
typedef void (__WINAPI set_minim_func)(lprec *lp);
typedef void (__WINAPI set_mip_gap_func)(lprec *lp, MYBOOL absolute, REAL mip_gap);
typedef MYBOOL (__WINAPI set_multiprice_func)(lprec *lp, int multiblockdiv);
typedef void (__WINAPI set_negrange_func)(lprec *lp, REAL negrange);
typedef MYBOOL (__WINAPI set_obj_func)(lprec *lp, int colnr, REAL value);
typedef void (__WINAPI set_obj_bound_func)(lprec *lp, REAL obj_bound);
typedef MYBOOL (__WINAPI set_obj_fn_func)(lprec *lp, REAL *row);
typedef MYBOOL (__WINAPI set_obj_fnex_func)(lprec *lp, int count, REAL *row, int *colno);
typedef void (__WINAPI set_obj_in_basis_func)(lprec *lp, MYBOOL obj_in_basis);
typedef MYBOOL (__WINAPI set_outputfile_func)(lprec *lp, char *filename);
typedef void (__WINAPI set_outputstream_func)(lprec *lp, FILE *stream);
typedef MYBOOL (__WINAPI set_partialprice_func)(lprec *lp, int blockcount, int *blockstart, MYBOOL isrow);
typedef void (__WINAPI set_pivoting_func)(lprec *lp, int piv_rule);
typedef void (__WINAPI set_preferdual_func)(lprec *lp, MYBOOL dodual);
typedef void (__WINAPI set_presolve_func)(lprec *lp, int presolvemode, int maxloops);
typedef void (__WINAPI set_print_sol_func)(lprec *lp, int print_sol);
typedef MYBOOL (__WINAPI set_pseudocosts_func)(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit);
typedef MYBOOL (__WINAPI set_rh_func)(lprec *lp, int rownr, REAL value);
typedef MYBOOL (__WINAPI set_rh_range_func)(lprec *lp, int rownr, REAL deltavalue);
typedef void (__WINAPI set_rh_vec_func)(lprec *lp, REAL *rh);
typedef MYBOOL (__WINAPI set_row_func)(lprec *lp, int rownr, REAL *row);
typedef MYBOOL (__WINAPI set_rowex_func)(lprec *lp, int rownr, int count, REAL *row, int *colno);
typedef MYBOOL (__WINAPI set_row_name_func)(lprec *lp, int rownr, char *new_name);
typedef void (__WINAPI set_scalelimit_func)(lprec *lp, REAL scalelimit);
typedef void (__WINAPI set_scaling_func)(lprec *lp, int scalemode);
typedef MYBOOL (__WINAPI set_semicont_func)(lprec *lp, int colnr, MYBOOL must_be_sc);
typedef void (__WINAPI set_sense_func)(lprec *lp, MYBOOL maximize);
typedef void (__WINAPI set_simplextype_func)(lprec *lp, int simplextype);
typedef void (__WINAPI set_solutionlimit_func)(lprec *lp, int limit);
typedef void (__WINAPI set_timeout_func)(lprec *lp, long sectimeout);
typedef void (__WINAPI set_trace_func)(lprec *lp, MYBOOL trace);
typedef MYBOOL (__WINAPI set_upbo_func)(lprec *lp, int colnr, REAL value);
typedef MYBOOL (__WINAPI set_var_branch_func)(lprec *lp, int colnr, int branch_mode);
typedef MYBOOL (__WINAPI set_var_weights_func)(lprec *lp, REAL *weights);
typedef void (__WINAPI set_verbose_func)(lprec *lp, int verbose);
typedef MYBOOL (__WINAPI set_XLI_func)(lprec *lp, char *filename);
typedef int (__WINAPI solve_func)(lprec *lp);
typedef MYBOOL (__WINAPI str_add_column_func)(lprec *lp, char *col_string);
typedef MYBOOL (__WINAPI str_add_constraint_func)(lprec *lp, char *row_string ,int constr_type, REAL rh);
typedef MYBOOL (__WINAPI str_add_lag_con_func)(lprec *lp, char *row_string, int con_type, REAL rhs);
typedef MYBOOL (__WINAPI str_set_obj_fn_func)(lprec *lp, char *row_string);
typedef MYBOOL (__WINAPI str_set_rh_vec_func)(lprec *lp, char *rh_string);
typedef REAL (__WINAPI time_elapsed_func)(lprec *lp);
typedef void (__WINAPI unscale_func)(lprec *lp);
typedef MYBOOL (__WINAPI write_lp_func)(lprec *lp, char *filename);
typedef MYBOOL (__WINAPI write_LP_func)(lprec *lp, FILE *output);
typedef MYBOOL (__WINAPI write_mps_func)(lprec *lp, char *filename);
typedef MYBOOL (__WINAPI write_MPS_func)(lprec *lp, FILE *output);
typedef MYBOOL (__WINAPI write_freemps_func)(lprec *lp, char *filename);
typedef MYBOOL (__WINAPI write_freeMPS_func)(lprec *lp, FILE *output);
typedef MYBOOL (__WINAPI write_XLI_func)(lprec *lp, char *filename, char *options, MYBOOL results);
typedef MYBOOL (__WINAPI write_basis_func)(lprec *lp, char *filename);
typedef MYBOOL (__WINAPI write_params_func)(lprec *lp, char *filename, char *options);


/* Prototypes for callbacks from basis inverse/factorization libraries       */
/* ------------------------------------------------------------------------- */
typedef MYBOOL (__WINAPI userabortfunc)(lprec *lp, int level);
typedef void   (__VACALL reportfunc)(lprec *lp, int level, char *format, ...);
typedef char * (__VACALL explainfunc)(lprec *lp, char *format, ...);
typedef int    (__WINAPI getvectorfunc)(lprec *lp, int varin, REAL *pcol, int *nzlist, int *maxabs);
typedef int    (__WINAPI getpackedfunc)(lprec *lp, int j, int rn[], double bj[]);
typedef REAL    (__WINAPI get_OF_activefunc)(lprec *lp, int varnr, REAL mult);
typedef int    (__WINAPI getMDOfunc)(lprec *lp, MYBOOL *usedpos, int *colorder, int *size, MYBOOL symmetric);
typedef MYBOOL (__WINAPI invertfunc)(lprec *lp, MYBOOL shiftbounds, MYBOOL final);
typedef void   (__WINAPI set_actionfunc)(int *actionvar, int actionmask);
typedef MYBOOL (__WINAPI is_actionfunc)(int actionvar, int testmask);
typedef void   (__WINAPI clear_actionfunc)(int *actionvar, int actionmask);


/* Prototypes for basis inverse/factorization libraries                      */
/* ------------------------------------------------------------------------- */
typedef char   *(BFP_CALLMODEL BFPchar)(void);
typedef void   (BFP_CALLMODEL BFP_lp)(lprec *lp);
typedef void   (BFP_CALLMODEL BFP_lpint)(lprec *lp, int newsize);
typedef int    (BFP_CALLMODEL BFPint_lp)(lprec *lp);
typedef int    (BFP_CALLMODEL BFPint_lpint)(lprec *lp, int kind);
typedef REAL   (BFP_CALLMODEL BFPreal_lp)(lprec *lp);
typedef REAL   *(BFP_CALLMODEL BFPrealp_lp)(lprec *lp);
typedef void   (BFP_CALLMODEL BFP_lpbool)(lprec *lp, MYBOOL maximum);
typedef int    (BFP_CALLMODEL BFPint_lpbool)(lprec *lp, MYBOOL maximum);
typedef int    (BFP_CALLMODEL BFPint_lpintintboolbool)(lprec *lp, int uservars, int Bsize, MYBOOL *usedpos, MYBOOL final);
typedef void   (BFP_CALLMODEL BFP_lprealint)(lprec *lp, REAL *pcol, int *nzidx);
typedef void   (BFP_CALLMODEL BFP_lprealintrealint)(lprec *lp, REAL *prow, int *pnzidx, REAL *drow, int *dnzidx);
typedef MYBOOL (BFP_CALLMODEL BFPbool_lp)(lprec *lp);
typedef MYBOOL (BFP_CALLMODEL BFPbool_lpbool)(lprec *lp, MYBOOL changesign);
typedef MYBOOL (BFP_CALLMODEL BFPbool_lpint)(lprec *lp, int size);
typedef MYBOOL (BFP_CALLMODEL BFPbool_lpintintchar)(lprec *lp, int size, int deltasize, char *options);
typedef MYBOOL (BFP_CALLMODEL BFPbool_lpintintint)(lprec *lp, int size, int deltasize, int sizeofvar);
typedef LREAL  (BFP_CALLMODEL BFPlreal_lpintintreal)(lprec *lp, int row_nr, int col_nr, REAL *pcol);
typedef REAL   (BFP_CALLMODEL BFPreal_lplrealreal)(lprec *lp, LREAL theta, REAL *pcol);

typedef int    (BFP_CALLMODEL getcolumnex_func)(lprec *lp, int colnr, REAL *nzvalues, int *nzrows, int *mapin);
typedef int    (BFP_CALLMODEL BFPint_lpintrealcbintint)(lprec *lp, int items, getcolumnex_func cb, int *maprow, int*mapcol);

/* Prototypes for external language libraries                                */
/* ------------------------------------------------------------------------- */
typedef char   *(XLI_CALLMODEL XLIchar)(void);
typedef MYBOOL (XLI_CALLMODEL XLIbool_lpintintint)(lprec* lp, int size, int deltasize, int sizevar);
typedef MYBOOL (XLI_CALLMODEL XLIbool_lpcharcharcharint)(lprec *lp, char *modelname, char *dataname, char *options, int verbose);
typedef MYBOOL (XLI_CALLMODEL XLIbool_lpcharcharbool)(lprec *lp, char *filename, char *options, MYBOOL results);


/* Main lp_solve prototypes and function definitions                         */
/* ------------------------------------------------------------------------- */
struct _lprec
{
  /* Full list of exported functions made available in a quasi object-oriented fashion */
  add_column_func               *add_column;
  add_columnex_func             *add_columnex;
  add_constraint_func           *add_constraint;
  add_constraintex_func         *add_constraintex;
  add_lag_con_func              *add_lag_con;
  add_SOS_func                  *add_SOS;
  column_in_lp_func             *column_in_lp;
  copy_lp_func                  *copy_lp;
  default_basis_func            *default_basis;
  del_column_func               *del_column;
  del_constraint_func           *del_constraint;
  delete_lp_func                *delete_lp;
  dualize_lp_func               *dualize_lp;
  free_lp_func                  *free_lp;
  get_anti_degen_func           *get_anti_degen;
  get_basis_func                *get_basis;
  get_basiscrash_func           *get_basiscrash;
  get_bb_depthlimit_func        *get_bb_depthlimit;
  get_bb_floorfirst_func        *get_bb_floorfirst;
  get_bb_rule_func              *get_bb_rule;
  get_bounds_tighter_func       *get_bounds_tighter;
  get_break_at_value_func       *get_break_at_value;
  get_col_name_func             *get_col_name;
  get_columnex_func             *get_columnex;
  get_constr_type_func          *get_constr_type;
  get_constr_value_func         *get_constr_value;
  get_constraints_func          *get_constraints;
  get_dual_solution_func        *get_dual_solution;
  get_epsb_func                 *get_epsb;
  get_epsd_func                 *get_epsd;
  get_epsel_func                *get_epsel;
  get_epsint_func               *get_epsint;
  get_epsperturb_func           *get_epsperturb;
  get_epspivot_func             *get_epspivot;
  get_improve_func              *get_improve;
  get_infinite_func             *get_infinite;
  get_lambda_func               *get_lambda;
  get_lowbo_func                *get_lowbo;
  get_lp_index_func             *get_lp_index;
  get_lp_name_func              *get_lp_name;
  get_Lrows_func                *get_Lrows;
  get_mat_func                  *get_mat;
  get_mat_byindex_func          *get_mat_byindex;
  get_max_level_func            *get_max_level;
  get_maxpivot_func             *get_maxpivot;
  get_mip_gap_func              *get_mip_gap;
  get_multiprice_func           *get_multiprice;
  get_nameindex_func            *get_nameindex;
  get_Ncolumns_func             *get_Ncolumns;
  get_negrange_func             *get_negrange;
  get_nz_func                   *get_nonzeros;
  get_Norig_columns_func        *get_Norig_columns;
  get_Norig_rows_func           *get_Norig_rows;
  get_Nrows_func                *get_Nrows;
  get_obj_bound_func            *get_obj_bound;
  get_objective_func            *get_objective;
  get_orig_index_func           *get_orig_index;
  get_origcol_name_func         *get_origcol_name;
  get_origrow_name_func         *get_origrow_name;
  get_partialprice_func         *get_partialprice;
  get_pivoting_func             *get_pivoting;
  get_presolve_func             *get_presolve;
  get_presolveloops_func        *get_presolveloops;
  get_primal_solution_func      *get_primal_solution;
  get_print_sol_func            *get_print_sol;
  get_pseudocosts_func          *get_pseudocosts;
  get_ptr_constraints_func      *get_ptr_constraints;
  get_ptr_dual_solution_func    *get_ptr_dual_solution;
  get_ptr_lambda_func           *get_ptr_lambda;
  get_ptr_primal_solution_func  *get_ptr_primal_solution;
  get_ptr_sensitivity_obj_func  *get_ptr_sensitivity_obj;
  get_ptr_sensitivity_objex_func *get_ptr_sensitivity_objex;
  get_ptr_sensitivity_rhs_func  *get_ptr_sensitivity_rhs;
  get_ptr_variables_func        *get_ptr_variables;
  get_rh_func                   *get_rh;
  get_rh_range_func             *get_rh_range;
  get_row_func                  *get_row;
  get_rowex_func                *get_rowex;
  get_row_name_func             *get_row_name;
  get_scalelimit_func           *get_scalelimit;
  get_scaling_func              *get_scaling;
  get_sensitivity_obj_func      *get_sensitivity_obj;
  get_sensitivity_objex_func    *get_sensitivity_objex;
  get_sensitivity_rhs_func      *get_sensitivity_rhs;
  get_simplextype_func          *get_simplextype;
  get_solutioncount_func        *get_solutioncount;
  get_solutionlimit_func        *get_solutionlimit;
  get_status_func               *get_status;
  get_statustext_func           *get_statustext;
  get_timeout_func              *get_timeout;
  get_total_iter_func           *get_total_iter;
  get_total_nodes_func          *get_total_nodes;
  get_upbo_func                 *get_upbo;
  get_var_branch_func           *get_var_branch;
  get_var_dualresult_func       *get_var_dualresult;
  get_var_primalresult_func     *get_var_primalresult;
  get_var_priority_func         *get_var_priority;
  get_variables_func            *get_variables;
  get_verbose_func              *get_verbose;
  get_working_objective_func    *get_working_objective;
  has_BFP_func                  *has_BFP;
  has_XLI_func                  *has_XLI;
  is_add_rowmode_func           *is_add_rowmode;
  is_anti_degen_func            *is_anti_degen;
  is_binary_func                *is_binary;
  is_break_at_first_func        *is_break_at_first;
  is_constr_type_func           *is_constr_type;
  is_debug_func                 *is_debug;
  is_feasible_func              *is_feasible;
  is_infinite_func              *is_infinite;
  is_int_func                   *is_int;
  is_integerscaling_func        *is_integerscaling;
  is_lag_trace_func             *is_lag_trace;
  is_maxim_func                 *is_maxim;
  is_nativeBFP_func             *is_nativeBFP;
  is_nativeXLI_func             *is_nativeXLI;
  is_negative_func              *is_negative;
  is_obj_in_basis_func          *is_obj_in_basis;
  is_piv_mode_func              *is_piv_mode;
  is_piv_rule_func              *is_piv_rule;
  is_presolve_func              *is_presolve;
  is_scalemode_func             *is_scalemode;
  is_scaletype_func             *is_scaletype;
  is_semicont_func              *is_semicont;
  is_SOS_var_func               *is_SOS_var;
  is_trace_func                 *is_trace;
  is_unbounded_func             *is_unbounded;
  is_use_names_func             *is_use_names;
  lp_solve_version_func         *lp_solve_version;
  make_lp_func                  *make_lp;
  print_constraints_func        *print_constraints;
  print_debugdump_func          *print_debugdump;
  print_duals_func              *print_duals;
  print_lp_func                 *print_lp;
  print_objective_func          *print_objective;
  print_scales_func             *print_scales;
  print_solution_func           *print_solution;
  print_str_func                *print_str;
  print_tableau_func            *print_tableau;
  put_abortfunc_func            *put_abortfunc;
  put_bb_nodefunc_func          *put_bb_nodefunc;
  put_bb_branchfunc_func        *put_bb_branchfunc;
  put_logfunc_func              *put_logfunc;
  put_msgfunc_func              *put_msgfunc;
  read_LPhandle_func            *read_LPhandle;
  read_MPShandle_func           *read_MPShandle;
  read_XLI_func                 *read_XLI;
  read_params_func              *read_params;
  read_basis_func               *read_basis;
  reset_basis_func              *reset_basis;
  reset_params_func             *reset_params;
  resize_lp_func                *resize_lp;
  set_add_rowmode_func          *set_add_rowmode;
  set_anti_degen_func           *set_anti_degen;
  set_basisvar_func             *set_basisvar;
  set_basis_func                *set_basis;
  set_basiscrash_func           *set_basiscrash;
  set_bb_depthlimit_func        *set_bb_depthlimit;
  set_bb_floorfirst_func        *set_bb_floorfirst;
  set_bb_rule_func              *set_bb_rule;
  set_BFP_func                  *set_BFP;
  set_binary_func               *set_binary;
  set_bounds_func               *set_bounds;
  set_bounds_tighter_func       *set_bounds_tighter;
  set_break_at_first_func       *set_break_at_first;
  set_break_at_value_func       *set_break_at_value;
  set_column_func               *set_column;
  set_columnex_func             *set_columnex;
  set_col_name_func             *set_col_name;
  set_constr_type_func          *set_constr_type;
  set_debug_func                *set_debug;
  set_epsb_func                 *set_epsb;
  set_epsd_func                 *set_epsd;
  set_epsel_func                *set_epsel;
  set_epsint_func               *set_epsint;
  set_epslevel_func             *set_epslevel;
  set_epsperturb_func           *set_epsperturb;
  set_epspivot_func             *set_epspivot;
  set_unbounded_func            *set_unbounded;
  set_improve_func              *set_improve;
  set_infinite_func             *set_infinite;
  set_int_func                  *set_int;
  set_lag_trace_func            *set_lag_trace;
  set_lowbo_func                *set_lowbo;
  set_lp_name_func              *set_lp_name;
  set_mat_func                  *set_mat;
  set_maxim_func                *set_maxim;
  set_maxpivot_func             *set_maxpivot;
  set_minim_func                *set_minim;
  set_mip_gap_func              *set_mip_gap;
  set_multiprice_func           *set_multiprice;
  set_negrange_func             *set_negrange;
  set_obj_bound_func            *set_obj_bound;
  set_obj_fn_func               *set_obj_fn;
  set_obj_fnex_func             *set_obj_fnex;
  set_obj_func                  *set_obj;
  set_obj_in_basis_func         *set_obj_in_basis;
  set_outputfile_func           *set_outputfile;
  set_outputstream_func         *set_outputstream;
  set_partialprice_func         *set_partialprice;
  set_pivoting_func             *set_pivoting;
  set_preferdual_func           *set_preferdual;
  set_presolve_func             *set_presolve;
  set_print_sol_func            *set_print_sol;
  set_pseudocosts_func          *set_pseudocosts;
  set_rh_func                   *set_rh;
  set_rh_range_func             *set_rh_range;
  set_rh_vec_func               *set_rh_vec;
  set_row_func                  *set_row;
  set_rowex_func                *set_rowex;
  set_row_name_func             *set_row_name;
  set_scalelimit_func           *set_scalelimit;
  set_scaling_func              *set_scaling;
  set_semicont_func             *set_semicont;
  set_sense_func                *set_sense;
  set_simplextype_func          *set_simplextype;
  set_solutionlimit_func        *set_solutionlimit;
  set_timeout_func              *set_timeout;
  set_trace_func                *set_trace;
  set_upbo_func                 *set_upbo;
  set_use_names_func            *set_use_names;
  set_var_branch_func           *set_var_branch;
  set_var_weights_func          *set_var_weights;
  set_verbose_func              *set_verbose;
  set_XLI_func                  *set_XLI;
  solve_func                    *solve;
  str_add_column_func           *str_add_column;
  str_add_constraint_func       *str_add_constraint;
  str_add_lag_con_func          *str_add_lag_con;
  str_set_obj_fn_func           *str_set_obj_fn;
  str_set_rh_vec_func           *str_set_rh_vec;
  time_elapsed_func             *time_elapsed;
  unscale_func                  *unscale;
  write_lp_func                 *write_lp;
  write_LP_func                 *write_LP;
  write_mps_func                *write_mps;
  write_MPS_func                *write_MPS;
  write_freemps_func            *write_freemps;
  write_freeMPS_func            *write_freeMPS;
  write_XLI_func                *write_XLI;
  write_basis_func              *write_basis;
  write_params_func             *write_params;

  /* Spacer */
  int       *alignmentspacer;

  /* Problem description */
  char      *lp_name;           /* The name of the model */

  /* Problem sizes */
  int       sum;                /* The total number of variables, including slacks */
  int       rows;
  int       columns;
  int       equalities;         /* No of non-Lagrangean equality constraints in the problem */
  int       boundedvars;        /* Count of bounded variables */
  int       INTfuture1;

  /* Memory allocation sizes */
  int       sum_alloc;          /* The allocated memory for row+column-sized data */
  int       rows_alloc;         /* The allocated memory for row-sized data */
  int       columns_alloc;      /* The allocated memory for column-sized data */

  /* Model status and solver result variables */
  MYBOOL    source_is_file;     /* The base model was read from a file */
  MYBOOL    model_is_pure;      /* The model has been built entirely from row and column additions */
  MYBOOL    model_is_valid;     /* Has this lp pased the 'test' */
  MYBOOL    tighten_on_set;     /* Specify if bounds will be tightened or overriden at bound setting */
  MYBOOL    names_used;         /* Flag to indicate if names for rows and columns are used */
  MYBOOL    use_row_names;      /* Flag to indicate if names for rows are used */
  MYBOOL    use_col_names;      /* Flag to indicate if names for columns are used */

  MYBOOL    lag_trace;          /* Print information on Lagrange progression */
  MYBOOL    spx_trace;          /* Print information on simplex progression */
  MYBOOL    bb_trace;           /* TRUE to print extra debug information */
  MYBOOL    streamowned;        /* TRUE if the handle should be closed at delete_lp() */
  MYBOOL    obj_in_basis;       /* TRUE if the objective function is in the basis matrix */

  int       spx_status;         /* Simplex solver feasibility/mode code */
  int       lag_status;         /* Extra status variable for lag_solve */
  int       solutioncount;      /* number of equal-valued solutions found (up to solutionlimit) */
  int       solutionlimit;      /* upper number of equal-valued solutions kept track of */

  REAL      real_solution;      /* Optimal non-MIP solution base */
  REAL      *solution;          /* sum_alloc+1 : Solution array of the next to optimal LP,
                                   Index   0           : Objective function value,
                                   Indeces 1..rows     : Slack variable values,
                                   Indeced rows+1..sum : Variable values */
  REAL      *best_solution;     /* sum_alloc+1 : Solution array of optimal 'Integer' LP,
                                   structured as the solution array above */
  REAL      *full_solution;     /* sum_alloc+1 : Final solution array expanded for deleted variables */
  REAL      *edgeVector;        /* Array of reduced cost scaling norms (DEVEX and Steepest Edge) */

  REAL      *drow;              /* sum+1: Reduced costs of the last simplex */
  int       *nzdrow;            /* sum+1: Indeces of non-zero reduced costs of the last simplex */
  REAL      *duals;             /* rows_alloc+1 : The dual variables of the last LP */
  REAL      *full_duals;        /* sum_alloc+1: Final duals array expanded for deleted variables */
  REAL      *dualsfrom;         /* sum_alloc+1 :The sensitivity on dual variables/reduced costs
                                   of the last LP */
  REAL      *dualstill;         /* sum_alloc+1 :The sensitivity on dual variables/reduced costs
                                   of the last LP */
  REAL      *objfrom;           /* columns_alloc+1 :The sensitivity on objective function
                                   of the last LP */
  REAL      *objtill;           /* columns_alloc+1 :The sensitivity on objective function
                                   of the last LP */
  REAL      *objfromvalue;      /* columns_alloc+1 :The value of the variables when objective value
                                   is at its from value of the last LP */
  REAL      *orig_obj;          /* Unused pointer - Placeholder for OF not part of B */
  REAL      *obj;               /* Special vector used to temporarily change the OF vector */

  COUNTER   current_iter;       /* Number of iterations in the current/last simplex */
  COUNTER   total_iter;         /* Number of iterations over all B&B steps */
  COUNTER   current_bswap;      /* Number of bound swaps in the current/last simplex */
  COUNTER   total_bswap;        /* Number of bount swaps over all B&B steps */
  int       solvecount;         /* The number of solve() performed in this model */
  int       max_pivots;         /* Number of pivots between refactorizations of the basis */

  /* Various execution parameters */
  int       simplex_strategy;   /* Set desired combination of primal and dual simplex algorithms */
  int       simplex_mode;       /* Specifies the current simplex mode during solve; see simplex_strategy */
  int       verbose;            /* Set amount of run-time messages and results */
  int       print_sol;          /* TRUE to print optimal solution; AUTOMATIC skips zeros */
  FILE      *outstream;         /* Output stream, initialized to STDOUT */

  /* Main Branch and Bound settings */
  MYBOOL    *bb_varbranch;      /* Determines branching strategy at the individual variable level;
                                   the setting here overrides the bb_floorfirst setting */
  int       piv_strategy;       /* Strategy for selecting row and column entering/leaving */
  int       _piv_rule_;         /* Internal working rule-part of piv_strategy above */
  int       bb_rule;            /* Rule for selecting B&B variables */
  MYBOOL    bb_floorfirst;      /* Set BRANCH_FLOOR for B&B to set variables to floor bound first;
                                   conversely with BRANCH_CEILING, the ceiling value is set first */
  MYBOOL    bb_breakfirst;      /* TRUE to stop at first feasible solution */
  MYBOOL    _piv_left_;         /* Internal variable indicating active pricing loop order */
  MYBOOL    BOOLfuture1;

  REAL      scalelimit;         /* Relative convergence criterion for iterated scaling */
  int       scalemode;          /* OR-ed codes for data scaling */
  int       improve;            /* Set to non-zero for iterative improvement */
  int       anti_degen;         /* Anti-degen strategy (or none) TRUE to avoid cycling */
  int       do_presolve;        /* PRESOLVE_ parameters for LP presolving */
  int       presolveloops;      /* Maximum number of presolve loops */

  int       perturb_count;      /* The number of bound relaxation retries performed */

  /* Row and column names storage variables */
  hashelem  **row_name;         /* rows_alloc+1 */
  hashelem  **col_name;         /* columns_alloc+1 */
  hashtable *rowname_hashtab;   /* hash table to store row names */
  hashtable *colname_hashtab;   /* hash table to store column names */

  /* Optionally specify continuous rows/column blocks for partial pricing */
  partialrec *rowblocks;
  partialrec *colblocks;

  /* Row and column type codes */
  MYBOOL    *var_type;          /* sum_alloc+1 : TRUE if variable must be integer */

  /* Data for multiple pricing */
  multirec  *multivars;
  int       multiblockdiv;      /* The divisor used to set or augment pricing block */

  /* Variable (column) parameters */
  int       fixedvars;          /* The current number of basic fixed variables in the model */
  int       int_vars;           /* Number of variables required to be integer */

  int       sc_vars;            /* Number of semi-continuous variables */
  REAL      *sc_lobound;        /* sum_columns+1 : TRUE if variable is semi-continuous;
                                   value replaced by conventional lower bound during solve */
  int       *var_is_free;       /* columns+1: Index of twin variable if variable is free */
  int       *var_priority;      /* columns: Priority-mapping of variables */

  SOSgroup  *GUB;               /* Pointer to record containing GUBs */

  int       sos_vars;           /* Number of variables in the sos_priority list */
  int       sos_ints;           /* Number of integers in SOS'es above */
  SOSgroup  *SOS;               /* Pointer to record containing all SOS'es */
  int       *sos_priority;      /* Priority-sorted list of variables (no duplicates) */

  /* Optionally specify list of active rows/columns used in multiple pricing */
  REAL      *bsolveVal;         /* rows+1: bsolved solution vector for reduced costs */
  int       *bsolveIdx;         /* rows+1: Non-zero indeces of bsolveVal */

  /* RHS storage */
  REAL      *orig_rhs;          /* rows_alloc+1 : The RHS after scaling and sign
                                   changing, but before 'Bound transformation' */
  LREAL     *rhs;               /* rows_alloc+1 : The RHS of the current simplex tableau */

  /* Row (constraint) parameters */
  int       *row_type;          /* rows_alloc+1 : Row/constraint type coding */

  /* Optionally specify data for dual long-step */
  multirec  *longsteps;

  /* Original and working row and variable bounds */
  REAL      *orig_upbo;         /* sum_alloc+1 : Bound before transformations */
  REAL      *upbo;              /*  " " : Upper bound after transformation and B&B work */
  REAL      *orig_lowbo;        /*  "       "                                 */
  REAL      *lowbo;             /*  " " : Lower bound after transformation and B&B work */

  /* User data and basis factorization matrices (ETA or LU, product form) */
  MATrec    *matA;
  INVrec    *invB;

  /* Basis and bounds */
  BBrec     *bb_bounds;         /* The linked list of B&B bounds */
  BBrec     *rootbounds;        /* The bounds at the lowest B&B level */
  basisrec  *bb_basis;          /* The linked list of B&B bases */
  basisrec  *rootbasis;
  OBJmonrec *monitor;           /* Objective monitoring record for stalling/degeneracy handling */

  /* Scaling parameters */
  REAL      *scalars;           /* sum_alloc+1:0..Rows the scaling of the rows,
                                   Rows+1..Sum the scaling of the columns */
  MYBOOL    scaling_used;       /* TRUE if scaling is used */
  MYBOOL    columns_scaled;     /* TRUE if the columns are scaled too */
  MYBOOL    varmap_locked;      /* Determines whether the var_to_orig and orig_to_var are fixed */

  /* Variable state information */
  MYBOOL    basis_valid;        /* TRUE is the basis is still valid */
  int       crashmode;          /* Basis crashing mode (or none) */
  int       *var_basic;         /* rows_alloc+1: The list of columns in the basis */
  REAL      *val_nonbasic;      /* Array to store current values of non-basic variables */
  MYBOOL    *is_basic;          /* sum_alloc+1: TRUE if the column is in the basis */
  MYBOOL    *is_lower;          /*  "       " : TRUE if the variable is at its
                                   lower bound (or in the basis), FALSE otherwise */

  /* Simplex basis indicators */
  int       *rejectpivot;       /* List of unacceptable pivot choices due to division-by-zero */
  BBPSrec   *bb_PseudoCost;     /* Data structure for costing of node branchings */
  int       bb_PseudoUpdates;   /* Maximum number of updates for pseudo-costs */
  int       bb_strongbranches;  /* The number of strong B&B branches performed */
  int       is_strongbranch;    /* Are we currently in a strong branch mode? */
  int       bb_improvements;    /* The number of discrete B&B objective improvement steps */

  /* Solver working variables */
  REAL      rhsmax;             /* The maximum |value| of the rhs vector at any iteration */
  REAL      suminfeas;          /* The working sum of primal and dual infeasibilities */
  REAL      bigM;               /* Original objective weighting in primal phase 1 */
  REAL      P1extraVal;         /* Phase 1 OF/RHS offset for feasibility */
  int       P1extraDim;         /* Phase 1 additional columns/rows for feasibility */
  int       spx_action;         /* ACTION_ variables for the simplex routine */
  MYBOOL    spx_perturbed;      /* The variable bounds were relaxed/perturbed into this simplex */
  MYBOOL    bb_break;           /* Solver working variable; signals break of the B&B */
  MYBOOL    wasPreprocessed;    /* The solve preprocessing was performed */
  MYBOOL    wasPresolved;       /* The solve presolver was invoked */
  int      INTfuture2;

  /* Lagragean solver storage and parameters */
  MATrec    *matL;
  REAL      *lag_rhs;           /* Array of Lagrangean rhs vector */
  int       *lag_con_type;      /* Array of GT, LT or EQ */
  REAL      *lambda;            /* Lambda values (Lagrangean multipliers) */
  REAL      lag_bound;          /* The Lagrangian lower OF bound */
  REAL      lag_accept;         /* The Lagrangian convergence criterion */

  /* Solver thresholds */
  REAL      infinite;           /* Limit for dynamic range */
  REAL      negrange;           /* Limit for negative variable range */
  REAL      epsmachine;         /* Default machine accuracy */
  REAL      epsvalue;           /* Input data precision / rounding of data values to 0 */
  REAL      epsprimal;          /* For rounding RHS values to 0/infeasibility */
  REAL      epsdual;            /* For rounding reduced costs to zero */
  REAL      epspivot;           /* Pivot reject tolerance */
  REAL      epsperturb;         /* Perturbation scalar */
  REAL      epssolution;        /* The solution tolerance for final validation */

  /* Branch & Bound working parameters */
  int       bb_status;          /* Indicator that the last solvelp() gave an improved B&B solution */
  int       bb_level;           /* Solver B&B working variable (recursion depth) */
  int       bb_maxlevel;        /* The deepest B&B level of the last solution */
  int       bb_limitlevel;      /* The maximum B&B level allowed */
  COUNTER   bb_totalnodes;      /* Total number of nodes processed in B&B */
  int       bb_solutionlevel;   /* The B&B level of the last / best solution */
  int       bb_cutpoolsize;     /* Size of the B&B cut pool */
  int       bb_cutpoolused;     /* Currently used cut pool */
  int       bb_constraintOF;    /* General purpose B&B parameter (typically for testing) */
  int       *bb_cuttype;        /* The type of the currently used cuts */
  int       *bb_varactive;      /* The B&B state of the variable; 0 means inactive */
  DeltaVrec *bb_upperchange;    /* Changes to upper bounds during the B&B phase */
  DeltaVrec *bb_lowerchange;    /* Changes to lower bounds during the B&B phase */

  REAL      bb_deltaOF;         /* Minimum OF step value; computed at beginning of solve() */

  REAL      bb_breakOF;         /* User-settable value for the objective function deemed
                               to be sufficiently good in an integer problem */
  REAL      bb_limitOF;         /* "Dual" bound / limit to final optimal MIP solution */
  REAL      bb_heuristicOF;     /* Set initial "at least better than" guess for objective function
                               (can significantly speed up B&B iterations) */
  REAL      bb_parentOF;        /* The OF value of the previous BB simplex */
  REAL      bb_workOF;          /* The unadjusted OF value for the current best solution */

  /* Internal work arrays allocated as required */
  presolveundorec *presolve_undo;
  workarraysrec   *workarrays;

  /* MIP parameters */
  REAL      epsint;             /* Margin of error in determining if a float value is integer */
  REAL      mip_absgap;         /* Absolute MIP gap */
  REAL      mip_relgap;         /* Relative MIP gap */

  /* Time/timer variables and extended status text */
  double    timecreate;
  double    timestart;
  double    timeheuristic;
  double    timepresolved;
  double    timeend;
  long      sectimeout;

  /* Extended status message text set via explain() */
  char      *ex_status;

  /* Refactorization engine interface routines (for dynamic DLL/SO BFPs) */
#if LoadInverseLib == TRUE
  #ifdef WIN32
    HINSTANCE                   hBFP;
  #else
    void                        *hBFP;
  #endif
#endif
  BFPchar                       *bfp_name;
  BFPbool_lpintintint           *bfp_compatible;
  BFPbool_lpintintchar          *bfp_init;
  BFP_lp                        *bfp_free;
  BFPbool_lpint                 *bfp_resize;
  BFPint_lp                     *bfp_memallocated;
  BFPbool_lp                    *bfp_restart;
  BFPbool_lp                    *bfp_mustrefactorize;
  BFPint_lp                     *bfp_preparefactorization;
  BFPint_lpintintboolbool       *bfp_factorize;
  BFP_lp                        *bfp_finishfactorization;
  BFP_lp                        *bfp_updaterefactstats;
  BFPlreal_lpintintreal         *bfp_prepareupdate;
  BFPreal_lplrealreal           *bfp_pivotRHS;
  BFPbool_lpbool                *bfp_finishupdate;
  BFP_lprealint                 *bfp_ftran_prepare;
  BFP_lprealint                 *bfp_ftran_normal;
  BFP_lprealint                 *bfp_btran_normal;
  BFP_lprealintrealint          *bfp_btran_double;
  BFPint_lp                     *bfp_status;
  BFPint_lpbool                 *bfp_nonzeros;
  BFPbool_lp                    *bfp_implicitslack;
  BFPint_lp                     *bfp_indexbase;
  BFPint_lp                     *bfp_rowoffset;
  BFPint_lp                     *bfp_pivotmax;
  BFPbool_lpint                 *bfp_pivotalloc;
  BFPint_lp                     *bfp_colcount;
  BFPbool_lp                    *bfp_canresetbasis;
  BFPreal_lp                    *bfp_efficiency;
  BFPrealp_lp                   *bfp_pivotvector;
  BFPint_lp                     *bfp_pivotcount;
  BFPint_lpint                  *bfp_refactcount;
  BFPbool_lp                    *bfp_isSetI;
  BFPint_lpintrealcbintint      *bfp_findredundant;

  /* External language interface routines (for dynamic DLL/SO XLIs) */
#if LoadLanguageLib == TRUE
  #ifdef WIN32
    HINSTANCE                   hXLI;
  #else
    void                        *hXLI;
  #endif
#endif
  XLIchar                       *xli_name;
  XLIbool_lpintintint           *xli_compatible;
  XLIbool_lpcharcharcharint     *xli_readmodel;
  XLIbool_lpcharcharbool        *xli_writemodel;

  /* Miscellaneous internal functions made available externally */
  userabortfunc                 *userabort;
  reportfunc                    *report;
  explainfunc                   *explain;
  getvectorfunc                 *get_lpcolumn;
  getpackedfunc                 *get_basiscolumn;
  get_OF_activefunc             *get_OF_active;
  getMDOfunc                    *getMDO;
  invertfunc                    *invert;
  set_actionfunc                *set_action;
  is_actionfunc                 *is_action;
  clear_actionfunc              *clear_action;

  /* User program interface callbacks */
  lphandle_intfunc              *ctrlc;
    void                          *ctrlchandle;     /* User-specified "owner process ID" */
  lphandlestr_func              *writelog;
    void                          *loghandle;       /* User-specified "owner process ID" */
  lphandlestr_func              *debuginfo;
  lphandleint_func              *usermessage;
    int                           msgmask;
    void                          *msghandle;       /* User-specified "owner process ID" */
  lphandleint_intfunc           *bb_usenode;
    void                          *bb_nodehandle;   /* User-specified "owner process ID" */
  lphandleint_intfunc           *bb_usebranch;
    void                          *bb_branchhandle; /* User-specified "owner process ID" */

};


#ifdef __cplusplus
__EXTERN_C {
#endif


/* User and system function interfaces                                       */
/* ------------------------------------------------------------------------- */

void __EXPORT_TYPE __WINAPI lp_solve_version(int *majorversion, int *minorversion, int *release, int *build);

lprec __EXPORT_TYPE * __WINAPI make_lp(int rows, int columns);
MYBOOL __EXPORT_TYPE __WINAPI resize_lp(lprec *lp, int rows, int columns);
int __EXPORT_TYPE __WINAPI get_status(lprec *lp);
char __EXPORT_TYPE * __WINAPI get_statustext(lprec *lp, int statuscode);
MYBOOL __EXPORT_TYPE __WINAPI is_obj_in_basis(lprec *lp);
void __EXPORT_TYPE __WINAPI set_obj_in_basis(lprec *lp, MYBOOL obj_in_basis);
/* Create and initialise a lprec structure defaults */

lprec __EXPORT_TYPE * __WINAPI copy_lp(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI dualize_lp(lprec *lp);
STATIC MYBOOL memopt_lp(lprec *lp, int rowextra, int colextra, int nzextra);
/* Copy or dualize the lp */

void __EXPORT_TYPE __WINAPI delete_lp(lprec *lp);
void __EXPORT_TYPE __WINAPI free_lp(lprec **plp);
/* Remove problem from memory */

MYBOOL __EXPORT_TYPE __WINAPI set_lp_name(lprec *lp, char *lpname);
char __EXPORT_TYPE * __WINAPI get_lp_name(lprec *lp);
/* Set and get the problem name */

MYBOOL __EXPORT_TYPE __WINAPI has_BFP(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_nativeBFP(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI set_BFP(lprec *lp, char *filename);
/* Set basis factorization engine */

lprec __EXPORT_TYPE * __WINAPI read_XLI(char *xliname, char *modelname, char *dataname, char *options, int verbose);
MYBOOL __EXPORT_TYPE __WINAPI write_XLI(lprec *lp, char *filename, char *options, MYBOOL results);
MYBOOL __EXPORT_TYPE __WINAPI has_XLI(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_nativeXLI(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI set_XLI(lprec *lp, char *filename);
/* Set external language interface */

MYBOOL __EXPORT_TYPE __WINAPI set_obj(lprec *lp, int colnr, REAL value);
MYBOOL __EXPORT_TYPE __WINAPI set_obj_fn(lprec *lp, REAL *row);
MYBOOL __EXPORT_TYPE __WINAPI set_obj_fnex(lprec *lp, int count, REAL *row, int *colno);
/* set the objective function (Row 0) of the matrix */
MYBOOL __EXPORT_TYPE __WINAPI str_set_obj_fn(lprec *lp, char *row_string);
/* The same, but with string input */
void __EXPORT_TYPE __WINAPI set_sense(lprec *lp, MYBOOL maximize);
void __EXPORT_TYPE __WINAPI set_maxim(lprec *lp);
void __EXPORT_TYPE __WINAPI set_minim(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_maxim(lprec *lp);
/* Set optimization direction for the objective function */

MYBOOL __EXPORT_TYPE __WINAPI add_constraint(lprec *lp, REAL *row, int constr_type, REAL rh);
MYBOOL __EXPORT_TYPE __WINAPI add_constraintex(lprec *lp, int count, REAL *row, int *colno, int constr_type, REAL rh);
MYBOOL __EXPORT_TYPE __WINAPI set_add_rowmode(lprec *lp, MYBOOL turnon);
MYBOOL __EXPORT_TYPE __WINAPI is_add_rowmode(lprec *lp);
/* Add a constraint to the problem, row is the constraint row, rh is the right hand side,
   constr_type is the type of constraint (LE (<=), GE(>=), EQ(=)) */
MYBOOL __EXPORT_TYPE __WINAPI str_add_constraint(lprec *lp, char *row_string, int constr_type, REAL rh);
/* The same, but with string input */

MYBOOL __EXPORT_TYPE __WINAPI set_row(lprec *lp, int rownr, REAL *row);
MYBOOL __EXPORT_TYPE __WINAPI set_rowex(lprec *lp, int rownr, int count, REAL *row, int *colno);
MYBOOL __EXPORT_TYPE __WINAPI get_row(lprec *lp, int rownr, REAL *row);
int __EXPORT_TYPE __WINAPI get_rowex(lprec *lp, int rownr, REAL *row, int *colno);
/* Fill row with the row row_nr from the problem */

MYBOOL __EXPORT_TYPE __WINAPI del_constraint(lprec *lp, int rownr);
STATIC MYBOOL del_constraintex(lprec *lp, LLrec *rowmap);
/* Remove constrain nr del_row from the problem */

MYBOOL __EXPORT_TYPE __WINAPI add_lag_con(lprec *lp, REAL *row, int con_type, REAL rhs);
/* add a Lagrangian constraint of form Row' x contype Rhs */
MYBOOL __EXPORT_TYPE __WINAPI str_add_lag_con(lprec *lp, char *row_string, int con_type, REAL rhs);
/* The same, but with string input */
void __EXPORT_TYPE __WINAPI set_lag_trace(lprec *lp, MYBOOL lag_trace);
MYBOOL __EXPORT_TYPE __WINAPI is_lag_trace(lprec *lp);
/* Set debugging/tracing mode of the Lagrangean solver */

MYBOOL __EXPORT_TYPE __WINAPI set_constr_type(lprec *lp, int rownr, int con_type);
int __EXPORT_TYPE __WINAPI get_constr_type(lprec *lp, int rownr);
REAL __EXPORT_TYPE __WINAPI get_constr_value(lprec *lp, int rownr, int count, REAL *primsolution, int *nzindex);
MYBOOL __EXPORT_TYPE __WINAPI is_constr_type(lprec *lp, int rownr, int mask);
STATIC char *get_str_constr_type(lprec *lp, int con_type);
STATIC int get_constr_class(lprec *lp, int rownr);
STATIC char *get_str_constr_class(lprec *lp, int con_class);
/* Set the type of constraint in row Row (LE, GE, EQ) */

MYBOOL __EXPORT_TYPE __WINAPI set_rh(lprec *lp, int rownr, REAL value);
REAL __EXPORT_TYPE __WINAPI get_rh(lprec *lp, int rownr);
/* Set and get the right hand side of a constraint row */
MYBOOL __EXPORT_TYPE __WINAPI set_rh_range(lprec *lp, int rownr, REAL deltavalue);
REAL __EXPORT_TYPE __WINAPI get_rh_range(lprec *lp, int rownr);
/* Set the RHS range; i.e. the lower and upper bounds of a constraint row */
void __EXPORT_TYPE __WINAPI set_rh_vec(lprec *lp, REAL *rh);
/* Set the right hand side vector */
MYBOOL __EXPORT_TYPE __WINAPI str_set_rh_vec(lprec *lp, char *rh_string);
/* The same, but with string input */

MYBOOL __EXPORT_TYPE __WINAPI add_column(lprec *lp, REAL *column);
MYBOOL __EXPORT_TYPE __WINAPI add_columnex(lprec *lp, int count, REAL *column, int *rowno);
MYBOOL __EXPORT_TYPE __WINAPI str_add_column(lprec *lp, char *col_string);
/* Add a column to the problem */

MYBOOL __EXPORT_TYPE __WINAPI set_column(lprec *lp, int colnr, REAL *column);
MYBOOL __EXPORT_TYPE __WINAPI set_columnex(lprec *lp, int colnr, int count, REAL *column, int *rowno);
/* Overwrite existing column data */

int __EXPORT_TYPE __WINAPI column_in_lp(lprec *lp, REAL *column);
/* Returns the column index if column is already present in lp, otherwise 0.
   (Does not look at bounds and types, only looks at matrix values */

int __EXPORT_TYPE __WINAPI get_columnex(lprec *lp, int colnr, REAL *column, int *nzrow);
MYBOOL __EXPORT_TYPE __WINAPI get_column(lprec *lp, int colnr, REAL *column);
/* Fill column with the column col_nr from the problem */

MYBOOL __EXPORT_TYPE __WINAPI del_column(lprec *lp, int colnr);
STATIC MYBOOL del_columnex(lprec *lp, LLrec *colmap);
/* Delete a column */

MYBOOL __EXPORT_TYPE __WINAPI set_mat(lprec *lp, int rownr, int colnr, REAL value);
/* Fill in element (Row,Column) of the matrix
   Row in [0..Rows] and Column in [1..Columns] */
REAL __EXPORT_TYPE __WINAPI get_mat(lprec *lp, int rownr, int colnr);
REAL __EXPORT_TYPE __WINAPI get_mat_byindex(lprec *lp, int matindex, MYBOOL isrow, MYBOOL adjustsign);
int __EXPORT_TYPE __WINAPI get_nonzeros(lprec *lp);
/* get a single element from the matrix */  /* Name changed from "mat_elm" by KE */

void __EXPORT_TYPE __WINAPI set_bounds_tighter(lprec *lp, MYBOOL tighten);
MYBOOL get_bounds(lprec *lp, int column, REAL *lower, REAL *upper);
MYBOOL __EXPORT_TYPE __WINAPI get_bounds_tighter(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI set_upbo(lprec *lp, int colnr, REAL value);
REAL __EXPORT_TYPE __WINAPI get_upbo(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI set_lowbo(lprec *lp, int colnr, REAL value);
REAL __EXPORT_TYPE __WINAPI get_lowbo(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI set_bounds(lprec *lp, int colnr, REAL lower, REAL upper);
MYBOOL __EXPORT_TYPE __WINAPI set_unbounded(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI is_unbounded(lprec *lp, int colnr);
/* Set the upper and lower bounds of a variable */

MYBOOL __EXPORT_TYPE __WINAPI set_int(lprec *lp, int colnr, MYBOOL must_be_int);
MYBOOL __EXPORT_TYPE __WINAPI is_int(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI set_binary(lprec *lp, int colnr, MYBOOL must_be_bin);
MYBOOL __EXPORT_TYPE __WINAPI is_binary(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI set_semicont(lprec *lp, int colnr, MYBOOL must_be_sc);
MYBOOL __EXPORT_TYPE __WINAPI is_semicont(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI is_negative(lprec *lp, int colnr);
MYBOOL __EXPORT_TYPE __WINAPI set_var_weights(lprec *lp, REAL *weights);
int __EXPORT_TYPE __WINAPI get_var_priority(lprec *lp, int colnr);
/* Set the type of variable */

MYBOOL __EXPORT_TYPE __WINAPI set_pseudocosts(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit);
MYBOOL __EXPORT_TYPE __WINAPI get_pseudocosts(lprec *lp, REAL *clower, REAL *cupper, int *updatelimit);
/* Set initial values for, or get computed pseudocost vectors;
   note that setting of pseudocosts can only happen in response to a
   call-back function optionally requesting this */

int  __EXPORT_TYPE __WINAPI add_SOS(lprec *lp, char *name, int sostype, int priority, int count, int *sosvars, REAL *weights);
MYBOOL __EXPORT_TYPE __WINAPI is_SOS_var(lprec *lp, int colnr);
/* Add SOS constraints */

MYBOOL __EXPORT_TYPE __WINAPI set_row_name(lprec *lp, int rownr, char *new_name);
char __EXPORT_TYPE * __WINAPI get_row_name(lprec *lp, int rownr);
char __EXPORT_TYPE * __WINAPI get_origrow_name(lprec *lp, int rownr);
/* Set/Get the name of a constraint row */   /* Get added by KE */

MYBOOL __EXPORT_TYPE __WINAPI set_col_name(lprec *lp, int colnr, char *new_name);
char __EXPORT_TYPE * __WINAPI get_col_name(lprec *lp, int colnr);
char __EXPORT_TYPE * __WINAPI get_origcol_name(lprec *lp, int colnr);
/* Set/Get the name of a variable column */  /* Get added by KE */

void __EXPORT_TYPE __WINAPI unscale(lprec *lp);
/* Undo previous scaling of the problem */

void __EXPORT_TYPE __WINAPI set_preferdual(lprec *lp, MYBOOL dodual);
void __EXPORT_TYPE __WINAPI set_simplextype(lprec *lp, int simplextype);
int __EXPORT_TYPE __WINAPI get_simplextype(lprec *lp);
/* Set/Get if lp_solve should prefer the dual simplex over the primal -- added by KE */

void __EXPORT_TYPE __WINAPI default_basis(lprec *lp);
void __EXPORT_TYPE __WINAPI set_basiscrash(lprec *lp, int mode);
int __EXPORT_TYPE __WINAPI get_basiscrash(lprec *lp);
int __EXPORT_TYPE __WINAPI set_basisvar(lprec *lp, int basisPos, int enteringCol);
MYBOOL __EXPORT_TYPE __WINAPI set_basis(lprec *lp, int *bascolumn, MYBOOL nonbasic);
MYBOOL __EXPORT_TYPE __WINAPI get_basis(lprec *lp, int *bascolumn, MYBOOL nonbasic);
void __EXPORT_TYPE __WINAPI reset_basis(lprec *lp);
/* Set/Get basis for a re-solved system */  /* Added by KE */
MYBOOL __EXPORT_TYPE __WINAPI guess_basis(lprec *lp, REAL *guessvector, int *basisvector);

MYBOOL __EXPORT_TYPE __WINAPI is_feasible(lprec *lp, REAL *values, REAL threshold);
/* returns TRUE if the vector in values is a feasible solution to the lp */

int __EXPORT_TYPE __WINAPI solve(lprec *lp);
/* Solve the problem */

REAL __EXPORT_TYPE __WINAPI time_elapsed(lprec *lp);
/* Return the number of seconds since start of solution process */

void __EXPORT_TYPE __WINAPI put_bb_nodefunc(lprec *lp, lphandleint_intfunc newnode, void *bbnodehandle);
void __EXPORT_TYPE __WINAPI put_bb_branchfunc(lprec *lp, lphandleint_intfunc newbranch, void *bbbranchhandle);
/* Allow the user to override B&B node and branching decisions */

void __EXPORT_TYPE __WINAPI put_abortfunc(lprec *lp, lphandle_intfunc newctrlc, void *ctrlchandle);
/* Allow the user to define an interruption callback function */

void __EXPORT_TYPE __WINAPI put_logfunc(lprec *lp, lphandlestr_func newlog, void *loghandle);
/* Allow the user to define a logging function */

void __EXPORT_TYPE __WINAPI put_msgfunc(lprec *lp, lphandleint_func newmsg, void *msghandle, int mask);
/* Allow the user to define an event-driven message/reporting */

MYBOOL __EXPORT_TYPE __WINAPI get_primal_solution(lprec *lp, REAL *pv);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_primal_solution(lprec *lp, REAL **pv);
MYBOOL __EXPORT_TYPE __WINAPI get_dual_solution(lprec *lp, REAL *rc);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_dual_solution(lprec *lp, REAL **rc);
MYBOOL __EXPORT_TYPE __WINAPI get_lambda(lprec *lp, REAL *lambda);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_lambda(lprec *lp, REAL **lambda);
/* Get the primal, dual/reduced costs and Lambda vectors */

/* Read an MPS file */
lprec __EXPORT_TYPE * __WINAPI read_MPS(char *filename, int verbose);
lprec __EXPORT_TYPE * __WINAPI read_mps(FILE *filename, int verbose);
lprec __EXPORT_TYPE * __WINAPI read_freeMPS(char *filename, int verbose);
lprec __EXPORT_TYPE * __WINAPI read_freemps(FILE *filename, int verbose);

/* Write a MPS file to output */
MYBOOL __EXPORT_TYPE __WINAPI write_mps(lprec *lp, char *filename);
MYBOOL __EXPORT_TYPE __WINAPI write_MPS(lprec *lp, FILE *output);
MYBOOL __EXPORT_TYPE __WINAPI write_freemps(lprec *lp, char *filename);
MYBOOL __EXPORT_TYPE __WINAPI write_freeMPS(lprec *lp, FILE *output);

MYBOOL __EXPORT_TYPE __WINAPI write_lp(lprec *lp, char *filename);
MYBOOL __EXPORT_TYPE __WINAPI write_LP(lprec *lp, FILE *output);
 /* Write a LP file to output */

MYBOOL __WINAPI LP_readhandle(lprec **lp, FILE *filename, int verbose, char *lp_name);
lprec __EXPORT_TYPE * __WINAPI read_lp(FILE *filename, int verbose, char *lp_name);
lprec __EXPORT_TYPE * __WINAPI read_LP(char *filename, int verbose, char *lp_name);
/* Old-style lp format file parser */

MYBOOL __EXPORT_TYPE __WINAPI write_basis(lprec *lp, char *filename);
MYBOOL __EXPORT_TYPE __WINAPI read_basis(lprec *lp, char *filename, char *info);
/* Read and write basis from/to file in CPLEX BAS format */

MYBOOL __EXPORT_TYPE __WINAPI write_params(lprec *lp, char *filename, char *options);
MYBOOL __EXPORT_TYPE __WINAPI read_params(lprec *lp, char *filename, char *options);
void __EXPORT_TYPE __WINAPI reset_params(lprec *lp);
/* Read and write parameter file */

void __EXPORT_TYPE __WINAPI print_lp(lprec *lp);
void __EXPORT_TYPE __WINAPI print_tableau(lprec *lp);
/* Print the current problem, only useful in very small (test) problems */

void __EXPORT_TYPE __WINAPI print_objective(lprec *lp);
void __EXPORT_TYPE __WINAPI print_solution(lprec *lp, int columns);
void __EXPORT_TYPE __WINAPI print_constraints(lprec *lp, int columns);
/* Print the solution to stdout */

void __EXPORT_TYPE __WINAPI print_duals(lprec *lp);
/* Print the dual variables of the solution */

void __EXPORT_TYPE __WINAPI print_scales(lprec *lp);
/* If scaling is used, print the scaling factors */

void __EXPORT_TYPE __WINAPI print_str(lprec *lp, char *str);

void __EXPORT_TYPE __WINAPI set_outputstream(lprec *lp, FILE *stream);
MYBOOL __EXPORT_TYPE __WINAPI set_outputfile(lprec *lp, char *filename);

void __EXPORT_TYPE __WINAPI set_verbose(lprec *lp, int verbose);
int __EXPORT_TYPE __WINAPI get_verbose(lprec *lp);

void __EXPORT_TYPE __WINAPI set_timeout(lprec *lp, long sectimeout);
long __EXPORT_TYPE __WINAPI get_timeout(lprec *lp);

void __EXPORT_TYPE __WINAPI set_print_sol(lprec *lp, int print_sol);
int __EXPORT_TYPE __WINAPI get_print_sol(lprec *lp);

void __EXPORT_TYPE __WINAPI set_debug(lprec *lp, MYBOOL debug);
MYBOOL __EXPORT_TYPE __WINAPI is_debug(lprec *lp);

void __EXPORT_TYPE __WINAPI set_trace(lprec *lp, MYBOOL trace);
MYBOOL __EXPORT_TYPE __WINAPI is_trace(lprec *lp);

MYBOOL __EXPORT_TYPE __WINAPI print_debugdump(lprec *lp, char *filename);

void __EXPORT_TYPE __WINAPI set_anti_degen(lprec *lp, int anti_degen);
int __EXPORT_TYPE __WINAPI get_anti_degen(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_anti_degen(lprec *lp, int testmask);

void __EXPORT_TYPE __WINAPI set_presolve(lprec *lp, int presolvemode, int maxloops);
int __EXPORT_TYPE __WINAPI get_presolve(lprec *lp);
int __EXPORT_TYPE __WINAPI get_presolveloops(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_presolve(lprec *lp, int testmask);

int __EXPORT_TYPE __WINAPI get_orig_index(lprec *lp, int lp_index);
int __EXPORT_TYPE __WINAPI get_lp_index(lprec *lp, int orig_index);

void __EXPORT_TYPE __WINAPI set_maxpivot(lprec *lp, int max_num_inv);
int __EXPORT_TYPE __WINAPI get_maxpivot(lprec *lp);

void __EXPORT_TYPE __WINAPI set_obj_bound(lprec *lp, REAL obj_bound);
REAL __EXPORT_TYPE __WINAPI get_obj_bound(lprec *lp);

void __EXPORT_TYPE __WINAPI set_mip_gap(lprec *lp, MYBOOL absolute, REAL mip_gap);
REAL __EXPORT_TYPE __WINAPI get_mip_gap(lprec *lp, MYBOOL absolute);

void __EXPORT_TYPE __WINAPI set_bb_rule(lprec *lp, int bb_rule);
int __EXPORT_TYPE __WINAPI get_bb_rule(lprec *lp);

MYBOOL __EXPORT_TYPE __WINAPI set_var_branch(lprec *lp, int colnr, int branch_mode);
int __EXPORT_TYPE __WINAPI get_var_branch(lprec *lp, int colnr);

MYBOOL __EXPORT_TYPE __WINAPI is_infinite(lprec *lp, REAL value);
void __EXPORT_TYPE __WINAPI set_infinite(lprec *lp, REAL infinite);
REAL __EXPORT_TYPE __WINAPI get_infinite(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epsint(lprec *lp, REAL epsint);
REAL __EXPORT_TYPE __WINAPI get_epsint(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epsb(lprec *lp, REAL epsb);
REAL __EXPORT_TYPE __WINAPI get_epsb(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epsd(lprec *lp, REAL epsd);
REAL __EXPORT_TYPE __WINAPI get_epsd(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epsel(lprec *lp, REAL epsel);
REAL __EXPORT_TYPE __WINAPI get_epsel(lprec *lp);

MYBOOL __EXPORT_TYPE __WINAPI set_epslevel(lprec *lp, int epslevel);

void __EXPORT_TYPE __WINAPI set_scaling(lprec *lp, int scalemode);
int __EXPORT_TYPE __WINAPI get_scaling(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI is_scalemode(lprec *lp, int testmask);
MYBOOL __EXPORT_TYPE __WINAPI is_scaletype(lprec *lp, int scaletype);
MYBOOL __EXPORT_TYPE __WINAPI is_integerscaling(lprec *lp);
void __EXPORT_TYPE __WINAPI set_scalelimit(lprec *lp, REAL scalelimit);
REAL __EXPORT_TYPE __WINAPI get_scalelimit(lprec *lp);

void __EXPORT_TYPE __WINAPI set_improve(lprec *lp, int improve);
int __EXPORT_TYPE __WINAPI get_improve(lprec *lp);

void __EXPORT_TYPE __WINAPI set_pivoting(lprec *lp, int piv_rule);
int __EXPORT_TYPE __WINAPI get_pivoting(lprec *lp);
MYBOOL __EXPORT_TYPE __WINAPI set_partialprice(lprec *lp, int blockcount, int *blockstart, MYBOOL isrow);
void __EXPORT_TYPE __WINAPI get_partialprice(lprec *lp, int *blockcount, int *blockstart, MYBOOL isrow);

MYBOOL __EXPORT_TYPE __WINAPI set_multiprice(lprec *lp, int multiblockdiv);
int __EXPORT_TYPE __WINAPI get_multiprice(lprec *lp, MYBOOL getabssize);

MYBOOL __WINAPI is_use_names(lprec *lp, MYBOOL isrow);
void __WINAPI set_use_names(lprec *lp, MYBOOL isrow, MYBOOL use_names);

int __EXPORT_TYPE __WINAPI get_nameindex(lprec *lp, char *varname, MYBOOL isrow);

MYBOOL __EXPORT_TYPE __WINAPI is_piv_mode(lprec *lp, int testmask);
MYBOOL __EXPORT_TYPE __WINAPI is_piv_rule(lprec *lp, int rule);

void __EXPORT_TYPE __WINAPI set_break_at_first(lprec *lp, MYBOOL break_at_first);
MYBOOL __EXPORT_TYPE __WINAPI is_break_at_first(lprec *lp);

void __EXPORT_TYPE __WINAPI set_bb_floorfirst(lprec *lp, int bb_floorfirst);
int __EXPORT_TYPE __WINAPI get_bb_floorfirst(lprec *lp);

void __EXPORT_TYPE __WINAPI set_bb_depthlimit(lprec *lp, int bb_maxlevel);
int __EXPORT_TYPE __WINAPI get_bb_depthlimit(lprec *lp);

void __EXPORT_TYPE __WINAPI set_break_at_value(lprec *lp, REAL break_at_value);
REAL __EXPORT_TYPE __WINAPI get_break_at_value(lprec *lp);

void __EXPORT_TYPE __WINAPI set_negrange(lprec *lp, REAL negrange);
REAL __EXPORT_TYPE __WINAPI get_negrange(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epsperturb(lprec *lp, REAL epsperturb);
REAL __EXPORT_TYPE __WINAPI get_epsperturb(lprec *lp);

void __EXPORT_TYPE __WINAPI set_epspivot(lprec *lp, REAL epspivot);
REAL __EXPORT_TYPE __WINAPI get_epspivot(lprec *lp);

int __EXPORT_TYPE __WINAPI get_max_level(lprec *lp);
COUNTER __EXPORT_TYPE __WINAPI get_total_nodes(lprec *lp);
COUNTER __EXPORT_TYPE __WINAPI get_total_iter(lprec *lp);

REAL __EXPORT_TYPE __WINAPI get_objective(lprec *lp);
REAL __EXPORT_TYPE __WINAPI get_working_objective(lprec *lp);

REAL __EXPORT_TYPE __WINAPI get_var_primalresult(lprec *lp, int index);
REAL __EXPORT_TYPE __WINAPI get_var_dualresult(lprec *lp, int index);

MYBOOL __EXPORT_TYPE __WINAPI get_variables(lprec *lp, REAL *var);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_variables(lprec *lp, REAL **var);

MYBOOL __EXPORT_TYPE __WINAPI get_constraints(lprec *lp, REAL *constr);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_constraints(lprec *lp, REAL **constr);

MYBOOL __EXPORT_TYPE __WINAPI get_sensitivity_rhs(lprec *lp, REAL *duals, REAL *dualsfrom, REAL *dualstill);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_sensitivity_rhs(lprec *lp, REAL **duals, REAL **dualsfrom, REAL **dualstill);

MYBOOL __EXPORT_TYPE __WINAPI get_sensitivity_obj(lprec *lp, REAL *objfrom, REAL *objtill);
MYBOOL __EXPORT_TYPE __WINAPI get_sensitivity_objex(lprec *lp, REAL *objfrom, REAL *objtill, REAL *objfromvalue, REAL *objtillvalue);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_sensitivity_obj(lprec *lp, REAL **objfrom, REAL **objtill);
MYBOOL __EXPORT_TYPE __WINAPI get_ptr_sensitivity_objex(lprec *lp, REAL **objfrom, REAL **objtill, REAL **objfromvalue, REAL **objtillvalue);

void __EXPORT_TYPE __WINAPI set_solutionlimit(lprec *lp, int limit);
int __EXPORT_TYPE __WINAPI get_solutionlimit(lprec *lp);
int __EXPORT_TYPE __WINAPI get_solutioncount(lprec *lp);

int __EXPORT_TYPE __WINAPI get_Norig_rows(lprec *lp);
int __EXPORT_TYPE __WINAPI get_Nrows(lprec *lp);
int __EXPORT_TYPE __WINAPI get_Lrows(lprec *lp);

int __EXPORT_TYPE __WINAPI get_Norig_columns(lprec *lp);
int __EXPORT_TYPE __WINAPI get_Ncolumns(lprec *lp);


#ifdef __cplusplus
}
#endif


/* Forward definitions of functions used internaly by the lp toolkit */
MYBOOL set_callbacks(lprec *lp);
STATIC int yieldformessages(lprec *lp);
MYBOOL __WINAPI userabort(lprec *lp, int message);
/*char * __VACALL explain(lprec *lp, char *format, ...);
void __VACALL report(lprec *lp, int level, char *format, ...);*/

/* Memory management routines */
STATIC MYBOOL append_rows(lprec *lp, int deltarows);
STATIC MYBOOL append_columns(lprec *lp, int deltacolumns);
STATIC void inc_rows(lprec *lp, int delta);
STATIC void inc_columns(lprec *lp, int delta);
STATIC MYBOOL init_rowcol_names(lprec *lp);
STATIC MYBOOL inc_row_space(lprec *lp, int deltarows);
STATIC MYBOOL inc_col_space(lprec *lp, int deltacols);
STATIC MYBOOL shift_rowcoldata(lprec *lp, int base, int delta, LLrec *usedmap, MYBOOL isrow);
STATIC MYBOOL shift_basis(lprec *lp, int base, int delta, LLrec *usedmap, MYBOOL isrow);
STATIC MYBOOL shift_rowdata(lprec *lp, int base, int delta, LLrec *usedmap);
STATIC MYBOOL shift_coldata(lprec *lp, int base, int delta, LLrec *usedmap);

/* INLINE */ MYBOOL is_chsign(lprec *lp, int rownr);

STATIC MYBOOL inc_lag_space(lprec *lp, int deltarows, MYBOOL ignoreMAT);
lprec *make_lag(lprec *server);

REAL get_rh_upper(lprec *lp, int rownr);
REAL get_rh_lower(lprec *lp, int rownr);
MYBOOL set_rh_upper(lprec *lp, int rownr, REAL value);
MYBOOL set_rh_lower(lprec *lp, int rownr, REAL value);
STATIC int bin_count(lprec *lp, MYBOOL working);
STATIC int MIP_count(lprec *lp);
STATIC int SOS_count(lprec *lp);
STATIC int GUB_count(lprec *lp);
STATIC int identify_GUB(lprec *lp, MYBOOL mark);
STATIC int prepare_GUB(lprec *lp);

STATIC MYBOOL refactRecent(lprec *lp);
STATIC MYBOOL check_if_less(lprec *lp, REAL x, REAL y, int variable);
STATIC MYBOOL feasiblePhase1(lprec *lp, REAL epsvalue);
STATIC void free_duals(lprec *lp);
STATIC void initialize_solution(lprec *lp, MYBOOL shiftbounds);
STATIC void recompute_solution(lprec *lp, MYBOOL shiftbounds);
STATIC int verify_solution(lprec *lp, MYBOOL reinvert, char *info);
STATIC int check_solution(lprec *lp, int  lastcolumn, REAL *solution,
                          REAL *upbo, REAL *lowbo, REAL tolerance);
/* INLINE */ MYBOOL is_fixedvar(lprec *lp, int variable);
/* INLINE */ MYBOOL is_splitvar(lprec *lp, int colnr);

void   __WINAPI set_action(int *actionvar, int actionmask);
void   __WINAPI clear_action(int *actionvar, int actionmask);
MYBOOL __WINAPI is_action(int actionvar, int testmask);

INLINE MYBOOL is_bb_rule(lprec *lp, int bb_rule);
/* INLINE */ MYBOOL is_bb_mode(lprec *lp, int bb_mask);
/* INLINE */ int get_piv_rule(lprec *lp);
STATIC char *get_str_piv_rule(int rule);
STATIC MYBOOL __WINAPI set_var_priority(lprec *lp);
STATIC int find_sc_bbvar(lprec *lp, int *count);
STATIC int find_sos_bbvar(lprec *lp, int *count, MYBOOL intsos);
STATIC int find_int_bbvar(lprec *lp, int *count, BBrec *BB, MYBOOL *isfeasible);

/* Solution-related functions */
STATIC REAL compute_dualslacks(lprec *lp, int target, REAL **dvalues, int **nzdvalues, MYBOOL dosum);
STATIC MYBOOL solution_is_int(lprec *lp, int index, MYBOOL checkfixed);
STATIC MYBOOL bb_better(lprec *lp, int target, int mode);
STATIC void construct_solution(lprec *lp, REAL *target);
STATIC void transfer_solution_var(lprec *lp, int uservar);
STATIC MYBOOL construct_duals(lprec *lp);
STATIC MYBOOL construct_sensitivity_duals(lprec *lp);
STATIC MYBOOL construct_sensitivity_obj(lprec *lp);

STATIC int add_GUB(lprec *lp, char *name, int priority, int count, int *sosvars);
STATIC basisrec *push_basis(lprec *lp, int *basisvar, MYBOOL *isbasic, MYBOOL *islower);
STATIC MYBOOL compare_basis(lprec *lp);
STATIC MYBOOL restore_basis(lprec *lp);
STATIC MYBOOL pop_basis(lprec *lp, MYBOOL restore);
STATIC MYBOOL is_BasisReady(lprec *lp);
STATIC MYBOOL is_slackbasis(lprec *lp);
STATIC MYBOOL verify_basis(lprec *lp);
STATIC int unload_basis(lprec *lp, MYBOOL restorelast);

STATIC int perturb_bounds(lprec *lp, BBrec *perturbed, MYBOOL doRows, MYBOOL doCols, MYBOOL includeFIXED);
STATIC MYBOOL validate_bounds(lprec *lp, REAL *upbo, REAL *lowbo);
STATIC MYBOOL impose_bounds(lprec *lp, REAL * upbo, REAL *lowbo);
STATIC int unload_BB(lprec *lp);

STATIC REAL feasibilityOffset(lprec *lp, MYBOOL isdual);
STATIC MYBOOL isP1extra(lprec *lp);
STATIC REAL get_refactfrequency(lprec *lp, MYBOOL final);
STATIC int findBasicFixedvar(lprec *lp, int afternr, MYBOOL slacksonly);
STATIC MYBOOL isBasisVarFeasible(lprec *lp, REAL tol, int basis_row);
STATIC MYBOOL isPrimalFeasible(lprec *lp, REAL tol, int infeasibles[], REAL *feasibilitygap);
STATIC MYBOOL isDualFeasible(lprec *lp, REAL tol, int *boundflips, int infeasibles[], REAL *feasibilitygap);

/* Main simplex driver routines */
STATIC int preprocess(lprec *lp);
STATIC void postprocess(lprec *lp);
STATIC MYBOOL performiteration(lprec *lp, int rownr, int varin, LREAL theta, MYBOOL primal, MYBOOL allowminit, REAL *prow, int *nzprow, REAL *pcol, int *nzpcol, int *boundswaps);
STATIC void transfer_solution_var(lprec *lp, int uservar);
STATIC void transfer_solution(lprec *lp, MYBOOL dofinal);

/* Scaling utilities */
STATIC REAL scaled_floor(lprec *lp, int colnr, REAL value, REAL epsscale);
STATIC REAL scaled_ceil(lprec *lp, int colnr, REAL value, REAL epsscale);

/* Variable mapping utility routines */
STATIC void varmap_lock(lprec *lp);
STATIC void varmap_clear(lprec *lp);
STATIC MYBOOL varmap_canunlock(lprec *lp);
STATIC void varmap_addconstraint(lprec *lp);
STATIC void varmap_addcolumn(lprec *lp);
STATIC void varmap_delete(lprec *lp, int base, int delta, LLrec *varmap);
STATIC void varmap_compact(lprec *lp, int prev_rows, int prev_cols);
STATIC MYBOOL varmap_validate(lprec *lp, int varno);
STATIC MYBOOL del_varnameex(lprec *lp, hashelem **namelist, hashtable *ht, int varnr, LLrec *varmap);

/* Pseudo-cost routines (internal) */
STATIC BBPSrec *init_pseudocost(lprec *lp, int pseudotype);
STATIC void free_pseudocost(lprec *lp);
STATIC REAL get_pseudorange(BBPSrec *pc, int mipvar, int varcode);
STATIC void update_pseudocost(BBPSrec *pc, int mipvar, int varcode, MYBOOL capupper, REAL varsol);
STATIC REAL get_pseudobranchcost(BBPSrec *pc, int mipvar, MYBOOL dofloor);
STATIC REAL get_pseudonodecost(BBPSrec *pc, int mipvar, int vartype, REAL varsol);

/* Matrix access and equation solving routines */
STATIC void set_OF_override(lprec *lp, REAL *ofVector);
STATIC void set_OF_p1extra(lprec *lp, REAL p1extra);
STATIC void unset_OF_p1extra(lprec *lp);
MYBOOL modifyOF1(lprec *lp, int index, REAL *ofValue, REAL mult);
REAL __WINAPI get_OF_active(lprec *lp, int varnr, REAL mult);
STATIC MYBOOL is_OF_nz(lprec *lp, int colnr);

STATIC int get_basisOF(lprec *lp, int coltarget[], REAL crow[], int colno[]);
int    __WINAPI get_basiscolumn(lprec *lp, int j, int rn[], double bj[]);
int    __WINAPI obtain_column(lprec *lp, int varin, REAL *pcol, int *nzlist, int *maxabs);
STATIC int compute_theta(lprec *lp, int rownr, LREAL *theta, int isupbound, REAL HarrisScalar, MYBOOL primal);

/* Pivot utility routines */
STATIC int findBasisPos(lprec *lp, int notint, int *var_basic);
STATIC MYBOOL check_degeneracy(lprec *lp, REAL *pcol, int *degencount);

typedef int (__WINAPI read_modeldata_func)(void *userhandle, char *buf, int max_size);
typedef int (__WINAPI write_modeldata_func)(void *userhandle, char *buf);
MYBOOL __WINAPI MPS_readex(lprec **newlp, void *userhandle, read_modeldata_func read_modeldata, int typeMPS, int verbose);
#if defined develop
lprec __EXPORT_TYPE * __WINAPI read_lpex(void *userhandle, read_modeldata_func read_modeldata, int verbose, char *lp_name);
MYBOOL __EXPORT_TYPE __WINAPI write_lpex(lprec *lp, void *userhandle, write_modeldata_func write_modeldata);

lprec __EXPORT_TYPE * __WINAPI read_mpsex(void *userhandle, read_modeldata_func read_modeldata, int verbose);
lprec __EXPORT_TYPE * __WINAPI read_freempsex(void *userhandle, read_modeldata_func read_modeldata, int verbose);

MYBOOL MPS_writefileex(lprec *lp, int typeMPS, void *userhandle, write_modeldata_func write_modeldata);
#endif

#endif /* HEADER_lp_lib */
