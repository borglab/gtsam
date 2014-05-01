#ifndef HEADER_lp_LUSOL
#define HEADER_lp_LUSOL

/* Include libraries for this inverse system */
#include "lp_types.h"
#include "lusol.h"

/* LUSOL defines */
#ifdef WIN32
# define LUSOL_UseBLAS
#endif
/*#define MAPSINGULARCOLUMN*/
#define MATINDEXBASE LUSOL_ARRAYOFFSET /* Inversion engine index start for arrays */
#define LU_START_SIZE           10000  /* Start size of LU; realloc'ed if needed */
#define DEF_MAXPIVOT              250  /* Maximum number of pivots before refactorization */
#define MAX_DELTAFILLIN           2.0  /* Do refactorizations based on sparsity considerations */
#define TIGHTENAFTER               10  /* Tighten LU pivot criteria only after this number of singularities */

/* typedef */ struct _INVrec
{
  int       status;                 /* Last operation status code */
  int       dimcount;               /* The actual number of LU rows/columns */
  int       dimalloc;               /* The allocated LU rows/columns size */
  int       user_colcount;          /* The number of user LU columns */
  LUSOLrec  *LUSOL;
  int       col_enter;              /* The full index of the entering column */
  int       col_leave;              /* The full index of the leaving column */
  int       col_pos;                /* The B column to be changed at the next update using data in value[.]*/
  REAL      *value;
  REAL      *pcol;                  /* Reference to the elimination vector */
  REAL      theta_enter;            /* Value of the entering column theta */

  int       max_Bsize;              /* The largest B matrix of user variables */
  int       max_colcount;           /* The maximum number of user columns in LU */
  int       max_LUsize;             /* The largest NZ-count of LU-files generated */
  int       num_refact;             /* Number of times the basis was refactored */
  int       num_timed_refact;
  int       num_dense_refact;
  double    time_refactstart;       /* Time since start of last refactorization-pivots cyle */
  double    time_refactnext;        /* Time estimated to next refactorization */
  int       num_pivots;             /* Number of pivots since last refactorization */
  int       num_singular;           /* The total number of singular updates */
  char      *opts;
  MYBOOL    is_dirty;               /* Specifies if a column is incompletely processed */
  MYBOOL    force_refact;           /* Force refactorization at the next opportunity */
  MYBOOL    timed_refact;           /* Set if timer-driven refactorization should be active */
  MYBOOL    set_Bidentity;          /* Force B to be the identity matrix at the next refactorization */
} /* INVrec */;


#ifdef __cplusplus
/* namespace LUSOL */
extern "C" {
#endif

/* Put function headers here */
#include "lp_BFP.h"

#ifdef __cplusplus
 }
#endif

#endif /* HEADER_lp_LUSOL */
