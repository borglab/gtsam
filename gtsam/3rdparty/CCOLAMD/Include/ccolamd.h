/* ========================================================================== */
/* === CCOLAMD/ccolamd.h ==================================================== */
/* ========================================================================== */

/* ----------------------------------------------------------------------------
 * CCOLAMD Copyright (C), Univ. of Florida.  Authors: Timothy A. Davis,
 * Sivasankaran Rajamanickam, and Stefan Larimore
 * -------------------------------------------------------------------------- */

/*
 *  You must include this file (ccolamd.h) in any routine that uses ccolamd,
 *  csymamd, or the related macros and definitions.
 */

#ifndef CCOLAMD_H
#define CCOLAMD_H

/* make it easy for C++ programs to include CCOLAMD */
#ifdef __cplusplus
extern "C" {
#endif

/* for size_t definition: */
#include <stdlib.h>

/* ========================================================================== */
/* === CCOLAMD version ====================================================== */
/* ========================================================================== */

/* All versions of CCOLAMD will include the following definitions.
 * As an example, to test if the version you are using is 1.3 or later:
 *
 *	if (CCOLAMD_VERSION >= CCOLAMD_VERSION_CODE (1,3)) ...
 *
 * This also works during compile-time:
 *
 *	#if CCOLAMD_VERSION >= CCOLAMD_VERSION_CODE (1,3)
 *	    printf ("This is version 1.3 or later\n") ;
 *	#else
 *	    printf ("This is an early version\n") ;
 *	#endif
 */

#define CCOLAMD_DATE "May 4, 2016"
#define CCOLAMD_VERSION_CODE(main,sub) ((main) * 1000 + (sub))
#define CCOLAMD_MAIN_VERSION 2
#define CCOLAMD_SUB_VERSION 9
#define CCOLAMD_SUBSUB_VERSION 6
#define CCOLAMD_VERSION \
	CCOLAMD_VERSION_CODE(CCOLAMD_MAIN_VERSION,CCOLAMD_SUB_VERSION)

/* ========================================================================== */
/* === Knob and statistics definitions ====================================== */
/* ========================================================================== */

/* size of the knobs [ ] array.  Only knobs [0..3] are currently used. */
#define CCOLAMD_KNOBS 20

/* number of output statistics.  Only stats [0..10] are currently used. */
#define CCOLAMD_STATS 20

/* knobs [0] and stats [0]: dense row knob and output statistic. */
#define CCOLAMD_DENSE_ROW 0

/* knobs [1] and stats [1]: dense column knob and output statistic. */
#define CCOLAMD_DENSE_COL 1

/* knobs [2]: aggressive absorption option */
#define CCOLAMD_AGGRESSIVE 2

/* knobs [3]: LU or Cholesky factorization option */
#define CCOLAMD_LU 3

/* stats [2]: memory defragmentation count output statistic */
#define CCOLAMD_DEFRAG_COUNT 2

/* stats [3]: ccolamd status:  zero OK, > 0 warning or notice, < 0 error */
#define CCOLAMD_STATUS 3

/* stats [4..6]: error info, or info on jumbled columns */ 
#define CCOLAMD_INFO1 4
#define CCOLAMD_INFO2 5
#define CCOLAMD_INFO3 6

/* stats [7]: number of originally empty rows */
#define CCOLAMD_EMPTY_ROW 7
/* stats [8]: number of originally empty cols */
#define CCOLAMD_EMPTY_COL 8
/* stats [9]: number of rows with entries only in dense cols */
#define CCOLAMD_NEWLY_EMPTY_ROW 9
/* stats [10]: number of cols with entries only in dense rows */
#define CCOLAMD_NEWLY_EMPTY_COL 10

/* error codes returned in stats [3]: */
#define CCOLAMD_OK				(0)
#define CCOLAMD_OK_BUT_JUMBLED			(1)
#define CCOLAMD_ERROR_A_not_present		(-1)
#define CCOLAMD_ERROR_p_not_present		(-2)
#define CCOLAMD_ERROR_nrow_negative		(-3)
#define CCOLAMD_ERROR_ncol_negative		(-4)
#define CCOLAMD_ERROR_nnz_negative		(-5)
#define CCOLAMD_ERROR_p0_nonzero		(-6)
#define CCOLAMD_ERROR_A_too_small		(-7)
#define CCOLAMD_ERROR_col_length_negative	(-8)
#define CCOLAMD_ERROR_row_index_out_of_bounds	(-9)
#define CCOLAMD_ERROR_out_of_memory		(-10)
#define CCOLAMD_ERROR_invalid_cmember		(-11)
#define CCOLAMD_ERROR_internal_error		(-999)

/* ========================================================================== */
/* === Prototypes of user-callable routines ================================= */
/* ========================================================================== */

#include "SuiteSparse_config.h"

size_t ccolamd_recommended	/* returns recommended value of Alen, */
				/* or 0 if input arguments are erroneous */
(
    int nnz,			/* nonzeros in A */
    int n_row,			/* number of rows in A */
    int n_col			/* number of columns in A */
) ;

size_t ccolamd_l_recommended	/* returns recommended value of Alen, */
				/* or 0 if input arguments are erroneous */
(
    SuiteSparse_long nnz,		/* nonzeros in A */
    SuiteSparse_long n_row,		/* number of rows in A */
    SuiteSparse_long n_col		/* number of columns in A */
) ;

void ccolamd_set_defaults	/* sets default parameters */
(				/* knobs argument is modified on output */
    double knobs [CCOLAMD_KNOBS]	/* parameter settings for ccolamd */
) ;

void ccolamd_l_set_defaults	/* sets default parameters */
(				/* knobs argument is modified on output */
    double knobs [CCOLAMD_KNOBS]	/* parameter settings for ccolamd */
) ;

int ccolamd			/* returns (1) if successful, (0) otherwise*/
(				/* A and p arguments are modified on output */
    int n_row,			/* number of rows in A */
    int n_col,			/* number of columns in A */
    int Alen,			/* size of the array A */
    int A [ ],			/* row indices of A, of size Alen */
    int p [ ],			/* column pointers of A, of size n_col+1 */
    double knobs [CCOLAMD_KNOBS],/* parameter settings for ccolamd */
    int stats [CCOLAMD_STATS],	/* ccolamd output statistics and error codes */
    int cmember [ ]		/* Constraint set of A, of size n_col */
) ;

SuiteSparse_long ccolamd_l      /* as ccolamd w/ SuiteSparse_long integers */
(
    SuiteSparse_long n_row,
    SuiteSparse_long n_col,
    SuiteSparse_long Alen,
    SuiteSparse_long A [ ],
    SuiteSparse_long p [ ],
    double knobs [CCOLAMD_KNOBS],
    SuiteSparse_long stats [CCOLAMD_STATS],
    SuiteSparse_long cmember [ ]
) ;

int csymamd			/* return (1) if OK, (0) otherwise */
(
    int n,			/* number of rows and columns of A */
    int A [ ],			/* row indices of A */
    int p [ ],			/* column pointers of A */
    int perm [ ],		/* output permutation, size n_col+1 */
    double knobs [CCOLAMD_KNOBS],/* parameters (uses defaults if NULL) */
    int stats [CCOLAMD_STATS],	/* output statistics and error codes */
    void * (*allocate) (size_t, size_t), /* pointer to calloc (ANSI C) or */
				/* mxCalloc (for MATLAB mexFunction) */
    void (*release) (void *),	/* pointer to free (ANSI C) or */
    				/* mxFree (for MATLAB mexFunction) */
    int cmember [ ],		/* Constraint set of A */
    int stype			/* 0: use both parts, >0: upper, <0: lower */
) ;

SuiteSparse_long csymamd_l      /* as csymamd, w/ SuiteSparse_long integers */
(
    SuiteSparse_long n,
    SuiteSparse_long A [ ],
    SuiteSparse_long p [ ],
    SuiteSparse_long perm [ ],
    double knobs [CCOLAMD_KNOBS],
    SuiteSparse_long stats [CCOLAMD_STATS],
    void * (*allocate) (size_t, size_t),
    void (*release) (void *),
    SuiteSparse_long cmember [ ],
    SuiteSparse_long stype
) ;

void ccolamd_report
(
    int stats [CCOLAMD_STATS]
) ;

void ccolamd_l_report
(
    SuiteSparse_long stats [CCOLAMD_STATS]
) ;

void csymamd_report
(
    int stats [CCOLAMD_STATS]
) ;

void csymamd_l_report
(
    SuiteSparse_long stats [CCOLAMD_STATS]
) ;


/* ========================================================================== */
/* === Prototypes of "expert" routines ====================================== */
/* ========================================================================== */

/* These routines are meant to be used internally, or in a future version of
 * UMFPACK.  They appear here so that UMFPACK can use them, but they should not
 * be called directly by the user.
 */

int ccolamd2
(				/* A and p arguments are modified on output */
    int n_row,			/* number of rows in A */
    int n_col,			/* number of columns in A */
    int Alen,			/* size of the array A */
    int A [ ],			/* row indices of A, of size Alen */
    int p [ ],			/* column pointers of A, of size n_col+1 */
    double knobs [CCOLAMD_KNOBS],/* parameter settings for ccolamd */
    int stats [CCOLAMD_STATS],	/* ccolamd output statistics and error codes */
    /* each Front_ array is of size n_col+1: */
    int Front_npivcol [ ],	/* # pivot cols in each front */
    int Front_nrows [ ],	/* # of rows in each front (incl. pivot rows) */
    int Front_ncols [ ],	/* # of cols in each front (incl. pivot cols) */
    int Front_parent [ ],	/* parent of each front */
    int Front_cols [ ],		/* link list of pivot columns for each front */
    int *p_nfr,			/* total number of frontal matrices */
    int InFront [ ],		/* InFront [row] = f if row in front f */
    int cmember [ ]		/* Constraint set of A */
) ;

SuiteSparse_long ccolamd2_l     /* as ccolamd2, w/ SuiteSparse_long integers */
(
    SuiteSparse_long n_row,
    SuiteSparse_long n_col,
    SuiteSparse_long Alen,
    SuiteSparse_long A [ ],
    SuiteSparse_long p [ ],
    double knobs [CCOLAMD_KNOBS],
    SuiteSparse_long stats [CCOLAMD_STATS],
    SuiteSparse_long Front_npivcol [ ],
    SuiteSparse_long Front_nrows [ ],
    SuiteSparse_long Front_ncols [ ],
    SuiteSparse_long Front_parent [ ],
    SuiteSparse_long Front_cols [ ],
    SuiteSparse_long *p_nfr,
    SuiteSparse_long InFront [ ],
    SuiteSparse_long cmember [ ]
) ;

void ccolamd_apply_order
(
    int Front [ ],
    const int Order [ ],
    int Temp [ ],
    int nn,
    int nfr
) ;

void ccolamd_l_apply_order
(
    SuiteSparse_long Front [ ],
    const SuiteSparse_long Order [ ],
    SuiteSparse_long Temp [ ],
    SuiteSparse_long nn,
    SuiteSparse_long nfr
) ;


void ccolamd_fsize
(
    int nn,
    int MaxFsize [ ],
    int Fnrows [ ],
    int Fncols [ ],
    int Parent [ ],
    int Npiv [ ]
) ;

void ccolamd_l_fsize
(
    SuiteSparse_long nn,
    SuiteSparse_long MaxFsize [ ],
    SuiteSparse_long Fnrows [ ],
    SuiteSparse_long Fncols [ ],
    SuiteSparse_long Parent [ ],
    SuiteSparse_long Npiv [ ]
) ;

void ccolamd_postorder
(
    int nn,
    int Parent [ ],
    int Npiv [ ],
    int Fsize [ ],
    int Order [ ],
    int Child [ ],
    int Sibling [ ],
    int Stack [ ],
    int Front_cols [ ],
    int cmember [ ]
) ;

void ccolamd_l_postorder
(
    SuiteSparse_long nn,
    SuiteSparse_long Parent [ ],
    SuiteSparse_long Npiv [ ],
    SuiteSparse_long Fsize [ ],
    SuiteSparse_long Order [ ],
    SuiteSparse_long Child [ ],
    SuiteSparse_long Sibling [ ],
    SuiteSparse_long Stack [ ],
    SuiteSparse_long Front_cols [ ],
    SuiteSparse_long cmember [ ]
) ;

int ccolamd_post_tree
(
    int root,
    int k,
    int Child [ ],
    const int Sibling [ ],
    int Order [ ],
    int Stack [ ]
) ;

SuiteSparse_long ccolamd_l_post_tree
(
    SuiteSparse_long root,
    SuiteSparse_long k,
    SuiteSparse_long Child [ ],
    const SuiteSparse_long Sibling [ ],
    SuiteSparse_long Order [ ],
    SuiteSparse_long Stack [ ]
) ;

#ifdef __cplusplus
}
#endif

#endif
