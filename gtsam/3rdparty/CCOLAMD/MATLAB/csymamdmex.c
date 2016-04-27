/* ========================================================================== */
/* === csymamd mexFunction ================================================== */
/* ========================================================================== */

/* ----------------------------------------------------------------------------
 * CCOLAMD, Copyright (C), Univ. of Florida.  Authors: Timothy A. Davis,
 * Sivasankaran Rajamanickam, and Stefan Larimore
 * -------------------------------------------------------------------------- */

/* 
 * Usage:
 *	p = csymamd (A) ;
 *	[p stats] = csymamd (A, knobs, cmember) ;
 *
 * See csymamd.m for a description.
 */

/* ========================================================================== */
/* === Include files ======================================================== */
/* ========================================================================== */

#include "ccolamd.h"
#include "mex.h"
#include "matrix.h"
#include <stdlib.h>
#define Long SuiteSparse_long

/* ========================================================================== */
/* === csymamd mexFunction ================================================== */
/* ========================================================================== */

void mexFunction
(
    /* === Parameters ======================================================= */

    int nargout,
    mxArray *pargout [ ],
    int nargin,
    const mxArray *pargin [ ]
)
{
    /* === Local variables ================================================== */

    Long *A ;                   /* row indices of input matrix A */
    Long *perm ;                /* column ordering of M and ordering of A */
    Long *cmember ;             /* csymamd's copy of the constraint set */
    double *in_cmember ;        /* input constraint set */
    Long *p ;                   /* column pointers of input matrix A */
    Long cslen ;                /* size of constraint set */
    Long n_col ;                /* number of columns of A */
    Long n_row ;                /* number of rows of A */
    Long full ;                 /* TRUE if input matrix full, FALSE if sparse */
    double knobs [CCOLAMD_KNOBS] ; /* csymamd user-controllable parameters */
    double *out_perm ;          /* output permutation vector */
    double *out_stats ;         /* output stats vector */
    double *in_knobs ;          /* input knobs vector */
    Long i ;                    /* loop counter */
    mxArray *Ainput ;           /* input matrix handle */
    Long spumoni ;              /* verbosity variable */
    Long stats [CCOLAMD_STATS] ;/* stats for symamd */

    /* === Check inputs ===================================================== */

    if (nargin < 1 || nargin > 3 || nargout < 0 || nargout > 2)
    {
	mexErrMsgTxt ("Usage: [p stats] = csymamd (S, knobs, cmember)") ;
    }

    /* === Get cmember ====================================================== */

    cmember = NULL ;
    cslen = 0 ;
    if (nargin > 2)
    {
	in_cmember = mxGetPr (pargin [2]) ;
	cslen = mxGetNumberOfElements (pargin [2]) ;
	if (cslen != 0)
	{
	    cmember = (Long *) mxCalloc (cslen, sizeof (Long)) ;
	    for (i = 0 ; i < cslen ; i++)
	    {
		/* convert cmember from 1-based to 0-based */
		cmember[i] = ((Long) in_cmember [i] - 1) ;
	    }
	}
    }

    /* === Get knobs ======================================================== */

    ccolamd_l_set_defaults (knobs) ;
    spumoni = 0 ;

    /* check for user-passed knobs */
    i = 0 ;
    if (nargin > 1)
    {
	in_knobs = mxGetPr (pargin [1]) ;
	i = mxGetNumberOfElements (pargin [1]) ;
	if (i > 0) knobs [CCOLAMD_DENSE_ROW] = in_knobs [0] ;
	if (i > 1) knobs [CCOLAMD_AGGRESSIVE] = in_knobs [1] ;
	if (i > 2) spumoni = (in_knobs [2] != 0) ;
    }

    /* print knob settings if spumoni is set */
    if (spumoni)
    {
	mexPrintf ("\ncsymamd version %d.%d, %s:\n",
	    CCOLAMD_MAIN_VERSION, CCOLAMD_SUB_VERSION, CCOLAMD_DATE) ;
	if (knobs [CCOLAMD_DENSE_ROW] >= 0)
	{
	    mexPrintf ("knobs(1): %g, rows/cols with > "
		"max(16,%g*sqrt(size(A,2))) entries removed\n",
		in_knobs [0], knobs [CCOLAMD_DENSE_ROW]) ;
	}
	else
	{
	    mexPrintf ("knobs(1): %g, no dense rows removed\n",
		in_knobs [0]) ;
	}
	mexPrintf ("knobs(2): %g, aggressive absorption: %s\n",
	    in_knobs [1], (knobs [CCOLAMD_AGGRESSIVE] != 0) ? "yes" : "no") ;
	mexPrintf ("knobs(3): %g, statistics and knobs printed\n",
		in_knobs [2]) ;
    }

    /* === If A is full, convert to a sparse matrix ========================= */

    Ainput = (mxArray *) pargin [0] ;
    if (mxGetNumberOfDimensions (Ainput) != 2)
    {
	mexErrMsgTxt ("csymamd: input matrix must be 2-dimensional.") ;
    }
    full = !mxIsSparse (Ainput) ;
    if (full)
    {
	mexCallMATLAB (1, &Ainput, 1, (mxArray **) pargin, "sparse") ;
    }

    /* === Allocate workspace for csymamd =================================== */

    /* get size of matrix */
    n_row = mxGetM (Ainput) ;
    n_col = mxGetN (Ainput) ;
    if (n_col != n_row)
    {
	mexErrMsgTxt ("csymamd: matrix must be square.") ;
    }

    if (cmember != NULL && cslen != n_col)
    {
    	mexErrMsgTxt ("csymamd: cmember must be of length equal to #cols of A");
    }

    A = (Long *) mxGetIr (Ainput) ;
    p = (Long *) mxGetJc (Ainput) ;
    perm = (Long *) mxCalloc (n_col+1, sizeof (Long)) ;

    /* === Order the rows and columns of A (does not destroy A) ============= */

    if (!csymamd_l (n_col, A, p, perm, knobs, stats, &mxCalloc, &mxFree,
	    cmember, -1))
    {
	csymamd_l_report (stats) ;
	mexErrMsgTxt ("csymamd error!") ;
    }

    if (full)
    {
	mxDestroyArray (Ainput) ;
    }

    /* === Return the permutation vector ==================================== */

    pargout [0] = mxCreateDoubleMatrix (1, n_col, mxREAL) ;
    out_perm = mxGetPr (pargout [0]) ;
    for (i = 0 ; i < n_col ; i++)
    {
	/* symamd is 0-based, but MATLAB expects this to be 1-based */
	out_perm [i] = perm [i] + 1 ;
    }
    mxFree (perm) ;
    mxFree (cmember) ;

    /* === Return the stats vector ========================================== */

    /* print stats if spumoni is set */
    if (spumoni)
    {
	csymamd_l_report (stats) ;
    }

    if (nargout == 2)
    {
	pargout [1] = mxCreateDoubleMatrix (1, CCOLAMD_STATS, mxREAL) ;
	out_stats = mxGetPr (pargout [1]) ;
	for (i = 0 ; i < CCOLAMD_STATS ; i++)
	{
	    out_stats [i] = stats [i] ;
	}

	/* fix stats (5) and (6), for 1-based information on jumbled matrix. */
	/* note that this correction doesn't occur if symamd returns FALSE */
	out_stats [CCOLAMD_INFO1] ++ ; 
	out_stats [CCOLAMD_INFO2] ++ ; 
    }
}
