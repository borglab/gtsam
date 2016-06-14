/* ========================================================================== */
/* === csymamdtest mexFunction ============================================== */
/* ========================================================================== */

/* ----------------------------------------------------------------------------
 * CCOLAMD Copyright (C), Univ. of Florida.  Authors: Timothy A. Davis,
 * Sivasankaran Rajamanickam, and Stefan Larimore
 * -------------------------------------------------------------------------- */

/*
 *  This MATLAB mexFunction is for testing only.  It is not meant for
 *  production use.  See csymamdmex.c and csymamd.m instead.
 *
 *  Usage:
 *
 *	[ P, stats ] = csymamdtest (A, knobs, cmember) ;
 *
 *  The knobs and stats vectors are optional:
 *
 *	knobs (1)	dense row/col control. default 10
 *	knobs (2)	spumoni, default 0.
 *	knobs (3)	aggresive absorption if nonzero.  default 1
 *
 *	knobs (4)	for testing only.  Controls how the input matrix is
 *			jumbled prior to calling colamd, to test its error
 *			handling capability.
 */

/* ========================================================================== */
/* === Include files ======================================================== */
/* ========================================================================== */

#include "ccolamd.h"
#include "mex.h"
#include "matrix.h"
#include <stdlib.h>
#include <string.h>
#define Long SuiteSparse_long

#ifdef MIN
#undef MIN
#endif
#define MIN(a,b) (((a) < (b)) ? (a) : (b))


static void dump_matrix
(
    Long A [ ],
    Long p [ ],
    Long n_row,
    Long n_col,
    Long Alen,
    Long limit
)
{
    Long col, k, row ;

    mexPrintf ("dump matrix: nrow %d ncol %d Alen %d\n", n_row, n_col, Alen) ;

    if (!A)
    {
    	mexPrintf ("A not present\n") ;
	return ;
    }

    if (!p)
    {
    	mexPrintf ("p not present\n") ;
	return ;
    }

    for (col = 0 ; col < MIN (n_col, limit) ; col++)
    {
	mexPrintf ("column %d, p[col] %d, p [col+1] %d, length %d\n",
		col, p [col], p [col+1], p [col+1] - p [col]) ;
    	for (k = p [col] ; k < p [col+1] ; k++)
	{
	    row = A [k] ;
	    mexPrintf (" %d", row) ;
	}
	mexPrintf ("\n") ;
    }
}

/* ========================================================================== */
/* === csymamd mexFunction ================================================== */
/* ========================================================================== */

void mexFunction
(
    /* === Parameters ======================================================= */

    int nargout,		/* number of left-hand sides */
    mxArray *pargout [ ],	/* left-hand side matrices */
    int nargin,			/* number of right--hand sides */
    const mxArray *pargin [ ]	/* right-hand side matrices */
)
{
    /* === Local variables ================================================== */

    Long *perm ;                /* column ordering of M and ordering of A */
    Long *A ;                   /* row indices of input matrix A */
    Long *p ;                   /* column pointers of input matrix A */
    Long n_col ;                /* number of columns of A */
    Long n_row ;                /* number of rows of A */
    Long full ;                 /* TRUE if input matrix full, FALSE if sparse */
    double knobs [CCOLAMD_KNOBS] ; /* ccolamd user-controllable parameters */
    double *out_perm ;          /* output permutation vector */
    double *out_stats ;         /* output stats vector */
    double *in_knobs ;          /* input knobs vector */
    Long i ;                    /* loop counter */
    mxArray *Ainput ;           /* input matrix handle */
    Long spumoni ;              /* verbosity variable */
    Long stats2 [CCOLAMD_STATS] ;/* stats for csymamd */

    Long *cp, *cp_end, result, nnz, col, length, ok ;
    Long *stats ;
    stats = stats2 ;

    /* === Check inputs ===================================================== */

    if (nargin != 3 || nargout > 2)
    {
	mexErrMsgTxt (
	"csymamdtest: incorrect number of input and/or output arguments.") ;
    }
    /* for testing we require all 4 knobs */
    if (mxGetNumberOfElements (pargin [1]) != 4)
    {
	mexErrMsgTxt ("csymamdtest: must have 4 knobs for testing") ;
    }

    /* === Get knobs ======================================================== */

    ccolamd_l_set_defaults (knobs) ;
    spumoni = 0 ;

    in_knobs = mxGetPr (pargin [1]) ;

    i = mxGetNumberOfElements (pargin [1]) ;
    knobs [CCOLAMD_DENSE_ROW] = in_knobs [0] ;
    knobs [CCOLAMD_DENSE_COL] = in_knobs [0] ;
    knobs [CCOLAMD_AGGRESSIVE] = (in_knobs [1] != 0) ;
    spumoni = (in_knobs [2] != 0) ;

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
	mexPrintf ("Testing: %g\n", in_knobs [3]) ;
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

    /* p = mxGetJc (Ainput) ; */
    p = (Long *) mxCalloc (n_col+1, sizeof (Long)) ;
    (void) memcpy (p, mxGetJc (Ainput), (n_col+1)*sizeof (Long)) ;

    nnz = p [n_col] ;
    if (spumoni)
    {
	mexPrintf ("csymamdtest: nnz %d\n", nnz) ;
    }

    /* A = mxGetIr (Ainput) ; */
    A = (Long *) mxCalloc (nnz+1, sizeof (Long)) ;
    (void) memcpy (A, mxGetIr (Ainput), nnz*sizeof (Long)) ;

    perm = (Long *) mxCalloc (n_col+1, sizeof (Long)) ;

    /* === Jumble matrix ==================================================== */


    /*
	knobs [4]	FOR TESTING ONLY: Specifies how to jumble matrix
			0 : No jumbling
			1 : (no errors)
			2 : Make first pointer non-zero
			3 : Make column pointers not non-decreasing
			4 : (no errors)
			5 : Make row indices not strictly increasing
			6 : Make a row index greater or equal to n_row
			7 : Set A = NULL
			8 : Set p = NULL
			9 : Repeat row index
			10: make row indices not sorted
			11: jumble columns massively (note this changes
				the pattern of the matrix A.)
			12: Set stats = NULL
			13: Make n_col less than zero
    */

    /* jumble appropriately */
    switch ((Long) in_knobs [3])
    {

	case 0 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: no errors expected\n") ;
	    }
	    result = 1 ;		/* no errors */
	    break ;

	case 1 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: no errors expected (1)\n") ;
	    }
	    result = 1 ;
	    break ;

	case 2 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: p [0] nonzero\n") ;
	    }
	    result = 0 ;		/* p [0] must be zero */
	    p [0] = 1 ;
	    break ;

	case 3 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: negative length last column\n") ;
	    }
	    result = (n_col == 0) ;	/* p must be monotonically inc. */
	    p [n_col] = p [0] ;
	    break ;

	case 4 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: no errors expected (4)\n") ;
	    }
	    result = 1 ;
	    break ;

	case 5 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: row index out of range (-1)\n") ;
	    }
	    if (nnz > 0)		/* row index out of range */
	    {
		result = 0 ;
		A [nnz-1] = -1 ;
	    }
	    else
	    {
	        if (spumoni)
		{
		    mexPrintf ("Note: no row indices to put out of range\n") ;
		}
		result = 1 ;
	    }
	    break ;

	case 6 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: row index out of range (ncol)\n") ;
	    }
	    if (nnz > 0)		/* row index out of range */
	    {
		result = 0 ;
		A [nnz-1] = n_col ;
	    }
	    else
	    {
	        if (spumoni)
		{
		    mexPrintf ("Note: no row indices to put out of range\n") ;
		}
		result = 1 ;
	    }
	    break ;

	case 7 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: A not present\n") ;
	    }
	    result = 0 ;		/* A not present */
	    A = (Long *) NULL ;
	    break ;

	case 8 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: p not present\n") ;
	    }
	    result = 0 ;		/* p not present */
	    p = (Long *) NULL ;
	    break ;

	case 9 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: duplicate row index\n") ;
	    }
	    result = 1 ;		/* duplicate row index */

	    for (col = 0 ; col < n_col ; col++)
	    {
		length = p [col+1] - p [col] ;
	    	if (length > 1)
		{
		    A [p [col+1]-2] = A [p [col+1] - 1] ;
		    if (spumoni)
		    {
			mexPrintf ("Made duplicate row %d in col %d\n",
		    	 A [p [col+1] - 1], col) ;
		    }
		    break ;
		}
	    }

	    if (spumoni > 1)
	    {
		dump_matrix (A, p, n_row, n_col, nnz, col+2) ;
	    }
	    break ;

	case 10 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: unsorted column\n") ;
	    }
	    result = 1 ;		/* jumbled columns */

	    for (col = 0 ; col < n_col ; col++)
	    {
		length = p [col+1] - p [col] ;
	    	if (length > 1)
		{
		    i = A[p [col]] ;
		    A [p [col]] = A[p [col] + 1] ;
		    A [p [col] + 1] = i ;
		    if (spumoni)
		    {
			mexPrintf ("Unsorted column %d \n", col) ;
		    }
		    break ;
		}
	    }

	    if (spumoni > 1)
	    {
		dump_matrix (A, p, n_row, n_col, nnz, col+2) ;
	    }
	    break ;

	case 11 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: massive jumbling\n") ;
	    }
	    result = 1 ;		/* massive jumbling, but no errors */
	    srand (1) ;
	    for (i = 0 ; i < n_col ; i++)
	    {
		cp = &A [p [i]] ;
		cp_end = &A [p [i+1]] ;
		while (cp < cp_end)
		{
		    *cp++ = rand() % n_row ;
		}
	    }
	    if (spumoni > 1)
	    {
		dump_matrix (A, p, n_row, n_col, nnz, n_col) ;
	    }
	    break ;

	case 12 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: stats not present\n") ;
	    }
	    result = 0 ;		/* stats not present */
	    stats = (Long *) NULL ;
	    break ;

	case 13 :
	    if (spumoni)
	    {
		mexPrintf ("csymamdtest: ncol out of range\n") ;
	    }
	    result = 0 ;		/* ncol out of range */
	    n_col = -1 ;
	    break ;

    }

    /* === Order the rows and columns of A (does not destroy A) ============= */

    ok = csymamd_l (n_col, A, p, perm, knobs, stats, &mxCalloc, &mxFree,
	    NULL, -1) ;

    if (full)
    {
	mxDestroyArray (Ainput) ;
    }

    if (spumoni)
    {
	csymamd_l_report (stats) ;
    }

    /* === Return the stats vector ========================================== */

    if (nargout == 2)
    {
	pargout [1] = mxCreateDoubleMatrix (1, CCOLAMD_STATS, mxREAL) ;
	out_stats = mxGetPr (pargout [1]) ;
	for (i = 0 ; i < CCOLAMD_STATS ; i++)
	{
	    out_stats [i] = (stats == NULL) ? (-1) : (stats [i]) ;
	}
	/* fix stats (5) and (6), for 1-based information on jumbled matrix. */
	/* note that this correction doesn't occur if csymamd returns FALSE */
	out_stats [CCOLAMD_INFO1] ++ ; 
	out_stats [CCOLAMD_INFO2] ++ ; 
    }

    mxFree (A) ;

    if (ok)
    {

	/* === Return the permutation vector ================================ */

	pargout [0] = mxCreateDoubleMatrix (1, n_col, mxREAL) ;
	out_perm = mxGetPr (pargout [0]) ;
	for (i = 0 ; i < n_col ; i++)
	{
	    /* csymamd is 0-based, but MATLAB expects this to be 1-based */
	    out_perm [i] = perm [i] + 1 ;
	}
	if (!result)
	{
	    csymamd_l_report (stats) ;
	    mexErrMsgTxt ("csymamd should have returned TRUE\n") ;
	}
    }
    else
    {

	/* return p = -1 if csymamd failed */
	pargout [0] = mxCreateDoubleMatrix (1, 1, mxREAL) ;
	out_perm = mxGetPr (pargout [0]) ;
	out_perm [0] = -1 ;
	if (result)
	{
	    csymamd_l_report (stats) ;
	    mexErrMsgTxt ("csymamd should have returned FALSE\n") ;
	}
    }

    mxFree (p) ;
    mxFree (perm) ;
}
