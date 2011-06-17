/* ========================================================================== */
/* === ccolamdtest mexFunction ============================================== */
/* ========================================================================== */

/* ----------------------------------------------------------------------------
 * CCOLAMD Copyright (C), Univ. of Florida.  Authors: Timothy A. Davis,
 * Sivasankaran Rajamanickam, and Stefan Larimore
 * See License.txt for the Version 2.1 of the GNU Lesser General Public License
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/*
 *  This MATLAB mexFunction is for testing only.  It is not meant for
 *  production use.  See ccolamdmex.c and ccolamd.m instead.
 *
 *  Usage:
 *
 *	[ P, stats ] = ccolamdtest (A, knobs) ;
 *
 *  The stats vector is optional.  knobs is required:
 *
 *	knobs (1)	order for LU if nonzero, Cholesky otherwise. default 0
 *	knobs (2)	spumoni, default 0.
 *	knobs (3)	dense row control. default 10
 *	knobs (4)	dense column control. default 10
 *	knobs (5)	aggresive absorption if nonzero. default: 1
 *
 *	knobs (6)	for testing only.  Controls the workspace used.
 *	knobs (7)	for testing only.  Controls how the input matrix is
 *			jumbled prior to calling colamd, to test its error
 *			handling capability.
 *
 *	see ccolamd.c for a description of the stats array.
 *
 */

/* ========================================================================== */
/* === Include files ======================================================== */
/* ========================================================================== */

#include "ccolamd.h"
#include "mex.h"
#include "matrix.h"
#include <stdlib.h>
#include <string.h>
#include "UFconfig.h"

/* Here only for testing */
#undef MIN
#undef MAX
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define CCOLAMD_MIN_MEMORY(nnz,n_row,n_col) \
	(MAX (2 * nnz, 4 * n_col) + \
	    8*n_col + 6*n_row + n_col + (nnz / 5) \
	    + ((3 * n_col) + 1)  + 5 * (n_col + 1) + n_row) 

/* ========================================================================== */
/* === dump_matrix ========================================================== */
/* ========================================================================== */

static void dump_matrix
(
    UF_long A [ ],
    UF_long p [ ],
    UF_long n_row,
    UF_long n_col,
    UF_long Alen,
    UF_long limit
)
{
    UF_long col, k, row ;

    mexPrintf ("dump matrix: nrow %d ncol %d Alen %d\n", n_row, n_col, Alen) ;

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
/* === ccolamd mexFunction ================================================== */
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

    UF_long *A ;		/* ccolamd's copy of the matrix and workspace */
    UF_long *p ;		/* ccolamd's copy of the column pointers */
    UF_long Alen ;		/* size of A */
    UF_long n_col ;		/* number of columns of A */
    UF_long n_row ;		/* number of rows of A */
    UF_long nnz ;		/* number of entries in A */
    UF_long full ;		/* TRUE if input matrix full, FALSE if sparse */
    double knobs [CCOLAMD_KNOBS] ; /* ccolamd user-controllable parameters */
    double *out_perm ;		/* output permutation vector */
    double *out_stats ;		/* output stats vector */
    double *in_knobs ;		/* input knobs vector */
    UF_long i ;			/* loop counter */
    mxArray *Ainput ;		/* input matrix handle */
    UF_long spumoni ;		/* verbosity variable */
    UF_long stats2 [CCOLAMD_STATS] ;	/* stats for ccolamd */

    UF_long *cp, *cp_end, result, col, length, ok ;
    UF_long *stats ;
    stats = stats2 ;

    /* === Check inputs ===================================================== */

    if (nargin != 3 || nargout < 0 || nargout > 2)
    {
	mexErrMsgTxt (
	"ccolamdtest: incorrect number of input and/or output arguments") ;
    }
    /* for testing we require all 7 knobs */
    if (mxGetNumberOfElements (pargin [1]) != 7)
    {
	mexErrMsgTxt ("ccolamdtest: must have all 7 knobs for testing") ;
    }

    /* === Get knobs ======================================================== */

    ccolamd_l_set_defaults (knobs) ;
    spumoni = 0 ;

    in_knobs = mxGetPr (pargin [1]) ;
    knobs [CCOLAMD_LU] = (in_knobs [0] != 0) ;
    knobs [CCOLAMD_DENSE_ROW]  = in_knobs [1] ;
    knobs [CCOLAMD_DENSE_COL]  = in_knobs [2] ;
    knobs [CCOLAMD_AGGRESSIVE] = (in_knobs [3] != 0) ;
    spumoni = (in_knobs [4] != 0) ;

    /* print knob settings if spumoni is set */
    if (spumoni)
    {
	mexPrintf ("\nccolamd version %d.%d, %s:\nknobs(1): %g, order for %s\n",
	    CCOLAMD_MAIN_VERSION, CCOLAMD_SUB_VERSION, CCOLAMD_DATE,
	    in_knobs [0],
	    (knobs [CCOLAMD_LU] != 0) ? "lu(A)" : "chol(A'*A)") ;
	if (knobs [CCOLAMD_DENSE_ROW] >= 0)
	{
	    mexPrintf ("knobs(2): %g, rows with > max(16,%g*sqrt(size(A,2)))"
		" entries removed\n", in_knobs [1], knobs [CCOLAMD_DENSE_ROW]) ;
	}
	else
	{
	    mexPrintf ("knobs(2): %g, no dense rows removed\n", in_knobs [1]) ;
	}
	if (knobs [CCOLAMD_DENSE_COL] >= 0)
	{
	    mexPrintf ("knobs(3): %g, cols with > max(16,%g*sqrt(min(size(A)))"
		" entries removed\n", in_knobs [2], knobs [CCOLAMD_DENSE_COL]) ;
	}
	else
	{
	    mexPrintf ("knobs(3): no dense columns removed\n", in_knobs [2]) ;
	}
	mexPrintf ("knobs(4): %g, aggressive absorption: %s\n",
	    in_knobs [3], (knobs [CCOLAMD_AGGRESSIVE] != 0) ? "yes" : "no") ;
	mexPrintf ("knobs(5): %g, statistics and knobs printed\n",
	    in_knobs [4]) ;
	mexPrintf ("Testing: %g %g\n", in_knobs [5], in_knobs[6]) ;
    }

    /* === If A is full, convert to a sparse matrix ========================= */

    Ainput = (mxArray *) pargin [0] ;
    if (mxGetNumberOfDimensions (Ainput) != 2)
    {
	mexErrMsgTxt ("ccolamd: input matrix must be 2-dimensional") ;
    }
    full = !mxIsSparse (Ainput) ;
    if (full)
    {
	mexCallMATLAB (1, &Ainput, 1, (mxArray **) pargin, "sparse") ;
    }

    /* === Allocate workspace for ccolamd =================================== */

    /* get size of matrix */
    n_row = mxGetM (Ainput) ;
    n_col = mxGetN (Ainput) ;

    /* get column pointer vector so we can find nnz */
    p = (UF_long *) mxCalloc (n_col+1, sizeof (UF_long)) ;
    (void) memcpy (p, mxGetJc (Ainput), (n_col+1)*sizeof (UF_long)) ;
    nnz = p [n_col] ;
    Alen = (UF_long) ccolamd_l_recommended (nnz, n_row, n_col) ;
    if (Alen == 0)
    {
    	mexErrMsgTxt ("ccolamd: problem too large") ;
    }

    /* === Modify size of Alen if testing =================================== */
    /*
	knobs [5]	amount of workspace given to colamd.
			<  0 : TIGHT memory
			>  0 : MIN + knob [3] - 1
			== 0 : RECOMMENDED memory
    */

    /* get knob [5], if negative */
    if (in_knobs [5] < 0)
    {
	Alen = CCOLAMD_MIN_MEMORY (nnz, n_row, n_col) + n_col ;
    }
    else if (in_knobs [5] > 0)
    {
	Alen = CCOLAMD_MIN_MEMORY (nnz, n_row, n_col) + in_knobs [5] - 1 ;
    }

    /* otherwise, we use the recommended amount set above */

    /* === Copy input matrix into workspace ================================= */

    A = (UF_long *) mxCalloc (Alen, sizeof (UF_long)) ;
    (void) memcpy (A, mxGetIr (Ainput), nnz*sizeof (UF_long)) ;

    if (full)
    {
	mxDestroyArray (Ainput) ;
    }


    /* === Jumble matrix ==================================================== */

    /*
	knobs [6]	FOR TESTING ONLY: Specifies how to jumble matrix
			0 : No jumbling
			1 : Make n_row less than zero
			2 : Make first pointer non-zero
			3 : Make column pointers not non-decreasing
			4 : Make a column pointer greater or equal to Alen
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
    switch ((UF_long) in_knobs [6])
    {

	case 0 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: no errors expected\n") ;
	    }
	    result = 1 ;		/* no errors */
	    break ;

	case 1 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: nrow out of range\n") ;
	    }
	    result = 0 ;		/* nrow out of range */
	    n_row = -1 ;
	    break ;

	case 2 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: p [0] nonzero\n") ;
	    }
	    result = 0 ;		/* p [0] must be zero */
	    p [0] = 1 ;
	    break ;

	case 3 :
	    if (spumoni)
	    {
		mexPrintf ("colamdtest: negative length last column\n") ;
	    }
	    result = (n_col == 0) ;	/* p must be monotonically inc. */
	    p [n_col] = p [0] ;
	    break ;

	case 4 :
	    if (spumoni)
	    {
		mexPrintf ("colamdtest: Alen too small\n") ;
	    }
	    result = 0 ;		/* out of memory */
	    p [n_col] = Alen ;
	    break ;

	case 5 :
	    if (spumoni)
	    {
		mexPrintf ("colamdtest: row index out of range (-1)\n") ;
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
		mexPrintf ("ccolamdtest: row index out of range (n_row)\n") ;
	    }
	    if (nnz > 0)		/* row index out of range */
	    {
		if (spumoni)
		{
		    mexPrintf ("Changing A[nnz-1] from %d to %d\n",
			    A [nnz-1], n_row) ; 
		}
		result = 0 ;
		A [nnz-1] = n_row ;
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
		mexPrintf ("ccolamdtest: A not present\n") ;
	    }
	    result = 0 ;		/* A not present */
	    A = (UF_long *) NULL ;
	    break ;

	case 8 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: p not present\n") ;
	    }
	    result = 0 ;		/* p not present */
	    p = (UF_long *) NULL ;
	    break ;

	case 9 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: duplicate row index\n") ;
	    }
	    result = 1 ;		/* duplicate row index */

	    for (col = 0 ; col < n_col ; col++)
	    {
		length = p [col+1] - p [col] ;
	    	if (length > 1)
		{
		    A [p [col]] = A [p [col] + 1] ;
		    if (spumoni)
		    {
			mexPrintf ("Made duplicate row %d in col %d\n",
		    	 A [p [col] + 1], col) ;
		    }
		    break ;
		}
	    }

	    if (spumoni > 1)
	    {
		dump_matrix (A, p, n_row, n_col, Alen, col+2) ;
	    }
	    break ;

	case 10 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: unsorted column\n") ;
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
		dump_matrix (A, p, n_row, n_col, Alen, col+2) ;
	    }
	    break ;

	case 11 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: massive jumbling\n") ;
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
		dump_matrix (A, p, n_row, n_col, Alen, n_col) ;
	    }
	    break ;

	case 12 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: stats not present\n") ;
	    }
	    result = 0 ;		/* stats not present */
	    stats = (UF_long *) NULL ;
	    break ;

	case 13 :
	    if (spumoni)
	    {
		mexPrintf ("ccolamdtest: ncol out of range\n") ;
	    }
	    result = 0 ;		/* ncol out of range */
	    n_col = -1 ;
	    break ;

    }


    /* === Order the columns (destroys A) =================================== */

    ok = ccolamd_l (n_row, n_col, Alen, A, p, knobs, stats, NULL) ;
    if (spumoni)
    {
	ccolamd_l_report (stats) ;
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
	    /* ccolamd is 0-based, but MATLAB expects this to be 1-based */
	    out_perm [i] = p [i] + 1 ;
	}
	if (!result)
	{
	    ccolamd_l_report (stats) ;
	    mexErrMsgTxt ("ccolamd should have returned TRUE\n") ;
	}
    }
    else
    {

	/* return p = -1 if ccolamd failed */
	pargout [0] = mxCreateDoubleMatrix (1, 1, mxREAL) ;
	out_perm = mxGetPr (pargout [0]) ;
	out_perm [0] = -1 ;
	if (result)
	{
	    ccolamd_l_report (stats) ;
	    mexErrMsgTxt ("ccolamd should have returned FALSE\n") ;
	}
    }

    mxFree (p) ;
}
