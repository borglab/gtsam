/* ========================================================================== */
/* === Core/cholmod_common ================================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Core Module.  Copyright (C) 2005-2006,
 * Univ. of Florida.  Author: Timothy A. Davis
 * The CHOLMOD/Core Module is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* Core utility routines for the cholmod_common object:
 *
 * Primary routines:
 * -----------------
 * cholmod_start		the first call to CHOLMOD
 * cholmod_finish		the last call to CHOLMOD
 *
 * Secondary routines:
 * -------------------
 * cholmod_defaults		restore (most) default control parameters
 * cholmod_allocate_work	allocate (or reallocate) workspace in Common
 * cholmod_free_work		free workspace in Common
 * cholmod_clear_flag		clear Common->Flag in workspace
 * cholmod_maxrank		column dimension of Common->Xwork workspace
 *
 * The Common object is unique.  It cannot be allocated or deallocated by
 * CHOLMOD, since it contains the definition of the memory management routines
 * used (pointers to malloc, free, realloc, and calloc, or their equivalent).
 * The Common object contains workspace that is used between calls to
 * CHOLMOD routines.  This workspace allocated by CHOLMOD as needed, by
 * cholmod_allocate_work and cholmod_free_work.
 */

#include "cholmod_internal.h"
#include "cholmod_core.h"

/* ========================================================================== */
/* === cholmod_start ======================================================== */
/* ========================================================================== */

/* Initialize Common default parameters and statistics.  Sets workspace
 * pointers to NULL.
 *
 * This routine must be called just once, prior to calling any other CHOLMOD
 * routine.  Do not call this routine after any other CHOLMOD routine (except
 * cholmod_finish, to start a new CHOLMOD session), or a memory leak will
 * occur.
 *
 * workspace: none
 */

int CHOLMOD(start)
(
    cholmod_common *Common
)
{
    int k ;

    if (Common == NULL)
    {
	return (FALSE) ;
    }

    /* ---------------------------------------------------------------------- */
    /* user error handling routine */
    /* ---------------------------------------------------------------------- */

    Common->error_handler = NULL ;

    /* ---------------------------------------------------------------------- */
    /* integer and numerical types */
    /* ---------------------------------------------------------------------- */

    Common->itype = ITYPE ;
    Common->dtype = DTYPE ;

    /* ---------------------------------------------------------------------- */
    /* default control parameters */
    /* ---------------------------------------------------------------------- */

    cholmod_l_defaults(Common) ;
    Common->try_catch = FALSE ;

    /* ---------------------------------------------------------------------- */
    /* memory management routines */
    /* ---------------------------------------------------------------------- */

    /* The user can replace cholmod's memory management routines by redefining
     * these function pointers. */

#ifndef NMALLOC
    /* stand-alone ANSI C program */
    Common->malloc_memory  = malloc ;
    Common->free_memory    = free ;
    Common->realloc_memory = realloc ;
    Common->calloc_memory  = calloc ;
#else
    /* no memory manager defined at compile-time; MUST define one at run-time */
    Common->malloc_memory  = NULL ;
    Common->free_memory    = NULL ;
    Common->realloc_memory = NULL ;
    Common->calloc_memory  = NULL ;
#endif

    /* ---------------------------------------------------------------------- */
    /* complex arithmetic routines */
    /* ---------------------------------------------------------------------- */

//    Common->complex_divide = CHOLMOD(divcomplex) ;
//    Common->hypotenuse = CHOLMOD(hypot) ;

    /* ---------------------------------------------------------------------- */
    /* print routine */
    /* ---------------------------------------------------------------------- */

#ifndef NPRINT
    /* stand-alone ANSI C program */
    Common->print_function = printf ;
#else
    /* printing disabled */
    Common->print_function = NULL ;
#endif

    /* ---------------------------------------------------------------------- */
    /* workspace */
    /* ---------------------------------------------------------------------- */

    /* This code assumes the workspace held in Common is not initialized.  If
     * it is, then a memory leak will occur because the pointers are
     * overwritten with NULL. */

    Common->nrow = 0 ;
    Common->mark = EMPTY ;
    Common->xworksize = 0 ;
    Common->iworksize = 0 ;
    Common->Flag = NULL ;
    Common->Head = NULL ;
    Common->Iwork = NULL ;
    Common->Xwork = NULL ;
    Common->no_workspace_reallocate = FALSE ;

    /* ---------------------------------------------------------------------- */
    /* statistics */
    /* ---------------------------------------------------------------------- */

    /* fl and lnz are computed in cholmod_analyze and cholmod_rowcolcounts */
    Common->fl = EMPTY ;
    Common->lnz = EMPTY ;

    /* modfl is computed in cholmod_updown, cholmod_rowadd, and cholmod_rowdel*/
    Common->modfl = EMPTY ;

    /* all routines use status as their error-report code */
    Common->status = CHOLMOD_OK ;

    Common->malloc_count = 0 ;	/* # calls to malloc minus # calls to free */
    Common->memory_usage = 0 ;	/* peak memory usage (in bytes) */
    Common->memory_inuse = 0 ;	/* current memory in use (in bytes) */

    Common->nrealloc_col = 0 ;
    Common->nrealloc_factor = 0 ;
    Common->ndbounds_hit = 0 ;
    Common->rowfacfl = 0 ;
    Common->aatfl = EMPTY ;

    /* Common->called_nd is TRUE if cholmod_analyze called or NESDIS */
    Common->called_nd = FALSE ;

    Common->blas_ok = TRUE ;    /* false if BLAS int overflow occurs */

    /* ---------------------------------------------------------------------- */
    /* default SuiteSparseQR knobs and statististics */
    /* ---------------------------------------------------------------------- */

    for (k = 0 ; k < 4  ; k++) Common->SPQR_xstat [k] = 0 ;
    for (k = 0 ; k < 10 ; k++) Common->SPQR_istat [k] = 0 ;
    Common->SPQR_grain = 1 ;    /* no Intel TBB multitasking, by default */
    Common->SPQR_small = 1e6 ;  /* target min task size for TBB */
    Common->SPQR_shrink = 1 ;   /* controls SPQR shrink realloc */
    Common->SPQR_nthreads = 0 ; /* 0: let TBB decide how many threads to use */

    DEBUG_INIT ("cholmod start", Common) ;
    return (TRUE) ;
}


/* ========================================================================== */
/* === cholmod_defaults ===================================================== */
/* ========================================================================== */

/* Set Common default parameters, except for the function pointers.
 *
 * workspace: none
 */

int CHOLMOD(defaults)
(
    cholmod_common *Common
)
{
    Int i ;

    RETURN_IF_NULL_COMMON (FALSE) ;

    /* ---------------------------------------------------------------------- */
    /* default control parameters */
    /* ---------------------------------------------------------------------- */

    Common->dbound = 0.0 ;
    Common->grow0 = 1.2 ;
    Common->grow1 = 1.2 ;
    Common->grow2 = 5 ;
    Common->maxrank = 8 ;

    Common->final_asis = TRUE ;
    Common->final_super = TRUE ;
    Common->final_ll = FALSE ;
    Common->final_pack = TRUE ;
    Common->final_monotonic = TRUE ;
    Common->final_resymbol = FALSE ;

    /* use simplicial factorization if flop/nnz(L) < 40, supernodal otherwise */
    Common->supernodal = CHOLMOD_AUTO ;
    Common->supernodal_switch = 40 ;

    Common->nrelax [0] = 4 ;
    Common->nrelax [1] = 16 ;
    Common->nrelax [2] = 48 ;
    Common->zrelax [0] = 0.8 ;
    Common->zrelax [1] = 0.1 ;
    Common->zrelax [2] = 0.05 ;

    Common->prefer_zomplex = FALSE ;
    Common->prefer_upper = TRUE ;
    Common->prefer_binary = FALSE ;
    Common->quick_return_if_not_posdef = FALSE ;

    /* METIS workarounds */
    Common->metis_memory = 0.0 ;    /* > 0 for memory guard (2 is reasonable) */
    Common->metis_nswitch = 3000 ;
    Common->metis_dswitch = 0.66 ;

    Common->print = 3 ;
    Common->precise = FALSE ;

    /* ---------------------------------------------------------------------- */
    /* default ordering methods */
    /* ---------------------------------------------------------------------- */

    /* Note that if the Partition module is not installed, the CHOLMOD_METIS
     * and CHOLMOD_NESDIS methods will not be available.  cholmod_analyze will
     * report the CHOLMOD_NOT_INSTALLED error, and safely skip over them.
     */

#if (CHOLMOD_MAXMETHODS < 9)
#error "CHOLMOD_MAXMETHODS must be 9 or more (defined in cholmod_core.h)."
#endif

    /* default strategy: try given, AMD, and then METIS if AMD reports high
     * fill-in.  NESDIS can be used instead, if Common->default_nesdis is TRUE.
     */
    Common->nmethods = 0 ;		/* use default strategy */
    Common->default_nesdis = FALSE ;	/* use METIS in default strategy */

    Common->current = 0 ;	/* current method being tried */
    Common->selected = 0 ;	/* the best method selected */

    /* first, fill each method with default parameters */
    for (i = 0 ; i <= CHOLMOD_MAXMETHODS ; i++)
    {
	/* CHOLMOD's default method is AMD for A or AA' */
	Common->method [i].ordering = CHOLMOD_AMD ;

	/* CHOLMOD nested dissection and minimum degree parameter */
	Common->method [i].prune_dense = 10.0 ;	/* dense row/col control */

	/* min degree parameters (AMD, COLAMD, SYMAMD, CAMD, CCOLAMD, CSYMAMD)*/
	Common->method [i].prune_dense2 = -1 ;	/* COLAMD dense row control */
	Common->method [i].aggressive = TRUE ;	/* aggressive absorption */
	Common->method [i].order_for_lu = FALSE ;/* order for Cholesky not LU */

	/* CHOLMOD's nested dissection (METIS + constrained AMD) */
	Common->method [i].nd_small = 200 ;	/* small graphs aren't cut */
	Common->method [i].nd_compress = TRUE ;	/* compress graph & subgraphs */
	Common->method [i].nd_camd = 1 ;	/* use CAMD */
	Common->method [i].nd_components = FALSE ;  /* lump connected comp. */
	Common->method [i].nd_oksep = 1.0 ;	/* sep ok if < oksep*n */

	/* statistics for each method are not yet computed */
	Common->method [i].fl = EMPTY ;
	Common->method [i].lnz = EMPTY ;
    }

    Common->postorder = TRUE ;	/* follow ordering with weighted postorder */

    /* Next, define some methods.  The first five use default parameters. */
    Common->method [0].ordering = CHOLMOD_GIVEN ;   /* skip if UserPerm NULL */
    Common->method [1].ordering = CHOLMOD_AMD ;
    Common->method [2].ordering = CHOLMOD_METIS ;
    Common->method [3].ordering = CHOLMOD_NESDIS ;
    Common->method [4].ordering = CHOLMOD_NATURAL ;

    /* CHOLMOD's nested dissection with large leaves of separator tree */
    Common->method [5].ordering = CHOLMOD_NESDIS ;
    Common->method [5].nd_small = 20000 ;

    /* CHOLMOD's nested dissection with tiny leaves, and no AMD ordering */
    Common->method [6].ordering = CHOLMOD_NESDIS ;
    Common->method [6].nd_small = 4 ;
    Common->method [6].nd_camd = 0 ;		/* no CSYMAMD or CAMD */

    /* CHOLMOD's nested dissection with no dense node removal */
    Common->method [7].ordering = CHOLMOD_NESDIS ;
    Common->method [7].prune_dense = -1. ;

    /* COLAMD for A*A', AMD for A */
    Common->method [8].ordering = CHOLMOD_COLAMD ;

    return (TRUE) ;
}


/* ========================================================================== */
/* === cholmod_finish ======================================================= */
/* ========================================================================== */

/* The last call to CHOLMOD must be cholmod_finish.  You may call this routine
 * more than once, and can safely call any other CHOLMOD routine after calling
 * it (including cholmod_start).
 *
 * The statistics and parameter settings in Common are preserved.  The
 * workspace in Common is freed.  This routine is just another name for
 * cholmod_free_work.
 */

int CHOLMOD(finish)
(
    cholmod_common *Common
)
{
    return (CHOLMOD(free_work) (Common)) ;
}


/* ========================================================================== */
/* === cholmod_allocate_work ================================================ */
/* ========================================================================== */

/* Allocate and initialize workspace for CHOLMOD routines, or increase the size
 * of already-allocated workspace.  If enough workspace is already allocated,
 * then nothing happens.
 *
 * workspace: Flag (nrow), Head (nrow+1), Iwork (iworksize), Xwork (xworksize)
 */

int CHOLMOD(allocate_work)
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows in the matrix A */
    size_t iworksize,	/* size of Iwork */
    size_t xworksize,	/* size of Xwork */
    /* --------------- */
    cholmod_common *Common
)
{
    double *W ;
    Int *Head ;
    Int i ;
    size_t nrow1 ;
    int ok = TRUE ;

    /* ---------------------------------------------------------------------- */
    /* get inputs */
    /* ---------------------------------------------------------------------- */

    RETURN_IF_NULL_COMMON (FALSE) ;
    Common->status = CHOLMOD_OK ;

    /* ---------------------------------------------------------------------- */
    /* Allocate Flag (nrow) and Head (nrow+1) */
    /* ---------------------------------------------------------------------- */

    nrow = MAX (1, nrow) ;

    /* nrow1 = nrow + 1 */
    nrow1 = CHOLMOD(add_size_t) (nrow, 1, &ok) ;
    if (!ok)
    {
	/* nrow+1 causes size_t overflow ; problem is too large */
	Common->status = CHOLMOD_TOO_LARGE ;
	CHOLMOD(free_work) (Common) ;
	return (FALSE) ;
    }

    if (nrow > Common->nrow)
    {

	if (Common->no_workspace_reallocate)
	{
	    /* CHOLMOD is not allowed to change the workspace here */
	    Common->status = CHOLMOD_INVALID ;
	    return (FALSE) ;
	}

	/* free the old workspace (if any) and allocate new space */
	Common->Flag = CHOLMOD(free) (Common->nrow,  sizeof (Int), Common->Flag,
		Common) ;
	Common->Head = CHOLMOD(free) (Common->nrow+1,sizeof (Int), Common->Head,
		Common) ;
	Common->Flag = CHOLMOD(malloc) (nrow,   sizeof (Int), Common) ;
	Common->Head = CHOLMOD(malloc) (nrow1, sizeof (Int), Common) ;

	/* record the new size of Flag and Head */
	Common->nrow = nrow ;

	if (Common->status < CHOLMOD_OK)
	{
	    CHOLMOD(free_work) (Common) ;
	    return (FALSE) ;
	}

	/* initialize Flag and Head */
	Common->mark = EMPTY ;
	CHOLMOD(clear_flag) (Common) ;
	Head = Common->Head ;
	for (i = 0 ; i <= (Int) (nrow) ; i++)
	{
	    Head [i] = EMPTY ;
	}
    }

    /* ---------------------------------------------------------------------- */
    /* Allocate Iwork (iworksize) */
    /* ---------------------------------------------------------------------- */

    iworksize = MAX (1, iworksize) ;
    if (iworksize > Common->iworksize)
    {

	if (Common->no_workspace_reallocate)
	{
	    /* CHOLMOD is not allowed to change the workspace here */
	    Common->status = CHOLMOD_INVALID ;
	    return (FALSE) ;
	}

	/* free the old workspace (if any) and allocate new space.
	 * integer overflow safely detected in cholmod_malloc */
	CHOLMOD(free) (Common->iworksize, sizeof (Int), Common->Iwork, Common) ;
	Common->Iwork = CHOLMOD(malloc) (iworksize, sizeof (Int), Common) ;

	/* record the new size of Iwork */
	Common->iworksize = iworksize ;

	if (Common->status < CHOLMOD_OK)
	{
	    CHOLMOD(free_work) (Common) ;
	    return (FALSE) ;
	}

	/* note that Iwork does not need to be initialized */
    }

    /* ---------------------------------------------------------------------- */
    /* Allocate Xwork (xworksize) and set it to ((double) 0.) */
    /* ---------------------------------------------------------------------- */

    /* make sure xworksize is >= 1 */
    xworksize = MAX (1, xworksize) ;
    if (xworksize > Common->xworksize)
    {

	if (Common->no_workspace_reallocate)
	{
	    /* CHOLMOD is not allowed to change the workspace here */
	    Common->status = CHOLMOD_INVALID ;
	    return (FALSE) ;
	}

	/* free the old workspace (if any) and allocate new space */
	CHOLMOD(free) (Common->xworksize, sizeof (double), Common->Xwork,
		Common) ;
	Common->Xwork = CHOLMOD(malloc) (xworksize, sizeof (double), Common) ;

	/* record the new size of Xwork */
	Common->xworksize = xworksize ;

	if (Common->status < CHOLMOD_OK)
	{
	    CHOLMOD(free_work) (Common) ;
	    return (FALSE) ;
	}

	/* initialize Xwork */
	W = Common->Xwork ;
	for (i = 0 ; i < (Int) xworksize ; i++)
	{
	    W [i] = 0. ;
	}
    }

    return (TRUE) ;
}


/* ========================================================================== */
/* === cholmod_free_work ==================================================== */
/* ========================================================================== */

/* Deallocate the CHOLMOD workspace.
 *
 * workspace: deallocates all workspace in Common
 */

int CHOLMOD(free_work)
(
    cholmod_common *Common
)
{
    RETURN_IF_NULL_COMMON (FALSE) ;
    Common->Flag  = CHOLMOD(free) (Common->nrow, sizeof (Int),
	    Common->Flag, Common) ;
    Common->Head  = CHOLMOD(free) (Common->nrow+1, sizeof (Int),
	    Common->Head, Common) ;
    Common->Iwork = CHOLMOD(free) (Common->iworksize, sizeof (Int),
	    Common->Iwork, Common) ;
    Common->Xwork = CHOLMOD(free) (Common->xworksize, sizeof (double),
	    Common->Xwork, Common) ;
    Common->nrow = 0 ;
    Common->iworksize = 0 ;
    Common->xworksize = 0 ;
    return (TRUE) ;
}


/* ========================================================================== */
/* === cholmod_clear_flag =================================================== */
/* ========================================================================== */

/* Increment mark to ensure Flag [0..nrow-1] < mark.  If integer overflow
 * occurs, or mark was initially negative, reset the entire array.  This is
 * not an error condition, but an intended function of the Flag workspace.
 *
 * workspace: Flag (nrow).  Does not modify Flag if nrow is zero.
 */

UF_long cholmod_l_clear_flag
(
    cholmod_common *Common
)
{
    Int i, nrow, *Flag ;

    RETURN_IF_NULL_COMMON (-1) ;

    Common->mark++ ;
    if (Common->mark <= 0)
    {
	nrow = Common->nrow ;
	Flag = Common->Flag ;
	PRINT2 (("reset Flag: nrow "ID"\n", nrow)) ;
	PRINT2 (("reset Flag: mark %ld\n", Common->mark)) ;
	for (i = 0 ; i < nrow ; i++)
	{
	    Flag [i] = EMPTY ;
	}
	Common->mark = 0 ;
    }
    return (Common->mark) ;
}


/* ========================================================================== */
/* ==== cholmod_maxrank ===================================================== */
/* ========================================================================== */

/* Find a valid value of Common->maxrank.  Returns 0 if error, or 2, 4, or 8
 * if successful. */

size_t cholmod_l_maxrank	/* returns validated value of Common->maxrank */
(
    /* ---- input ---- */
    size_t n,		/* A and L will have n rows */
    /* --------------- */
    cholmod_common *Common
)
{
    size_t maxrank ;
    RETURN_IF_NULL_COMMON (0) ;
    maxrank = Common->maxrank ;
    if (n > 0)
    {
	/* Ensure maxrank*n*sizeof(double) does not result in integer overflow.
	 * If n is so large that 2*n*sizeof(double) results in integer overflow
	 * (n = 268,435,455 if an Int is 32 bits), then maxrank will be 0 or 1,
	 * but maxrank will be set to 2 below.  2*n will not result in integer
	 * overflow, and CHOLMOD will run out of memory or safely detect integer
	 * overflow elsewhere.
	 */
	maxrank = MIN (maxrank, Size_max / (n * sizeof (double))) ;
    }
    if (maxrank <= 2)
    {
	maxrank = 2 ;
    }
    else if (maxrank <= 4)
    {
	maxrank = 4 ;
    }
    else
    {
	maxrank = 8 ;
    }
    return (maxrank) ;
}


/* ========================================================================== */
/* === cholmod_dbound ======================================================= */
/* ========================================================================== */

/* Ensure the absolute value of a diagonal entry, D (j,j), is greater than
 * Common->dbound.  This routine is not meant for the user to call.  It is used
 * by the various LDL' factorization and update/downdate routines.  The
 * default value of Common->dbound is zero, and in that case this routine is not
 * called at all.  No change is made if D (j,j) is NaN.  CHOLMOD does not call
 * this routine if Common->dbound is NaN.
 */

double cholmod_l_dbound	/* returns modified diagonal entry of D */
(
    /* ---- input ---- */
    double dj,		/* diagonal entry of D, for LDL' factorization */
    /* --------------- */
    cholmod_common *Common
)
{
    double dbound ;
    RETURN_IF_NULL_COMMON (0) ;
    if (!IS_NAN (dj))
    {
	dbound = Common->dbound ;
	if (dj < 0)
	{
	    if (dj > -dbound)
	    {
		dj = -dbound ;
		Common->ndbounds_hit++ ;
		if (Common->status == CHOLMOD_OK)
		{
		    ERROR (CHOLMOD_DSMALL, "diagonal below threshold") ;
		}
	    }
	}
	else
	{
	    if (dj < dbound)
	    {
		dj = dbound ;
		Common->ndbounds_hit++ ;
		if (Common->status == CHOLMOD_OK)
		{
		    ERROR (CHOLMOD_DSMALL, "diagonal below threshold") ;
		}
	    }
	}
    }
    return (dj) ;
}
