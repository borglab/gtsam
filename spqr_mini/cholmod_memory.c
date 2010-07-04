/* ========================================================================== */
/* === Core/cholmod_memory ================================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Core Module.  Copyright (C) 2005-2006,
 * Univ. of Florida.  Author: Timothy A. Davis
 * The CHOLMOD/Core Module is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* Core memory management routines:
 *
 * Primary routines:
 * -----------------
 * cholmod_malloc		malloc wrapper
 * cholmod_free			free wrapper
 *
 * Secondary routines:
 * -------------------
 * cholmod_calloc		calloc wrapper
 * cholmod_realloc		realloc wrapper
 * cholmod_realloc_multiple	realloc wrapper for multiple objects
 *
 * The user may make use of these, just like malloc and free.  You can even
 * malloc an object and safely free it with cholmod_free, and visa versa
 * (except that the memory usage statistics will be corrupted).  These routines
 * do differ from malloc and free.  If cholmod_free is given a NULL pointer,
 * for example, it does nothing (unlike the ANSI free).  cholmod_realloc does
 * not return NULL if given a non-NULL pointer and a nonzero size, even if it
 * fails (it sets an error code in Common->status instead).
 *
 * CHOLMOD keeps track of the amount of memory it has allocated, and so the
 * cholmod_free routine includes as a parameter the size of the object being
 * freed.  This is only used for memory usage statistics, which are very useful
 * in finding memory leaks in your program.  If you, the user of CHOLMOD, pass
 * the wrong size, the only consequence is that the memory usage statistics
 * will be invalid.  This will causes assertions to fail if CHOLMOD is
 * compiled with debugging enabled, but otherwise it will cause no errors.
 *
 * The cholmod_free_* routines for each CHOLMOD object keep track of the size
 * of the blocks they free, so they do not require you to pass their sizes
 * as a parameter.
 *
 * If a block of size zero is requested, these routines allocate a block of
 * size one instead.
 */

#include "cholmod_internal.h"
#include "cholmod_core.h"

/* ========================================================================== */
/* === cholmod_add_size_t =================================================== */
/* ========================================================================== */

/* Safely compute a+b, and check for integer overflow.  If overflow occurs,
 * return 0 and set OK to FALSE.  Also return 0 if OK is FALSE on input. */

size_t CHOLMOD(add_size_t) (size_t a, size_t b, int *ok)
{
    size_t s = a + b ;
    (*ok) = (*ok) && (s >= a) ;
    return ((*ok) ? s : 0) ;
}

/* ========================================================================== */
/* === cholmod_mult_size_t ================================================== */
/* ========================================================================== */

/* Safely compute a*k, where k should be small, and check for integer overflow.
 * If overflow occurs, return 0 and set OK to FALSE.  Also return 0 if OK is
 * FALSE on input. */

size_t CHOLMOD(mult_size_t) (size_t a, size_t k, int *ok)
{
    size_t p = 0, s ;
    while (*ok)
    {
	if (k % 2)
	{
	    p = p + a ;
	    (*ok) = (*ok) && (p >= a) ;
	}
	k = k / 2 ;
	if (!k) return (p) ;
	s = a + a ;
	(*ok) = (*ok) && (s >= a) ;
	a = s ;
    }
    return (0) ;
}


/* ========================================================================== */
/* === cholmod_malloc ======================================================= */
/* ========================================================================== */

/* Wrapper around malloc routine.  Allocates space of size MAX(1,n)*size, where
 * size is normally a sizeof (...).
 *
 * This routine, cholmod_calloc, and cholmod_realloc do not set Common->status
 * to CHOLMOD_OK on success, so that a sequence of cholmod_malloc's, _calloc's,
 * or _realloc's can be used.  If any of them fails, the Common->status will
 * hold the most recent error status.
 *
 * Usage, for a pointer to int:
 *
 *	p = cholmod_malloc (n, sizeof (int), Common)
 *
 * Uses a pointer to the malloc routine (or its equivalent) defined in Common.
 */

void *CHOLMOD(malloc)	/* returns pointer to the newly malloc'd block */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* --------------- */
    cholmod_common *Common
)
{
    void *p ;
    size_t s ;
    int ok = TRUE ;

    RETURN_IF_NULL_COMMON (NULL) ;
    if (size == 0)
    {
	ERROR (CHOLMOD_INVALID, "sizeof(item) must be > 0")  ;
	p = NULL ;
    }
    else if (n >= (Size_max / size) || n >= Int_max)
    {
	/* object is too big to allocate without causing integer overflow */
	ERROR (CHOLMOD_TOO_LARGE, "problem too large") ;
	p = NULL ;
    }
    else
    {
	/* call malloc, or its equivalent */
	s = CHOLMOD(mult_size_t) (MAX (1,n), size, &ok) ;
	p = ok ? ((Common->malloc_memory) (s)) : NULL ;
	if (p == NULL)
	{
	    /* failure: out of memory */
	    ERROR (CHOLMOD_OUT_OF_MEMORY, "out of memory") ;
	}
	else
	{
	    /* success: increment the count of objects allocated */
	    Common->malloc_count++ ;
	    Common->memory_inuse += (n * size) ;
	    Common->memory_usage =
		MAX (Common->memory_usage, Common->memory_inuse) ;
	    PRINTM (("cholmod_malloc %p %d cnt: %d inuse %d\n",
		    p, n*size, Common->malloc_count, Common->memory_inuse)) ;
	}
    }
    return (p) ;
}


/* ========================================================================== */
/* === cholmod_free ========================================================= */
/* ========================================================================== */

/* Wrapper around free routine.  Returns NULL, which can be assigned to the
 * pointer being freed, as in:
 *
 *	p = cholmod_free (n, sizeof (int), p, Common) ;
 *
 * In CHOLMOD, the syntax:
 *
 *	cholmod_free (n, sizeof (int), p, Common) ;
 *
 * is used if p is a local pointer and the routine is returning shortly.
 * Uses a pointer to the free routine (or its equivalent) defined in Common.
 * Nothing is freed if the pointer is NULL.
 */

void *CHOLMOD(free)	/* always returns NULL */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* ---- in/out --- */
    void *p,		/* block of memory to free */
    /* --------------- */
    cholmod_common *Common
)
{
    RETURN_IF_NULL_COMMON (NULL) ;
    if (p != NULL)
    {
	/* only free the object if the pointer is not NULL */
	/* call free, or its equivalent */
	(Common->free_memory) (p) ;
	Common->malloc_count-- ;
	Common->memory_inuse -= (n * size) ;
	PRINTM (("cholmod_free   %p %d cnt: %d inuse %d\n",
		p, n*size, Common->malloc_count, Common->memory_inuse)) ;
	/* This assertion will fail if the user calls cholmod_malloc and
	 * cholmod_free with mismatched memory sizes.  It shouldn't fail
	 * otherwise. */
	ASSERT (IMPLIES (Common->malloc_count == 0, Common->memory_inuse == 0));
    }
    /* return NULL, and the caller should assign this to p.  This avoids
     * freeing the same pointer twice. */
    return (NULL) ;
}


/* ========================================================================== */
/* === cholmod_calloc ======================================================= */
/* ========================================================================== */

/* Wrapper around calloc routine.
 *
 * Uses a pointer to the calloc routine (or its equivalent) defined in Common.
 * This routine is identical to malloc, except that it zeros the newly allocated
 * block to zero.
 */

void *CHOLMOD(calloc)	/* returns pointer to the newly calloc'd block */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* --------------- */
    cholmod_common *Common
)
{
    void *p ;

    RETURN_IF_NULL_COMMON (NULL) ;
    if (size == 0)
    {
	ERROR (CHOLMOD_INVALID, "sizeof(item) must be > 0") ;
	p = NULL ;
    }
    else if (n >= (Size_max / size) || n >= Int_max)
    {
	/* object is too big to allocate without causing integer overflow */
	ERROR (CHOLMOD_TOO_LARGE, "problem too large") ;
	p = NULL ;
    }
    else
    {
	/* call calloc, or its equivalent */
	p = (Common->calloc_memory) (MAX (1,n), size) ;
	if (p == NULL)
	{
	    /* failure: out of memory */
	    ERROR (CHOLMOD_OUT_OF_MEMORY, "out of memory") ;
	}
	else
	{
	    /* success: increment the count of objects allocated */
	    Common->malloc_count++ ;
	    Common->memory_inuse += (n * size) ;
	    Common->memory_usage =
		MAX (Common->memory_usage, Common->memory_inuse) ;
	    PRINTM (("cholmod_malloc %p %d cnt: %d inuse %d\n",
		    p, n*size, Common->malloc_count, Common->memory_inuse)) ;
	}
    }
    return (p) ;
}


/* ========================================================================== */
/* === cholmod_realloc ====================================================== */
/* ========================================================================== */

/* Wrapper around realloc routine.  Given a pointer p to a block of size
 * (*n)*size memory, it changes the size of the block pointed to by p to be
 * MAX(1,nnew)*size in size.  It may return a pointer different than p.  This
 * should be used as (for a pointer to int):
 *
 *	p = cholmod_realloc (nnew, sizeof (int), p, *n, Common) ;
 *
 * If p is NULL, this is the same as p = cholmod_malloc (...).
 * A size of nnew=0 is treated as nnew=1.
 *
 * If the realloc fails, p is returned unchanged and Common->status is set
 * to CHOLMOD_OUT_OF_MEMORY.  If successful, Common->status is not modified,
 * and p is returned (possibly changed) and pointing to a large block of memory.
 *
 * Uses a pointer to the realloc routine (or its equivalent) defined in Common.
 */

void *CHOLMOD(realloc)	/* returns pointer to reallocated block */
(
    /* ---- input ---- */
    size_t nnew,	/* requested # of items in reallocated block */
    size_t size,	/* size of each item */
    /* ---- in/out --- */
    void *p,		/* block of memory to realloc */
    size_t *n,		/* current size on input, nnew on output if successful*/
    /* --------------- */
    cholmod_common *Common
)
{
    size_t nold = (*n) ;
    void *pnew ;
    size_t s ;
    int ok = TRUE ;

    RETURN_IF_NULL_COMMON (NULL) ;
    if (size == 0)
    {
	ERROR (CHOLMOD_INVALID, "sizeof(item) must be > 0") ;
	p = NULL ;
    }
    else if (p == NULL)
    {
	/* A fresh object is being allocated. */
	PRINT1 (("realloc fresh: %d %d\n", nnew, size)) ;
	p = CHOLMOD(malloc) (nnew, size, Common) ;
	*n = (p == NULL) ? 0 : nnew ;
    }
    else if (nold == nnew)
    {
	/* Nothing to do.  Do not change p or n. */
	PRINT1 (("realloc nothing: %d %d\n", nnew, size)) ;
    }
    else if (nnew >= (Size_max / size) || nnew >= Int_max)
    {
	/* failure: nnew is too big.  Do not change p or n. */
	ERROR (CHOLMOD_TOO_LARGE, "problem too large") ;
    }
    else
    {
	/* The object exists, and is changing to some other nonzero size. */
	/* call realloc, or its equivalent */
	PRINT1 (("realloc : %d to %d, %d\n", nold, nnew, size)) ;
	pnew = NULL ;

	s = CHOLMOD(mult_size_t) (MAX (1,nnew), size, &ok) ;
	pnew = ok ? ((Common->realloc_memory) (p, s)) : NULL ;

	if (pnew == NULL)
	{
	    /* Do not change p, since it still points to allocated memory */
	    if (nnew <= nold)
	    {
		/* The attempt to reduce the size of the block from n to
		 * nnew has failed.  The current block is not modified, so
		 * pretend to succeed, but do not change p.  Do change
		 * CHOLMOD's notion of the size of the block, however. */
		*n = nnew ;
		PRINTM (("nnew <= nold failed, pretend to succeed\n")) ;
		PRINTM (("cholmod_free %p %d cnt: %d inuse %d\n"
			 "cholmod_malloc %p %d cnt: %d inuse %d\n",
		    p, nold*size, Common->malloc_count-1,
				  Common->memory_inuse - nold*size,
		    p, nnew*size, Common->malloc_count,
				  Common->memory_inuse + (nnew-nold)*size)) ;
		Common->memory_inuse += ((nnew-nold) * size) ;
	    }
	    else
	    {
		/* Increasing the size of the block has failed.
		 * Do not change n. */
		ERROR (CHOLMOD_OUT_OF_MEMORY, "out of memory") ;
	    }
	}
	else
	{
	    /* success: return revised p and change the size of the block */
	    PRINTM (("cholmod_free %p %d cnt: %d inuse %d\n"
		     "cholmod_malloc %p %d cnt: %d inuse %d\n",
		p, nold*size,    Common->malloc_count-1,
				 Common->memory_inuse - nold*size,
		pnew, nnew*size, Common->malloc_count,
				 Common->memory_inuse + (nnew-nold)*size)) ;
	    p = pnew ;
	    *n = nnew ;
	    Common->memory_inuse += ((nnew-nold) * size) ;
	}
	Common->memory_usage = MAX (Common->memory_usage, Common->memory_inuse);
    }

    return (p) ;
}


/* ========================================================================== */
/* === cholmod_realloc_multiple ============================================= */
/* ========================================================================== */

/* reallocate multiple blocks of memory, all of the same size (up to two integer
 * and two real blocks).  Either reallocations all succeed, or all are returned
 * in the original size (they are freed if the original size is zero).  The nnew
 * blocks are of size 1 or more.
 */

int CHOLMOD(realloc_multiple)
(
    /* ---- input ---- */
    size_t nnew,	/* requested # of items in reallocated blocks */
    int nint,		/* number of int/UF_long blocks */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* ---- in/out --- */
    void **I,		/* int or UF_long block */
    void **J,		/* int or UF_long block */
    void **X,		/* complex or double block */
    void **Z,		/* zomplex case only: double block */
    size_t *nold_p,	/* current size of the I,J,X,Z blocks on input,
			 * nnew on output if successful */
    /* --------------- */
    cholmod_common *Common
)
{
    double *xx, *zz ;
    size_t i, j, x, z, nold ;

    RETURN_IF_NULL_COMMON (FALSE) ;

    if (xtype < CHOLMOD_PATTERN || xtype > CHOLMOD_ZOMPLEX)
    {
	ERROR (CHOLMOD_INVALID, "invalid xtype") ;
	return (FALSE) ;
    }

    nold = *nold_p ;

    if (nint < 1 && xtype == CHOLMOD_PATTERN)
    {
	/* nothing to do */
	return (TRUE) ;
    }

    i = nold ;
    j = nold ;
    x = nold ;
    z = nold ;

    if (nint > 0)
    {
	*I = CHOLMOD(realloc) (nnew, sizeof (Int), *I, &i, Common) ;
    }
    if (nint > 1)
    {
	*J = CHOLMOD(realloc) (nnew, sizeof (Int), *J, &j, Common) ;
    }

    switch (xtype)
    {
	case CHOLMOD_REAL:
	    *X = CHOLMOD(realloc) (nnew, sizeof (double), *X, &x, Common) ;
	    break ;

	case CHOLMOD_COMPLEX:
	    *X = CHOLMOD(realloc) (nnew, 2*sizeof (double), *X, &x, Common) ;
	    break ;

	case CHOLMOD_ZOMPLEX:
	    *X = CHOLMOD(realloc) (nnew, sizeof (double), *X, &x, Common) ;
	    *Z = CHOLMOD(realloc) (nnew, sizeof (double), *Z, &z, Common) ;
	    break ;
    }

    if (Common->status < CHOLMOD_OK)
    {
	/* one or more realloc's failed.  Resize all back down to nold. */

	if (nold == 0)
	{

	    if (nint > 0)
	    {
		*I = CHOLMOD(free) (i, sizeof (Int), *I, Common) ;
	    }
	    if (nint > 1)
	    {
		*J = CHOLMOD(free) (j, sizeof (Int), *J, Common) ;
	    }

	    switch (xtype)
	    {
		case CHOLMOD_REAL:
		    *X = CHOLMOD(free) (x, sizeof (double), *X, Common) ;
		    break ;

		case CHOLMOD_COMPLEX:
		    *X = CHOLMOD(free) (x, 2*sizeof (double), *X, Common) ;
		    break ;

		case CHOLMOD_ZOMPLEX:
		    *X = CHOLMOD(free) (x, sizeof (double), *X, Common) ;
		    *Z = CHOLMOD(free) (x, sizeof (double), *Z, Common) ;
		    break ;
	    }

	}
	else
	{
	    if (nint > 0)
	    {
		*I = CHOLMOD(realloc) (nold, sizeof (Int), *I, &i, Common) ;
	    }
	    if (nint > 1)
	    {
		*J = CHOLMOD(realloc) (nold, sizeof (Int), *J, &j, Common) ;
	    }

	    switch (xtype)
	    {
		case CHOLMOD_REAL:
		    *X = CHOLMOD(realloc) (nold, sizeof (double), *X, &x,
			    Common) ;
		    break ;

		case CHOLMOD_COMPLEX:
		    *X = CHOLMOD(realloc) (nold, 2*sizeof (double), *X, &x,
			    Common) ;
		    break ;

		case CHOLMOD_ZOMPLEX:
		    *X = CHOLMOD(realloc) (nold, sizeof (double), *X, &x,
			    Common) ;
		    *Z = CHOLMOD(realloc) (nold, sizeof (double), *Z, &z,
			    Common) ;
		    break ;
	    }

	}

	return (FALSE) ;
    }

    if (nold == 0)
    {
	/* New space was allocated.  Clear the first entry so that valgrind
	 * doesn't complain about its access in change_complexity
	 * (Core/cholmod_complex.c). */
	xx = *X ;
	zz = *Z ;
	switch (xtype)
	{
	    case CHOLMOD_REAL:
		xx [0] = 0 ;
		break ;

	    case CHOLMOD_COMPLEX:
		xx [0] = 0 ;
		xx [1] = 0 ;
		break ;

	    case CHOLMOD_ZOMPLEX:
		xx [0] = 0 ;
		zz [0] = 0 ;
		break ;
	}
    }

    /* all realloc's succeeded, change size to reflect realloc'ed size. */
    *nold_p = nnew ;
    return (TRUE) ;
}
