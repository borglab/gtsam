/* ========================================================================== */
/* === Include/cholmod_core.h =============================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Include/cholmod_core.h.
 * Copyright (C) 2005-2006, Univ. of Florida.  Author: Timothy A. Davis
 * CHOLMOD/Include/cholmod_core.h is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* CHOLMOD Core module: basic CHOLMOD objects and routines.
 * Required by all CHOLMOD modules.  Requires no other module or package.
 *
 * The CHOLMOD modules are:
 *
 * Core		basic data structures and definitions
 * Check	check/print the 5 CHOLMOD objects, & 3 types of integer vectors
 * Cholesky	sparse Cholesky factorization
 * Modify	sparse Cholesky update/downdate/row-add/row-delete
 * MatrixOps	sparse matrix functions (add, multiply, norm, ...)
 * Supernodal	supernodal sparse Cholesky factorization
 * Partition	graph-partitioning based orderings
 *
 * The CHOLMOD objects:
 * --------------------
 *
 * cholmod_common   parameters, statistics, and workspace
 * cholmod_sparse   a sparse matrix in compressed column form
 * cholmod_factor   an LL' or LDL' factorization
 * cholmod_dense    a dense matrix
 * cholmod_triplet  a sparse matrix in "triplet" form
 *
 * The Core module described here defines the CHOLMOD data structures, and
 * basic operations on them.  To create and solve a sparse linear system Ax=b,
 * the user must create A and b, populate them with values, and then pass them
 * to the routines in the CHOLMOD Cholesky module.  There are two primary
 * methods for creating A: (1) allocate space for a column-oriented sparse
 * matrix and fill it with pattern and values, or (2) create a triplet form
 * matrix and convert it to a sparse matrix.  The latter option is simpler.
 *
 * The matrices b and x are typically dense matrices, but can also be sparse.
 * You can allocate and free them as dense matrices with the
 * cholmod_allocate_dense and cholmod_free_dense routines.
 *
 * The cholmod_factor object contains the symbolic and numeric LL' or LDL'
 * factorization of sparse symmetric matrix.  The matrix must be positive
 * definite for an LL' factorization.  It need only be symmetric and have well-
 * conditioned leading submatrices for it to have an LDL' factorization
 * (CHOLMOD does not pivot for numerical stability).  It is typically created
 * with the cholmod_factorize routine in the Cholesky module, but can also
 * be initialized to L=D=I in the Core module and then modified by the Modify
 * module.  It must be freed with cholmod_free_factor, defined below.
 *
 * The Core routines for each object are described below.  Each list is split
 * into two parts: the primary routines and secondary routines.
 *
 * ============================================================================
 * === cholmod_common =========================================================
 * ============================================================================
 *
 * The Common object contains control parameters, statistics, and
 * You must call cholmod_start before calling any other CHOLMOD routine, and
 * must call cholmod_finish as your last call to CHOLMOD, with two exceptions:
 * you may call cholmod_print_common and cholmod_check_common in the Check
 * module after calling cholmod_finish.
 *
 * cholmod_start		first call to CHOLMOD
 * cholmod_finish		last call to CHOLMOD
 * -----------------------------
 * cholmod_defaults		restore default parameters
 * cholmod_maxrank		maximum rank for update/downdate
 * cholmod_allocate_work	allocate workspace in Common
 * cholmod_free_work		free workspace in Common
 * cholmod_clear_flag		clear Flag workspace in Common
 * cholmod_error		called when CHOLMOD encounters an error
 * cholmod_dbound		for internal use in CHOLMOD only
 * cholmod_hypot		compute sqrt (x*x + y*y) accurately
 * cholmod_divcomplex		complex division, c = a/b
 *
 * ============================================================================
 * === cholmod_sparse =========================================================
 * ============================================================================
 *
 * A sparse matrix is held in compressed column form.  In the basic type
 * ("packed", which corresponds to a MATLAB sparse matrix), an n-by-n matrix
 * with nz entries is held in three arrays: p of size n+1, i of size nz, and x
 * of size nz.  Row indices of column j are held in i [p [j] ... p [j+1]-1] and
 * in the same locations in x.  There may be no duplicate entries in a column.
 * Row indices in each column may be sorted or unsorted (CHOLMOD keeps track).
 * A->stype determines the storage mode: 0 if both upper/lower parts are stored,
 * -1 if A is symmetric and just tril(A) is stored, +1 if symmetric and triu(A)
 * is stored.
 *
 * cholmod_allocate_sparse	allocate a sparse matrix
 * cholmod_free_sparse		free a sparse matrix
 * -----------------------------
 * cholmod_reallocate_sparse	change the size (# entries) of sparse matrix
 * cholmod_nnz			number of nonzeros in a sparse matrix
 * cholmod_speye		sparse identity matrix
 * cholmod_spzeros		sparse zero matrix
 * cholmod_transpose		transpose a sparse matrix
 * cholmod_ptranspose		transpose/permute a sparse matrix
 * cholmod_transpose_unsym	transpose/permute an unsymmetric sparse matrix
 * cholmod_transpose_sym	transpose/permute a symmetric sparse matrix
 * cholmod_sort			sort row indices in each column of sparse matrix
 * cholmod_band			C = tril (triu (A,k1), k2)
 * cholmod_band_inplace		A = tril (triu (A,k1), k2)
 * cholmod_aat			C = A*A'
 * cholmod_copy_sparse		C = A, create an exact copy of a sparse matrix
 * cholmod_copy			C = A, with possible change of stype
 * cholmod_add			C = alpha*A + beta*B
 * cholmod_sparse_xtype		change the xtype of a sparse matrix
 *
 * ============================================================================
 * === cholmod_factor =========================================================
 * ============================================================================
 *
 * The data structure for an LL' or LDL' factorization is too complex to
 * describe in one sentence.  This object can hold the symbolic analysis alone,
 * or in combination with a "simplicial" (similar to a sparse matrix) or
 * "supernodal" form of the numerical factorization.  Only the routine to free
 * a factor is primary, since a factor object is created by the factorization
 * routine (cholmod_factorize).  It must be freed with cholmod_free_factor.
 *
 * cholmod_free_factor		free a factor
 * -----------------------------
 * cholmod_allocate_factor	allocate a factor (LL' or LDL')
 * cholmod_reallocate_factor	change the # entries in a factor
 * cholmod_change_factor	change the type of factor (e.g., LDL' to LL')
 * cholmod_pack_factor		pack the columns of a factor
 * cholmod_reallocate_column	resize a single column of a factor
 * cholmod_factor_to_sparse	create a sparse matrix copy of a factor
 * cholmod_copy_factor		create a copy of a factor
 * cholmod_factor_xtype		change the xtype of a factor
 *
 * Note that there is no cholmod_sparse_to_factor routine to create a factor
 * as a copy of a sparse matrix.  It could be done, after a fashion, but a
 * lower triangular sparse matrix would not necessarily have a chordal graph,
 * which would break the many CHOLMOD routines that rely on this property.
 *
 * ============================================================================
 * === cholmod_dense ==========================================================
 * ============================================================================
 *
 * The solve routines and some of the MatrixOps and Modify routines use dense
 * matrices as inputs.  These are held in column-major order.  With a leading
 * dimension of d, the entry in row i and column j is held in x [i+j*d].
 *
 * cholmod_allocate_dense	allocate a dense matrix
 * cholmod_free_dense		free a dense matrix
 * -----------------------------
 * cholmod_zeros		allocate a dense matrix of all zeros
 * cholmod_ones			allocate a dense matrix of all ones
 * cholmod_eye			allocate a dense identity matrix
 * cholmod_sparse_to_dense	create a dense matrix copy of a sparse matrix
 * cholmod_dense_to_sparse	create a sparse matrix copy of a dense matrix
 * cholmod_copy_dense		create a copy of a dense matrix
 * cholmod_copy_dense2		copy a dense matrix (pre-allocated)
 * cholmod_dense_xtype		change the xtype of a dense matrix
 *
 * ============================================================================
 * === cholmod_triplet ========================================================
 * ============================================================================
 *
 * A sparse matrix held in triplet form is the simplest one for a user to
 * create.  It consists of a list of nz entries in arbitrary order, held in
 * three arrays: i, j, and x, each of length nk.  The kth entry is in row i[k],
 * column j[k], with value x[k].  There may be duplicate values; if A(i,j)
 * appears more than once, its value is the sum of the entries with those row
 * and column indices.
 *
 * cholmod_allocate_triplet	allocate a triplet matrix
 * cholmod_triplet_to_sparse	create a sparse matrix copy of a triplet matrix
 * cholmod_free_triplet		free a triplet matrix
 * -----------------------------
 * cholmod_reallocate_triplet	change the # of entries in a triplet matrix
 * cholmod_sparse_to_triplet	create a triplet matrix copy of a sparse matrix
 * cholmod_copy_triplet		create a copy of a triplet matrix
 * cholmod_triplet_xtype	change the xtype of a triplet matrix
 *
 * ============================================================================
 * === memory management ======================================================
 * ============================================================================
 *
 * cholmod_malloc		malloc wrapper
 * cholmod_calloc		calloc wrapper
 * cholmod_free			free wrapper
 * cholmod_realloc		realloc wrapper
 * cholmod_realloc_multiple	realloc wrapper for multiple objects
 *
 * ============================================================================
 * === Core CHOLMOD prototypes ================================================
 * ============================================================================
 *
 * All CHOLMOD routines (in all modules) use the following protocol for return
 * values, with one exception:
 *
 * int			TRUE (1) if successful, or FALSE (0) otherwise.
 *			(exception: cholmod_divcomplex)
 * UF_long		a value >= 0 if successful, or -1 otherwise.
 * double		a value >= 0 if successful, or -1 otherwise.
 * size_t		a value > 0 if successful, or 0 otherwise.
 * void *		a non-NULL pointer to newly allocated memory if
 *			successful, or NULL otherwise.
 * cholmod_sparse *	a non-NULL pointer to a newly allocated matrix
 *			if successful, or NULL otherwise.
 * cholmod_factor *	a non-NULL pointer to a newly allocated factor
 *			if successful, or NULL otherwise.
 * cholmod_triplet *	a non-NULL pointer to a newly allocated triplet
 *			matrix if successful, or NULL otherwise.
 * cholmod_dense *	a non-NULL pointer to a newly allocated triplet
 *			matrix if successful, or NULL otherwise.
 *
 * The last parameter to all routines is always a pointer to the CHOLMOD
 * Common object.
 *
 * TRUE and FALSE are not defined here, since they may conflict with the user
 * program.  A routine that described here returning TRUE or FALSE returns 1
 * or 0, respectively.  Any TRUE/FALSE parameter is true if nonzero, false if
 * zero.
 */

#ifndef CHOLMOD_CORE_H
#define CHOLMOD_CORE_H

/* ========================================================================== */
/* === CHOLMOD version ====================================================== */
/* ========================================================================== */

/* All versions of CHOLMOD will include the following definitions.
 * As an example, to test if the version you are using is 1.3 or later:
 *
 *	if (CHOLMOD_VERSION >= CHOLMOD_VER_CODE (1,3)) ...
 *
 * This also works during compile-time:
 *
 *	#if CHOLMOD_VERSION >= CHOLMOD_VER_CODE (1,3)
 *	    printf ("This is version 1.3 or later\n") ;
 *	#else
 *	    printf ("This is version is earlier than 1.3\n") ;
 *	#endif
 */

#define CHOLMOD_DATE "Nov 30, 2009"
#define CHOLMOD_VER_CODE(main,sub) ((main) * 1000 + (sub))
#define CHOLMOD_MAIN_VERSION 1
#define CHOLMOD_SUB_VERSION 7
#define CHOLMOD_SUBSUB_VERSION 2
#define CHOLMOD_VERSION \
    CHOLMOD_VER_CODE(CHOLMOD_MAIN_VERSION,CHOLMOD_SUB_VERSION)


/* ========================================================================== */
/* === non-CHOLMOD include files ============================================ */
/* ========================================================================== */

/* This is the only non-CHOLMOD include file imposed on the user program.
 * It required for size_t definition used here.  CHOLMOD itself includes other
 * ANSI C89 standard #include files, but does not expose them to the user.
 *
 * CHOLMOD assumes that your C compiler is ANSI C89 compliant.  It does not make
 * use of ANSI C99 features.
 */

#include <stddef.h>
#include <stdlib.h>

/* ========================================================================== */
/* === CHOLMOD objects ====================================================== */
/* ========================================================================== */

/* Each CHOLMOD object has its own type code. */

#define CHOLMOD_COMMON 0
#define CHOLMOD_SPARSE 1
#define CHOLMOD_FACTOR 2
#define CHOLMOD_DENSE 3
#define CHOLMOD_TRIPLET 4

/* ========================================================================== */
/* === CHOLMOD Common ======================================================= */
/* ========================================================================== */

/* itype defines the types of integer used: */
#define CHOLMOD_INT 0		/* all integer arrays are int */
#define CHOLMOD_INTLONG 1	/* most are int, some are UF_long */
#define CHOLMOD_LONG 2		/* all integer arrays are UF_long */

/* The itype of all parameters for all CHOLMOD routines must match.
 * FUTURE WORK: CHOLMOD_INTLONG is not yet supported.
 */

/* dtype defines what the numerical type is (double or float): */
#define CHOLMOD_DOUBLE 0	/* all numerical values are double */
#define CHOLMOD_SINGLE 1	/* all numerical values are float */

/* The dtype of all parameters for all CHOLMOD routines must match.
 *
 * Scalar floating-point values are always passed as double arrays of size 2
 * (for the real and imaginary parts).  They are typecast to float as needed.
 * FUTURE WORK: the float case is not supported yet.
 */

/* xtype defines the kind of numerical values used: */
#define CHOLMOD_PATTERN 0	/* pattern only, no numerical values */
#define CHOLMOD_REAL 1		/* a real matrix */
#define CHOLMOD_COMPLEX 2	/* a complex matrix (ANSI C99 compatible) */
#define CHOLMOD_ZOMPLEX 3	/* a complex matrix (MATLAB compatible) */

/* The xtype of all parameters for all CHOLMOD routines must match.
 *
 * CHOLMOD_PATTERN: x and z are ignored.
 * CHOLMOD_DOUBLE:  x is non-null of size nzmax, z is ignored.
 * CHOLMOD_COMPLEX: x is non-null of size 2*nzmax doubles, z is ignored.
 * CHOLMOD_ZOMPLEX: x and z are non-null of size nzmax
 *
 * In the real case, z is ignored.  The kth entry in the matrix is x [k].
 * There are two methods for the complex case.  In the ANSI C99-compatible
 * CHOLMOD_COMPLEX case, the real and imaginary parts of the kth entry
 * are in x [2*k] and x [2*k+1], respectively.  z is ignored.  In the
 * MATLAB-compatible CHOLMOD_ZOMPLEX case, the real and imaginary
 * parts of the kth entry are in x [k] and z [k].
 *
 * Scalar floating-point values are always passed as double arrays of size 2
 * (real and imaginary parts).  The imaginary part of a scalar is ignored if
 * the routine operates on a real matrix.
 *
 * These Modules support complex and zomplex matrices, with a few exceptions:
 *
 *	Check	    all routines
 *	Cholesky    all routines
 *	Core	    all except cholmod_aat, add, band, copy
 *	Demo	    all routines
 *	Partition   all routines
 *	Supernodal  all routines support any real, complex, or zomplex input.
 *			There will never be a supernodal zomplex L; a complex
 *			supernodal L is created if A is zomplex.
 *	Tcov	    all routines
 *	Valgrind    all routines
 *
 * These Modules provide partial support for complex and zomplex matrices:
 *
 *	MATLAB	    all routines support real and zomplex only, not complex,
 *			with the exception of ldlupdate, which supports
 *			real matrices only.  This is a minor constraint since
 *			MATLAB's matrices are all real or zomplex.
 *	MatrixOps   only norm_dense, norm_sparse, and sdmult support complex
 *			and zomplex
 *
 * These Modules do not support complex and zomplex matrices at all:
 *
 *	Modify	    all routines support real matrices only
 */

/* Definitions for cholmod_common: */
#define CHOLMOD_MAXMETHODS 9	/* maximum number of different methods that */
				/* cholmod_analyze can try. Must be >= 9. */

/* Common->status values.  zero means success, negative means a fatal error,
 * positive is a warning. */
#define CHOLMOD_OK 0			/* success */
#define CHOLMOD_NOT_INSTALLED (-1)	/* failure: method not installed */
#define CHOLMOD_OUT_OF_MEMORY (-2)	/* failure: out of memory */
#define CHOLMOD_TOO_LARGE (-3)		/* failure: integer overflow occured */
#define CHOLMOD_INVALID (-4)		/* failure: invalid input */
#define CHOLMOD_NOT_POSDEF (1)		/* warning: matrix not pos. def. */
#define CHOLMOD_DSMALL (2)		/* warning: D for LDL'  or diag(L) or */
					/* LL' has tiny absolute value */

/* ordering method (also used for L->ordering) */
#define CHOLMOD_NATURAL 0	/* use natural ordering */
#define CHOLMOD_GIVEN 1		/* use given permutation */
#define CHOLMOD_AMD 2		/* use minimum degree (AMD) */
#define CHOLMOD_METIS 3		/* use METIS' nested dissection */
#define CHOLMOD_NESDIS 4	/* use CHOLMOD's version of nested dissection:*/
				/* node bisector applied recursively, followed
				 * by constrained minimum degree (CSYMAMD or
				 * CCOLAMD) */
#define CHOLMOD_COLAMD 5	/* use AMD for A, COLAMD for A*A' */

/* POSTORDERED is not a method, but a result of natural ordering followed by a
 * weighted postorder.  It is used for L->ordering, not method [ ].ordering. */
#define CHOLMOD_POSTORDERED 6	/* natural ordering, postordered. */

/* supernodal strategy (for Common->supernodal) */
#define CHOLMOD_SIMPLICIAL 0	/* always do simplicial */
#define CHOLMOD_AUTO 1		/* select simpl/super depending on matrix */
#define CHOLMOD_SUPERNODAL 2	/* always do supernodal */

typedef struct cholmod_common_struct
{
    /* ---------------------------------------------------------------------- */
    /* parameters for symbolic/numeric factorization and update/downdate */
    /* ---------------------------------------------------------------------- */

    double dbound ;	/* Smallest absolute value of diagonal entries of D
			 * for LDL' factorization and update/downdate/rowadd/
	* rowdel, or the diagonal of L for an LL' factorization.
	* Entries in the range 0 to dbound are replaced with dbound.
	* Entries in the range -dbound to 0 are replaced with -dbound.  No
	* changes are made to the diagonal if dbound <= 0.  Default: zero */

    double grow0 ;	/* For a simplicial factorization, L->i and L->x can
			 * grow if necessary.  grow0 is the factor by which
	* it grows.  For the initial space, L is of size MAX (1,grow0) times
	* the required space.  If L runs out of space, the new size of L is
	* MAX(1.2,grow0) times the new required space.   If you do not plan on
	* modifying the LDL' factorization in the Modify module, set grow0 to
	* zero (or set grow2 to 0, see below).  Default: 1.2 */

    double grow1 ;

    size_t grow2 ;	/* For a simplicial factorization, each column j of L
			 * is initialized with space equal to
	* grow1*L->ColCount[j] + grow2.  If grow0 < 1, grow1 < 1, or grow2 == 0,
	* then the space allocated is exactly equal to L->ColCount[j].  If the
	* column j runs out of space, it increases to grow1*need + grow2 in
	* size, where need is the total # of nonzeros in that column.  If you do
	* not plan on modifying the factorization in the Modify module, set
	* grow2 to zero.  Default: grow1 = 1.2, grow2 = 5. */

    size_t maxrank ;	/* rank of maximum update/downdate.  Valid values:
			 * 2, 4, or 8.  A value < 2 is set to 2, and a
	* value > 8 is set to 8.  It is then rounded up to the next highest
	* power of 2, if not already a power of 2.  Workspace (Xwork, below) of
	* size nrow-by-maxrank double's is allocated for the update/downdate.
	* If an update/downdate of rank-k is requested, with k > maxrank,
	* it is done in steps of maxrank.  Default: 8, which is fastest.
	* Memory usage can be reduced by setting maxrank to 2 or 4.
	*/

    double supernodal_switch ;	/* supernodal vs simplicial factorization */
    int supernodal ;		/* If Common->supernodal <= CHOLMOD_SIMPLICIAL
				 * (0) then cholmod_analyze performs a
	* simplicial analysis.  If >= CHOLMOD_SUPERNODAL (2), then a supernodal
	* analysis is performed.  If == CHOLMOD_AUTO (1) and
	* flop/nnz(L) < Common->supernodal_switch, then a simplicial analysis
	* is done.  A supernodal analysis done otherwise.
	* Default:  CHOLMOD_AUTO.  Default supernodal_switch = 40 */

    int final_asis ;	/* If TRUE, then ignore the other final_* parameters
			 * (except for final_pack).
			 * The factor is left as-is when done.  Default: TRUE.*/

    int final_super ;	/* If TRUE, leave a factor in supernodal form when
			 * supernodal factorization is finished.  If FALSE,
			 * then convert to a simplicial factor when done.
			 * Default: TRUE */

    int final_ll ;	/* If TRUE, leave factor in LL' form when done.
			 * Otherwise, leave in LDL' form.  Default: FALSE */

    int final_pack ;	/* If TRUE, pack the columns when done.  If TRUE, and
			 * cholmod_factorize is called with a symbolic L, L is
	* allocated with exactly the space required, using L->ColCount.  If you
	* plan on modifying the factorization, set Common->final_pack to FALSE,
	* and each column will be given a little extra slack space for future
	* growth in fill-in due to updates.  Default: TRUE */

    int final_monotonic ;   /* If TRUE, ensure columns are monotonic when done.
			 * Default: TRUE */

    int final_resymbol ;/* if cholmod_factorize performed a supernodal
			 * factorization, final_resymbol is true, and
	* final_super is FALSE (convert a simplicial numeric factorization),
	* then numerically zero entries that resulted from relaxed supernodal
	* amalgamation are removed.  This does not remove entries that are zero
	* due to exact numeric cancellation, since doing so would break the
	* update/downdate rowadd/rowdel routines.  Default: FALSE. */

    /* supernodal relaxed amalgamation parameters: */
    double zrelax [3] ;
    size_t nrelax [3] ;

	/* Let ns be the total number of columns in two adjacent supernodes.
	 * Let z be the fraction of zero entries in the two supernodes if they
	 * are merged (z includes zero entries from prior amalgamations).  The
	 * two supernodes are merged if:
	 *    (ns <= nrelax [0]) || (no new zero entries added) ||
	 *    (ns <= nrelax [1] && z < zrelax [0]) ||
	 *    (ns <= nrelax [2] && z < zrelax [1]) || (z < zrelax [2])
	 *
	 * Default parameters result in the following rule:
	 *    (ns <= 4) || (no new zero entries added) ||
	 *    (ns <= 16 && z < 0.8) || (ns <= 48 && z < 0.1) || (z < 0.05)
	 */

    int prefer_zomplex ;    /* X = cholmod_solve (sys, L, B, Common) computes
			     * x=A\b or solves a related system.  If L and B are
	 * both real, then X is real.  Otherwise, X is returned as
	 * CHOLMOD_COMPLEX if Common->prefer_zomplex is FALSE, or
	 * CHOLMOD_ZOMPLEX if Common->prefer_zomplex is TRUE.  This parameter
	 * is needed because there is no supernodal zomplex L.  Suppose the
	 * caller wants all complex matrices to be stored in zomplex form
	 * (MATLAB, for example).  A supernodal L is returned in complex form
	 * if A is zomplex.  B can be real, and thus X = cholmod_solve (L,B)
	 * should return X as zomplex.  This cannot be inferred from the input
	 * arguments L and B.  Default: FALSE, since all data types are
	 * supported in CHOLMOD_COMPLEX form and since this is the native type
	 * of LAPACK and the BLAS.  Note that the MATLAB/cholmod.c mexFunction
	 * sets this parameter to TRUE, since MATLAB matrices are in
	 * CHOLMOD_ZOMPLEX form.
	 */

    int prefer_upper ;	    /* cholmod_analyze and cholmod_factorize work
			     * fastest when a symmetric matrix is stored in
	 * upper triangular form when a fill-reducing ordering is used.  In
	 * MATLAB, this corresponds to how x=A\b works.  When the matrix is
	 * ordered as-is, they work fastest when a symmetric matrix is in lower
	 * triangular form.  In MATLAB, R=chol(A) does the opposite.  This
	 * parameter affects only how cholmod_read returns a symmetric matrix.
	 * If TRUE (the default case), a symmetric matrix is always returned in
	 * upper-triangular form (A->stype = 1).  */

    int quick_return_if_not_posdef ;	/* if TRUE, the supernodal numeric
					 * factorization will return quickly if
	* the matrix is not positive definite.  Default: FALSE. */

    /* ---------------------------------------------------------------------- */
    /* printing and error handling options */
    /* ---------------------------------------------------------------------- */

    int print ;		/* print level. Default: 3 */
    int precise ;	/* if TRUE, print 16 digits.  Otherwise print 5 */
    int (*print_function) (const char *, ...) ;	/* pointer to printf */

    int try_catch ;	/* if TRUE, then ignore errors; CHOLMOD is in the middle
			 * of a try/catch block.  No error message is printed
	 * and the Common->error_handler function is not called. */

    void (*error_handler) (int status, const char *file,
        int line, const char *message) ;

	/* Common->error_handler is the user's error handling routine.  If not
	 * NULL, this routine is called if an error occurs in CHOLMOD.  status
	 * can be CHOLMOD_OK (0), negative for a fatal error, and positive for
	 * a warning. file is a string containing the name of the source code
	 * file where the error occured, and line is the line number in that
	 * file.  message is a string describing the error in more detail. */

    /* ---------------------------------------------------------------------- */
    /* ordering options */
    /* ---------------------------------------------------------------------- */

    /* The cholmod_analyze routine can try many different orderings and select
     * the best one.  It can also try one ordering method multiple times, with
     * different parameter settings.  The default is to use three orderings,
     * the user's permutation (if provided), AMD which is the fastest ordering
     * and generally gives good fill-in, and METIS.  CHOLMOD's nested dissection
     * (METIS with a constrained AMD) usually gives a better ordering than METIS
     * alone (by about 5% to 10%) but it takes more time.
     *
     * If you know the method that is best for your matrix, set Common->nmethods
     * to 1 and set Common->method [0] to the set of parameters for that method.
     * If you set it to 1 and do not provide a permutation, then only AMD will
     * be called.
     *
     * If METIS is not available, the default # of methods tried is 2 (the user
     * permutation, if any, and AMD).
     *
     * To try other methods, set Common->nmethods to the number of methods you
     * want to try.  The suite of default methods and their parameters is
     * described in the cholmod_defaults routine, and summarized here:
     *
     *	    Common->method [i]:
     *	    i = 0: user-provided ordering (cholmod_analyze_p only)
     *	    i = 1: AMD (for both A and A*A')
     *	    i = 2: METIS
     *	    i = 3: CHOLMOD's nested dissection (NESDIS), default parameters
     *	    i = 4: natural
     *	    i = 5: NESDIS with nd_small = 20000
     *	    i = 6: NESDIS with nd_small = 4, no constrained minimum degree
     *	    i = 7: NESDIS with no dense node removal
     *	    i = 8: AMD for A, COLAMD for A*A'
     *
     * You can modify the suite of methods you wish to try by modifying
     * Common.method [...] after calling cholmod_start or cholmod_defaults.
     *
     * For example, to use AMD, followed by a weighted postordering:
     *
     *	    Common->nmethods = 1 ;
     *	    Common->method [0].ordering = CHOLMOD_AMD ;
     *	    Common->postorder = TRUE ;
     *
     * To use the natural ordering (with no postordering):
     *
     *	    Common->nmethods = 1 ;
     *	    Common->method [0].ordering = CHOLMOD_NATURAL ;
     *	    Common->postorder = FALSE ;
     *
     * If you are going to factorize hundreds or more matrices with the same
     * nonzero pattern, you may wish to spend a great deal of time finding a
     * good permutation.  In this case, try setting Common->nmethods to 9.
     * The time spent in cholmod_analysis will be very high, but you need to
     * call it only once.
     *
     * cholmod_analyze sets Common->current to a value between 0 and nmethods-1.
     * Each ordering method uses the set of options defined by this parameter.
     */

    int nmethods ;	/* The number of ordering methods to try.  Default: 0.
			 * nmethods = 0 is a special case.  cholmod_analyze
	* will try the user-provided ordering (if given) and AMD.  Let fl and
	* lnz be the flop count and nonzeros in L from AMD's ordering.  Let
	* anz be the number of nonzeros in the upper or lower triangular part
	* of the symmetric matrix A.  If fl/lnz < 500 or lnz/anz < 5, then this
	* is a good ordering, and METIS is not attempted.  Otherwise, METIS is
	* tried.   The best ordering found is used.  If nmethods > 0, the
	* methods used are given in the method[ ] array, below.  The first
	* three methods in the default suite of orderings is (1) use the given
	* permutation (if provided), (2) use AMD, and (3) use METIS.  Maximum
	* allowed value is CHOLMOD_MAXMETHODS.  */

    int current ;	/* The current method being tried.  Default: 0.  Valid
			 * range is 0 to nmethods-1. */

    int selected ;	/* The best method found. */

    /* The suite of ordering methods and parameters: */

    struct cholmod_method_struct
    {
	/* statistics for this method */
	double lnz ;	    /* nnz(L) excl. zeros from supernodal amalgamation,
			     * for a "pure" L */

	double fl ;	    /* flop count for a "pure", real simplicial LL'
			     * factorization, with no extra work due to
	    * amalgamation.  Subtract n to get the LDL' flop count.   Multiply
	    * by about 4 if the matrix is complex or zomplex. */

	/* ordering method parameters */
	double prune_dense ;/* dense row/col control for AMD, SYMAMD, CSYMAMD,
			     * and NESDIS (cholmod_nested_dissection).  For a
	    * symmetric n-by-n matrix, rows/columns with more than
	    * MAX (16, prune_dense * sqrt (n)) entries are removed prior to
	    * ordering.  They appear at the end of the re-ordered matrix.
	    *
	    * If prune_dense < 0, only completely dense rows/cols are removed.
	    *
	    * This paramater is also the dense column control for COLAMD and
	    * CCOLAMD.  For an m-by-n matrix, columns with more than
	    * MAX (16, prune_dense * sqrt (MIN (m,n))) entries are removed prior
	    * to ordering.  They appear at the end of the re-ordered matrix.
	    * CHOLMOD factorizes A*A', so it calls COLAMD and CCOLAMD with A',
	    * not A.  Thus, this parameter affects the dense *row* control for
	    * CHOLMOD's matrix, and the dense *column* control for COLAMD and
	    * CCOLAMD.
	    *
	    * Removing dense rows and columns improves the run-time of the
	    * ordering methods.  It has some impact on ordering quality
	    * (usually minimal, sometimes good, sometimes bad).
	    *
	    * Default: 10. */

	double prune_dense2 ;/* dense row control for COLAMD and CCOLAMD.
			    *  Rows with more than MAX (16, dense2 * sqrt (n))
	    * for an m-by-n matrix are removed prior to ordering.  CHOLMOD's
	    * matrix is transposed before ordering it with COLAMD or CCOLAMD,
	    * so this controls the dense *columns* of CHOLMOD's matrix, and
	    * the dense *rows* of COLAMD's or CCOLAMD's matrix.
	    *
	    * If prune_dense2 < 0, only completely dense rows/cols are removed.
	    *
	    * Default: -1.  Note that this is not the default for COLAMD and
	    * CCOLAMD.  -1 is best for Cholesky.  10 is best for LU.  */

	double nd_oksep ;   /* in NESDIS, when a node separator is computed, it
			     * discarded if nsep >= nd_oksep*n, where nsep is
	    * the number of nodes in the separator, and n is the size of the
	    * graph being cut.  Valid range is 0 to 1.  If 1 or greater, the
	    * separator is discarded if it consists of the entire graph.
	    * Default: 1 */

	double other1 [4] ; /* future expansion */

	size_t nd_small ;    /* do not partition graphs with fewer nodes than
			     * nd_small, in NESDIS.  Default: 200 (same as
			     * METIS) */

	size_t other2 [4] ; /* future expansion */

	int aggressive ;    /* Aggresive absorption in AMD, COLAMD, SYMAMD,
			     * CCOLAMD, and CSYMAMD.  Default: TRUE */

	int order_for_lu ;  /* CCOLAMD can be optimized to produce an ordering
			     * for LU or Cholesky factorization.  CHOLMOD only
	    * performs a Cholesky factorization.  However, you may wish to use
	    * CHOLMOD as an interface for CCOLAMD but use it for your own LU
	    * factorization.  In this case, order_for_lu should be set to FALSE.
	    * When factorizing in CHOLMOD itself, you should *** NEVER *** set
	    * this parameter FALSE.  Default: TRUE. */

	int nd_compress ;   /* If TRUE, compress the graph and subgraphs before
			     * partitioning them in NESDIS.  Default: TRUE */

	int nd_camd ;	    /* If 1, follow the nested dissection ordering
			     * with a constrained minimum degree ordering that
	    * respects the partitioning just found (using CAMD).  If 2, use
	    * CSYMAMD instead.  If you set nd_small very small, you may not need
	    * this ordering, and can save time by setting it to zero (no
	    * constrained minimum degree ordering).  Default: 1. */

	int nd_components ; /* The nested dissection ordering finds a node
			     * separator that splits the graph into two parts,
	    * which may be unconnected.  If nd_components is TRUE, each of
	    * these connected components is split independently.  If FALSE,
	    * each part is split as a whole, even if it consists of more than
	    * one connected component.  Default: FALSE */

	/* fill-reducing ordering to use */
	int ordering ;

	size_t other3 [4] ; /* future expansion */

    } method [CHOLMOD_MAXMETHODS + 1] ;

    int postorder ;	/* If TRUE, cholmod_analyze follows the ordering with a
			 * weighted postorder of the elimination tree.  Improves
	* supernode amalgamation.  Does not affect fundamental nnz(L) and
	* flop count.  Default: TRUE. */

    /* ---------------------------------------------------------------------- */
    /* memory management routines */
    /* ---------------------------------------------------------------------- */

    void *(*malloc_memory) (size_t) ;		/* pointer to malloc */
    void *(*realloc_memory) (void *, size_t) ;  /* pointer to realloc */
    void (*free_memory) (void *) ;		/* pointer to free */
    void *(*calloc_memory) (size_t, size_t) ;	/* pointer to calloc */

    /* ---------------------------------------------------------------------- */
    /* routines for complex arithmetic */
    /* ---------------------------------------------------------------------- */

    int (*complex_divide) (double ax, double az, double bx, double bz,
	    double *cx, double *cz) ;

	/* flag = complex_divide (ax, az, bx, bz, &cx, &cz) computes the complex
	 * division c = a/b, where ax and az hold the real and imaginary part
	 * of a, and b and c are stored similarly.  flag is returned as 1 if
	 * a divide-by-zero occurs, or 0 otherwise.  By default, the function
	 * pointer Common->complex_divide is set equal to cholmod_divcomplex.
	 */

    double (*hypotenuse) (double x, double y) ;

	/* s = hypotenuse (x,y) computes s = sqrt (x*x + y*y), but does so more
	 * accurately.  By default, the function pointer Common->hypotenuse is
	 * set equal to cholmod_hypot.  See also the hypot function in the C99
	 * standard, which has an identical syntax and function.  If you have
	 * a C99-compliant compiler, you can set Common->hypotenuse = hypot.  */

    /* ---------------------------------------------------------------------- */
    /* METIS workarounds */
    /* ---------------------------------------------------------------------- */

    double metis_memory ;   /* This is a parameter for CHOLMOD's interface to
			     * METIS, not a parameter to METIS itself.  METIS
	* uses an amount of memory that is difficult to estimate precisely
	* beforehand.  If it runs out of memory, it terminates your program.
	* All routines in CHOLMOD except for CHOLMOD's interface to METIS
	* return an error status and safely return to your program if they run
	* out of memory.  To mitigate this problem, the CHOLMOD interface
	* can allocate a single block of memory equal in size to an empirical
	* upper bound of METIS's memory usage times the Common->metis_memory
	* parameter, and then immediately free it.  It then calls METIS.  If
	* this pre-allocation fails, it is possible that METIS will fail as
	* well, and so CHOLMOD returns with an out-of-memory condition without
	* calling METIS.
	*
	* METIS_NodeND (used in the CHOLMOD_METIS ordering option) with its
	* default parameter settings typically uses about (4*nz+40n+4096)
	* times sizeof(int) memory, where nz is equal to the number of entries
	* in A for the symmetric case or AA' if an unsymmetric matrix is
	* being ordered (where nz includes both the upper and lower parts
	* of A or AA').  The observed "upper bound" (with 2 exceptions),
	* measured in an instrumented copy of METIS 4.0.1 on thousands of
	* matrices, is (10*nz+50*n+4096) * sizeof(int).  Two large matrices
	* exceeded this bound, one by almost a factor of 2 (Gupta/gupta2).
	*
	* If your program is terminated by METIS, try setting metis_memory to
	* 2.0, or even higher if needed.  By default, CHOLMOD assumes that METIS
	* does not have this problem (so that CHOLMOD will work correctly when
	* this issue is fixed in METIS).  Thus, the default value is zero.
	* This work-around is not guaranteed anyway.
	*
	* If a matrix exceeds this predicted memory usage, AMD is attempted
	* instead.  It, too, may run out of memory, but if it does so it will
	* not terminate your program.
	*/

    double metis_dswitch ;	/* METIS_NodeND in METIS 4.0.1 gives a seg */
    size_t metis_nswitch ;	/* fault with one matrix of order n = 3005 and
				 * nz = 6,036,025.  This is a very dense graph.
     * The workaround is to use AMD instead of METIS for matrices of dimension
     * greater than Common->metis_nswitch (default 3000) or more and with
     * density of Common->metis_dswitch (default 0.66) or more.
     * cholmod_nested_dissection has no problems with the same matrix, even
     * though it uses METIS_NodeComputeSeparator on this matrix.  If this
     * seg fault does not affect you, set metis_nswitch to zero or less,
     * and CHOLMOD will not switch to AMD based just on the density of the
     * matrix (it will still switch to AMD if the metis_memory parameter
     * causes the switch).
     */

    /* ---------------------------------------------------------------------- */
    /* workspace */
    /* ---------------------------------------------------------------------- */

    /* CHOLMOD has several routines that take less time than the size of
     * workspace they require.  Allocating and initializing the workspace would
     * dominate the run time, unless workspace is allocated and initialized
     * just once.  CHOLMOD allocates this space when needed, and holds it here
     * between calls to CHOLMOD.  cholmod_start sets these pointers to NULL
     * (which is why it must be the first routine called in CHOLMOD).
     * cholmod_finish frees the workspace (which is why it must be the last
     * call to CHOLMOD).
     */

    size_t nrow ;	/* size of Flag and Head */
    UF_long mark ;	/* mark value for Flag array */
    size_t iworksize ;	/* size of Iwork.  Upper bound: 6*nrow+ncol */
    size_t xworksize ;	/* size of Xwork,  in bytes.
			 * maxrank*nrow*sizeof(double) for update/downdate.
			 * 2*nrow*sizeof(double) otherwise */

    /* initialized workspace: contents needed between calls to CHOLMOD */
    void *Flag ;	/* size nrow, an integer array.  Kept cleared between
			 * calls to cholmod rouines (Flag [i] < mark) */

    void *Head ;	/* size nrow+1, an integer array. Kept cleared between
			 * calls to cholmod routines (Head [i] = EMPTY) */

    void *Xwork ; 	/* a double array.  Its size varies.  It is nrow for
			 * most routines (cholmod_rowfac, cholmod_add,
	* cholmod_aat, cholmod_norm, cholmod_ssmult) for the real case, twice
	* that when the input matrices are complex or zomplex.  It is of size
	* 2*nrow for cholmod_rowadd and cholmod_rowdel.  For cholmod_updown,
	* its size is maxrank*nrow where maxrank is 2, 4, or 8.  Kept cleared
	* between calls to cholmod (set to zero). */

    /* uninitialized workspace, contents not needed between calls to CHOLMOD */
    void *Iwork ;	/* size iworksize, 2*nrow+ncol for most routines,
			 * up to 6*nrow+ncol for cholmod_analyze. */

    int itype ;		/* If CHOLMOD_LONG, Flag, Head, and Iwork are UF_long.
			 * Otherwise all three arrays are int. */

    int dtype ;		/* double or float */

	/* Common->itype and Common->dtype are used to define the types of all
	 * sparse matrices, triplet matrices, dense matrices, and factors
	 * created using this Common struct.  The itypes and dtypes of all
	 * parameters to all CHOLMOD routines must match.  */

    int no_workspace_reallocate ;   /* this is an internal flag, used as a
	* precaution by cholmod_analyze.  It is normally false.  If true,
	* cholmod_allocate_work is not allowed to reallocate any workspace;
	* they must use the existing workspace in Common (Iwork, Flag, Head,
	* and Xwork).  Added for CHOLMOD v1.1 */

    /* ---------------------------------------------------------------------- */
    /* statistics */
    /* ---------------------------------------------------------------------- */

    /* fl and lnz are set only in cholmod_analyze and cholmod_rowcolcounts,
     * in the Cholesky modudle.  modfl is set only in the Modify module. */

    int status ;	    /* error code */
    double fl ;		    /* LL' flop count from most recent analysis */
    double lnz ;	    /* fundamental nz in L */
    double anz ;	    /* nonzeros in tril(A) if A is symmetric/lower,
			     * triu(A) if symmetric/upper, or tril(A*A') if
			     * unsymmetric, in last call to cholmod_analyze. */
    double modfl ;	    /* flop count from most recent update/downdate/
			     * rowadd/rowdel (excluding flops to modify the
			     * solution to Lx=b, if computed) */
    size_t malloc_count ;   /* # of objects malloc'ed minus the # free'd*/
    size_t memory_usage ;   /* peak memory usage in bytes */
    size_t memory_inuse ;   /* current memory usage in bytes */

    double nrealloc_col ;   /* # of column reallocations */
    double nrealloc_factor ;/* # of factor reallocations due to col. reallocs */
    double ndbounds_hit ;   /* # of times diagonal modified by dbound */

    double rowfacfl ;	    /* # of flops in last call to cholmod_rowfac */
    double aatfl ;	    /* # of flops to compute A(:,f)*A(:,f)' */

    /* ---------------------------------------------------------------------- */
    /* future expansion */
    /* ---------------------------------------------------------------------- */

    /* To allow CHOLMOD to be updated without recompiling the user application,
     * additional space is set aside here for future statistics, parameters,
     * and workspace.  Note:  additional entries were added in v1.1 to the
     * method array, above, and thus v1.0 and v1.1 are not binary compatible.
     *
     * v1.1 to the current version are binary compatible.
     */

    /* ---------------------------------------------------------------------- */
    double other1 [10] ;

    double SPQR_xstat [4] ;     /* for SuiteSparseQR statistics */

    /* SuiteSparseQR control parameters: */
    double SPQR_grain ;         /* task size is >= max (total flops / grain) */
    double SPQR_small ;         /* task size is >= small */

    /* ---------------------------------------------------------------------- */
    UF_long SPQR_istat [10] ;   /* for SuiteSparseQR statistics */
    UF_long other2 [6] ;        /* reduced from size 16 in v1.6 */

    /* ---------------------------------------------------------------------- */
    int other3 [10] ;       /* reduced from size 16 in v1.1. */

    int prefer_binary ;	    /* cholmod_read_triplet converts a symmetric
			     * pattern-only matrix into a real matrix.  If
	* prefer_binary is FALSE, the diagonal entries are set to 1 + the degree
	* of the row/column, and off-diagonal entries are set to -1 (resulting
	* in a positive definite matrix if the diagonal is zero-free).  Most
	* symmetric patterns are the pattern a positive definite matrix.  If
	* this parameter is TRUE, then the matrix is returned with a 1 in each
	* entry, instead.  Default: FALSE.  Added in v1.3. */

    /* control parameter (added for v1.2): */
    int default_nesdis ;    /* Default: FALSE.  If FALSE, then the default
			     * ordering strategy (when Common->nmethods == 0)
	* is to try the given ordering (if present), AMD, and then METIS if AMD
	* reports high fill-in.  If Common->default_nesdis is TRUE then NESDIS
	* is used instead in the default strategy. */

    /* statistic (added for v1.2): */
    int called_nd ;	    /* TRUE if the last call to
			     * cholmod_analyze called NESDIS or METIS. */

    int blas_ok ;           /* FALSE if BLAS int overflow; TRUE otherwise */

    /* SuiteSparseQR control parameters: */
    int SPQR_shrink ;        /* controls stack realloc method */
    int SPQR_nthreads ;      /* number of TBB threads, 0 = auto */

    /* ---------------------------------------------------------------------- */
    size_t  other4 [16] ;

    /* ---------------------------------------------------------------------- */
    void   *other5 [16] ;

} cholmod_common ;

/* -------------------------------------------------------------------------- */
/* cholmod_start:  first call to CHOLMOD */
/* -------------------------------------------------------------------------- */

int cholmod_start
(
    cholmod_common *Common
) ;

int cholmod_l_start (cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_finish:  last call to CHOLMOD */
/* -------------------------------------------------------------------------- */

int cholmod_finish
(
    cholmod_common *Common
) ;

int cholmod_l_finish (cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_defaults:  restore default parameters */
/* -------------------------------------------------------------------------- */

int cholmod_defaults
(
    cholmod_common *Common
) ;

int cholmod_l_defaults (cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_maxrank:  return valid maximum rank for update/downdate */
/* -------------------------------------------------------------------------- */

size_t cholmod_maxrank	/* returns validated value of Common->maxrank */
(
    /* ---- input ---- */
    size_t n,		/* A and L will have n rows */
    /* --------------- */
    cholmod_common *Common
) ;

size_t cholmod_l_maxrank (size_t, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_allocate_work:  allocate workspace in Common */
/* -------------------------------------------------------------------------- */

int cholmod_allocate_work
(
    /* ---- input ---- */
    size_t nrow,	/* size: Common->Flag (nrow), Common->Head (nrow+1) */
    size_t iworksize,	/* size of Common->Iwork */
    size_t xworksize,	/* size of Common->Xwork */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_allocate_work (size_t, size_t, size_t, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_free_work:  free workspace in Common */
/* -------------------------------------------------------------------------- */

int cholmod_free_work
(
    cholmod_common *Common
) ;

int cholmod_l_free_work (cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_clear_flag:  clear Flag workspace in Common */
/* -------------------------------------------------------------------------- */

/* use a macro for speed */
#define CHOLMOD_CLEAR_FLAG(Common) \
{ \
    Common->mark++ ; \
    if (Common->mark <= 0) \
    { \
	Common->mark = EMPTY ; \
	CHOLMOD (clear_flag) (Common) ; \
    } \
}

UF_long cholmod_clear_flag
(
    cholmod_common *Common
) ;

UF_long cholmod_l_clear_flag (cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_error:  called when CHOLMOD encounters an error */
/* -------------------------------------------------------------------------- */

int cholmod_error
(
    /* ---- input ---- */
    int status,		/* error status */
    const char *file,	/* name of source code file where error occured */
    int line,		/* line number in source code file where error occured*/
    const char *message,/* error message */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_error (int, const char *, int, const char *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_dbound:  for internal use in CHOLMOD only */
/* -------------------------------------------------------------------------- */

double cholmod_dbound	/* returns modified diagonal entry of D or L */
(
    /* ---- input ---- */
    double dj,		/* diagonal entry of D for LDL' or L for LL' */
    /* --------------- */
    cholmod_common *Common
) ;

double cholmod_l_dbound (double, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_hypot:  compute sqrt (x*x + y*y) accurately */
/* -------------------------------------------------------------------------- */

double cholmod_hypot
(
    /* ---- input ---- */
    double x, double y
) ;

double cholmod_l_hypot (double, double) ;

/* -------------------------------------------------------------------------- */
/* cholmod_divcomplex:  complex division, c = a/b */
/* -------------------------------------------------------------------------- */

int cholmod_divcomplex		/* return 1 if divide-by-zero, 0 otherise */
(
    /* ---- input ---- */
    double ar, double ai,	/* real and imaginary parts of a */
    double br, double bi,	/* real and imaginary parts of b */
    /* ---- output --- */
    double *cr, double *ci	/* real and imaginary parts of c */
) ;

int cholmod_l_divcomplex (double, double, double, double, double *, double *) ;


/* ========================================================================== */
/* === Core/cholmod_sparse ================================================== */
/* ========================================================================== */

/* A sparse matrix stored in compressed-column form. */

typedef struct cholmod_sparse_struct
{
    size_t nrow ;	/* the matrix is nrow-by-ncol */
    size_t ncol ;
    size_t nzmax ;	/* maximum number of entries in the matrix */

    /* pointers to int or UF_long: */
    void *p ;		/* p [0..ncol], the column pointers */
    void *i ;		/* i [0..nzmax-1], the row indices */

    /* for unpacked matrices only: */
    void *nz ;		/* nz [0..ncol-1], the # of nonzeros in each col.  In
			 * packed form, the nonzero pattern of column j is in
	* A->i [A->p [j] ... A->p [j+1]-1].  In unpacked form, column j is in
	* A->i [A->p [j] ... A->p [j]+A->nz[j]-1] instead.  In both cases, the
	* numerical values (if present) are in the corresponding locations in
	* the array x (or z if A->xtype is CHOLMOD_ZOMPLEX). */

    /* pointers to double or float: */
    void *x ;		/* size nzmax or 2*nzmax, if present */
    void *z ;		/* size nzmax, if present */

    int stype ;		/* Describes what parts of the matrix are considered:
			 *
	* 0:  matrix is "unsymmetric": use both upper and lower triangular parts
	*     (the matrix may actually be symmetric in pattern and value, but
	*     both parts are explicitly stored and used).  May be square or
	*     rectangular.
	* >0: matrix is square and symmetric, use upper triangular part.
	*     Entries in the lower triangular part are ignored.
	* <0: matrix is square and symmetric, use lower triangular part.
	*     Entries in the upper triangular part are ignored.
	*
	* Note that stype>0 and stype<0 are different for cholmod_sparse and
	* cholmod_triplet.  See the cholmod_triplet data structure for more
	* details.
	*/

    int itype ;		/* CHOLMOD_INT:     p, i, and nz are int.
			 * CHOLMOD_INTLONG: p is UF_long, i and nz are int.
			 * CHOLMOD_LONG:    p, i, and nz are UF_long.  */

    int xtype ;		/* pattern, real, complex, or zomplex */
    int dtype ;		/* x and z are double or float */
    int sorted ;	/* TRUE if columns are sorted, FALSE otherwise */
    int packed ;	/* TRUE if packed (nz ignored), FALSE if unpacked
			 * (nz is required) */

} cholmod_sparse ;

/* -------------------------------------------------------------------------- */
/* cholmod_allocate_sparse:  allocate a sparse matrix */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_allocate_sparse
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of A */
    size_t ncol,	/* # of columns of A */
    size_t nzmax,	/* max # of nonzeros of A */
    int sorted,		/* TRUE if columns of A sorted, FALSE otherwise */
    int packed,		/* TRUE if A will be packed, FALSE otherwise */
    int stype,		/* stype of A */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_allocate_sparse (size_t, size_t, size_t, int, int,
    int, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_free_sparse:  free a sparse matrix */
/* -------------------------------------------------------------------------- */

int cholmod_free_sparse
(
    /* ---- in/out --- */
    cholmod_sparse **A,	/* matrix to deallocate, NULL on output */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_free_sparse (cholmod_sparse **, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_reallocate_sparse:  change the size (# entries) of sparse matrix */
/* -------------------------------------------------------------------------- */

int cholmod_reallocate_sparse
(
    /* ---- input ---- */
    size_t nznew,	/* new # of entries in A */
    /* ---- in/out --- */
    cholmod_sparse *A,	/* matrix to reallocate */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_reallocate_sparse ( size_t, cholmod_sparse *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_nnz:  return number of nonzeros in a sparse matrix */
/* -------------------------------------------------------------------------- */

UF_long cholmod_nnz
(
    /* ---- input ---- */
    cholmod_sparse *A,
    /* --------------- */
    cholmod_common *Common
) ;

UF_long cholmod_l_nnz (cholmod_sparse *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_speye:  sparse identity matrix */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_speye
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of A */
    size_t ncol,	/* # of columns of A */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_speye (size_t, size_t, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_spzeros:  sparse zero matrix */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_spzeros
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of A */
    size_t ncol,	/* # of columns of A */
    size_t nzmax,	/* max # of nonzeros of A */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_spzeros (size_t, size_t, size_t, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_transpose:  transpose a sparse matrix */
/* -------------------------------------------------------------------------- */

/* Return A' or A.'  The "values" parameter is 0, 1, or 2 to denote the pattern
 * transpose, the array transpose (A.'), and the complex conjugate transpose
 * (A').
 */

cholmod_sparse *cholmod_transpose
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to transpose */
    int values,		/* 0: pattern, 1: array transpose, 2: conj. transpose */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_transpose (cholmod_sparse *, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_transpose_unsym:  transpose an unsymmetric sparse matrix */
/* -------------------------------------------------------------------------- */

/* Compute F = A', A (:,f)', or A (p,f)', where A is unsymmetric and F is
 * already allocated.  See cholmod_transpose for a simpler routine. */

int cholmod_transpose_unsym
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to transpose */
    int values,		/* 0: pattern, 1: array transpose, 2: conj. transpose */
    int *Perm,		/* size nrow, if present (can be NULL) */
    int *fset,		/* subset of 0:(A->ncol)-1 */
    size_t fsize,	/* size of fset */
    /* ---- output --- */
    cholmod_sparse *F,	/* F = A', A(:,f)', or A(p,f)' */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_transpose_unsym (cholmod_sparse *, int, UF_long *, UF_long *,
    size_t, cholmod_sparse *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_transpose_sym:  transpose a symmetric sparse matrix */
/* -------------------------------------------------------------------------- */

/* Compute F = A' or A (p,p)', where A is symmetric and F is already allocated.
 * See cholmod_transpose for a simpler routine. */

int cholmod_transpose_sym
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to transpose */
    int values,		/* 0: pattern, 1: array transpose, 2: conj. transpose */
    int *Perm,		/* size nrow, if present (can be NULL) */
    /* ---- output --- */
    cholmod_sparse *F,	/* F = A' or A(p,p)' */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_transpose_sym (cholmod_sparse *, int, UF_long *, cholmod_sparse *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_ptranspose:  transpose a sparse matrix */
/* -------------------------------------------------------------------------- */

/* Return A' or A(p,p)' if A is symmetric.  Return A', A(:,f)', or A(p,f)' if
 * A is unsymmetric. */

cholmod_sparse *cholmod_ptranspose
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to transpose */
    int values,		/* 0: pattern, 1: array transpose, 2: conj. transpose */
    int *Perm,		/* if non-NULL, F = A(p,f) or A(p,p) */
    int *fset,		/* subset of 0:(A->ncol)-1 */
    size_t fsize,	/* size of fset */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_ptranspose (cholmod_sparse *, int, UF_long *,
    UF_long *, size_t, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_sort:  sort row indices in each column of sparse matrix */
/* -------------------------------------------------------------------------- */

int cholmod_sort
(
    /* ---- in/out --- */
    cholmod_sparse *A,	/* matrix to sort */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_sort (cholmod_sparse *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_band:  C = tril (triu (A,k1), k2) */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_band
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to extract band matrix from */
    UF_long k1,		/* ignore entries below the k1-st diagonal */
    UF_long k2,		/* ignore entries above the k2-nd diagonal */
    int mode,		/* >0: numerical, 0: pattern, <0: pattern (no diag) */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_band (cholmod_sparse *, UF_long, UF_long, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_band_inplace:  A = tril (triu (A,k1), k2) */
/* -------------------------------------------------------------------------- */

int cholmod_band_inplace
(
    /* ---- input ---- */
    UF_long k1,		/* ignore entries below the k1-st diagonal */
    UF_long k2,		/* ignore entries above the k2-nd diagonal */
    int mode,		/* >0: numerical, 0: pattern, <0: pattern (no diag) */
    /* ---- in/out --- */
    cholmod_sparse *A,	/* matrix from which entries not in band are removed */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_band_inplace (UF_long, UF_long, int, cholmod_sparse *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_aat:  C = A*A' or A(:,f)*A(:,f)' */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_aat
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* input matrix; C=A*A' is constructed */
    int *fset,		/* subset of 0:(A->ncol)-1 */
    size_t fsize,	/* size of fset */
    int mode,		/* >0: numerical, 0: pattern, <0: pattern (no diag),
			 * -2: pattern only, no diagonal, add 50%+n extra
			 * space to C */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_aat (cholmod_sparse *, UF_long *, size_t, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy_sparse:  C = A, create an exact copy of a sparse matrix */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_copy_sparse
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_copy_sparse (cholmod_sparse *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy:  C = A, with possible change of stype */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_copy 
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to copy */
    int stype,		/* requested stype of C */
    int mode,		/* >0: numerical, 0: pattern, <0: pattern (no diag) */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_copy (cholmod_sparse *, int, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_add: C = alpha*A + beta*B */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_add
(
    /* ---- input ---- */
    cholmod_sparse *A,	    /* matrix to add */
    cholmod_sparse *B,	    /* matrix to add */
    double alpha [2],	    /* scale factor for A */
    double beta [2],	    /* scale factor for B */
    int values,		    /* if TRUE compute the numerical values of C */
    int sorted,		    /* if TRUE, sort columns of C */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_add (cholmod_sparse *, cholmod_sparse *, double *,
    double *, int, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_sparse_xtype: change the xtype of a sparse matrix */
/* -------------------------------------------------------------------------- */

int cholmod_sparse_xtype
(
    /* ---- input ---- */
    int to_xtype,	/* requested xtype (pattern, real, complex, zomplex) */
    /* ---- in/out --- */
    cholmod_sparse *A,	/* sparse matrix to change */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_sparse_xtype (int, cholmod_sparse *, cholmod_common *) ;


/* ========================================================================== */
/* === Core/cholmod_factor ================================================== */
/* ========================================================================== */

/* A symbolic and numeric factorization, either simplicial or supernodal.
 * In all cases, the row indices in the columns of L are kept sorted. */

typedef struct cholmod_factor_struct
{
    /* ---------------------------------------------------------------------- */
    /* for both simplicial and supernodal factorizations */
    /* ---------------------------------------------------------------------- */

    size_t n ;		/* L is n-by-n */

    size_t minor ;	/* If the factorization failed, L->minor is the column
			 * at which it failed (in the range 0 to n-1).  A value
			 * of n means the factorization was successful or
			 * the matrix has not yet been factorized. */

    /* ---------------------------------------------------------------------- */
    /* symbolic ordering and analysis */
    /* ---------------------------------------------------------------------- */

    void *Perm ;	/* size n, permutation used */
    void *ColCount ;	/* size n, column counts for simplicial L */

    /* ---------------------------------------------------------------------- */
    /* simplicial factorization */
    /* ---------------------------------------------------------------------- */

    size_t nzmax ;	/* size of i and x */

    void *p ;		/* p [0..ncol], the column pointers */
    void *i ;		/* i [0..nzmax-1], the row indices */
    void *x ;		/* x [0..nzmax-1], the numerical values */
    void *z ;
    void *nz ;		/* nz [0..ncol-1], the # of nonzeros in each column.
			 * i [p [j] ... p [j]+nz[j]-1] contains the row indices,
			 * and the numerical values are in the same locatins
			 * in x. The value of i [p [k]] is always k. */

    void *next ;	/* size ncol+2. next [j] is the next column in i/x */
    void *prev ;	/* size ncol+2. prev [j] is the prior column in i/x.
			 * head of the list is ncol+1, and the tail is ncol. */

    /* ---------------------------------------------------------------------- */
    /* supernodal factorization */
    /* ---------------------------------------------------------------------- */

    /* Note that L->x is shared with the simplicial data structure.  L->x has
     * size L->nzmax for a simplicial factor, and size L->xsize for a supernodal
     * factor. */

    size_t nsuper ;	/* number of supernodes */
    size_t ssize ;	/* size of s, integer part of supernodes */
    size_t xsize ;	/* size of x, real part of supernodes */
    size_t maxcsize ;	/* size of largest update matrix */
    size_t maxesize ;	/* max # of rows in supernodes, excl. triangular part */

    void *super ;	/* size nsuper+1, first col in each supernode */
    void *pi ;		/* size nsuper+1, pointers to integer patterns */
    void *px ;		/* size nsuper+1, pointers to real parts */
    void *s ;		/* size ssize, integer part of supernodes */

    /* ---------------------------------------------------------------------- */
    /* factorization type */
    /* ---------------------------------------------------------------------- */

    int ordering ;	/* ordering method used */

    int is_ll ;		/* TRUE if LL', FALSE if LDL' */
    int is_super ;	/* TRUE if supernodal, FALSE if simplicial */
    int is_monotonic ;	/* TRUE if columns of L appear in order 0..n-1.
			 * Only applicable to simplicial numeric types. */

    /* There are 8 types of factor objects that cholmod_factor can represent
     * (only 6 are used):
     *
     * Numeric types (xtype is not CHOLMOD_PATTERN)
     * --------------------------------------------
     *
     * simplicial LDL':  (is_ll FALSE, is_super FALSE).  Stored in compressed
     *	    column form, using the simplicial components above (nzmax, p, i,
     *	    x, z, nz, next, and prev).  The unit diagonal of L is not stored,
     *	    and D is stored in its place.  There are no supernodes.
     *
     * simplicial LL': (is_ll TRUE, is_super FALSE).  Uses the same storage
     *	    scheme as the simplicial LDL', except that D does not appear.
     *	    The first entry of each column of L is the diagonal entry of
     *	    that column of L.
     *
     * supernodal LDL': (is_ll FALSE, is_super TRUE).  Not used.
     *	    FUTURE WORK:  add support for supernodal LDL'
     *
     * supernodal LL': (is_ll TRUE, is_super TRUE).  A supernodal factor,
     *	    using the supernodal components described above (nsuper, ssize,
     *	    xsize, maxcsize, maxesize, super, pi, px, s, x, and z).
     *
     *
     * Symbolic types (xtype is CHOLMOD_PATTERN)
     * -----------------------------------------
     *
     * simplicial LDL': (is_ll FALSE, is_super FALSE).  Nothing is present
     *	    except Perm and ColCount.
     *
     * simplicial LL': (is_ll TRUE, is_super FALSE).  Identical to the
     *	    simplicial LDL', except for the is_ll flag.
     *
     * supernodal LDL': (is_ll FALSE, is_super TRUE).  Not used.
     *	    FUTURE WORK:  add support for supernodal LDL'
     *
     * supernodal LL': (is_ll TRUE, is_super TRUE).  A supernodal symbolic
     *	    factorization.  The simplicial symbolic information is present
     *	    (Perm and ColCount), as is all of the supernodal factorization
     *	    except for the numerical values (x and z).
     */

    int itype ;		/* The integer arrays are Perm, ColCount, p, i, nz,
			 * next, prev, super, pi, px, and s.  If itype is
			 * CHOLMOD_INT, all of these are int arrays.
			 * CHOLMOD_INTLONG: p, pi, px are UF_long, others int.
			 * CHOLMOD_LONG:    all integer arrays are UF_long. */
    int xtype ;		/* pattern, real, complex, or zomplex */
    int dtype ;		/* x and z double or float */

} cholmod_factor ;


/* -------------------------------------------------------------------------- */
/* cholmod_allocate_factor: allocate a factor (symbolic LL' or LDL') */
/* -------------------------------------------------------------------------- */

cholmod_factor *cholmod_allocate_factor
(
    /* ---- input ---- */
    size_t n,		/* L is n-by-n */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_factor *cholmod_l_allocate_factor (size_t, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_free_factor:  free a factor */
/* -------------------------------------------------------------------------- */

int cholmod_free_factor
(
    /* ---- in/out --- */
    cholmod_factor **L,	/* factor to free, NULL on output */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_free_factor (cholmod_factor **, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_reallocate_factor:  change the # entries in a factor */
/* -------------------------------------------------------------------------- */

int cholmod_reallocate_factor
(
    /* ---- input ---- */
    size_t nznew,	/* new # of entries in L */
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to modify */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_reallocate_factor (size_t, cholmod_factor *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_change_factor:  change the type of factor (e.g., LDL' to LL') */
/* -------------------------------------------------------------------------- */

int cholmod_change_factor
(
    /* ---- input ---- */
    int to_xtype,	/* to CHOLMOD_PATTERN, _REAL, _COMPLEX, _ZOMPLEX */
    int to_ll,		/* TRUE: convert to LL', FALSE: LDL' */
    int to_super,	/* TRUE: convert to supernodal, FALSE: simplicial */
    int to_packed,	/* TRUE: pack simplicial columns, FALSE: do not pack */
    int to_monotonic,	/* TRUE: put simplicial columns in order, FALSE: not */
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to modify */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_change_factor ( int, int, int, int, int, cholmod_factor *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_pack_factor:  pack the columns of a factor */
/* -------------------------------------------------------------------------- */

/* Pack the columns of a simplicial factor.  Unlike cholmod_change_factor,
 * it can pack the columns of a factor even if they are not stored in their
 * natural order (non-monotonic). */

int cholmod_pack_factor
(
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to modify */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_pack_factor (cholmod_factor *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_reallocate_column:  resize a single column of a factor */
/* -------------------------------------------------------------------------- */

int cholmod_reallocate_column
(
    /* ---- input ---- */
    size_t j,		/* the column to reallocate */
    size_t need,	/* required size of column j */
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to modify */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_reallocate_column (size_t, size_t, cholmod_factor *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_factor_to_sparse:  create a sparse matrix copy of a factor */
/* -------------------------------------------------------------------------- */

/* Only operates on numeric factors, not symbolic ones */

cholmod_sparse *cholmod_factor_to_sparse
(
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to copy, converted to symbolic on output */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_factor_to_sparse (cholmod_factor *,
	cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy_factor:  create a copy of a factor */
/* -------------------------------------------------------------------------- */

cholmod_factor *cholmod_copy_factor
(
    /* ---- input ---- */
    cholmod_factor *L,	/* factor to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_factor *cholmod_l_copy_factor (cholmod_factor *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_factor_xtype: change the xtype of a factor */
/* -------------------------------------------------------------------------- */

int cholmod_factor_xtype
(
    /* ---- input ---- */
    int to_xtype,	/* requested xtype (real, complex, or zomplex) */
    /* ---- in/out --- */
    cholmod_factor *L,	/* factor to change */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_factor_xtype (int, cholmod_factor *, cholmod_common *) ;


/* ========================================================================== */
/* === Core/cholmod_dense =================================================== */
/* ========================================================================== */

/* A dense matrix in column-oriented form.  It has no itype since it contains
 * no integers.  Entry in row i and column j is located in x [i+j*d].
 */

typedef struct cholmod_dense_struct
{
    size_t nrow ;	/* the matrix is nrow-by-ncol */
    size_t ncol ;
    size_t nzmax ;	/* maximum number of entries in the matrix */
    size_t d ;		/* leading dimension (d >= nrow must hold) */
    void *x ;		/* size nzmax or 2*nzmax, if present */
    void *z ;		/* size nzmax, if present */
    int xtype ;		/* pattern, real, complex, or zomplex */
    int dtype ;		/* x and z double or float */

} cholmod_dense ;

/* -------------------------------------------------------------------------- */
/* cholmod_allocate_dense:  allocate a dense matrix (contents uninitialized) */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_allocate_dense
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of matrix */
    size_t ncol,	/* # of columns of matrix */
    size_t d,		/* leading dimension */
    int xtype,		/* CHOLMOD_REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_allocate_dense (size_t, size_t, size_t, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_zeros: allocate a dense matrix and set it to zero */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_zeros
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of matrix */
    size_t ncol,	/* # of columns of matrix */
    int xtype,		/* CHOLMOD_REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_zeros (size_t, size_t, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_ones: allocate a dense matrix and set it to all ones */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_ones
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of matrix */
    size_t ncol,	/* # of columns of matrix */
    int xtype,		/* CHOLMOD_REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_ones (size_t, size_t, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_eye: allocate a dense matrix and set it to the identity matrix */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_eye
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of matrix */
    size_t ncol,	/* # of columns of matrix */
    int xtype,		/* CHOLMOD_REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_eye (size_t, size_t, int, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_free_dense:  free a dense matrix */
/* -------------------------------------------------------------------------- */

int cholmod_free_dense
(
    /* ---- in/out --- */
    cholmod_dense **X,	/* dense matrix to deallocate, NULL on output */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_free_dense (cholmod_dense **, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_sparse_to_dense:  create a dense matrix copy of a sparse matrix */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_sparse_to_dense
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_sparse_to_dense (cholmod_sparse *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_dense_to_sparse:  create a sparse matrix copy of a dense matrix */
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_dense_to_sparse
(
    /* ---- input ---- */
    cholmod_dense *X,	/* matrix to copy */
    int values,		/* TRUE if values to be copied, FALSE otherwise */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_dense_to_sparse (cholmod_dense *, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy_dense:  create a copy of a dense matrix */
/* -------------------------------------------------------------------------- */

cholmod_dense *cholmod_copy_dense
(
    /* ---- input ---- */
    cholmod_dense *X,	/* matrix to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_dense *cholmod_l_copy_dense (cholmod_dense *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy_dense2:  copy a dense matrix (pre-allocated) */
/* -------------------------------------------------------------------------- */

int cholmod_copy_dense2
(
    /* ---- input ---- */
    cholmod_dense *X,	/* matrix to copy */
    /* ---- output --- */
    cholmod_dense *Y,	/* copy of matrix X */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_copy_dense2 (cholmod_dense *, cholmod_dense *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_dense_xtype: change the xtype of a dense matrix */
/* -------------------------------------------------------------------------- */

int cholmod_dense_xtype
(
    /* ---- input ---- */
    int to_xtype,	/* requested xtype (real, complex,or zomplex) */
    /* ---- in/out --- */
    cholmod_dense *X,	/* dense matrix to change */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_dense_xtype (int, cholmod_dense *, cholmod_common *) ;


/* ========================================================================== */
/* === Core/cholmod_triplet ================================================= */
/* ========================================================================== */

/* A sparse matrix stored in triplet form. */

typedef struct cholmod_triplet_struct
{
    size_t nrow ;	/* the matrix is nrow-by-ncol */
    size_t ncol ;
    size_t nzmax ;	/* maximum number of entries in the matrix */
    size_t nnz ;	/* number of nonzeros in the matrix */

    void *i ;		/* i [0..nzmax-1], the row indices */
    void *j ;		/* j [0..nzmax-1], the column indices */
    void *x ;		/* size nzmax or 2*nzmax, if present */
    void *z ;		/* size nzmax, if present */

    int stype ;		/* Describes what parts of the matrix are considered:
			 *
	* 0:  matrix is "unsymmetric": use both upper and lower triangular parts
	*     (the matrix may actually be symmetric in pattern and value, but
	*     both parts are explicitly stored and used).  May be square or
	*     rectangular.
	* >0: matrix is square and symmetric.  Entries in the lower triangular
	*     part are transposed and added to the upper triangular part when
	*     the matrix is converted to cholmod_sparse form.
	* <0: matrix is square and symmetric.  Entries in the upper triangular
	*     part are transposed and added to the lower triangular part when
	*     the matrix is converted to cholmod_sparse form.
	*
	* Note that stype>0 and stype<0 are different for cholmod_sparse and
	* cholmod_triplet.  The reason is simple.  You can permute a symmetric
	* triplet matrix by simply replacing a row and column index with their
	* new row and column indices, via an inverse permutation.  Suppose
	* P = L->Perm is your permutation, and Pinv is an array of size n.
	* Suppose a symmetric matrix A is represent by a triplet matrix T, with
	* entries only in the upper triangular part.  Then the following code:
	*
	*	Ti = T->i ;
	*	Tj = T->j ;
	*	for (k = 0 ; k < n  ; k++) Pinv [P [k]] = k ;
	*	for (k = 0 ; k < nz ; k++) Ti [k] = Pinv [Ti [k]] ;
	*	for (k = 0 ; k < nz ; k++) Tj [k] = Pinv [Tj [k]] ;
	*
	* creates the triplet form of C=P*A*P'.  However, if T initially
	* contains just the upper triangular entries (T->stype = 1), after
	* permutation it has entries in both the upper and lower triangular
	* parts.  These entries should be transposed when constructing the
	* cholmod_sparse form of A, which is what cholmod_triplet_to_sparse
	* does.  Thus:
	*
	*	C = cholmod_triplet_to_sparse (T, 0, &Common) ;
	*
	* will return the matrix C = P*A*P'.
	*
	* Since the triplet matrix T is so simple to generate, it's quite easy
	* to remove entries that you do not want, prior to converting T to the
	* cholmod_sparse form.  So if you include these entries in T, CHOLMOD
	* assumes that there must be a reason (such as the one above).  Thus,
	* no entry in a triplet matrix is ever ignored.
	*/

    int itype ;		/* CHOLMOD_LONG: i and j are UF_long.  Otherwise int. */
    int xtype ;		/* pattern, real, complex, or zomplex */
    int dtype ;		/* x and z are double or float */

} cholmod_triplet ;

/* -------------------------------------------------------------------------- */
/* cholmod_allocate_triplet:  allocate a triplet matrix */
/* -------------------------------------------------------------------------- */

cholmod_triplet *cholmod_allocate_triplet
(
    /* ---- input ---- */
    size_t nrow,	/* # of rows of T */
    size_t ncol,	/* # of columns of T */
    size_t nzmax,	/* max # of nonzeros of T */
    int stype,		/* stype of T */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_triplet *cholmod_l_allocate_triplet (size_t, size_t, size_t, int, int,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_free_triplet:  free a triplet matrix */
/* -------------------------------------------------------------------------- */

int cholmod_free_triplet
(
    /* ---- in/out --- */
    cholmod_triplet **T,    /* triplet matrix to deallocate, NULL on output */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_free_triplet (cholmod_triplet **, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_reallocate_triplet:  change the # of entries in a triplet matrix */
/* -------------------------------------------------------------------------- */

int cholmod_reallocate_triplet
(
    /* ---- input ---- */
    size_t nznew,	/* new # of entries in T */
    /* ---- in/out --- */
    cholmod_triplet *T,	/* triplet matrix to modify */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_reallocate_triplet (size_t, cholmod_triplet *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_sparse_to_triplet:  create a triplet matrix copy of a sparse matrix*/
/* -------------------------------------------------------------------------- */

cholmod_triplet *cholmod_sparse_to_triplet
(
    /* ---- input ---- */
    cholmod_sparse *A,	/* matrix to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_triplet *cholmod_l_sparse_to_triplet (cholmod_sparse *,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_triplet_to_sparse:  create a sparse matrix copy of a triplet matrix*/
/* -------------------------------------------------------------------------- */

cholmod_sparse *cholmod_triplet_to_sparse
(
    /* ---- input ---- */
    cholmod_triplet *T,	/* matrix to copy */
    size_t nzmax,	/* allocate at least this much space in output matrix */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_sparse *cholmod_l_triplet_to_sparse (cholmod_triplet *, size_t,
    cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_copy_triplet:  create a copy of a triplet matrix */
/* -------------------------------------------------------------------------- */

cholmod_triplet *cholmod_copy_triplet
(
    /* ---- input ---- */
    cholmod_triplet *T,	/* matrix to copy */
    /* --------------- */
    cholmod_common *Common
) ;

cholmod_triplet *cholmod_l_copy_triplet (cholmod_triplet *, cholmod_common *) ;

/* -------------------------------------------------------------------------- */
/* cholmod_triplet_xtype: change the xtype of a triplet matrix */
/* -------------------------------------------------------------------------- */

int cholmod_triplet_xtype
(
    /* ---- input ---- */
    int to_xtype,	/* requested xtype (pattern, real, complex,or zomplex)*/
    /* ---- in/out --- */
    cholmod_triplet *T,	/* triplet matrix to change */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_triplet_xtype (int, cholmod_triplet *, cholmod_common *) ;


/* ========================================================================== */
/* === Core/cholmod_memory ================================================== */
/* ========================================================================== */

/* The user may make use of these, just like malloc and free.  You can even
 * malloc an object and safely free it with cholmod_free, and visa versa
 * (except that the memory usage statistics will be corrupted).  These routines
 * do differ from malloc and free.  If cholmod_free is given a NULL pointer,
 * for example, it does nothing (unlike the ANSI free).  cholmod_realloc does
 * not return NULL if given a non-NULL pointer and a nonzero size, even if it
 * fails (it returns the original pointer and sets an error code in
 * Common->status instead).
 *
 * CHOLMOD keeps track of the amount of memory it has allocated, and so the
 * cholmod_free routine also takes the size of the object being freed.  This
 * is only used for statistics.  If you, the user of CHOLMOD, pass the wrong
 * size, the only consequence is that the memory usage statistics will be
 * corrupted.
 */

void *cholmod_malloc	/* returns pointer to the newly malloc'd block */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* --------------- */
    cholmod_common *Common
) ;

void *cholmod_l_malloc (size_t, size_t, cholmod_common *) ;

void *cholmod_calloc	/* returns pointer to the newly calloc'd block */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* --------------- */
    cholmod_common *Common
) ;

void *cholmod_l_calloc (size_t, size_t, cholmod_common *) ;

void *cholmod_free	/* always returns NULL */
(
    /* ---- input ---- */
    size_t n,		/* number of items */
    size_t size,	/* size of each item */
    /* ---- in/out --- */
    void *p,		/* block of memory to free */
    /* --------------- */
    cholmod_common *Common
) ;

void *cholmod_l_free (size_t, size_t, void *, cholmod_common *) ;

void *cholmod_realloc	/* returns pointer to reallocated block */
(
    /* ---- input ---- */
    size_t nnew,	/* requested # of items in reallocated block */
    size_t size,	/* size of each item */
    /* ---- in/out --- */
    void *p,		/* block of memory to realloc */
    size_t *n,		/* current size on input, nnew on output if successful*/
    /* --------------- */
    cholmod_common *Common
) ;

void *cholmod_l_realloc (size_t, size_t, void *, size_t *, cholmod_common *) ;

int cholmod_realloc_multiple
(
    /* ---- input ---- */
    size_t nnew,	/* requested # of items in reallocated blocks */
    int nint,		/* number of int/UF_long blocks */
    int xtype,		/* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
    /* ---- in/out --- */
    void **I,		/* int or UF_long block */
    void **J,		/* int or UF_long block */
    void **X,		/* complex, double, or float block */
    void **Z,		/* zomplex case only: double or float block */
    size_t *n,		/* current size of the I,J,X,Z blocks on input,
			 * nnew on output if successful */
    /* --------------- */
    cholmod_common *Common
) ;

int cholmod_l_realloc_multiple (size_t, int, int, void **, void **, void **,
    void **, size_t *, cholmod_common *) ;

/* ========================================================================== */
/* === symmetry types ======================================================= */
/* ========================================================================== */

#define CHOLMOD_MM_RECTANGULAR 1
#define CHOLMOD_MM_UNSYMMETRIC 2
#define CHOLMOD_MM_SYMMETRIC 3
#define CHOLMOD_MM_HERMITIAN 4
#define CHOLMOD_MM_SKEW_SYMMETRIC 5
#define CHOLMOD_MM_SYMMETRIC_POSDIAG 6
#define CHOLMOD_MM_HERMITIAN_POSDIAG 7

/* ========================================================================== */
/* === Numerical relop macros =============================================== */
/* ========================================================================== */

/* These macros correctly handle the NaN case.
 *
 *  CHOLMOD_IS_NAN(x):
 *	True if x is NaN.  False otherwise.  The commonly-existing isnan(x)
 *	function could be used, but it's not in Kernighan & Ritchie 2nd edition
 *	(ANSI C89).  It may appear in <math.h>, but I'm not certain about
 *	portability.  The expression x != x is true if and only if x is NaN,
 *	according to the IEEE 754 floating-point standard.
 *
 *  CHOLMOD_IS_ZERO(x):
 *	True if x is zero.  False if x is nonzero, NaN, or +/- Inf.
 *	This is (x == 0) if the compiler is IEEE 754 compliant.
 *
 *  CHOLMOD_IS_NONZERO(x):
 *	True if x is nonzero, NaN, or +/- Inf.  False if x zero.
 *	This is (x != 0) if the compiler is IEEE 754 compliant.
 *
 *  CHOLMOD_IS_LT_ZERO(x):
 *	True if x is < zero or -Inf.  False if x is >= 0, NaN, or +Inf.
 *	This is (x < 0) if the compiler is IEEE 754 compliant.
 *
 *  CHOLMOD_IS_GT_ZERO(x):
 *	True if x is > zero or +Inf.  False if x is <= 0, NaN, or -Inf.
 *	This is (x > 0) if the compiler is IEEE 754 compliant.
 *
 *  CHOLMOD_IS_LE_ZERO(x):
 *	True if x is <= zero or -Inf.  False if x is > 0, NaN, or +Inf.
 *	This is (x <= 0) if the compiler is IEEE 754 compliant.
 */

#ifdef CHOLMOD_WINDOWS

/* Yes, this is exceedingly ugly.  Blame Microsoft, which hopelessly */
/* violates the IEEE 754 floating-point standard in a bizarre way. */
/* If you're using an IEEE 754-compliant compiler, then x != x is true */
/* iff x is NaN.  For Microsoft, (x < x) is true iff x is NaN. */
/* So either way, this macro safely detects a NaN. */
#define CHOLMOD_IS_NAN(x)	(((x) != (x)) || (((x) < (x))))
#define CHOLMOD_IS_ZERO(x)	(((x) == 0.) && !CHOLMOD_IS_NAN(x))
#define CHOLMOD_IS_NONZERO(x)	(((x) != 0.) || CHOLMOD_IS_NAN(x))
#define CHOLMOD_IS_LT_ZERO(x)	(((x) < 0.) && !CHOLMOD_IS_NAN(x))
#define CHOLMOD_IS_GT_ZERO(x)	(((x) > 0.) && !CHOLMOD_IS_NAN(x))
#define CHOLMOD_IS_LE_ZERO(x)	(((x) <= 0.) && !CHOLMOD_IS_NAN(x))

#else

/* These all work properly, according to the IEEE 754 standard ... except on */
/* a PC with windows.  Works fine in Linux on the same PC... */
#define CHOLMOD_IS_NAN(x)	((x) != (x))
#define CHOLMOD_IS_ZERO(x)	((x) == 0.)
#define CHOLMOD_IS_NONZERO(x)	((x) != 0.)
#define CHOLMOD_IS_LT_ZERO(x)	((x) < 0.)
#define CHOLMOD_IS_GT_ZERO(x)	((x) > 0.)
#define CHOLMOD_IS_LE_ZERO(x)	((x) <= 0.)

#endif

#endif
