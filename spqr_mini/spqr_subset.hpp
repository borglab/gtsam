// =============================================================================
// === spqr.hpp ================================================================
// =============================================================================

// Internal definitions and non-user-callable routines.  This should not be
// included in the user's code.

#ifndef SPQR_INTERNAL_H
#define SPQR_INTERNAL_H

// -----------------------------------------------------------------------------
// include files
// -----------------------------------------------------------------------------

#include "UFconfig.h"
extern "C" {
#include "cholmod_core.h"
#include "cholmod_blas.h"
}
#include "SuiteSparseQR_definitions.h"
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <cstring>

#include <complex>
typedef std::complex<double> Complex ;

// -----------------------------------------------------------------------------
// debugging and printing control
// -----------------------------------------------------------------------------

// force debugging off
#ifndef NDEBUG
#define NDEBUG
#endif

// force printing off
#ifndef NPRINT
#define NPRINT
#endif

// uncomment the following line to turn on debugging (SPQR will be slow!)
/*
#undef NDEBUG
*/

// uncomment the following line to turn on printing (LOTS of output!)
/*
#undef NPRINT
*/

// uncomment the following line to turn on expensive debugging (very slow!)
/*
#define DEBUG_EXPENSIVE
*/

// -----------------------------------------------------------------------------
// Int is defined at UF_long, from UFconfig.h
// -----------------------------------------------------------------------------

#define Int UF_long
#define Int_max UF_long_max

// -----------------------------------------------------------------------------
// basic macros
// -----------------------------------------------------------------------------

#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define EMPTY (-1)
#define TRUE 1
#define FALSE 0 
#define IMPLIES(p,q) (!(p) || (q))

// NULL should already be defined, but ensure it is here.
#ifndef NULL
#define NULL ((void *) 0)
#endif

// column-major indexing; A[i,j] is A (INDEX (i,j,lda))
#define INDEX(i,j,lda) ((i) + ((j)*(lda)))

// FLIP is a "negation about -1", and is used to mark an integer i that is
// normally non-negative.  FLIP (EMPTY) is EMPTY.  FLIP of a number > EMPTY
// is negative, and FLIP of a number < EMTPY is positive.  FLIP (FLIP (i)) = i
// for all integers i.  UNFLIP (i) is >= EMPTY.
#define EMPTY (-1)
#define FLIP(i) (-(i)-2)
#define UNFLIP(i) (((i) < EMPTY) ? FLIP (i) : (i))

// -----------------------------------------------------------------------------
// additional include files
// -----------------------------------------------------------------------------

#ifdef MATLAB_MEX_FILE
#include "mex.h"
#endif

#define ITYPE CHOLMOD_LONG
#define DTYPE CHOLMOD_DOUBLE
#define ID UF_long_id

// -----------------------------------------------------------------------------

#define ERROR(status,msg) \
    printf ("CHOLMOD error: %s\n",msg) // Kai: disable cholmod_l_error to prevent from including tons of files

// Check a pointer and return if null.  Set status to invalid, unless the
// status is already "out of memory"
#define RETURN_IF_NULL(A,result) \
{ \
    if ((A) == NULL) \
    { \
	if (cc->status != CHOLMOD_OUT_OF_MEMORY) \
	{ \
	    ERROR (CHOLMOD_INVALID, NULL) ; \
	} \
	return (result) ; \
    } \
}

// Return if Common is NULL or invalid
#define RETURN_IF_NULL_COMMON(result) \
{ \
    if (cc == NULL) \
    { \
	return (result) ; \
    } \
    if (cc->itype != ITYPE || cc->dtype != DTYPE) \
    { \
	cc->status = CHOLMOD_INVALID ; \
	return (result) ; \
    } \
}

#define RETURN_IF_XTYPE_INVALID(A,result) \
{ \
    if (A->xtype != xtype) \
    { \
        ERROR (CHOLMOD_INVALID, "invalid xtype") ; \
        return (result) ; \
    } \
}

// -----------------------------------------------------------------------------
// debugging and printing macros
// -----------------------------------------------------------------------------

#ifndef NDEBUG

    #ifdef MATLAB_MEX_FILE

        // #define ASSERT(e) mxAssert (e, "error: ")

        extern char spqr_mx_debug_string [200] ;
        char *spqr_mx_id (int line) ;

        #define ASSERT(e) \
            ((e) ? (void) 0 : \
            mexErrMsgIdAndTxt (spqr_mx_id (__LINE__), \
            "assert: (" #e ") file:"  __FILE__ ))

    #else

        #include <assert.h>
        #define ASSERT(e) assert (e)

    #endif

    #define DEBUG(e) e
    #ifdef DEBUG_EXPENSIVE
        #define DEBUG2(e) e
        #define ASSERT2(e) ASSERT(e)
    #else
        #define DEBUG2(e)
        #define ASSERT2(e)
    #endif

#else

    #define ASSERT(e)
    #define ASSERT2(e)
    #define DEBUG(e)
    #define DEBUG2(e)

#endif

#ifndef NPRINT

    #ifdef MATLAB_MEX_FILE
        #define PR(e) mexPrintf e
    #else
        #define PR(e) printf e
    #endif

    #define PRVAL(e) spqrDebug_print (e)

#else

    #define PR(e)
    #define PRVAL(e)

#endif

// -----------------------------------------------------------------------------
// For counting flops; disabled if TBB is used or timing not enabled
// -----------------------------------------------------------------------------

#if defined(TIMING)
#define FLOP_COUNT(f) { if (cc->SPQR_grain <= 1) cc->other1 [0] += (f) ; }
#else
#define FLOP_COUNT(f)
#endif

template <typename Entry> void spqr_larftb
(
		// inputs, not modified (V is modified and then restored on output)
		int method,     // 0,1,2,3
		Int m,          // C is m-by-n
		Int n,
		Int k,          // V is v-by-k
										// for methods 0 and 1, v = m,
										// for methods 2 and 3, v = n
		Int ldc,        // leading dimension of C
		Int ldv,        // leading dimension of V
		Entry *V,       // V is v-by-k, unit lower triangular (diag not stored)
		Entry *Tau,     // size k, the k Householder coefficients

		// input/output
		Entry *C,       // C is m-by-n, with leading dimension ldc

		// workspace, not defined on input or output
		Entry *W,       // for methods 0,1: size k*k + n*k
										// for methods 2,3: size k*k + m*k
		cholmod_common *cc
) ;

// returns rank of F, or 0 on error
template <typename Entry> Int spqr_front
(
    // input, not modified
    Int m,              // F is m-by-n with leading dimension m
    Int n,
    Int npiv,           // number of pivot columns
    double tol,         // a column is flagged as dead if its norm is <= tol
    Int ntol,           // apply tol only to first ntol pivot columns
    Int fchunk,         // block size for compact WY Householder reflections,
                        // treated as 1 if fchunk <= 1

    // input/output
    Entry *F,           // frontal matrix F of size m-by-n
    Int *Stair,         // size n, entries F (Stair[k]:m-1, k) are all zero,
                        // and remain zero on output.
    char *Rdead,        // size npiv; all zero on input.  If k is dead,
                        // Rdead [k] is set to 1

    // output, not defined on input
    Entry *Tau,         // size n, Householder coefficients

    // workspace, undefined on input and output
    Entry *W,           // size b*(n+b), where b = min (fchunk,n,m)

    // input/output
    double *wscale,
    double *wssq,

    cholmod_common *cc  // for cc->hypotenuse function
) ;


// =============================================================================
// === spqr_conj ===============================================================
// =============================================================================

inline double spqr_conj (double x)
{
    return (x) ;
}

inline Complex spqr_conj (Complex x)
{
    return (std::conj (x)) ;
}

// =============================================================================
// === spqr_abs ================================================================
// =============================================================================

inline double spqr_abs (double x, cholmod_common *cc)       // cc is unused
{
    return (fabs (x)) ;
}

inline double spqr_abs (Complex x, cholmod_common *cc)
{
    return (cc->hypotenuse (x.real ( ), x.imag ( ))) ;
}

// =============================================================================
// === BLAS interface ==========================================================
// =============================================================================

// To compile SuiteSparseQR with 64-bit BLAS, use -DBLAS64.  See also
// CHOLMOD/Include/cholmod_blas.h

extern "C" {
#include "cholmod_blas.h"
}

#ifdef SUN64

#define BLAS_DNRM2    dnrm2_64_
#define LAPACK_DLARF  dlarf_64_
#define LAPACK_DLARFG dlarfg_64_
#define LAPACK_DLARFT dlarft_64_
#define LAPACK_DLARFB dlarfb_64_

#define BLAS_DZNRM2   dznrm2_64_
#define LAPACK_ZLARF  zlarf_64_
#define LAPACK_ZLARFG zlarfg_64_
#define LAPACK_ZLARFT zlarft_64_
#define LAPACK_ZLARFB zlarfb_64_

#elif defined (BLAS_NO_UNDERSCORE)

#define BLAS_DNRM2    dnrm2
#define LAPACK_DLARF  dlarf
#define LAPACK_DLARFG dlarfg
#define LAPACK_DLARFT dlarft
#define LAPACK_DLARFB dlarfb

#define BLAS_DZNRM2   dznrm2
#define LAPACK_ZLARF  zlarf
#define LAPACK_ZLARFG zlarfg
#define LAPACK_ZLARFT zlarft
#define LAPACK_ZLARFB zlarfb

#else

#define BLAS_DNRM2    dnrm2_
#define LAPACK_DLARF  dlarf_
#define LAPACK_DLARFG dlarfg_
#define LAPACK_DLARFT dlarft_
#define LAPACK_DLARFB dlarfb_

#define BLAS_DZNRM2   dznrm2_
#define LAPACK_ZLARF  zlarf_
#define LAPACK_ZLARFG zlarfg_
#define LAPACK_ZLARFT zlarft_
#define LAPACK_ZLARFB zlarfb_

#endif

// =============================================================================
// === BLAS and LAPACK prototypes ==============================================
// =============================================================================

extern "C"
{

void LAPACK_DLARFT (char *direct, char *storev, BLAS_INT *n, BLAS_INT *k,
    double *V, BLAS_INT *ldv, double *Tau, double *T, BLAS_INT *ldt) ;

void LAPACK_ZLARFT (char *direct, char *storev, BLAS_INT *n, BLAS_INT *k,
    Complex *V, BLAS_INT *ldv, Complex *Tau, Complex *T, BLAS_INT *ldt) ;

void LAPACK_DLARFB (char *side, char *trans, char *direct, char *storev,
    BLAS_INT *m, BLAS_INT *n, BLAS_INT *k, double *V, BLAS_INT *ldv,
    double *T, BLAS_INT *ldt, double *C, BLAS_INT *ldc, double *Work,
    BLAS_INT *ldwork) ;

void LAPACK_ZLARFB (char *side, char *trans, char *direct, char *storev,
    BLAS_INT *m, BLAS_INT *n, BLAS_INT *k, Complex *V, BLAS_INT *ldv,
    Complex *T, BLAS_INT *ldt, Complex *C, BLAS_INT *ldc, Complex *Work,
    BLAS_INT *ldwork) ;

double BLAS_DNRM2 (BLAS_INT *n, double *X, BLAS_INT *incx) ;

double BLAS_DZNRM2 (BLAS_INT *n, Complex *X, BLAS_INT *incx) ;

void LAPACK_DLARFG (BLAS_INT *n, double *alpha, double *X, BLAS_INT *incx,
    double *tau) ;

void LAPACK_ZLARFG (BLAS_INT *n, Complex *alpha, Complex *X, BLAS_INT *incx,
    Complex *tau) ;

void LAPACK_DLARF (char *side, BLAS_INT *m, BLAS_INT *n, double *V,
    BLAS_INT *incv, double *tau, double *C, BLAS_INT *ldc, double *Work) ;

void LAPACK_ZLARF (char *side, BLAS_INT *m, BLAS_INT *n, Complex *V,
    BLAS_INT *incv, Complex *tau, Complex *C, BLAS_INT *ldc, Complex *Work) ;

}

#endif
