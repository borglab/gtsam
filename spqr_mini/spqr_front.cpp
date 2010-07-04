// =============================================================================
// === spqr_front ==============================================================
// =============================================================================

/* Given an m-by-n frontal matrix, use Householder reflections to reduce it
   to upper trapezoidal form.  Columns 0:npiv-1 are checked against tol.

        0    # x x x x x x
        1    # x x x x x x
        2    # x x x x x x
        3    # x x x x x x
        4    - # x x x x x   <- Stair [0] = 4
        5      # x x x x x
        6      # x x x x x
        7      # x x x x x
        8      - # x x x x   <- Stair [1] = 8
        9        # x x x x
       10        # x x x x
       11        # x x x x
       12        - # x x x   <- Stair [2] = 12
       13          # x x x
       14          - # x x   <- Stair [3] = 14
       15            - # x   <- Stair [4] = 15
       16              - #   <- Stair [5] = 16
                         -   <- Stair [6] = 17

    Suppose npiv = 3, and no columns fall below tol:

        0    R r r r r r r
        1    h R r r r r r
        2    h h R r r r r
        3    h h h C c c c
        4    - h h h C c c   <- Stair [0] = 4
        5      h h h h C c
        6      h h h h h C
        7      h h h h h h
        8      - h h h h h   <- Stair [1] = 8
        9        h h h h h
       10        h h h h h
       11        h h h h h
       12        - h h h h   <- Stair [2] = 12
       13          h h h h
       14          - h h h   <- Stair [3] = 14
       15            - h h   <- Stair [4] = 15
       16              - h   <- Stair [5] = 16
                         -   <- Stair [6] = 17

    where r is an entry in the R block, c is an entry in the C (contribution)
    block, and h is an entry in the H block (Householder vectors).  Diagonal
    entries are capitalized.

    Suppose the 2nd column has a norm <= tol; the result is a "squeezed" R:

        0    R r r r r r r   <- Stair [1] = 0 to denote a dead pivot column
        1    h - R r r r r
        2    h - h C c c c
        3    h - h h C c c
        4    - - h h h C c   <- Stair [0] = 4
        5      - h h h h C
        6      - h h h h h
        7      - h h h h h
        8      - h h h h h
        9        h h h h h
       10        h h h h h
       11        h h h h h
       12        - h h h h   <- Stair [2] = 12
       13          h h h h
       14          - h h h   <- Stair [3] = 14
       15            - h h   <- Stair [4] = 15
       16              - h   <- Stair [5] = 16
                         -   <- Stair [6] = 17

    where "diagonal" entries are capitalized.  The 2nd H vector is missing
    (it is H2 = I, identity).  The 2nd column of R is not deleted.  The
    contribution block is always triangular, but the first npiv columns of
    the R can become "staggered".  Columns npiv to n-1 in the R block are
    always the same length.

    If columns are found "dead", the staircase may be updated.  Stair[k] is
    set to zero if k is dead.  Also, Stair[k] is increased, if necessary, to
    ensure that R and C reside within the staircase.  For example:

        0 0 0
        0 0 x

    with npiv = 2 has a Stair = [ 0 1 2 ] on output, to reflect the C block:

        - C c
        - - C

    A tol of zero means that any nonzero norm (however small) is accepted;
    only exact zero columns are flagged as dead.  A negative tol means that
    the norms are ignored; a column is never flagged dead.  The default tol
    is set elsewhere as 20 * (m+1) * eps * max column 2-norm of A.

    LAPACK's dlarf* routines are used to construct and apply the Householder
    reflections.  The panel size (block size) is provided as an input
    parameter, which defines the number of Householder vectors in a panel.
    However, when the front is small (or when the remaining part
    of a front is small) the block size is increased to include the entire
    front.  "Small" is defined, below, as fronts with fewer than 5000 entries.

    NOTE: this function does not check its inputs.  If the caller runs out of
    memory and passes NULL pointers, this function will segfault.
*/

#include "spqr_subset.hpp"
#include "iostream"

#define SMALL 5000
#define MINCHUNK 4
#define MINCHUNK_RATIO 4

// =============================================================================
// === spqr_private_house ======================================================
// =============================================================================

// Construct a Householder reflection H = I - tau * v * v' such that H*x is
//  reduced to zero except for the first element.  Returns X [0] = the first
//  entry of H*x, and X [1:n-1] = the Householder vector V [1:n-1], where
//  V [0] = 1.  If X [1:n-1] is zero, then the H=I (tau = 0) is returned,
//  and V [1:n-1] is all zero.  In MATLAB (1-based indexing), ignoring the
//  rescaling done in dlarfg/zlarfg, this function computes the following:

/*
    function [x, tau] = house (x)
    n = length (x) ;
    beta = norm (x) ;
    if (x (1) > 0)
        beta = -beta ;
    end
    tau = (beta - x (1)) / beta ;
    x (2:n) = x (2:n) / (x (1) - beta) ;
    x (1) = beta ;
*/

//  Note that for the complex case, the reflection must be applied as H'*x,
//  which requires that tau be conjugated in spqr_private_apply1.
//
//  This function performs about 3*n+2 flops

inline double spqr_private_larfg (Int n, double *X, cholmod_common *cc)
{
    double tau = 0 ;
    BLAS_INT N = n, one = 1 ;
    if (CHECK_BLAS_INT && !EQ (N,n))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_DLARFG (&N, X, X + 1, &one, &tau) ;
    }
    return (tau) ;
}


inline Complex spqr_private_larfg (Int n, Complex *X, cholmod_common *cc)
{
    Complex tau = 0 ;
    BLAS_INT N = n, one = 1 ;
    if (CHECK_BLAS_INT && !EQ (N,n))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_ZLARFG (&N, X, X + 1, &one, &tau) ;
    }
    return (tau) ;
}


template <typename Entry> Entry spqr_private_house  // returns tau
(
    // inputs, not modified
    Int n,

    // input/output
    Entry *X,           // size n

    cholmod_common *cc
)
{
    return (spqr_private_larfg (n, X, cc)) ;
}


// =============================================================================
// === spqr_private_apply1 =====================================================
// =============================================================================

// Apply a single Householder reflection; C = C - tau * v * v' * C.  The
// algorithm used by dlarf is:

/*
    function C = apply1 (C, v, tau)
    w = C'*v ;
    C = C - tau*v*w' ;
*/

//  For the complex case, we need to apply H', which requires that tau be
//  conjugated.
//
//  If applied to a single column, this function performs 2*n-1 flops to
//  compute w, and 2*n+1 to apply it to C, for a total of 4*n flops.

inline void spqr_private_larf (Int m, Int n, double *V, double tau,
    double *C, Int ldc, double *W, cholmod_common *cc)
{
    BLAS_INT M = m, N = n, LDC = ldc, one = 1 ;
    char left = 'L' ;
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDC,ldc)))
    {
        cc->blas_ok = FALSE ;
        
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_DLARF (&left, &M, &N, V, &one, &tau, C, &LDC, W) ;
    }
}

inline void spqr_private_larf (Int m, Int n, Complex *V, Complex tau,
    Complex *C, Int ldc, Complex *W, cholmod_common *cc)
{
    BLAS_INT M = m, N = n, LDC = ldc, one = 1 ;
    char left = 'L' ;
    Complex conj_tau = spqr_conj (tau) ;
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDC,ldc)))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_ZLARF (&left, &M, &N, V, &one, &conj_tau, C, &LDC, W) ;
    }
}


template <typename Entry> void spqr_private_apply1
(
    // inputs, not modified
    Int m,              // C is m-by-n
    Int n,
    Int ldc,            // leading dimension of C
    Entry *V,           // size m, Householder vector V
    Entry tau,          // Householder coefficient

    // input/output
    Entry *C,           // size m-by-n

    // workspace, not defined on input or output
    Entry *W,           // size n

    cholmod_common *cc
)
{
    Entry vsave ;
    if (m <= 0 || n <= 0)
    {
        return ;        // nothing to do
    }
    vsave = V [0] ;     // temporarily restore unit diagonal of V
    V [0] = 1 ;
    spqr_private_larf (m, n, V, tau, C, ldc, W, cc) ;
    V [0] = vsave ;     // restore V [0]
}


// =============================================================================
// === spqr_front ==============================================================
// =============================================================================

// Factorize a front F into a sequence of Householder vectors H, and an upper
// trapezoidal matrix R.  R may be a squeezed upper trapezoidal matrix if any
// of the leading npiv columns are linearly dependent.  Returns the row index
// rank that indicates the first entry in C, which is F (rank,npiv), or 0
// on error.

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
                        // for each k = 0:n-1, and remain zero on output.
    char *Rdead,        // size npiv; all zero on input.  If k is dead,
                        // Rdead [k] is set to 1

    // output, not defined on input
    Entry *Tau,         // size n, Householder coefficients

    // workspace, undefined on input and output
    Entry *W,           // size b*n, where b = min (fchunk,n,m)

    // input/output
    double *wscale,
    double *wssq,

    cholmod_common *cc  // for cc->hypotenuse function
)
{
//		std::cout << "**************spqr_front dumping started****************" << std::endl;
//		std::cout << "m: " << m << " n: " << n << " npiv: " << npiv << " tol: " << tol
//				<< " ntol: " << ntol << " fchunk: " << fchunk << std::endl;
//		for (int i=0; i<m; i++) {
//			for (int j=0; j<n; j++)
//				std::cout << F[j*m+i] << "\t";
//			std::cout << std::endl;
//		}
//		std::cout << "**************spqr_front duming finished****************" << std::endl;

    Entry tau ;
    double wk ;
    Entry *V ;
    Int k, t, g, g1, nv, k1, k2, i, t0, vzeros, mleft, nleft, vsize, minchunk,
        rank ;

    // NOTE: inputs are not checked for NULL (except if debugging enabled)

    ASSERT (F != NULL) ;
    ASSERT (Stair != NULL) ;
    ASSERT (Rdead != NULL) ;
    ASSERT (Tau != NULL) ;
    ASSERT (W != NULL) ;
    ASSERT (m >= 0 && n >= 0) ;

    npiv = MAX (0, npiv) ;  // npiv must be between 0 and n
    npiv = MIN (n, npiv) ;

    g1 = 0 ;                // row index of first queued-up Householder
    k1 = 0 ;                // pending Householders are in F (g1:t, k1:k2-1)
    k2 = 0 ;
    V = F ;                 // Householder vectors start here
    g = 0 ;                 // number of good Householders found
    nv = 0 ;                // number of Householder reflections queued up
    vzeros = 0 ;            // number of explicit zeros in queued-up H's
    t = 0 ;                 // staircase of current column
    fchunk = MAX (fchunk, 1) ;
    minchunk = MAX (MINCHUNK, fchunk/MINCHUNK_RATIO) ;
    rank = MIN (m,npiv) ;  // F (rank,npiv) is the first entry in C.  rank
                           // is also the number of rows in the R block,
                           // and the number of good pivot columns found.

    ntol = MIN (ntol, npiv) ;   // Note ntol can be negative, which means to
                                // not use tol at all.

    PR (("Front %ld by %ld with %ld pivots\n", m, n, npiv)) ;
    for (k = 0 ; k < n ; k++)
    {

        // ---------------------------------------------------------------------
        // reduce the kth column of F to eliminate all but "diagonal" F (g,k)
        // ---------------------------------------------------------------------

        // get the staircase for the kth column, and operate on the "diagonal"
        // F (g,k); eliminate F (g+1:t-1, k) to zero
        t0 = t ;            // t0 = staircase of column k-1
        t = Stair [k] ;     // t = staircase of this column k

        PR (("k %ld g %ld m %ld n %ld npiv %ld\n", k, g, m, n, npiv)) ;
        if (g >= m)
        {
            // F (g,k) is outside the matrix, so we're done.  If this happens
            // when k < npiv, then there is no contribution block.
            PR (("hit the wall, npiv: %ld k %ld rank %ld\n", npiv, k, rank)) ;
            for ( ; k < npiv ; k++)
            {
                Rdead [k] = 1 ;
                Stair [k] = 0 ;         // remaining pivot columns all dead
                Tau [k] = 0 ;
            }
            for ( ; k < n ; k++)
            {
                Stair [k] = m ;         // non-pivotal columns
                Tau [k] = 0 ;
            }
            ASSERT (nv == 0) ;          // there can be no pending updates
            return (rank) ;
        }

        // if t < g+1, then this column is all zero; fix staircase so that R is
        // always inside the staircase.
        t = MAX (g+1,t) ;
        Stair [k] = t ;

        // ---------------------------------------------------------------------
        // If t just grew a lot since the last t, apply H now to all of F
        // ---------------------------------------------------------------------

        // vzeros is the number of zero entries in V after including the next
        // Householder vector.  If it would exceed 50% of the size of V,
        // apply the pending Householder reflections now, but only if
        // enough vectors have accumulated.

        vzeros += nv * (t - t0) ;
        if (nv >= minchunk)
        {
            vsize = (nv*(nv+1))/2 + nv*(t-g1-nv) ;
            if (vzeros > MAX (16, vsize/2))
            {
                // apply pending block of Householder reflections
                PR (("(1) apply k1 %ld k2 %ld\n", k1, k2)) ;
                spqr_larftb (
                    0,                          // method 0: Left, Transpose
                    t0-g1, n-k2, nv, m, m,
                    V,                          // F (g1:t-1, k1:k1+nv-1)
                    &Tau [k1],                  // Tau (k1:k-1)
                    &F [INDEX (g1,k2,m)],       // F (g1:t-1, k2:n-1)
                    W, cc) ;                    // size nv*nv + nv*(n-k2)
                nv = 0 ;        // clear queued-up Householder reflections
                vzeros = 0 ;
            }
        }

        // ---------------------------------------------------------------------
        // find a Householder reflection that reduces column k
        // ---------------------------------------------------------------------

        tau = spqr_private_house (t-g, &F [INDEX (g,k,m)], cc) ;

        // ---------------------------------------------------------------------
        // check to see if the kth column is OK
        // ---------------------------------------------------------------------

        if (k < ntol && (wk = spqr_abs (F [INDEX (g,k,m)], cc)) <= tol)
        {
            // -----------------------------------------------------------------
            // norm (F (g:t-1, k)) is too tiny; the kth pivot column is dead
            // -----------------------------------------------------------------

            // keep track of the 2-norm of w, the dead column 2-norms
            if (wk != 0)
            {
                // see also LAPACK's dnrm2 function
                if ((*wscale) == 0)
                {
                    // this is the nonzero first entry in w
                    (*wssq) = 1 ;
                }
                if ((*wscale) < wk)
                {
                    double rr = (*wscale) / wk ;
                    (*wssq) = 1 + (*wssq) * rr * rr ;
                    (*wscale) = wk ;
                }
                else
                {
                    double rr = wk / (*wscale) ;
                    (*wssq) += rr * rr ;
                }
            }

            // zero out F (g:m-1,k) and flag it as dead
            for (i = g ; i < m ; i++)
            {
                // This is not strictly necessary.  On output, entries outside
                // the staircase are ignored.
                F [INDEX (i,k,m)] = 0 ;
            }
            Stair [k] = 0 ;
            Tau [k] = 0 ;
            Rdead [k] = 1 ;

            if (nv > 0)
            {
                // apply pending block of Householder reflections
                PR (("(2) apply k1 %ld k2 %ld\n", k1, k2)) ;
                spqr_larftb (
                    0,                          // method 0: Left, Transpose
                    t0-g1, n-k2, nv, m, m,
                    V,                          // F (g1:t-1, k1:k1+nv-1)
                    &Tau [k1],                  // Tau (k1:k-1)
                    &F [INDEX (g1,k2,m)],       // F (g1:t-1, k2:n-1)
                    W, cc) ;                    // size nv*nv + nv*(n-k2)
                nv = 0 ;        // clear queued-up Householder reflections
                vzeros = 0 ;
            }

        }
        else
        {
            // -----------------------------------------------------------------
            // one more good pivot column found
            // -----------------------------------------------------------------

            Tau [k] = tau ;             // save the Householder coefficient
            if (nv == 0)
            {
                // start the queue of pending Householder updates, and define
                // the current panel as k1:k2-1
                g1 = g ;                        // first row of V
                k1 = k ;                        // first column of V
                k2 = MIN (n, k+fchunk) ;        // k2-1 is last col in panel
                V = &F [INDEX (g1,k1,m)] ;      // pending V starts here

                // check for switch to unblocked code
                mleft = m-g1 ;                  // number of rows left
                nleft = n-k1 ;                  // number of columns left
                if (mleft * (nleft-(fchunk+4)) < SMALL || mleft <= fchunk/2
                    || fchunk <= 1)
                {
                    // remaining matrix is small; switch to unblocked code by
                    // including the rest of the matrix in the panel.  Always
                    // use unblocked code if fchunk <= 1.
                    k2 = n ;
                }
            }
            nv++ ;  // one more pending update; V is F (g1:t-1, k1:k1+nv-1)

            // -----------------------------------------------------------------
            // keep track of "pure" flops, for performance testing only
            // -----------------------------------------------------------------

            // The Householder vector is of length t-g, including the unit
            // diagonal, and takes 3*(t-g) flops to compute.  It will be
            // applied as a block, but compute the "pure" flops by assuming
            // that this single Householder vector is computed and then applied
            // just by itself to the rest of the frontal matrix (columns
            // k+1:n-1, or n-k-1 columns).  Applying the Householder reflection
            // to just one column takes 4*(t-g) flops.  This computation only
            // works if TBB is disabled, merely because it uses a global
            // variable to keep track of the flop count.  If TBB is used, this
            // computation may result in a race condition; it is disabled in
            // that case.

            FLOP_COUNT ((t-g) * (3 + 4 * (n-k-1))) ;

            // -----------------------------------------------------------------
            // apply the kth Householder reflection to the current panel
            // -----------------------------------------------------------------

            // F (g:t-1, k+1:k2-1) -= v * tau * v' * F (g:t-1, k+1:k2-1), where
            // v is stored in F (g:t-1,k).  This applies just one reflection
            // to the current panel.
            PR (("apply 1: k %ld\n", k)) ;
            spqr_private_apply1 (t-g, k2-k-1, m, &F [INDEX (g,k,m)], tau,
                &F [INDEX (g,k+1,m)], W, cc) ;

            g++ ;   // one more pivot found

            // -----------------------------------------------------------------
            // apply the Householder reflections if end of panel reached
            // -----------------------------------------------------------------

            if (k == k2-1 || g == m)            // or if last pivot is found
            {
                // apply pending block of Householder reflections
                PR (("(3) apply k1 %ld k2 %ld\n", k1, k2)) ;
                spqr_larftb (
                    0,                          // method 0: Left, Transpose
                    t-g1, n-k2, nv, m, m,
                    V,                          // F (g1:t-1, k1:k1+nv-1)
                    &Tau [k1],                  // Tau (k1:k2-1)
                    &F [INDEX (g1,k2,m)],       // F (g1:t-1, k2:n-1)
                    W, cc) ;                    // size nv*nv + nv*(n-k2)
                nv = 0 ;        // clear queued-up Householder reflections
                vzeros = 0 ;
            }
        }

        // ---------------------------------------------------------------------
        // determine the rank of the pivot columns
        // ---------------------------------------------------------------------

        if (k == npiv-1)
        {
            // the rank is the number of good columns found in the first
            // npiv columns.  It is also the number of rows in the R block.
            // F (rank,npiv) is first entry in the C block.
            rank = g ;
            PR (("rank of Front pivcols: %ld\n", rank)) ;
        }
    }

    if (CHECK_BLAS_INT && !cc->blas_ok)
    {
    	  // This cannot occur if the BLAS_INT and the Int are the same integer.
        // In that case, CHECK_BLAS_INT is FALSE at compile-time, and the
        // compiler will then remove this as dead code.
        ERROR (CHOLMOD_INVALID, "problem too large for the BLAS") ;
        return (0) ;
    }

    return (rank) ;
}


// =============================================================================

template Int spqr_front <double>
(
    // input, not modified
    Int m,              // F is m-by-n with leading dimension m
    Int n,
    Int npiv,           // number of pivot columns
    double tol,         // a column is flagged as dead if its norm is <= tol
    Int ntol,           // apply tol only to first ntol pivot columns
    Int fchunk,         // block size for compact WY Householder reflections,
                        // treated as 1 if fchunk <= 1 (in which case the
                        // unblocked code is used).

    // input/output
    double *F,          // frontal matrix F of size m-by-n
    Int *Stair,         // size n, entries F (Stair[k]:m-1, k) are all zero,
                        // and remain zero on output.
    char *Rdead,        // size npiv; all zero on input.  If k is dead,
                        // Rdead [k] is set to 1

    // output, not defined on input
    double *Tau,        // size n, Householder coefficients

    // workspace, undefined on input and output
    double *W,          // size b*n, where b = min (fchunk,n,m)

    // input/output
    double *wscale,
    double *wssq,

    cholmod_common *cc  // for cc->hypotenuse function
) ;

// =============================================================================

template Int spqr_front <Complex>
(
    // input, not modified
    Int m,              // F is m-by-n with leading dimension m
    Int n,
    Int npiv,           // number of pivot columns
    double tol,         // a column is flagged as dead if its norm is <= tol
    Int ntol,           // apply tol only to first ntol pivot columns
    Int fchunk,         // block size for compact WY Householder reflections,
                        // treated as 1 if fchunk <= 1 (in which case the
                        // unblocked code is used). 

    // input/output
    Complex *F,         // frontal matrix F of size m-by-n
    Int *Stair,         // size n, entries F (Stair[k]:m-1, k) are all zero,
                        // and remain zero on output.
    char *Rdead,        // size npiv; all zero on input.  If k is dead,
                        // Rdead [k] is set to 1

    // output, not defined on input
    Complex *Tau,       // size n, Householder coefficients

    // workspace, undefined on input and output
    Complex *W,         // size b*n, where b = min (fchunk,n,m)

    // input/output
    double *wscale,
    double *wssq,

    cholmod_common *cc  // for cc->hypotenuse function
) ;
