// =============================================================================
// === spqr_larftb =============================================================
// =============================================================================

// Apply a set of Householder reflections to a matrix.  Given the vectors
// V and coefficients Tau, construct the matrix T and then apply the updates.
// In MATLAB (1-based indexing), this function computes the following:

/*
    function C = larftb (C, V, Tau, method)
    [v k] = size (V) ;
    [m n] = size (C) ;
    % construct T for the compact WY representation
    V = tril (V,-1) + eye (v,k) ;
    T = zeros (k,k) ;
    T (1,1) = Tau (1) ;
    for j = 2:k
        tau = Tau (j) ;
        z = -tau * V (:, 1:j-1)' * V (:,j) ;
        T (1:j-1,j) = T (1:j-1,1:j-1) * z ;
        T (j,j) = tau ;
    end
    % apply the updates
    if (method == 0)
        C = C - V * T' * V' * C ;       % method 0: Left, Transpose
    elseif (method == 1)
        C = C - V * T * V' * C ;        % method 1: Left, No Transpose
    elseif (method == 2)
        C = C - C * V * T' * V' ;       % method 2: Right, Transpose
    elseif (method == 3)
        C = C - C * V * T * V' ;        % method 3: Right, No Transpose
    end
*/

#include "spqr_subset.hpp"

inline void spqr_private_larft (char direct, char storev, Int n, Int k,
    double *V, Int ldv, double *Tau, double *T, Int ldt, cholmod_common *cc)
{
    BLAS_INT N = n, K = k, LDV = ldv, LDT = ldt ;
    if (CHECK_BLAS_INT &&
        !(EQ (N,n) && EQ (K,k) && EQ (LDV,ldv) && EQ (LDT,ldt)))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_DLARFT (&direct, &storev, &N, &K, V, &LDV, Tau, T, &LDT) ;
    }
}

inline void spqr_private_larft (char direct, char storev, Int n, Int k,
    Complex *V, Int ldv, Complex *Tau, Complex *T, Int ldt, cholmod_common *cc)
{
    BLAS_INT N = n, K = k, LDV = ldv, LDT = ldt ;
    if (CHECK_BLAS_INT &&
        !(EQ (N,n) && EQ (K,k) && EQ (LDV,ldv) && EQ (LDT,ldt)))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_ZLARFT (&direct, &storev, &N, &K, V, &LDV, Tau, T, &LDT) ;
    }
}


inline void spqr_private_larfb (char side, char trans, char direct, char storev,
    Int m, Int n, Int k, double *V, Int ldv, double *T, Int ldt, double *C,
    Int ldc, double *Work, Int ldwork, cholmod_common *cc)
{
    BLAS_INT M = m, N = n, K = k, LDV = ldv, LDT = ldt, LDC = ldc,
        LDWORK = ldwork ;
    if (CHECK_BLAS_INT &&
        !(EQ (M,m) && EQ (N,n) && EQ (K,k) && EQ (LDV,ldv) &&
          EQ (LDT,ldt) && EQ (LDV,ldv) && EQ (LDWORK,ldwork)))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_DLARFB (&side, &trans, &direct, &storev, &M, &N, &K, V, &LDV,
            T, &LDT, C, &LDC, Work, &LDWORK) ;
    }
}


inline void spqr_private_larfb (char side, char trans, char direct, char storev,
    Int m, Int n, Int k, Complex *V, Int ldv, Complex *T, Int ldt, Complex *C,
    Int ldc, Complex *Work, Int ldwork, cholmod_common *cc)
{
    char tr = (trans == 'T') ? 'C' : 'N' ;      // change T to C
    BLAS_INT M = m, N = n, K = k, LDV = ldv, LDT = ldt, LDC = ldc,
        LDWORK = ldwork ;
    if (CHECK_BLAS_INT &&
        !(EQ (M,m) && EQ (N,n) && EQ (K,k) && EQ (LDV,ldv) &&
          EQ (LDT,ldt) && EQ (LDV,ldv) && EQ (LDWORK,ldwork)))
    {
        cc->blas_ok = FALSE ;
    }
    if (!CHECK_BLAS_INT || cc->blas_ok)
    {
        LAPACK_ZLARFB (&side, &tr, &direct, &storev, &M, &N, &K, V, &LDV,
            T, &LDT, C, &LDC, Work, &LDWORK) ;
    }
}


// =============================================================================

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
)
{
    Entry *T, *Work ;

    // -------------------------------------------------------------------------
    // check inputs and split up workspace
    // -------------------------------------------------------------------------

    if (m <= 0 || n <= 0 || k <= 0)
    {
        return ; // nothing to do
    }

    T = W ;             // triangular k-by-k matrix for block reflector
    Work = W + k*k ;    // workspace of size n*k or m*k for larfb

    // -------------------------------------------------------------------------
    // construct and apply the k-by-k upper triangular matrix T
    // -------------------------------------------------------------------------

    // larft and larfb are always used "Forward" and "Columnwise"

    if (method == SPQR_QTX)
    {
        ASSERT (m >= k) ;
        spqr_private_larft ('F', 'C', m, k, V, ldv, Tau, T, k, cc) ;
        // Left, Transpose, Forward, Columwise:
        spqr_private_larfb ('L', 'T', 'F', 'C', m, n, k, V, ldv, T, k, C, ldc,
            Work, n, cc) ;
    }
    else if (method == SPQR_QX)
    {
        ASSERT (m >= k) ;
        spqr_private_larft ('F', 'C', m, k, V, ldv, Tau, T, k, cc) ;
        // Left, No Transpose, Forward, Columwise:
        spqr_private_larfb ('L', 'N', 'F', 'C', m, n, k, V, ldv, T, k, C, ldc,
            Work, n, cc) ;
    }
    else if (method == SPQR_XQT)
    {
        ASSERT (n >= k) ;
        spqr_private_larft ('F', 'C', n, k, V, ldv, Tau, T, k, cc) ;
        // Right, Transpose, Forward, Columwise:
        spqr_private_larfb ('R', 'T', 'F', 'C', m, n, k, V, ldv, T, k, C, ldc,
            Work, m, cc) ;
    }
    else if (method == SPQR_XQ)
    {
        ASSERT (n >= k) ;
        spqr_private_larft ('F', 'C', n, k, V, ldv, Tau, T, k, cc) ;
        // Right, No Transpose, Forward, Columwise:
        spqr_private_larfb ('R', 'N', 'F', 'C', m, n, k, V, ldv, T, k, C, ldc,
            Work, m, cc) ;
    }
}

// =============================================================================

template void spqr_larftb <double>
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
    double *V,      // V is v-by-k, unit lower triangular (diag not stored)
    double *Tau,    // size k, the k Householder coefficients

    // input/output
    double *C,      // C is m-by-n, with leading dimension ldc

    // workspace, not defined on input or output
    double *W,      // for methods 0,1: size k*k + n*k
                    // for methods 2,3: size k*k + m*k
    cholmod_common *cc
) ;

// =============================================================================

template void spqr_larftb <Complex>
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
    Complex *V,     // V is v-by-k, unit lower triangular (diag not stored)
    Complex *Tau,   // size k, the k Householder coefficients

    // input/output
    Complex *C,     // C is m-by-n, with leading dimension ldc

    // workspace, not defined on input or output
    Complex *W,     // for methods 0,1: size k*k + n*k
                    // for methods 2,3: size k*k + m*k
    cholmod_common *cc
) ;
