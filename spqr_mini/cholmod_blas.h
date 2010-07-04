/* ========================================================================== */
/* === Include/cholmod_blas.h =============================================== */
/* ========================================================================== */

/* -----------------------------------------------------------------------------
 * CHOLMOD/Include/cholmod_blas.h.
 * Copyright (C) 2005-2006, Univ. of Florida.  Author: Timothy A. Davis
 * CHOLMOD/Include/cholmod_blas.h is licensed under Version 2.1 of the GNU
 * Lesser General Public License.  See lesser.txt for a text of the license.
 * CHOLMOD is also available under other licenses; contact authors for details.
 * http://www.cise.ufl.edu/research/sparse
 * -------------------------------------------------------------------------- */

/* This does not need to be included in the user's program. */

#ifndef CHOLMOD_BLAS_H
#define CHOLMOD_BLAS_H

/* ========================================================================== */
/* === Architecture ========================================================= */
/* ========================================================================== */

#if defined (__sun) || defined (MSOL2) || defined (ARCH_SOL2)
#define CHOLMOD_SOL2
#define CHOLMOD_ARCHITECTURE "Sun Solaris"

#elif defined (__sgi) || defined (MSGI) || defined (ARCH_SGI)
#define CHOLMOD_SGI
#define CHOLMOD_ARCHITECTURE "SGI Irix"

#elif defined (__linux) || defined (MGLNX86) || defined (ARCH_GLNX86)
#define CHOLMOD_LINUX
#define CHOLMOD_ARCHITECTURE "Linux"

#elif defined (__APPLE__)
#define CHOLMOD_MAC
#define CHOLMOD_ARCHITECTURE "Mac"

#elif defined (_AIX) || defined (MIBM_RS) || defined (ARCH_IBM_RS)
#define CHOLMOD_AIX
#define CHOLMOD_ARCHITECTURE "IBM AIX"
/* recent reports from IBM AIX seem to indicate that this is not needed: */
/* #define BLAS_NO_UNDERSCORE */

#elif defined (__alpha) || defined (MALPHA) || defined (ARCH_ALPHA)
#define CHOLMOD_ALPHA
#define CHOLMOD_ARCHITECTURE "Compaq Alpha"

#elif defined (_WIN32) || defined (WIN32) || defined (_WIN64) || defined (WIN64)
#if defined (__MINGW32__) || defined (__MINGW32__)
#define CHOLMOD_MINGW
#elif defined (__CYGWIN32__) || defined (__CYGWIN32__)
#define CHOLMOD_CYGWIN
#else
#define CHOLMOD_WINDOWS
#define BLAS_NO_UNDERSCORE
#endif
#define CHOLMOD_ARCHITECTURE "Microsoft Windows"

#elif defined (__hppa) || defined (__hpux) || defined (MHPUX) || defined (ARCH_HPUX)
#define CHOLMOD_HP
#define CHOLMOD_ARCHITECTURE "HP Unix"
#define BLAS_NO_UNDERSCORE

#elif defined (__hp700) || defined (MHP700) || defined (ARCH_HP700)
#define CHOLMOD_HP
#define CHOLMOD_ARCHITECTURE "HP 700 Unix"
#define BLAS_NO_UNDERSCORE

#else
/* If the architecture is unknown, and you call the BLAS, you may need to */
/* define BLAS_BY_VALUE, BLAS_NO_UNDERSCORE, and/or BLAS_CHAR_ARG yourself. */
#define CHOLMOD_ARCHITECTURE "unknown"
#endif

/* ========================================================================== */
/* === BLAS and LAPACK names ================================================ */
/* ========================================================================== */

/* Prototypes for the various versions of the BLAS.  */

/* Determine if the 64-bit Sun Performance BLAS is to be used */
#if defined(CHOLMOD_SOL2) && !defined(NSUNPERF) && defined(BLAS64)
#define SUN64
#endif

#ifdef SUN64

#define BLAS_DTRSV dtrsv_64_
#define BLAS_DGEMV dgemv_64_
#define BLAS_DTRSM dtrsm_64_
#define BLAS_DGEMM dgemm_64_
#define BLAS_DSYRK dsyrk_64_
#define BLAS_DGER  dger_64_
#define BLAS_DSCAL dscal_64_
#define LAPACK_DPOTRF dpotrf_64_

#define BLAS_ZTRSV ztrsv_64_
#define BLAS_ZGEMV zgemv_64_
#define BLAS_ZTRSM ztrsm_64_
#define BLAS_ZGEMM zgemm_64_
#define BLAS_ZHERK zherk_64_
#define BLAS_ZGER  zgeru_64_
#define BLAS_ZSCAL zscal_64_
#define LAPACK_ZPOTRF zpotrf_64_

#elif defined (BLAS_NO_UNDERSCORE)

#define BLAS_DTRSV dtrsv
#define BLAS_DGEMV dgemv
#define BLAS_DTRSM dtrsm
#define BLAS_DGEMM dgemm
#define BLAS_DSYRK dsyrk
#define BLAS_DGER  dger
#define BLAS_DSCAL dscal
#define LAPACK_DPOTRF dpotrf

#define BLAS_ZTRSV ztrsv
#define BLAS_ZGEMV zgemv
#define BLAS_ZTRSM ztrsm
#define BLAS_ZGEMM zgemm
#define BLAS_ZHERK zherk
#define BLAS_ZGER  zgeru
#define BLAS_ZSCAL zscal
#define LAPACK_ZPOTRF zpotrf

#else

#define BLAS_DTRSV dtrsv_
#define BLAS_DGEMV dgemv_
#define BLAS_DTRSM dtrsm_
#define BLAS_DGEMM dgemm_
#define BLAS_DSYRK dsyrk_
#define BLAS_DGER  dger_
#define BLAS_DSCAL dscal_
#define LAPACK_DPOTRF dpotrf_

#define BLAS_ZTRSV ztrsv_
#define BLAS_ZGEMV zgemv_
#define BLAS_ZTRSM ztrsm_
#define BLAS_ZGEMM zgemm_
#define BLAS_ZHERK zherk_
#define BLAS_ZGER  zgeru_
#define BLAS_ZSCAL zscal_
#define LAPACK_ZPOTRF zpotrf_

#endif

/* ========================================================================== */
/* === BLAS and LAPACK integer arguments ==================================== */
/* ========================================================================== */

/* Compile CHOLMOD, UMFPACK, and SPQR with -DBLAS64 if you have a BLAS that
 * uses 64-bit integers */

#if defined (LONGBLAS) || defined (BLAS64)
#define BLAS_INT UF_long
#else
#define BLAS_INT int
#endif

/* If the BLAS integer is smaller than the basic CHOLMOD integer, then we need
 * to check for integer overflow when converting from Int to BLAS_INT.  If
 * any integer overflows, the externally-defined BLAS_OK variable is
 * set to FALSE.  BLAS_OK should be set to TRUE before calling any
 * BLAS_* macro.
 */

#define CHECK_BLAS_INT (sizeof (BLAS_INT) < sizeof (Int))
#define EQ(K,k) (((BLAS_INT) K) == ((Int) k))

/* ========================================================================== */
/* === BLAS and LAPACK prototypes and macros ================================ */
/* ========================================================================== */

void BLAS_DGEMV (char *trans, BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *A, BLAS_INT *lda, double *X, BLAS_INT *incx, double *beta,
	double *Y, BLAS_INT *incy) ;

#define BLAS_dgemv(trans,m,n,alpha,A,lda,X,incx,beta,Y,incy) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
        EQ (INCX,incx) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DGEMV (trans, &M, &N, alpha, A, &LDA, X, &INCX, beta, Y, &INCY) ; \
    } \
}

void BLAS_ZGEMV (char *trans, BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *A, BLAS_INT *lda, double *X, BLAS_INT *incx, double *beta,
	double *Y, BLAS_INT *incy) ;

#define BLAS_zgemv(trans,m,n,alpha,A,lda,X,incx,beta,Y,incy) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
        EQ (INCX,incx) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZGEMV (trans, &M, &N, alpha, A, &LDA, X, &INCX, beta, Y, &INCY) ; \
    } \
}

void BLAS_DTRSV (char *uplo, char *trans, char *diag, BLAS_INT *n, double *A,
	BLAS_INT *lda, double *X, BLAS_INT *incx) ;

#define BLAS_dtrsv(uplo,trans,diag,n,A,lda,X,incx) \
{ \
    BLAS_INT N = n, LDA = lda, INCX = incx ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DTRSV (uplo, trans, diag, &N, A, &LDA, X, &INCX) ; \
    } \
}

void BLAS_ZTRSV (char *uplo, char *trans, char *diag, BLAS_INT *n, double *A,
	BLAS_INT *lda, double *X, BLAS_INT *incx) ;

#define BLAS_ztrsv(uplo,trans,diag,n,A,lda,X,incx) \
{ \
    BLAS_INT N = n, LDA = lda, INCX = incx ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (LDA,lda) && EQ (INCX,incx))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZTRSV (uplo, trans, diag, &N, A, &LDA, X, &INCX) ; \
    } \
}

void BLAS_DTRSM (char *side, char *uplo, char *transa, char *diag, BLAS_INT *m,
	BLAS_INT *n, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb) ;

#define BLAS_dtrsm(side,uplo,transa,diag,m,n,alpha,A,lda,B,ldb) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, LDB = ldb ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
        EQ (LDB,ldb))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DTRSM (side, uplo, transa, diag, &M, &N, alpha, A, &LDA, B, &LDB);\
    } \
}

void BLAS_ZTRSM (char *side, char *uplo, char *transa, char *diag, BLAS_INT *m,
	BLAS_INT *n, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb) ;

#define BLAS_ztrsm(side,uplo,transa,diag,m,n,alpha,A,lda,B,ldb) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, LDB = ldb ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
        EQ (LDB,ldb))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZTRSM (side, uplo, transa, diag, &M, &N, alpha, A, &LDA, B, &LDB);\
    } \
}

void BLAS_DGEMM (char *transa, char *transb, BLAS_INT *m, BLAS_INT *n,
	BLAS_INT *k, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb, double *beta, double *C, BLAS_INT *ldc) ;

#define BLAS_dgemm(transa,transb,m,n,k,alpha,A,lda,B,ldb,beta,C,ldc) \
{ \
    BLAS_INT M = m, N = n, K = k, LDA = lda, LDB = ldb, LDC = ldc ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (K,k) && \
        EQ (LDA,lda) && EQ (LDB,ldb) && EQ (LDC,ldc))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DGEMM (transa, transb, &M, &N, &K, alpha, A, &LDA, B, &LDB, beta, \
	    C, &LDC) ; \
    } \
}

void BLAS_ZGEMM (char *transa, char *transb, BLAS_INT *m, BLAS_INT *n,
	BLAS_INT *k, double *alpha, double *A, BLAS_INT *lda, double *B,
	BLAS_INT *ldb, double *beta, double *C, BLAS_INT *ldc) ;

#define BLAS_zgemm(transa,transb,m,n,k,alpha,A,lda,B,ldb,beta,C,ldc) \
{ \
    BLAS_INT M = m, N = n, K = k, LDA = lda, LDB = ldb, LDC = ldc ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (K,k) && \
        EQ (LDA,lda) && EQ (LDB,ldb) && EQ (LDC,ldc))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZGEMM (transa, transb, &M, &N, &K, alpha, A, &LDA, B, &LDB, beta, \
	    C, &LDC) ; \
    } \
}

void BLAS_DSYRK (char *uplo, char *trans, BLAS_INT *n, BLAS_INT *k,
	double *alpha, double *A, BLAS_INT *lda, double *beta, double *C,
	BLAS_INT *ldc) ;

#define BLAS_dsyrk(uplo,trans,n,k,alpha,A,lda,beta,C,ldc) \
{ \
    BLAS_INT N = n, K = k, LDA = lda, LDC = ldc ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (K,k) && EQ (LDA,lda) && \
        EQ (LDC,ldc))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DSYRK (uplo, trans, &N, &K, alpha, A, &LDA, beta, C, &LDC) ; \
    } \
} \

void BLAS_ZHERK (char *uplo, char *trans, BLAS_INT *n, BLAS_INT *k,
	double *alpha, double *A, BLAS_INT *lda, double *beta, double *C,
	BLAS_INT *ldc) ;

#define BLAS_zherk(uplo,trans,n,k,alpha,A,lda,beta,C,ldc) \
{ \
    BLAS_INT N = n, K = k, LDA = lda, LDC = ldc ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (K,k) && EQ (LDA,lda) && \
        EQ (LDC,ldc))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZHERK (uplo, trans, &N, &K, alpha, A, &LDA, beta, C, &LDC) ; \
    } \
} \

void LAPACK_DPOTRF (char *uplo, BLAS_INT *n, double *A, BLAS_INT *lda,
	BLAS_INT *info) ;

#define LAPACK_dpotrf(uplo,n,A,lda,info) \
{ \
    BLAS_INT N = n, LDA = lda, INFO = 1 ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (LDA,lda))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	LAPACK_DPOTRF (uplo, &N, A, &LDA, &INFO) ; \
    } \
    info = INFO ; \
}

void LAPACK_ZPOTRF (char *uplo, BLAS_INT *n, double *A, BLAS_INT *lda,
	BLAS_INT *info) ;

#define LAPACK_zpotrf(uplo,n,A,lda,info) \
{ \
    BLAS_INT N = n, LDA = lda, INFO = 1 ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (LDA,lda))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	LAPACK_ZPOTRF (uplo, &N, A, &LDA, &INFO) ; \
    } \
    info = INFO ; \
}

/* ========================================================================== */

void BLAS_DSCAL (BLAS_INT *n, double *alpha, double *Y, BLAS_INT *incy) ;

#define BLAS_dscal(n,alpha,Y,incy) \
{ \
    BLAS_INT N = n, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DSCAL (&N, alpha, Y, &INCY) ; \
    } \
}

void BLAS_ZSCAL (BLAS_INT *n, double *alpha, double *Y, BLAS_INT *incy) ;

#define BLAS_zscal(n,alpha,Y,incy) \
{ \
    BLAS_INT N = n, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (N,n) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZSCAL (&N, alpha, Y, &INCY) ; \
    } \
}

void BLAS_DGER (BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *X, BLAS_INT *incx, double *Y, BLAS_INT *incy,
	double *A, BLAS_INT *lda) ;

#define BLAS_dger(m,n,alpha,X,incx,Y,incy,A,lda) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
          EQ (INCX,incx) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_DGER (&M, &N, alpha, X, &INCX, Y, &INCY, A, &LDA) ; \
    } \
}

void BLAS_ZGER (BLAS_INT *m, BLAS_INT *n, double *alpha,
	double *X, BLAS_INT *incx, double *Y, BLAS_INT *incy,
	double *A, BLAS_INT *lda) ;

#define BLAS_zgeru(m,n,alpha,X,incx,Y,incy,A,lda) \
{ \
    BLAS_INT M = m, N = n, LDA = lda, INCX = incx, INCY = incy ; \
    if (CHECK_BLAS_INT && !(EQ (M,m) && EQ (N,n) && EQ (LDA,lda) && \
          EQ (INCX,incx) && EQ (INCY,incy))) \
    { \
	BLAS_OK = FALSE ; \
    } \
    if (!CHECK_BLAS_INT || BLAS_OK) \
    { \
	BLAS_ZGER (&M, &N, alpha, X, &INCX, Y, &INCY, A, &LDA) ; \
    } \
}

#endif
