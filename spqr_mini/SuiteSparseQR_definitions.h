/* ========================================================================== */
/* === SuiteSparseQR_definitions.h ========================================== */
/* ========================================================================== */

/* Core definitions for both C and C++ programs. */

#ifndef SUITESPARSEQR_DEFINITIONS_H
#define SUITESPARSEQR_DEFINITIONS_H

/* ordering options */
#define SPQR_ORDERING_FIXED 0
#define SPQR_ORDERING_NATURAL 1
#define SPQR_ORDERING_COLAMD 2
#define SPQR_ORDERING_GIVEN 3       /* only used for C/C++ interface */
#define SPQR_ORDERING_CHOLMOD 4     /* CHOLMOD best-effort (COLAMD, METIS,...)*/
#define SPQR_ORDERING_AMD 5         /* AMD(A'*A) */
#define SPQR_ORDERING_METIS 6       /* metis(A'*A) */
#define SPQR_ORDERING_DEFAULT 7     /* SuiteSparseQR default ordering */
#define SPQR_ORDERING_BEST 8        /* try COLAMD, AMD, and METIS; pick best */
#define SPQR_ORDERING_BESTAMD 9     /* try COLAMD and AMD; pick best */

/* Let [m n] = size of the matrix after pruning singletons.  The default
 * ordering strategy is to use COLAMD if m <= 2*n.  Otherwise, AMD(A'A) is
 * tried.  If there is a high fill-in with AMD then try METIS(A'A) and take
 * the best of AMD and METIS.  METIS is not tried if it isn't installed. */

/* tol options */
#define SPQR_DEFAULT_TOL (-2)       /* if tol <= -2, the default tol is used */
#define SPQR_NO_TOL (-1)            /* if -2 < tol < 0, then no tol is used */

/* for qmult, method can be 0,1,2,3: */
#define SPQR_QTX 0
#define SPQR_QX  1
#define SPQR_XQT 2
#define SPQR_XQ  3

/* system can be 0,1,2,3:  Given Q*R=A*E from SuiteSparseQR_factorize: */
#define SPQR_RX_EQUALS_B    0       /* solve R*X=B      or X = R\B          */
#define SPQR_RETX_EQUALS_B  1       /* solve R*E'*X=B   or X = E*(R\B)      */
#define SPQR_RTX_EQUALS_B   2       /* solve R'*X=B     or X = R'\B         */
#define SPQR_RTX_EQUALS_ETB 3       /* solve R'*X=E'*B  or X = R'\(E'*B)    */

/* ========================================================================== */
/* === SuiteSparseQR version ================================================ */
/* ========================================================================== */

/*
   All versions of SuiteSparseQR will include the following definitions.
   As an example, to test if the version you are using is 1.3 or later:
  
        if (SPQR_VERSION >= SPQR_VER_CODE (1,3)) ...
  
   This also works during compile-time:
  
        #if SPQR_VERSION >= SPQR_VER_CODE (1,3)
            printf ("This is version 1.3 or later\n") ;
        #else
            printf ("This is version is earlier than 1.3\n") ;
        #endif
 */

#define SPQR_DATE "Nov 30, 2009"
#define SPQR_VER_CODE(main,sub) ((main) * 1000 + (sub))
#define SPQR_MAIN_VERSION 1
#define SPQR_SUB_VERSION 2
#define SPQR_SUBSUB_VERSION 0
#define SPQR_VERSION SPQR_VER_CODE(SPQR_MAIN_VERSION,SPQR_SUB_VERSION)

#endif
