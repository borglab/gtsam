// =============================================================================
// === SuiteSparseQR.hpp =======================================================
// =============================================================================

// User include file for C++ programs.

#ifndef SUITESPARSEQR_H
#define SUITESPARSEQR_H

// -----------------------------------------------------------------------------
// include files
// -----------------------------------------------------------------------------

#include "cholmod.h"
#include "UFconfig.h"
#include "SuiteSparseQR_definitions.h"

// =============================================================================
// === spqr_symbolic ===========================================================
// =============================================================================

// The contents of this object do not change during numeric factorization.  The
// Symbolic object depends only on the pattern of the input matrix, and not its
// values.  These contents also do not change with column pivoting for rank
// detection.  This makes parallelism easier to manage, since all threads can
// have access to this object without synchronization.
//
// The total size of the Symbolic object is (10 + 2*m + anz + 2*n + 5*nf + rnz)
// Int's, where the user's input A matrix is m-by-n with anz nonzeros, nf <=
// MIN(m,n) is the number of frontal matrices, and rnz <= nnz(R) is the number
// of column indices used to represent the supernodal form of R (one Int per
// non-pivotal column index in the leading row of each block of R).

struct spqr_symbolic
{

    // -------------------------------------------------------------------------
    // row-form of the input matrix and its permutations
    // -------------------------------------------------------------------------

    // During symbolic analysis, the nonzero pattern of S = A(P,Q) is
    // constructed, where A is the user's input matrix.  Its numerical values
    // are also constructed, but they do not become part of the Symbolic
    // object.  The matrix S is stored in row-oriented form.  The rows of S are
    // sorted according to their leftmost column index (via PLinv).  Column
    // indices in each row of S are in strictly ascending order, even though
    // the input matrix A need not be sorted.

    UF_long m, n, anz ; // S is m-by-n with anz entries

    UF_long *Sp ;       // size m+1, row pointers of S

    UF_long *Sj ;       // size anz = Sp [n], column indices of S

    UF_long *Qfill ;    // size n, fill-reducing column permutation.
                        // Qfill [k] = j if column k of A is column j of S.

    UF_long *PLinv ;    // size m, inverse row permutation that places S=A(P,Q)
                        // in increasing order of leftmost column index.
                        // PLinv [i] = k if row i of A is row k of S.

    UF_long *Sleft ;    // size n+2.  The list of rows of S whose leftmost
                        // column index is j is given by
            // Sleft [j] ... Sleft [j+1]-1.  This can be empty (that is, Sleft
            // [j] can equal Sleft [j+1]).  Sleft [n] is the number of
            // non-empty rows of S, and Sleft [n+1] == m.  That is, Sleft [n]
            // ... Sleft [n+1]-1 gives the empty rows of S, if any.

    // -------------------------------------------------------------------------
    // frontal matrices: pattern and tree
    // -------------------------------------------------------------------------

    // Each frontal matrix is fm-by-fn, with fnpiv pivot columns.  The fn
    // column indices are given by a set of size fnpiv pivot columns, defined
    // by Super, followed by the pattern Rj [ Rp[f] ...  Rp[f+1]-1 ].

    // The row indices of the front are not kept.  If the Householder vectors
    // are not kept, the row indices are not needed.  If the Householder
    // vectors are kept, the row indices are computed dynamically during
    // numerical factorization.

    UF_long nf ;        // number of frontal matrices; nf <= MIN (m,n)
    UF_long maxfn ;     // max # of columns in any front

    // parent, child, and childp define the row merge tree or etree (A'A)
    UF_long *Parent ;   // size nf+1
    UF_long *Child ;    // size nf+1
    UF_long *Childp ;   // size nf+2

    // The parent of a front f is Parent [f], or EMPTY if f=nf.
    // A list of children of f can be obtained in the list
    // Child [Childp [f] ... Childp [f+1]-1].

    // Node nf in the tree is a placeholder; it does not represent a frontal
    // matrix.  All roots of the frontal "tree" (may be a forest) have the
    // placeholder node nf as their parent.  Thus, the tree of nodes 0:nf is
    // truly a tree, with just one parent (node nf).

    UF_long *Super ;    // size nf+1.  Super [f] gives the first pivot column
                        // in the front F.  This refers to a column of S.  The
        // number of expected pivot columns in F is thus
        // Super [f+1] - Super [f].

    UF_long *Rp ;       // size nf+1
    UF_long *Rj ;       // size rjsize; compressed supernodal form of R

    UF_long *Post ;     // size nf+1, post ordering of frontal tree. f=Post[k]
                        // gives the kth node in the postordered tree

    UF_long rjsize ;    // size of Rj

    UF_long do_rank_detection ; // TRUE: allow for tol >= 0. FALSE: ignore tol

    // the rest depends on whether or not rank-detection is allowed:
    UF_long maxstack  ; // max stack size (sequential case)
    UF_long hisize ;    // size of Hii

    UF_long keepH ;     // TRUE if H is present

    UF_long *Hip ;      // size nf+1.  If H is kept, the row indices of frontal
                        // matrix f are in Hii [Hip [f] ... Hip [f] + Hm [f]],
        // where Hii and Hm are stored in the numeric object.

        // There is one block row of R per frontal matrix.
        // The fn column indices of R are given by Rj [Rp [f] ... Rp [f+1]-1],
        // where the first fp column indices are Super [f] ... Super [f+1]-1.
        // The remaining column indices in Rj [...] are non-pivotal, and are
        // in the range Super [f+1] to n.  The number of rows of R is at
        // most fp, but can be less if dead columns appear in the matrix.
        // The number of columns in the contribution block C is always
        // cn = fn - fp, where fn = Rp [f+1] - Rp [f].

    UF_long ntasks ;    // number of tasks in task graph
    UF_long ns ;        // number of stacks

    // -------------------------------------------------------------------------
    // the rest of the QR symbolic object is present only if ntasks > 1
    // -------------------------------------------------------------------------

    // Task tree (nodes 0:ntasks), including placeholder node
    UF_long *TaskChildp ;       // size ntasks+2
    UF_long *TaskChild ;        // size ntasks+1

    UF_long *TaskStack ;        // size ntasks+1

    // list of fronts for each task
    UF_long *TaskFront ;        // size nf+1
    UF_long *TaskFrontp  ;      // size ntasks+2

    UF_long *On_stack  ;        // size nf+1, front f is on stack On_stack [f]

    // size of each stack
    UF_long *Stack_maxstack ;   // size ns+2

} ;


// =============================================================================
// === spqr_numeric ============================================================
// =============================================================================

// The Numeric object contains the numerical values of the triangular/
// trapezoidal factor R, and optionally the Householder vectors H if they
// are kept.

template <typename Entry> struct spqr_numeric
{

    // -------------------------------------------------------------------------
    // Numeric R factor
    // -------------------------------------------------------------------------

    Entry **Rblock ;    // size nf.  R [f] is an (Entry *) pointer to the
                        // R block for front F.  It is an upper trapezoidal
                        // of size Rm(f)-by-Rn(f), but only the upper
                        // triangular part is stored in column-packed format.

    Entry **Stacks ;    // size ns; an array of stacks holding the R and H
                        // factors and the current frontal matrix F at the head.
                        // This is followed by empty space, then the C blocks of
                        // prior frontal matrices at the bottom.  When the
                        // factorization is complete, only the R and H part at
                        // the head of each stack is left.

    UF_long *Stack_size ;   // size ns; Stack_size [s] is the size of Stacks [s]

    UF_long hisize ;        // size of Hii

    UF_long n ;             // A is m-by-n
    UF_long m ;
    UF_long nf ;            // number of frontal matrices
    UF_long ntasks ;        // number of tasks in task graph actually used
    UF_long ns ;            // number of stacks actually used
    UF_long maxstack ;      // size of sequential stack, if used

    // -------------------------------------------------------------------------
    // for rank detection and m < n case
    // -------------------------------------------------------------------------

    char *Rdead ;       // size n, Rdead [k] = 1 if k is a dead pivot column,
                        // Rdead [k] = 0 otherwise.  If no columns are dead,
                        // this is NULL.  If m < n, then at least m-n columns
                        // will be dead. 

    UF_long rank ;      // number of live pivot columns
    UF_long rank1 ;     // number of live pivot columns in first ntol columns
                        // of A

    UF_long maxfrank ;  // max number of rows in any R block

    double norm_E_fro ; // 2-norm of w, the vector of dead column 2-norms

    // -------------------------------------------------------------------------
    // for keeping Householder vectors
    // -------------------------------------------------------------------------

    // The factorization is R = (H_s * ... * H_2 * H_1) * P_H
    // where P_H is the permutation HPinv, and H_1, ... H_s are the Householder
    // vectors (s = rjsize).

    UF_long keepH ;     // TRUE if H is present

    UF_long rjsize ;    // size of Hstair and HTau

    UF_long *HStair ;   // size rjsize.  The list Hstair [Rp [f] ... Rp [f+1]-1]
                        // gives the staircase for front F

    Entry *HTau ;       // size rjsize.  The list HTau [Rp [f] ... Rp [f+1]-1]
                        // gives the Householder coefficients for front F

    UF_long *Hii ;      // size hisize, row indices of H.

    UF_long *HPinv ;    // size m.  HPinv [i] = k if row i of A and H is row k
                        // of R.  This permutation includes QRsym->PLinv, and
                        // the permutation constructed via pivotal row ordering
                        // during factorization.

    UF_long *Hm ;       // size nf, Hm [f] = # of rows in front F
    UF_long *Hr ;       // size nf, Hr [f] = # of rows in R block of front F
    UF_long maxfm ;     // max (Hm [0:nf-1]), computed only if H kept

} ;


// =============================================================================
// === SuiteSparseQR_factorization =============================================
// =============================================================================

// A combined symbolic+numeric QR factorization of A or [A B],
// with singletons

template <typename Entry> struct SuiteSparseQR_factorization
{

    // QR factorization of A or [A Binput] after singletons have been removed
    double tol ;        // tol used
    spqr_symbolic *QRsym ;
    spqr_numeric <Entry> *QRnum ;

    // singletons, in compressed-row form; R is n1rows-by-n
    UF_long *R1p ;      // size n1rows+1
    UF_long *R1j ;
    Entry *R1x ;
    UF_long r1nz ;      // nnz (R1)

    // combined singleton and fill-reducing permutation
    UF_long *Q1fill ;
    UF_long *P1inv ;
    UF_long *HP1inv ;   // NULL if n1cols == 0, in which case QRnum->HPinv
                        // serves in its place.

    // Rmap and RmapInv are NULL if QR->rank == A->ncol
    UF_long *Rmap ;     // size n.  Rmap [j] = k if column j of R is the kth
                        // live column and where k < QR->rank; otherwise, if
                        // j is a dead column, then k >= QR->rank.
    UF_long *RmapInv ;

    UF_long n1rows ;    // number of singleton rows of [A B]
    UF_long n1cols ;    // number of singleton columns of [A B]

    UF_long narows ;    // number of rows of A
    UF_long nacols ;    // number of columns of A
    UF_long bncols ;    // number of columns of B
    UF_long rank ;      // rank estimate of A (n1rows + QRnum->rank1), ranges
                        // from 0 to min(m,n)

    int allow_tol ;     // if TRUE, do rank detection
} ;


// =============================================================================
// === Simple user-callable SuiteSparseQR functions ============================
// =============================================================================

//  SuiteSparseQR           Sparse QR factorization and solve
//  SuiteSparseQR_qmult     Q*X, Q'*X, X*Q, or X*Q' for X full or sparse

// returns rank(A) estimate, or EMPTY on failure
template <typename Entry> UF_long SuiteSparseQR
(
    // inputs, not modified
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,             // only accept singletons above tol

    UF_long econ,           // number of rows of C and R to return; a value
                            // less than the rank r of A is treated as r, and
                            // a value greater than m is treated as m.

    int getCTX,             // if 0: return Z = C of size econ-by-bncols
                            // if 1: return Z = C' of size bncols-by-econ
                            // if 2: return Z = X of size econ-by-bncols

    cholmod_sparse *A,      // m-by-n sparse matrix

    // B is either sparse or dense.  If Bsparse is non-NULL, B is sparse and
    // Bdense is ignored.  If Bsparse is NULL and Bdense is non-NULL, then B is
    // dense.  B is not present if both are NULL.
    cholmod_sparse *Bsparse,
    cholmod_dense *Bdense,

    // output arrays, neither allocated nor defined on input.

    // Z is the matrix C, C', or X
    cholmod_sparse **Zsparse,
    cholmod_dense  **Zdense,
    cholmod_sparse **R,     // the R factor
    UF_long **E,            // size n; fill-reducing ordering of A.
    cholmod_sparse **H,     // the Householder vectors (m-by-nh)
    UF_long **HPinv,        // size m; row permutation for H
    cholmod_dense **HTau,   // size nh, Householder coefficients

    // workspace and parameters
    cholmod_common *cc
) ;

// X = A\dense(B)
template <typename Entry> cholmod_dense *SuiteSparseQR
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    cholmod_sparse *A,      // m-by-n sparse matrix
    cholmod_dense  *B,      // m-by-nrhs
    cholmod_common *cc      // workspace and parameters
) ;

// X = A\dense(B) using default ordering and tolerance
template <typename Entry> cholmod_dense *SuiteSparseQR
(
    cholmod_sparse *A,      // m-by-n sparse matrix
    cholmod_dense  *B,      // m-by-nrhs
    cholmod_common *cc      // workspace and parameters
) ;

// X = A\sparse(B)
template <typename Entry> cholmod_sparse *SuiteSparseQR
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    cholmod_sparse *A,      // m-by-n sparse matrix
    cholmod_sparse *B,      // m-by-nrhs
    cholmod_common *cc      // workspace and parameters
) ;

// [Q,R,E] = qr(A), returning Q as a sparse matrix
template <typename Entry> UF_long SuiteSparseQR     // returns rank(A) estimate
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    UF_long econ,
    cholmod_sparse *A,      // m-by-n sparse matrix
    // outputs
    cholmod_sparse **Q,     // m-by-e sparse matrix where e=max(econ,rank(A))
    cholmod_sparse **R,     // e-by-n sparse matrix
    UF_long **E,            // permutation of 0:n-1, NULL if identity
    cholmod_common *cc      // workspace and parameters
) ;

// [Q,R,E] = qr(A), discarding Q
template <typename Entry> UF_long SuiteSparseQR     // returns rank(A) estimate
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    UF_long econ,
    cholmod_sparse *A,      // m-by-n sparse matrix
    // outputs
    cholmod_sparse **R,     // e-by-n sparse matrix
    UF_long **E,            // permutation of 0:n-1, NULL if identity
    cholmod_common *cc      // workspace and parameters
) ;

// [C,R,E] = qr(A,B), where C and B are dense
template <typename Entry> UF_long SuiteSparseQR
(
    // inputs, not modified
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,             // only accept singletons above tol
    UF_long econ,           // number of rows of C and R to return
    cholmod_sparse *A,      // m-by-n sparse matrix
    cholmod_dense  *B,      // m-by-nrhs dense matrix
    // outputs
    cholmod_dense  **C,     // C = Q'*B, an e-by-nrhs dense matrix
    cholmod_sparse **R,     // e-by-n sparse matrix where e=max(econ,rank(A))
    UF_long **E,            // permutation of 0:n-1, NULL if identity
    cholmod_common *cc      // workspace and parameters
) ;

// [C,R,E] = qr(A,B), where C and B are sparse
template <typename Entry> UF_long SuiteSparseQR
(
    // inputs, not modified
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,             // only accept singletons above tol
    UF_long econ,           // number of rows of C and R to return
    cholmod_sparse *A,      // m-by-n sparse matrix
    cholmod_sparse *B,      // m-by-nrhs sparse matrix
    // outputs
    cholmod_sparse **C,     // C = Q'*B, an e-by-nrhs sparse matrix
    cholmod_sparse **R,     // e-by-n sparse matrix where e=max(econ,rank(A))
    UF_long **E,            // permutation of 0:n-1, NULL if identity
    cholmod_common *cc      // workspace and parameters
) ;

// [Q,R,E] = qr(A) where Q is returned in Householder form
template <typename Entry> UF_long SuiteSparseQR
(
    // inputs, not modified
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,             // only accept singletons above tol
    UF_long econ,           // number of rows of C and R to return
    cholmod_sparse *A,      // m-by-n sparse matrix
    // outputs
    cholmod_sparse **R,     // the R factor
    UF_long **E,            // permutation of 0:n-1, NULL if identity
    cholmod_sparse **H,     // the Householder vectors (m-by-nh)
    UF_long **HPinv,        // size m; row permutation for H
    cholmod_dense **HTau,   // size nh, Householder coefficients
    cholmod_common *cc      // workspace and parameters
) ;

// =============================================================================
// === SuiteSparseQR_qmult =====================================================
// =============================================================================

// This function takes as input the matrix Q in Householder form, as returned
// by SuiteSparseQR (... H, HPinv, HTau, cc) above.

// returns Y of size m-by-n (NULL on failure)
template <typename Entry> cholmod_dense *SuiteSparseQR_qmult
(
    // inputs, no modified
    int method,         // 0,1,2,3
    cholmod_sparse *H,      // either m-by-nh or n-by-nh
    cholmod_dense *HTau,    // size 1-by-nh
    UF_long *HPinv,         // size mh
    cholmod_dense *Xdense,  // size m-by-n

    // workspace and parameters
    cholmod_common *cc
) ;

template <typename Entry> cholmod_sparse *SuiteSparseQR_qmult
(
    // inputs, no modified
    int method,             // 0,1,2,3
    cholmod_sparse *H,      // either m-by-nh or n-by-nh
    cholmod_dense *HTau,    // size 1-by-nh
    UF_long *HPinv,         // size mh
    cholmod_sparse *X,

    // workspace and parameters
    cholmod_common *cc
) ;

// =============================================================================
// === Expert user-callable SuiteSparseQR functions ============================
// =============================================================================

#ifndef NEXPERT

// These functions are "expert" routines, allowing reuse of the QR
// factorization for different right-hand-sides.  They also allow the user to
// find the minimum 2-norm solution to an undertermined system of equations.

template <typename Entry>
SuiteSparseQR_factorization <Entry> *SuiteSparseQR_factorize
(
    // inputs, not modified:
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,             // treat columns with 2-norm <= tol as zero
    cholmod_sparse *A,      // sparse matrix to factorize
    // workspace and parameters
    cholmod_common *cc
) ;

template <typename Entry> cholmod_dense *SuiteSparseQR_solve    // returns X
(
    // inputs, not modified:
    int system,                 // which system to solve
    SuiteSparseQR_factorization <Entry> *QR, // of an m-by-n sparse matrix A
    cholmod_dense *B,           // right-hand-side, m-by-nrhs or n-by-nrhs
    // workspace and parameters
    cholmod_common *cc
) ;

template <typename Entry> cholmod_sparse *SuiteSparseQR_solve    // returns X
(
    // inputs, not modified:
    int system,                 // which system to solve (0,1,2,3)
    SuiteSparseQR_factorization <Entry> *QR, // of an m-by-n sparse matrix A
    cholmod_sparse *Bsparse,    // right-hand-side, m-by-nrhs or n-by-nrhs
    // workspace and parameters
    cholmod_common *cc
) ;

// returns Y of size m-by-n, or NULL on failure
template <typename Entry> cholmod_dense *SuiteSparseQR_qmult
(
    // inputs, not modified
    int method,                 // 0,1,2,3 (same as SuiteSparseQR_qmult)
    SuiteSparseQR_factorization <Entry> *QR, // of an m-by-n sparse matrix A
    cholmod_dense *Xdense,      // size m-by-n with leading dimension ldx
    // workspace and parameters
    cholmod_common *cc
) ;

// returns Y of size m-by-n, or NULL on failure
template <typename Entry> cholmod_sparse *SuiteSparseQR_qmult
(
    // inputs, not modified
    int method,                 // 0,1,2,3
    SuiteSparseQR_factorization <Entry> *QR, // of an m-by-n sparse matrix A
    cholmod_sparse *Xsparse,    // size m-by-n
    // workspace and parameters
    cholmod_common *cc
) ;

// free the QR object
template <typename Entry> int SuiteSparseQR_free
(
    SuiteSparseQR_factorization <Entry> **QR, // of an m-by-n sparse matrix A
    cholmod_common *cc
) ;

// find the min 2-norm solution to a sparse linear system
template <typename Entry> cholmod_dense *SuiteSparseQR_min2norm
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    cholmod_sparse *A,
    cholmod_dense *B,
    cholmod_common *cc
) ;

template <typename Entry> cholmod_sparse *SuiteSparseQR_min2norm
(
    int ordering,           // all, except 3:given treated as 0:fixed
    double tol,
    cholmod_sparse *A,
    cholmod_sparse *B,
    cholmod_common *cc
) ;

// symbolic QR factorization; no singletons exploited
template <typename Entry> 
SuiteSparseQR_factorization <Entry> *SuiteSparseQR_symbolic
(
    // inputs:
    int ordering,           // all, except 3:given treated as 0:fixed
    int allow_tol,          // if FALSE, tol is ignored by the numeric
                            // factorization, and no rank detection is performed
    cholmod_sparse *A,      // sparse matrix to factorize (A->x ignored)
    cholmod_common *cc      // workspace and parameters
) ;

// numeric QR factorization;
template <typename Entry> int SuiteSparseQR_numeric
(
    // inputs:
    double tol,             // treat columns with 2-norm <= tol as zero
    cholmod_sparse *A,      // sparse matrix to factorize
    // input/output
    SuiteSparseQR_factorization <Entry> *QR,
    cholmod_common *cc      // workspace and parameters
) ;

#endif

// =============================================================================
// === high-resolution timing ==================================================
// =============================================================================

#ifdef TIMING

extern "C" {

#include <time.h>
#include <sys/time.h>

double spqr_time ( ) ;   // returns current time in seconds

}
#endif

#endif
