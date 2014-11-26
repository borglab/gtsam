/*!
\file  gklib.c
\brief Various helper routines generated using GKlib's templates

\date   Started 4/12/2007
\author George  
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version\verbatim $Id: gklib.c 10395 2011-06-23 23:28:06Z karypis $ \endverbatim
*/


#include "metislib.h"


/*************************************************************************/
/*! BLAS routines */
/*************************************************************************/
GK_MKBLAS(i,  idx_t,  idx_t)
GK_MKBLAS(r,  real_t, real_t)

/*************************************************************************/
/*! Memory allocation routines */
/*************************************************************************/
GK_MKALLOC(i,    idx_t)
GK_MKALLOC(r,    real_t)
GK_MKALLOC(ikv,  ikv_t)
GK_MKALLOC(rkv,  rkv_t)

/*************************************************************************/
/*! Priority queues routines */
/*************************************************************************/
#define key_gt(a, b) ((a) > (b))
GK_MKPQUEUE(ipq, ipq_t, ikv_t, idx_t, idx_t, ikvmalloc, IDX_MAX, key_gt)
GK_MKPQUEUE(rpq, rpq_t, rkv_t, real_t, idx_t, rkvmalloc, REAL_MAX, key_gt)
#undef key_gt

/*************************************************************************/
/*! Random number generation routines */
/*************************************************************************/
GK_MKRANDOM(i, idx_t, idx_t)

/*************************************************************************/
/*! Utility routines */
/*************************************************************************/
GK_MKARRAY2CSR(i, idx_t)

/*************************************************************************/
/*! Sorting routines */
/*************************************************************************/
void isorti(size_t n, idx_t *base)
{
#define i_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(idx_t, base, n, i_lt);
#undef i_lt
}

void isortd(size_t n, idx_t *base)
{
#define i_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(idx_t, base, n, i_gt);
#undef i_gt
}

void rsorti(size_t n, real_t *base)
{
#define r_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(real_t, base, n, r_lt);
#undef r_lt
}

void rsortd(size_t n, real_t *base)
{
#define r_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(real_t, base, n, r_gt);
#undef r_gt
}

void ikvsorti(size_t n, ikv_t *base)
{
#define ikey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(ikv_t, base, n, ikey_lt);
#undef ikey_lt
}

/* Sorts based both on key and val */
void ikvsortii(size_t n, ikv_t *base)
{
#define ikeyval_lt(a, b) ((a)->key < (b)->key || ((a)->key == (b)->key && (a)->val < (b)->val))
  GK_MKQSORT(ikv_t, base, n, ikeyval_lt);
#undef ikeyval_lt
}

void ikvsortd(size_t n, ikv_t *base)
{
#define ikey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(ikv_t, base, n, ikey_gt);
#undef ikey_gt
}

void rkvsorti(size_t n, rkv_t *base)
{
#define rkey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(rkv_t, base, n, rkey_lt);
#undef rkey_lt
}

void rkvsortd(size_t n, rkv_t *base)
{
#define rkey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(rkv_t, base, n, rkey_gt);
#undef rkey_gt
}

void uvwsorti(size_t n, uvw_t *base)
{
#define uvwkey_lt(a, b) ((a)->u < (b)->u || ((a)->u == (b)->u && (a)->v < (b)->v))
  GK_MKQSORT(uvw_t, base, n, uvwkey_lt);
#undef uvwkey_lt
}

