/*!
\file  sort.c
\brief This file contains GKlib's various sorting routines

These routines are implemented using the GKSORT macro that is defined
in gk_qsort.h and is based on GNU's GLIBC qsort() implementation.

Additional sorting routines can be created using the same way that
these routines where defined.

\date   Started 4/4/07
\author George
\version\verbatim $Id: sort.c 10796 2011-09-23 21:33:09Z karypis $ \endverbatim
*/

#include <GKlib.h>



/*************************************************************************/
/*! Sorts an array of chars in increasing order */
/*************************************************************************/
void gk_csorti(size_t n, char *base)
{
#define char_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(char, base, n, char_lt);
#undef char_lt
}


/*************************************************************************/
/*! Sorts an array of chars in decreasing order */
/*************************************************************************/
void gk_csortd(size_t n, char *base)
{
#define char_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(char, base, n, char_gt);
#undef char_gt
}


/*************************************************************************/
/*! Sorts an array of integers in increasing order */
/*************************************************************************/
void gk_isorti(size_t n, int *base)
{
#define int_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(int, base, n, int_lt);
#undef int_lt
}


/*************************************************************************/
/*! Sorts an array of integers in decreasing order */
/*************************************************************************/
void gk_isortd(size_t n, int *base)
{
#define int_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(int, base, n, int_gt);
#undef int_gt
}


/*************************************************************************/
/*! Sorts an array of floats in increasing order */
/*************************************************************************/
void gk_fsorti(size_t n, float *base)
{
#define float_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(float, base, n, float_lt);
#undef float_lt
}


/*************************************************************************/
/*! Sorts an array of floats in decreasing order */
/*************************************************************************/
void gk_fsortd(size_t n, float *base)
{
#define float_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(float, base, n, float_gt);
#undef float_gt
}


/*************************************************************************/
/*! Sorts an array of doubles in increasing order */
/*************************************************************************/
void gk_dsorti(size_t n, double *base)
{
#define double_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(double, base, n, double_lt);
#undef double_lt
}


/*************************************************************************/
/*! Sorts an array of doubles in decreasing order */
/*************************************************************************/
void gk_dsortd(size_t n, double *base)
{
#define double_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(double, base, n, double_gt);
#undef double_gt
}


/*************************************************************************/
/*! Sorts an array of gk_idx_t in increasing order */
/*************************************************************************/
void gk_idxsorti(size_t n, gk_idx_t *base)
{
#define idx_lt(a, b) ((*a) < (*b))
  GK_MKQSORT(gk_idx_t, base, n, idx_lt);
#undef idx_lt
}


/*************************************************************************/
/*! Sorts an array of gk_idx_t in decreasing order */
/*************************************************************************/
void gk_idxsortd(size_t n, gk_idx_t *base)
{
#define idx_gt(a, b) ((*a) > (*b))
  GK_MKQSORT(gk_idx_t, base, n, idx_gt);
#undef idx_gt
}




/*************************************************************************/
/*! Sorts an array of gk_ckv_t in increasing order */
/*************************************************************************/
void gk_ckvsorti(size_t n, gk_ckv_t *base)
{
#define ckey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_ckv_t, base, n, ckey_lt);
#undef ckey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_ckv_t in decreasing order */
/*************************************************************************/
void gk_ckvsortd(size_t n, gk_ckv_t *base)
{
#define ckey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_ckv_t, base, n, ckey_gt);
#undef ckey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_ikv_t in increasing order */
/*************************************************************************/
void gk_ikvsorti(size_t n, gk_ikv_t *base)
{
#define ikey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_ikv_t, base, n, ikey_lt);
#undef ikey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_ikv_t in decreasing order */
/*************************************************************************/
void gk_ikvsortd(size_t n, gk_ikv_t *base)
{
#define ikey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_ikv_t, base, n, ikey_gt);
#undef ikey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_i32kv_t in increasing order */
/*************************************************************************/
void gk_i32kvsorti(size_t n, gk_i32kv_t *base)
{
#define ikey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_i32kv_t, base, n, ikey_lt);
#undef ikey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_i32kv_t in decreasing order */
/*************************************************************************/
void gk_i32kvsortd(size_t n, gk_i32kv_t *base)
{
#define ikey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_i32kv_t, base, n, ikey_gt);
#undef ikey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_i64kv_t in increasing order */
/*************************************************************************/
void gk_i64kvsorti(size_t n, gk_i64kv_t *base)
{
#define ikey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_i64kv_t, base, n, ikey_lt);
#undef ikey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_i64kv_t in decreasing order */
/*************************************************************************/
void gk_i64kvsortd(size_t n, gk_i64kv_t *base)
{
#define ikey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_i64kv_t, base, n, ikey_gt);
#undef ikey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_zkv_t in increasing order */
/*************************************************************************/
void gk_zkvsorti(size_t n, gk_zkv_t *base)
{
#define zkey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_zkv_t, base, n, zkey_lt);
#undef zkey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_zkv_t in decreasing order */
/*************************************************************************/
void gk_zkvsortd(size_t n, gk_zkv_t *base)
{
#define zkey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_zkv_t, base, n, zkey_gt);
#undef zkey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_fkv_t in increasing order */
/*************************************************************************/
void gk_fkvsorti(size_t n, gk_fkv_t *base)
{
#define fkey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_fkv_t, base, n, fkey_lt);
#undef fkey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_fkv_t in decreasing order */
/*************************************************************************/
void gk_fkvsortd(size_t n, gk_fkv_t *base)
{
#define fkey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_fkv_t, base, n, fkey_gt);
#undef fkey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_dkv_t in increasing order */
/*************************************************************************/
void gk_dkvsorti(size_t n, gk_dkv_t *base)
{
#define dkey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_dkv_t, base, n, dkey_lt);
#undef dkey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_fkv_t in decreasing order */
/*************************************************************************/
void gk_dkvsortd(size_t n, gk_dkv_t *base)
{
#define dkey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_dkv_t, base, n, dkey_gt);
#undef dkey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_skv_t in increasing order */
/*************************************************************************/
void gk_skvsorti(size_t n, gk_skv_t *base)
{
#define skey_lt(a, b) (strcmp((a)->key, (b)->key) < 0)
  GK_MKQSORT(gk_skv_t, base, n, skey_lt);
#undef skey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_skv_t in decreasing order */
/*************************************************************************/
void gk_skvsortd(size_t n, gk_skv_t *base)
{
#define skey_gt(a, b) (strcmp((a)->key, (b)->key) > 0)
  GK_MKQSORT(gk_skv_t, base, n, skey_gt);
#undef skey_gt
}


/*************************************************************************/
/*! Sorts an array of gk_idxkv_t in increasing order */
/*************************************************************************/
void gk_idxkvsorti(size_t n, gk_idxkv_t *base)
{
#define idxkey_lt(a, b) ((a)->key < (b)->key)
  GK_MKQSORT(gk_idxkv_t, base, n, idxkey_lt);
#undef idxkey_lt
}


/*************************************************************************/
/*! Sorts an array of gk_idxkv_t in decreasing order */
/*************************************************************************/
void gk_idxkvsortd(size_t n, gk_idxkv_t *base)
{
#define idxkey_gt(a, b) ((a)->key > (b)->key)
  GK_MKQSORT(gk_idxkv_t, base, n, idxkey_gt);
#undef idxkey_gt
}
