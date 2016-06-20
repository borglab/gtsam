/*!
\file  gk_mksort.h
\brief Templates for the qsort routine

\date   Started 3/28/07
\author George
\version\verbatim $Id: gk_mksort.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/


#ifndef _GK_MKSORT_H_
#define _GK_MKSORT_H_

/* $Id: gk_mksort.h 10711 2011-08-31 22:23:04Z karypis $
 * Adopted from GNU glibc by Mjt.
 * See stdlib/qsort.c in glibc */

/* Copyright (C) 1991, 1992, 1996, 1997, 1999 Free Software Foundation, Inc.
   This file is part of the GNU C Library.
   Written by Douglas C. Schmidt (schmidt@ics.uci.edu).

   The GNU C Library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   The GNU C Library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the GNU C Library; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.  */

/* in-line qsort implementation.  Differs from traditional qsort() routine
 * in that it is a macro, not a function, and instead of passing an address
 * of a comparision routine to the function, it is possible to inline
 * comparision routine, thus speed up sorting alot.
 *
 * Usage:
 *  #include "iqsort.h"
 *  #define islt(a,b) (strcmp((*a),(*b))<0)
 *  char *arr[];
 *  int n;
 *  GKQSORT(char*, arr, n, islt);
 *
 * The "prototype" and 4 arguments are:
 *  GKQSORT(TYPE,BASE,NELT,ISLT)
 *  1) type of each element, TYPE,
 *  2) address of the beginning of the array, of type TYPE*,
 *  3) number of elements in the array, and
 *  4) comparision routine.
 * Array pointer and number of elements are referenced only once.
 * This is similar to a call
 *  qsort(BASE,NELT,sizeof(TYPE),ISLT)
 * with the difference in last parameter.
 * Note the islt macro/routine (it receives pointers to two elements):
 * the only condition of interest is whenever one element is less than
 * another, no other conditions (greather than, equal to etc) are tested.
 * So, for example, to define integer sort, use:
 *  #define islt(a,b) ((*a)<(*b))
 *  GKQSORT(int, arr, n, islt)
 *
 * The macro could be used to implement a sorting function (see examples
 * below), or to implement the sorting algorithm inline.  That is, either
 * create a sorting function and use it whenever you want to sort something,
 * or use GKQSORT() macro directly instead a call to such routine.  Note that
 * the macro expands to quite some code (compiled size of int qsort on x86
 * is about 700..800 bytes).
 *
 * Using this macro directly it isn't possible to implement traditional
 * qsort() routine, because the macro assumes sizeof(element) == sizeof(TYPE),
 * while qsort() allows element size to be different.
 *
 * Several ready-to-use examples:
 *
 * Sorting array of integers:
 * void int_qsort(int *arr, unsigned n) {
 * #define int_lt(a,b) ((*a)<(*b))
 *   GKQSORT(int, arr, n, int_lt);
 * }
 *
 * Sorting array of string pointers:
 * void str_qsort(char *arr[], unsigned n) {
 * #define str_lt(a,b) (strcmp((*a),(*b)) < 0)
 *   GKQSORT(char*, arr, n, str_lt);
 * }
 *
 * Sorting array of structures:
 *
 * struct elt {
 *   int key;
 *   ...
 * };
 * void elt_qsort(struct elt *arr, unsigned n) {
 * #define elt_lt(a,b) ((a)->key < (b)->key)
 *  GKQSORT(struct elt, arr, n, elt_lt);
 * }
 *
 * And so on.
 */

/* Swap two items pointed to by A and B using temporary buffer t. */
#define _GKQSORT_SWAP(a, b, t) ((void)((t = *a), (*a = *b), (*b = t)))

/* Discontinue quicksort algorithm when partition gets below this size.
   This particular magic number was chosen to work best on a Sun 4/260. */
#define _GKQSORT_MAX_THRESH 4

/* The next 4 #defines implement a very fast in-line stack abstraction. */
#define _GKQSORT_STACK_SIZE	    (8 * sizeof(size_t))
#define _GKQSORT_PUSH(top, low, high) (((top->_lo = (low)), (top->_hi = (high)), ++top))
#define	_GKQSORT_POP(low, high, top)  ((--top, (low = top->_lo), (high = top->_hi)))
#define	_GKQSORT_STACK_NOT_EMPTY	    (_stack < _top)


/* The main code starts here... */
#define GK_MKQSORT(GKQSORT_TYPE,GKQSORT_BASE,GKQSORT_NELT,GKQSORT_LT)   \
{									\
  GKQSORT_TYPE *const _base = (GKQSORT_BASE);				\
  const size_t _elems = (GKQSORT_NELT);					\
  GKQSORT_TYPE _hold;							\
									\
  if (_elems == 0)                                                      \
    return;                                                             \
                                                                        \
  /* Don't declare two variables of type GKQSORT_TYPE in a single	\
   * statement: eg `TYPE a, b;', in case if TYPE is a pointer,		\
   * expands to `type* a, b;' wich isn't what we want.			\
   */									\
									\
  if (_elems > _GKQSORT_MAX_THRESH) {					\
    GKQSORT_TYPE *_lo = _base;						\
    GKQSORT_TYPE *_hi = _lo + _elems - 1;				\
    struct {								\
      GKQSORT_TYPE *_hi; GKQSORT_TYPE *_lo;				\
    } _stack[_GKQSORT_STACK_SIZE], *_top = _stack + 1;			\
									\
    while (_GKQSORT_STACK_NOT_EMPTY) {					\
      GKQSORT_TYPE *_left_ptr; GKQSORT_TYPE *_right_ptr;		\
									\
      /* Select median value from among LO, MID, and HI. Rearrange	\
         LO and HI so the three values are sorted. This lowers the	\
         probability of picking a pathological pivot value and		\
         skips a comparison for both the LEFT_PTR and RIGHT_PTR in	\
         the while loops. */						\
									\
      GKQSORT_TYPE *_mid = _lo + ((_hi - _lo) >> 1);			\
									\
      if (GKQSORT_LT (_mid, _lo))					\
        _GKQSORT_SWAP (_mid, _lo, _hold);				\
      if (GKQSORT_LT (_hi, _mid))					\
        _GKQSORT_SWAP (_mid, _hi, _hold);				\
      else								\
        goto _jump_over;						\
      if (GKQSORT_LT (_mid, _lo))					\
        _GKQSORT_SWAP (_mid, _lo, _hold);				\
  _jump_over:;								\
									\
      _left_ptr  = _lo + 1;						\
      _right_ptr = _hi - 1;						\
									\
      /* Here's the famous ``collapse the walls'' section of quicksort.	\
         Gotta like those tight inner loops!  They are the main reason	\
         that this algorithm runs much faster than others. */		\
      do {								\
        while (GKQSORT_LT (_left_ptr, _mid))				\
         ++_left_ptr;							\
									\
        while (GKQSORT_LT (_mid, _right_ptr))				\
          --_right_ptr;							\
									\
        if (_left_ptr < _right_ptr) {					\
          _GKQSORT_SWAP (_left_ptr, _right_ptr, _hold);			\
          if (_mid == _left_ptr)					\
            _mid = _right_ptr;						\
          else if (_mid == _right_ptr)					\
            _mid = _left_ptr;						\
          ++_left_ptr;							\
          --_right_ptr;							\
        }								\
        else if (_left_ptr == _right_ptr) {				\
          ++_left_ptr;							\
          --_right_ptr;							\
          break;							\
        }								\
      } while (_left_ptr <= _right_ptr);				\
									\
     /* Set up pointers for next iteration.  First determine whether	\
        left and right partitions are below the threshold size.  If so,	\
        ignore one or both.  Otherwise, push the larger partition's	\
        bounds on the stack and continue sorting the smaller one. */	\
									\
      if (_right_ptr - _lo <= _GKQSORT_MAX_THRESH) {			\
        if (_hi - _left_ptr <= _GKQSORT_MAX_THRESH)			\
          /* Ignore both small partitions. */				\
          _GKQSORT_POP (_lo, _hi, _top);				\
        else								\
          /* Ignore small left partition. */				\
          _lo = _left_ptr;						\
      }									\
      else if (_hi - _left_ptr <= _GKQSORT_MAX_THRESH)			\
        /* Ignore small right partition. */				\
        _hi = _right_ptr;						\
      else if (_right_ptr - _lo > _hi - _left_ptr) {			\
        /* Push larger left partition indices. */			\
        _GKQSORT_PUSH (_top, _lo, _right_ptr);				\
        _lo = _left_ptr;						\
      }									\
      else {								\
        /* Push larger right partition indices. */			\
        _GKQSORT_PUSH (_top, _left_ptr, _hi);				\
        _hi = _right_ptr;						\
      }									\
    }									\
  }									\
									\
  /* Once the BASE array is partially sorted by quicksort the rest	\
     is completely sorted using insertion sort, since this is efficient	\
     for partitions below MAX_THRESH size. BASE points to the		\
     beginning of the array to sort, and END_PTR points at the very	\
     last element in the array (*not* one beyond it!). */		\
									\
  {									\
    GKQSORT_TYPE *const _end_ptr = _base + _elems - 1;			\
    GKQSORT_TYPE *_tmp_ptr = _base;					\
    register GKQSORT_TYPE *_run_ptr;					\
    GKQSORT_TYPE *_thresh;						\
									\
    _thresh = _base + _GKQSORT_MAX_THRESH;				\
    if (_thresh > _end_ptr)						\
      _thresh = _end_ptr;						\
									\
    /* Find smallest element in first threshold and place it at the	\
       array's beginning.  This is the smallest array element,		\
       and the operation speeds up insertion sort's inner loop. */	\
									\
    for (_run_ptr = _tmp_ptr + 1; _run_ptr <= _thresh; ++_run_ptr)	\
      if (GKQSORT_LT (_run_ptr, _tmp_ptr))				\
        _tmp_ptr = _run_ptr;						\
									\
    if (_tmp_ptr != _base)						\
      _GKQSORT_SWAP (_tmp_ptr, _base, _hold);				\
									\
    /* Insertion sort, running from left-hand-side			\
     * up to right-hand-side.  */					\
									\
    _run_ptr = _base + 1;						\
    while (++_run_ptr <= _end_ptr) {					\
      _tmp_ptr = _run_ptr - 1;						\
      while (GKQSORT_LT (_run_ptr, _tmp_ptr))				\
        --_tmp_ptr;							\
									\
      ++_tmp_ptr;							\
      if (_tmp_ptr != _run_ptr) {					\
        GKQSORT_TYPE *_trav = _run_ptr + 1;				\
        while (--_trav >= _run_ptr) {					\
          GKQSORT_TYPE *_hi; GKQSORT_TYPE *_lo;				\
          _hold = *_trav;						\
									\
          for (_hi = _lo = _trav; --_lo >= _tmp_ptr; _hi = _lo)		\
            *_hi = *_lo;						\
          *_hi = _hold;							\
        }								\
      }									\
    }									\
  }									\
									\
}

#endif
