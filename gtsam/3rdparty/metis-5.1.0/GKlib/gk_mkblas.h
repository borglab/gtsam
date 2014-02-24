/*!
\file  gk_mkblas.h
\brief Templates for BLAS-like routines

\date   Started 3/28/07
\author George
\version\verbatim $Id: gk_mkblas.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_MKBLAS_H_
#define _GK_MKBLAS_H_


#define GK_MKBLAS(PRFX, TYPE, OUTTYPE) \
/*************************************************************************/\
/*! The macro for gk_?incset()-class of routines */\
/*************************************************************************/\
TYPE *PRFX ## incset(size_t n, TYPE baseval, TYPE *x)\
{\
  size_t i;\
\
  for (i=0; i<n; i++)\
    x[i] = baseval+i;\
\
  return x;\
}\
\
/*************************************************************************/\
/*! The macro for gk_?max()-class of routines */\
/*************************************************************************/\
TYPE PRFX ## max(size_t n, TYPE *x)\
{\
  size_t i, max=0; \
\
  if (n <= 0) return (TYPE) 0;\
\
  for (i=1; i<n; i++)\
    max = (x[i] > x[max] ? i : max);\
\
  return x[max];\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?min()-class of routines */\
/*************************************************************************/\
TYPE PRFX ## min(size_t n, TYPE *x)\
{\
  size_t i, min=0;\
\
  if (n <= 0) return (TYPE) 0;\
\
  for (i=1; i<n; i++)\
    min = (x[i] < x[min] ? i : min);\
\
  return x[min];\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?argmax()-class of routines */\
/*************************************************************************/\
size_t PRFX ## argmax(size_t n, TYPE *x)\
{\
  size_t i, max=0;\
\
  for (i=1; i<n; i++)\
    max = (x[i] > x[max] ? i : max);\
\
  return max;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?argmin()-class of routines */\
/*************************************************************************/\
size_t PRFX ## argmin(size_t n, TYPE *x)\
{\
  size_t i, min=0;\
\
  for (i=1; i<n; i++)\
    min = (x[i] < x[min] ? i : min);\
\
  return min;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?argmax_n()-class of routines */\
/*************************************************************************/\
size_t PRFX ## argmax_n(size_t n, TYPE *x, size_t k)\
{\
  size_t i, max_n;\
  PRFX ## kv_t *cand;\
\
  cand = PRFX ## kvmalloc(n, "GK_ARGMAX_N: cand");\
\
  for (i=0; i<n; i++) {\
    cand[i].val = i;\
    cand[i].key = x[i];\
  }\
  PRFX ## kvsortd(n, cand);\
\
  max_n = cand[k-1].val;\
\
  gk_free((void *)&cand, LTERM);\
\
  return max_n;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?sum()-class of routines */\
/**************************************************************************/\
OUTTYPE PRFX ## sum(size_t n, TYPE *x, size_t incx)\
{\
  size_t i;\
  OUTTYPE sum = 0;\
\
  for (i=0; i<n; i++, x+=incx)\
    sum += (*x);\
\
  return sum;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?scale()-class of routines */\
/**************************************************************************/\
TYPE *PRFX ## scale(size_t n, TYPE alpha, TYPE *x, size_t incx)\
{\
  size_t i;\
\
  for (i=0; i<n; i++, x+=incx)\
    (*x) *= alpha;\
\
  return x;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?norm2()-class of routines */\
/**************************************************************************/\
OUTTYPE PRFX ## norm2(size_t n, TYPE *x, size_t incx)\
{\
  size_t i;\
  OUTTYPE partial = 0;\
\
  for (i=0; i<n; i++, x+=incx)\
    partial += (*x) * (*x);\
\
  return (partial > 0 ? (OUTTYPE)sqrt((double)partial) : (OUTTYPE)0);\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?dot()-class of routines */\
/**************************************************************************/\
OUTTYPE PRFX ## dot(size_t n, TYPE *x, size_t incx, TYPE *y, size_t incy)\
{\
  size_t i;\
  OUTTYPE partial = 0.0;\
 \
  for (i=0; i<n; i++, x+=incx, y+=incy)\
    partial += (*x) * (*y);\
\
  return partial;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?axpy()-class of routines */\
/**************************************************************************/\
TYPE *PRFX ## axpy(size_t n, TYPE alpha, TYPE *x, size_t incx, TYPE *y, size_t incy)\
{\
  size_t i;\
  TYPE *y_in = y;\
\
  for (i=0; i<n; i++, x+=incx, y+=incy)\
    *y += alpha*(*x);\
\
  return y_in;\
}\



#define GK_MKBLAS_PROTO(PRFX, TYPE, OUTTYPE) \
  TYPE    *PRFX ## incset(size_t n, TYPE baseval, TYPE *x);\
  TYPE     PRFX ## max(size_t n, TYPE *x);\
  TYPE     PRFX ## min(size_t n, TYPE *x);\
  size_t   PRFX ## argmax(size_t n, TYPE *x);\
  size_t   PRFX ## argmin(size_t n, TYPE *x);\
  size_t   PRFX ## argmax_n(size_t n, TYPE *x, size_t k);\
  OUTTYPE  PRFX ## sum(size_t n, TYPE *x, size_t incx);\
  TYPE    *PRFX ## scale(size_t n, TYPE alpha, TYPE *x, size_t incx);\
  OUTTYPE  PRFX ## norm2(size_t n, TYPE *x, size_t incx);\
  OUTTYPE  PRFX ## dot(size_t n, TYPE *x, size_t incx, TYPE *y, size_t incy);\
  TYPE    *PRFX ## axpy(size_t n, TYPE alpha, TYPE *x, size_t incx, TYPE *y, size_t incy);\


#endif
