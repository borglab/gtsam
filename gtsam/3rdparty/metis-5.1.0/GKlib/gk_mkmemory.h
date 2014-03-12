/*!
\file  gk_mkmemory.h
\brief Templates for memory allocation routines

\date   Started 3/29/07
\author George
\version\verbatim $Id: gk_mkmemory.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_MKMEMORY_H_
#define _GK_MKMEMORY_H_


#define GK_MKALLOC(PRFX, TYPE)\
/*************************************************************************/\
/*! The macro for gk_?malloc()-class of routines */\
/**************************************************************************/\
TYPE *PRFX ## malloc(size_t n, char *msg)\
{\
  return (TYPE *)gk_malloc(sizeof(TYPE)*n, msg);\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?realloc()-class of routines */\
/**************************************************************************/\
TYPE *PRFX ## realloc(TYPE *ptr, size_t n, char *msg)\
{\
  return (TYPE *)gk_realloc((void *)ptr, sizeof(TYPE)*n, msg);\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?smalloc()-class of routines */\
/**************************************************************************/\
TYPE *PRFX ## smalloc(size_t n, TYPE ival, char *msg)\
{\
  TYPE *ptr;\
\
  ptr = (TYPE *)gk_malloc(sizeof(TYPE)*n, msg);\
  if (ptr == NULL) \
    return NULL; \
\
  return PRFX ## set(n, ival, ptr); \
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?set()-class of routines */\
/*************************************************************************/\
TYPE *PRFX ## set(size_t n, TYPE val, TYPE *x)\
{\
  size_t i;\
\
  for (i=0; i<n; i++)\
    x[i] = val;\
\
  return x;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?set()-class of routines */\
/*************************************************************************/\
TYPE *PRFX ## copy(size_t n, TYPE *a, TYPE *b)\
{\
  return (TYPE *)memmove((void *)b, (void *)a, sizeof(TYPE)*n);\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?AllocMatrix()-class of routines */\
/**************************************************************************/\
TYPE **PRFX ## AllocMatrix(size_t ndim1, size_t ndim2, TYPE value, char *errmsg)\
{\
  gk_idx_t i, j;\
  TYPE **matrix;\
\
  matrix = (TYPE **)gk_malloc(ndim1*sizeof(TYPE *), errmsg);\
  if (matrix == NULL) \
    return NULL;\
\
  for (i=0; i<ndim1; i++) { \
    matrix[i] = PRFX ## smalloc(ndim2, value, errmsg);\
    if (matrix[i] == NULL) { \
      for (j=0; j<i; j++) \
        gk_free((void **)&matrix[j], LTERM); \
      return NULL; \
    } \
  }\
\
  return matrix;\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?AllocMatrix()-class of routines */\
/**************************************************************************/\
void PRFX ## FreeMatrix(TYPE ***r_matrix, size_t ndim1, size_t ndim2)\
{\
  gk_idx_t i;\
  TYPE **matrix;\
\
  if (*r_matrix == NULL) \
    return; \
\
  matrix = *r_matrix;\
\
  for (i=0; i<ndim1; i++) \
    gk_free((void **)&(matrix[i]), LTERM);\
\
  gk_free((void **)r_matrix, LTERM);\
}\
\
\
/*************************************************************************/\
/*! The macro for gk_?SetMatrix()-class of routines */\
/**************************************************************************/\
void PRFX ## SetMatrix(TYPE **matrix, size_t ndim1, size_t ndim2, TYPE value)\
{\
  gk_idx_t i, j;\
\
  for (i=0; i<ndim1; i++) {\
    for (j=0; j<ndim2; j++)\
      matrix[i][j] = value;\
  }\
}\


#define GK_MKALLOC_PROTO(PRFX, TYPE)\
  TYPE  *PRFX ## malloc(size_t n, char *msg);\
  TYPE  *PRFX ## realloc(TYPE *ptr, size_t n, char *msg);\
  TYPE  *PRFX ## smalloc(size_t n, TYPE ival, char *msg);\
  TYPE  *PRFX ## set(size_t n, TYPE val, TYPE *x);\
  TYPE  *PRFX ## copy(size_t n, TYPE *a, TYPE *b);\
  TYPE **PRFX ## AllocMatrix(size_t ndim1, size_t ndim2, TYPE value, char *errmsg);\
  void   PRFX ## FreeMatrix(TYPE ***r_matrix, size_t ndim1, size_t ndim2);\
  void   PRFX ## SetMatrix(TYPE **matrix, size_t ndim1, size_t ndim2, TYPE value);\



#endif
