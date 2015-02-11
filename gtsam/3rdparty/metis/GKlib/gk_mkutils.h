/*!
\file  
\brief Templates for various utility routines

\date   Started 5/28/07
\author George
\version\verbatim $Id: gk_mkutils.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_MKUTILS_H_
#define _GK_MKUTILS_H_


#define GK_MKARRAY2CSR(PRFX, TYPE)\
/*************************************************************************/\
/*! The macro for gk_?array2csr() routine */\
/**************************************************************************/\
void PRFX ## array2csr(TYPE n, TYPE range, TYPE *array, TYPE *ptr, TYPE *ind)\
{\
  TYPE i;\
\
  for (i=0; i<=range; i++)\
    ptr[i] = 0;\
\
  for (i=0; i<n; i++)\
    ptr[array[i]]++;\
\
  /* Compute the ptr, ind structure */\
  MAKECSR(i, range, ptr);\
  for (i=0; i<n; i++)\
    ind[ptr[array[i]]++] = i;\
  SHIFTCSR(i, range, ptr);\
}


#define GK_MKARRAY2CSR_PROTO(PRFX, TYPE)\
  void PRFX ## array2csr(TYPE n, TYPE range, TYPE *array, TYPE *ptr, TYPE *ind);\


#endif
