/*!
\file  gk_types.h
\brief This file contains basic scalar datatype used in GKlib

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_types.h 10711 2011-08-31 22:23:04Z karypis $ \endverbatim
*/

#ifndef _GK_TYPES_H_
#define _GK_TYPES_H_

/*************************************************************************
* Basic data type definitions. These definitions allow GKlib to separate
* the following elemental types:
* - loop iterator variables, which are set to size_t
* - signed and unsigned int variables that can be set to any # of bits
* - signed and unsigned long variables that can be set to any # of bits
* - real variables, which can be set to single or double precision.
**************************************************************************/
/*typedef ptrdiff_t       gk_idx_t;       */  /* index variable */
typedef ssize_t         gk_idx_t;         /* index variable */
typedef int32_t         gk_int_t;         /* integer values */
typedef uint32_t        gk_uint_t;        /* unsigned integer values */
typedef int64_t         gk_long_t;        /* long integer values */
typedef uint64_t        gk_ulong_t;       /* unsigned long integer values */
typedef float           gk_real_t;        /* real type */
typedef double          gk_dreal_t;       /* double precission real type */
typedef double          gk_wclock_t;	  /* wall-clock time */

/*#define GK_IDX_MAX PTRDIFF_MAX*/
#define GK_IDX_MAX ((SIZE_MAX>>1)-2)

#define PRIGKIDX "zd"
#define SCNGKIDX "zd"


#endif
