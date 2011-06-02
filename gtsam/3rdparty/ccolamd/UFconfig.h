/* ========================================================================== */
/* === UFconfig.h =========================================================== */
/* ========================================================================== */

/* Configuration file for SuiteSparse: a Suite of Sparse matrix packages
 * (AMD, COLAMD, CCOLAMD, CAMD, CHOLMOD, UMFPACK, CXSparse, and others).
 *
 * UFconfig.h provides the definition of the long integer.  On most systems,
 * a C program can be compiled in LP64 mode, in which long's and pointers are
 * both 64-bits, and int's are 32-bits.  Windows 64, however, uses the LLP64
 * model, in which int's and long's are 32-bits, and long long's and pointers
 * are 64-bits.
 *
 * SuiteSparse packages that include long integer versions are
 * intended for the LP64 mode.  However, as a workaround for Windows 64
 * (and perhaps other systems), the long integer can be redefined.
 *
 * If _WIN64 is defined, then the __int64 type is used instead of long.
 *
 * The long integer can also be defined at compile time.  For example, this
 * could be added to UFconfig.mk:
 *
 * CFLAGS = -O -D'UF_long=long long' -D'UF_long_max=9223372036854775801' \
 *   -D'UF_long_idd="lld"'
 *
 * This file defines UF_long as either long (on all but _WIN64) or
 * __int64 on Windows 64.  The intent is that a UF_long is always a 64-bit
 * integer in a 64-bit code.  ptrdiff_t might be a better choice than long;
 * it is always the same size as a pointer.
 *
 * This file also defines the SUITESPARSE_VERSION and related definitions.
 *
 * Copyright (c) 2007, University of Florida.  No licensing restrictions
 * apply to this file or to the UFconfig directory.  Author: Timothy A. Davis.
 */

#ifndef _UFCONFIG_H
#define _UFCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <limits.h>
#include <stdlib.h>

/* ========================================================================== */
/* === UF_long ============================================================== */
/* ========================================================================== */

#ifndef UF_long

#ifdef _WIN64

#define UF_long __int64
#define UF_long_max _I64_MAX
#define UF_long_idd "I64d"

#else

#define UF_long long
#define UF_long_max LONG_MAX
#define UF_long_idd "ld"

#endif
#define UF_long_id "%" UF_long_idd
#endif

/* ========================================================================== */
/* === UFconfig parameters and functions ==================================== */
/* ========================================================================== */

/* SuiteSparse-wide parameters will be placed in this struct.  So far, they
   are only used by RBio. */

typedef struct UFconfig_struct
{
    void *(*malloc_memory) (size_t) ;		/* pointer to malloc */
    void *(*realloc_memory) (void *, size_t) ;  /* pointer to realloc */
    void (*free_memory) (void *) ;		/* pointer to free */
    void *(*calloc_memory) (size_t, size_t) ;	/* pointer to calloc */

} UFconfig ;

void *UFmalloc              /* pointer to allocated block of memory */
(
    size_t nitems,          /* number of items to malloc (>=1 is enforced) */
    size_t size_of_item,    /* sizeof each item */
    int *ok,                /* TRUE if successful, FALSE otherwise */
    UFconfig *config        /* SuiteSparse-wide configuration */
) ;

void *UFfree                /* always returns NULL */
(
    void *p,                /* block to free */
    UFconfig *config        /* SuiteSparse-wide configuration */
) ;


/* ========================================================================== */
/* === SuiteSparse version ================================================== */
/* ========================================================================== */

/* SuiteSparse is not a package itself, but a collection of packages, some of
 * which must be used together (UMFPACK requires AMD, CHOLMOD requires AMD,
 * COLAMD, CAMD, and CCOLAMD, etc).  A version number is provided here for the
 * collection itself.  The versions of packages within each version of
 * SuiteSparse are meant to work together.  Combining one packge from one
 * version of SuiteSparse, with another package from another version of
 * SuiteSparse, may or may not work.
 *
 * SuiteSparse Version 3.6.1 contains the following packages:
 *
 *  AMD		    version 2.2.2
 *  BTF		    version 1.1.2
 *  CAMD	    version 2.2.2
 *  CCOLAMD	    version 2.7.3
 *  CHOLMOD	    version 1.7.3
 *  COLAMD	    version 2.7.3
 *  CSparse	    version 2.2.5
 *  CSparse3	    version 3.0.1
 *  CXSparse	    version 2.2.5
 *  KLU		    version 1.1.2
 *  LDL		    version 2.0.3
 *  RBio	    version 2.0.1
 *  SPQR            version 1.2.2 (also called SuiteSparseQR)
 *  UFcollection    version 1.5.0
 *  UFconfig	    version number is the same as SuiteSparse
 *  UMFPACK	    version 5.5.1
 *  LINFACTOR       version 1.1.0
 *  MESHND          version 1.1.1
 *  SSMULT          version 2.0.2
 *  MATLAB_Tools    no specific version number
 *
 * Other package dependencies:
 *  BLAS	    required by CHOLMOD and UMFPACK
 *  LAPACK	    required by CHOLMOD
 *  METIS 4.0.1	    required by CHOLMOD (optional) and KLU (optional)
 */

#define SUITESPARSE_DATE "May 10, 2011"
#define SUITESPARSE_VER_CODE(main,sub) ((main) * 1000 + (sub))
#define SUITESPARSE_MAIN_VERSION 3
#define SUITESPARSE_SUB_VERSION 6
#define SUITESPARSE_SUBSUB_VERSION 1
#define SUITESPARSE_VERSION \
    SUITESPARSE_VER_CODE(SUITESPARSE_MAIN_VERSION,SUITESPARSE_SUB_VERSION)

#ifdef __cplusplus
}
#endif
#endif
