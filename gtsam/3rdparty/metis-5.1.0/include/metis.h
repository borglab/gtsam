/*!
\file metis.h 
\brief This file contains function prototypes and constant definitions for METIS
 *
\author George
\date   Started 8/9/02
\version\verbatim $Id$\endverbatim
*/

#ifndef _METIS_H_
#define _METIS_H_ 

/****************************************************************************
* A set of defines that can be modified by the user
*****************************************************************************/

/*--------------------------------------------------------------------------
 Specifies the width of the elementary data type that will hold information
 about vertices and their adjacency lists.

 Possible values:
   32 : Use 32 bit signed integers
   64 : Use 64 bit signed integers

 A width of 64 should be specified if the number of vertices or the total
 number of edges in the graph exceed the limits of a 32 bit signed integer
 i.e., 2^31-1.
 Proper use of 64 bit integers requires that the c99 standard datatypes
 int32_t and int64_t are supported by the compiler.
 GCC does provides these definitions in stdint.h, but it may require some
 modifications on other architectures.
--------------------------------------------------------------------------*/
#define IDXTYPEWIDTH 32


/*--------------------------------------------------------------------------
 Specifies the data type that will hold floating-point style information.

 Possible values:
   32 : single precission floating point (float)
   64 : double precission floating point (double)
--------------------------------------------------------------------------*/
#define REALTYPEWIDTH 32



/****************************************************************************
* In principle, nothing needs to be changed beyond this point, unless the
* int32_t and int64_t cannot be found in the normal places.
*****************************************************************************/

/* Uniform definitions for various compilers */
#if defined(_MSC_VER)
  #define COMPILER_MSC
#endif
#if defined(__ICC)
  #define COMPILER_ICC
#endif
#if defined(__GNUC__)
  #define COMPILER_GCC
#endif

/* Include c99 int definitions and need constants. When building the library,
 * these are already defined by GKlib; hence the test for _GKLIB_H_ */
#ifndef _GKLIB_H_
#ifdef COMPILER_MSC
#include <limits.h>

typedef __int32 int32_t;
typedef __int64 int64_t;
#define PRId32       "I32d"
#define PRId64       "I64d"
#define SCNd32       "ld"
#define SCNd64       "I64d"
#define INT32_MIN    ((int32_t)_I32_MIN)
#define INT32_MAX    _I32_MAX
#define INT64_MIN    ((int64_t)_I64_MIN)
#define INT64_MAX    _I64_MAX
#else
#include <inttypes.h>
#endif
#endif


/*------------------------------------------------------------------------
* Setup the basic datatypes
*-------------------------------------------------------------------------*/
#if IDXTYPEWIDTH == 32
  typedef int32_t idx_t;

  #define IDX_MAX   INT32_MAX
  #define IDX_MIN   INT32_MIN

  #define SCIDX  SCNd32
  #define PRIDX  PRId32

  #define strtoidx      strtol
  #define iabs          abs
#elif IDXTYPEWIDTH == 64
  typedef int64_t idx_t;

  #define IDX_MAX   INT64_MAX
  #define IDX_MIN   INT64_MIN

  #define SCIDX  SCNd64
  #define PRIDX  PRId64

#ifdef COMPILER_MSC
  #define strtoidx      _strtoi64
#else
  #define strtoidx      strtoll
#endif
  #define iabs          labs
#else
  #error "Incorrect user-supplied value fo IDXTYPEWIDTH"
#endif


#if REALTYPEWIDTH == 32
  typedef float real_t;

  #define SCREAL         "f"
  #define PRREAL         "f"
  #define REAL_MAX       FLT_MAX
  #define REAL_MIN       FLT_MIN
  #define REAL_EPSILON   FLT_EPSILON

  #define rabs          fabsf
  #define REALEQ(x,y) ((rabs((x)-(y)) <= FLT_EPSILON))

#ifdef COMPILER_MSC
  #define strtoreal     (float)strtod
#else
  #define strtoreal     strtof
#endif
#elif REALTYPEWIDTH == 64
  typedef double real_t;

  #define SCREAL         "lf"
  #define PRREAL         "lf"
  #define REAL_MAX       DBL_MAX
  #define REAL_MIN       DBL_MIN
  #define REAL_EPSILON   DBL_EPSILON

  #define rabs          fabs
  #define REALEQ(x,y) ((rabs((x)-(y)) <= DBL_EPSILON))

  #define strtoreal     strtod
#else
  #error "Incorrect user-supplied value for REALTYPEWIDTH"
#endif


/*------------------------------------------------------------------------
* Constant definitions 
*-------------------------------------------------------------------------*/
/* Metis's version number */
#define METIS_VER_MAJOR         5
#define METIS_VER_MINOR         1
#define METIS_VER_SUBMINOR      0

/* The maximum length of the options[] array */
#define METIS_NOPTIONS          40



/*------------------------------------------------------------------------
* Function prototypes 
*-------------------------------------------------------------------------*/

#ifdef _WINDLL
#define METIS_API(type) __declspec(dllexport) type __cdecl
#elif defined(__cdecl)
#define METIS_API(type) type __cdecl
#else
#define METIS_API(type) type
#endif



#ifdef __cplusplus
extern "C" {
#endif

METIS_API(int) METIS_PartGraphRecursive(idx_t *nvtxs, idx_t *ncon, idx_t *xadj, 
                  idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt, 
                  idx_t *nparts, real_t *tpwgts, real_t *ubvec, idx_t *options, 
                  idx_t *edgecut, idx_t *part);

METIS_API(int) METIS_PartGraphKway(idx_t *nvtxs, idx_t *ncon, idx_t *xadj, 
                  idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt, 
                  idx_t *nparts, real_t *tpwgts, real_t *ubvec, idx_t *options, 
                  idx_t *edgecut, idx_t *part);

METIS_API(int) METIS_MeshToDual(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind, 
                  idx_t *ncommon, idx_t *numflag, idx_t **r_xadj, idx_t **r_adjncy);

METIS_API(int) METIS_MeshToNodal(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind, 
                  idx_t *numflag, idx_t **r_xadj, idx_t **r_adjncy);

METIS_API(int) METIS_PartMeshNodal(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind,
                  idx_t *vwgt, idx_t *vsize, idx_t *nparts, real_t *tpwgts, 
                  idx_t *options, idx_t *objval, idx_t *epart, idx_t *npart);

METIS_API(int) METIS_PartMeshDual(idx_t *ne, idx_t *nn, idx_t *eptr, idx_t *eind,
                  idx_t *vwgt, idx_t *vsize, idx_t *ncommon, idx_t *nparts, 
                  real_t *tpwgts, idx_t *options, idx_t *objval, idx_t *epart, 
                  idx_t *npart);

METIS_API(int) METIS_NodeND(idx_t *nvtxs, idx_t *xadj, idx_t *adjncy, idx_t *vwgt,
                  idx_t *options, idx_t *perm, idx_t *iperm);

METIS_API(int) METIS_Free(void *ptr);

METIS_API(int) METIS_SetDefaultOptions(idx_t *options);


/* These functions are used by ParMETIS */

METIS_API(int) METIS_NodeNDP(idx_t nvtxs, idx_t *xadj, idx_t *adjncy, idx_t *vwgt,
                   idx_t npes, idx_t *options, idx_t *perm, idx_t *iperm, 
                   idx_t *sizes);

METIS_API(int) METIS_ComputeVertexSeparator(idx_t *nvtxs, idx_t *xadj, idx_t *adjncy, 
                   idx_t *vwgt, idx_t *options, idx_t *sepsize, idx_t *part);

METIS_API(int) METIS_NodeRefine(idx_t nvtxs, idx_t *xadj, idx_t *vwgt, idx_t *adjncy,
                   idx_t *where, idx_t *hmarker, real_t ubfactor);


#ifdef __cplusplus
}
#endif



/*------------------------------------------------------------------------
* Enum type definitions 
*-------------------------------------------------------------------------*/
/*! Return codes */
typedef enum {
  METIS_OK              = 1,    /*!< Returned normally */
  METIS_ERROR_INPUT     = -2,   /*!< Returned due to erroneous inputs and/or options */
  METIS_ERROR_MEMORY    = -3,   /*!< Returned due to insufficient memory */
  METIS_ERROR           = -4    /*!< Some other errors */
} rstatus_et; 


/*! Operation type codes */
typedef enum {
  METIS_OP_PMETIS,       
  METIS_OP_KMETIS,
  METIS_OP_OMETIS
} moptype_et;


/*! Options codes (i.e., options[]) */
typedef enum {
  METIS_OPTION_PTYPE,
  METIS_OPTION_OBJTYPE,
  METIS_OPTION_CTYPE,
  METIS_OPTION_IPTYPE,
  METIS_OPTION_RTYPE,
  METIS_OPTION_DBGLVL,
  METIS_OPTION_NITER,
  METIS_OPTION_NCUTS,
  METIS_OPTION_SEED,
  METIS_OPTION_NO2HOP,
  METIS_OPTION_MINCONN,
  METIS_OPTION_CONTIG,
  METIS_OPTION_COMPRESS,
  METIS_OPTION_CCORDER,
  METIS_OPTION_PFACTOR,
  METIS_OPTION_NSEPS,
  METIS_OPTION_UFACTOR,
  METIS_OPTION_NUMBERING,

  /* Used for command-line parameter purposes */
  METIS_OPTION_HELP,
  METIS_OPTION_TPWGTS,
  METIS_OPTION_NCOMMON,
  METIS_OPTION_NOOUTPUT,
  METIS_OPTION_BALANCE,
  METIS_OPTION_GTYPE,
  METIS_OPTION_UBVEC
} moptions_et;


/*! Partitioning Schemes */
typedef enum {
  METIS_PTYPE_RB, 
  METIS_PTYPE_KWAY                
} mptype_et;

/*! Graph types for meshes */
typedef enum {
  METIS_GTYPE_DUAL,
  METIS_GTYPE_NODAL               
} mgtype_et;

/*! Coarsening Schemes */
typedef enum {
  METIS_CTYPE_RM,
  METIS_CTYPE_SHEM
} mctype_et;

/*! Initial partitioning schemes */
typedef enum {
  METIS_IPTYPE_GROW,
  METIS_IPTYPE_RANDOM,
  METIS_IPTYPE_EDGE,
  METIS_IPTYPE_NODE,
  METIS_IPTYPE_METISRB
} miptype_et;


/*! Refinement schemes */
typedef enum {
  METIS_RTYPE_FM,
  METIS_RTYPE_GREEDY,
  METIS_RTYPE_SEP2SIDED,
  METIS_RTYPE_SEP1SIDED
} mrtype_et;


/*! Debug Levels */
typedef enum {
  METIS_DBG_INFO       = 1,       /*!< Shows various diagnostic messages */
  METIS_DBG_TIME       = 2,       /*!< Perform timing analysis */
  METIS_DBG_COARSEN    = 4,	  /*!< Show the coarsening progress */
  METIS_DBG_REFINE     = 8,	  /*!< Show the refinement progress */
  METIS_DBG_IPART      = 16, 	  /*!< Show info on initial partitioning */
  METIS_DBG_MOVEINFO   = 32, 	  /*!< Show info on vertex moves during refinement */
  METIS_DBG_SEPINFO    = 64, 	  /*!< Show info on vertex moves during sep refinement */
  METIS_DBG_CONNINFO   = 128,     /*!< Show info on minimization of subdomain connectivity */
  METIS_DBG_CONTIGINFO = 256,     /*!< Show info on elimination of connected components */ 
  METIS_DBG_MEMORY     = 2048,    /*!< Show info related to wspace allocation */
} mdbglvl_et;


/* Types of objectives */
typedef enum {
  METIS_OBJTYPE_CUT,
  METIS_OBJTYPE_VOL,
  METIS_OBJTYPE_NODE
} mobjtype_et;



#endif  /* _METIS_H_ */
