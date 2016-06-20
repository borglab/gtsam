/*!
\file gk_struct.h
\brief This file contains various datastructures used/provided by GKlib

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_struct.h 13005 2012-10-23 22:34:36Z karypis $ \endverbatim
*/

#ifndef _GK_STRUCT_H_
#define _GK_STRUCT_H_


/********************************************************************/
/*! Generator for gk_??KeyVal_t data structure */
/********************************************************************/
#define GK_MKKEYVALUE_T(NAME, KEYTYPE, VALTYPE) \
typedef struct {\
  KEYTYPE key;\
  VALTYPE val;\
} NAME;\

/* The actual KeyVal data structures */
GK_MKKEYVALUE_T(gk_ckv_t,   char,     ssize_t)
GK_MKKEYVALUE_T(gk_ikv_t,   int,      ssize_t)
GK_MKKEYVALUE_T(gk_i32kv_t, int32_t,  ssize_t)
GK_MKKEYVALUE_T(gk_i64kv_t, int64_t,  ssize_t)
GK_MKKEYVALUE_T(gk_zkv_t,   ssize_t,  ssize_t)
GK_MKKEYVALUE_T(gk_fkv_t,   float,    ssize_t)
GK_MKKEYVALUE_T(gk_dkv_t,   double,   ssize_t)
GK_MKKEYVALUE_T(gk_skv_t,   char *,   ssize_t)
GK_MKKEYVALUE_T(gk_idxkv_t, gk_idx_t, gk_idx_t)



/********************************************************************/
/*! Generator for gk_?pq_t data structure */
/********************************************************************/
#define GK_MKPQUEUE_T(NAME, KVTYPE)\
typedef struct {\
  gk_idx_t nnodes;\
  gk_idx_t maxnodes;\
\
  /* Heap version of the data structure */ \
  KVTYPE   *heap;\
  gk_idx_t *locator;\
} NAME;\

GK_MKPQUEUE_T(gk_ipq_t,    gk_ikv_t)
GK_MKPQUEUE_T(gk_i32pq_t,  gk_i32kv_t)
GK_MKPQUEUE_T(gk_i64pq_t,  gk_i64kv_t)
GK_MKPQUEUE_T(gk_fpq_t,    gk_fkv_t)
GK_MKPQUEUE_T(gk_dpq_t,    gk_dkv_t)
GK_MKPQUEUE_T(gk_idxpq_t,  gk_idxkv_t)


#define GK_MKPQUEUE2_T(NAME, KTYPE, VTYPE)\
typedef struct {\
  ssize_t nnodes;\
  ssize_t maxnodes;\
\
  /* Heap version of the data structure */ \
  KTYPE *keys;\
  VTYPE *vals;\
} NAME;\



/*-------------------------------------------------------------
 * The following data structure stores a sparse CSR format
 *-------------------------------------------------------------*/
typedef struct gk_csr_t {
  int32_t nrows, ncols;
  ssize_t *rowptr, *colptr;
  int32_t *rowind, *colind;
  int32_t *rowids, *colids;
  float *rowval, *colval;
  float *rnorms, *cnorms;
  float *rsums, *csums;
  float *rsizes, *csizes;
  float *rvols, *cvols;
  float *rwgts, *cwgts;
} gk_csr_t;


/*-------------------------------------------------------------
 * The following data structure stores a sparse graph 
 *-------------------------------------------------------------*/
typedef struct gk_graph_t {
  int32_t nvtxs;                /*!< The number of vertices in the graph */
  ssize_t *xadj;                /*!< The ptr-structure of the adjncy list */
  int32_t *adjncy;              /*!< The adjacency list of the graph */
  int32_t *iadjwgt;             /*!< The integer edge weights */
  float *fadjwgt;               /*!< The floating point edge weights */
  int32_t *ivwgts;              /*!< The integer vertex weights */
  float *fvwgts;                /*!< The floating point vertex weights */
  int32_t *ivsizes;             /*!< The integer vertex sizes */
  float *fvsizes;               /*!< The floating point vertex sizes */
  int32_t *vlabels;             /*!< The labels of the vertices */
} gk_graph_t;


/*-------------------------------------------------------------
 * The following data structure stores stores a string as a 
 * pair of its allocated buffer and the buffer itself.
 *-------------------------------------------------------------*/
typedef struct gk_str_t {
  size_t len;
  char *buf;
} gk_str_t;




/*-------------------------------------------------------------
* The following data structure implements a string-2-int mapping
* table used for parsing command-line options
*-------------------------------------------------------------*/
typedef struct gk_StringMap_t {
  char *name;
  int id;
} gk_StringMap_t;


/*------------------------------------------------------------
 * This structure implements a simple hash table
 *------------------------------------------------------------*/
typedef struct gk_HTable_t {
  int nelements;          /* The overall size of the hash-table */
  int htsize;             /* The current size of the hash-table */
  gk_ikv_t *harray;       /* The actual hash-table */
} gk_HTable_t;


/*------------------------------------------------------------
 * This structure implements a gk_Tokens_t list returned by the
 * string tokenizer
 *------------------------------------------------------------*/
typedef struct gk_Tokens_t {
  int ntoks;        /* The number of tokens in the input string */
  char *strbuf;     /* The memory that stores all the entries */
  char **list;      /* Pointers to the strbuf for each element */
} gk_Tokens_t;

/*------------------------------------------------------------
 * This structure implements storage for an atom in a pdb file
 *------------------------------------------------------------*/
typedef struct atom {
  int       serial;
  char      *name;
  char	    altLoc;
  char      *resname;
  char      chainid;	
  int       rserial;
  char	    icode;
  char      element;
  double    x;
  double    y;
  double    z;
  double    opcy;
  double    tmpt;
} atom;


/*------------------------------------------------------------
 * This structure implements storage for a center of mass for
 * a single residue.
 *------------------------------------------------------------*/
typedef struct center_of_mass {
  char name;
  double x;
  double y;
  double z;
} center_of_mass;


/*------------------------------------------------------------
 * This structure implements storage for a pdb protein 
 *------------------------------------------------------------*/
typedef struct pdbf {
	int natoms;			/* Number of atoms */
	int nresidues;  /* Number of residues based on coordinates */
	int ncas;
	int nbbs;
	int corruption;
	char *resSeq;	      /* Residue sequence based on coordinates    */
  char **threeresSeq; /* three-letter residue sequence */
	atom *atoms;
	atom **bbs;
	atom **cas;
  center_of_mass *cm;
} pdbf;



/*************************************************************
* Localization Structures for converting characters to integers
**************************************************************/
typedef struct gk_i2cc2i_t {
    int n;
    char *i2c;
    int *c2i;
} gk_i2cc2i_t;
 

/*******************************************************************
 *This structure implements storage of a protein sequence
 * *****************************************************************/
typedef struct gk_seq_t {
    
    int len; /*Number of Residues */
    int *sequence; /* Stores the sequence*/
    
    
    int **pssm; /* Stores the pssm matrix */
    int **psfm; /* Stores the psfm matrix */
    char *name; /* Stores the name of the sequence */

    int nsymbols;

    
} gk_seq_t;




/*************************************************************************/
/*! The following data structure stores information about a memory 
    allocation operation that can either be served from gk_mcore_t or by
    a gk_malloc if not sufficient workspace memory is available. */
/*************************************************************************/
typedef struct gk_mop_t {
  int type;
  ssize_t nbytes;
  void *ptr;
} gk_mop_t;


/*************************************************************************/
/*! The following structure stores information used by Metis */
/*************************************************************************/
typedef struct gk_mcore_t {
  /* Workspace information */
  size_t coresize;     /*!< The amount of core memory that has been allocated */
  size_t corecpos;     /*!< Index of the first free location in core */
  void *core;	       /*!< Pointer to the core itself */

  /* These are for implementing a stack-based allocation scheme using both
     core and also dynamically allocated memory */
  size_t nmops;         /*!< The number of maop_t entries that have been allocated */
  size_t cmop;          /*!< Index of the first free location in maops */
  gk_mop_t *mops;       /*!< The array recording the maop_t operations */

  /* These are for keeping various statistics for wspacemalloc */
  size_t num_callocs;   /*!< The number of core mallocs */
  size_t num_hallocs;   /*!< The number of heap mallocs */
  size_t size_callocs;  /*!< The total # of bytes in core mallocs */
  size_t size_hallocs;  /*!< The total # of bytes in heap mallocs */
  size_t cur_callocs;   /*!< The current # of bytes in core mallocs */
  size_t cur_hallocs;   /*!< The current # of bytes in heap mallocs */
  size_t max_callocs;   /*!< The maximum # of bytes in core mallocs at any given time */
  size_t max_hallocs;   /*!< The maximum # of bytes in heap mallocs at any given time */

} gk_mcore_t;



#endif
