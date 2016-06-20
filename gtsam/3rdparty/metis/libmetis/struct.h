/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * struct.h
 *
 * This file contains data structures for ILU routines.
 *
 * Started 9/26/95
 * George
 *
 * $Id: struct.h 13900 2013-03-24 15:27:07Z karypis $
 */

#ifndef _LIBMETIS_STRUCT_H_
#define _LIBMETIS_STRUCT_H_



/*************************************************************************/
/*! This data structure stores cut-based k-way refinement info about an
    adjacent subdomain for a given vertex. */
/*************************************************************************/
typedef struct cnbr_t {
  idx_t pid;            /*!< The partition ID */
  idx_t ed;             /*!< The sum of the weights of the adjacent edges
                             that are incident on pid */
} cnbr_t;


/*************************************************************************/
/*! The following data structure stores holds information on degrees for k-way
    partition */
/*************************************************************************/
typedef struct ckrinfo_t {
 idx_t id;              /*!< The internal degree of a vertex (sum of weights) */
 idx_t ed;            	/*!< The total external degree of a vertex */
 idx_t nnbrs;          	/*!< The number of neighboring subdomains */
 idx_t inbr;            /*!< The index in the cnbr_t array where the nnbrs list 
                             of neighbors is stored */
} ckrinfo_t;


/*************************************************************************/
/*! This data structure stores volume-based k-way refinement info about an
    adjacent subdomain for a given vertex. */
/*************************************************************************/
typedef struct vnbr_t {
  idx_t pid;            /*!< The partition ID */
  idx_t ned;            /*!< The number of the adjacent edges
                             that are incident on pid */
  idx_t gv;             /*!< The gain in volume achieved by moving the
                             vertex to pid */
} vnbr_t;


/*************************************************************************/
/*! The following data structure holds information on degrees for k-way
    vol-based partition */
/*************************************************************************/
typedef struct vkrinfo_t {
 idx_t nid;             /*!< The internal degree of a vertex (count of edges) */
 idx_t ned;            	/*!< The total external degree of a vertex (count of edges) */
 idx_t gv;            	/*!< The volume gain of moving that vertex */
 idx_t nnbrs;          	/*!< The number of neighboring subdomains */
 idx_t inbr;            /*!< The index in the vnbr_t array where the nnbrs list 
                             of neighbors is stored */
} vkrinfo_t;


/*************************************************************************/
/*! The following data structure holds information on degrees for k-way
    partition */
/*************************************************************************/
typedef struct nrinfo_t {
 idx_t edegrees[2];  
} nrinfo_t;


/*************************************************************************/
/*! This data structure holds a graph */
/*************************************************************************/
typedef struct graph_t {
  idx_t nvtxs, nedges;	/* The # of vertices and edges in the graph */
  idx_t ncon;		/* The # of constrains */ 
  idx_t *xadj;		/* Pointers to the locally stored vertices */
  idx_t *vwgt;		/* Vertex weights */
  idx_t *vsize;		/* Vertex sizes for min-volume formulation */
  idx_t *adjncy;        /* Array that stores the adjacency lists of nvtxs */
  idx_t *adjwgt;        /* Array that stores the weights of the adjacency lists */

  idx_t *tvwgt;         /* The sum of the vertex weights in the graph */
  real_t *invtvwgt;     /* The inverse of the sum of the vertex weights in the graph */


  /* These are to keep track control if the corresponding fields correspond to
     application or library memory */
  int free_xadj, free_vwgt, free_vsize, free_adjncy, free_adjwgt;

  idx_t *label;

  idx_t *cmap;

  /* Partition parameters */
  idx_t mincut, minvol;
  idx_t *where, *pwgts;
  idx_t nbnd;
  idx_t *bndptr, *bndind;

  /* Bisection refinement parameters */
  idx_t *id, *ed;

  /* K-way refinement parameters */
  ckrinfo_t *ckrinfo;   /*!< The per-vertex cut-based refinement info */
  vkrinfo_t *vkrinfo;   /*!< The per-vertex volume-based refinement info */

  /* Node refinement information */
  nrinfo_t *nrinfo;

  struct graph_t *coarser, *finer;
} graph_t;


/*************************************************************************/
/*! This data structure holds a mesh */
/*************************************************************************/
typedef struct mesh_t {
  idx_t ne, nn;	        /*!< The # of elements and nodes in the mesh */
  idx_t ncon;           /*!< The number of element balancing constraints (element weights) */

  idx_t *eptr, *eind;   /*!< The CSR-structure storing the nodes in the elements */
  idx_t *ewgt;          /*!< The weights of the elements */
} mesh_t;



/*************************************************************************/
/*! The following structure stores information used by Metis */
/*************************************************************************/
typedef struct ctrl_t {
  moptype_et  optype;	        /* Type of operation */
  mobjtype_et objtype;          /* Type of refinement objective */
  mdbglvl_et  dbglvl;		/* Controls the debuging output of the program */
  mctype_et   ctype;		/* The type of coarsening */
  miptype_et  iptype;		/* The type of initial partitioning */
  mrtype_et   rtype;		/* The type of refinement */

  idx_t CoarsenTo;		/* The # of vertices in the coarsest graph */
  idx_t nIparts;                /* The number of initial partitions to compute */
  idx_t no2hop;                 /* Indicates if 2-hop matching will be used */
  idx_t minconn;                /* Indicates if the subdomain connectivity will be minimized */
  idx_t contig;                 /* Indicates if contigous partitions are required */
  idx_t nseps;			/* The number of separators to be found during multiple bisections */
  idx_t ufactor;                /* The user-supplied load imbalance factor */
  idx_t compress;               /* If the graph will be compressed prior to ordering */
  idx_t ccorder;                /* If connected components will be ordered separately */
  idx_t seed;                   /* The seed for the random number generator */
  idx_t ncuts;                  /* The number of different partitionings to compute */
  idx_t niter;                  /* The number of iterations during each refinement */
  idx_t numflag;                /* The user-supplied numflag for the graph */
  idx_t *maxvwgt;		/* The maximum allowed weight for a vertex */

  idx_t ncon;                   /*!< The number of balancing constraints */
  idx_t nparts;                 /*!< The number of partitions */

  real_t pfactor;		/* .1*(user-supplied prunning factor) */

  real_t *ubfactors;            /*!< The per-constraint ubfactors */
  
  real_t *tpwgts;               /*!< The target partition weights */
  real_t *pijbm;                /*!< The nparts*ncon multiplies for the ith partition
                                     and jth constraint for obtaining the balance */

  real_t cfactor;               /*!< The achieved compression factor */

  /* Various Timers */
  double TotalTmr, InitPartTmr, MatchTmr, ContractTmr, CoarsenTmr, UncoarsenTmr, 
         RefTmr, ProjectTmr, SplitTmr, Aux1Tmr, Aux2Tmr, Aux3Tmr;

  /* Workspace information */
  gk_mcore_t *mcore;    /*!< The persistent memory core for within function 
                             mallocs/frees */

  /* These are for use by the k-way refinement routines */
  size_t nbrpoolsize;      /*!< The number of {c,v}nbr_t entries that have been allocated */
  size_t nbrpoolcpos;      /*!< The position of the first free entry in the array */
  size_t nbrpoolreallocs;  /*!< The number of times the pool was resized */

  cnbr_t *cnbrpool;     /*!< The pool of cnbr_t entries to be used during refinement.
                             The size and current position of the pool is controlled
                             by nnbrs & cnbrs */
  vnbr_t *vnbrpool;     /*!< The pool of vnbr_t entries to be used during refinement.
                             The size and current position of the pool is controlled
                             by nnbrs & cnbrs */

  /* The subdomain graph, in sparse format  */ 
  idx_t *maxnads;               /* The maximum allocated number of adjacent domains */
  idx_t *nads;                  /* The number of adjacent domains */
  idx_t **adids;                /* The IDs of the adjacent domains */
  idx_t **adwgts;               /* The edge-weight to the adjacent domains */
  idx_t *pvec1, *pvec2;         /* Auxiliar nparts-size vectors for efficiency */

} ctrl_t;



#endif
