/*!
\file 
\brief Functions dealing with memory allocation and workspace management

\date Started 2/24/96
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version $Id: wspace.c 10492 2011-07-06 09:28:42Z karypis $
*/

#include "metislib.h"


/*************************************************************************/
/*! This function allocates memory for the workspace */
/*************************************************************************/
void AllocateWorkSpace(ctrl_t *ctrl, graph_t *graph)
{
  size_t coresize;

  switch (ctrl->optype) {
    case METIS_OP_PMETIS:
      coresize = 3*(graph->nvtxs+1)*sizeof(idx_t) + 
                 5*(ctrl->nparts+1)*graph->ncon*sizeof(idx_t) + 
                 5*(ctrl->nparts+1)*graph->ncon*sizeof(real_t);
      break;
    default:
      coresize = 4*(graph->nvtxs+1)*sizeof(idx_t) + 
                 5*(ctrl->nparts+1)*graph->ncon*sizeof(idx_t) + 
                 5*(ctrl->nparts+1)*graph->ncon*sizeof(real_t);
  }
  /*coresize = 0;*/
  ctrl->mcore = gk_mcoreCreate(coresize);

  ctrl->nbrpoolsize = 0;
  ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function allocates refinement-specific memory for the workspace */
/*************************************************************************/
void AllocateRefinementWorkSpace(ctrl_t *ctrl, idx_t nbrpoolsize)
{
  ctrl->nbrpoolsize     = nbrpoolsize;
  ctrl->nbrpoolcpos     = 0;
  ctrl->nbrpoolreallocs = 0;

  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      ctrl->cnbrpool = (cnbr_t *)gk_malloc(ctrl->nbrpoolsize*sizeof(cnbr_t), 
                             "AllocateRefinementWorkSpace: cnbrpool");
      break;

    case METIS_OBJTYPE_VOL:
      ctrl->vnbrpool = (vnbr_t *)gk_malloc(ctrl->nbrpoolsize*sizeof(vnbr_t), 
                             "AllocateRefinementWorkSpace: vnbrpool");
      break;

    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }


  /* Allocate the memory for the sparse subdomain graph */
  if (ctrl->minconn) {
    ctrl->pvec1   = imalloc(ctrl->nparts+1, "AllocateRefinementWorkSpace: pvec1");
    ctrl->pvec2   = imalloc(ctrl->nparts+1, "AllocateRefinementWorkSpace: pvec2");
    ctrl->maxnads = ismalloc(ctrl->nparts, INIT_MAXNAD, "AllocateRefinementWorkSpace: maxnads");
    ctrl->nads    = imalloc(ctrl->nparts, "AllocateRefinementWorkSpace: nads");
    ctrl->adids   = iAllocMatrix(ctrl->nparts, INIT_MAXNAD, 0, "AllocateRefinementWorkSpace: adids");
    ctrl->adwgts  = iAllocMatrix(ctrl->nparts, INIT_MAXNAD, 0, "AllocateRefinementWorkSpace: adwgts");
  }
}


/*************************************************************************/
/*! This function frees the workspace */
/*************************************************************************/
void FreeWorkSpace(ctrl_t *ctrl)
{
  gk_mcoreDestroy(&ctrl->mcore, ctrl->dbglvl&METIS_DBG_INFO);

  IFSET(ctrl->dbglvl, METIS_DBG_INFO,
      printf(" nbrpool statistics\n" 
             "        nbrpoolsize: %12zu   nbrpoolcpos: %12zu\n"
             "    nbrpoolreallocs: %12zu\n\n",
             ctrl->nbrpoolsize,  ctrl->nbrpoolcpos, 
             ctrl->nbrpoolreallocs));

  gk_free((void **)&ctrl->cnbrpool, &ctrl->vnbrpool, LTERM);
  ctrl->nbrpoolsize = 0;
  ctrl->nbrpoolcpos = 0;

  if (ctrl->minconn) {
    iFreeMatrix(&(ctrl->adids),  ctrl->nparts, INIT_MAXNAD);
    iFreeMatrix(&(ctrl->adwgts), ctrl->nparts, INIT_MAXNAD);

    gk_free((void **)&ctrl->pvec1, &ctrl->pvec2, 
        &ctrl->maxnads, &ctrl->nads, LTERM);
  }
}


/*************************************************************************/
/*! This function allocate space from the workspace/heap */
/*************************************************************************/
void *wspacemalloc(ctrl_t *ctrl, size_t nbytes)
{
  return gk_mcoreMalloc(ctrl->mcore, nbytes);
}


/*************************************************************************/
/*! This function sets a marker in the stack of malloc ops to be used
    subsequently for freeing purposes */
/*************************************************************************/
void wspacepush(ctrl_t *ctrl)
{
  gk_mcorePush(ctrl->mcore);
}


/*************************************************************************/
/*! This function frees all mops since the last push */
/*************************************************************************/
void wspacepop(ctrl_t *ctrl)
{
  gk_mcorePop(ctrl->mcore);
}


/*************************************************************************/
/*! This function allocate space from the core  */
/*************************************************************************/
idx_t *iwspacemalloc(ctrl_t *ctrl, idx_t n)
{
  return (idx_t *)wspacemalloc(ctrl, n*sizeof(idx_t));
}


/*************************************************************************/
/*! This function allocate space from the core */
/*************************************************************************/
real_t *rwspacemalloc(ctrl_t *ctrl, idx_t n)
{
  return (real_t *)wspacemalloc(ctrl, n*sizeof(real_t));
}


/*************************************************************************/
/*! This function allocate space from the core  */
/*************************************************************************/
ikv_t *ikvwspacemalloc(ctrl_t *ctrl, idx_t n)
{
  return (ikv_t *)wspacemalloc(ctrl, n*sizeof(ikv_t));
}


/*************************************************************************/
/*! This function resets the cnbrpool */
/*************************************************************************/
void cnbrpoolReset(ctrl_t *ctrl)
{
  ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function gets the next free index from cnbrpool */
/*************************************************************************/
idx_t cnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs)
{
  ctrl->nbrpoolcpos += nnbrs;

  if (ctrl->nbrpoolcpos > ctrl->nbrpoolsize) {
    ctrl->nbrpoolsize += gk_max(10*nnbrs, ctrl->nbrpoolsize/2);

    ctrl->cnbrpool = (cnbr_t *)gk_realloc(ctrl->cnbrpool,  
                          ctrl->nbrpoolsize*sizeof(cnbr_t), "cnbrpoolGet: cnbrpool");
    ctrl->nbrpoolreallocs++;
  }

  return ctrl->nbrpoolcpos - nnbrs;
}


/*************************************************************************/
/*! This function resets the vnbrpool */
/*************************************************************************/
void vnbrpoolReset(ctrl_t *ctrl)
{
  ctrl->nbrpoolcpos = 0;
}


/*************************************************************************/
/*! This function gets the next free index from vnbrpool */
/*************************************************************************/
idx_t vnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs)
{
  ctrl->nbrpoolcpos += nnbrs;

  if (ctrl->nbrpoolcpos > ctrl->nbrpoolsize) {
    ctrl->nbrpoolsize += gk_max(10*nnbrs, ctrl->nbrpoolsize/2);

    ctrl->vnbrpool = (vnbr_t *)gk_realloc(ctrl->vnbrpool,  
                          ctrl->nbrpoolsize*sizeof(vnbr_t), "vnbrpoolGet: vnbrpool");
    ctrl->nbrpoolreallocs++;
  }

  return ctrl->nbrpoolcpos - nnbrs;
}

