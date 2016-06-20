/*!
\file
\brief Driving routines for multilevel k-way refinement

\date   Started 7/28/1997
\author George 
\author  Copyright 1997-2009, Regents of the University of Minnesota 
\version $Id: kwayrefine.c 10737 2011-09-13 13:37:25Z karypis $ 
*/

#include "metislib.h"


/*************************************************************************/
/*! This function is the entry point of cut-based refinement */
/*************************************************************************/
void RefineKWay(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph)
{
  idx_t i, nlevels, contig=ctrl->contig;
  graph_t *ptr;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->UncoarsenTmr));

  /* Determine how many levels are there */
  for (ptr=graph, nlevels=0; ptr!=orggraph; ptr=ptr->finer, nlevels++); 

  /* Compute the parameters of the coarsest graph */
  ComputeKWayPartitionParams(ctrl, graph);

  /* Try to minimize the sub-domain connectivity */
  if (ctrl->minconn) 
    EliminateSubDomainEdges(ctrl, graph);
  
  /* Deal with contiguity constraints at the beginning */
  if (contig && FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts) { 
    EliminateComponents(ctrl, graph);

    ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
    Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE); 

    ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
    Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE); 

    ctrl->contig = 0;
  }

  /* Refine each successively finer graph */
  for (i=0; ;i++) {
    if (ctrl->minconn && i == nlevels/2) 
      EliminateSubDomainEdges(ctrl, graph);

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->RefTmr));

    if (2*i >= nlevels && !IsBalanced(ctrl, graph, .02)) {
      ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
      Greedy_KWayOptimize(ctrl, graph, 1, 0, OMODE_BALANCE); 
      ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
    }

    Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 5.0, OMODE_REFINE); 

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->RefTmr));

    /* Deal with contiguity constraints in the middle */
    if (contig && i == nlevels/2) {
      if (FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts) {
        EliminateComponents(ctrl, graph);

        if (!IsBalanced(ctrl, graph, .02)) {
          ctrl->contig = 1;
          ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
          Greedy_KWayOptimize(ctrl, graph, 5, 0, OMODE_BALANCE); 
  
          ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
          Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE); 
          ctrl->contig = 0;
        }
      }
    }

    if (graph == orggraph)
      break;

    graph = graph->finer;

    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ProjectTmr));
    ASSERT(graph->vwgt != NULL);

    ProjectKWayPartition(ctrl, graph);
    IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ProjectTmr));
  }

  /* Deal with contiguity requirement at the end */
  ctrl->contig = contig;
  if (contig && FindPartitionInducedComponents(graph, graph->where, NULL, NULL) > ctrl->nparts) 
    EliminateComponents(ctrl, graph);

  if (!IsBalanced(ctrl, graph, 0.0)) {
    ComputeKWayBoundary(ctrl, graph, BNDTYPE_BALANCE);
    Greedy_KWayOptimize(ctrl, graph, 10, 0, OMODE_BALANCE); 

    ComputeKWayBoundary(ctrl, graph, BNDTYPE_REFINE);
    Greedy_KWayOptimize(ctrl, graph, ctrl->niter, 0, OMODE_REFINE); 
  }

  if (ctrl->contig) 
    ASSERT(FindPartitionInducedComponents(graph, graph->where, NULL, NULL) == ctrl->nparts);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->UncoarsenTmr));
}


/*************************************************************************/
/*! This function allocates memory for the k-way cut-based refinement */
/*************************************************************************/
void AllocateKWayPartitionMemory(ctrl_t *ctrl, graph_t *graph)
{

  graph->pwgts  = imalloc(ctrl->nparts*graph->ncon, "AllocateKWayPartitionMemory: pwgts");
  graph->where  = imalloc(graph->nvtxs,  "AllocateKWayPartitionMemory: where");
  graph->bndptr = imalloc(graph->nvtxs,  "AllocateKWayPartitionMemory: bndptr");
  graph->bndind = imalloc(graph->nvtxs,  "AllocateKWayPartitionMemory: bndind");

  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      graph->ckrinfo  = (ckrinfo_t *)gk_malloc(graph->nvtxs*sizeof(ckrinfo_t), 
                          "AllocateKWayPartitionMemory: ckrinfo");
      break;

    case METIS_OBJTYPE_VOL:
      graph->vkrinfo = (vkrinfo_t *)gk_malloc(graph->nvtxs*sizeof(vkrinfo_t), 
                          "AllocateKWayVolPartitionMemory: vkrinfo");

      /* This is to let the cut-based -minconn and -contig large-scale graph
         changes to go through */
      graph->ckrinfo = (ckrinfo_t *)graph->vkrinfo;
      break;

    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }

}


/*************************************************************************/
/*! This function computes the initial id/ed  for cut-based partitioning */
/**************************************************************************/
void ComputeKWayPartitionParams(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, j, k, l, nvtxs, ncon, nparts, nbnd, mincut, me, other;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *pwgts, *where, *bndind, *bndptr;

  nparts = ctrl->nparts;

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  where  = graph->where;
  pwgts  = iset(nparts*ncon, 0, graph->pwgts);
  bndind = graph->bndind;
  bndptr = iset(nvtxs, -1, graph->bndptr);

  nbnd = mincut = 0;

  /* Compute pwgts */
  if (ncon == 1) {
    for (i=0; i<nvtxs; i++) {
      ASSERT(where[i] >= 0 && where[i] < nparts);
      pwgts[where[i]] += vwgt[i];
    }
  }
  else {
    for (i=0; i<nvtxs; i++) {
      me = where[i];
      for (j=0; j<ncon; j++)
        pwgts[me*ncon+j] += vwgt[i*ncon+j];
    }
  }

  /* Compute the required info for refinement */
  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      {
        ckrinfo_t *myrinfo;
        cnbr_t *mynbrs;

        memset(graph->ckrinfo, 0, sizeof(ckrinfo_t)*nvtxs);
        cnbrpoolReset(ctrl);

        for (i=0; i<nvtxs; i++) {
          me      = where[i];
          myrinfo = graph->ckrinfo+i;

          for (j=xadj[i]; j<xadj[i+1]; j++) {
            if (me == where[adjncy[j]])
              myrinfo->id += adjwgt[j];
            else
              myrinfo->ed += adjwgt[j];
          }

          /* Time to compute the particular external degrees */
          if (myrinfo->ed > 0) {
            mincut += myrinfo->ed;

            myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
            mynbrs        = ctrl->cnbrpool + myrinfo->inbr;

            for (j=xadj[i]; j<xadj[i+1]; j++) {
              other = where[adjncy[j]];
              if (me != other) {
                for (k=0; k<myrinfo->nnbrs; k++) {
                  if (mynbrs[k].pid == other) {
                    mynbrs[k].ed += adjwgt[j];
                    break;
                  }
                }
                if (k == myrinfo->nnbrs) {
                  mynbrs[k].pid = other;
                  mynbrs[k].ed  = adjwgt[j];
                  myrinfo->nnbrs++;
                }
              }
            }

            ASSERT(myrinfo->nnbrs <= xadj[i+1]-xadj[i]);

            /* Only ed-id>=0 nodes are considered to be in the boundary */
            if (myrinfo->ed-myrinfo->id >= 0)
              BNDInsert(nbnd, bndind, bndptr, i);
          }
          else {
            myrinfo->inbr = -1;
          }
        }

        graph->mincut = mincut/2;
        graph->nbnd   = nbnd;

      }
      ASSERT(CheckBnd2(graph));
      break;

    case METIS_OBJTYPE_VOL:
      {
        vkrinfo_t *myrinfo;
        vnbr_t *mynbrs;

        memset(graph->vkrinfo, 0, sizeof(vkrinfo_t)*nvtxs);
        vnbrpoolReset(ctrl);

        /* Compute now the id/ed degrees */
        for (i=0; i<nvtxs; i++) {
          me      = where[i];
          myrinfo = graph->vkrinfo+i;
      
          for (j=xadj[i]; j<xadj[i+1]; j++) {
            if (me == where[adjncy[j]]) 
              myrinfo->nid++;
            else 
              myrinfo->ned++;
          }
      
          /* Time to compute the particular external degrees */
          if (myrinfo->ned > 0) { 
            mincut += myrinfo->ned;

            myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
            mynbrs        = ctrl->vnbrpool + myrinfo->inbr;

            for (j=xadj[i]; j<xadj[i+1]; j++) {
              other = where[adjncy[j]];
              if (me != other) {
                for (k=0; k<myrinfo->nnbrs; k++) {
                  if (mynbrs[k].pid == other) {
                    mynbrs[k].ned++;
                    break;
                  }
                }
                if (k == myrinfo->nnbrs) {
                  mynbrs[k].gv  = 0;
                  mynbrs[k].pid = other;
                  mynbrs[k].ned = 1;
                  myrinfo->nnbrs++;
                }
              }
            }
            ASSERT(myrinfo->nnbrs <= xadj[i+1]-xadj[i]);
          }
          else {
            myrinfo->inbr = -1;
          }
        }
        graph->mincut = mincut/2;
      
        ComputeKWayVolGains(ctrl, graph);
      }
      ASSERT(graph->minvol == ComputeVolume(graph, graph->where));
      break;
    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }

}


/*************************************************************************/
/*! This function projects a partition, and at the same time computes the
 parameters for refinement. */
/*************************************************************************/
void ProjectKWayPartition(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, j, k, nvtxs, nbnd, nparts, me, other, istart, iend, tid, ted;
  idx_t *xadj, *adjncy, *adjwgt;
  idx_t *cmap, *where, *bndptr, *bndind, *cwhere, *htable;
  graph_t *cgraph;

  WCOREPUSH;

  nparts = ctrl->nparts;

  cgraph = graph->coarser;
  cwhere = cgraph->where;

  nvtxs   = graph->nvtxs;
  cmap    = graph->cmap;
  xadj    = graph->xadj;
  adjncy  = graph->adjncy;
  adjwgt  = graph->adjwgt;

  AllocateKWayPartitionMemory(ctrl, graph);

  where  = graph->where;
  bndind = graph->bndind;
  bndptr = iset(nvtxs, -1, graph->bndptr);

  htable = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

  /* Compute the required info for refinement */
  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      ASSERT(CheckBnd2(cgraph));
      {
        ckrinfo_t *myrinfo;
        cnbr_t *mynbrs;

        /* go through and project partition and compute id/ed for the nodes */
        for (i=0; i<nvtxs; i++) {
          k        = cmap[i];
          where[i] = cwhere[k];
          cmap[i]  = cgraph->ckrinfo[k].ed;  /* For optimization */
        }

        memset(graph->ckrinfo, 0, sizeof(ckrinfo_t)*nvtxs);
        cnbrpoolReset(ctrl);

        for (nbnd=0, i=0; i<nvtxs; i++) {
          istart = xadj[i];
          iend   = xadj[i+1];

          myrinfo = graph->ckrinfo+i;

          if (cmap[i] == 0) { /* Interior node. Note that cmap[i] = crinfo[cmap[i]].ed */
            for (tid=0, j=istart; j<iend; j++) 
              tid += adjwgt[j];

            myrinfo->id   = tid;
            myrinfo->inbr = -1;
          }
          else { /* Potentially an interface node */
            myrinfo->inbr = cnbrpoolGetNext(ctrl, iend-istart+1);
            mynbrs        = ctrl->cnbrpool + myrinfo->inbr;

            me = where[i];
            for (tid=0, ted=0, j=istart; j<iend; j++) {
              other = where[adjncy[j]];
              if (me == other) {
                tid += adjwgt[j];
              }
              else {
                ted += adjwgt[j];
                if ((k = htable[other]) == -1) {
                  htable[other]               = myrinfo->nnbrs;
                  mynbrs[myrinfo->nnbrs].pid  = other;
                  mynbrs[myrinfo->nnbrs++].ed = adjwgt[j];
                }
                else {
                  mynbrs[k].ed += adjwgt[j];
                }
              }
            }
            myrinfo->id = tid;
            myrinfo->ed = ted;
      
            /* Remove space for edegrees if it was interior */
            if (ted == 0) { 
              ctrl->nbrpoolcpos -= iend-istart+1;
              myrinfo->inbr      = -1;
            }
            else {
              if (ted-tid >= 0) 
                BNDInsert(nbnd, bndind, bndptr, i); 
      
              for (j=0; j<myrinfo->nnbrs; j++)
                htable[mynbrs[j].pid] = -1;
            }
          }
        }
      
        graph->nbnd = nbnd;

      }
      ASSERT(CheckBnd2(graph));
      break;

    case METIS_OBJTYPE_VOL:
      {
        vkrinfo_t *myrinfo;
        vnbr_t *mynbrs;

        ASSERT(cgraph->minvol == ComputeVolume(cgraph, cgraph->where));

        /* go through and project partition and compute id/ed for the nodes */
        for (i=0; i<nvtxs; i++) {
          k        = cmap[i];
          where[i] = cwhere[k];
          cmap[i]  = cgraph->vkrinfo[k].ned;  /* For optimization */
        }

        memset(graph->vkrinfo, 0, sizeof(vkrinfo_t)*nvtxs);
        vnbrpoolReset(ctrl);

        for (i=0; i<nvtxs; i++) {
          istart = xadj[i];
          iend   = xadj[i+1];
          myrinfo = graph->vkrinfo+i;

          if (cmap[i] == 0) { /* Note that cmap[i] = crinfo[cmap[i]].ed */
            myrinfo->nid  = iend-istart;
            myrinfo->inbr = -1;
          }
          else { /* Potentially an interface node */
            myrinfo->inbr = vnbrpoolGetNext(ctrl, iend-istart+1);
            mynbrs        = ctrl->vnbrpool + myrinfo->inbr;

            me = where[i];
            for (tid=0, ted=0, j=istart; j<iend; j++) {
              other = where[adjncy[j]];
              if (me == other) {
                tid++;
              }
              else {
                ted++;
                if ((k = htable[other]) == -1) {
                  htable[other]                = myrinfo->nnbrs;
                  mynbrs[myrinfo->nnbrs].gv    = 0;
                  mynbrs[myrinfo->nnbrs].pid   = other;
                  mynbrs[myrinfo->nnbrs++].ned = 1;
                }
                else {
                  mynbrs[k].ned++;
                }
              }
            }
            myrinfo->nid = tid;
            myrinfo->ned = ted;
      
            /* Remove space for edegrees if it was interior */
            if (ted == 0) { 
              ctrl->nbrpoolcpos -= iend-istart+1;
              myrinfo->inbr = -1;
            }
            else {
              for (j=0; j<myrinfo->nnbrs; j++)
                htable[mynbrs[j].pid] = -1;
            }
          }
        }
      
        ComputeKWayVolGains(ctrl, graph);

        ASSERT(graph->minvol == ComputeVolume(graph, graph->where));
      }
      break;

    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }

  graph->mincut = cgraph->mincut;
  icopy(nparts*graph->ncon, cgraph->pwgts, graph->pwgts);

  FreeGraph(&graph->coarser);
  graph->coarser = NULL;

  WCOREPOP;
}


/*************************************************************************/
/*! This function computes the boundary definition for balancing. */
/*************************************************************************/
void ComputeKWayBoundary(ctrl_t *ctrl, graph_t *graph, idx_t bndtype)
{
  idx_t i, nvtxs, nbnd;
  idx_t *bndind, *bndptr;

  nvtxs  = graph->nvtxs;
  bndind = graph->bndind;
  bndptr = iset(nvtxs, -1, graph->bndptr);

  nbnd = 0;

  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      /* Compute the boundary */
      if (bndtype == BNDTYPE_REFINE) {
        for (i=0; i<nvtxs; i++) {
          if (graph->ckrinfo[i].ed-graph->ckrinfo[i].id >= 0) 
            BNDInsert(nbnd, bndind, bndptr, i);
        }
      }
      else { /* BNDTYPE_BALANCE */
        for (i=0; i<nvtxs; i++) {
          if (graph->ckrinfo[i].ed > 0) 
            BNDInsert(nbnd, bndind, bndptr, i);
        }
      }
      break;

    case METIS_OBJTYPE_VOL:
      /* Compute the boundary */
      if (bndtype == BNDTYPE_REFINE) {
        for (i=0; i<nvtxs; i++) {
          if (graph->vkrinfo[i].gv >= 0)
            BNDInsert(nbnd, bndind, bndptr, i);
        }
      }
      else { /* BNDTYPE_BALANCE */
        for (i=0; i<nvtxs; i++) {
          if (graph->vkrinfo[i].ned > 0) 
            BNDInsert(nbnd, bndind, bndptr, i);
        }
      }
      break;

    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }

  graph->nbnd = nbnd;
}


/*************************************************************************/
/*! This function computes the initial gains in the communication volume */
/*************************************************************************/
void ComputeKWayVolGains(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, k, l, nvtxs, nparts, me, other, pid; 
  idx_t *xadj, *vsize, *adjncy, *adjwgt, *where, 
        *bndind, *bndptr, *ophtable;
  vkrinfo_t *myrinfo, *orinfo;
  vnbr_t *mynbrs, *onbrs;

  WCOREPUSH;

  nparts = ctrl->nparts;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vsize  = graph->vsize;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  where  = graph->where;
  bndind = graph->bndind;
  bndptr = iset(nvtxs, -1, graph->bndptr);

  ophtable = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

  /* Compute the volume gains */
  graph->minvol = graph->nbnd = 0;
  for (i=0; i<nvtxs; i++) {
    myrinfo     = graph->vkrinfo+i;
    myrinfo->gv = IDX_MIN;

    if (myrinfo->nnbrs > 0) {
      me     = where[i];
      mynbrs = ctrl->vnbrpool + myrinfo->inbr;

      graph->minvol += myrinfo->nnbrs*vsize[i];

      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo+ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;

        for (k=0; k<orinfo->nnbrs; k++) 
          ophtable[onbrs[k].pid] = k;
        ophtable[other] = 1;  /* this is to simplify coding */

        if (me == other) {
          /* Find which domains 'i' is connected to but 'ii' is not 
             and update their gain */
          for (k=0; k<myrinfo->nnbrs; k++) {
            if (ophtable[mynbrs[k].pid] == -1)
              mynbrs[k].gv -= vsize[ii];
          }
        }
        else {
          ASSERT(ophtable[me] != -1);

          if (onbrs[ophtable[me]].ned == 1) { 
            /* I'm the only connection of 'ii' in 'me' */
            /* Increase the gains for all the common domains between 'i' and 'ii' */
            for (k=0; k<myrinfo->nnbrs; k++) {
              if (ophtable[mynbrs[k].pid] != -1) 
                mynbrs[k].gv += vsize[ii];
            }
          }
          else {
            /* Find which domains 'i' is connected to and 'ii' is not 
               and update their gain */
            for (k=0; k<myrinfo->nnbrs; k++) {
              if (ophtable[mynbrs[k].pid] == -1) 
                mynbrs[k].gv -= vsize[ii];
            }
          }
        }

        /* Reset the marker vector */
        for (k=0; k<orinfo->nnbrs; k++) 
          ophtable[onbrs[k].pid] = -1;
        ophtable[other] = -1;
      }

      /* Compute the max vgain */
      for (k=0; k<myrinfo->nnbrs; k++) {
        if (mynbrs[k].gv > myrinfo->gv)
          myrinfo->gv = mynbrs[k].gv;
      }

      /* Add the extra gain due to id == 0 */
      if (myrinfo->ned > 0 && myrinfo->nid == 0)
        myrinfo->gv += vsize[i];
    }

    if (myrinfo->gv >= 0)
      BNDInsert(graph->nbnd, bndind, bndptr, i);
  }

  WCOREPOP;
}


/*************************************************************************/
/*! This function checks if the partition weights are within the balance
contraints */
/*************************************************************************/
int IsBalanced(ctrl_t *ctrl, graph_t *graph, real_t ffactor)
{
  return 
    (ComputeLoadImbalanceDiff(graph, ctrl->nparts, ctrl->pijbm, ctrl->ubfactors) 
         <= ffactor);
}

