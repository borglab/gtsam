/*!
\file 
\brief Functions that deal with eliminating disconnected partitions

\date Started 7/15/98
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version $Id: contig.c 10513 2011-07-07 22:06:03Z karypis $
*/

#include "metislib.h"


/*************************************************************************/
/*! This function finds the connected components induced by the 
    partitioning vector.

    \param graph is the graph structure
    \param where is the partitioning vector. If this is NULL, then the
           entire graph is treated to belong into a single partition.
    \param cptr is the ptr structure of the CSR representation of the 
           components. The length of this vector must be graph->nvtxs+1.
    \param cind is the indices structure of the CSR representation of 
           the components. The length of this vector must be graph->nvtxs.

    \returns the number of components that it found.

    \note The cptr and cind parameters can be NULL, in which case only the
          number of connected components is returned.
*/
/*************************************************************************/
idx_t FindPartitionInducedComponents(graph_t *graph, idx_t *where, 
          idx_t *cptr, idx_t *cind)
{
  idx_t i, ii, j, jj, k, me=0, nvtxs, first, last, nleft, ncmps;
  idx_t *xadj, *adjncy;
  idx_t *touched, *perm, *todo;
  idx_t mustfree_ccsr=0, mustfree_where=0;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  /* Deal with NULL supplied cptr/cind vectors */
  if (cptr == NULL) {
    cptr = imalloc(nvtxs+1, "FindPartitionInducedComponents: cptr");
    cind = imalloc(nvtxs, "FindPartitionInducedComponents: cind");
    mustfree_ccsr = 1;
  }

  /* Deal with NULL supplied where vector */
  if (where == NULL) {
    where = ismalloc(nvtxs, 0, "FindPartitionInducedComponents: where");
    mustfree_where = 1;
  }

  /* Allocate memory required for the BFS traversal */
  perm    = iincset(nvtxs, 0, imalloc(nvtxs, "FindPartitionInducedComponents: perm"));
  todo    = iincset(nvtxs, 0, imalloc(nvtxs, "FindPartitionInducedComponents: todo"));
  touched = ismalloc(nvtxs, 0, "FindPartitionInducedComponents: touched");


  /* Find the connected componends induced by the partition */
  ncmps = -1;
  first = last = 0;
  nleft = nvtxs;
  while (nleft > 0) {
    if (first == last) { /* Find another starting vertex */
      cptr[++ncmps] = first;
      ASSERT(touched[todo[0]] == 0);
      i = todo[0];
      cind[last++] = i;
      touched[i] = 1;
      me = where[i];
    }

    i = cind[first++];
    k = perm[i];
    j = todo[k] = todo[--nleft];
    perm[j] = k;

    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];
      if (where[k] == me && !touched[k]) {
        cind[last++] = k;
        touched[k] = 1;
      }
    }
  }
  cptr[++ncmps] = first;

  if (mustfree_ccsr)
    gk_free((void **)&cptr, &cind, LTERM);
  if (mustfree_where)
    gk_free((void **)&where, LTERM);

  gk_free((void **)&perm, &todo, &touched, LTERM);

  return ncmps;
}


/*************************************************************************/
/*! This function computes a permutation of the vertices based on a
    breadth-first-traversal. It can be used for re-ordering the graph
    to reduce its bandwidth for better cache locality.

    \param ctrl is the control structure
    \param graph is the graph structure
    \param perm is the array that upon completion, perm[i] will store
           the ID of the vertex that corresponds to the ith vertex in the
           re-ordered graph.
*/
/*************************************************************************/
void ComputeBFSOrdering(ctrl_t *ctrl, graph_t *graph, idx_t *bfsperm)
{
  idx_t i, j, k, nvtxs, first, last;
  idx_t *xadj, *adjncy, *perm;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;

  /* Allocate memory required for the BFS traversal */
  perm = iincset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));

  iincset(nvtxs, 0, bfsperm);  /* this array will also store the vertices
                                  still to be processed */

  /* Find the connected componends induced by the partition */
  first = last = 0;
  while (first < nvtxs) {
    if (first == last) { /* Find another starting vertex */
      k = bfsperm[last];
      ASSERT(perm[k] != -1);
      perm[k] = -1; /* mark node as being visited */
      last++;
    }

    i = bfsperm[first++];
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];
      /* if a node has been already been visited, its perm[] will be -1 */
      if (perm[k] != -1) {
        /* perm[k] is the location within bfsperm of where k resides; 
           put in that location bfsperm[last] that we are about to
           overwrite and update perm[bfsperm[last]] to reflect that. */
        bfsperm[perm[k]]    = bfsperm[last];
        perm[bfsperm[last]] = perm[k];

        bfsperm[last++] = k;  /* put node at the end of the "queue" */
        perm[k]         = -1; /* mark node as being visited */
      }
    }
  }

  WCOREPOP;
}


/*************************************************************************/
/*! This function checks whether a graph is contiguous or not. 
 */
/**************************************************************************/
idx_t IsConnected(graph_t *graph, idx_t report)
{
  idx_t ncmps;

  ncmps = FindPartitionInducedComponents(graph, NULL, NULL, NULL);

  if (ncmps != 1 && report)
    printf("The graph is not connected. It has %"PRIDX" connected components.\n", ncmps);

  return (ncmps == 1);
}


/*************************************************************************/
/*! This function checks whether or not partition pid is contigous
  */
/*************************************************************************/
idx_t IsConnectedSubdomain(ctrl_t *ctrl, graph_t *graph, idx_t pid, idx_t report)
{
  idx_t i, j, k, nvtxs, first, last, nleft, ncmps, wgt;
  idx_t *xadj, *adjncy, *where, *touched, *queue;
  idx_t *cptr;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  where  = graph->where;

  touched = ismalloc(nvtxs, 0, "IsConnected: touched");
  queue   = imalloc(nvtxs, "IsConnected: queue");
  cptr    = imalloc(nvtxs+1, "IsConnected: cptr");

  nleft = 0;
  for (i=0; i<nvtxs; i++) {
    if (where[i] == pid) 
      nleft++;
  }

  for (i=0; i<nvtxs; i++) {
    if (where[i] == pid) 
      break;
  }

  touched[i] = 1;
  queue[0] = i;
  first = 0; last = 1;

  cptr[0] = 0;  /* This actually points to queue */
  ncmps = 0;
  while (first != nleft) {
    if (first == last) { /* Find another starting vertex */
      cptr[++ncmps] = first;
      for (i=0; i<nvtxs; i++) {
        if (where[i] == pid && !touched[i])
          break;
      }
      queue[last++] = i;
      touched[i] = 1;
    }

    i = queue[first++];
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];
      if (where[k] == pid && !touched[k]) {
        queue[last++] = k;
        touched[k] = 1;
      }
    }
  }
  cptr[++ncmps] = first;

  if (ncmps > 1 && report) {
    printf("The graph has %"PRIDX" connected components in partition %"PRIDX":\t", ncmps, pid);
    for (i=0; i<ncmps; i++) {
      wgt = 0;
      for (j=cptr[i]; j<cptr[i+1]; j++)
        wgt += graph->vwgt[queue[j]];
      printf("[%5"PRIDX" %5"PRIDX"] ", cptr[i+1]-cptr[i], wgt);
      /*
      if (cptr[i+1]-cptr[i] == 1)
        printf("[%"PRIDX" %"PRIDX"] ", queue[cptr[i]], xadj[queue[cptr[i]]+1]-xadj[queue[cptr[i]]]);
      */
    }
    printf("\n");
  }

  gk_free((void **)&touched, &queue, &cptr, LTERM);

  return (ncmps == 1 ? 1 : 0);
}


/*************************************************************************/
/*! This function identifies the number of connected components in a graph
    that result after removing the vertices that belong to the vertex 
    separator (i.e., graph->where[i] == 2).
    The connected component memberships are returned in the CSR-style 
    pair of arrays cptr, cind.
*/
/**************************************************************************/
idx_t FindSepInducedComponents(ctrl_t *ctrl, graph_t *graph, idx_t *cptr, 
          idx_t *cind)
{
  idx_t i, j, k, nvtxs, first, last, nleft, ncmps, wgt;
  idx_t *xadj, *adjncy, *where, *touched, *queue;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  where  = graph->where;

  touched = ismalloc(nvtxs, 0, "IsConnected: queue");

  for (i=0; i<graph->nbnd; i++)
    touched[graph->bndind[i]] = 1;

  queue = cind;

  nleft = 0;
  for (i=0; i<nvtxs; i++) {
    if (where[i] != 2) 
      nleft++;
  }

  for (i=0; i<nvtxs; i++) {
    if (where[i] != 2)
      break;
  }

  touched[i] = 1;
  queue[0]   = i;
  first      = 0; 
  last       = 1;
  cptr[0]    = 0;  /* This actually points to queue */
  ncmps      = 0;

  while (first != nleft) {
    if (first == last) { /* Find another starting vertex */
      cptr[++ncmps] = first;
      for (i=0; i<nvtxs; i++) {
        if (!touched[i])
          break;
      }
      queue[last++] = i;
      touched[i] = 1;
    }

    i = queue[first++];
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];
      if (!touched[k]) {
        queue[last++] = k;
        touched[k] = 1;
      }
    }
  }
  cptr[++ncmps] = first;

  gk_free((void **)&touched, LTERM);

  return ncmps;
}


/*************************************************************************/
/*! This function finds all the connected components induced by the 
    partitioning vector in graph->where and tries to push them around to 
    remove some of them. */
/*************************************************************************/
void EliminateComponents(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, jj, k, me, nparts, nvtxs, ncon, ncmps, other, 
        ncand, target;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *where, *pwgts;
  idx_t *cptr, *cind, *cpvec, *pcptr, *pcind, *cwhere;
  idx_t cid, bestcid, *cwgt, *bestcwgt;
  idx_t ntodo, oldntodo, *todo;
  rkv_t *cand;
  real_t *tpwgts;
  idx_t *vmarker=NULL, *pmarker=NULL, *modind=NULL;  /* volume specific work arrays */

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;
  adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt);

  where = graph->where;
  pwgts = graph->pwgts;

  nparts = ctrl->nparts;
  tpwgts = ctrl->tpwgts;

  cptr = iwspacemalloc(ctrl, nvtxs+1);
  cind = iwspacemalloc(ctrl, nvtxs);

  ncmps = FindPartitionInducedComponents(graph, where, cptr, cind);

  IFSET(ctrl->dbglvl, METIS_DBG_CONTIGINFO, 
      printf("I found %"PRIDX" components, for this %"PRIDX"-way partition\n", 
          ncmps, nparts)); 

  /* There are more components than partitions */
  if (ncmps > nparts) {
    cwgt     = iwspacemalloc(ctrl, ncon);
    bestcwgt = iwspacemalloc(ctrl, ncon);
    cpvec    = iwspacemalloc(ctrl, nparts);
    pcptr    = iset(nparts+1, 0, iwspacemalloc(ctrl, nparts+1));
    pcind    = iwspacemalloc(ctrl, ncmps);
    cwhere   = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
    todo     = iwspacemalloc(ctrl, ncmps);
    cand     = (rkv_t *)wspacemalloc(ctrl, nparts*sizeof(rkv_t));

    if (ctrl->objtype == METIS_OBJTYPE_VOL) {
      /* Vol-refinement specific working arrays */
      modind  = iwspacemalloc(ctrl, nvtxs);
      vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
      pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));
    }


    /* Get a CSR representation of the components-2-partitions mapping */
    for (i=0; i<ncmps; i++) 
      pcptr[where[cind[cptr[i]]]]++;
    MAKECSR(i, nparts, pcptr);
    for (i=0; i<ncmps; i++) 
      pcind[pcptr[where[cind[cptr[i]]]]++] = i;
    SHIFTCSR(i, nparts, pcptr);

    /* Assign the heaviest component of each partition to its original partition */
    for (ntodo=0, i=0; i<nparts; i++) {
      if (pcptr[i+1]-pcptr[i] == 1)
        bestcid = pcind[pcptr[i]];
      else {
        for (bestcid=-1, j=pcptr[i]; j<pcptr[i+1]; j++) {
          cid = pcind[j];
          iset(ncon, 0, cwgt);
          for (ii=cptr[cid]; ii<cptr[cid+1]; ii++)
            iaxpy(ncon, 1, vwgt+cind[ii]*ncon, 1, cwgt, 1);
          if (bestcid == -1 || isum(ncon, bestcwgt, 1) < isum(ncon, cwgt, 1)) {
            bestcid  = cid;
            icopy(ncon, cwgt, bestcwgt);
          }
        }
        /* Keep track of those that need to be dealt with */
        for (j=pcptr[i]; j<pcptr[i+1]; j++) {
          if (pcind[j] != bestcid)
            todo[ntodo++] = pcind[j];
        }
      }

      for (j=cptr[bestcid]; j<cptr[bestcid+1]; j++) {
        ASSERT(where[cind[j]] == i);
        cwhere[cind[j]] = i;
      }
    }


    while (ntodo > 0) {
      oldntodo = ntodo;
      for (i=0; i<ntodo; i++) {
        cid = todo[i];
        me = where[cind[cptr[cid]]];  /* Get the domain of this component */

        /* Determine the weight of the block to be moved */
        iset(ncon, 0, cwgt);
        for (j=cptr[cid]; j<cptr[cid+1]; j++) 
          iaxpy(ncon, 1, vwgt+cind[j]*ncon, 1, cwgt, 1);

        IFSET(ctrl->dbglvl, METIS_DBG_CONTIGINFO, 
            printf("Trying to move %"PRIDX" [%"PRIDX"] from %"PRIDX"\n", 
                cid, isum(ncon, cwgt, 1), me)); 

        /* Determine the connectivity */
        iset(nparts, 0, cpvec);
        for (j=cptr[cid]; j<cptr[cid+1]; j++) {
          ii = cind[j];
          for (jj=xadj[ii]; jj<xadj[ii+1]; jj++) 
            if (cwhere[adjncy[jj]] != -1)
              cpvec[cwhere[adjncy[jj]]] += (adjwgt ? adjwgt[jj] : 1);
        }

        /* Put the neighbors into a cand[] array for sorting */
        for (ncand=0, j=0; j<nparts; j++) {
          if (cpvec[j] > 0) {
            cand[ncand].key   = cpvec[j];
            cand[ncand++].val = j;
          }
        }
        if (ncand == 0)
          continue;

        rkvsortd(ncand, cand);

        /* Limit the moves to only the top candidates, which are defined as 
           those with connectivity at least 50% of the best.
           This applies only when ncon=1, as for multi-constraint, balancing
           will be hard. */
        if (ncon == 1) {
          for (j=1; j<ncand; j++) {
            if (cand[j].key < .5*cand[0].key)
              break;
          }
          ncand = j;
        }
      
        /* Now among those, select the one with the best balance */
        target = cand[0].val;
        for (j=1; j<ncand; j++) {
          if (BetterBalanceKWay(ncon, cwgt, ctrl->ubfactors,
                1, pwgts+target*ncon, ctrl->pijbm+target*ncon,
                1, pwgts+cand[j].val*ncon, ctrl->pijbm+cand[j].val*ncon))
            target = cand[j].val;
        }

        IFSET(ctrl->dbglvl, METIS_DBG_CONTIGINFO, 
            printf("\tMoving it to %"PRIDX" [%"PRIDX"] [%"PRIDX"]\n", target, cpvec[target], ncand));

        /* Note that as a result of a previous movement, a connected component may
           now will like to stay to its original partition */
        if (target != me) {
          switch (ctrl->objtype) {
            case METIS_OBJTYPE_CUT:
              MoveGroupContigForCut(ctrl, graph, target, cid, cptr, cind);
              break;

            case METIS_OBJTYPE_VOL:
              MoveGroupContigForVol(ctrl, graph, target, cid, cptr, cind, 
                  vmarker, pmarker, modind);
              break;

            default:
              gk_errexit(SIGERR, "Unknown objtype %d\n", ctrl->objtype);
          }
        }

        /* Update the cwhere vector */
        for (j=cptr[cid]; j<cptr[cid+1]; j++) 
          cwhere[cind[j]] = target;

        todo[i] = todo[--ntodo];
      }
      if (oldntodo == ntodo) {
        IFSET(ctrl->dbglvl, METIS_DBG_CONTIGINFO, printf("Stopped at ntodo: %"PRIDX"\n", ntodo));
        break;
      }
    }

    for (i=0; i<nvtxs; i++)
      ASSERT(where[i] == cwhere[i]);

  }

  WCOREPOP;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo 
 */
/*************************************************************************/
void MoveGroupContigForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid, 
         idx_t *ptr, idx_t *ind)
{
  idx_t i, ii, iii, j, jj, k, l, nvtxs, nbnd, from, me;
  idx_t *xadj, *adjncy, *adjwgt, *where, *bndptr, *bndind;
  ckrinfo_t *myrinfo;
  cnbr_t *mynbrs;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  where  = graph->where;
  bndptr = graph->bndptr;
  bndind = graph->bndind;

  nbnd = graph->nbnd;

  for (iii=ptr[gid]; iii<ptr[gid+1]; iii++) {
    i    = ind[iii];
    from = where[i];

    myrinfo = graph->ckrinfo+i;
    if (myrinfo->inbr == -1) {
      myrinfo->inbr = cnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
      myrinfo->nnbrs = 0;
    }
    mynbrs = ctrl->cnbrpool + myrinfo->inbr; 

    /* find the location of 'to' in myrinfo or create it if it is not there */
    for (k=0; k<myrinfo->nnbrs; k++) {
      if (mynbrs[k].pid == to)
        break;
    }
    if (k == myrinfo->nnbrs) {
      mynbrs[k].pid = to;
      mynbrs[k].ed = 0;
      myrinfo->nnbrs++;
    }

    graph->mincut -= mynbrs[k].ed-myrinfo->id;

    /* Update ID/ED and BND related information for the moved vertex */
    iaxpy(graph->ncon,  1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+to*graph->ncon,   1);
    iaxpy(graph->ncon, -1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+from*graph->ncon, 1);
    UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd, 
        bndptr, bndind, BNDTYPE_REFINE);

    /* Update the degrees of adjacent vertices */
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      ii = adjncy[j];
      me = where[ii];
      myrinfo = graph->ckrinfo+ii;

      UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii+1]-xadj[ii], me,
          from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, BNDTYPE_REFINE);
    }

    ASSERT(CheckRInfo(ctrl, graph->ckrinfo+i));
  }

  graph->nbnd = nbnd;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo 
 */
/*************************************************************************/
void MoveGroupContigForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid, 
         idx_t *ptr, idx_t *ind, idx_t *vmarker, idx_t *pmarker, 
         idx_t *modind)
{
  idx_t i, ii, iii, j, jj, k, l, nvtxs, from, me, other, xgain;
  idx_t *xadj, *vsize, *adjncy, *where;
  vkrinfo_t *myrinfo, *orinfo;
  vnbr_t *mynbrs, *onbrs;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vsize  = graph->vsize;
  adjncy = graph->adjncy;
  where  = graph->where;

  for (iii=ptr[gid]; iii<ptr[gid+1]; iii++) {
    i    = ind[iii];
    from = where[i];

    myrinfo = graph->vkrinfo+i;
    if (myrinfo->inbr == -1) {
      myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
      myrinfo->nnbrs = 0;
    }
    mynbrs = ctrl->vnbrpool + myrinfo->inbr; 

    xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

    /* find the location of 'to' in myrinfo or create it if it is not there */
    for (k=0; k<myrinfo->nnbrs; k++) {
      if (mynbrs[k].pid == to)
        break;
    }
    if (k == myrinfo->nnbrs) {
      if (myrinfo->nid > 0)
        xgain -= vsize[i];

      /* determine the volume gain resulting from that move */
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo+ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;
        ASSERT(other != to)

        if (from == other) {
          /* Same subdomain vertex: Decrease the gain if 'to' is a new neighbor. */
          for (l=0; l<orinfo->nnbrs; l++) {
            if (onbrs[l].pid == to)
              break;
          }
          if (l == orinfo->nnbrs) 
            xgain -= vsize[ii];
        }
        else {
          /* Remote vertex: increase if 'to' is a new subdomain */
          for (l=0; l<orinfo->nnbrs; l++) {
            if (onbrs[l].pid == to)
              break;
          }
          if (l == orinfo->nnbrs) 
            xgain -= vsize[ii];

          /* Remote vertex: decrease if i is the only connection to 'from' */
          for (l=0; l<orinfo->nnbrs; l++) {
            if (onbrs[l].pid == from && onbrs[l].ned == 1) {
              xgain += vsize[ii];
              break;
            }
          }
        }
      }
      graph->minvol -= xgain;
      graph->mincut -= -myrinfo->nid;
    }
    else {
      graph->minvol -= (xgain + mynbrs[k].gv);
      graph->mincut -= mynbrs[k].ned-myrinfo->nid;
    }


    /* Update where and pwgts */
    where[i] = to;
    iaxpy(graph->ncon,  1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+to*graph->ncon,   1);
    iaxpy(graph->ncon, -1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+from*graph->ncon, 1);

    /* Update the id/ed/gains/bnd of potentially affected nodes */
    KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL,
        NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

    /*CheckKWayVolPartitionParams(ctrl, graph);*/
  }

  ASSERT(ComputeCut(graph, where) == graph->mincut);
  ASSERTP(ComputeVolume(graph, where) == graph->minvol,
      ("%"PRIDX" %"PRIDX"\n", ComputeVolume(graph, where), graph->minvol));

}

