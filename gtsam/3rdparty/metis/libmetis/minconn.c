/*!
\file 
\brief Functions that deal with prunning the number of adjacent subdomains in kmetis

\date Started 7/15/98
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version $Id: minconn.c 10513 2011-07-07 22:06:03Z karypis $
*/

#include "metislib.h"


/*************************************************************************/
/*! This function computes the subdomain graph storing the result in the
    pre-allocated worspace arrays */
/*************************************************************************/
void ComputeSubDomainGraph(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, pid, other, nparts, nvtxs, nnbrs;
  idx_t *xadj, *adjncy, *adjwgt, *where;
  idx_t *pptr, *pind;
  idx_t nads=0, *vadids, *vadwgts;

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  where  = graph->where;

  nparts = ctrl->nparts; 

  vadids  = ctrl->pvec1;
  vadwgts = iset(nparts, 0, ctrl->pvec2);

  pptr = iwspacemalloc(ctrl, nparts+1);
  pind = iwspacemalloc(ctrl, nvtxs);
  iarray2csr(nvtxs, nparts, where, pptr, pind);

  for (pid=0; pid<nparts; pid++) {
    switch (ctrl->objtype) {
      case METIS_OBJTYPE_CUT:
        {
          ckrinfo_t *rinfo;
          cnbr_t *nbrs;

          rinfo = graph->ckrinfo;
          for (nads=0, ii=pptr[pid]; ii<pptr[pid+1]; ii++) {
            i = pind[ii];
            ASSERT(pid == where[i]);
      
            if (rinfo[i].ed > 0) {
              nnbrs = rinfo[i].nnbrs;
              nbrs  = ctrl->cnbrpool + rinfo[i].inbr;
      
              for (j=0; j<nnbrs; j++) {
                other = nbrs[j].pid;
                if (vadwgts[other] == 0)
                  vadids[nads++] = other;
                vadwgts[other] += nbrs[j].ed;
              }
            }
          }
        }
        break;

      case METIS_OBJTYPE_VOL:
        {
          vkrinfo_t *rinfo;
          vnbr_t *nbrs;

          rinfo = graph->vkrinfo;
          for (nads=0, ii=pptr[pid]; ii<pptr[pid+1]; ii++) {
            i = pind[ii];
            ASSERT(pid == where[i]);
      
            if (rinfo[i].ned > 0) {
              nnbrs = rinfo[i].nnbrs;
              nbrs  = ctrl->vnbrpool + rinfo[i].inbr;
      
              for (j=0; j<nnbrs; j++) {
                other = nbrs[j].pid;
                if (vadwgts[other] == 0)
                  vadids[nads++] = other;
                vadwgts[other] += nbrs[j].ned;
              }
            }
          }
        }
        break;

      default:
        gk_errexit(SIGERR, "Unknown objtype: %d\n", ctrl->objtype);
    }

    /* See if you have enough memory to store the adjacent info for that subdomain */
    if (ctrl->maxnads[pid] < nads) {
      ctrl->maxnads[pid] = 2*nads;
      ctrl->adids[pid]   = irealloc(ctrl->adids[pid], ctrl->maxnads[pid], 
                               "ComputeSubDomainGraph: adids[pid]");
      ctrl->adwgts[pid]  = irealloc(ctrl->adwgts[pid], ctrl->maxnads[pid], 
                               "ComputeSubDomainGraph: adids[pid]");
    }

    ctrl->nads[pid] = nads;
    for (j=0; j<nads; j++) {
      ctrl->adids[pid][j]  = vadids[j];
      ctrl->adwgts[pid][j] = vadwgts[vadids[j]];

      vadwgts[vadids[j]] = 0;
    }
  }
      
  WCOREPOP;
}


/*************************************************************************/
/*! This function updates the weight of an edge in the subdomain graph by
    adding to it the value of ewgt. The update can either increase or
    decrease the weight of the subdomain edge based on the value of ewgt.

    \param u is the ID of one of the incident subdomains to the edge
    \param v is the ID of the other incident subdomains to the edge
    \param ewgt is the weight to be added to the subdomain edge
    \param nparts is the number of subdomains
    \param r_maxndoms is the maximum number of adjacent subdomains and is
           updated as necessary. The update is skipped if a NULL value is
           supplied.
*/
/*************************************************************************/
void UpdateEdgeSubDomainGraph(ctrl_t *ctrl, idx_t u, idx_t v, idx_t ewgt, 
         idx_t *r_maxndoms)
{
  idx_t i, j, nads;

  if (ewgt == 0)
    return;

  for (i=0; i<2; i++) {
    nads = ctrl->nads[u];
    /* Find the edge */
    for (j=0; j<nads; j++) {
      if (ctrl->adids[u][j] == v) {
        ctrl->adwgts[u][j] += ewgt;
        break;
      }
    }

    if (j == nads) {
      /* Deal with the case in which the edge was not found */
      ASSERT(ewgt > 0);
      if (ctrl->maxnads[u] == nads) {
        ctrl->maxnads[u] = 2*(nads+1);
        ctrl->adids[u]   = irealloc(ctrl->adids[u], ctrl->maxnads[u], 
                               "IncreaseEdgeSubDomainGraph: adids[pid]");
        ctrl->adwgts[u]  = irealloc(ctrl->adwgts[u], ctrl->maxnads[u], 
                               "IncreaseEdgeSubDomainGraph: adids[pid]");
      }
      ctrl->adids[u][nads]  = v;
      ctrl->adwgts[u][nads] = ewgt;
      nads++;
      if (r_maxndoms != NULL && nads > *r_maxndoms) {
        printf("You just increased the maxndoms: %"PRIDX" %"PRIDX"\n", 
            nads, *r_maxndoms);
        *r_maxndoms = nads;
      }
    }
    else {
      /* See if the updated edge becomes 0 */
      ASSERT(ctrl->adwgts[u][j] >= 0);
      if (ctrl->adwgts[u][j] == 0) {
        ctrl->adids[u][j]  = ctrl->adids[u][nads-1];
        ctrl->adwgts[u][j] = ctrl->adwgts[u][nads-1];
        nads--;
        if (r_maxndoms != NULL && nads+1 == *r_maxndoms)
          *r_maxndoms = ctrl->nads[iargmax(ctrl->nparts, ctrl->nads)];
      }
    }
    ctrl->nads[u] = nads;

    SWAP(u, v, j);
  }
}


/*************************************************************************/
/*! This function computes the subdomain graph */
/*************************************************************************/
void EliminateSubDomainEdges(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, ii, j, k, ncon, nparts, scheme, pid_from, pid_to, me, other, nvtxs, 
        total, max, avg, totalout, nind=0, ncand=0, ncand2, target, target2, 
        nadd, bestnadd=0;
  idx_t min, move, *cpwgt;
  idx_t *xadj, *adjncy, *vwgt, *adjwgt, *pwgts, *where, *maxpwgt, 
        *mypmat, *otherpmat, *kpmat, *ind;
  idx_t *nads, **adids, **adwgts;
  ikv_t *cand, *cand2;
  ipq_t queue;
  real_t *tpwgts, badfactor=1.4;
  idx_t *pptr, *pind;
  idx_t *vmarker=NULL, *pmarker=NULL, *modind=NULL;  /* volume specific work arrays */

  WCOREPUSH;

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vwgt   = graph->vwgt;
  adjwgt = (ctrl->objtype == METIS_OBJTYPE_VOL ? NULL : graph->adjwgt);

  where = graph->where;
  pwgts = graph->pwgts;  /* We assume that this is properly initialized */

  nparts = ctrl->nparts;
  tpwgts = ctrl->tpwgts;

  cpwgt     = iwspacemalloc(ctrl, ncon);
  maxpwgt   = iwspacemalloc(ctrl, nparts*ncon);
  ind       = iwspacemalloc(ctrl, nvtxs);
  otherpmat = iset(nparts, 0, iwspacemalloc(ctrl, nparts));

  cand  = ikvwspacemalloc(ctrl, nparts);
  cand2 = ikvwspacemalloc(ctrl, nparts);

  pptr = iwspacemalloc(ctrl, nparts+1);
  pind = iwspacemalloc(ctrl, nvtxs);
  iarray2csr(nvtxs, nparts, where, pptr, pind);

  if (ctrl->objtype == METIS_OBJTYPE_VOL) {
    /* Vol-refinement specific working arrays */
    modind  = iwspacemalloc(ctrl, nvtxs);
    vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));
  }


  /* Compute the pmat matrix and ndoms */
  ComputeSubDomainGraph(ctrl, graph);

  nads   = ctrl->nads;
  adids  = ctrl->adids;
  adwgts = ctrl->adwgts;

  mypmat = iset(nparts, 0, ctrl->pvec1);
  kpmat  = iset(nparts, 0, ctrl->pvec2);

  /* Compute the maximum allowed weight for each domain */
  for (i=0; i<nparts; i++) {
    for (j=0; j<ncon; j++)
      maxpwgt[i*ncon+j] = 
          (ncon == 1 ? 1.25 : 1.025)*tpwgts[i]*graph->tvwgt[j]*ctrl->ubfactors[j];
  }

  ipqInit(&queue, nparts);

  /* Get into the loop eliminating subdomain connections */
  while (1) {
    total = isum(nparts, nads, 1);
    avg   = total/nparts;
    max   = nads[iargmax(nparts, nads)];

    IFSET(ctrl->dbglvl, METIS_DBG_CONNINFO, 
          printf("Adjacent Subdomain Stats: Total: %3"PRIDX", "
                 "Max: %3"PRIDX"[%zu], Avg: %3"PRIDX"\n", 
                 total, max, iargmax(nparts, nads), avg)); 

    if (max < badfactor*avg)
      break;

    /* Add the subdomains that you will try to reduce their connectivity */
    ipqReset(&queue);
    for (i=0; i<nparts; i++) {
      if (nads[i] >= avg + (max-avg)/2)
        ipqInsert(&queue, i, nads[i]);
    }

    move = 0;
    while ((me = ipqGetTop(&queue)) != -1) {
      totalout = isum(nads[me], adwgts[me], 1);

      for (ncand2=0, i=0; i<nads[me]; i++) {
        mypmat[adids[me][i]] = adwgts[me][i];

        /* keep track of the weakly connected adjacent subdomains */
        if (2*nads[me]*adwgts[me][i] < totalout) {
          cand2[ncand2].val   = adids[me][i];
          cand2[ncand2++].key = adwgts[me][i];
        }
      }

      IFSET(ctrl->dbglvl, METIS_DBG_CONNINFO, 
            printf("Me: %"PRIDX", Degree: %4"PRIDX", TotalOut: %"PRIDX",\n", 
                me, nads[me], totalout));

      /* Sort the connections according to their cut */
      ikvsorti(ncand2, cand2);

      /* Two schemes are used for eliminating subdomain edges.
         The first, tries to eliminate subdomain edges by moving remote groups 
         of vertices to subdomains that 'me' is already connected to.
         The second, tries to eliminate subdomain edges by moving entire sets of 
         my vertices that connect to the 'other' subdomain to a subdomain that 
         I'm already connected to.
         These two schemes are applied in sequence. */
      target = target2 = -1;
      for (scheme=0; scheme<2; scheme++) {
        for (min=0; min<ncand2; min++) {
          other = cand2[min].val;

          /* pid_from is the subdomain from where the vertices will be removed.
             pid_to is the adjacent subdomain to pid_from that defines the 
             (me, other) subdomain edge that needs to be removed */
          if (scheme == 0) {
            pid_from = other;
            pid_to   = me;
          }
          else {
            pid_from  = me;
            pid_to    = other;
          }
  
          /* Go and find the vertices in 'other' that are connected in 'me' */
          for (nind=0, ii=pptr[pid_from]; ii<pptr[pid_from+1]; ii++) {
            i = pind[ii];
            ASSERT(where[i] == pid_from);
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              if (where[adjncy[j]] == pid_to) {
                ind[nind++] = i;
                break;
              }
            }
          }
  
          /* Go and construct the otherpmat to see where these nind vertices are 
             connected to */
          iset(ncon, 0, cpwgt);
          for (ncand=0, ii=0; ii<nind; ii++) {
            i = ind[ii];
            iaxpy(ncon, 1, vwgt+i*ncon, 1, cpwgt, 1);
    
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              if ((k = where[adjncy[j]]) == pid_from)
                continue;
              if (otherpmat[k] == 0)
                cand[ncand++].val = k;
              otherpmat[k] += (adjwgt ? adjwgt[j] : 1);
            }
          }
    
          for (i=0; i<ncand; i++) {
            cand[i].key = otherpmat[cand[i].val];
            ASSERT(cand[i].key > 0);
          }

          ikvsortd(ncand, cand);
    
          IFSET(ctrl->dbglvl, METIS_DBG_CONNINFO, 
                printf("\tMinOut: %4"PRIDX", to: %3"PRIDX", TtlWgt: %5"PRIDX"[#:%"PRIDX"]\n", 
                    mypmat[other], other, isum(ncon, cpwgt, 1), nind));

          /* Go through and select the first domain that is common with 'me', and does
             not increase the nads[target] higher than nads[me], subject to the maxpwgt
             constraint. Traversal is done from the mostly connected to the least. */
          for (i=0; i<ncand; i++) {
            k = cand[i].val;
    
            if (mypmat[k] > 0) {
              /* Check if balance will go off */
              if (!ivecaxpylez(ncon, 1, cpwgt, pwgts+k*ncon, maxpwgt+k*ncon))
                continue;
    
              /* get a dense vector out of k's connectivity */
              for (j=0; j<nads[k]; j++) 
                kpmat[adids[k][j]] = adwgts[k][j];
    
              /* Check if the move to domain k will increase the nads of another
                 subdomain j that the set of vertices being moved are connected
                 to but domain k is not connected to. */
              for (j=0; j<nparts; j++) {
                if (otherpmat[j] > 0 && kpmat[j] == 0 && nads[j]+1 >= nads[me]) 
                  break;
              }
  
              /* There were no bad second level effects. See if you can find a
                 subdomain to move to. */
              if (j == nparts) { 
                for (nadd=0, j=0; j<nparts; j++) {
                  if (otherpmat[j] > 0 && kpmat[j] == 0)
                    nadd++;
                }
    
                IFSET(ctrl->dbglvl, METIS_DBG_CONNINFO, 
                      printf("\t\tto=%"PRIDX", nadd=%"PRIDX", %"PRIDX"\n", k, nadd, nads[k]));
    
                if (nads[k]+nadd < nads[me]) {
                  if (target2 == -1 || nads[target2]+bestnadd > nads[k]+nadd ||
                      (nads[target2]+bestnadd == nads[k]+nadd && bestnadd > nadd)) {
                    target2  = k;
                    bestnadd = nadd;
                  }
                }
  
                if (nadd == 0) 
                  target = k;
              }

              /* reset kpmat for the next iteration */
              for (j=0; j<nads[k]; j++) 
                kpmat[adids[k][j]] = 0;
            }

            if (target != -1)
              break;
          }

          /* reset the otherpmat for the next iteration */
          for (i=0; i<ncand; i++) 
            otherpmat[cand[i].val] = 0;

          if (target == -1 && target2 != -1)
            target = target2;
    
          if (target != -1) {
            IFSET(ctrl->dbglvl, METIS_DBG_CONNINFO, 
                printf("\t\tScheme: %"PRIDX". Moving to %"PRIDX"\n", scheme, target));
            move = 1;
            break;
          }
        }

        if (target != -1)
          break;  /* A move was found. No need to try the other scheme */
      }

      /* reset the mypmat for next iteration */
      for (i=0; i<nads[me]; i++) 
        mypmat[adids[me][i]] = 0;

      /* Note that once a target is found the above loops exit right away. So the
         following variables are valid */
      if (target != -1) {
        switch (ctrl->objtype) {
          case METIS_OBJTYPE_CUT:
            MoveGroupMinConnForCut(ctrl, graph, target, nind, ind);
            break;
          case METIS_OBJTYPE_VOL:
            MoveGroupMinConnForVol(ctrl, graph, target, nind, ind, vmarker, 
                pmarker, modind);
            break;
          default:
            gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
        }

        /* Update the csr representation of the partitioning vector */
        iarray2csr(nvtxs, nparts, where, pptr, pind);
      }
    }

    if (move == 0)
      break;
  }

  ipqFree(&queue);

  WCOREPOP;
}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo */
/*************************************************************************/
void MoveGroupMinConnForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind)
{
  idx_t i, ii, j, jj, k, l, nvtxs, nbnd, from, me;
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

  while (--nind>=0) {
    i    = ind[nind];
    from = where[i];

    myrinfo = graph->ckrinfo+i;
    if (myrinfo->inbr == -1) {
      myrinfo->inbr  = cnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
      myrinfo->nnbrs = 0;
    }
    mynbrs = ctrl->cnbrpool + myrinfo->inbr;

    /* find the location of 'to' in myrinfo or create it if it is not there */
    for (k=0; k<myrinfo->nnbrs; k++) {
      if (mynbrs[k].pid == to)
        break;
    }
    if (k == myrinfo->nnbrs) {
      ASSERT(k < xadj[i+1]-xadj[i]);
      mynbrs[k].pid = to;
      mynbrs[k].ed  = 0;
      myrinfo->nnbrs++;
    }

    /* Update pwgts */
    iaxpy(graph->ncon,  1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+to*graph->ncon,   1);
    iaxpy(graph->ncon, -1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+from*graph->ncon, 1);

    /* Update mincut */
    graph->mincut -= mynbrs[k].ed-myrinfo->id;

    /* Update subdomain connectivity graph to reflect the move of 'i' */
    UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id-mynbrs[k].ed, NULL);

    /* Update ID/ED and BND related information for the moved vertex */
    UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd, 
        bndptr, bndind, BNDTYPE_REFINE);

    /* Update the degrees of adjacent vertices */
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      ii = adjncy[j];
      me = where[ii];
      myrinfo = graph->ckrinfo+ii;

      UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii+1]-xadj[ii], me,
          from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, BNDTYPE_REFINE);

      /* Update subdomain graph to reflect the move of 'i' for domains other 
         than 'from' and 'to' */
      if (me != from && me != to) {
        UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], NULL);
        UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], NULL);
      }
    }
  }

  ASSERT(ComputeCut(graph, where) == graph->mincut);

  graph->nbnd = nbnd;

}


/*************************************************************************/
/*! This function moves a collection of vertices and updates their rinfo */
/*************************************************************************/
void MoveGroupMinConnForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind, idx_t *vmarker, idx_t *pmarker, idx_t *modind)
{
  idx_t i, ii, j, jj, k, l, nvtxs, from, me, other, xgain, ewgt;
  idx_t *xadj, *vsize, *adjncy, *where;
  vkrinfo_t *myrinfo, *orinfo;
  vnbr_t *mynbrs, *onbrs;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  vsize  = graph->vsize;
  adjncy = graph->adjncy;
  where  = graph->where;

  while (--nind>=0) {
    i    = ind[nind];
    from = where[i];

    myrinfo = graph->vkrinfo+i;
    if (myrinfo->inbr == -1) {
      myrinfo->inbr  = vnbrpoolGetNext(ctrl, xadj[i+1]-xadj[i]+1);
      myrinfo->nnbrs = 0;
    }
    mynbrs = ctrl->vnbrpool + myrinfo->inbr;

    xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? vsize[i] : 0);

    //printf("Moving %"PRIDX" from %"PRIDX" to %"PRIDX" [vsize: %"PRIDX"] [xgain: %"PRIDX"]\n", 
    //    i, from, to, vsize[i], xgain);
    
    /* find the location of 'to' in myrinfo or create it if it is not there */
    for (k=0; k<myrinfo->nnbrs; k++) {
      if (mynbrs[k].pid == to)
        break;
    }

    if (k == myrinfo->nnbrs) {
      //printf("Missing neighbor\n");

      if (myrinfo->nid > 0)
        xgain -= vsize[i];

      /* determine the volume gain resulting from that move */
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo+ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;
        ASSERT(other != to)

        //printf("  %8d %8d %3d\n", (int)ii, (int)vsize[ii], (int)other);

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
      ewgt = myrinfo->nid;
    }
    else {
      graph->minvol -= (xgain + mynbrs[k].gv);
      graph->mincut -= mynbrs[k].ned-myrinfo->nid;
      ewgt = myrinfo->nid-mynbrs[k].ned;
    }

    /* Update where and pwgts */
    where[i] = to;
    iaxpy(graph->ncon,  1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+to*graph->ncon,   1);
    iaxpy(graph->ncon, -1, graph->vwgt+i*graph->ncon, 1, graph->pwgts+from*graph->ncon, 1);

    /* Update subdomain connectivity graph to reflect the move of 'i' */
    UpdateEdgeSubDomainGraph(ctrl, from, to, ewgt, NULL);

    /* Update the subdomain connectivity of the adjacent vertices */
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      me = where[adjncy[j]];
      if (me != from && me != to) {
        UpdateEdgeSubDomainGraph(ctrl, from, me, -1, NULL);
        UpdateEdgeSubDomainGraph(ctrl, to, me, 1, NULL);
      }
    }

    /* Update the id/ed/gains/bnd of potentially affected nodes */
    KWayVolUpdate(ctrl, graph, i, from, to, NULL, NULL, NULL, NULL,
        NULL, BNDTYPE_REFINE, vmarker, pmarker, modind);

    /*CheckKWayVolPartitionParams(ctrl, graph);*/
  }
  ASSERT(ComputeCut(graph, where) == graph->mincut);
  ASSERTP(ComputeVolume(graph, where) == graph->minvol, 
      ("%"PRIDX" %"PRIDX"\n", ComputeVolume(graph, where), graph->minvol));

}


/*************************************************************************/
/*! This function computes the subdomain graph. For deubuging purposes. */
/*************************************************************************/
void PrintSubDomainGraph(graph_t *graph, idx_t nparts, idx_t *where)
{
  idx_t i, j, k, me, nvtxs, total, max;
  idx_t *xadj, *adjncy, *adjwgt, *pmat;

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  pmat = ismalloc(nparts*nparts, 0, "ComputeSubDomainGraph: pmat");

  for (i=0; i<nvtxs; i++) {
    me = where[i];
    for (j=xadj[i]; j<xadj[i+1]; j++) {
      k = adjncy[j];
      if (where[k] != me) 
        pmat[me*nparts+where[k]] += adjwgt[j];
    }
  }

  /* printf("Subdomain Info\n"); */
  total = max = 0;
  for (i=0; i<nparts; i++) {
    for (k=0, j=0; j<nparts; j++) {
      if (pmat[i*nparts+j] > 0)
        k++;
    }
    total += k;

    if (k > max)
      max = k;
/*
    printf("%2"PRIDX" -> %2"PRIDX"  ", i, k);
    for (j=0; j<nparts; j++) {
      if (pmat[i*nparts+j] > 0)
        printf("[%2"PRIDX" %4"PRIDX"] ", j, pmat[i*nparts+j]);
    }
    printf("\n");
*/
  }
  printf("Total adjacent subdomains: %"PRIDX", Max: %"PRIDX"\n", total, max);

  gk_free((void **)&pmat, LTERM);
}


