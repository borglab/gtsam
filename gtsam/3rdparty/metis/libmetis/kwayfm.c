/*!
\file 
\brief Routines for k-way refinement 

\date Started 7/28/97
\author George
\author Copyright 1997-2009, Regents of the University of Minnesota 
\version $Id: kwayfm.c 10567 2011-07-13 16:17:07Z karypis $
*/

#include "metislib.h"



/*************************************************************************/
/* Top-level routine for k-way partitioning refinement. This routine just
   calls the appropriate refinement routine based on the objectives and
   constraints. */
/*************************************************************************/
void Greedy_KWayOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode)
{
  switch (ctrl->objtype) {
    case METIS_OBJTYPE_CUT:
      if (graph->ncon == 1)
        Greedy_KWayCutOptimize(ctrl, graph, niter, ffactor, omode);
      else
        Greedy_McKWayCutOptimize(ctrl, graph, niter, ffactor, omode);
      break;

    case METIS_OBJTYPE_VOL:
      if (graph->ncon == 1)
        Greedy_KWayVolOptimize(ctrl, graph, niter, ffactor, omode);
      else
        Greedy_McKWayVolOptimize(ctrl, graph, niter, ffactor, omode);
      break;

    default:
      gk_errexit(SIGERR, "Unknown objtype of %d\n", ctrl->objtype);
  }
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in 
    decreasing ed/sqrt(nnbrs)-id order. Note this is just an 
    approximation, as the ed is often split across different subdomains 
    and the sqrt(nnbrs) is just a crude approximation.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves 
         to violate the max-pwgt constraint.
  \param omode is the type of optimization that will performed among
         OMODE_REFINE and OMODE_BALANCE 
         

*/
/**************************************************************************/
void Greedy_KWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode)
{
  /* Common variables to all types of kway-refinement/balancing routines */
  idx_t i, ii, iii, j, k, l, pass, nvtxs, nparts, gain; 
  idx_t from, me, to, oldcut, vwgt;
  idx_t *xadj, *adjncy, *adjwgt;
  idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minwgt, *maxwgt, *itpwgts;
  idx_t nmoved, nupd, *vstatus, *updptr, *updind;
  idx_t maxndoms, *safetos=NULL, *nads=NULL, *doms=NULL, **adids=NULL, **adwgts=NULL;
  idx_t *bfslvl=NULL, *bfsind=NULL, *bfsmrk=NULL;
  idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);

  /* Edgecut-specific/different variables */
  idx_t nbnd, oldnnbrs;
  rpq_t *queue;
  real_t rgain;
  ckrinfo_t *myrinfo;
  cnbr_t *mynbrs;

  WCOREPUSH;

  /* Link the graph fields */
  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  bndind = graph->bndind;
  bndptr = graph->bndptr;

  where = graph->where;
  pwgts = graph->pwgts;
  
  nparts = ctrl->nparts;

  /* Setup the weight intervals of the various subdomains */
  minwgt  = iwspacemalloc(ctrl, nparts);
  maxwgt  = iwspacemalloc(ctrl, nparts);
  itpwgts = iwspacemalloc(ctrl, nparts);

  for (i=0; i<nparts; i++) {
    itpwgts[i] = ctrl->tpwgts[i]*graph->tvwgt[0];
    maxwgt[i]  = ctrl->tpwgts[i]*graph->tvwgt[0]*ctrl->ubfactors[0];
    minwgt[i]  = ctrl->tpwgts[i]*graph->tvwgt[0]*(1.0/ctrl->ubfactors[0]);
  }

  perm = iwspacemalloc(ctrl, nvtxs);


  /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made. 
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
  safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

  if (ctrl->minconn) {
    ComputeSubDomainGraph(ctrl, graph);

    nads    = ctrl->nads;
    adids   = ctrl->adids;
    adwgts  = ctrl->adwgts;
    doms    = iset(nparts, 0, ctrl->pvec1);
  }


  /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
  vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
  updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
  updind  = iwspacemalloc(ctrl, nvtxs);

  if (ctrl->contig) {
    /* The arrays that will be used for limited check of articulation points */
    bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    bfsind = iwspacemalloc(ctrl, nvtxs);
    bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  }

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
     printf("%s: [%6"PRIDX" %6"PRIDX"]-[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL"," 
            " Nv-Nb[%6"PRIDX" %6"PRIDX"], Cut: %6"PRIDX,
            (omode == OMODE_REFINE ? "GRC" : "GBC"),
            pwgts[iargmin(nparts, pwgts)], imax(nparts, pwgts), minwgt[0], maxwgt[0], 
            ComputeLoadImbalance(graph, nparts, ctrl->pijbm), 
            graph->nvtxs, graph->nbnd, graph->mincut);
     if (ctrl->minconn) 
       printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
     printf("\n");
  }

  queue = rpqCreate(nvtxs);

  /*=====================================================================
  * The top-level refinement loop 
  *======================================================================*/
  for (pass=0; pass<niter; pass++) {
    ASSERT(ComputeCut(graph, where) == graph->mincut);

    if (omode == OMODE_BALANCE) {
      /* Check to see if things are out of balance, given the tolerance */
      for (i=0; i<nparts; i++) {
        if (pwgts[i] > maxwgt[i])
          break;
      }
      if (i == nparts) /* Things are balanced. Return right away */
        break;
    }

    oldcut = graph->mincut;
    nbnd   = graph->nbnd;
    nupd   = 0;

    if (ctrl->minconn)
      maxndoms = imax(nparts, nads);

    /* Insert the boundary vertices in the priority queue */
    irandArrayPermute(nbnd, perm, nbnd/4, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = bndind[perm[ii]];
      rgain = (graph->ckrinfo[i].nnbrs > 0 ? 
               1.0*graph->ckrinfo[i].ed/sqrt(graph->ckrinfo[i].nnbrs) : 0.0) 
               - graph->ckrinfo[i].id;
      rpqInsert(queue, i, rgain);
      vstatus[i] = VPQSTATUS_PRESENT;
      ListInsert(nupd, updind, updptr, i);
    }

    /* Start extracting vertices from the queue and try to move them */
    for (nmoved=0, iii=0;;iii++) {
      if ((i = rpqGetTop(queue)) == -1) 
        break;
      vstatus[i] = VPQSTATUS_EXTRACTED;

      myrinfo = graph->ckrinfo+i;
      mynbrs  = ctrl->cnbrpool + myrinfo->inbr;

      from = where[i];
      vwgt = graph->vwgt[i];

      /* Prevent moves that make 'from' domain underbalanced */
      if (omode == OMODE_REFINE) {
        if (myrinfo->id > 0 && pwgts[from]-vwgt < minwgt[from]) 
          continue;   
      }
      else { /* OMODE_BALANCE */
        if (pwgts[from]-vwgt < minwgt[from]) 
          continue;   
      }

      if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
        continue;

      if (ctrl->minconn)
        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

      /* Find the most promising subdomain to move to */
      if (omode == OMODE_REFINE) {
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          gain = mynbrs[k].ed-myrinfo->id; 
          if (gain >= 0 && pwgts[to]+vwgt <= maxwgt[to]+ffactor*gain)  
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          gain = mynbrs[j].ed-myrinfo->id; 
          if ((mynbrs[j].ed > mynbrs[k].ed && pwgts[to]+vwgt <= maxwgt[to]+ffactor*gain) 
              ||
              (mynbrs[j].ed == mynbrs[k].ed && 
               itpwgts[mynbrs[k].pid]*pwgts[to] < itpwgts[to]*pwgts[mynbrs[k].pid]))
            k = j;
        }

        to = mynbrs[k].pid;

        gain = mynbrs[k].ed-myrinfo->id;
        if (!(gain > 0 
              || (gain == 0  
                  && (pwgts[from] >= maxwgt[from] 
                      || itpwgts[to]*pwgts[from] > itpwgts[from]*(pwgts[to]+vwgt) 
                      || (iii%2 == 0 && safetos[to] == 2)
                     )
                 )
             )
           )
          continue;
      }
      else {  /* OMODE_BALANCE */
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          if (pwgts[to]+vwgt <= maxwgt[to] || 
              itpwgts[from]*(pwgts[to]+vwgt) <= itpwgts[to]*pwgts[from]) 
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          if (itpwgts[mynbrs[k].pid]*pwgts[to] < itpwgts[to]*pwgts[mynbrs[k].pid]) 
            k = j;
        }

        to = mynbrs[k].pid;

        if (pwgts[from] < maxwgt[from] && pwgts[to] > minwgt[to] && 
            mynbrs[k].ed-myrinfo->id < 0) 
          continue;
      }



      /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to' 
      *======================================================================*/
      graph->mincut -= mynbrs[k].ed-myrinfo->id;
      nmoved++;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
          printf("\t\tMoving %6"PRIDX" to %3"PRIDX". Gain: %4"PRIDX". Cut: %6"PRIDX"\n", 
              i, to, mynbrs[k].ed-myrinfo->id, graph->mincut));

      /* Update the subdomain connectivity information */
      if (ctrl->minconn) {
        /* take care of i's move itself */
        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id-mynbrs[k].ed, &maxndoms);

        /* take care of the adjancent vertices */
        for (j=xadj[i]; j<xadj[i+1]; j++) {
          me = where[adjncy[j]];
          if (me != from && me != to) {
            UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
            UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
          }
        }
      }

      /* Update ID/ED and BND related information for the moved vertex */
      INC_DEC(pwgts[to], pwgts[from], vwgt);
      UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, nbnd, 
          bndptr, bndind, bndtype);
      
      /* Update the degrees of adjacent vertices */
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii = adjncy[j];
        me = where[ii];
        myrinfo = graph->ckrinfo+ii;

        oldnnbrs = myrinfo->nnbrs;

        UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii+1]-xadj[ii], me, 
            from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

        UpdateQueueInfo(queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs, 
            nupd, updptr, updind, bndtype);

        ASSERT(myrinfo->nnbrs <= xadj[ii+1]-xadj[ii]);
      }
    }

    graph->nbnd = nbnd;

    /* Reset the vstatus and associated data structures */
    for (i=0; i<nupd; i++) {
      ASSERT(updptr[updind[i]] != -1);
      ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
      vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
      updptr[updind[i]]  = -1;
    }

    if (ctrl->dbglvl&METIS_DBG_REFINE) {
       printf("\t[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL", Nb: %6"PRIDX"."
              " Nmoves: %5"PRIDX", Cut: %6"PRIDX", Vol: %6"PRIDX,
              pwgts[iargmin(nparts, pwgts)], imax(nparts, pwgts),
              ComputeLoadImbalance(graph, nparts, ctrl->pijbm), 
              graph->nbnd, nmoved, graph->mincut, ComputeVolume(graph, where));
       if (ctrl->minconn) 
         printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
       printf("\n");
    }

    if (nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
      break;
  }

  rpqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************/
/*! K-way refinement that minimizes the communication volume. This is a 
    greedy routine and the vertices are visited in decreasing gv order.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves 
         to violate the max-pwgt constraint.

*/
/**************************************************************************/
void Greedy_KWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode)
{
  /* Common variables to all types of kway-refinement/balancing routines */
  idx_t i, ii, iii, j, k, l, pass, nvtxs, nparts, gain; 
  idx_t from, me, to, oldcut, vwgt;
  idx_t *xadj, *adjncy;
  idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minwgt, *maxwgt, *itpwgts;
  idx_t nmoved, nupd, *vstatus, *updptr, *updind;
  idx_t maxndoms, *safetos=NULL, *nads=NULL, *doms=NULL, **adids=NULL, **adwgts=NULL;
  idx_t *bfslvl=NULL, *bfsind=NULL, *bfsmrk=NULL;
  idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);

  /* Volume-specific/different variables */
  ipq_t *queue;
  idx_t oldvol, xgain;
  idx_t *vmarker, *pmarker, *modind;
  vkrinfo_t *myrinfo;
  vnbr_t *mynbrs;

  WCOREPUSH;

  /* Link the graph fields */
  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  bndptr = graph->bndptr;
  bndind = graph->bndind;
  where  = graph->where;
  pwgts  = graph->pwgts;
  
  nparts = ctrl->nparts;

  /* Setup the weight intervals of the various subdomains */
  minwgt  = iwspacemalloc(ctrl, nparts);
  maxwgt  = iwspacemalloc(ctrl, nparts);
  itpwgts = iwspacemalloc(ctrl, nparts);

  for (i=0; i<nparts; i++) {
    itpwgts[i] = ctrl->tpwgts[i]*graph->tvwgt[0];
    maxwgt[i]  = ctrl->tpwgts[i]*graph->tvwgt[0]*ctrl->ubfactors[0];
    minwgt[i]  = ctrl->tpwgts[i]*graph->tvwgt[0]*(1.0/ctrl->ubfactors[0]);
  }

  perm = iwspacemalloc(ctrl, nvtxs);


  /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made. 
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
  safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

  if (ctrl->minconn) {
    ComputeSubDomainGraph(ctrl, graph);

    nads    = ctrl->nads;
    adids   = ctrl->adids;
    adwgts  = ctrl->adwgts;
    doms    = iset(nparts, 0, ctrl->pvec1);
  }


  /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
  vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
  updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
  updind  = iwspacemalloc(ctrl, nvtxs);

  if (ctrl->contig) {
    /* The arrays that will be used for limited check of articulation points */
    bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    bfsind = iwspacemalloc(ctrl, nvtxs);
    bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  }

  /* Vol-refinement specific working arrays */
  modind  = iwspacemalloc(ctrl, nvtxs);
  vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
     printf("%s: [%6"PRIDX" %6"PRIDX"]-[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL
         ", Nv-Nb[%6"PRIDX" %6"PRIDX"], Cut: %5"PRIDX", Vol: %5"PRIDX,
         (omode == OMODE_REFINE ? "GRV" : "GBV"),
         pwgts[iargmin(nparts, pwgts)], imax(nparts, pwgts), minwgt[0], maxwgt[0], 
         ComputeLoadImbalance(graph, nparts, ctrl->pijbm), 
         graph->nvtxs, graph->nbnd, graph->mincut, graph->minvol);
     if (ctrl->minconn) 
       printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
     printf("\n");
  }

  queue = ipqCreate(nvtxs);


  /*=====================================================================
  * The top-level refinement loop 
  *======================================================================*/
  for (pass=0; pass<niter; pass++) {
    ASSERT(ComputeVolume(graph, where) == graph->minvol);

    if (omode == OMODE_BALANCE) {
      /* Check to see if things are out of balance, given the tolerance */
      for (i=0; i<nparts; i++) {
        if (pwgts[i] > maxwgt[i])
          break;
      }
      if (i == nparts) /* Things are balanced. Return right away */
        break;
    }

    oldcut = graph->mincut;
    oldvol = graph->minvol;
    nupd   = 0;

    if (ctrl->minconn)
      maxndoms = imax(nparts, nads);

    /* Insert the boundary vertices in the priority queue */
    irandArrayPermute(graph->nbnd, perm, graph->nbnd/4, 1);
    for (ii=0; ii<graph->nbnd; ii++) {
      i = bndind[perm[ii]];
      ipqInsert(queue, i, graph->vkrinfo[i].gv);
      vstatus[i] = VPQSTATUS_PRESENT;
      ListInsert(nupd, updind, updptr, i);
    }

    /* Start extracting vertices from the queue and try to move them */
    for (nmoved=0, iii=0;;iii++) {
      if ((i = ipqGetTop(queue)) == -1) 
        break;
      vstatus[i] = VPQSTATUS_EXTRACTED;

      myrinfo = graph->vkrinfo+i;
      mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

      from = where[i];
      vwgt = graph->vwgt[i];

      /* Prevent moves that make 'from' domain underbalanced */
      if (omode == OMODE_REFINE) {
        if (myrinfo->nid > 0 && pwgts[from]-vwgt < minwgt[from]) 
          continue;
      }
      else { /* OMODE_BALANCE */
        if (pwgts[from]-vwgt < minwgt[from]) 
          continue;
      }

      if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
        continue;

      if (ctrl->minconn)
        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

      xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

      /* Find the most promising subdomain to move to */
      if (omode == OMODE_REFINE) {
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          gain = mynbrs[k].gv + xgain;
          if (gain >= 0 && pwgts[to]+vwgt <= maxwgt[to]+ffactor*gain)  
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          gain = mynbrs[j].gv + xgain;
          if ((mynbrs[j].gv > mynbrs[k].gv && 
               pwgts[to]+vwgt <= maxwgt[to]+ffactor*gain) 
              ||
              (mynbrs[j].gv == mynbrs[k].gv && 
               mynbrs[j].ned > mynbrs[k].ned &&
               pwgts[to]+vwgt <= maxwgt[to]) 
              ||
              (mynbrs[j].gv == mynbrs[k].gv && 
               mynbrs[j].ned == mynbrs[k].ned &&
               itpwgts[mynbrs[k].pid]*pwgts[to] < itpwgts[to]*pwgts[mynbrs[k].pid])
             )
            k = j;
        }
        to = mynbrs[k].pid;

        ASSERT(xgain+mynbrs[k].gv >= 0);

        j = 0;
        if (xgain+mynbrs[k].gv > 0 || mynbrs[k].ned-myrinfo->nid > 0)
          j = 1;
        else if (mynbrs[k].ned-myrinfo->nid == 0) {
          if ((iii%2 == 0 && safetos[to] == 2) || 
              pwgts[from] >= maxwgt[from] || 
              itpwgts[from]*(pwgts[to]+vwgt) < itpwgts[to]*pwgts[from])
            j = 1;
        }
        if (j == 0)
          continue;
      }
      else { /* OMODE_BALANCE */
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          if (pwgts[to]+vwgt <= maxwgt[to] || 
              itpwgts[from]*(pwgts[to]+vwgt) <= itpwgts[to]*pwgts[from])  
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          if (itpwgts[mynbrs[k].pid]*pwgts[to] < itpwgts[to]*pwgts[mynbrs[k].pid])
            k = j;
        }
        to = mynbrs[k].pid;

        if (pwgts[from] < maxwgt[from] && pwgts[to] > minwgt[to] && 
            (xgain+mynbrs[k].gv < 0 || 
             (xgain+mynbrs[k].gv == 0 &&  mynbrs[k].ned-myrinfo->nid < 0))
           )
          continue;
      }
          
          
      /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to' 
      *======================================================================*/
      INC_DEC(pwgts[to], pwgts[from], vwgt);
      graph->mincut -= mynbrs[k].ned-myrinfo->nid;
      graph->minvol -= (xgain+mynbrs[k].gv);
      where[i] = to;
      nmoved++;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
          printf("\t\tMoving %6"PRIDX" from %3"PRIDX" to %3"PRIDX". "
                 "Gain: [%4"PRIDX" %4"PRIDX"]. Cut: %6"PRIDX", Vol: %6"PRIDX"\n", 
              i, from, to, xgain+mynbrs[k].gv, mynbrs[k].ned-myrinfo->nid, 
              graph->mincut, graph->minvol));

      /* Update the subdomain connectivity information */
      if (ctrl->minconn) {
        /* take care of i's move itself */
        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->nid-mynbrs[k].ned, &maxndoms);

        /* take care of the adjancent vertices */
        for (j=xadj[i]; j<xadj[i+1]; j++) {
          me = where[adjncy[j]];
          if (me != from && me != to) {
            UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
            UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
          }
        }
      }

      /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
      KWayVolUpdate(ctrl, graph, i, from, to, queue, vstatus, &nupd, updptr, 
          updind, bndtype, vmarker, pmarker, modind);

      /*CheckKWayVolPartitionParams(ctrl, graph); */
    }


    /* Reset the vstatus and associated data structures */
    for (i=0; i<nupd; i++) {
      ASSERT(updptr[updind[i]] != -1);
      ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
      vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
      updptr[updind[i]]  = -1;
    }

    if (ctrl->dbglvl&METIS_DBG_REFINE) {
       printf("\t[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL", Nb: %6"PRIDX"."
              " Nmoves: %5"PRIDX", Cut: %6"PRIDX", Vol: %6"PRIDX,
              pwgts[iargmin(nparts, pwgts)], imax(nparts, pwgts),
              ComputeLoadImbalance(graph, nparts, ctrl->pijbm), 
              graph->nbnd, nmoved, graph->mincut, graph->minvol);
       if (ctrl->minconn) 
         printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
       printf("\n");
    }

    if (nmoved == 0 || 
        (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
      break;
  }

  ipqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************/
/*! K-way partitioning optimization in which the vertices are visited in 
    decreasing ed/sqrt(nnbrs)-id order. Note this is just an 
    approximation, as the ed is often split across different subdomains 
    and the sqrt(nnbrs) is just a crude approximation.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves 
         to violate the max-pwgt constraint.
  \param omode is the type of optimization that will performed among
         OMODE_REFINE and OMODE_BALANCE 
         

*/
/**************************************************************************/
void Greedy_McKWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode)
{
  /* Common variables to all types of kway-refinement/balancing routines */
  idx_t i, ii, iii, j, k, l, pass, nvtxs, ncon, nparts, gain; 
  idx_t from, me, to, cto, oldcut;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt;
  idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minwgt, *maxwgt;
  idx_t nmoved, nupd, *vstatus, *updptr, *updind;
  idx_t maxndoms, *safetos=NULL, *nads=NULL, *doms=NULL, **adids=NULL, **adwgts=NULL;
  idx_t *bfslvl=NULL, *bfsind=NULL, *bfsmrk=NULL;
  idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
  real_t *ubfactors, *pijbm;
  real_t origbal;

  /* Edgecut-specific/different variables */
  idx_t nbnd, oldnnbrs;
  rpq_t *queue;
  real_t rgain;
  ckrinfo_t *myrinfo;
  cnbr_t *mynbrs;

  WCOREPUSH;

  /* Link the graph fields */
  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;

  bndind = graph->bndind;
  bndptr = graph->bndptr;

  where = graph->where;
  pwgts = graph->pwgts;
  
  nparts = ctrl->nparts;
  pijbm  = ctrl->pijbm;


  /* Determine the ubfactors. The method used is different based on omode. 
     When OMODE_BALANCE, the ubfactors are those supplied by the user. 
     When OMODE_REFINE, the ubfactors are the max of the current partition
     and the user-specified ones. */
  ubfactors = rwspacemalloc(ctrl, ncon);
  ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors);
  origbal = rvecmaxdiff(ncon, ubfactors, ctrl->ubfactors);
  if (omode == OMODE_BALANCE) {
    rcopy(ncon, ctrl->ubfactors, ubfactors);
  }
  else {
    for (i=0; i<ncon; i++)
      ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] : ctrl->ubfactors[i]);
  }


  /* Setup the weight intervals of the various subdomains */
  minwgt  = iwspacemalloc(ctrl, nparts*ncon);
  maxwgt  = iwspacemalloc(ctrl, nparts*ncon);

  for (i=0; i<nparts; i++) {
    for (j=0; j<ncon; j++) {
      maxwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*ubfactors[j];
      /*minwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]);*/
      minwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*.2;
    }
  }

  perm = iwspacemalloc(ctrl, nvtxs);


  /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made. 
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
  safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

  if (ctrl->minconn) {
    ComputeSubDomainGraph(ctrl, graph);

    nads    = ctrl->nads;
    adids   = ctrl->adids;
    adwgts  = ctrl->adwgts;
    doms    = iset(nparts, 0, ctrl->pvec1);
  }


  /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
  vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
  updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
  updind  = iwspacemalloc(ctrl, nvtxs);

  if (ctrl->contig) {
    /* The arrays that will be used for limited check of articulation points */
    bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    bfsind = iwspacemalloc(ctrl, nvtxs);
    bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  }

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
     printf("%s: [%6"PRIDX" %6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL"(%.3"PRREAL")," 
            " Nv-Nb[%6"PRIDX" %6"PRIDX"], Cut: %6"PRIDX", (%"PRIDX")",
            (omode == OMODE_REFINE ? "GRC" : "GBC"),
            imin(nparts*ncon, pwgts), imax(nparts*ncon, pwgts), imax(nparts*ncon, maxwgt),
            ComputeLoadImbalance(graph, nparts, pijbm), origbal,
            graph->nvtxs, graph->nbnd, graph->mincut, niter);
     if (ctrl->minconn) 
       printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
     printf("\n");
  }

  queue = rpqCreate(nvtxs);


  /*=====================================================================
  * The top-level refinement loop 
  *======================================================================*/
  for (pass=0; pass<niter; pass++) {
    ASSERT(ComputeCut(graph, where) == graph->mincut);

    /* In balancing mode, exit as soon as balance is reached */
    if (omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0)) 
      break;
    
    oldcut = graph->mincut;
    nbnd   = graph->nbnd;
    nupd   = 0;

    if (ctrl->minconn)
      maxndoms = imax(nparts, nads);

    /* Insert the boundary vertices in the priority queue */
    irandArrayPermute(nbnd, perm, nbnd/4, 1);
    for (ii=0; ii<nbnd; ii++) {
      i = bndind[perm[ii]];
      rgain = (graph->ckrinfo[i].nnbrs > 0 ? 
               1.0*graph->ckrinfo[i].ed/sqrt(graph->ckrinfo[i].nnbrs) : 0.0) 
               - graph->ckrinfo[i].id;
      rpqInsert(queue, i, rgain);
      vstatus[i] = VPQSTATUS_PRESENT;
      ListInsert(nupd, updind, updptr, i);
    }

    /* Start extracting vertices from the queue and try to move them */
    for (nmoved=0, iii=0;;iii++) {
      if ((i = rpqGetTop(queue)) == -1) 
        break;
      vstatus[i] = VPQSTATUS_EXTRACTED;

      myrinfo = graph->ckrinfo+i;
      mynbrs  = ctrl->cnbrpool + myrinfo->inbr;

      from = where[i];

      /* Prevent moves that make 'from' domain underbalanced */
      if (omode == OMODE_REFINE) {
        if (myrinfo->id > 0 && 
            !ivecaxpygez(ncon, -1, vwgt+i*ncon, pwgts+from*ncon, minwgt+from*ncon))
          continue;   
      }
      else { /* OMODE_BALANCE */
        if (!ivecaxpygez(ncon, -1, vwgt+i*ncon, pwgts+from*ncon, minwgt+from*ncon)) 
          continue;   
      }

      if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
        continue;

      if (ctrl->minconn)
        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

      /* Find the most promising subdomain to move to */
      if (omode == OMODE_REFINE) {
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          gain = mynbrs[k].ed-myrinfo->id; 
          if (gain >= 0 && ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon))
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        cto = to;
        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          if ((mynbrs[j].ed > mynbrs[k].ed && 
               ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon))
              ||
              (mynbrs[j].ed == mynbrs[k].ed && 
               BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors, 
                   1, pwgts+cto*ncon, pijbm+cto*ncon,
                   1, pwgts+to*ncon, pijbm+to*ncon))) {
            k   = j;
            cto = to;
          }
        }
        to = cto;

        gain = mynbrs[k].ed-myrinfo->id;
        if (!(gain > 0 
              || (gain == 0  
                  && (BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                             -1, pwgts+from*ncon, pijbm+from*ncon,
                             +1, pwgts+to*ncon, pijbm+to*ncon)
                      || (iii%2 == 0 && safetos[to] == 2)
                     )
                 )
             )
           )
          continue;
      }
      else {  /* OMODE_BALANCE */
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          if (ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon) || 
              BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                  -1, pwgts+from*ncon, pijbm+from*ncon,
                  +1, pwgts+to*ncon, pijbm+to*ncon))
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        cto = to;
        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          if (BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors, 
                   1, pwgts+cto*ncon, pijbm+cto*ncon,
                   1, pwgts+to*ncon, pijbm+to*ncon)) {
            k   = j;
            cto = to;
          }
        }
        to = cto;

        if (mynbrs[k].ed-myrinfo->id < 0 &&
            !BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                  -1, pwgts+from*ncon, pijbm+from*ncon,
                  +1, pwgts+to*ncon, pijbm+to*ncon))
          continue;
      }



      /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to' 
      *======================================================================*/
      graph->mincut -= mynbrs[k].ed-myrinfo->id;
      nmoved++;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
          printf("\t\tMoving %6"PRIDX" to %3"PRIDX". Gain: %4"PRIDX". Cut: %6"PRIDX"\n", 
              i, to, mynbrs[k].ed-myrinfo->id, graph->mincut));

      /* Update the subdomain connectivity information */
      if (ctrl->minconn) {
        /* take care of i's move itself */
        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->id-mynbrs[k].ed, &maxndoms);

        /* take care of the adjancent vertices */
        for (j=xadj[i]; j<xadj[i+1]; j++) {
          me = where[adjncy[j]];
          if (me != from && me != to) {
            UpdateEdgeSubDomainGraph(ctrl, from, me, -adjwgt[j], &maxndoms);
            UpdateEdgeSubDomainGraph(ctrl, to, me, adjwgt[j], &maxndoms);
          }
        }
      }

      /* Update ID/ED and BND related information for the moved vertex */
      iaxpy(ncon,  1, vwgt+i*ncon, 1, pwgts+to*ncon,   1);
      iaxpy(ncon, -1, vwgt+i*ncon, 1, pwgts+from*ncon, 1);
      UpdateMovedVertexInfoAndBND(i, from, k, to, myrinfo, mynbrs, where, 
          nbnd, bndptr, bndind, bndtype);
      
      /* Update the degrees of adjacent vertices */
      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii = adjncy[j];
        me = where[ii];
        myrinfo = graph->ckrinfo+ii;

        oldnnbrs = myrinfo->nnbrs;

        UpdateAdjacentVertexInfoAndBND(ctrl, ii, xadj[ii+1]-xadj[ii], me, 
            from, to, myrinfo, adjwgt[j], nbnd, bndptr, bndind, bndtype);

        UpdateQueueInfo(queue, vstatus, ii, me, from, to, myrinfo, oldnnbrs, 
            nupd, updptr, updind, bndtype);

        ASSERT(myrinfo->nnbrs <= xadj[ii+1]-xadj[ii]);
      }
    }

    graph->nbnd = nbnd;

    /* Reset the vstatus and associated data structures */
    for (i=0; i<nupd; i++) {
      ASSERT(updptr[updind[i]] != -1);
      ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
      vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
      updptr[updind[i]]  = -1;
    }

    if (ctrl->dbglvl&METIS_DBG_REFINE) {
       printf("\t[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL", Nb: %6"PRIDX"."
              " Nmoves: %5"PRIDX", Cut: %6"PRIDX", Vol: %6"PRIDX,
              imin(nparts*ncon, pwgts), imax(nparts*ncon, pwgts), 
              ComputeLoadImbalance(graph, nparts, pijbm), 
              graph->nbnd, nmoved, graph->mincut, ComputeVolume(graph, where));
       if (ctrl->minconn) 
         printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
       printf("\n");
    }

    if (nmoved == 0 || (omode == OMODE_REFINE && graph->mincut == oldcut))
      break;
  }

  rpqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************/
/*! K-way refinement that minimizes the communication volume. This is a 
    greedy routine and the vertices are visited in decreasing gv order.

  \param graph is the graph that is being refined.
  \param niter is the number of refinement iterations.
  \param ffactor is the \em fudge-factor for allowing positive gain moves 
         to violate the max-pwgt constraint.

*/
/**************************************************************************/
void Greedy_McKWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode)
{
  /* Common variables to all types of kway-refinement/balancing routines */
  idx_t i, ii, iii, j, k, l, pass, nvtxs, ncon, nparts, gain; 
  idx_t from, me, to, cto, oldcut;
  idx_t *xadj, *vwgt, *adjncy;
  idx_t *where, *pwgts, *perm, *bndptr, *bndind, *minwgt, *maxwgt;
  idx_t nmoved, nupd, *vstatus, *updptr, *updind;
  idx_t maxndoms, *safetos=NULL, *nads=NULL, *doms=NULL, **adids=NULL, **adwgts=NULL;
  idx_t *bfslvl=NULL, *bfsind=NULL, *bfsmrk=NULL;
  idx_t bndtype = (omode == OMODE_REFINE ? BNDTYPE_REFINE : BNDTYPE_BALANCE);
  real_t *ubfactors, *pijbm;
  real_t origbal;

  /* Volume-specific/different variables */
  ipq_t *queue;
  idx_t oldvol, xgain;
  idx_t *vmarker, *pmarker, *modind;
  vkrinfo_t *myrinfo;
  vnbr_t *mynbrs;

  WCOREPUSH;

  /* Link the graph fields */
  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  bndptr = graph->bndptr;
  bndind = graph->bndind;
  where  = graph->where;
  pwgts  = graph->pwgts;
  
  nparts = ctrl->nparts;
  pijbm  = ctrl->pijbm;


  /* Determine the ubfactors. The method used is different based on omode. 
     When OMODE_BALANCE, the ubfactors are those supplied by the user. 
     When OMODE_REFINE, the ubfactors are the max of the current partition
     and the user-specified ones. */
  ubfactors = rwspacemalloc(ctrl, ncon);
  ComputeLoadImbalanceVec(graph, nparts, pijbm, ubfactors);
  origbal = rvecmaxdiff(ncon, ubfactors, ctrl->ubfactors);
  if (omode == OMODE_BALANCE) {
    rcopy(ncon, ctrl->ubfactors, ubfactors);
  }
  else {
    for (i=0; i<ncon; i++)
      ubfactors[i] = (ubfactors[i] > ctrl->ubfactors[i] ? ubfactors[i] : ctrl->ubfactors[i]);
  }


  /* Setup the weight intervals of the various subdomains */
  minwgt  = iwspacemalloc(ctrl, nparts*ncon);
  maxwgt  = iwspacemalloc(ctrl, nparts*ncon);

  for (i=0; i<nparts; i++) {
    for (j=0; j<ncon; j++) {
      maxwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*ubfactors[j];
      /*minwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*(.9/ubfactors[j]); */
      minwgt[i*ncon+j]  = ctrl->tpwgts[i*ncon+j]*graph->tvwgt[j]*.2;
    }
  }

  perm = iwspacemalloc(ctrl, nvtxs);


  /* This stores the valid target subdomains. It is used when ctrl->minconn to
     control the subdomains to which moves are allowed to be made. 
     When ctrl->minconn is false, the default values of 2 allow all moves to
     go through and it does not interfere with the zero-gain move selection. */
  safetos = iset(nparts, 2, iwspacemalloc(ctrl, nparts));

  if (ctrl->minconn) {
    ComputeSubDomainGraph(ctrl, graph);

    nads    = ctrl->nads;
    adids   = ctrl->adids;
    adwgts  = ctrl->adwgts;
    doms    = iset(nparts, 0, ctrl->pvec1);
  }


  /* Setup updptr, updind like boundary info to keep track of the vertices whose
     vstatus's need to be reset at the end of the inner iteration */
  vstatus = iset(nvtxs, VPQSTATUS_NOTPRESENT, iwspacemalloc(ctrl, nvtxs));
  updptr  = iset(nvtxs, -1, iwspacemalloc(ctrl, nvtxs));
  updind  = iwspacemalloc(ctrl, nvtxs);

  if (ctrl->contig) {
    /* The arrays that will be used for limited check of articulation points */
    bfslvl = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
    bfsind = iwspacemalloc(ctrl, nvtxs);
    bfsmrk = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  }

  /* Vol-refinement specific working arrays */
  modind  = iwspacemalloc(ctrl, nvtxs);
  vmarker = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  pmarker = iset(nparts, -1, iwspacemalloc(ctrl, nparts));

  if (ctrl->dbglvl&METIS_DBG_REFINE) {
     printf("%s: [%6"PRIDX" %6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL"(%.3"PRREAL"),"
         ", Nv-Nb[%6"PRIDX" %6"PRIDX"], Cut: %5"PRIDX", Vol: %5"PRIDX", (%"PRIDX")",
         (omode == OMODE_REFINE ? "GRV" : "GBV"),
         imin(nparts*ncon, pwgts), imax(nparts*ncon, pwgts), imax(nparts*ncon, maxwgt),
         ComputeLoadImbalance(graph, nparts, pijbm), origbal,
         graph->nvtxs, graph->nbnd, graph->mincut, graph->minvol, niter);
     if (ctrl->minconn) 
       printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
     printf("\n");
  }

  queue = ipqCreate(nvtxs);


  /*=====================================================================
  * The top-level refinement loop 
  *======================================================================*/
  for (pass=0; pass<niter; pass++) {
    ASSERT(ComputeVolume(graph, where) == graph->minvol);

    /* In balancing mode, exit as soon as balance is reached */
    if (omode == OMODE_BALANCE && IsBalanced(ctrl, graph, 0))
      break;

    oldcut = graph->mincut;
    oldvol = graph->minvol;
    nupd   = 0;

    if (ctrl->minconn)
      maxndoms = imax(nparts, nads);

    /* Insert the boundary vertices in the priority queue */
    irandArrayPermute(graph->nbnd, perm, graph->nbnd/4, 1);
    for (ii=0; ii<graph->nbnd; ii++) {
      i = bndind[perm[ii]];
      ipqInsert(queue, i, graph->vkrinfo[i].gv);
      vstatus[i] = VPQSTATUS_PRESENT;
      ListInsert(nupd, updind, updptr, i);
    }

    /* Start extracting vertices from the queue and try to move them */
    for (nmoved=0, iii=0;;iii++) {
      if ((i = ipqGetTop(queue)) == -1) 
        break;
      vstatus[i] = VPQSTATUS_EXTRACTED;

      myrinfo = graph->vkrinfo+i;
      mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

      from = where[i];

      /* Prevent moves that make 'from' domain underbalanced */
      if (omode == OMODE_REFINE) {
        if (myrinfo->nid > 0 &&
            !ivecaxpygez(ncon, -1, vwgt+i*ncon, pwgts+from*ncon, minwgt+from*ncon))
          continue;
      }
      else { /* OMODE_BALANCE */
        if (!ivecaxpygez(ncon, -1, vwgt+i*ncon, pwgts+from*ncon, minwgt+from*ncon))
          continue;
      }

      if (ctrl->contig && IsArticulationNode(i, xadj, adjncy, where, bfslvl, bfsind, bfsmrk))
        continue;

      if (ctrl->minconn)
        SelectSafeTargetSubdomains(myrinfo, mynbrs, nads, adids, maxndoms, safetos, doms);

      xgain = (myrinfo->nid == 0 && myrinfo->ned > 0 ? graph->vsize[i] : 0);

      /* Find the most promising subdomain to move to */
      if (omode == OMODE_REFINE) {
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          gain = mynbrs[k].gv + xgain;
          if (gain >= 0 && ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon))
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        cto = to;
        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          gain = mynbrs[j].gv + xgain;
          if ((mynbrs[j].gv > mynbrs[k].gv && 
               ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon))
              ||
              (mynbrs[j].gv == mynbrs[k].gv && 
               mynbrs[j].ned > mynbrs[k].ned &&
               ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon))
              ||
              (mynbrs[j].gv == mynbrs[k].gv && 
               mynbrs[j].ned == mynbrs[k].ned &&
               BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                   1, pwgts+cto*ncon, pijbm+cto*ncon,
                   1, pwgts+to*ncon, pijbm+to*ncon))) {
            k   = j;
            cto = to;
          }
        }
        to = cto;

        j = 0;
        if (xgain+mynbrs[k].gv > 0 || mynbrs[k].ned-myrinfo->nid > 0)
          j = 1;
        else if (mynbrs[k].ned-myrinfo->nid == 0) {
          if ((iii%2 == 0 && safetos[to] == 2) ||
              BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                  -1, pwgts+from*ncon, pijbm+from*ncon,
                  +1, pwgts+to*ncon, pijbm+to*ncon))
            j = 1;
        }
        if (j == 0)
          continue;
      }
      else { /* OMODE_BALANCE */
        for (k=myrinfo->nnbrs-1; k>=0; k--) {
          if (!safetos[to=mynbrs[k].pid])
            continue;
          if (ivecaxpylez(ncon, 1, vwgt+i*ncon, pwgts+to*ncon, maxwgt+to*ncon) ||
              BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                  -1, pwgts+from*ncon, pijbm+from*ncon,
                  +1, pwgts+to*ncon, pijbm+to*ncon))
            break;
        }
        if (k < 0)
          continue;  /* break out if you did not find a candidate */

        cto = to;
        for (j=k-1; j>=0; j--) {
          if (!safetos[to=mynbrs[j].pid])
            continue;
          if (BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                  1, pwgts+cto*ncon, pijbm+cto*ncon,
                  1, pwgts+to*ncon, pijbm+to*ncon)) {
            k   = j;
            cto = to;
          }
        }
        to = cto;

        if ((xgain+mynbrs[k].gv < 0 || 
             (xgain+mynbrs[k].gv == 0 && mynbrs[k].ned-myrinfo->nid < 0))
            &&
            !BetterBalanceKWay(ncon, vwgt+i*ncon, ubfactors,
                 -1, pwgts+from*ncon, pijbm+from*ncon,
                 +1, pwgts+to*ncon, pijbm+to*ncon))
          continue;
      }
          
          
      /*=====================================================================
      * If we got here, we can now move the vertex from 'from' to 'to' 
      *======================================================================*/
      graph->mincut -= mynbrs[k].ned-myrinfo->nid;
      graph->minvol -= (xgain+mynbrs[k].gv);
      where[i] = to;
      nmoved++;

      IFSET(ctrl->dbglvl, METIS_DBG_MOVEINFO, 
          printf("\t\tMoving %6"PRIDX" from %3"PRIDX" to %3"PRIDX". "
                 "Gain: [%4"PRIDX" %4"PRIDX"]. Cut: %6"PRIDX", Vol: %6"PRIDX"\n", 
              i, from, to, xgain+mynbrs[k].gv, mynbrs[k].ned-myrinfo->nid, 
              graph->mincut, graph->minvol));

      /* Update the subdomain connectivity information */
      if (ctrl->minconn) {
        /* take care of i's move itself */
        UpdateEdgeSubDomainGraph(ctrl, from, to, myrinfo->nid-mynbrs[k].ned, &maxndoms);

        /* take care of the adjancent vertices */
        for (j=xadj[i]; j<xadj[i+1]; j++) {
          me = where[adjncy[j]];
          if (me != from && me != to) {
            UpdateEdgeSubDomainGraph(ctrl, from, me, -1, &maxndoms);
            UpdateEdgeSubDomainGraph(ctrl, to, me, 1, &maxndoms);
          }
        }
      }

      /* Update pwgts */
      iaxpy(ncon,  1, vwgt+i*ncon, 1, pwgts+to*ncon,   1);
      iaxpy(ncon, -1, vwgt+i*ncon, 1, pwgts+from*ncon, 1);

      /* Update the id/ed/gains/bnd/queue of potentially affected nodes */
      KWayVolUpdate(ctrl, graph, i, from, to, queue, vstatus, &nupd, updptr, 
          updind, bndtype, vmarker, pmarker, modind);

      /*CheckKWayVolPartitionParams(ctrl, graph); */
    }


    /* Reset the vstatus and associated data structures */
    for (i=0; i<nupd; i++) {
      ASSERT(updptr[updind[i]] != -1);
      ASSERT(vstatus[updind[i]] != VPQSTATUS_NOTPRESENT);
      vstatus[updind[i]] = VPQSTATUS_NOTPRESENT;
      updptr[updind[i]]  = -1;
    }

    if (ctrl->dbglvl&METIS_DBG_REFINE) {
       printf("\t[%6"PRIDX" %6"PRIDX"], Bal: %5.3"PRREAL", Nb: %6"PRIDX"."
              " Nmoves: %5"PRIDX", Cut: %6"PRIDX", Vol: %6"PRIDX,
              imin(nparts*ncon, pwgts), imax(nparts*ncon, pwgts), 
              ComputeLoadImbalance(graph, nparts, pijbm), 
              graph->nbnd, nmoved, graph->mincut, graph->minvol);
       if (ctrl->minconn) 
         printf(", Doms: [%3"PRIDX" %4"PRIDX"]", imax(nparts, nads), isum(nparts, nads,1));
       printf("\n");
    }

    if (nmoved == 0 || 
        (omode == OMODE_REFINE && graph->minvol == oldvol && graph->mincut == oldcut))
      break;
  }

  ipqDestroy(queue);

  WCOREPOP;
}


/*************************************************************************/
/*! This function performs an approximate articulation vertex test.
    It assumes that the bfslvl, bfsind, and bfsmrk arrays are initialized
    appropriately. */
/*************************************************************************/
idx_t IsArticulationNode(idx_t i, idx_t *xadj, idx_t *adjncy, idx_t *where,
          idx_t *bfslvl, idx_t *bfsind, idx_t *bfsmrk)
{
  idx_t ii, j, k=0, head, tail, nhits, tnhits, from, BFSDEPTH=5;

  from = where[i];

  /* Determine if the vertex is safe to move from a contiguity standpoint */
  for (tnhits=0, j=xadj[i]; j<xadj[i+1]; j++) {
    if (where[adjncy[j]] == from) {
      ASSERT(bfsmrk[adjncy[j]] == 0);
      ASSERT(bfslvl[adjncy[j]] == 0);
      bfsmrk[k=adjncy[j]] = 1;
      tnhits++;
    }
  }

  /* Easy cases */
  if (tnhits == 0)
    return 0;
  if (tnhits == 1) {
    bfsmrk[k] = 0;
    return 0;
  }

  ASSERT(bfslvl[i] == 0);
  bfslvl[i] = 1;

  bfsind[0] = k; /* That was the last one from the previous loop */
  bfslvl[k] = 1;
  bfsmrk[k] = 0;
  head = 0;
  tail = 1;

  /* Do a limited BFS traversal to see if you can get to all the other nodes */
  for (nhits=1; head<tail; ) {
    ii = bfsind[head++];
    for (j=xadj[ii]; j<xadj[ii+1]; j++) {
      if (where[k=adjncy[j]] == from) {
        if (bfsmrk[k]) {
          bfsmrk[k] = 0;
          if (++nhits == tnhits)
            break;
        }
        if (bfslvl[k] == 0 && bfslvl[ii] < BFSDEPTH) {
          bfsind[tail++] = k;
          bfslvl[k] = bfslvl[ii]+1;
        }
      }
    }
    if (nhits == tnhits)
      break;
  }

  /* Reset the various BFS related arrays */
  bfslvl[i] = 0;
  for (j=0; j<tail; j++)
    bfslvl[bfsind[j]] = 0;


  /* Reset the bfsmrk array for the next vertex when has not already being cleared */
  if (nhits < tnhits) {
    for (j=xadj[i]; j<xadj[i+1]; j++) 
      if (where[adjncy[j]] == from) 
        bfsmrk[adjncy[j]] = 0;
  }

  return (nhits != tnhits);
}


/*************************************************************************/
/*! 
 This function updates the edge and volume gains due to a vertex movement. 
 v from 'from' to 'to'.

 \param ctrl is the control structure.
 \param graph is the graph being partitioned.
 \param v is the vertex that is moving.
 \param from is the original partition of v.
 \param to is the new partition of v.
 \param queue is the priority queue. If the queue is NULL, no priority-queue
        related updates are performed. 
 \param vstatus is an array that marks the status of the vertex in terms
        of the priority queue. If queue is NULL, this parameter is ignored.
 \param r_nqupd is the number of vertices that have been inserted/removed
        from the queue. If queue is NULL, this parameter is ignored.
 \param updptr stores the index of each vertex in updind. If queue is NULL, 
        this parameter is ignored.
 \param updind is the list of vertices that have been inserted/removed from 
        the queue. If queue is NULL, this parameter is ignored.
 \param vmarker is of size nvtxs and is used internally as a temporary array. 
        On entry and return all of its entries are 0.
 \param pmarker is of sie nparts and is used internally as a temporary marking
        array. On entry and return all of its entries are -1.
 \param modind is an array of size nvtxs and is used to keep track of the 
        list of vertices whose gains need to be updated.
*/
/*************************************************************************/
void KWayVolUpdate(ctrl_t *ctrl, graph_t *graph, idx_t v, idx_t from, 
         idx_t to, ipq_t *queue, idx_t *vstatus, idx_t *r_nupd, idx_t *updptr, 
         idx_t *updind, idx_t bndtype, idx_t *vmarker, idx_t *pmarker, 
         idx_t *modind)
{
  idx_t i, ii, iii, j, jj, k, kk, l, u, nmod, other, me, myidx; 
  idx_t *xadj, *vsize, *adjncy, *where;
  vkrinfo_t *myrinfo, *orinfo;
  vnbr_t *mynbrs, *onbrs;

  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  vsize  = graph->vsize;
  where  = graph->where;

  myrinfo = graph->vkrinfo+v;
  mynbrs  = ctrl->vnbrpool + myrinfo->inbr;


  /*======================================================================
   * Remove the contributions on the gain made by 'v'. 
   *=====================================================================*/
  for (k=0; k<myrinfo->nnbrs; k++)
    pmarker[mynbrs[k].pid] = k;
  pmarker[from] = k;

  myidx = pmarker[to];  /* Keep track of the index in mynbrs of the 'to' domain */

  for (j=xadj[v]; j<xadj[v+1]; j++) {
    ii     = adjncy[j];
    other  = where[ii];
    orinfo = graph->vkrinfo+ii;
    onbrs  = ctrl->vnbrpool + orinfo->inbr;

    if (other == from) {
      for (k=0; k<orinfo->nnbrs; k++) {
        if (pmarker[onbrs[k].pid] == -1) 
          onbrs[k].gv += vsize[v];
      }
    }
    else {
      ASSERT(pmarker[other] != -1);

      if (mynbrs[pmarker[other]].ned > 1) {
        for (k=0; k<orinfo->nnbrs; k++) {
          if (pmarker[onbrs[k].pid] == -1) 
            onbrs[k].gv += vsize[v];
        }
      }
      else { /* There is only one connection */
        for (k=0; k<orinfo->nnbrs; k++) {
          if (pmarker[onbrs[k].pid] != -1) 
            onbrs[k].gv -= vsize[v];
        }
      }
    }
  }

  for (k=0; k<myrinfo->nnbrs; k++)
    pmarker[mynbrs[k].pid] = -1;
  pmarker[from] = -1;


  /*======================================================================
   * Update the id/ed of vertex 'v'
   *=====================================================================*/
  if (myidx == -1) {
    myidx = myrinfo->nnbrs++;
    ASSERT(myidx < xadj[v+1]-xadj[v]);
    mynbrs[myidx].ned = 0;
  }
  myrinfo->ned += myrinfo->nid-mynbrs[myidx].ned;
  SWAP(myrinfo->nid, mynbrs[myidx].ned, j);
  if (mynbrs[myidx].ned == 0) 
    mynbrs[myidx] = mynbrs[--myrinfo->nnbrs];
  else
    mynbrs[myidx].pid = from;


  /*======================================================================
   * Update the degrees of adjacent vertices and their volume gains
   *=====================================================================*/
  vmarker[v] = 1;
  modind[0]  = v;
  nmod       = 1;
  for (j=xadj[v]; j<xadj[v+1]; j++) {
    ii = adjncy[j];
    me = where[ii];

    if (!vmarker[ii]) {  /* The marking is done for boundary and max gv calculations */
      vmarker[ii] = 2;
      modind[nmod++] = ii;
    }

    myrinfo = graph->vkrinfo+ii;
    if (myrinfo->inbr == -1) 
      myrinfo->inbr = vnbrpoolGetNext(ctrl, xadj[ii+1]-xadj[ii]+1);
    mynbrs = ctrl->vnbrpool + myrinfo->inbr;

    if (me == from) {
      INC_DEC(myrinfo->ned, myrinfo->nid, 1);
    } 
    else if (me == to) {
      INC_DEC(myrinfo->nid, myrinfo->ned, 1);
    }

    /* Remove the edgeweight from the 'pid == from' entry of the vertex */
    if (me != from) {
      for (k=0; k<myrinfo->nnbrs; k++) {
        if (mynbrs[k].pid == from) {
          if (mynbrs[k].ned == 1) {
            mynbrs[k] = mynbrs[--myrinfo->nnbrs];
            vmarker[ii] = 1;  /* You do a complete .gv calculation */

            /* All vertices adjacent to 'ii' need to be updated */
            for (jj=xadj[ii]; jj<xadj[ii+1]; jj++) {
              u      = adjncy[jj];
              other  = where[u];
              orinfo = graph->vkrinfo+u;
              onbrs  = ctrl->vnbrpool + orinfo->inbr;

              for (kk=0; kk<orinfo->nnbrs; kk++) {
                if (onbrs[kk].pid == from) {
                  onbrs[kk].gv -= vsize[ii];
                  if (!vmarker[u]) { /* Need to update boundary etc */
                    vmarker[u]      = 2;
                    modind[nmod++] = u;
                  }
                  break;
                }
              }
            }
          }
          else {
            mynbrs[k].ned--;

            /* Update the gv due to single 'ii' connection to 'from' */
            if (mynbrs[k].ned == 1) {
              /* find the vertex 'u' that 'ii' was connected into 'from' */
              for (jj=xadj[ii]; jj<xadj[ii+1]; jj++) {
                u     = adjncy[jj];
                other = where[u];

                if (other == from) {
                  orinfo = graph->vkrinfo+u;
                  onbrs  = ctrl->vnbrpool + orinfo->inbr;

                  /* The following is correct because domains in common
                     between ii and u will lead to a reduction over the
                     previous gain, whereas domains only in u but not in
                     ii, will lead to no change as opposed to the earlier
                     increase */
                  for (kk=0; kk<orinfo->nnbrs; kk++) 
                    onbrs[kk].gv += vsize[ii];

                  if (!vmarker[u]) { /* Need to update boundary etc */
                    vmarker[u]     = 2;
                    modind[nmod++] = u;
                  }
                  break;  
                }
              }
            }
          }
          break; 
        }
      }
    }


    /* Add the edgeweight to the 'pid == to' entry of the vertex */
    if (me != to) {
      for (k=0; k<myrinfo->nnbrs; k++) {
        if (mynbrs[k].pid == to) {
          mynbrs[k].ned++;

          /* Update the gv due to non-single 'ii' connection to 'to' */
          if (mynbrs[k].ned == 2) {
            /* find the vertex 'u' that 'ii' was connected into 'to' */
            for (jj=xadj[ii]; jj<xadj[ii+1]; jj++) {
              u     = adjncy[jj];
              other = where[u];

              if (u != v && other == to) {
                orinfo = graph->vkrinfo+u;
                onbrs  = ctrl->vnbrpool + orinfo->inbr;
                for (kk=0; kk<orinfo->nnbrs; kk++) 
                  onbrs[kk].gv -= vsize[ii];

                if (!vmarker[u]) { /* Need to update boundary etc */
                  vmarker[u]      = 2;
                  modind[nmod++] = u;
                }
                break;  
              }
            }
          }
          break;
        }
      }

      if (k == myrinfo->nnbrs) {
        mynbrs[myrinfo->nnbrs].pid   = to;
        mynbrs[myrinfo->nnbrs++].ned = 1;
        vmarker[ii] = 1;  /* You do a complete .gv calculation */

        /* All vertices adjacent to 'ii' need to be updated */
        for (jj=xadj[ii]; jj<xadj[ii+1]; jj++) {
          u      = adjncy[jj];
          other  = where[u];
          orinfo = graph->vkrinfo+u;
          onbrs  = ctrl->vnbrpool + orinfo->inbr;

          for (kk=0; kk<orinfo->nnbrs; kk++) {
            if (onbrs[kk].pid == to) {
              onbrs[kk].gv += vsize[ii];
              if (!vmarker[u]) { /* Need to update boundary etc */
                vmarker[u] = 2;
                modind[nmod++] = u;
              }
              break;
            }
          }
        }
      }
    }

    ASSERT(myrinfo->nnbrs <= xadj[ii+1]-xadj[ii]);
  }


  /*======================================================================
   * Add the contributions on the volume gain due to 'v'
   *=====================================================================*/
  myrinfo = graph->vkrinfo+v;
  mynbrs  = ctrl->vnbrpool + myrinfo->inbr;
  for (k=0; k<myrinfo->nnbrs; k++)
    pmarker[mynbrs[k].pid] = k;
  pmarker[to] = k;

  for (j=xadj[v]; j<xadj[v+1]; j++) {
    ii     = adjncy[j];
    other  = where[ii];
    orinfo = graph->vkrinfo+ii;
    onbrs  = ctrl->vnbrpool + orinfo->inbr;

    if (other == to) {
      for (k=0; k<orinfo->nnbrs; k++) {
        if (pmarker[onbrs[k].pid] == -1) 
          onbrs[k].gv -= vsize[v];
      }
    }
    else {
      ASSERT(pmarker[other] != -1);

      if (mynbrs[pmarker[other]].ned > 1) {
        for (k=0; k<orinfo->nnbrs; k++) {
          if (pmarker[onbrs[k].pid] == -1) 
            onbrs[k].gv -= vsize[v];
        }
      }
      else { /* There is only one connection */
        for (k=0; k<orinfo->nnbrs; k++) {
          if (pmarker[onbrs[k].pid] != -1) 
            onbrs[k].gv += vsize[v];
        }
      }
    }
  }
  for (k=0; k<myrinfo->nnbrs; k++)
    pmarker[mynbrs[k].pid] = -1;
  pmarker[to] = -1;


  /*======================================================================
   * Recompute the volume information of the 'hard' nodes, and update the
   * max volume gain for all the modified vertices and the priority queue
   *=====================================================================*/
  for (iii=0; iii<nmod; iii++) {
    i  = modind[iii];
    me = where[i];

    myrinfo = graph->vkrinfo+i;
    mynbrs  = ctrl->vnbrpool + myrinfo->inbr;

    if (vmarker[i] == 1) {  /* Only complete gain updates go through */
      for (k=0; k<myrinfo->nnbrs; k++) 
        mynbrs[k].gv = 0;

      for (j=xadj[i]; j<xadj[i+1]; j++) {
        ii     = adjncy[j];
        other  = where[ii];
        orinfo = graph->vkrinfo+ii;
        onbrs  = ctrl->vnbrpool + orinfo->inbr;

        for (kk=0; kk<orinfo->nnbrs; kk++) 
          pmarker[onbrs[kk].pid] = kk;
        pmarker[other] = 1;

        if (me == other) {
          /* Find which domains 'i' is connected and 'ii' is not and update their gain */
          for (k=0; k<myrinfo->nnbrs; k++) {
            if (pmarker[mynbrs[k].pid] == -1)
              mynbrs[k].gv -= vsize[ii];
          }
        }
        else {
          ASSERT(pmarker[me] != -1);

          /* I'm the only connection of 'ii' in 'me' */
          if (onbrs[pmarker[me]].ned == 1) { 
            /* Increase the gains for all the common domains between 'i' and 'ii' */
            for (k=0; k<myrinfo->nnbrs; k++) {
              if (pmarker[mynbrs[k].pid] != -1) 
                mynbrs[k].gv += vsize[ii];
            }
          }
          else {
            /* Find which domains 'i' is connected and 'ii' is not and update their gain */
            for (k=0; k<myrinfo->nnbrs; k++) {
              if (pmarker[mynbrs[k].pid] == -1) 
                mynbrs[k].gv -= vsize[ii];
            }
          }
        }

        for (kk=0; kk<orinfo->nnbrs; kk++) 
          pmarker[onbrs[kk].pid] = -1;
        pmarker[other] = -1;
  
      }
    }

    /* Compute the overall gv for that node */
    myrinfo->gv = IDX_MIN;
    for (k=0; k<myrinfo->nnbrs; k++) {
      if (mynbrs[k].gv > myrinfo->gv)
        myrinfo->gv = mynbrs[k].gv;
    }

    /* Add the xtra gain due to id == 0 */
    if (myrinfo->ned > 0 && myrinfo->nid == 0)
      myrinfo->gv += vsize[i];


    /*======================================================================
     * Maintain a consistent boundary
     *=====================================================================*/
    if (bndtype == BNDTYPE_REFINE) {
      if (myrinfo->gv >= 0 && graph->bndptr[i] == -1)
        BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

      if (myrinfo->gv < 0 && graph->bndptr[i] != -1)
        BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
    }
    else {
      if (myrinfo->ned > 0 && graph->bndptr[i] == -1)
        BNDInsert(graph->nbnd, graph->bndind, graph->bndptr, i);

      if (myrinfo->ned == 0 && graph->bndptr[i] != -1)
        BNDDelete(graph->nbnd, graph->bndind, graph->bndptr, i);
    }


    /*======================================================================
     * Update the priority queue appropriately (if allowed)
     *=====================================================================*/
    if (queue != NULL) {
      if (vstatus[i] != VPQSTATUS_EXTRACTED) {
        if (graph->bndptr[i] != -1) { /* In-boundary vertex */
          if (vstatus[i] == VPQSTATUS_PRESENT) {
            ipqUpdate(queue, i, myrinfo->gv);
          }
          else {
            ipqInsert(queue, i, myrinfo->gv);
            vstatus[i] = VPQSTATUS_PRESENT;
            ListInsert(*r_nupd, updind, updptr, i);
          }
        }
        else { /* Off-boundary vertex */
          if (vstatus[i] == VPQSTATUS_PRESENT) {
            ipqDelete(queue, i);
            vstatus[i] = VPQSTATUS_NOTPRESENT;
            ListDelete(*r_nupd, updind, updptr, i);
          }
        }
      }
    }
  
    vmarker[i] = 0;
  }
}

