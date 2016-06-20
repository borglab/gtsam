/*!
\file  
\brief Functions for computing matchings during graph coarsening

\date Started 7/23/97
\author George  
\author Copyright 1997-2011, Regents of the University of Minnesota 
\version\verbatim $Id: coarsen.c 13936 2013-03-30 03:59:09Z karypis $ \endverbatim
*/


#include "metislib.h"

#define UNMATCHEDFOR2HOP  0.10  /* The fraction of unmatched vertices that triggers 2-hop */
                                  

/*************************************************************************/
/*! This function takes a graph and creates a sequence of coarser graphs.
    It implements the coarsening phase of the multilevel paradigm. 
 */
/*************************************************************************/
graph_t *CoarsenGraph(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, eqewgts, level=0;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->CoarsenTmr));

  /* determine if the weights on the edges are all the same */
  for (eqewgts=1, i=1; i<graph->nedges; i++) {
    if (graph->adjwgt[0] != graph->adjwgt[i]) {
      eqewgts = 0;
      break;
    }
  }

  /* set the maximum allowed coarsest vertex weight */
  for (i=0; i<graph->ncon; i++)
    ctrl->maxvwgt[i] = 1.5*graph->tvwgt[i]/ctrl->CoarsenTo;

  do {
    IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));

    /* allocate memory for cmap, if it has not already been done due to
       multiple cuts */
    if (graph->cmap == NULL)
      graph->cmap = imalloc(graph->nvtxs, "CoarsenGraph: graph->cmap");

    /* determine which matching scheme you will use */
    switch (ctrl->ctype) {
      case METIS_CTYPE_RM:
        Match_RM(ctrl, graph);
        break;
      case METIS_CTYPE_SHEM:
        if (eqewgts || graph->nedges == 0)
          Match_RM(ctrl, graph);
        else
          Match_SHEM(ctrl, graph);
        break;
      default:
        gk_errexit(SIGERR, "Unknown ctype: %d\n", ctrl->ctype);
    }

    graph = graph->coarser;
    eqewgts = 0;
    level++;

    ASSERT(CheckGraph(graph, 0, 1));

  } while (graph->nvtxs > ctrl->CoarsenTo && 
           graph->nvtxs < COARSEN_FRACTION*graph->finer->nvtxs && 
           graph->nedges > graph->nvtxs/2);

  IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->CoarsenTmr));

  return graph;
}


/*************************************************************************/
/*! This function takes a graph and creates a sequence of nlevels coarser 
    graphs, where nlevels is an input parameter.
 */
/*************************************************************************/
graph_t *CoarsenGraphNlevels(ctrl_t *ctrl, graph_t *graph, idx_t nlevels)
{
  idx_t i, eqewgts, level;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->CoarsenTmr));

  /* determine if the weights on the edges are all the same */
  for (eqewgts=1, i=1; i<graph->nedges; i++) {
    if (graph->adjwgt[0] != graph->adjwgt[i]) {
      eqewgts = 0;
      break;
    }
  }

  /* set the maximum allowed coarsest vertex weight */
  for (i=0; i<graph->ncon; i++)
    ctrl->maxvwgt[i] = 1.5*graph->tvwgt[i]/ctrl->CoarsenTo;

  for (level=0; level<nlevels; level++) {
    IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));

    /* allocate memory for cmap, if it has not already been done due to
       multiple cuts */
    if (graph->cmap == NULL)
      graph->cmap = imalloc(graph->nvtxs, "CoarsenGraph: graph->cmap");

    /* determine which matching scheme you will use */
    switch (ctrl->ctype) {
      case METIS_CTYPE_RM:
        Match_RM(ctrl, graph);
        break;
      case METIS_CTYPE_SHEM:
        if (eqewgts || graph->nedges == 0)
          Match_RM(ctrl, graph);
        else
          Match_SHEM(ctrl, graph);
        break;
      default:
        gk_errexit(SIGERR, "Unknown ctype: %d\n", ctrl->ctype);
    }

    graph = graph->coarser;
    eqewgts = 0;

    ASSERT(CheckGraph(graph, 0, 1));

    if (graph->nvtxs < ctrl->CoarsenTo || 
        graph->nvtxs > COARSEN_FRACTION*graph->finer->nvtxs || 
        graph->nedges < graph->nvtxs/2)
      break; 
  } 

  IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, PrintCGraphStats(ctrl, graph));
  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->CoarsenTmr));

  return graph;
}


/*************************************************************************/
/*! This function finds a matching by randomly selecting one of the 
    unmatched adjacent vertices. 
 */
/**************************************************************************/
idx_t Match_RM(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, pi, ii, j, jj, jjinc, k, nvtxs, ncon, cnvtxs, maxidx, last_unmatched;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *maxvwgt;
  idx_t *match, *cmap, *perm;
  size_t nunmatched=0;

  WCOREPUSH;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  cmap   = graph->cmap;

  maxvwgt  = ctrl->maxvwgt;

  match = iset(nvtxs, UNMATCHED, iwspacemalloc(ctrl, nvtxs));
  perm  = iwspacemalloc(ctrl, nvtxs);

  irandArrayPermute(nvtxs, perm, nvtxs/8, 1);

  for (cnvtxs=0, last_unmatched=0, pi=0; pi<nvtxs; pi++) {
    i = perm[pi];

    if (match[i] == UNMATCHED) {  /* Unmatched */
      maxidx = i;

      if ((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt+i*ncon, maxvwgt))) {
        /* Deal with island vertices. Find a non-island and match it with. 
           The matching ignores ctrl->maxvwgt requirements */
        if (xadj[i] == xadj[i+1]) {
          last_unmatched = gk_max(pi, last_unmatched)+1;
          for (; last_unmatched<nvtxs; last_unmatched++) {
            j = perm[last_unmatched];
            if (match[j] == UNMATCHED) {
              maxidx = j;
              break;
            }
          }
        }
        else {
          /* Find a random matching, subject to maxvwgt constraints */
          if (ncon == 1) {
            /* single constraint version */
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              k = adjncy[j];
              if (match[k] == UNMATCHED && vwgt[i]+vwgt[k] <= maxvwgt[0]) {
                maxidx = k;
                break;
              }
            }

            /* If it did not match, record for a 2-hop matching. */
            if (maxidx == i && 3*vwgt[i] < maxvwgt[0]) {
              nunmatched++;
              maxidx = UNMATCHED;
            }
          }
          else {
            /* multi-constraint version */
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              k = adjncy[j];
              if (match[k] == UNMATCHED && 
                  ivecaxpylez(ncon, 1, vwgt+i*ncon, vwgt+k*ncon, maxvwgt)) {
                maxidx = k;
                break;
              }
            }

            /* If it did not match, record for a 2-hop matching. */
            if (maxidx == i && ivecaxpylez(ncon, 2, vwgt+i*ncon, vwgt+i*ncon, maxvwgt)) {
              nunmatched++;
              maxidx = UNMATCHED;
            }
          }
        }
      }

      if (maxidx != UNMATCHED) {
        cmap[i]  = cmap[maxidx] = cnvtxs++;
        match[i] = maxidx;
        match[maxidx] = i;
      }
    }
  }

  //printf("nunmatched: %zu\n", nunmatched);

  /* see if a 2-hop matching is required/allowed */
  if (!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP*nvtxs) 
    cnvtxs = Match_2Hop(ctrl, graph, perm, match, cnvtxs, nunmatched);


  /* match the final unmatched vertices with themselves and reorder the vertices 
     of the coarse graph for memory-friendly contraction */
  for (cnvtxs=0, i=0; i<nvtxs; i++) {
    if (match[i] == UNMATCHED) {
      match[i] = i;
      cmap[i]  = cnvtxs++;
    }
    else {
      if (i <= match[i]) 
        cmap[i] = cmap[match[i]] = cnvtxs++;
    }
  }

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

  CreateCoarseGraph(ctrl, graph, cnvtxs, match);

  WCOREPOP;

  return cnvtxs;
}


/**************************************************************************/
/*! This function finds a matching using the HEM heuristic. The vertices 
    are visited based on increasing degree to ensure that all vertices are 
    given a chance to match with something. 
 */
/**************************************************************************/
idx_t Match_SHEM(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i, pi, ii, j, jj, jjinc, k, nvtxs, ncon, cnvtxs, maxidx, maxwgt, 
        last_unmatched, avgdegree;
  idx_t *xadj, *vwgt, *adjncy, *adjwgt, *maxvwgt;
  idx_t *match, *cmap, *degrees, *perm, *tperm;
  size_t nunmatched=0;

  WCOREPUSH;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->MatchTmr));

  nvtxs  = graph->nvtxs;
  ncon   = graph->ncon;
  xadj   = graph->xadj;
  vwgt   = graph->vwgt;
  adjncy = graph->adjncy;
  adjwgt = graph->adjwgt;
  cmap   = graph->cmap;

  maxvwgt  = ctrl->maxvwgt;

  match   = iset(nvtxs, UNMATCHED, iwspacemalloc(ctrl, nvtxs));
  perm    = iwspacemalloc(ctrl, nvtxs);
  tperm   = iwspacemalloc(ctrl, nvtxs);
  degrees = iwspacemalloc(ctrl, nvtxs);

  irandArrayPermute(nvtxs, tperm, nvtxs/8, 1);

  avgdegree = 0.7*(xadj[nvtxs]/nvtxs);
  for (i=0; i<nvtxs; i++) 
    degrees[i] = (xadj[i+1]-xadj[i] > avgdegree ? avgdegree : xadj[i+1]-xadj[i]);
  BucketSortKeysInc(ctrl, nvtxs, avgdegree, degrees, tperm, perm);

  for (cnvtxs=0, last_unmatched=0, pi=0; pi<nvtxs; pi++) {
    i = perm[pi];

    if (match[i] == UNMATCHED) {  /* Unmatched */
      maxidx = i;
      maxwgt = -1;

      if ((ncon == 1 ? vwgt[i] < maxvwgt[0] : ivecle(ncon, vwgt+i*ncon, maxvwgt))) {
        /* Deal with island vertices. Find a non-island and match it with. 
           The matching ignores ctrl->maxvwgt requirements */
        if (xadj[i] == xadj[i+1]) { 
          last_unmatched = gk_max(pi, last_unmatched)+1;
          for (; last_unmatched<nvtxs; last_unmatched++) {
            j = perm[last_unmatched];
            if (match[j] == UNMATCHED) {
              maxidx = j;
              break;
            }
          }
        }
        else {
          /* Find a heavy-edge matching, subject to maxvwgt constraints */
          if (ncon == 1) {
            /* single constraint version */
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              k = adjncy[j];
              if (match[k] == UNMATCHED && 
                  maxwgt < adjwgt[j] && vwgt[i]+vwgt[k] <= maxvwgt[0]) {
                maxidx = k;
                maxwgt = adjwgt[j];
              }
            }

            /* If it did not match, record for a 2-hop matching. */
            if (maxidx == i && 3*vwgt[i] < maxvwgt[0]) {
              nunmatched++;
              maxidx = UNMATCHED;
            }
          }
          else {
            /* multi-constraint version */
            for (j=xadj[i]; j<xadj[i+1]; j++) {
              k = adjncy[j];
              if (match[k] == UNMATCHED && 
                  ivecaxpylez(ncon, 1, vwgt+i*ncon, vwgt+k*ncon, maxvwgt) &&
                  (maxwgt < adjwgt[j] || 
                   (maxwgt == adjwgt[j] && 
                    BetterVBalance(ncon, graph->invtvwgt, vwgt+i*ncon, 
                        vwgt+maxidx*ncon, vwgt+k*ncon)))) {
                maxidx = k;
                maxwgt = adjwgt[j];
              }
            }

            /* If it did not match, record for a 2-hop matching. */
            if (maxidx == i && ivecaxpylez(ncon, 2, vwgt+i*ncon, vwgt+i*ncon, maxvwgt)) {
              nunmatched++;
              maxidx = UNMATCHED;
            }
          }
        }
      }

      if (maxidx != UNMATCHED) {
        cmap[i]  = cmap[maxidx] = cnvtxs++;
        match[i] = maxidx;
        match[maxidx] = i;
      }
    }
  }

  //printf("nunmatched: %zu\n", nunmatched);

  /* see if a 2-hop matching is required/allowed */
  if (!ctrl->no2hop && nunmatched > UNMATCHEDFOR2HOP*nvtxs) 
    cnvtxs = Match_2Hop(ctrl, graph, perm, match, cnvtxs, nunmatched);


  /* match the final unmatched vertices with themselves and reorder the vertices 
     of the coarse graph for memory-friendly contraction */
  for (cnvtxs=0, i=0; i<nvtxs; i++) {
    if (match[i] == UNMATCHED) {
      match[i] = i;
      cmap[i] = cnvtxs++;
    }
    else {
      if (i <= match[i]) 
        cmap[i] = cmap[match[i]] = cnvtxs++;
    }
  }

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->MatchTmr));

  CreateCoarseGraph(ctrl, graph, cnvtxs, match);

  WCOREPOP;

  return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices using a 2-hop matching 
    that involves vertices that are two hops away from each other. */
/**************************************************************************/
idx_t Match_2Hop(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match, 
          idx_t cnvtxs, size_t nunmatched)
{

  cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 2);
  cnvtxs = Match_2HopAll(ctrl, graph, perm, match, cnvtxs, &nunmatched, 64);
  if (nunmatched > 1.5*UNMATCHEDFOR2HOP*graph->nvtxs) 
    cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, 3);
  if (nunmatched > 2.0*UNMATCHEDFOR2HOP*graph->nvtxs) 
    cnvtxs = Match_2HopAny(ctrl, graph, perm, match, cnvtxs, &nunmatched, graph->nvtxs);

  return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices whose degree is less than
    maxdegree using a 2-hop matching that involves vertices that are two 
    hops away from each other. 
    The requirement of the 2-hop matching is a simple non-empty overlap
    between the adjancency lists of the vertices. */
/**************************************************************************/
idx_t Match_2HopAny(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match, 
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree)
{
  idx_t i, pi, ii, j, jj, k, nvtxs;
  idx_t *xadj, *adjncy, *colptr, *rowind;
  idx_t *cmap;
  size_t nunmatched;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux3Tmr));

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  cmap   = graph->cmap;

  nunmatched = *r_nunmatched;

  /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("IN: nunmatched: %zu\t", * nunmatched)); */

  /* create the inverted index */
  WCOREPUSH;
  colptr = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs+1));
  for (i=0; i<nvtxs; i++) {
    if (match[i] == UNMATCHED && xadj[i+1]-xadj[i] < maxdegree) {
      for (j=xadj[i]; j<xadj[i+1]; j++)
        colptr[adjncy[j]]++;
    }
  }
  MAKECSR(i, nvtxs, colptr);

  rowind = iwspacemalloc(ctrl, colptr[nvtxs]);
  for (pi=0; pi<nvtxs; pi++) {
    i = perm[pi];
    if (match[i] == UNMATCHED && xadj[i+1]-xadj[i] < maxdegree) {
      for (j=xadj[i]; j<xadj[i+1]; j++)
        rowind[colptr[adjncy[j]]++] = i;
    }
  }
  SHIFTCSR(i, nvtxs, colptr);

  /* compute matchings by going down the inverted index */
  for (pi=0; pi<nvtxs; pi++) {
    i = perm[pi];
    if (colptr[i+1]-colptr[i] < 2)
      continue;

    for (jj=colptr[i+1], j=colptr[i]; j<jj; j++) {
      if (match[rowind[j]] == UNMATCHED) {
        for (jj--; jj>j; jj--) {
          if (match[rowind[jj]] == UNMATCHED) {
            cmap[rowind[j]] = cmap[rowind[jj]] = cnvtxs++;
            match[rowind[j]]  = rowind[jj];
            match[rowind[jj]] = rowind[j];
            nunmatched -= 2;
            break;
          }
        }
      }
    }
  }
  WCOREPOP;

  /* IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("OUT: nunmatched: %zu\n", nunmatched)); */

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux3Tmr));

  *r_nunmatched = nunmatched;
  return cnvtxs;
}


/*************************************************************************/
/*! This function matches the unmatched vertices whose degree is less than
    maxdegree using a 2-hop matching that involves vertices that are two 
    hops away from each other. 
    The requirement of the 2-hop matching is that of identical adjacency
    lists.
 */
/**************************************************************************/
idx_t Match_2HopAll(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match, 
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree)
{
  idx_t i, pi, pk, ii, j, jj, k, nvtxs, mask, idegree;
  idx_t *xadj, *adjncy;
  idx_t *cmap, *mark;
  ikv_t *keys;
  size_t nunmatched, ncand;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->Aux3Tmr));

  nvtxs  = graph->nvtxs;
  xadj   = graph->xadj;
  adjncy = graph->adjncy;
  cmap   = graph->cmap;

  nunmatched = *r_nunmatched;
  mask = IDX_MAX/maxdegree;

  /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("IN: nunmatched: %zu\t", nunmatched)); */

  WCOREPUSH;

  /* collapse vertices with identical adjancency lists */
  keys = ikvwspacemalloc(ctrl, nunmatched);
  for (ncand=0, pi=0; pi<nvtxs; pi++) {
    i = perm[pi];
    idegree = xadj[i+1]-xadj[i];
    if (match[i] == UNMATCHED && idegree > 1 && idegree < maxdegree) {
      for (k=0, j=xadj[i]; j<xadj[i+1]; j++) 
        k += adjncy[j]%mask;
      keys[ncand].val = i;
      keys[ncand].key = (k%mask)*maxdegree + idegree;
      ncand++;
    }
  }
  ikvsorti(ncand, keys);

  mark = iset(nvtxs, 0, iwspacemalloc(ctrl, nvtxs));
  for (pi=0; pi<ncand; pi++) {
    i = keys[pi].val;
    if (match[i] != UNMATCHED)
      continue;

    for (j=xadj[i]; j<xadj[i+1]; j++)
      mark[adjncy[j]] = i;

    for (pk=pi+1; pk<ncand; pk++) {
      k = keys[pk].val;
      if (match[k] != UNMATCHED)
        continue;

      if (keys[pi].key != keys[pk].key)
        break;
      if (xadj[i+1]-xadj[i] != xadj[k+1]-xadj[k])
        break;

      for (jj=xadj[k]; jj<xadj[k+1]; jj++) {
        if (mark[adjncy[jj]] != i)
          break;
      }
      if (jj == xadj[k+1]) {
        cmap[i] = cmap[k] = cnvtxs++;
        match[i] = k;
        match[k] = i;
        nunmatched -= 2;
        break;
      }
    }
  }
  WCOREPOP;

  /*IFSET(ctrl->dbglvl, METIS_DBG_COARSEN, printf("OUT: ncand: %zu, nunmatched: %zu\n", ncand, nunmatched)); */

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->Aux3Tmr));

  *r_nunmatched = nunmatched;
  return cnvtxs;
}


/*************************************************************************/
/*! This function prints various stats for each graph during coarsening 
 */
/*************************************************************************/
void PrintCGraphStats(ctrl_t *ctrl, graph_t *graph)
{
  idx_t i;

  printf("%10"PRIDX" %10"PRIDX" %10"PRIDX" [%"PRIDX"] [", 
      graph->nvtxs, graph->nedges, isum(graph->nedges, graph->adjwgt, 1), ctrl->CoarsenTo);

  for (i=0; i<graph->ncon; i++)
    printf(" %8"PRIDX":%8"PRIDX, ctrl->maxvwgt[i], graph->tvwgt[i]);
  printf(" ]\n");
}


/*************************************************************************/
/*! This function creates the coarser graph. It uses a simple hash-table 
    for identifying the adjacent vertices that get collapsed to the same
    node. The hash-table can have conflicts, which are handled via a
    linear scan. 
 */
/*************************************************************************/
void CreateCoarseGraph(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match)
{
  idx_t j, jj, k, kk, l, m, istart, iend, nvtxs, nedges, ncon, cnedges, 
        v, u, mask, dovsize;
  idx_t *xadj, *vwgt, *vsize, *adjncy, *adjwgt;
  idx_t *cmap, *htable;
  idx_t *cxadj, *cvwgt, *cvsize, *cadjncy, *cadjwgt;
  graph_t *cgraph;

  dovsize = (ctrl->objtype == METIS_OBJTYPE_VOL ? 1 : 0);

  /* Check if the mask-version of the code is a good choice */
  mask = HTLENGTH;
  if (cnvtxs < 2*mask || graph->nedges/graph->nvtxs > mask/20) { 
    CreateCoarseGraphNoMask(ctrl, graph, cnvtxs, match);
    return;
  }

  nvtxs = graph->nvtxs;
  xadj  = graph->xadj;
  for (v=0; v<nvtxs; v++) {
    if (xadj[v+1]-xadj[v] > (mask>>3)) {
      CreateCoarseGraphNoMask(ctrl, graph, cnvtxs, match);
      return;
    }
  }


  WCOREPUSH;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ContractTmr));

  ncon    = graph->ncon;
  vwgt    = graph->vwgt;
  vsize   = graph->vsize;
  adjncy  = graph->adjncy;
  adjwgt  = graph->adjwgt;
  cmap    = graph->cmap;

  /* Initialize the coarser graph */
  cgraph   = SetupCoarseGraph(graph, cnvtxs, dovsize);
  cxadj    = cgraph->xadj;
  cvwgt    = cgraph->vwgt;
  cvsize   = cgraph->vsize;
  cadjncy  = cgraph->adjncy;
  cadjwgt  = cgraph->adjwgt;

  htable = iset(gk_min(cnvtxs+1, mask+1), -1, iwspacemalloc(ctrl, mask+1)); 

  cxadj[0] = cnvtxs = cnedges = 0;
  for (v=0; v<nvtxs; v++) {
    if ((u = match[v]) < v)
      continue;

    ASSERT(cmap[v] == cnvtxs);
    ASSERT(cmap[match[v]] == cnvtxs);

    if (ncon == 1)
      cvwgt[cnvtxs] = vwgt[v];
    else
      icopy(ncon, vwgt+v*ncon, cvwgt+cnvtxs*ncon);

    if (dovsize)
      cvsize[cnvtxs] = vsize[v];

    nedges = 0;

    istart = xadj[v];
    iend   = xadj[v+1];
    for (j=istart; j<iend; j++) {
      k  = cmap[adjncy[j]];
      kk = k&mask;
      if ((m = htable[kk]) == -1) {
        cadjncy[nedges] = k;
        cadjwgt[nedges] = adjwgt[j];
        htable[kk] = nedges++;
      }
      else if (cadjncy[m] == k) {
        cadjwgt[m] += adjwgt[j];
      }
      else {
        for (jj=0; jj<nedges; jj++) {
          if (cadjncy[jj] == k) {
            cadjwgt[jj] += adjwgt[j];
            break;
          }
        }
        if (jj == nedges) {
          cadjncy[nedges]   = k;
          cadjwgt[nedges++] = adjwgt[j];
        }
      }
    }

    if (v != u) { 
      if (ncon == 1)
        cvwgt[cnvtxs] += vwgt[u];
      else
        iaxpy(ncon, 1, vwgt+u*ncon, 1, cvwgt+cnvtxs*ncon, 1);

      if (dovsize)
        cvsize[cnvtxs] += vsize[u];

      istart = xadj[u];
      iend   = xadj[u+1];
      for (j=istart; j<iend; j++) {
        k  = cmap[adjncy[j]];
        kk = k&mask;
        if ((m = htable[kk]) == -1) {
          cadjncy[nedges] = k;
          cadjwgt[nedges] = adjwgt[j];
          htable[kk]      = nedges++;
        }
        else if (cadjncy[m] == k) {
          cadjwgt[m] += adjwgt[j];
        }
        else {
          for (jj=0; jj<nedges; jj++) {
            if (cadjncy[jj] == k) {
              cadjwgt[jj] += adjwgt[j];
              break;
            }
          }
          if (jj == nedges) {
            cadjncy[nedges]   = k;
            cadjwgt[nedges++] = adjwgt[j];
          }
        }
      }

      /* Remove the contracted adjacency weight */
      jj = htable[cnvtxs&mask];
      if (jj >= 0 && cadjncy[jj] != cnvtxs) {
        for (jj=0; jj<nedges; jj++) {
          if (cadjncy[jj] == cnvtxs) 
            break;
        }
      }
      /* This 2nd check is needed for non-adjacent matchings */
      if (jj >= 0 && jj < nedges && cadjncy[jj] == cnvtxs) { 
        cadjncy[jj] = cadjncy[--nedges];
        cadjwgt[jj] = cadjwgt[nedges];
      }
    }

    /* Zero out the htable */
    for (j=0; j<nedges; j++)
      htable[cadjncy[j]&mask] = -1;  
    htable[cnvtxs&mask] = -1;

    cnedges         += nedges;
    cxadj[++cnvtxs]  = cnedges;
    cadjncy         += nedges;
    cadjwgt         += nedges;
  }

  cgraph->nedges = cnedges;

  for (j=0; j<ncon; j++) {
    cgraph->tvwgt[j]    = isum(cgraph->nvtxs, cgraph->vwgt+j, ncon);
    cgraph->invtvwgt[j] = 1.0/(cgraph->tvwgt[j] > 0 ? cgraph->tvwgt[j] : 1);
  }


  ReAdjustMemory(ctrl, graph, cgraph);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ContractTmr));

  WCOREPOP;
}


/*************************************************************************/
/*! This function creates the coarser graph. It uses a full-size array
    (htable) for identifying the adjacent vertices that get collapsed to 
    the same node.  
 */
/*************************************************************************/
void CreateCoarseGraphNoMask(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match)
{
  idx_t j, k, m, istart, iend, nvtxs, nedges, ncon, cnedges, v, u, dovsize;
  idx_t *xadj, *vwgt, *vsize, *adjncy, *adjwgt;
  idx_t *cmap, *htable;
  idx_t *cxadj, *cvwgt, *cvsize, *cadjncy, *cadjwgt;
  graph_t *cgraph;

  WCOREPUSH;

  dovsize = (ctrl->objtype == METIS_OBJTYPE_VOL ? 1 : 0);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ContractTmr));

  nvtxs   = graph->nvtxs;
  ncon    = graph->ncon;
  xadj    = graph->xadj;
  vwgt    = graph->vwgt;
  vsize   = graph->vsize;
  adjncy  = graph->adjncy;
  adjwgt  = graph->adjwgt;
  cmap    = graph->cmap;


  /* Initialize the coarser graph */
  cgraph = SetupCoarseGraph(graph, cnvtxs, dovsize);
  cxadj    = cgraph->xadj;
  cvwgt    = cgraph->vwgt;
  cvsize   = cgraph->vsize;
  cadjncy  = cgraph->adjncy;
  cadjwgt  = cgraph->adjwgt;

  htable = iset(cnvtxs, -1, iwspacemalloc(ctrl, cnvtxs));

  cxadj[0] = cnvtxs = cnedges = 0;
  for (v=0; v<nvtxs; v++) {
    if ((u = match[v]) < v)
      continue;

    ASSERT(cmap[v] == cnvtxs);
    ASSERT(cmap[match[v]] == cnvtxs);

    if (ncon == 1)
      cvwgt[cnvtxs] = vwgt[v];
    else
      icopy(ncon, vwgt+v*ncon, cvwgt+cnvtxs*ncon);

    if (dovsize)
      cvsize[cnvtxs] = vsize[v];

    nedges = 0;

    istart = xadj[v];
    iend   = xadj[v+1];
    for (j=istart; j<iend; j++) {
      k = cmap[adjncy[j]];
      if ((m = htable[k]) == -1) {
        cadjncy[nedges] = k;
        cadjwgt[nedges] = adjwgt[j];
        htable[k] = nedges++;
      }
      else {
        cadjwgt[m] += adjwgt[j];
      }
    }

    if (v != u) { 
      if (ncon == 1)
        cvwgt[cnvtxs] += vwgt[u];
      else
        iaxpy(ncon, 1, vwgt+u*ncon, 1, cvwgt+cnvtxs*ncon, 1);

      if (dovsize)
        cvsize[cnvtxs] += vsize[u];

      istart = xadj[u];
      iend   = xadj[u+1];
      for (j=istart; j<iend; j++) {
        k = cmap[adjncy[j]];
        if ((m = htable[k]) == -1) {
          cadjncy[nedges] = k;
          cadjwgt[nedges] = adjwgt[j];
          htable[k] = nedges++;
        }
        else {
          cadjwgt[m] += adjwgt[j];
        }
      }

      /* Remove the contracted adjacency weight */
      if ((j = htable[cnvtxs]) != -1) {
        ASSERT(cadjncy[j] == cnvtxs);
        cadjncy[j]        = cadjncy[--nedges];
        cadjwgt[j]        = cadjwgt[nedges];
        htable[cnvtxs] = -1;
      }
    }

    /* Zero out the htable */
    for (j=0; j<nedges; j++)
      htable[cadjncy[j]] = -1;  

    cnedges         += nedges;
    cxadj[++cnvtxs]  = cnedges;
    cadjncy         += nedges;
    cadjwgt         += nedges;
  }

  cgraph->nedges = cnedges;

  for (j=0; j<ncon; j++) {
    cgraph->tvwgt[j]    = isum(cgraph->nvtxs, cgraph->vwgt+j, ncon);
    cgraph->invtvwgt[j] = 1.0/(cgraph->tvwgt[j] > 0 ? cgraph->tvwgt[j] : 1);
  }

  ReAdjustMemory(ctrl, graph, cgraph);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ContractTmr));

  WCOREPOP;
}


/*************************************************************************/
/*! This function creates the coarser graph. It uses a simple hash-table 
    for identifying the adjacent vertices that get collapsed to the same
    node. The hash-table can have conflicts, which are handled via a
    linear scan. It relies on the perm[] array to visit the vertices in
    increasing cnvtxs order.
 */
/*************************************************************************/
void CreateCoarseGraphPerm(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match, idx_t *perm)
{
  idx_t i, j, jj, k, kk, l, m, istart, iend, nvtxs, nedges, ncon, cnedges, 
        v, u, mask, dovsize;
  idx_t *xadj, *vwgt, *vsize, *adjncy, *adjwgt;
  idx_t *cmap, *htable;
  idx_t *cxadj, *cvwgt, *cvsize, *cadjncy, *cadjwgt;
  graph_t *cgraph;

  WCOREPUSH;

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_startcputimer(ctrl->ContractTmr));

  dovsize = (ctrl->objtype == METIS_OBJTYPE_VOL ? 1 : 0);

  mask = HTLENGTH;

  nvtxs   = graph->nvtxs;
  ncon    = graph->ncon;
  xadj    = graph->xadj;
  vwgt    = graph->vwgt;
  vsize   = graph->vsize;
  adjncy  = graph->adjncy;
  adjwgt  = graph->adjwgt;
  cmap    = graph->cmap;

  /* Initialize the coarser graph */
  cgraph   = SetupCoarseGraph(graph, cnvtxs, dovsize);
  cxadj    = cgraph->xadj;
  cvwgt    = cgraph->vwgt;
  cvsize   = cgraph->vsize;
  cadjncy  = cgraph->adjncy;
  cadjwgt  = cgraph->adjwgt;

  htable = iset(mask+1, -1, iwspacemalloc(ctrl, mask+1)); 

  cxadj[0] = cnvtxs = cnedges = 0;
  for (i=0; i<nvtxs; i++) {
    v = perm[i];
    if (cmap[v] != cnvtxs) 
      continue;

    u = match[v];
    if (ncon == 1)
      cvwgt[cnvtxs] = vwgt[v];
    else
      icopy(ncon, vwgt+v*ncon, cvwgt+cnvtxs*ncon);

    if (dovsize)
      cvsize[cnvtxs] = vsize[v];

    nedges = 0;

    istart = xadj[v];
    iend = xadj[v+1];
    for (j=istart; j<iend; j++) {
      k  = cmap[adjncy[j]];
      kk = k&mask;
      if ((m = htable[kk]) == -1) {
        cadjncy[nedges] = k;
        cadjwgt[nedges] = adjwgt[j];
        htable[kk] = nedges++;
      }
      else if (cadjncy[m] == k) {
        cadjwgt[m] += adjwgt[j];
      }
      else {
        for (jj=0; jj<nedges; jj++) {
          if (cadjncy[jj] == k) {
            cadjwgt[jj] += adjwgt[j];
            break;
          }
        }
        if (jj == nedges) {
          cadjncy[nedges] = k;
          cadjwgt[nedges++] = adjwgt[j];
        }
      }
    }

    if (v != u) { 
      if (ncon == 1)
        cvwgt[cnvtxs] += vwgt[u];
      else
        iaxpy(ncon, 1, vwgt+u*ncon, 1, cvwgt+cnvtxs*ncon, 1);

      if (dovsize)
        cvsize[cnvtxs] += vsize[u];

      istart = xadj[u];
      iend = xadj[u+1];
      for (j=istart; j<iend; j++) {
        k  = cmap[adjncy[j]];
        kk = k&mask;
        if ((m = htable[kk]) == -1) {
          cadjncy[nedges] = k;
          cadjwgt[nedges] = adjwgt[j];
          htable[kk] = nedges++;
        }
        else if (cadjncy[m] == k) {
          cadjwgt[m] += adjwgt[j];
        }
        else {
          for (jj=0; jj<nedges; jj++) {
            if (cadjncy[jj] == k) {
              cadjwgt[jj] += adjwgt[j];
              break;
            }
          }
          if (jj == nedges) {
            cadjncy[nedges] = k;
            cadjwgt[nedges++] = adjwgt[j];
          }
        }
      }

      /* Remove the contracted adjacency weight */
      jj = htable[cnvtxs&mask];
      if (jj >= 0 && cadjncy[jj] != cnvtxs) {
        for (jj=0; jj<nedges; jj++) {
          if (cadjncy[jj] == cnvtxs) 
            break;
        }
      }
      if (jj >= 0 && cadjncy[jj] == cnvtxs) { /* This 2nd check is needed for non-adjacent matchings */
        cadjncy[jj] = cadjncy[--nedges];
        cadjwgt[jj] = cadjwgt[nedges];
      }
    }

    for (j=0; j<nedges; j++)
      htable[cadjncy[j]&mask] = -1;  /* Zero out the htable */
    htable[cnvtxs&mask] = -1;

    cnedges += nedges;
    cxadj[++cnvtxs] = cnedges;
    cadjncy += nedges;
    cadjwgt += nedges;
  }

  cgraph->nedges = cnedges;

  for (i=0; i<ncon; i++) {
    cgraph->tvwgt[i]    = isum(cgraph->nvtxs, cgraph->vwgt+i, ncon);
    cgraph->invtvwgt[i] = 1.0/(cgraph->tvwgt[i] > 0 ? cgraph->tvwgt[i] : 1);
  }


  ReAdjustMemory(ctrl, graph, cgraph);

  IFSET(ctrl->dbglvl, METIS_DBG_TIME, gk_stopcputimer(ctrl->ContractTmr));

  WCOREPOP;
}


/*************************************************************************/
/*! Setup the various arrays for the coarse graph 
 */
/*************************************************************************/
graph_t *SetupCoarseGraph(graph_t *graph, idx_t cnvtxs, idx_t dovsize)
{
  graph_t *cgraph;

  cgraph = CreateGraph();

  cgraph->nvtxs = cnvtxs;
  cgraph->ncon  = graph->ncon;

  cgraph->finer  = graph;
  graph->coarser = cgraph;


  /* Allocate memory for the coarser graph */
  cgraph->xadj     = imalloc(cnvtxs+1, "SetupCoarseGraph: xadj");
  cgraph->adjncy   = imalloc(graph->nedges,   "SetupCoarseGraph: adjncy");
  cgraph->adjwgt   = imalloc(graph->nedges,   "SetupCoarseGraph: adjwgt");
  cgraph->vwgt     = imalloc(cgraph->ncon*cnvtxs, "SetupCoarseGraph: vwgt");
  cgraph->tvwgt    = imalloc(cgraph->ncon, "SetupCoarseGraph: tvwgt");
  cgraph->invtvwgt = rmalloc(cgraph->ncon, "SetupCoarseGraph: invtvwgt");

  if (dovsize)
    cgraph->vsize = imalloc(cnvtxs,   "SetupCoarseGraph: vsize");

  return cgraph;
}


/*************************************************************************/
/*! This function re-adjusts the amount of memory that was allocated if
    it will lead to significant savings 
 */
/*************************************************************************/
void ReAdjustMemory(ctrl_t *ctrl, graph_t *graph, graph_t *cgraph) 
{
  if (cgraph->nedges > 10000 && cgraph->nedges < 0.9*graph->nedges) {
    cgraph->adjncy = irealloc(cgraph->adjncy, cgraph->nedges, "ReAdjustMemory: adjncy");
    cgraph->adjwgt = irealloc(cgraph->adjwgt, cgraph->nedges, "ReAdjustMemory: adjwgt");
  }
}
