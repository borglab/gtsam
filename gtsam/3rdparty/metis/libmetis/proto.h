/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * proto.h
 *
 * This file contains header files
 *
 * Started 10/19/95
 * George
 *
 * $Id: proto.h 13933 2013-03-29 22:20:46Z karypis $
 *
 */

#ifndef _LIBMETIS_PROTO_H_
#define _LIBMETIS_PROTO_H_

/* auxapi.c */

/* balance.c */
void Balance2Way(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
void Bnd2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
void General2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
void McGeneral2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);


/* bucketsort.c */
void BucketSortKeysInc(ctrl_t *ctrl, idx_t n, idx_t max, idx_t *keys,
         idx_t *tperm, idx_t *perm);


/* checkgraph.c */
int CheckGraph(graph_t *graph, int numflag, int verbose);
int CheckInputGraphWeights(idx_t nvtxs, idx_t ncon, idx_t *xadj, idx_t *adjncy,
        idx_t *vwgt, idx_t *vsize, idx_t *adjwgt);
graph_t *FixGraph(graph_t *graph);


/* coarsen.c */
graph_t *CoarsenGraph(ctrl_t *ctrl, graph_t *graph);
graph_t *CoarsenGraphNlevels(ctrl_t *ctrl, graph_t *graph, idx_t nlevels);
idx_t Match_RM(ctrl_t *ctrl, graph_t *graph);
idx_t Match_SHEM(ctrl_t *ctrl, graph_t *graph);
idx_t Match_2Hop(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t nunmatched);
idx_t Match_2HopAny(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree);
idx_t Match_2HopAll(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree);
void PrintCGraphStats(ctrl_t *ctrl, graph_t *graph);
void CreateCoarseGraph(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match);
void CreateCoarseGraphNoMask(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match);
void CreateCoarseGraphPerm(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match, idx_t *perm);
graph_t *SetupCoarseGraph(graph_t *graph, idx_t cnvtxs, idx_t dovsize);
void ReAdjustMemory(ctrl_t *ctrl, graph_t *graph, graph_t *cgraph);



/* compress.c */
graph_t *CompressGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t *xadj, idx_t *adjncy, 
             idx_t *vwgt, idx_t *cptr, idx_t *cind);
graph_t *PruneGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t *xadj, idx_t *adjncy, 
             idx_t *vwgt, idx_t *iperm, real_t factor);


/* contig.c */
idx_t FindPartitionInducedComponents(graph_t *graph, idx_t *where, 
          idx_t *cptr, idx_t *cind);
void ComputeBFSOrdering(ctrl_t *ctrl, graph_t *graph, idx_t *bfsperm);
idx_t IsConnected(graph_t *graph, idx_t report);
idx_t IsConnectedSubdomain(ctrl_t *, graph_t *, idx_t, idx_t);
idx_t FindSepInducedComponents(ctrl_t *, graph_t *, idx_t *, idx_t *);
void EliminateComponents(ctrl_t *ctrl, graph_t *graph);
void MoveGroupContigForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid, 
         idx_t *ptr, idx_t *ind);
void MoveGroupContigForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid,
         idx_t *ptr, idx_t *ind, idx_t *vmarker, idx_t *pmarker,
         idx_t *modind);


/* debug.c */
idx_t ComputeCut(graph_t *graph, idx_t *where);
idx_t ComputeVolume(graph_t *, idx_t *);
idx_t ComputeMaxCut(graph_t *graph, idx_t nparts, idx_t *where);
idx_t CheckBnd(graph_t *);
idx_t CheckBnd2(graph_t *);
idx_t CheckNodeBnd(graph_t *, idx_t);
idx_t CheckRInfo(ctrl_t *ctrl, ckrinfo_t *rinfo);
idx_t CheckNodePartitionParams(graph_t *);
idx_t IsSeparable(graph_t *);
void CheckKWayVolPartitionParams(ctrl_t *ctrl, graph_t *graph);


/* fm.c */
void FM_2WayRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
void FM_2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
void FM_Mc2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
void SelectQueue(graph_t *graph, real_t *pijbm, real_t *ubfactors, rpq_t **queues, 
         idx_t *from, idx_t *cnum);
void Print2WayRefineStats(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         real_t deltabal, idx_t mincutorder);


/* fortran.c */
void Change2CNumbering(idx_t, idx_t *, idx_t *);
void Change2FNumbering(idx_t, idx_t *, idx_t *, idx_t *);
void Change2FNumbering2(idx_t, idx_t *, idx_t *);
void Change2FNumberingOrder(idx_t, idx_t *, idx_t *, idx_t *, idx_t *);
void ChangeMesh2CNumbering(idx_t n, idx_t *ptr, idx_t *ind);
void ChangeMesh2FNumbering(idx_t n, idx_t *ptr, idx_t *ind, idx_t nvtxs,
         idx_t *xadj, idx_t *adjncy);
void ChangeMesh2FNumbering2(idx_t ne, idx_t nn, idx_t *ptr, idx_t *ind,
         idx_t *epart, idx_t *npart);


/* graph.c */
graph_t *SetupGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t ncon, idx_t *xadj, 
             idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt);
void SetupGraph_tvwgt(graph_t *graph);
void SetupGraph_label(graph_t *graph);
graph_t *SetupSplitGraph(graph_t *graph, idx_t snvtxs, idx_t snedges);
graph_t *CreateGraph(void);
void InitGraph(graph_t *graph);
void FreeRData(graph_t *graph);
void FreeGraph(graph_t **graph);


/* initpart.c */
void Init2WayPartition(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void InitSeparator(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
void RandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void GrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void McRandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void McGrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void GrowBisectionNode(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
void GrowBisectionNode2(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);


/* kmetis.c */
idx_t MlevelKWayPartitioning(ctrl_t *ctrl, graph_t *graph, idx_t *part);
void InitKWayPartitioning(ctrl_t *ctrl, graph_t *graph);


/* kwayfm.c */
void Greedy_KWayOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
void Greedy_KWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
void Greedy_KWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
void Greedy_McKWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
void Greedy_McKWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
idx_t IsArticulationNode(idx_t i, idx_t *xadj, idx_t *adjncy, idx_t *where,
          idx_t *bfslvl, idx_t *bfsind, idx_t *bfsmrk);
void KWayVolUpdate(ctrl_t *ctrl, graph_t *graph, idx_t v, idx_t from,
         idx_t to, ipq_t *queue, idx_t *vstatus, idx_t *r_nupd, idx_t *updptr,
         idx_t *updind, idx_t bndtype, idx_t *vmarker, idx_t *pmarker,
         idx_t *modind);


/* kwayrefine.c */
void RefineKWay(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph);
void AllocateKWayPartitionMemory(ctrl_t *ctrl, graph_t *graph);
void ComputeKWayPartitionParams(ctrl_t *ctrl, graph_t *graph);
void ProjectKWayPartition(ctrl_t *ctrl, graph_t *graph);
void ComputeKWayBoundary(ctrl_t *ctrl, graph_t *graph, idx_t bndtype);
void ComputeKWayVolGains(ctrl_t *ctrl, graph_t *graph);
int IsBalanced(ctrl_t *ctrl, graph_t *graph, real_t ffactor);


/* mcutil.c */
int rvecle(idx_t n, real_t *x, real_t *y);
int rvecge(idx_t n, real_t *x, real_t *y);
int rvecsumle(idx_t n, real_t *x1, real_t *x2, real_t *y);
real_t rvecmaxdiff(idx_t n, real_t *x, real_t *y);
int ivecle(idx_t n, idx_t *x, idx_t *z);
int ivecge(idx_t n, idx_t *x, idx_t *z);
int ivecaxpylez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z);
int ivecaxpygez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z);
int BetterVBalance(idx_t ncon, real_t *itvwgt, idx_t *v_vwgt, idx_t *u1_vwgt,
            idx_t *u2_vwgt);
int BetterBalance2Way(idx_t n, real_t *x, real_t *y);
int BetterBalanceKWay(idx_t ncon, idx_t *vwgt, real_t *itvwgt, idx_t a1,
        idx_t *pt1, real_t *bm1, idx_t a2, idx_t *pt2, real_t *bm2);
real_t ComputeLoadImbalance(graph_t *graph, idx_t nparts, real_t *pijbm);
real_t ComputeLoadImbalanceDiff(graph_t *graph, idx_t nparts, real_t *pijbm, 
           real_t *ubvec);
real_t ComputeLoadImbalanceDiffVec(graph_t *graph, idx_t nparts, real_t *pijbm, 
         real_t *ubfactors, real_t *diffvec);
void ComputeLoadImbalanceVec(graph_t *graph, idx_t nparts, real_t *pijbm,
             real_t *lbvec);


/* mesh.c */
void CreateGraphDual(idx_t ne, idx_t nn, idx_t *eptr, idx_t *eind, idx_t ncommon,
          idx_t **r_xadj, idx_t **r_adjncy);
idx_t FindCommonElements(idx_t qid, idx_t elen, idx_t *eind, idx_t *nptr,
          idx_t *nind, idx_t *eptr, idx_t ncommon, idx_t *marker, idx_t *nbrs);
void CreateGraphNodal(idx_t ne, idx_t nn, idx_t *eptr, idx_t *eind, idx_t **r_xadj, 
          idx_t **r_adjncy);
idx_t FindCommonNodes(idx_t qid, idx_t nelmnts, idx_t *elmntids, idx_t *eptr,
          idx_t *eind, idx_t *marker, idx_t *nbrs);
mesh_t *CreateMesh(void);
void InitMesh(mesh_t *mesh);  
void FreeMesh(mesh_t **mesh);


/* meshpart.c */
void InduceRowPartFromColumnPart(idx_t nrows, idx_t *rowptr, idx_t *rowind,
         idx_t *rpart, idx_t *cpart, idx_t nparts, real_t *tpwgts);


/* minconn.c */
void ComputeSubDomainGraph(ctrl_t *ctrl, graph_t *graph);
void UpdateEdgeSubDomainGraph(ctrl_t *ctrl, idx_t u, idx_t v, idx_t ewgt, 
         idx_t *r_maxndoms);
void PrintSubDomainGraph(graph_t *graph, idx_t nparts, idx_t *where);
void EliminateSubDomainEdges(ctrl_t *ctrl, graph_t *graph);
void MoveGroupMinConnForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind);
void MoveGroupMinConnForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind, idx_t *vmarker, idx_t *pmarker, idx_t *modind);


/* mincover.o */
void MinCover(idx_t *, idx_t *, idx_t, idx_t, idx_t *, idx_t *);
idx_t MinCover_Augment(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t *, idx_t);
void MinCover_Decompose(idx_t *, idx_t *, idx_t, idx_t, idx_t *, idx_t *, idx_t *);
void MinCover_ColDFS(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t);
void MinCover_RowDFS(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t);


/* mmd.c */
void genmmd(idx_t, idx_t *, idx_t *, idx_t *, idx_t *, idx_t , idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t *);
void mmdelm(idx_t, idx_t *xadj, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t);
idx_t mmdint(idx_t, idx_t *xadj, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *);
void mmdnum(idx_t, idx_t *, idx_t *, idx_t *);
void mmdupd(idx_t, idx_t, idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t *tag);


/* ometis.c */
void MlevelNestedDissection(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx);
void MlevelNestedDissectionCC(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx);
void MlevelNodeBisectionMultiple(ctrl_t *ctrl, graph_t *graph);
void MlevelNodeBisectionL2(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
void MlevelNodeBisectionL1(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
void SplitGraphOrder(ctrl_t *ctrl, graph_t *graph, graph_t **r_lgraph, 
         graph_t **r_rgraph);
graph_t **SplitGraphOrderCC(ctrl_t *ctrl, graph_t *graph, idx_t ncmps,
              idx_t *cptr, idx_t *cind);
void MMDOrder(ctrl_t *ctrl, graph_t *graph, idx_t *order, idx_t lastvtx);


/* options.c */
ctrl_t *SetupCtrl(moptype_et optype, idx_t *options, idx_t ncon, idx_t nparts, 
            real_t *tpwgts, real_t *ubvec);
void SetupKWayBalMultipliers(ctrl_t *ctrl, graph_t *graph);
void Setup2WayBalMultipliers(ctrl_t *ctrl, graph_t *graph, real_t *tpwgts);
void PrintCtrl(ctrl_t *ctrl);
int CheckParams(ctrl_t *ctrl);
void FreeCtrl(ctrl_t **r_ctrl);


/* parmetis.c */
void MlevelNestedDissectionP(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx, idx_t npes, idx_t cpos, idx_t *sizes);
void FM_2WayNodeRefine1SidedP(ctrl_t *ctrl, graph_t *graph, idx_t *hmarker, 
         real_t ubfactor, idx_t npasses);
void FM_2WayNodeRefine2SidedP(ctrl_t *ctrl, graph_t *graph, idx_t *hmarker, 
         real_t ubfactor, idx_t npasses);


/* pmetis.c */
idx_t MlevelRecursiveBisection(ctrl_t *ctrl, graph_t *graph, idx_t nparts, 
          idx_t *part, real_t *tpwgts, idx_t fpart);
idx_t MultilevelBisect(ctrl_t *ctrl, graph_t *graph, real_t *tpwgts);
void SplitGraphPart(ctrl_t *ctrl, graph_t *graph, graph_t **r_lgraph, graph_t **r_rgraph);


/* refine.c */
void Refine2Way(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph, real_t *rtpwgts);
void Allocate2WayPartitionMemory(ctrl_t *ctrl, graph_t *graph);
void Compute2WayPartitionParams(ctrl_t *ctrl, graph_t *graph);
void Project2WayPartition(ctrl_t *ctrl, graph_t *graph);


/* separator.c */
void ConstructSeparator(ctrl_t *ctrl, graph_t *graph);
void ConstructMinCoverSeparator(ctrl_t *ctrl, graph_t *graph);


/* sfm.c */
void FM_2WayNodeRefine2Sided(ctrl_t *ctrl, graph_t *graph, idx_t niter);
void FM_2WayNodeRefine1Sided(ctrl_t *ctrl, graph_t *graph, idx_t niter);
void FM_2WayNodeBalance(ctrl_t *ctrl, graph_t *graph);


/* srefine.c */
void Refine2WayNode(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph);
void Allocate2WayNodePartitionMemory(ctrl_t *ctrl, graph_t *graph);
void Compute2WayNodePartitionParams(ctrl_t *ctrl, graph_t *graph);
void Project2WayNodePartition(ctrl_t *ctrl, graph_t *graph);


/* stat.c */
void ComputePartitionInfoBipartite(graph_t *, idx_t, idx_t *);
void ComputePartitionBalance(graph_t *, idx_t, idx_t *, real_t *);
real_t ComputeElementBalance(idx_t, idx_t, idx_t *);


/* timing.c */
void InitTimers(ctrl_t *);
void PrintTimers(ctrl_t *);

/* util.c */
idx_t iargmax_strd(size_t, idx_t *, idx_t);
idx_t iargmax_nrm(size_t n, idx_t *x, real_t *y);
idx_t iargmax2_nrm(size_t n, idx_t *x, real_t *y);
idx_t rargmax2(size_t, real_t *);
void InitRandom(idx_t);
int metis_rcode(int sigrval);



/* wspace.c */
void AllocateWorkSpace(ctrl_t *ctrl, graph_t *graph);
void AllocateRefinementWorkSpace(ctrl_t *ctrl, idx_t nbrpoolsize);
void FreeWorkSpace(ctrl_t *ctrl);
void *wspacemalloc(ctrl_t *ctrl, size_t nbytes);
void wspacepush(ctrl_t *ctrl);
void wspacepop(ctrl_t *ctrl);
idx_t *iwspacemalloc(ctrl_t *, idx_t);
real_t *rwspacemalloc(ctrl_t *, idx_t);
ikv_t *ikvwspacemalloc(ctrl_t *, idx_t);
void cnbrpoolReset(ctrl_t *ctrl);
idx_t cnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs);
void vnbrpoolReset(ctrl_t *ctrl);
idx_t vnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs);


#endif
