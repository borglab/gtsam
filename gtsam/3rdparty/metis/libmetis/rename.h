/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * rename.h
 *
 * This file contains header files
 *
 * Started 10/2/97
 * George
 *
 * $Id: rename.h 13933 2013-03-29 22:20:46Z karypis $
 *
 */


#ifndef _LIBMETIS_RENAME_H_
#define _LIBMETIS_RENAME_H_


/* balance.c */
#define Balance2Way			libmetis__Balance2Way
#define Bnd2WayBalance			libmetis__Bnd2WayBalance
#define General2WayBalance		libmetis__General2WayBalance
#define McGeneral2WayBalance            libmetis__McGeneral2WayBalance

/* bucketsort.c */
#define BucketSortKeysInc		libmetis__BucketSortKeysInc

/* checkgraph.c */
#define CheckGraph                      libmetis__CheckGraph
#define CheckInputGraphWeights          libmetis__CheckInputGraphWeights
#define FixGraph                        libmetis__FixGraph

/* coarsen.c */
#define CoarsenGraph			libmetis__CoarsenGraph
#define Match_RM                        libmetis__Match_RM
#define Match_SHEM                      libmetis__Match_SHEM
#define Match_2Hop                      libmetis__Match_2Hop
#define Match_2HopAny                   libmetis__Match_2HopAny
#define Match_2HopAll                   libmetis__Match_2HopAll
#define PrintCGraphStats                libmetis__PrintCGraphStats
#define CreateCoarseGraph		libmetis__CreateCoarseGraph
#define CreateCoarseGraphNoMask		libmetis__CreateCoarseGraphNoMask
#define CreateCoarseGraphPerm		libmetis__CreateCoarseGraphPerm
#define SetupCoarseGraph		libmetis__SetupCoarseGraph
#define ReAdjustMemory			libmetis__ReAdjustMemory

/* compress.c */
#define CompressGraph			libmetis__CompressGraph
#define PruneGraph			libmetis__PruneGraph

/* contig.c */
#define FindPartitionInducedComponents  libmetis__FindPartitionInducedComponents   
#define IsConnected                     libmetis__IsConnected
#define IsConnectedSubdomain            libmetis__IsConnectedSubdomain
#define FindSepInducedComponents        libmetis__FindSepInducedComponents
#define EliminateComponents             libmetis__EliminateComponents
#define MoveGroupContigForCut           libmetis__MoveGroupContigForCut
#define MoveGroupContigForVol           libmetis__MoveGroupContigForVol

/* debug.c */
#define ComputeCut			libmetis__ComputeCut
#define ComputeVolume			libmetis__ComputeVolume
#define ComputeMaxCut			libmetis__ComputeMaxCut
#define CheckBnd			libmetis__CheckBnd
#define CheckBnd2			libmetis__CheckBnd2
#define CheckNodeBnd			libmetis__CheckNodeBnd
#define CheckRInfo			libmetis__CheckRInfo
#define CheckNodePartitionParams	libmetis__CheckNodePartitionParams
#define IsSeparable			libmetis__IsSeparable
#define CheckKWayVolPartitionParams     libmetis__CheckKWayVolPartitionParams

/* fm.c */
#define FM_2WayRefine                   libmetis__FM_2WayRefine
#define FM_2WayCutRefine                libmetis__FM_2WayCutRefine
#define FM_Mc2WayCutRefine              libmetis__FM_Mc2WayCutRefine
#define SelectQueue                     libmetis__SelectQueue
#define Print2WayRefineStats            libmetis__Print2WayRefineStats

/* fortran.c */
#define Change2CNumbering		libmetis__Change2CNumbering
#define Change2FNumbering		libmetis__Change2FNumbering
#define Change2FNumbering2		libmetis__Change2FNumbering2
#define Change2FNumberingOrder		libmetis__Change2FNumberingOrder
#define ChangeMesh2CNumbering		libmetis__ChangeMesh2CNumbering
#define ChangeMesh2FNumbering		libmetis__ChangeMesh2FNumbering
#define ChangeMesh2FNumbering2		libmetis__ChangeMesh2FNumbering2

/* graph.c */
#define SetupGraph			libmetis__SetupGraph
#define SetupGraph_adjrsum              libmetis__SetupGraph_adjrsum
#define SetupGraph_tvwgt                libmetis__SetupGraph_tvwgt
#define SetupGraph_label                libmetis__SetupGraph_label
#define SetupSplitGraph                 libmetis__SetupSplitGraph
#define CreateGraph                     libmetis__CreateGraph
#define InitGraph                       libmetis__InitGraph
#define FreeRData                       libmetis__FreeRData
#define FreeGraph                       libmetis__FreeGraph

/* initpart.c */
#define Init2WayPartition		libmetis__Init2WayPartition
#define InitSeparator			libmetis__InitSeparator
#define RandomBisection			libmetis__RandomBisection
#define GrowBisection			libmetis__GrowBisection
#define McRandomBisection               libmetis__McRandomBisection
#define McGrowBisection                 libmetis__McGrowBisection
#define GrowBisectionNode		libmetis__GrowBisectionNode

/* kmetis.c */
#define MlevelKWayPartitioning		libmetis__MlevelKWayPartitioning
#define InitKWayPartitioning            libmetis__InitKWayPartitioning

/* kwayfm.c */
#define Greedy_KWayOptimize		libmetis__Greedy_KWayOptimize
#define Greedy_KWayCutOptimize		libmetis__Greedy_KWayCutOptimize
#define Greedy_KWayVolOptimize          libmetis__Greedy_KWayVolOptimize
#define Greedy_McKWayCutOptimize        libmetis__Greedy_McKWayCutOptimize
#define Greedy_McKWayVolOptimize        libmetis__Greedy_McKWayVolOptimize
#define IsArticulationNode              libmetis__IsArticulationNode
#define KWayVolUpdate                   libmetis__KWayVolUpdate

/* kwayrefine.c */
#define RefineKWay			libmetis__RefineKWay
#define AllocateKWayPartitionMemory	libmetis__AllocateKWayPartitionMemory
#define ComputeKWayPartitionParams	libmetis__ComputeKWayPartitionParams
#define ProjectKWayPartition		libmetis__ProjectKWayPartition
#define ComputeKWayBoundary		libmetis__ComputeKWayBoundary
#define ComputeKWayVolGains             libmetis__ComputeKWayVolGains
#define IsBalanced			libmetis__IsBalanced

/* mcutil */
#define rvecle                          libmetis__rvecle
#define rvecge                          libmetis__rvecge
#define rvecsumle                       libmetis__rvecsumle
#define rvecmaxdiff                     libmetis__rvecmaxdiff
#define ivecle                          libmetis__ivecle
#define ivecge                          libmetis__ivecge
#define ivecaxpylez                     libmetis__ivecaxpylez
#define ivecaxpygez                     libmetis__ivecaxpygez
#define BetterVBalance                  libmetis__BetterVBalance
#define BetterBalance2Way               libmetis__BetterBalance2Way
#define BetterBalanceKWay               libmetis__BetterBalanceKWay
#define ComputeLoadImbalance            libmetis__ComputeLoadImbalance
#define ComputeLoadImbalanceDiff        libmetis__ComputeLoadImbalanceDiff
#define ComputeLoadImbalanceDiffVec     libmetis__ComputeLoadImbalanceDiffVec
#define ComputeLoadImbalanceVec         libmetis__ComputeLoadImbalanceVec

/* mesh.c */
#define CreateGraphDual                 libmetis__CreateGraphDual
#define FindCommonElements              libmetis__FindCommonElements
#define CreateGraphNodal                libmetis__CreateGraphNodal
#define FindCommonNodes                 libmetis__FindCommonNodes
#define CreateMesh                      libmetis__CreateMesh
#define InitMesh                        libmetis__InitMesh
#define FreeMesh                        libmetis__FreeMesh

/* meshpart.c */
#define InduceRowPartFromColumnPart     libmetis__InduceRowPartFromColumnPart

/* minconn.c */
#define ComputeSubDomainGraph           libmetis__ComputeSubDomainGraph
#define UpdateEdgeSubDomainGraph        libmetis__UpdateEdgeSubDomainGraph
#define PrintSubDomainGraph             libmetis__PrintSubDomainGraph
#define EliminateSubDomainEdges         libmetis__EliminateSubDomainEdges
#define MoveGroupMinConnForCut          libmetis__MoveGroupMinConnForCut
#define MoveGroupMinConnForVol          libmetis__MoveGroupMinConnForVol

/* mincover.c */
#define MinCover			libmetis__MinCover
#define MinCover_Augment		libmetis__MinCover_Augment
#define MinCover_Decompose		libmetis__MinCover_Decompose
#define MinCover_ColDFS			libmetis__MinCover_ColDFS
#define MinCover_RowDFS			libmetis__MinCover_RowDFS

/* mmd.c */
#define genmmd				libmetis__genmmd
#define mmdelm				libmetis__mmdelm
#define mmdint				libmetis__mmdint
#define mmdnum				libmetis__mmdnum
#define mmdupd				libmetis__mmdupd


/* ometis.c */
#define MlevelNestedDissection		libmetis__MlevelNestedDissection
#define MlevelNestedDissectionCC	libmetis__MlevelNestedDissectionCC
#define MlevelNodeBisectionMultiple	libmetis__MlevelNodeBisectionMultiple
#define MlevelNodeBisectionL2		libmetis__MlevelNodeBisectionL2
#define MlevelNodeBisectionL1		libmetis__MlevelNodeBisectionL1
#define SplitGraphOrder			libmetis__SplitGraphOrder
#define SplitGraphOrderCC		libmetis__SplitGraphOrderCC
#define MMDOrder			libmetis__MMDOrder

/* options.c */
#define SetupCtrl                       libmetis__SetupCtrl
#define SetupKWayBalMultipliers         libmetis__SetupKWayBalMultipliers
#define Setup2WayBalMultipliers         libmetis__Setup2WayBalMultipliers
#define PrintCtrl                       libmetis__PrintCtrl
#define FreeCtrl                        libmetis__FreeCtrl
#define CheckParams                     libmetis__CheckParams

/* parmetis.c */
#define MlevelNestedDissectionP		libmetis__MlevelNestedDissectionP
#define FM_2WayNodeRefine1SidedP        libmetis__FM_2WayNodeRefine1SidedP
#define FM_2WayNodeRefine2SidedP        libmetis__FM_2WayNodeRefine2SidedP

/* pmetis.c */
#define MlevelRecursiveBisection	libmetis__MlevelRecursiveBisection
#define MultilevelBisect		libmetis__MultilevelBisect
#define SplitGraphPart			libmetis__SplitGraphPart

/* refine.c */
#define Refine2Way			libmetis__Refine2Way
#define Allocate2WayPartitionMemory	libmetis__Allocate2WayPartitionMemory
#define Compute2WayPartitionParams	libmetis__Compute2WayPartitionParams
#define Project2WayPartition		libmetis__Project2WayPartition

/* separator.c */
#define ConstructSeparator		libmetis__ConstructSeparator
#define ConstructMinCoverSeparator	libmetis__ConstructMinCoverSeparator

/* sfm.c */
#define FM_2WayNodeRefine2Sided         libmetis__FM_2WayNodeRefine2Sided 
#define FM_2WayNodeRefine1Sided         libmetis__FM_2WayNodeRefine1Sided
#define FM_2WayNodeBalance              libmetis__FM_2WayNodeBalance

/* srefine.c */
#define Refine2WayNode			libmetis__Refine2WayNode
#define Allocate2WayNodePartitionMemory	libmetis__Allocate2WayNodePartitionMemory
#define Compute2WayNodePartitionParams	libmetis__Compute2WayNodePartitionParams
#define Project2WayNodePartition	libmetis__Project2WayNodePartition

/* stat.c */
#define ComputePartitionInfoBipartite   libmetis__ComputePartitionInfoBipartite
#define ComputePartitionBalance		libmetis__ComputePartitionBalance
#define ComputeElementBalance		libmetis__ComputeElementBalance

/* timing.c */
#define InitTimers			libmetis__InitTimers
#define PrintTimers			libmetis__PrintTimers

/* util.c */
#define iargmax_strd                    libmetis__iargmax_strd 
#define iargmax_nrm                     libmetis__iargmax_nrm
#define iargmax2_nrm                    libmetis__iargmax2_nrm
#define rargmax2                        libmetis__rargmax2
#define InitRandom                      libmetis__InitRandom
#define metis_rcode                     libmetis__metis_rcode

/* wspace.c */
#define AllocateWorkSpace               libmetis__AllocateWorkSpace                  
#define AllocateRefinementWorkSpace     libmetis__AllocateRefinementWorkSpace
#define FreeWorkSpace                   libmetis__FreeWorkSpace
#define wspacemalloc                    libmetis__wspacemalloc
#define wspacepush                      libmetis__wspacepush
#define wspacepop                       libmetis__wspacepop
#define iwspacemalloc                   libmetis__iwspacemalloc
#define rwspacemalloc                   libmetis__rwspacemalloc
#define ikvwspacemalloc                 libmetis__ikvwspacemalloc
#define cnbrpoolReset                   libmetis__cnbrpoolReset
#define cnbrpoolGetNext                 libmetis__cnbrpoolGetNext
#define vnbrpoolReset                   libmetis__vnbrpoolReset
#define vnbrpoolGetNext                 libmetis__vnbrpoolGetNext

#endif


