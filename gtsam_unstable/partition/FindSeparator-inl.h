/*
 * FindSeparator-inl.h
 *
 *   Created on: Nov 23, 2010
 *   Updated: Feb 20. 2014
 *       Author: nikai
 *       Author: Andrew Melim
 *  Description: find the separator of bisectioning for a given graph
 */

#pragma once

#include <stdexcept>
#include <iostream>
#include <vector>
#include <optional>
#include <boost/shared_array.hpp>

#include <gtsam/base/timing.h>

#include "FindSeparator.h"

#include <metis.h>

extern "C" {
#include <metislib.h>
}



namespace gtsam { namespace partition {

  typedef boost::shared_array<idx_t> sharedInts;

  /* ************************************************************************* */
  /**
   * Return the size of the separator and the partiion indices {part}
   * Part [j] is 0, 1, or 2, depending on
   * whether node j is in the left part of the graph, the right part, or the
   * separator, respectively
   */
  std::pair<int, sharedInts> separatorMetis(idx_t n, const sharedInts& xadj,
    const sharedInts& adjncy, const sharedInts& adjwgt, bool verbose) {

    // control parameters
    std::vector<idx_t> vwgt;        // the weights of the vertices
    idx_t options[METIS_NOPTIONS];
    METIS_SetDefaultOptions(options);  // use defaults
    idx_t sepsize;                      // the size of the separator, output
    sharedInts part_(new idx_t[n]);      // the partition of each vertex, output

    // set uniform weights on the vertices
    vwgt.assign(n, 1);

    // TODO: Fix at later time
    //boost::timer::cpu_timer TOTALTmr;
    if (verbose) {
      printf("**********************************************************************\n");
      printf("Graph Information ---------------------------------------------------\n");
      printf("  #Vertices: %d, #Edges: %u\n", n, *(xadj.get()+n) / 2);
      printf("\nND Partitioning... -------------------------------------------\n");
      //TOTALTmr.start()
    }

    // call metis parition routine
    METIS_ComputeVertexSeparator(&n, xadj.get(), adjncy.get(),
           &vwgt[0], options, &sepsize, part_.get());

    if (verbose) {
      //boost::cpu_times const elapsed_times(timer.elapsed());
      //printf("\nTiming Information --------------------------------------------------\n");
      //printf("  Total:        \t\t %7.3f\n", elapsed_times);
      printf("  Sep size:        \t\t %d\n", sepsize);
      printf("**********************************************************************\n");
    }

    return std::make_pair(sepsize, part_);
  }

  /* ************************************************************************* */
  void modefied_EdgeComputeSeparator(idx_t *nvtxs, idx_t *xadj, idx_t *adjncy, idx_t *vwgt,
      idx_t *adjwgt, idx_t *options, idx_t *edgecut, idx_t *part)
  {
    idx_t i, ncon;
    graph_t *graph;
    real_t *tpwgts2;
    ctrl_t *ctrl;
    ctrl = SetupCtrl(METIS_OP_OMETIS, options, 1, 3, nullptr, nullptr);
    ctrl->iptype = METIS_IPTYPE_GROW;
    //if () == nullptr)
    //  return METIS_ERROR_INPUT;

    InitRandom(ctrl->seed);

    graph = SetupGraph(ctrl, *nvtxs, 1, xadj, adjncy, vwgt, nullptr, nullptr);

    AllocateWorkSpace(ctrl, graph);

    ncon = graph->ncon;
    ctrl->ncuts = 1;

    /* determine the weights of the two partitions as a function of the weight of the
       target partition weights */

    tpwgts2 = rwspacemalloc(ctrl, 2*ncon);
    for (i=0; i<ncon; i++) {
      tpwgts2[i]      = rsum((2>>1), ctrl->tpwgts+i, ncon);
      tpwgts2[ncon+i] = 1.0 - tpwgts2[i];
    }
    /* perform the bisection */
    *edgecut = MultilevelBisect(ctrl, graph, tpwgts2);

    //  ConstructMinCoverSeparator(&ctrl, &graph, 1.05);
  //  *edgecut = graph->mincut;
  //  *sepsize = graph.pwgts[2];
    icopy(*nvtxs, graph->where, part);
    std::cout << "Finished bisection:" << *edgecut << std::endl;
    FreeGraph(&graph);

    FreeCtrl(&ctrl);
  }

  /* ************************************************************************* */
  /**
   * Return the number of edge cuts and the partition indices {part}
   * Part [j] is 0 or 1, depending on
   * whether node j is in the left part of the graph or the right part respectively
   */
  std::pair<int, sharedInts> edgeMetis(idx_t n, const sharedInts& xadj,  const sharedInts& adjncy,
    const sharedInts& adjwgt, bool verbose) {

    // control parameters
    std::vector<idx_t> vwgt;                  // the weights of the vertices
    idx_t options[METIS_NOPTIONS];
    METIS_SetDefaultOptions(options);  // use defaults
    idx_t edgecut;                      // the number of edge cuts, output
    sharedInts part_(new idx_t[n]);      // the partition of each vertex, output

    // set uniform weights on the vertices
    vwgt.assign(n, 1);

    //TODO: Fix later
    //boost::timer TOTALTmr;
    if (verbose) {
      printf("**********************************************************************\n");
      printf("Graph Information ---------------------------------------------------\n");
      printf("  #Vertices: %d, #Edges: %u\n", n, *(xadj.get()+n) / 2);
      printf("\nND Partitioning... -------------------------------------------\n");
      //cleartimer(TOTALTmr);
      //starttimer(TOTALTmr);
    }

    //int wgtflag = 1; // only edge weights
    //int numflag = 0; // c style numbering starting from 0
    //int nparts = 2; // partition the graph to 2 submaps
    modefied_EdgeComputeSeparator(&n, xadj.get(), adjncy.get(), &vwgt[0], adjwgt.get(),
        options, &edgecut, part_.get());


    if (verbose) {
      //stoptimer(TOTALTmr);
      printf("\nTiming Information --------------------------------------------------\n");
      //printf("  Total:        \t\t %7.3f\n", gettimer(TOTALTmr));
      printf("  Edge cuts:    \t\t %d\n", edgecut);
      printf("**********************************************************************\n");
    }

    return std::make_pair(edgecut, part_);
  }

  /* ************************************************************************* */
  /**
   * Prepare the data structure {xadj} and {adjncy} required by metis
   * xadj always has the size equal to the no. of the nodes plus 1
   * adjncy always has the size equal to two times of the no. of the edges in the Metis graph
   */
  template <class GenericGraph>
  void prepareMetisGraph(const GenericGraph& graph, const std::vector<size_t>& keys, WorkSpace& workspace,
      sharedInts* ptr_xadj, sharedInts* ptr_adjncy, sharedInts* ptr_adjwgt) {

    typedef std::vector<int> Weights;
    typedef std::vector<int> Neighbors;
    typedef std::pair<Neighbors, Weights> NeighborsInfo;

    // set up dictionary
    std::vector<int>& dictionary = workspace.dictionary;
    workspace.prepareDictionary(keys);

    // prepare for {adjacencyMap}, a pair of neighbor indices and the correponding edge weights
    int numNodes = keys.size();
    int numEdges = 0;
    std::vector<NeighborsInfo> adjacencyMap;
    adjacencyMap.resize(numNodes);
    std::cout << "Number of nodes: " << adjacencyMap.size() << std::endl;
    int index1, index2;

    for(const typename GenericGraph::value_type& factor: graph){
      index1 = dictionary[factor->key1.index];
      index2 = dictionary[factor->key2.index];
      std::cout << "index1: " << index1 << std::endl;
      std::cout << "index2: " << index2 << std::endl;
      // if both nodes are in the current graph, i.e. not a joint factor between frontal and separator
      if (index1 >= 0 && index2 >= 0) {
        std::pair<Neighbors, Weights>& adjacencyMap1 = adjacencyMap[index1];
        std::pair<Neighbors, Weights>& adjacencyMap2 = adjacencyMap[index2];
        try{
         adjacencyMap1.first.push_back(index2);
         adjacencyMap1.second.push_back(factor->weight);
         adjacencyMap2.first.push_back(index1);
         adjacencyMap2.second.push_back(factor->weight);
        }catch(std::exception& e){
          std::cout << e.what() << std::endl;
        }
        numEdges++;
      }
    }

    // prepare for {xadj}, {adjncy}, and {adjwgt}
    *ptr_xadj   = sharedInts(new idx_t[numNodes+1]);
    *ptr_adjncy = sharedInts(new idx_t[numEdges*2]);
    *ptr_adjwgt = sharedInts(new idx_t[numEdges*2]);
    sharedInts& xadj = *ptr_xadj;
    sharedInts& adjncy = *ptr_adjncy;
    sharedInts& adjwgt = *ptr_adjwgt;
    int ind_xadj = 0, ind_adjncy = 0;
    for(const NeighborsInfo& info: adjacencyMap) {
      *(xadj.get() + ind_xadj) = ind_adjncy;
      std::copy(info.first .begin(), info.first .end(), adjncy.get() + ind_adjncy);
      std::copy(info.second.begin(), info.second.end(), adjwgt.get() + ind_adjncy);
      assert(info.first.size() == info.second.size());
      ind_adjncy += info.first.size();
      ind_xadj ++;
    }
    if (ind_xadj != numNodes) throw std::runtime_error("prepareMetisGraph_: ind_xadj != numNodes");
    *(xadj.get() + ind_xadj) = ind_adjncy;
  }

  /* ************************************************************************* */
  template<class GenericGraph>
  std::optional<MetisResult> separatorPartitionByMetis(const GenericGraph& graph,
    const std::vector<size_t>& keys, WorkSpace& workspace, bool verbose) {
    // create a metis graph
    size_t numKeys = keys.size();
    if (verbose)
      std::cout << graph.size() << " factors,\t" << numKeys << " nodes;\t" << std::endl;

    sharedInts xadj, adjncy, adjwgt;

    prepareMetisGraph<GenericGraph>(graph, keys, workspace, &xadj, &adjncy, &adjwgt);

    // run ND on the graph
    size_t sepsize;
    sharedInts part;
    std::tie(sepsize, part) = separatorMetis(numKeys, xadj, adjncy, adjwgt, verbose);
    if (!sepsize)  return std::optional<MetisResult>();

    // convert the 0-1-2 from Metis to 1-2-0, so that the separator is 0, as later
    //  we will have more submaps
    MetisResult result;
    result.C.reserve(sepsize);
    result.A.reserve(numKeys - sepsize);
    result.B.reserve(numKeys - sepsize);
    int* ptr_part = part.get();
    std::vector<size_t>::const_iterator itKey = keys.begin();
    std::vector<size_t>::const_iterator itKeyLast = keys.end();
    while(itKey != itKeyLast) {
      switch(*(ptr_part++)) {
      case 0: result.A.push_back(*(itKey++)); break;
      case 1: result.B.push_back(*(itKey++)); break;
      case 2: result.C.push_back(*(itKey++)); break;
      default: throw std::runtime_error("separatorPartitionByMetis: invalid results from Metis ND!");
      }
    }

    if (verbose) {
      std::cout << "total key: " << keys.size()
                    << " result(A,B,C) = " << result.A.size() << ", " << result.B.size() << ", "
                    << result.C.size() << "; sepsize from Metis = " << sepsize << std::endl;
      //throw runtime_error("separatorPartitionByMetis:stop for debug");
    }

    if(result.C.size() != sepsize) {
      std::cout << "total key: " << keys.size()
          << " result(A,B,C) = " << result.A.size() << ", " << result.B.size() << ", " << result.C.size()
          << "; sepsize from Metis = " << sepsize << std::endl;
      throw std::runtime_error("separatorPartitionByMetis: invalid sepsize from Metis ND!");
    }

    return result;
  }

  /* *************************************************************************/
  template<class GenericGraph>
  std::optional<MetisResult> edgePartitionByMetis(const GenericGraph& graph,
   const std::vector<size_t>& keys, WorkSpace& workspace, bool verbose) {

    // a small hack for handling the camera1-camera2 case used in the unit tests
    if (graph.size() == 1 && keys.size() == 2) {
      MetisResult result;
      result.A.push_back(keys.front());
      result.B.push_back(keys.back());
      return result;
    }

    // create a metis graph
    size_t numKeys = keys.size();
    if (verbose) std::cout << graph.size() << " factors,\t" << numKeys << " nodes;\t" << std::endl;
    sharedInts xadj, adjncy, adjwgt;
    prepareMetisGraph<GenericGraph>(graph, keys, workspace, &xadj, &adjncy, &adjwgt);

    // run metis on the graph
    int edgecut;
    sharedInts part;
    std::tie(edgecut, part) = edgeMetis(numKeys, xadj, adjncy, adjwgt, verbose);

    // convert the 0-1-2 from Metis to 1-2-0, so that the separator is 0, as later we will have more submaps
    MetisResult result;
    result.A.reserve(numKeys);
    result.B.reserve(numKeys);
    int* ptr_part = part.get();
    std::vector<size_t>::const_iterator itKey = keys.begin();
    std::vector<size_t>::const_iterator itKeyLast = keys.end();
    while(itKey != itKeyLast) {
      if (*ptr_part != 0 && *ptr_part != 1)
        std::cout << *ptr_part << "!!!" << std::endl;
      switch(*(ptr_part++)) {
      case 0: result.A.push_back(*(itKey++)); break;
      case 1: result.B.push_back(*(itKey++)); break;
      default: throw std::runtime_error("edgePartitionByMetis: invalid results from Metis ND!");
      }
    }

    if (verbose) {
      std::cout << "the size of two submaps in the reduced graph: " << result.A.size()
        << " " << result.B.size() << std::endl;
      int edgeCut = 0;

      for(const typename GenericGraph::value_type& factor: graph){
        int key1 = factor->key1.index;
        int key2 = factor->key2.index;
        // print keys and their subgraph assignment
        std::cout << key1;
        if (std::find(result.A.begin(), result.A.end(), key1) != result.A.end()) std::cout <<"A ";
        if (std::find(result.B.begin(), result.B.end(), key1) != result.B.end()) std::cout <<"B ";

        std::cout << key2;
        if (std::find(result.A.begin(), result.A.end(), key2) != result.A.end()) std::cout <<"A ";
        if (std::find(result.B.begin(), result.B.end(), key2) != result.B.end()) std::cout <<"B ";
        std::cout << "weight " << factor->weight;;

        // find vertices that were assigned to sets A & B. Their edge will be cut
        if ((std::find(result.A.begin(), result.A.end(), key1) != result.A.end() &&
             std::find(result.B.begin(), result.B.end(), key2) != result.B.end()) ||
            (std::find(result.B.begin(), result.B.end(), key1) != result.B.end() &&
              std::find(result.A.begin(), result.A.end(), key2) != result.A.end())){
          edgeCut ++;
          std::cout << " CUT ";
        }
        std::cout << std::endl;
      }
      std::cout << "edgeCut: " << edgeCut << std::endl;
    }

    return result;
  }

  /* ************************************************************************* */
  bool isLargerIsland(const std::vector<size_t>& island1, const std::vector<size_t>& island2) {
    return island1.size() > island2.size();
  }

  /* ************************************************************************* */
  // debug functions
  void printIsland(const std::vector<size_t>& island) {
    std::cout << "island: ";
    for(const size_t key: island)
      std::cout << key << " ";
    std::cout << std::endl;
  }

  void printIslands(const std::list<std::vector<size_t> >& islands) {
    for(const std::vector<std::size_t>& island: islands)
        printIsland(island);
  }

  void printNumCamerasLandmarks(const std::vector<size_t>& keys, const std::vector<Symbol>& int2symbol) {
    int numCamera = 0, numLandmark = 0;
    for(const size_t key: keys)
    if (int2symbol[key].chr() == 'x')
      numCamera++;
    else
      numLandmark++;
    std::cout << "numCamera: " << numCamera << " numLandmark: " << numLandmark << std::endl;
  }

  /* ************************************************************************* */
  template<class GenericGraph>
  void addLandmarkToPartitionResult(const GenericGraph& graph, const std::vector<size_t>& landmarkKeys,
      MetisResult& partitionResult, WorkSpace& workspace) {

    // set up cameras in the dictionary
    std::vector<size_t>& A = partitionResult.A;
    std::vector<size_t>& B = partitionResult.B;
    std::vector<size_t>& C = partitionResult.C;
    std::vector<int>& dictionary = workspace.dictionary;
    std::fill(dictionary.begin(), dictionary.end(), -1);
    for(const size_t a: A)
      dictionary[a] = 1;
    for(const size_t b: B)
      dictionary[b] = 2;
    if (!C.empty())
      throw std::runtime_error("addLandmarkToPartitionResult: C is not empty");

    // set up landmarks
    size_t i,j;
    for(const typename GenericGraph::value_type& factor: graph) {
      i = factor->key1.index;
      j = factor->key2.index;
      if (dictionary[j] == 0) // if the landmark is already in the separator, continue
        continue;
      else if (dictionary[j] == -1)
        dictionary[j] = dictionary[i];
      else {
        if (dictionary[j] != dictionary[i])
          dictionary[j] = 0;
      }
//      if (j == 67980)
//        std::cout << "dictionary[67980]" << dictionary[j] << std::endl;
    }

    for(const size_t j: landmarkKeys) {
      switch(dictionary[j]) {
      case 0: C.push_back(j); break;
      case 1: A.push_back(j); break;
      case 2: B.push_back(j); break;
      default: std::cout << j << ": " << dictionary[j] << std::endl;
      throw std::runtime_error("addLandmarkToPartitionResult: wrong status for landmark");
      }
    }
  }

#define REDUCE_CAMERA_GRAPH

  /* ************************************************************************* */
  template<class GenericGraph>
  std::optional<MetisResult> findPartitoning(const GenericGraph& graph, const std::vector<size_t>& keys,
      WorkSpace& workspace, bool verbose,
      const std::optional<std::vector<Symbol> >& int2symbol, const bool reduceGraph) {
    std::optional<MetisResult> result;
    GenericGraph reducedGraph;
    std::vector<size_t> keyToPartition;
    std::vector<size_t> cameraKeys, landmarkKeys;
    if (reduceGraph) {
      if (!int2symbol.has_value())
        throw std::invalid_argument("findSeparator: int2symbol must be valid!");

      // find out all the landmark keys, which are to be eliminated
      cameraKeys.reserve(keys.size());
      landmarkKeys.reserve(keys.size());
      for(const size_t key: keys) {
        if((*int2symbol)[key].chr() == 'x')
          cameraKeys.push_back(key);
        else
          landmarkKeys.push_back(key);
      }

      keyToPartition = cameraKeys;
      workspace.prepareDictionary(keyToPartition);
      const std::vector<int>& dictionary = workspace.dictionary;
      reduceGenericGraph(graph, cameraKeys, landmarkKeys, dictionary, reducedGraph);
      std::cout << "original graph: V" << keys.size() << ", E" << graph.size()
      << " --> reduced graph: V" << cameraKeys.size() << ", E" << reducedGraph.size() << std::endl;
      result = edgePartitionByMetis(reducedGraph, keyToPartition, workspace, verbose);
    }  else // call Metis to partition the graph to A, B, C
      result = separatorPartitionByMetis(graph, keys, workspace, verbose);

    if (!result.has_value()) {
      std::cout << "metis failed!" << std::endl;
      return {};
    }

    if (reduceGraph) {
      addLandmarkToPartitionResult(graph, landmarkKeys, *result, workspace);
      std::cout << "the separator size: " << result->C.size() << " landmarks" << std::endl;
    }

    return result;
  }

  /* ************************************************************************* */
  template<class GenericGraph>
  int findSeparator(const GenericGraph& graph, const std::vector<size_t>& keys,
      const int minNodesPerMap, WorkSpace& workspace, bool verbose,
      const std::optional<std::vector<Symbol> >& int2symbol, const bool reduceGraph,
      const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark) {

    std::optional<MetisResult> result = findPartitoning(graph, keys, workspace,
      verbose, int2symbol, reduceGraph);

    // find the island in A and B, and make them separated submaps
    typedef std::vector<size_t> Island;
    std::list<Island> islands;

    std::list<Island> islands_in_A = findIslands(graph, result->A, workspace,
      minNrConstraintsPerCamera, minNrConstraintsPerLandmark);

    std::list<Island> islands_in_B = findIslands(graph, result->B, workspace,
      minNrConstraintsPerCamera, minNrConstraintsPerLandmark);

    islands.insert(islands.end(), islands_in_A.begin(), islands_in_A.end());
    islands.insert(islands.end(), islands_in_B.begin(), islands_in_B.end());
    islands.sort(isLargerIsland);
    size_t numIsland0 = islands.size();

#ifdef NDEBUG
//    verbose = true;
//    if (!int2symbol) throw std::invalid_argument("findSeparator: int2symbol is not set!");
//    std::cout << "sep size: " << result->C.size() << "; ";
//    printNumCamerasLandmarks(result->C, *int2symbol);
//    std::cout << "no. of island: " << islands.size() << "; ";
//    std::cout << "island size: ";
//    for(const Island& island: islands)
//      std::cout << island.size() << " ";
//    std::cout << std::endl;

//    for(const Island& island: islands) {
//      printNumCamerasLandmarks(island, int2symbol);
//    }
#endif

    // absorb small components into the separator
    size_t oldSize = islands.size();
    while(true) {
      if (islands.size() < 2) {
        std::cout << "numIsland: " << numIsland0 << std::endl;
        throw std::runtime_error("findSeparator: found fewer than 2 submaps!");
      }

      std::list<Island>::reference island = islands.back();
      if ((int)island.size() >= minNodesPerMap) break;
      result->C.insert(result->C.end(), island.begin(), island.end());
      islands.pop_back();
    }
    if (islands.size() != oldSize){
      if (verbose) std::cout << oldSize << "-" << oldSize - islands.size() << " submap(s);\t" << std::endl;
    }
    else{
      if (verbose) std::cout << oldSize << " submap(s);\t" << std::endl;
    }

    // generate the node map
    std::vector<int>& partitionTable = workspace.partitionTable;
    std::fill(partitionTable.begin(), partitionTable.end(), -1);
    for(const size_t key: result->C)
      partitionTable[key] = 0;
    int idx = 0;
    for(const Island& island: islands) {
      idx++;
      for(const size_t key: island) {
        partitionTable[key] = idx;
      }
    }

    return islands.size();
  }

}} //namespace
