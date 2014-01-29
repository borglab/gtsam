/*
 * FindSeparator-inl.h
 *
 *   Created on: Nov 23, 2010
 *       Author: nikai
 *  Description: find the separator of bisectioning for a given graph
 */

#pragma once

#include <stdexcept>
#include <iostream>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_array.hpp>

extern "C" {
#include <metis.h>
}

#include "FindSeparator.h"

using namespace std;

namespace gtsam { namespace partition {

	typedef boost::shared_array<idxtype> sharedInts;

	/* ************************************************************************* */
	/**
	 * Return the size of the separator and the partiion indices {part}
	 * Part [j] is 0, 1, or 2, depending on
	 * whether node j is in the left part of the graph, the right part, or the
	 * separator, respectively
	 */
	pair<int, sharedInts> separatorMetis(int n, const sharedInts& xadj,	const sharedInts& adjncy, const sharedInts& adjwgt, bool verbose) {

		// control parameters
		idxtype vwgt[n];                  // the weights of the vertices
		int options[8]; options [0] = 0 ;	// use defaults
		int sepsize;                      // the size of the separator, output
		sharedInts part_(new int[n]);      // the partition of each vertex, output

		// set uniform weights on the vertices
		std::fill(vwgt, vwgt+n, 1);

		timer TOTALTmr;
		if (verbose) {
			printf("**********************************************************************\n");
			printf("Graph Information ---------------------------------------------------\n");
			printf("  #Vertices: %d, #Edges: %u\n", n, *(xadj.get()+n) / 2);
			printf("\nND Partitioning... -------------------------------------------\n");
			cleartimer(TOTALTmr);
			starttimer(TOTALTmr);
		}

		// call metis parition routine
		METIS_NodeComputeSeparator(&n, xadj.get(), adjncy.get(), vwgt, adjwgt.get(),
				options, &sepsize, part_.get());

		if (verbose) {
			stoptimer(TOTALTmr);
			printf("\nTiming Information --------------------------------------------------\n");
			printf("  Total:        \t\t %7.3f\n", gettimer(TOTALTmr));
			printf("  Sep size:        \t\t %d\n", sepsize);
			printf("**********************************************************************\n");
		}

		return make_pair(sepsize, part_);
	}

	/* ************************************************************************* */
	void modefied_EdgeComputeSeparator(int *nvtxs, idxtype *xadj, idxtype *adjncy, idxtype *vwgt,
			idxtype *adjwgt, int *options, int *edgecut, idxtype *part)
	{
	  int i, j, tvwgt, tpwgts[2];
	  GraphType graph;
	  CtrlType ctrl;

	  SetUpGraph(&graph, OP_ONMETIS, *nvtxs, 1, xadj, adjncy, vwgt, adjwgt, 3);
	  tvwgt = idxsum(*nvtxs, graph.vwgt);

	  if (options[0] == 0) {  /* Use the default parameters */
	    ctrl.CType = ONMETIS_CTYPE;
	    ctrl.IType = ONMETIS_ITYPE;
	    ctrl.RType = ONMETIS_RTYPE;
	    ctrl.dbglvl = ONMETIS_DBGLVL;
	  }
	  else {
	    ctrl.CType = options[OPTION_CTYPE];
	    ctrl.IType = options[OPTION_ITYPE];
	    ctrl.RType = options[OPTION_RTYPE];
	    ctrl.dbglvl = options[OPTION_DBGLVL];
	  }

	  ctrl.oflags  = 0;
	  ctrl.pfactor = 0;
	  ctrl.nseps = 1;
	  ctrl.optype = OP_OEMETIS;
	  ctrl.CoarsenTo = amin(100, *nvtxs-1);
	  ctrl.maxvwgt = 1.5*tvwgt/ctrl.CoarsenTo;

	  InitRandom(options[7]);

	  AllocateWorkSpace(&ctrl, &graph, 2);

	  /*============================================================
	   * Perform the bisection
	   *============================================================*/
	  tpwgts[0] = tvwgt/2;
	  tpwgts[1] = tvwgt-tpwgts[0];

	  MlevelEdgeBisection(&ctrl, &graph, tpwgts, 1.05);
	//  ConstructMinCoverSeparator(&ctrl, &graph, 1.05);
	  *edgecut = graph.mincut;
	//  *sepsize = graph.pwgts[2];
	  idxcopy(*nvtxs, graph.where, part);

	  GKfree((void**)&graph.gdata, &graph.rdata, &graph.label, LTERM);


	  FreeWorkSpace(&ctrl, &graph);

	}

	/* ************************************************************************* */
	/**
	 * Return the number of edge cuts and the partiion indices {part}
	 * Part [j] is 0 or 1, depending on
	 * whether node j is in the left part of the graph or the right part respectively
	 */
	pair<int, sharedInts> edgeMetis(int n, const sharedInts& xadj,	const sharedInts& adjncy, const sharedInts& adjwgt, bool verbose) {

		// control parameters
		idxtype vwgt[n];                  // the weights of the vertices
		int options[10]; options [0] = 1;	options [1] = 3; options [2] = 1; options [3] = 1; options [4] = 0; // use defaults
		int edgecut;                      // the number of edge cuts, output
		sharedInts part_(new int[n]);      // the partition of each vertex, output

		// set uniform weights on the vertices
		std::fill(vwgt, vwgt+n, 1);

		timer TOTALTmr;
		if (verbose) {
			printf("**********************************************************************\n");
			printf("Graph Information ---------------------------------------------------\n");
			printf("  #Vertices: %d, #Edges: %u\n", n, *(xadj.get()+n) / 2);
			printf("\nND Partitioning... -------------------------------------------\n");
			cleartimer(TOTALTmr);
			starttimer(TOTALTmr);
		}

		// call metis parition routine
		int wgtflag = 1; // only edge weights
		int numflag = 0; // c style numbering starting from 0
		int nparts = 2; // partition the graph to 2 submaps
//		METIS_PartGraphRecursive(&n, xadj.get(), adjncy.get(), NULL, adjwgt.get(), &wgtflag,
//				&numflag, &nparts, options, &edgecut, part_.get());

		modefied_EdgeComputeSeparator(&n, xadj.get(), adjncy.get(), vwgt, adjwgt.get(),
				options, &edgecut, part_.get());

		if (verbose) {
			stoptimer(TOTALTmr);
			printf("\nTiming Information --------------------------------------------------\n");
			printf("  Total:        \t\t %7.3f\n", gettimer(TOTALTmr));
			printf("  Edge cuts:    \t\t %d\n", edgecut);
			printf("**********************************************************************\n");
		}

		return make_pair(edgecut, part_);
	}

	/* ************************************************************************* */
	/**
	 * Prepare the data structure {xadj} and {adjncy} required by metis
	 * xadj always has the size equal to the no. of the nodes plus 1
	 * adjncy always has the size equal to two times of the no. of the edges in the Metis graph
	 */
	template <class GenericGraph>
	void prepareMetisGraph(const GenericGraph& graph, const vector<size_t>& keys, WorkSpace& workspace,
			sharedInts* ptr_xadj, sharedInts* ptr_adjncy, sharedInts* ptr_adjwgt) {

		typedef int Weight;
		typedef vector<int> Weights;
		typedef vector<int> Neighbors;
		typedef pair<Neighbors, Weights> NeighborsInfo;

		// set up dictionary
		std::vector<int>& dictionary = workspace.dictionary;
		workspace.prepareDictionary(keys);

		// prepare for {adjancyMap}, a pair of neighbor indices and the correponding edge weights
		int numNodes = keys.size();
		int numEdges = 0;
		vector<NeighborsInfo> adjancyMap; // TODO: set is slow, but have to use it to remove duplicated edges
		adjancyMap.resize(numNodes);
		int index1, index2;
		BOOST_FOREACH(const typename GenericGraph::value_type& factor, graph){
			index1 = dictionary[factor->key1.index];
			index2 = dictionary[factor->key2.index];
			if (index1 >= 0 && index2 >= 0) {       // if both nodes are in the current graph, i.e. not a joint factor between frontal and separator
				pair<Neighbors, Weights>& adjancyMap1 = adjancyMap[index1];
				pair<Neighbors, Weights>& adjancyMap2 = adjancyMap[index2];
				adjancyMap1.first .push_back(index2);
				adjancyMap1.second.push_back(factor->weight);
				adjancyMap2.first .push_back(index1);
				adjancyMap2.second.push_back(factor->weight);
				numEdges++;
			}
		}

		// prepare for {xadj}, {adjncy}, and {adjwgt}
		*ptr_xadj   = sharedInts(new int[numNodes+1]);
		*ptr_adjncy = sharedInts(new int[numEdges*2]);
		*ptr_adjwgt = sharedInts(new int[numEdges*2]);
		sharedInts& xadj = *ptr_xadj;
		sharedInts& adjncy = *ptr_adjncy;
		sharedInts& adjwgt = *ptr_adjwgt;
		int ind_xadj = 0, ind_adjncy = 0;
		BOOST_FOREACH(const NeighborsInfo& info, adjancyMap) {
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
	boost::optional<MetisResult> separatorPartitionByMetis(const GenericGraph& graph, const vector<size_t>& keys, WorkSpace& workspace, bool verbose) {
		// create a metis graph
		size_t numKeys = keys.size();
		if (verbose) cout << graph.size() << " factors,\t" << numKeys << " nodes;\t" << endl;
		sharedInts xadj, adjncy, adjwgt;
		prepareMetisGraph<GenericGraph>(graph, keys, workspace, &xadj, &adjncy, &adjwgt);

		// run ND on the graph
		int sepsize;
		sharedInts part;
		boost::tie(sepsize, part) = separatorMetis(numKeys, xadj, adjncy, adjwgt, verbose);
		if (!sepsize)	return boost::optional<MetisResult>();

		// convert the 0-1-2 from Metis to 1-2-0, so that the separator is 0, as later we will have more submaps
		MetisResult result;
		result.C.reserve(sepsize);
		result.A.reserve(numKeys - sepsize);
		result.B.reserve(numKeys - sepsize);
		int* ptr_part = part.get();
		vector<size_t>::const_iterator itKey = keys.begin();
		vector<size_t>::const_iterator itKeyLast = keys.end();
		while(itKey != itKeyLast) {
			switch(*(ptr_part++)) {
			case 0: result.A.push_back(*(itKey++)); break;
			case 1: result.B.push_back(*(itKey++)); break;
			case 2: result.C.push_back(*(itKey++)); break;
			default: throw runtime_error("separatorPartitionByMetis: invalid results from Metis ND!");
			}
		}

		if (verbose) {
			cout << "total key: " << keys.size()
										<< " result(A,B,C) = " << result.A.size() << ", " << result.B.size() << ", " << result.C.size()
										<< "; sepsize from Metis = " << sepsize << endl;
			throw runtime_error("separatorPartitionByMetis:stop for debug");
		}

		if(result.C.size() != sepsize) {
			cout << "total key: " << keys.size()
					<< " result(A,B,C) = " << result.A.size() << ", " << result.B.size() << ", " << result.C.size()
					<< "; sepsize from Metis = " << sepsize << endl;
			throw runtime_error("separatorPartitionByMetis: invalid sepsize from Metis ND!");
		}

		return boost::make_optional<MetisResult >(result);
	}

	/* ************************************************************************* */
	template<class GenericGraph>
	boost::optional<MetisResult> edgePartitionByMetis(const GenericGraph& graph, const vector<size_t>& keys, WorkSpace& workspace, bool verbose) {

		// a small hack for handling the camera1-camera2 case used in the unit tests
		if (graph.size() == 1 && keys.size() == 2) {
			MetisResult result;
			result.A.push_back(keys.front());
			result.B.push_back(keys.back());
			return result;
		}

		// create a metis graph
		size_t numKeys = keys.size();
		if (verbose) cout << graph.size() << " factors,\t" << numKeys << " nodes;\t" << endl;
		sharedInts xadj, adjncy, adjwgt;
		prepareMetisGraph<GenericGraph>(graph, keys, workspace, &xadj, &adjncy, &adjwgt);

		// run metis on the graph
		int edgecut;
		sharedInts part;
		boost::tie(edgecut, part) = edgeMetis(numKeys, xadj, adjncy, adjwgt, verbose);

		// convert the 0-1-2 from Metis to 1-2-0, so that the separator is 0, as later we will have more submaps
		MetisResult result;
		result.A.reserve(numKeys);
		result.B.reserve(numKeys);
		int* ptr_part = part.get();
		vector<size_t>::const_iterator itKey = keys.begin();
		vector<size_t>::const_iterator itKeyLast = keys.end();
		while(itKey != itKeyLast) {
			if (*ptr_part != 0 && *ptr_part != 1)
				cout << *ptr_part << "!!!" << endl;
			switch(*(ptr_part++)) {
			case 0: result.A.push_back(*(itKey++)); break;
			case 1: result.B.push_back(*(itKey++)); break;
			default: throw runtime_error("edgePartitionByMetis: invalid results from Metis ND!");
			}
		}

		if (verbose) {
			cout << "the size of two submaps in the reduced graph: " << result.A.size() << " " << result.B.size() << endl;
			int edgeCut = 0;

			BOOST_FOREACH(const typename GenericGraph::value_type& factor, graph){
				int key1 = factor->key1.index;
				int key2 = factor->key2.index;
				// print keys and their subgraph assignment
				cout << key1;
				if (std::find(result.A.begin(), result.A.end(), key1) != result.A.end()) cout <<"A ";
				if (std::find(result.B.begin(), result.B.end(), key1) != result.B.end()) cout <<"B ";

				cout << key2;
				if (std::find(result.A.begin(), result.A.end(), key2) != result.A.end()) cout <<"A ";
				if (std::find(result.B.begin(), result.B.end(), key2) != result.B.end()) cout <<"B ";
        cout << "weight " << factor->weight;;

				// find vertices that were assigned to sets A & B. Their edge will be cut
				if ((std::find(result.A.begin(), result.A.end(), key1) != result.A.end() &&
  					 std::find(result.B.begin(), result.B.end(), key2) != result.B.end()) ||
  					(std::find(result.B.begin(), result.B.end(), key1) != result.B.end() &&
   					 std::find(result.A.begin(), result.A.end(), key2) != result.A.end())){
					edgeCut ++;
					cout << " CUT ";
				}
				cout << endl;
			}
			cout << "edgeCut: " << edgeCut << endl;
		}

		return boost::make_optional<MetisResult >(result);
	}

	/* ************************************************************************* */
	bool isLargerIsland(const vector<size_t>& island1, const vector<size_t>& island2) {
		return island1.size() > island2.size();
	}

	/* ************************************************************************* */
	// debug functions
	void printIsland(const vector<size_t>& island) {
		cout << "island: ";
		BOOST_FOREACH(const size_t key, island)
			cout << key << " ";
		cout << endl;
	}

	void printIslands(const list<vector<size_t> >& islands) {
		BOOST_FOREACH(const vector<size_t>& island, islands)
				printIsland(island);
	}

	void printNumCamerasLandmarks(const vector<size_t>& keys, const vector<Symbol>& int2symbol) {
		int numCamera = 0, numLandmark = 0;
		BOOST_FOREACH(const size_t key, keys)
		if (int2symbol[key].chr() == 'x')
			numCamera++;
		else
			numLandmark++;
		cout << "numCamera: " << numCamera << " numLandmark: " << numLandmark << endl;
	}

	/* ************************************************************************* */
	template<class GenericGraph>
	void addLandmarkToPartitionResult(const GenericGraph& graph, const vector<size_t>& landmarkKeys,
			MetisResult& partitionResult, WorkSpace& workspace) {

		// set up cameras in the dictionary
		std::vector<size_t>& A = partitionResult.A;
		std::vector<size_t>& B = partitionResult.B;
		std::vector<size_t>& C = partitionResult.C;
		std::vector<int>& dictionary = workspace.dictionary;
		std::fill(dictionary.begin(), dictionary.end(), -1);
		BOOST_FOREACH(const size_t a, A)
			dictionary[a] = 1;
		BOOST_FOREACH(const size_t b, B)
			dictionary[b] = 2;
		if (!C.empty())
			throw runtime_error("addLandmarkToPartitionResult: C is not empty");

		// set up landmarks
		size_t i,j;
		BOOST_FOREACH(const typename GenericGraph::value_type& factor, graph) {
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
//			if (j == 67980)
//				cout << "dictionary[67980]" << dictionary[j] << endl;
		}

		BOOST_FOREACH(const size_t j, landmarkKeys) {
			switch(dictionary[j]) {
			case 0: C.push_back(j); break;
			case 1: A.push_back(j); break;
			case 2: B.push_back(j); break;
			default: cout << j << ": " << dictionary[j] << endl; throw runtime_error("addLandmarkToPartitionResult: wrong status for landmark");
			}
		}
	}

#define REDUCE_CAMERA_GRAPH

	/* ************************************************************************* */
	template<class GenericGraph>
	boost::optional<MetisResult> findPartitoning(const GenericGraph& graph, const vector<size_t>& keys,
			WorkSpace& workspace, bool verbose,
			const boost::optional<vector<Symbol> >& int2symbol, const bool reduceGraph) {
		boost::optional<MetisResult> result;
		GenericGraph reducedGraph;
		vector<size_t> keyToPartition;
		vector<size_t> cameraKeys, landmarkKeys;
		if (reduceGraph) {
			if (!int2symbol.is_initialized())
				throw std::invalid_argument("findSeparator: int2symbol must be valid!");

			// find out all the landmark keys, which are to be eliminated
			cameraKeys.reserve(keys.size());
			landmarkKeys.reserve(keys.size());
			BOOST_FOREACH(const size_t key, keys) {
				if((*int2symbol)[key].chr() == 'x')
					cameraKeys.push_back(key);
				else
					landmarkKeys.push_back(key);
			}

			keyToPartition = cameraKeys;
			workspace.prepareDictionary(keyToPartition);
			const std::vector<int>& dictionary = workspace.dictionary;
			reduceGenericGraph(graph, cameraKeys, landmarkKeys, dictionary, reducedGraph);
			cout << "original graph: V" << keys.size() << ", E" << graph.size() << " --> reduced graph: V" << cameraKeys.size() << ", E" << reducedGraph.size() << endl;
			result = edgePartitionByMetis(reducedGraph, keyToPartition, workspace, verbose);
		}	else // call Metis to partition the graph to A, B, C
			result = separatorPartitionByMetis(graph, keys, workspace, verbose);

		if (!result.is_initialized()) {
			cout << "metis failed!" << endl;
			return 0;
		}

		if (reduceGraph) {
			addLandmarkToPartitionResult(graph, landmarkKeys, *result, workspace);
			cout << "the separator size: " << result->C.size() << " landmarks" << endl;
		}

		return result;
	}

	/* ************************************************************************* */
	template<class GenericGraph>
	int findSeparator(const GenericGraph& graph, const vector<size_t>& keys,
			const int minNodesPerMap, WorkSpace& workspace, bool verbose,
			const boost::optional<vector<Symbol> >& int2symbol, const bool reduceGraph,
			const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark) {

		boost::optional<MetisResult> result = findPartitoning(graph, keys, workspace, verbose, int2symbol, reduceGraph);

		// find the island in A and B, and make them separated submaps
		typedef vector<size_t> Island;
		list<Island> islands;
		list<Island> islands_in_A = findIslands(graph, result->A, workspace, minNrConstraintsPerCamera, minNrConstraintsPerLandmark);
		list<Island> islands_in_B = findIslands(graph, result->B, workspace, minNrConstraintsPerCamera, minNrConstraintsPerLandmark);
		islands.insert(islands.end(), islands_in_A.begin(), islands_in_A.end());
		islands.insert(islands.end(), islands_in_B.begin(), islands_in_B.end());
		islands.sort(isLargerIsland);
		size_t numIsland0 = islands.size();

#ifdef NDEBUG
//		verbose = true;
//		if (!int2symbol) throw std::invalid_argument("findSeparator: int2symbol is not set!");
//		cout << "sep size: " << result->C.size() << "; ";
//		printNumCamerasLandmarks(result->C, *int2symbol);
//		cout << "no. of island: " << islands.size() << "; ";
//		cout << "island size: ";
//		BOOST_FOREACH(const Island& island, islands)
//			cout << island.size() << " ";
//		cout << endl;

//		BOOST_FOREACH(const Island& island, islands) {
//			printNumCamerasLandmarks(island, int2symbol);
//		}
#endif

		// absorb small components into the separator
		int oldSize = islands.size();
		while(true) {
			if (islands.size() < 2) {
				cout << "numIsland: " << numIsland0 << endl;
				throw runtime_error("findSeparator: found fewer than 2 submaps!");
			}

			list<Island>::reference island = islands.back();
			if ((int)island.size() >= minNodesPerMap) break;
			result->C.insert(result->C.end(), island.begin(), island.end());
			islands.pop_back();
		}
		if (islands.size() != oldSize)
			if (verbose) cout << oldSize << "-" << oldSize - islands.size() << " submap(s);\t" << endl;
		else
			if (verbose) cout << oldSize << " submap(s);\t" << endl;

		// generate the node map
		std::vector<int>& partitionTable = workspace.partitionTable;
		std::fill(partitionTable.begin(), partitionTable.end(), -1);
		BOOST_FOREACH(const size_t key, result->C)
			partitionTable[key] = 0;
		int idx = 0;
		BOOST_FOREACH(const Island& island, islands) {
			idx++;
			BOOST_FOREACH(const size_t key, island) {
				partitionTable[key] = idx;
			}
		}

		return islands.size();
	}

}} //namespace
