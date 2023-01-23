/*
 * PartitionWorkSpace.h
 *
 *   Created on: Nov 24, 2010
 *       Author: nikai
 *  Description: a preallocated memory space used in partitioning
 */

#pragma once

#include <vector>
#include <memory>

namespace gtsam { namespace partition {

  typedef std::vector<int> PartitionTable;

  // the work space, preallocated memory
  struct WorkSpace {
    std::vector<int> dictionary;                          // a mapping from the integer key in the original graph to 0-based index in the subgraph, useful when handling a subset of keys and graphs
    std::shared_ptr<std::vector<size_t> > dsf;          // a block memory pre-allocated for DSFVector
    PartitionTable partitionTable;                        // a mapping from a key to the submap index, 0 means the separator, i means the ith submap

    // constructor
    WorkSpace(const size_t numNodes) : dictionary(numNodes,0),
     dsf(new std::vector<size_t>(numNodes, 0)), partitionTable(numNodes, -1) { }

    // set up dictionary: -1: no such key, none-zero: the corresponding 0-based index
    inline void prepareDictionary(const std::vector<size_t>& keys) {
      int index = 0;
      std::fill(dictionary.begin(), dictionary.end(), -1);
      std::vector<size_t>::const_iterator it=keys.begin(), itLast=keys.end();
      while(it!=itLast)  dictionary[*(it++)] = index++;
    }
  };


  // manually defined cuts
  struct Cuts {
    PartitionTable partitionTable;
    std::vector<std::shared_ptr<Cuts> > children;
  };

}} // namespace
