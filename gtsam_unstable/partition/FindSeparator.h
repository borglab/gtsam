/*
 * FindSeparator.h
 *
 *   Created on: Nov 23, 2010
 *       Author: nikai
 *  Description: find the separator of bisectioning for a given graph
 */

#pragma once

#include <map>
#include <vector>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include "PartitionWorkSpace.h"

namespace gtsam { namespace partition {

//  typedef std::map<size_t, size_t> PartitionTable; // from the key to the partition: 0 - separator, > 1: submap id

  /** the metis Nest dissection result */
  struct MetisResult {
    std::vector<size_t> A, B;  // frontals
    std::vector<size_t> C;     // separator
  };

  /**
   * use Metis library to partition, return the size of separator and the optional partition table
   * the size of dictionary mush be equal to the number of variables in the original graph (the largest one)
   */
  template<class GenericGraph>
  boost::optional<MetisResult> separatorPartitionByMetis(const GenericGraph& graph, const std::vector<size_t>& keys,
      WorkSpace& workspace, bool verbose);

  /**
   * return the number of submaps and the parition table of the partitioned graph (**stored in workspace.partitionTable**).
   * return 0 if failed Note that the original output of Metis is 0,1 for submap, and 2 for the separator.
   */
  template<class GenericGraph>
  int findSeparator(const GenericGraph& graph, const std::vector<size_t>& keys,
      const int minNodesPerMap, WorkSpace& workspace, bool verbose, const boost::optional<std::vector<Symbol> >& int2symbol,
      const bool reduceGraph, const int minNrConstraintsPerCamera, const int minNrConstraintsPerLandmark);

}} //namespace
