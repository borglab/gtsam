/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    MetisIndex-inl.h
 * @author  Andrew Melim
 * @date    Oct. 10, 2014
 */

#pragma once

#include <map>
#include <vector>

namespace gtsam {

/* ************************************************************************* */
template<class FACTORGRAPH>
void MetisIndex::augment(const FACTORGRAPH& factors) {
  std::map<int32_t, std::set<int32_t> > iAdjMap; // Stores a set of keys that are adjacent to key x, with  adjMap.first
  std::map<int32_t, std::set<int32_t> >::iterator iAdjMapIt;
  std::set<Key> keySet;

  /* ********** Convert to CSR format ********** */
  // Assuming that vertex numbering starts from 0 (C style),
  // then the adjacency list of vertex i is stored in array adjncy
  // starting at index xadj[i] and ending at(but not including)
  // index xadj[i + 1](i.e., adjncy[xadj[i]] through
  // and including adjncy[xadj[i + 1] - 1]).
  int32_t keyCounter = 0;

  // First: Record a copy of each key inside the factorgraph and create a
  // key to integer mapping. This is referenced during the adjaceny step
  for (size_t i = 0; i < factors.size(); i++) {
    if (factors[i]) {
      for(const Key& key: *factors[i]) {
        keySet.insert(keySet.end(), key); // Keep a track of all unique keys
        if (intKeyBMap_.left.find(key) == intKeyBMap_.left.end()) {
          intKeyBMap_.insert(key, keyCounter);
          keyCounter++;
        }
      }
    }
  }

  // Create an adjacency mapping that stores the set of all adjacent keys for every key
  for (size_t i = 0; i < factors.size(); i++) {
    if (factors[i]) {
      for(const Key& k1: *factors[i])
        for(const Key& k2: *factors[i])
          if (k1 != k2) {
            // Store both in Key and int32_t format
            int i = intKeyBMap_.left.at(k1);
            int j = intKeyBMap_.left.at(k2);
            iAdjMap[i].insert(iAdjMap[i].end(), j);
          }
    }
  }

  // Number of keys referenced in this factor graph
  nKeys_ = keySet.size();

  xadj_.push_back(0); // Always set the first index to zero
  for (iAdjMapIt = iAdjMap.begin(); iAdjMapIt != iAdjMap.end(); ++iAdjMapIt) {
    std::vector<int32_t> temp;
    // Copy from the FastSet into a temporary vector
    std::copy(iAdjMapIt->second.begin(), iAdjMapIt->second.end(),
        std::back_inserter(temp));
    // Insert each index's set in order by appending them to the end of adj_
    adj_.insert(adj_.end(), temp.begin(), temp.end());
    //adj_.push_back(temp);
    xadj_.push_back((int32_t) adj_.size());
  }
}

} // \ gtsam
