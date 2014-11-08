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
    template<class FACTOR>
    void MetisIndex::augment(const FactorGraph<FACTOR>& factors)
    {
        std::map<int, FastSet<int> > adjMap;
        std::map<int, FastSet<int> >::iterator adjMapIt;
        std::set<Key> keySet;
        
        /* ********** Convert to CSR format ********** */
        // Assuming that vertex numbering starts from 0 (C style),
        // then the adjacency list of vertex i is stored in array adjncy
        // starting at index xadj[i] and ending at(but not including)
        // index xadj[i + 1](i.e., adjncy[xadj[i]] through
        // and including adjncy[xadj[i + 1] - 1]).
        for (size_t i = 0; i < factors.size(); i++){
          if (factors[i]){
            BOOST_FOREACH(const Key& k1, *factors[i]){
              BOOST_FOREACH(const Key& k2, *factors[i]){
                if (k1 != k2)
                  adjMap[k1].insert(adjMap[k1].end(), k2); // Insert at the end
              }
              keySet.insert(keySet.end(), k1); // Keep a track of all unique keySet
            }
          }
        }

        // Number of keys referenced in this factor graph
        nKeys_ = keySet.size();

		
		// Starting with a nonzero key crashes METIS
		// Find the smallest key in the graph
		size_t minKey = *keySet.begin(); // set is ordered
		
        xadj_.push_back(0);// Always set the first index to zero
        for (adjMapIt = adjMap.begin(); adjMapIt != adjMap.end(); ++adjMapIt) {
            std::vector<Key> temp;
            // Copy from the FastSet into a temporary vector
            std::copy(adjMapIt->second.begin(), adjMapIt->second.end(), std::back_inserter(temp));
            // Insert each index's set in order by appending them to the end of adj_
            adj_.insert(adj_.end(), temp.begin(), temp.end());
            //adj_.push_back(temp);
            xadj_.push_back(adj_.size());
        }

		// Normalize, subtract the smallest key
		std::transform(adj_.begin(), adj_.end(), adj_.begin(), std::bind2nd(std::minus<size_t>(), minKey));


    }

}
