/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Assignment.h
 * @brief    An assignment from labels to a discrete value index (uint64_t)
 * @author  Frank Dellaert
 * @date    Feb 5, 2012
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <iostream>
#include <vector>
#include <map>


namespace gtsam {

  /**
   * An assignment from labels to value index (uint64_t).
   * Assigns to each label a value. Implemented as a simple map.
   * A discrete factor takes an Assignment and returns a value.
   */
  template<class L>
  class Assignment: public std::map<L, uint64_t> {
  public:
    void print(const std::string& s = "Assignment: ") const {
      std::cout << s << ": ";
      for(const typename Assignment::value_type& keyValue: *this)
        std::cout << "(" << keyValue.first << ", " << keyValue.second << ")";
      std::cout << std::endl;
    }

    bool equals(const Assignment& other, double tol = 1e-9) const {
      return (*this == other);
    }
  };  //Assignment


  /**
   * @brief Get Cartesian product consisting all possible configurations
   * @param vector list of keys (label,cardinality) pairs.
   * @return vector list of all possible value assignments
   *
   * This function returns a vector of Values for all possible
   * (Cartesian product) configurations of set of Keys which are nothing
   * but (Label,cardinality) pairs. This function should NOT be called for
   * more than a small number of variables and cardinalities. E.g. For 6
   * variables with each having cardinalities 4, we get 4096 possible
   * configurations!!
   */
  template <typename L>
  std::vector<Values> cartesianProduct(
      const std::vector<std::pair<L, uint64_t> >& keys) {
    std::vector<Values> allPossValues;
    Values values;
    typedef std::pair<L, uint64_t> DiscreteKey;
    for (const DiscreteKey& key : keys) {
      values.insert<uint64_t>(key.first, 0);  // Initialize from 0
    }

    while (true) {
      allPossValues.push_back(values);
      uint64_t j = 0;
      for (j = 0; j < keys.size(); j++) {
        L idx = keys[j].first;
        // increment the value at key `idx`
        uint64_t val = values.at<uint64_t>(idx);
        values.update<uint64_t>(idx, val + 1);

        if (values.at<uint64_t>(idx) < keys[j].second) {
          break;
        }
        // Wrap condition
        values.update<uint64_t>(idx, 0);
      }
      if (j == keys.size()) {
        break;
      }
    }
    return allPossValues;
  }

} // namespace gtsam
