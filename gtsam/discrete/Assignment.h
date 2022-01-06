/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Assignment.h
 * @brief    An assignment from labels to a discrete value index (size_t)
 * @author  Frank Dellaert
 * @date    Feb 5, 2012
 */

#pragma once

#include <iostream>
#include <vector>
#include <map>


namespace gtsam {

  /**
   * An assignment from labels to value index (size_t).
   * Assigns to each label a value. Implemented as a simple map.
   * A discrete factor takes an Assignment and returns a value.
   */
  template<class L>
  class Assignment: public std::map<L, size_t> {
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
   * This function returns a vector of Assignment values for all possible
   * (Cartesian product) configurations of set of Keys which are nothing
   * but (Label,cardinality) pairs. This function should NOT be called for
   * more than a small number of variables and cardinalities. E.g. For 6
   * variables with each having cardinalities 4, we get 4096 possible
   * configurations!!
   */
  template <typename L>
  std::vector<Assignment<L>> cartesianProduct(const std::map<L, size_t>& keys) {
    std::vector<Assignment<L>> allPossValues;
    Assignment<L> values;

    using LKey = std::pair<L, size_t>;
    for (const LKey& key : keys) {
      values[key.first] = 0;  // Initialize from 0
    }

    while (1) {
      allPossValues.push_back(values);

      typename std::map<L, size_t>::const_iterator it;
      for (it = keys.begin(); it != keys.end(); it++) {
        L idx = it->first;
        values[idx]++;
        if (values[idx] < it->second) break;
        break;
        // Wrap condition
        values[idx] = 0;
      }
      if (it == keys.end()) break;
    }
    return allPossValues;
  }

} // namespace gtsam
