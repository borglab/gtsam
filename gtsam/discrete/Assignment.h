/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Assignment.h
 * @brief   An assignment from labels to a discrete value index (size_t)
 * @author  Frank Dellaert
 * @date    Feb 5, 2012
 */

#pragma once

#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <utility>
#include <vector>

namespace gtsam {

/**
 * An assignment from labels to value index (size_t).
 * Assigns to each label a value. Implemented as a simple map.
 * A discrete factor takes an Assignment and returns a value.
 * @ingroup discrete
 */
template <class L>
class Assignment : public std::map<L, size_t> {
  /**
   * @brief Default method used by `labelFormatter` or `valueFormatter` when
   * printing.
   *
   * @param x The value passed to format.
   * @return std::string
   */
  static std::string DefaultFormatter(const L& x) {
    std::stringstream ss;
    ss << x;
    return ss.str();
  }

 public:
  using std::map<L, size_t>::operator=;

  // Define the implicit default constructor.
  Assignment() = default;

  // Construct from initializer list.
  Assignment(std::initializer_list<std::pair<const L, size_t>> init)
      : std::map<L, size_t>{init} {}

  void print(const std::string& s = "Assignment: ",
             const std::function<std::string(L)>& labelFormatter =
                 &DefaultFormatter) const {
    std::cout << s << ": ";
    for (const typename Assignment::value_type& keyValue : *this) {
      std::cout << "(" << labelFormatter(keyValue.first) << ", "
                << keyValue.second << ")";
    }
    std::cout << std::endl;
  }

  bool equals(const Assignment& other, double tol = 1e-9) const {
    return (*this == other);
  }

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
  template <typename AssignmentType = Assignment<L>>
  static std::vector<AssignmentType> CartesianProduct(
      const std::vector<std::pair<L, size_t>>& keys) {
    std::vector<AssignmentType> allPossValues;
    AssignmentType assignment;
    for (const auto& [idx, _] : keys) assignment[idx] = 0;  // Initialize from 0

    const size_t nrKeys = keys.size();
    while (true) {
      allPossValues.push_back(assignment);

      // Increment the assignment. This generalizes incrementing a binary number
      size_t j = 0;
      for (j = 0; j < nrKeys; j++) {
        auto [idx, cardinality] = keys[j];
        // Most of the time, we just increment the value for the first key, j=0:
        assignment[idx]++;
        // But if this key is done, we increment next key.
        const bool carry = (assignment[idx] == cardinality);
        if (!carry) break;
        assignment[idx] = 0; // wrap on carry, and continue to next variable
      }

      // If we propagated carry past the last key, exit:
      if (j == nrKeys) break;
    }
    return allPossValues;
  }
};  // Assignment

}  // namespace gtsam
