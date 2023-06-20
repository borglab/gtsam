/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file     LP.h
 * @brief    Struct used to hold a Linear Programming Problem
 * @author   Ivan Dario Jimenez
 * @date     1/24/16
 */

#pragma once

#include <gtsam_unstable/linear/LinearCost.h>
#include <gtsam_unstable/linear/EqualityFactorGraph.h>
#include <gtsam_unstable/linear/InequalityFactorGraph.h>

#include <string>

namespace gtsam {

using namespace std;

/// Mapping between variable's key and its corresponding dimensionality
using KeyDimMap = std::map<Key, size_t>;
/*
 * Iterates through every factor in a linear graph and generates a
 * mapping between every factor key and it's corresponding dimensionality.
 */
template <class LinearGraph>
KeyDimMap collectKeyDim(const LinearGraph& linearGraph) {
  KeyDimMap keyDimMap;
  for (const typename LinearGraph::sharedFactor& factor : linearGraph) {
    if (!factor) continue;
    for (Key key : factor->keys())
      keyDimMap[key] = factor->getDim(factor->find(key));
  }
  return keyDimMap;
}

/**
 * Data structure of a Linear Program
 */
struct LP {
  using shared_ptr = std::shared_ptr<LP>;

  LinearCost cost; //!< Linear cost factor
  EqualityFactorGraph equalities; //!< Linear equality constraints: cE(x) = 0
  InequalityFactorGraph inequalities; //!< Linear inequality constraints: cI(x) <= 0
private:
  mutable KeyDimMap cachedConstrainedKeyDimMap_; //!< cached key-dim map of all variables in the constraints

public:
  /// check feasibility
  bool isFeasible(const VectorValues& x) const {
    return (equalities.error(x) == 0 && inequalities.error(x) == 0);
  }

  /// print
  void print(const string& s = "") const {
    std::cout << s << std::endl;
    cost.print("Linear cost: ");
    equalities.print("Linear equality factors: ");
    inequalities.print("Linear inequality factors: ");
  }

  /// equals
  bool equals(const LP& other, double tol = 1e-9) const {
    return cost.equals(other.cost) && equalities.equals(other.equalities)
        && inequalities.equals(other.inequalities);
  }

  const KeyDimMap& constrainedKeyDimMap() const {
    if (!cachedConstrainedKeyDimMap_.empty())
      return cachedConstrainedKeyDimMap_;
    // Collect key-dim map of all variables in the constraints
    //TODO(Varun) seems like the templated function is causing the multiple symbols error on Windows
    // cachedConstrainedKeyDimMap_ = collectKeyDim(equalities);
    // KeyDimMap keysDim2 = collectKeyDim(inequalities);
    // cachedConstrainedKeyDimMap_.insert(keysDim2.begin(), keysDim2.end());
    cachedConstrainedKeyDimMap_.clear();
    for (auto&& factor : equalities) {
      if (!factor) continue;
      for (Key key : factor->keys()) {
        cachedConstrainedKeyDimMap_[key] = factor->getDim(factor->find(key));
      }
    }
    for (auto&& factor : inequalities) {
      if (!factor) continue;
      for (Key key : factor->keys()) {
        cachedConstrainedKeyDimMap_[key] = factor->getDim(factor->find(key));
      }
    }
    return cachedConstrainedKeyDimMap_;
  }

  Vector costGradient(Key key, const VectorValues& delta) const {
    Vector g = Vector::Zero(delta.at(key).size());
    Factor::const_iterator it = cost.find(key);
    if (it != cost.end()) g = cost.getA(it).transpose();
    return g;
  }
};

/// traits
template<> struct traits<LP> : public Testable<LP> {
};

}
