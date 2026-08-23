/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file DiscreteFactor.cpp
 * @brief discrete factor
 * @date Feb 14, 2011
 * @author Duy-Nguyen Ta
 * @author Frank Dellaert
 */

#include <gtsam/base/Vector.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/TableFactor.h>
#include <gtsam/hybrid/HybridValues.h>

#include <cmath>
#include <sstream>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
bool DiscreteFactor::equals(const DiscreteFactor& lf, double tol) const {
  return Base::equals(lf, tol) && cardinalities_ == lf.cardinalities_;
}

/* ************************************************************************ */
DiscreteKeys DiscreteFactor::discreteKeys() const {
  DiscreteKeys result;
  for (auto&& key : keys()) {
    DiscreteKey dkey(key, cardinality(key));
    if (std::find(result.begin(), result.end(), dkey) == result.end()) {
      result.push_back(dkey);
    }
  }
  return result;
}

/* ************************************************************************* */
double DiscreteFactor::error(const DiscreteValues& values) const {
  return -std::log((*this)(values));
}

/* ************************************************************************* */
double DiscreteFactor::error(const HybridValues& c) const {
  return this->error(c.discrete());
}

/* ************************************************************************ */
AlgebraicDecisionTree<Key> DiscreteFactor::errorTree() const {
  // Get all possible assignments
  DiscreteKeys dkeys = discreteKeys();
  // Reverse to make cartesian product output a more natural ordering.
  DiscreteKeys rdkeys(dkeys.rbegin(), dkeys.rend());
  const auto assignments = DiscreteValues::CartesianProduct(rdkeys);

  // Construct vector with error values
  std::vector<double> errors;
  for (const auto& assignment : assignments) {
    errors.push_back(error(assignment));
  }
  return AlgebraicDecisionTree<Key>(dkeys, errors);
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr DiscreteFactor::multiply(
    const DiscreteFactor::shared_ptr& factor) const {
  if (const auto table = std::dynamic_pointer_cast<TableFactor>(factor)) {
    // The cast and dereference borrow the existing table without copying it.
    // Only this factor's table representation and the product are materialized;
    // make_shared moves the temporary product into its owned allocation.
    return std::make_shared<TableFactor>(*table * toTableFactor());
  }
  return std::make_shared<DecisionTreeFactor>(
      this->operator*(factor->toDecisionTreeFactor()));
}

/* ************************************************************************ */
TableFactor DiscreteFactor::toTableFactor() const {
  return TableFactor(toDecisionTreeFactor());
}

/* ************************************************************************ */
DiscreteFactor::shared_ptr DiscreteFactor::scale() const {
  return this->operator*(1.0 / max());
}

}  // namespace gtsam
