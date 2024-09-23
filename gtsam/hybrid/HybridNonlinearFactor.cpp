/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  A set of nonlinear factors indexed by a set of discrete keys.
 * @author Varun Agrawal
 * @date   Sep 12, 2024
 */

#include <gtsam/hybrid/HybridNonlinearFactor.h>

namespace gtsam {

/* *******************************************************************************/
HybridNonlinearFactor::HybridNonlinearFactor(const KeyVector& keys,
                                             const DiscreteKeys& discreteKeys,
                                             const Factors& factors)
    : Base(keys, discreteKeys), factors_(factors) {}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridNonlinearFactor::errorTree(
    const Values& continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc =
      [continuousValues](const std::pair<sharedFactor, double>& f) {
        auto [factor, val] = f;
        return factor->error(continuousValues) + val;
      };
  DecisionTree<Key, double> result(factors_, errorFunc);
  return result;
}

/* *******************************************************************************/
double HybridNonlinearFactor::error(
    const Values& continuousValues,
    const DiscreteValues& discreteValues) const {
  // Retrieve the factor corresponding to the assignment in discreteValues.
  auto [factor, val] = factors_(discreteValues);
  // Compute the error for the selected factor
  const double factorError = factor->error(continuousValues);
  return factorError + val;
}

/* *******************************************************************************/
double HybridNonlinearFactor::error(const HybridValues& values) const {
  return error(values.nonlinear(), values.discrete());
}

/* *******************************************************************************/
size_t HybridNonlinearFactor::dim() const {
  const auto assignments = DiscreteValues::CartesianProduct(discreteKeys_);
  auto [factor, val] = factors_(assignments.at(0));
  return factor->dim();
}

/* *******************************************************************************/
void HybridNonlinearFactor::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  Base::print("", keyFormatter);
  std::cout << "\nHybridNonlinearFactor\n";
  auto valueFormatter = [](const std::pair<sharedFactor, double>& v) {
    auto [factor, val] = v;
    if (factor) {
      return "Nonlinear factor on " + std::to_string(factor->size()) + " keys";
    } else {
      return std::string("nullptr");
    }
  };
  factors_.print("", keyFormatter, valueFormatter);
}

/* *******************************************************************************/
bool HybridNonlinearFactor::equals(const HybridFactor& other,
                                   double tol) const {
  // We attempt a dynamic cast from HybridFactor to HybridNonlinearFactor. If
  // it fails, return false.
  if (!dynamic_cast<const HybridNonlinearFactor*>(&other)) return false;

  // If the cast is successful, we'll properly construct a
  // HybridNonlinearFactor object from `other`
  const HybridNonlinearFactor& f(
      static_cast<const HybridNonlinearFactor&>(other));

  // Ensure that this HybridNonlinearFactor and `f` have the same `factors_`.
  auto compare = [tol](const std::pair<sharedFactor, double>& a,
                       const std::pair<sharedFactor, double>& b) {
    return traits<NonlinearFactor>::Equals(*a.first, *b.first, tol) &&
           (a.second == b.second);
  };
  if (!factors_.equals(f.factors_, compare)) return false;

  // If everything above passes, and the keys_ and discreteKeys_
  // member variables are identical, return true.
  return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
          (discreteKeys_ == f.discreteKeys_));
}

/* *******************************************************************************/
GaussianFactor::shared_ptr HybridNonlinearFactor::linearize(
    const Values& continuousValues,
    const DiscreteValues& discreteValues) const {
  auto factor = factors_(discreteValues).first;
  return factor->linearize(continuousValues);
}

/* *******************************************************************************/
std::shared_ptr<HybridGaussianFactor> HybridNonlinearFactor::linearize(
    const Values& continuousValues) const {
  // functional to linearize each factor in the decision tree
  auto linearizeDT =
      [continuousValues](
          const std::pair<sharedFactor, double>& f) -> GaussianFactorValuePair {
    auto [factor, val] = f;
    return {factor->linearize(continuousValues), val};
  };

  DecisionTree<Key, std::pair<GaussianFactor::shared_ptr, double>>
      linearized_factors(factors_, linearizeDT);

  return std::make_shared<HybridGaussianFactor>(continuousKeys_, discreteKeys_,
                                                linearized_factors);
}

}  // namespace gtsam