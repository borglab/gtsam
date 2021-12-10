/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/hybrid/HybridFactor.h>

namespace gtsam {

/* ************************************************************************* */
DecisionTreeFactor HybridFactor::toDecisionTreeFactor(
    const Values& continuous_values,
    const DiscreteValues& discrete_values) const {
  DecisionTreeFactor converted;
  for (const DiscreteKey& dkey : discrete_keys_) {
    std::vector<double> probs = evalProbs(dkey, continuous_values);
    // Cardinality of DiscreteKey is located at `second`
    assert(probs.size() == dkey.second);
    DecisionTreeFactor unary(dkey, probs);
    converted = converted * unary;
  }
  return converted;
}

/* ************************************************************************* */
std::vector<double> HybridFactor::evalProbs(
    const DiscreteKey& discrete_key, const Values& continuous_values) const {
  /*
   * Normalizing a set of log probabilities in a numerically stable way is
   * tricky. To avoid overflow/underflow issues, we compute the largest
   * (finite) log probability and subtract it from each log probability before
   * normalizing. This comes from the observation that if:
   *    p_i = exp(L_i) / ( sum_j exp(L_j) ),
   * Then,
   *    p_i = exp(Z) exp(L_i - Z) / (exp(Z) sum_j exp(L_j - Z)),
   *        = exp(L_i - Z) / ( sum_j exp(L_j - Z) )
   *
   * Setting Z = max_j L_j, we can avoid numerical issues that arise when all
   * of the (unnormalized) log probabilities are either very large or very
   * small.
   */
  std::vector<double> log_probs;
  for (size_t i = 0; i < discrete_key.second; i++) {
    DiscreteValues discrete_vals;
    discrete_vals[discrete_key.first] = i;
    // Recall: `error` returns -log(prob), so we compute exp(-error) to
    // recover probability
    double log_prob = -error(continuous_values, discrete_vals);
    log_probs.push_back(log_prob);
  }
  return expNormalize(log_probs);
}

/* ************************************************************************* */
DecisionTreeFactor HybridFactor::conditionalTimes(
    const DecisionTreeFactor& f, const Values& continuous_values,
    const DiscreteValues& discrete_values) const {
  return toDecisionTreeFactor(continuous_values, discrete_values) * f;
}

}  // namespace gtsam
