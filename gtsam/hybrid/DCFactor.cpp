/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DCFactor.cpp
 * @brief  Custom discrete-continuous factor
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/hybrid/DCFactor.h>

namespace gtsam {

/* ************************************************************************* */
DecisionTreeFactor DCFactor::toDecisionTreeFactor(
    const Values& continuousVals, const DiscreteValues& discreteVals) const {
  DecisionTreeFactor converted;
  for (const DiscreteKey& dkey : discreteKeys_) {
    std::vector<double> probs = evalProbs(dkey, continuousVals);
    // Cardinality of DiscreteKey is located at `second`
    assert(probs.size() == dkey.second);
    DecisionTreeFactor unary(dkey, probs);
    converted = converted * unary;
  }
  return converted;
}

/* ************************************************************************* */
std::vector<double> DCFactor::evalProbs(const DiscreteKey& dk,
                                        const Values& continuousVals) const {
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
  std::vector<double> logProbs;
  for (size_t i = 0; i < dk.second; i++) {
    DiscreteValues testDiscreteVals;
    testDiscreteVals[dk.first] = i;
    // Recall: `error` returns -log(prob), so we compute exp(-error) to
    // recover probability
    double logProb = -error(continuousVals, testDiscreteVals);
    logProbs.push_back(logProb);
  }
  return expNormalize(logProbs);
}

/* ************************************************************************* */
DecisionTreeFactor DCFactor::conditionalTimes(
    const DecisionTreeFactor& f, const Values& continuousVals,
    const DiscreteValues& discreteVals) const {
  return toDecisionTreeFactor(continuousVals, discreteVals) * f;
}

}  // namespace gtsam