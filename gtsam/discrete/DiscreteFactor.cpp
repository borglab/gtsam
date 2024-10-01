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
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/hybrid/HybridValues.h>

#include <cmath>
#include <sstream>

using namespace std;

namespace gtsam {

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

/* ************************************************************************* */
std::vector<double> expNormalize(const std::vector<double>& logProbs) {
  double maxLogProb = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < logProbs.size(); i++) {
    double logProb = logProbs[i];
    if ((logProb != std::numeric_limits<double>::infinity()) &&
        logProb > maxLogProb) {
      maxLogProb = logProb;
    }
  }

  // After computing the max = "Z" of the log probabilities L_i, we compute
  // the log of the normalizing constant, log S, where S = sum_j exp(L_j - Z).
  double total = 0.0;
  for (size_t i = 0; i < logProbs.size(); i++) {
    double probPrime = exp(logProbs[i] - maxLogProb);
    total += probPrime;
  }
  double logTotal = log(total);

  // Now we compute the (normalized) probability (for each i):
  // p_i = exp(L_i - Z - log S)
  double checkNormalization = 0.0;
  std::vector<double> probs;
  for (size_t i = 0; i < logProbs.size(); i++) {
    double prob = exp(logProbs[i] - maxLogProb - logTotal);
    probs.push_back(prob);
    checkNormalization += prob;
  }

  // Numerical tolerance for floating point comparisons
  double tol = 1e-9;

  if (!gtsam::fpEqual(checkNormalization, 1.0, tol)) {
    std::string errMsg =
        std::string("expNormalize failed to normalize probabilities. ") +
        std::string("Expected normalization constant = 1.0. Got value: ") +
        std::to_string(checkNormalization) +
        std::string(
            "\n This could have resulted from numerical overflow/underflow.");
    throw std::logic_error(errMsg);
  }
  return probs;
}

}  // namespace gtsam
