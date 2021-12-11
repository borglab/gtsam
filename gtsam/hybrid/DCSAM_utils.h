/* ----------------------------------------------------------------------------
 * Copyright 2020 The Ambitious Folks of the MRG
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *
 * @file   DCSAM_utils.h
 * @brief  Some utilities for DCSAM
 * @author Kevin Doherty, kdoherty@mit.edu
 * @date   December 2021
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <math.h>

#include <limits>
#include <string>
#include <vector>

namespace gtsam {

inline std::vector<double> expNormalize(const std::vector<double> &logProbs) {
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
