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

#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/base/Vector.h>

#include <cmath>
#include <limits>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
std::vector<double> expNormalize(const std::vector<double>& log_probs) {
  double max_log_prob = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < log_probs.size(); i++) {
    double log_prob = log_probs[i];
    if ((log_prob != std::numeric_limits<double>::infinity()) &&
        log_prob > max_log_prob) {
      max_log_prob = log_prob;
    }
  }

  // After computing the max = "Z" of the log probabilities L_i, we compute
  // the log of the normalizing constant, log S, where S = sum_j exp(L_j - Z).
  double total = 0.0;
  for (size_t i = 0; i < log_probs.size(); i++) {
    double prob_prime = exp(log_probs[i] - max_log_prob);
    total += prob_prime;
  }
  double log_total = log(total);

  // Now we compute the (normalized) probability (for each i):
  // p_i = exp(L_i - Z - log S)
  double check_normalization = 0.0;
  std::vector<double> probs;
  for (size_t i = 0; i < log_probs.size(); i++) {
    double prob = exp(log_probs[i] - max_log_prob - log_total);
    probs.push_back(prob);
    check_normalization += prob;
  }

  // Numerical tolerance for floating point comparisons
  double tol = 1e-9;

  if (!fpEqual(check_normalization, 1.0, tol)) {
    std::string errMsg =
        std::string("expNormalize failed to normalize probabilities. ") +
        std::string("Expected normalization constant = 1.0. Got value: ") +
        std::to_string(check_normalization) +
        std::string(
            "\n This could have resulted from numerical overflow/underflow.");
    throw std::logic_error(errMsg);
  }
  return probs;
}

}  // namespace gtsam
