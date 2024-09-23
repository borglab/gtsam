/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    WeightedSampler.h
 * @brief   Fast sampling without replacement.
 * @author  Frank Dellaert
 * @date    May 2019
 **/

#pragma once

#include <cmath>
#include <queue>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {
/*
 * Fast sampling without replacement.
 * Example usage:
 *   std::mt19937 rng(42);
 *   WeightedSampler<std::mt19937> sampler(&rng);
 *   auto samples = sampler.sampleWithoutReplacement(5, weights);
 */
template <class Engine = std::mt19937>
class WeightedSampler {
 private:
  Engine* engine_;  // random number generation engine

 public:
  /**
   * Construct from random number generation engine
   * We only store a pointer to it.
   */
  explicit WeightedSampler(Engine* engine) : engine_(engine) {}

  std::vector<size_t> sampleWithoutReplacement(
      size_t numSamples, const std::vector<double>& weights) {
    // Implementation adapted from code accompanying paper at
    // https://www.ethz.ch/content/dam/ethz/special-interest/baug/ivt/ivt-dam/vpl/reports/1101-1200/ab1141.pdf
    const size_t n = weights.size();
    if (n < numSamples) {
      throw std::runtime_error(
          "numSamples must be smaller than weights.size()");
    }

    // Return empty array if numSamples==0
    std::vector<size_t> result(numSamples);
    if (numSamples == 0) return result;

    // Step 1: The first m items of V are inserted into reservoir
    // Step 2: For each item v_i ∈ reservoir: Calculate a key k_i = u_i^(1/w),
    // where u_i = random(0, 1)
    // (Modification: Calculate and store -log k_i = e_i / w where e_i = exp(1),
    //  reservoir is a priority queue that pops the *maximum* elements)
    std::priority_queue<std::pair<double, size_t> > reservoir;

    static const double kexp1 = std::exp(1.0);
    for (auto it = weights.begin(); it != weights.begin() + numSamples; ++it) {
      const double k_i = kexp1 / *it;
      reservoir.push({k_i, it - weights.begin() + 1});
    }

    // Step 4: Repeat Steps 5–10 until the population is exhausted
    {
      // Step 3: The threshold T_w is the minimum key of reservoir
      // (Modification: This is now the logarithm)
      // Step 10: The new threshold T w is the new minimum key of reservoir
      const std::pair<double, size_t>& T_w = reservoir.top();

      // Incrementing it is part of Step 7
      for (auto it = weights.begin() + numSamples; it != weights.end(); ++it) {
        // Step 5: Let r = random(0, 1) and X_w = log(r) / log(T_w)
        // (Modification: Use e = -exp(1) instead of log(r))
        const double X_w = kexp1 / T_w.first;

        // Step 6: From the current item v_c skip items until item v_i, such
        // that:
        double w = 0.0;

        // Step 7: w_c + w_{c+1} + ··· + w_{i−1} < X_w <= w_c + w_{c+1} + ··· +
        // w_{i−1} + w_i
        for (; it != weights.end(); ++it) {
          w += *it;
          if (X_w <= w) break;
        }

        // Step 7: No such item, terminate
        if (it == weights.end()) break;

        // Step 9: Let t_w = T_w^{w_i}, r_2 = random(t_w, 1) and v_i’s key: k_i
        // = (r_2)^{1/w_i} (Mod: Let t_w = log(T_w) * {w_i}, e_2 =
        // log(random(e^{t_w}, 1)) and v_i’s key: k_i = -e_2 / w_i)
        const double t_w = -T_w.first * *it;
        std::uniform_real_distribution<double> randomAngle(std::exp(t_w), 1.0);
        const double e_2 = std::log(randomAngle(*engine_));
        const double k_i = -e_2 / *it;

        // Step 8: The item in reservoir with the minimum key is replaced by
        // item v_i
        reservoir.pop();
        reservoir.push({k_i, it - weights.begin() + 1});
      }
    }

    for (auto iret = result.end(); iret != result.begin();) {
      --iret;

      if (reservoir.empty()) {
        throw std::runtime_error(
            "Reservoir empty before all elements have been filled");
      }

      *iret = reservoir.top().second - 1;
      reservoir.pop();
    }

    if (!reservoir.empty()) {
      throw std::runtime_error(
          "Reservoir not empty after all elements have been filled");
    }

    return result;
  }
};  // namespace gtsam
}  // namespace gtsam
