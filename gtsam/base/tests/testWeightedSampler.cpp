/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testWeightedSampler.cpp
 * @brief   Unit test for WeightedSampler
 * @author  Frank Dellaert
 * @date    MAy 2019
 **/

#include <vector>

namespace gtsam {
template <class Engine>
std::vector<size_t> sampleWithoutReplacement(Engine& rng, size_t s,
                                             std::vector<double> weights);
}

#include <CppUnitLite/TestHarness.h>

#include <boost/random/mersenne_twister.hpp>

using namespace std;
using namespace gtsam;

TEST(WeightedSampler, sampleWithoutReplacement) {
  vector<double> weights{1, 2, 3, 4, 3, 2, 1};
  boost::mt19937 rng(42);
  auto samples = sampleWithoutReplacement(rng, 5, weights);
  EXPECT_LONGS_EQUAL(5, samples.size());
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

namespace gtsam {
/* ************************************************************************* */
/* Implementation adapted from paper at
 * https://www.ethz.ch/content/dam/ethz/special-interest/baug/ivt/ivt-dam/vpl/reports/1101-1200/ab1141.pdf
 */
template <class Engine>
vector<size_t> sampleWithoutReplacement(Engine& rng, size_t s,
                                        vector<double> weights) {
  const size_t n = weights.size();
  if (n < s) {
    throw runtime_error("s must be smaller than weights.size()");
  }

  // Return empty array if s==0
  vector<size_t> result(s);
  if (s == 0) return result;

  // Step 1: The first m items of V are inserted into reservoir
  // Step 2: For each item v_i ∈ reservoir: Calculate a key k_i = u_i^(1/w),
  // where u_i = random(0, 1)
  // (Modification: Calculate and store -log k_i = e_i / w where e_i = exp(1),
  //  reservoir is a priority queue that pops the *maximum* elements)
  priority_queue<pair<double, size_t> > reservoir;

  static const double kexp1 = exp(1.0);
  for (auto iprob = weights.begin(); iprob != weights.begin() + s; ++iprob) {
    double k_i = kexp1 / *iprob;
    reservoir.push(make_pair(k_i, iprob - weights.begin() + 1));
  }

  // Step 4: Repeat Steps 5–10 until the population is exhausted
  {
    // Step 3: The threshold T_w is the minimum key of reservoir
    // (Modification: This is now the logarithm)
    // Step 10: The new threshold T w is the new minimum key of reservoir
    const pair<double, size_t>& T_w = reservoir.top();

    // Incrementing iprob is part of Step 7
    for (auto iprob = weights.begin() + s; iprob != weights.end(); ++iprob) {
      // Step 5: Let r = random(0, 1) and X_w = log(r) / log(T_w)
      // (Modification: Use e = -exp(1) instead of log(r))
      double X_w = kexp1 / T_w.first;

      // Step 6: From the current item v_c skip items until item v_i, such that:
      double w = 0.0;

      // Step 7: w_c + w_{c+1} + ··· + w_{i−1} < X_w <= w_c + w_{c+1} + ··· +
      // w_{i−1} + w_i
      for (; iprob != weights.end(); ++iprob) {
        w += *iprob;
        if (X_w <= w) break;
      }

      // Step 7: No such item, terminate
      if (iprob == weights.end()) break;

      // Step 9: Let t_w = T_w^{w_i}, r_2 = random(t_w, 1) and v_i’s key: k_i =
      // (r_2)^{1/w_i} (Mod: Let t_w = log(T_w) * {w_i}, e_2 =
      // log(random(e^{t_w}, 1)) and v_i’s key: k_i = -e_2 / w_i)
      double t_w = -T_w.first * *iprob;
      boost::uniform_real<double> randomAngle(exp(t_w), 1.0);
      double e_2 = log(randomAngle(rng));
      double k_i = -e_2 / *iprob;

      // Step 8: The item in reservoir with the minimum key is replaced by item
      // v_i
      reservoir.pop();
      reservoir.push(make_pair(k_i, iprob - weights.begin() + 1));
    }
  }

  for (auto iret = result.end(); iret != result.begin();) {
    --iret;

    if (reservoir.empty()) {
      throw runtime_error(
          "Reservoir empty before all elements have been filled");
    }

    *iret = reservoir.top().second;
    reservoir.pop();
  }

  if (!reservoir.empty()) {
    throw runtime_error(
        "Reservoir not empty after all elements have been filled");
  }

  return result;
}
}  // namespace gtsam
