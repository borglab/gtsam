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

#include <gtsam/base/WeightedSampler.h>

#include <CppUnitLite/TestHarness.h>

#include <random>

using namespace std;
using namespace gtsam;

TEST(WeightedSampler, sampleWithoutReplacement) {
  vector<double> weights{1, 2, 3, 4, 3, 2, 1};
  std::mt19937 rng(42);
  WeightedSampler<std::mt19937> sampler(&rng);
  auto samples = sampler.sampleWithoutReplacement(5, weights);
  EXPECT_LONGS_EQUAL(5, samples.size());
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
