/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 *  @file testHybridConditional.cpp
 *  @date Mar 11, 2022
 *  @author Fan Jiang
 */

#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>

#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/inference/BayesNet.h>

#include <gtsam/inference/Symbol.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::X;

/* ************************************************************************* */
TEST_UNSAFE(HybridFactorGraph, test) {
  HybridConditional test;
  GTSAM_PRINT(test);

  HybridFactorGraph hfg;

  hfg.add(HybridGaussianFactor(JacobianFactor(0, I_3x3, Z_3x1)));
  GTSAM_PRINT(hfg);
}

TEST_UNSAFE(HybridFactorGraph, eliminate) {
  HybridFactorGraph hfg;

  hfg.add(HybridGaussianFactor(JacobianFactor(0, I_3x3, Z_3x1)));

  auto result = hfg.eliminatePartialSequential({0});

  GTSAM_PRINT(*result.first);
}

TEST(HybridFactorGraph, eliminateMultifrontal) {
  HybridFactorGraph hfg;

  DiscreteKey x(X(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(x, {2, 8})));

  auto result = hfg.eliminatePartialMultifrontal({X(0)});

  GTSAM_PRINT(*result.first);
  GTSAM_PRINT(*result.second);
}

TEST(HybridFactorGraph, eliminateFullMultifrontal) {
  HybridFactorGraph hfg;

  DiscreteKey x(X(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(x, {2, 8})));

  auto result = hfg.eliminateMultifrontal();

  GTSAM_PRINT(*result);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

