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

#include <CppUnitLite/Test.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/hybrid/GaussianMixture.h>
#include <gtsam/hybrid/GaussianMixtureFactor.h>
#include <gtsam/hybrid/HybridBayesNet.h>
#include <gtsam/hybrid/HybridBayesTree.h>
#include <gtsam/hybrid/HybridConditional.h>
#include <gtsam/hybrid/HybridDiscreteFactor.h>
#include <gtsam/hybrid/HybridFactor.h>
#include <gtsam/hybrid/HybridFactorGraph.h>
#include <gtsam/hybrid/HybridGaussianFactor.h>
#include <gtsam/inference/BayesNet.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/JacobianFactor.h>

#include <boost/assign/std/map.hpp>
#include <cstddef>

#include "gtsam/inference/DotWriter.h"
#include "gtsam/inference/Key.h"
#include "gtsam/inference/Ordering.h"

using namespace boost::assign;

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::C;
using gtsam::symbol_shorthand::X;

#define BOOST_STACKTRACE_GNU_SOURCE_NOT_REQUIRED

#include <signal.h>  // ::signal, ::raise

#include <boost/stacktrace.hpp>

void my_signal_handler(int signum) {
  ::signal(signum, SIG_DFL);
  std::cout << boost::stacktrace::stacktrace();
  ::raise(SIGABRT);
}

/* ************************************************************************* */
TEST_DISABLED(HybridFactorGraph, creation) {
  HybridConditional test;

  HybridFactorGraph hfg;

  hfg.add(HybridGaussianFactor(JacobianFactor(0, I_3x3, Z_3x1)));

  GaussianMixture clgc({X(0)}, {X(1)}, DiscreteKeys(DiscreteKey{C(0), 2}),
                       GaussianMixture::Conditionals(
                           C(0),
                           boost::make_shared<GaussianConditional>(
                               X(0), Z_3x1, I_3x3, X(1), I_3x3),
                           boost::make_shared<GaussianConditional>(
                               X(0), Vector3::Ones(), I_3x3, X(1), I_3x3)));
  GTSAM_PRINT(clgc);
}

TEST_DISABLED(HybridFactorGraph, eliminate) {
  HybridFactorGraph hfg;

  hfg.add(HybridGaussianFactor(JacobianFactor(0, I_3x3, Z_3x1)));

  auto result = hfg.eliminatePartialSequential(KeyVector{0});

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
}

TEST_DISABLED(HybridFactorGraph, eliminateMultifrontal) {
  HybridFactorGraph hfg;

  DiscreteKey c(C(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(c, {2, 8})));

  auto result = hfg.eliminatePartialMultifrontal({X(0)});

  EXPECT_LONGS_EQUAL(result.first->size(), 1);
  EXPECT_LONGS_EQUAL(result.second->size(), 1);
}

TEST_DISABLED(HybridFactorGraph, eliminateFullSequentialSimple) {
  std::cout << ">>>>>>>>>>>>>>\n";

  HybridFactorGraph hfg;

  DiscreteKey c1(C(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      C(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  hfg.add(GaussianMixtureFactor({X(1)}, {c1}, dt));
  // hfg.add(GaussianMixtureFactor({X(0)}, {c1}, dt));
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(c1, {2, 8})));
  hfg.add(HybridDiscreteFactor(
      DecisionTreeFactor({{C(1), 2}, {C(2), 2}}, "1 2 3 4")));
  // hfg.add(HybridDiscreteFactor(DecisionTreeFactor({{C(2), 2}, {C(3), 2}}, "1
  // 2 3 4"))); hfg.add(HybridDiscreteFactor(DecisionTreeFactor({{C(3), 2},
  // {C(1), 2}}, "1 2 2 1")));

  auto result = hfg.eliminateSequential(
      Ordering::ColamdConstrainedLast(hfg, {C(1), C(2)}));

  GTSAM_PRINT(*result);
}

TEST_DISABLED(HybridFactorGraph, eliminateFullMultifrontalSimple) {
  std::cout << ">>>>>>>>>>>>>>\n";

  HybridFactorGraph hfg;

  DiscreteKey c1(C(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  // DecisionTree<Key, GaussianFactor::shared_ptr> dt(
  //     C(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
  //     boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  // hfg.add(GaussianMixtureFactor({X(1)}, {c1}, dt));
  hfg.add(GaussianMixtureFactor::FromFactorList(
      {X(1)}, {{C(1), 2}},
      {boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
       boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones())}));

  // hfg.add(GaussianMixtureFactor({X(0)}, {c1}, dt));
  hfg.add(DecisionTreeFactor(c1, {2, 8}));
  hfg.add(DecisionTreeFactor({{C(1), 2}, {C(2), 2}}, "1 2 3 4"));
  // hfg.add(HybridDiscreteFactor(DecisionTreeFactor({{C(2), 2}, {C(3), 2}}, "1
  // 2 3 4"))); hfg.add(HybridDiscreteFactor(DecisionTreeFactor({{C(3), 2},
  // {C(1), 2}}, "1 2 2 1")));

  auto result = hfg.eliminateMultifrontal(
      Ordering::ColamdConstrainedLast(hfg, {C(1), C(2)}));

  GTSAM_PRINT(*result);
  GTSAM_PRINT(*result->marginalFactor(C(2)));
}

TEST_DISABLED(HybridFactorGraph, eliminateFullMultifrontalCLG) {
  std::cout << ">>>>>>>>>>>>>>\n";

  HybridFactorGraph hfg;

  DiscreteKey c(C(1), 2);

  hfg.add(JacobianFactor(X(0), I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));

  DecisionTree<Key, GaussianFactor::shared_ptr> dt(
      C(1), boost::make_shared<JacobianFactor>(X(1), I_3x3, Z_3x1),
      boost::make_shared<JacobianFactor>(X(1), I_3x3, Vector3::Ones()));

  hfg.add(GaussianMixtureFactor({X(1)}, {c}, dt));
  hfg.add(HybridDiscreteFactor(DecisionTreeFactor(c, {2, 8})));
  //  hfg.add(HybridDiscreteFactor(DecisionTreeFactor({{C(1), 2}, {C(2), 2}}, "1
  //  2 3 4")));

  auto ordering_full = Ordering::ColamdConstrainedLast(hfg, {C(1)});

  HybridBayesTree::shared_ptr hbt = hfg.eliminateMultifrontal(ordering_full);

  GTSAM_PRINT(*hbt);
  /*
  Explanation: the Junction tree will need to reeliminate to get to the marginal
  on X(1), which is not possible because it involves eliminating discrete before
  continuous. The solution to this, however, is in Murphy02. TLDR is that this
  is 1. expensive and 2. inexact. neverless it is doable. And I believe that we
  should do this.
  */
}

/**
 * This test is about how to assemble the Bayes Tree roots after we do partial
 * elimination
 */
TEST_DISABLED(HybridFactorGraph, eliminateFullMultifrontalTwoClique) {
  std::cout << ">>>>>>>>>>>>>>\n";

  HybridFactorGraph hfg;

  hfg.add(JacobianFactor(X(0), I_3x3, X(1), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(1), I_3x3, X(2), -I_3x3, Z_3x1));

  {
    // DecisionTree<Key, GaussianFactor::shared_ptr> dt(
    //     C(0), boost::make_shared<JacobianFactor>(X(0), I_3x3, Z_3x1),
    //     boost::make_shared<JacobianFactor>(X(0), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor::FromFactorList(
        {X(0)}, {{C(0), 2}},
        {boost::make_shared<JacobianFactor>(X(0), I_3x3, Z_3x1),
         boost::make_shared<JacobianFactor>(X(0), I_3x3, Vector3::Ones())}));

    DecisionTree<Key, GaussianFactor::shared_ptr> dt1(
        C(1), boost::make_shared<JacobianFactor>(X(2), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(2), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(2)}, {{C(1), 2}}, dt1));
  }

  // hfg.add(HybridDiscreteFactor(DecisionTreeFactor(c, {2, 8})));
  hfg.add(HybridDiscreteFactor(
      DecisionTreeFactor({{C(1), 2}, {C(2), 2}}, "1 2 3 4")));

  hfg.add(JacobianFactor(X(3), I_3x3, X(4), -I_3x3, Z_3x1));
  hfg.add(JacobianFactor(X(4), I_3x3, X(5), -I_3x3, Z_3x1));

  {
    DecisionTree<Key, GaussianFactor::shared_ptr> dt(
        C(3), boost::make_shared<JacobianFactor>(X(3), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(3), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(3)}, {{C(3), 2}}, dt));

    DecisionTree<Key, GaussianFactor::shared_ptr> dt1(
        C(2), boost::make_shared<JacobianFactor>(X(5), I_3x3, Z_3x1),
        boost::make_shared<JacobianFactor>(X(5), I_3x3, Vector3::Ones()));

    hfg.add(GaussianMixtureFactor({X(5)}, {{C(2), 2}}, dt1));
  }

  auto ordering_full =
      Ordering::ColamdConstrainedLast(hfg, {C(0), C(1), C(2), C(3)});

  GTSAM_PRINT(ordering_full);

  HybridBayesTree::shared_ptr hbt;
  HybridFactorGraph::shared_ptr remaining;
  std::tie(hbt, remaining) = hfg.eliminatePartialMultifrontal(ordering_full);

  GTSAM_PRINT(*hbt);

  GTSAM_PRINT(*remaining);

  hbt->dot(std::cout);
  /*
  Explanation: the Junction tree will need to reeliminate to get to the marginal
  on X(1), which is not possible because it involves eliminating discrete before
  continuous. The solution to this, however, is in Murphy02. TLDR is that this
  is 1. expensive and 2. inexact. neverless it is doable. And I believe that we
  should do this.
  */
}

HybridFactorGraph::shared_ptr makeSwitchingChain(size_t n) {
  HybridFactorGraph hfg;

  hfg.add(JacobianFactor(X(1), I_3x3, Z_3x1));

  // X(1) to X(n+1)
  for (size_t t = 1; t < n; t++) {
    hfg.add(GaussianMixtureFactor::FromFactorList(
        {X(t), X(t + 1)}, {{C(t), 2}},
        {boost::make_shared<JacobianFactor>(X(t), I_3x3, X(t + 1), I_3x3,
                                            Z_3x1),
         boost::make_shared<JacobianFactor>(X(t), I_3x3, X(t + 1), I_3x3,
                                            Vector3::Ones())}));
  }

  return boost::make_shared<HybridFactorGraph>(std::move(hfg));
}

// TODO(fan): make a graph like Varun's paper one
TEST(HybridFactorGraph, Switching) {
  auto hfg = makeSwitchingChain(9);

  // X(5) will be the center, X(1-4), X(6-9)
  // X(3), X(7)
  // X(2), X(8)
  // X(1), X(4), X(6), X(9)
  // C(5) will be the center, C(1-4), C(6-8)
  // C(3), C(7)
  // C(1), C(4), C(2), C(6), C(8)
  auto ordering_full =
      Ordering(KeyVector{X(1), X(4), X(2), X(6), X(9), X(8), X(3), X(7), X(5),
                         C(1), C(4), C(2), C(6), C(8), C(3), C(7), C(5)});

  GTSAM_PRINT(*hfg);
  GTSAM_PRINT(ordering_full);

  HybridBayesTree::shared_ptr hbt;
  HybridFactorGraph::shared_ptr remaining;
  std::tie(hbt, remaining) = hfg->eliminatePartialMultifrontal(ordering_full);

  GTSAM_PRINT(*hbt);

  GTSAM_PRINT(*remaining);

  {
    DotWriter dw;
    dw.positionHints['c'] = 2;
    dw.positionHints['x'] = 1;
    std::cout << hfg->dot(DefaultKeyFormatter, dw);
    std::cout << "\n";
    hbt->dot(std::cout);
  }

  {
    DotWriter dw;
    dw.positionHints['x'] = 1;
    std::cout << "\n";
    std::cout << hfg->eliminateSequential(ordering_full)
                     ->dot(DefaultKeyFormatter, dw);
  }
  /*
  Explanation: the Junction tree will need to reeliminate to get to the marginal
  on X(1), which is not possible because it involves eliminating discrete before
  continuous. The solution to this, however, is in Murphy02. TLDR is that this
  is 1. expensive and 2. inexact. neverless it is doable. And I believe that we
  should do this.
  */
}

/* ************************************************************************* */
int main() {
  ::signal(SIGSEGV, &my_signal_handler);
  ::signal(SIGBUS, &my_signal_handler);

  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
