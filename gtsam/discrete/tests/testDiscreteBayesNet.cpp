/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteBayesNet.cpp
 *
 *  @date Feb 27, 2011
 *  @author Frank Dellaert
 */

#include <gtsam/discrete/DiscreteBayesNet.h>
#include <gtsam/discrete/DiscreteFactorGraph.h>
#include <gtsam/discrete/DiscreteMarginals.h>
#include <gtsam/base/debug.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Vector.h>

#include <CppUnitLite/TestHarness.h>


#include <boost/assign/list_inserter.hpp>
#include <boost/assign/std/map.hpp>

using namespace boost::assign;

#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DiscreteBayesNet, bayesNet) {
  DiscreteBayesNet bayesNet;
  DiscreteKey Parent(0, 2), Child(1, 2);

  auto prior = boost::make_shared<DiscreteConditional>(Parent % "6/4");
  CHECK(assert_equal(Potentials::ADT({Parent}, "0.6 0.4"),
                     (Potentials::ADT)*prior));
  bayesNet.push_back(prior);

  auto conditional =
      boost::make_shared<DiscreteConditional>(Child | Parent = "7/3 8/2");
  EXPECT_LONGS_EQUAL(1, *(conditional->beginFrontals()));
  Potentials::ADT expected(Child & Parent, "0.7 0.8 0.3 0.2");
  CHECK(assert_equal(expected, (Potentials::ADT)*conditional));
  bayesNet.push_back(conditional);

  DiscreteFactorGraph fg(bayesNet);
  LONGS_EQUAL(2, fg.back()->size());

  // Check the marginals
  const double expectedMarginal[2]{0.4, 0.6 * 0.3 + 0.4 * 0.2};
  DiscreteMarginals marginals(fg);
  for (size_t j = 0; j < 2; j++) {
    Vector FT = marginals.marginalProbabilities(DiscreteKey(j, 2));
    EXPECT_DOUBLES_EQUAL(expectedMarginal[j], FT[1], 1e-3);
    EXPECT_DOUBLES_EQUAL(FT[0], 1.0 - FT[1], 1e-9);
  }
}

/* ************************************************************************* */
TEST(DiscreteBayesNet, Asia) {
  DiscreteBayesNet asia;
  DiscreteKey Asia(0, 2), Smoking(4, 2), Tuberculosis(3, 2), LungCancer(6, 2),
      Bronchitis(7, 2), Either(5, 2), XRay(2, 2), Dyspnea(1, 2);

  asia.add(Asia % "99/1");
  asia.add(Smoking % "50/50");

  asia.add(Tuberculosis | Asia = "99/1 95/5");
  asia.add(LungCancer | Smoking = "99/1 90/10");
  asia.add(Bronchitis | Smoking = "70/30 40/60");

  asia.add((Either | Tuberculosis, LungCancer) = "F T T T");

  asia.add(XRay | Either = "95/5 2/98");
  asia.add((Dyspnea | Either, Bronchitis) = "9/1 2/8 3/7 1/9");

  // Convert to factor graph
  DiscreteFactorGraph fg(asia);
  LONGS_EQUAL(3, fg.back()->size());

  // Check the marginals we know (of the parent-less nodes)
  DiscreteMarginals marginals(fg);
  Vector2 va(0.99, 0.01), vs(0.5, 0.5);
  EXPECT(assert_equal(va, marginals.marginalProbabilities(Asia)));
  EXPECT(assert_equal(vs, marginals.marginalProbabilities(Smoking)));

  // Create solver and eliminate
  Ordering ordering;
  ordering += Key(0), Key(1), Key(2), Key(3), Key(4), Key(5), Key(6), Key(7);
  DiscreteBayesNet::shared_ptr chordal = fg.eliminateSequential(ordering);
  DiscreteConditional expected2(Bronchitis % "11/9");
  EXPECT(assert_equal(expected2, *chordal->back()));

  // solve
  DiscreteFactor::sharedValues actualMPE = chordal->optimize();
  DiscreteFactor::Values expectedMPE;
  insert(expectedMPE)(Asia.first, 0)(Dyspnea.first, 0)(XRay.first, 0)(
      Tuberculosis.first, 0)(Smoking.first, 0)(Either.first, 0)(
      LungCancer.first, 0)(Bronchitis.first, 0);
  EXPECT(assert_equal(expectedMPE, *actualMPE));

  // add evidence, we were in Asia and we have dyspnea
  fg.add(Asia, "0 1");
  fg.add(Dyspnea, "0 1");

  // solve again, now with evidence
  DiscreteBayesNet::shared_ptr chordal2 = fg.eliminateSequential(ordering);
  DiscreteFactor::sharedValues actualMPE2 = chordal2->optimize();
  DiscreteFactor::Values expectedMPE2;
  insert(expectedMPE2)(Asia.first, 1)(Dyspnea.first, 1)(XRay.first, 0)(
      Tuberculosis.first, 0)(Smoking.first, 1)(Either.first, 0)(
      LungCancer.first, 0)(Bronchitis.first, 1);
  EXPECT(assert_equal(expectedMPE2, *actualMPE2));

  // now sample from it
  DiscreteFactor::Values expectedSample;
  SETDEBUG("DiscreteConditional::sample", false);
  insert(expectedSample)(Asia.first, 1)(Dyspnea.first, 1)(XRay.first, 1)(
      Tuberculosis.first, 0)(Smoking.first, 1)(Either.first, 1)(
      LungCancer.first, 1)(Bronchitis.first, 0);
  DiscreteFactor::sharedValues actualSample = chordal2->sample();
  EXPECT(assert_equal(expectedSample, *actualSample));
}

/* ************************************************************************* */
TEST_UNSAFE(DiscreteBayesNet, Sugar) {
  DiscreteKey T(0, 2), L(1, 2), E(2, 2), C(8, 3), S(7, 2);

  DiscreteBayesNet bn;

  // try logic
  bn.add((E | T, L) = "OR");
  bn.add((E | T, L) = "AND");

  // try multivalued
  bn.add(C % "1/1/2");
  bn.add(C | S = "1/1/2 5/2/3");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
