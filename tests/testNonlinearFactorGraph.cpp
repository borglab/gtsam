/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/** 
 * @file    testNonlinearFactorGraph.cpp
 * @brief   Unit tests for Non-Linear Factor NonlinearFactorGraph
 * @brief   testNonlinearFactorGraph
 * @author  Carlos Nieto
 * @author  Christian Potthast
 */

/*STL/C++*/
#include <iostream>
using namespace std;

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <tests/smallExample.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( NonlinearFactorGraph, equals )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  NonlinearFactorGraph fg2 = createNonlinearFactorGraph();
  CHECK( fg.equals(fg2) );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, error )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values c1 = createValues();
  double actual1 = fg.error(c1);
  DOUBLES_EQUAL( 0.0, actual1, 1e-9 );

  Values c2 = createNoisyValues();
  double actual2 = fg.error(c2);
  DOUBLES_EQUAL( 5.625, actual2, 1e-9 );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, keys )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  KeySet actual = fg.keys();
  LONGS_EQUAL(3, (long)actual.size());
  KeySet::const_iterator it = actual.begin();
  LONGS_EQUAL((long)L(1), (long)*(it++));
  LONGS_EQUAL((long)X(1), (long)*(it++));
  LONGS_EQUAL((long)X(2), (long)*(it++));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, GET_ORDERING)
{
  Ordering expected; expected += L(1), X(2), X(1); // For starting with l1,x1,x2
  NonlinearFactorGraph nlfg = createNonlinearFactorGraph();
  Ordering actual = Ordering::Colamd(nlfg);
  EXPECT(assert_equal(expected,actual));

  // Constrained ordering - put x2 at the end
  Ordering expectedConstrained; expectedConstrained += L(1), X(1), X(2);
  FastMap<Key, int> constraints;
  constraints[X(2)] = 1;
  Ordering actualConstrained = Ordering::ColamdConstrained(nlfg, constraints);
  EXPECT(assert_equal(expectedConstrained, actualConstrained));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, probPrime )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values cfg = createValues();

  // evaluate the probability of the factor graph
  double actual = fg.probPrime(cfg);
  double expected = 1.0;
  DOUBLES_EQUAL(expected,actual,0);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, linearize )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values initial = createNoisyValues();
  GaussianFactorGraph linearized = *fg.linearize(initial);
  GaussianFactorGraph expected = createGaussianFactorGraph();
  CHECK(assert_equal(expected,linearized)); // Needs correct linearizations
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, clone )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  NonlinearFactorGraph actClone = fg.clone();
  EXPECT(assert_equal(fg, actClone));
  for (size_t i=0; i<fg.size(); ++i)
    EXPECT(fg[i] != actClone[i]);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, rekey )
{
  NonlinearFactorGraph init = createNonlinearFactorGraph();
  map<Key,Key> rekey_mapping;
  rekey_mapping.insert(make_pair(L(1), L(4)));
  NonlinearFactorGraph actRekey = init.rekey(rekey_mapping);

  // ensure deep clone
  LONGS_EQUAL((long)init.size(), (long)actRekey.size());
  for (size_t i=0; i<init.size(); ++i)
      EXPECT(init[i] != actRekey[i]);

  NonlinearFactorGraph expRekey;
  // original measurements
  expRekey.push_back(init[0]);
  expRekey.push_back(init[1]);

  // updated measurements
  Point2 z3(0, -1),  z4(-1.5, -1.);
  SharedDiagonal sigma0_2 = noiseModel::Isotropic::Sigma(2,0.2);
  expRekey += simulated2D::Measurement(z3, sigma0_2, X(1), L(4));
  expRekey += simulated2D::Measurement(z4, sigma0_2, X(2), L(4));

  EXPECT(assert_equal(expRekey, actRekey));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, symbolic )
{
  NonlinearFactorGraph graph = createNonlinearFactorGraph();

  SymbolicFactorGraph expected;
  expected.push_factor(X(1));
  expected.push_factor(X(1), X(2));
  expected.push_factor(X(1), L(1));
  expected.push_factor(X(2), L(1));

  SymbolicFactorGraph actual = *graph.symbolic();

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
