/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactor.cpp
 * @brief   Unit tests for a symbolic IndexFactor
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/IndexFactorOrdered.h>
#include <gtsam/inference/IndexConditionalOrdered.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>

#include <boost/assign/std/vector.hpp>
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactor, constructor) {

  // Frontals sorted, parents not sorted
  vector<Index> keys1; keys1 += 3, 4, 5, 9, 7, 8;
  (void)IndexConditionalOrdered(keys1, 3);

//  // Frontals not sorted
//  vector<Index> keys2; keys2 += 3, 5, 4, 9, 7, 8;
//  (void)IndexConditionalOrdered::FromRange(keys2.begin(), keys2.end(), 3);

//  // Frontals not before parents
//  vector<Index> keys3; keys3 += 3, 4, 5, 1, 7, 8;
//  (void)IndexConditionalOrdered::FromRange(keys3.begin(), keys3.end(), 3);
}

/* ************************************************************************* */
#ifdef TRACK_ELIMINATE
TEST(SymbolicFactor, eliminate) {
  vector<Index> keys; keys += 2, 3, 4, 6, 7, 9, 10, 11;
  IndexFactorOrdered actual(keys.begin(), keys.end());
  BayesNetOrdered<IndexConditionalOrdered> fragment = *actual.eliminate(3);

  IndexFactorOrdered expected(keys.begin()+3, keys.end());
  IndexConditionalOrdered::shared_ptr expected0 = IndexConditionalOrdered::FromRange(keys.begin(), keys.end(), 1);
  IndexConditionalOrdered::shared_ptr expected1 = IndexConditionalOrdered::FromRange(keys.begin()+1, keys.end(), 1);
  IndexConditionalOrdered::shared_ptr expected2 = IndexConditionalOrdered::FromRange(keys.begin()+2, keys.end(), 1);

  CHECK(assert_equal(fragment.size(), size_t(3)));
  CHECK(assert_equal(expected, actual));
  BayesNetOrdered<IndexConditionalOrdered>::const_iterator fragmentCond = fragment.begin();
  CHECK(assert_equal(**fragmentCond++, *expected0));
  CHECK(assert_equal(**fragmentCond++, *expected1));
  CHECK(assert_equal(**fragmentCond++, *expected2));
}
#endif
/* ************************************************************************* */
TEST(SymbolicFactor, EliminateSymbolic) {
  SymbolicFactorGraphOrdered factors;
  factors.push_factor(2,4,6);
  factors.push_factor(1,2,5);
  factors.push_factor(0,3);

  IndexFactorOrdered expectedFactor(4,5,6);
  std::vector<Index> keys; keys += 0,1,2,3,4,5,6;
  IndexConditionalOrdered::shared_ptr expectedConditional(new IndexConditionalOrdered(keys, 4));

  IndexFactorOrdered::shared_ptr actualFactor;
  IndexConditionalOrdered::shared_ptr actualConditional;
  boost::tie(actualConditional, actualFactor) = EliminateSymbolic(factors, 4);

  CHECK(assert_equal(*expectedConditional, *actualConditional));
  CHECK(assert_equal(expectedFactor, *actualFactor));

//  BayesNetOrdered<IndexConditionalOrdered> expected_bn;
//  vector<Index> parents;
//
//  parents.clear(); parents += 1,2,3,4,5,6;
//  expected_bn.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(0, parents)));
//
//  parents.clear(); parents += 2,3,4,5,6;
//  expected_bn.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(1, parents)));
//
//  parents.clear(); parents += 3,4,5,6;
//  expected_bn.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(2, parents)));
//
//  parents.clear(); parents += 4,5,6;
//  expected_bn.push_back(IndexConditionalOrdered::shared_ptr(new IndexConditionalOrdered(3, parents)));
//
//  BayesNetOrdered<IndexConditionalOrdered>::shared_ptr actual_bn;
//  IndexFactor::shared_ptr actual_factor;
//  boost::tie(actual_bn, actual_factor) = EliminateSymbolic(factors, 4);
//
//  CHECK(assert_equal(expected_bn, *actual_bn));
//  CHECK(assert_equal(expected_factor, *actual_factor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
