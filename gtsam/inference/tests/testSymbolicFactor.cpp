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
#include <gtsam/inference/IndexFactor.h>
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/SymbolicFactorGraph.h>

#include <boost/assign/std/vector.hpp>
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactor, constructor) {

  // Frontals sorted, parents not sorted
  vector<Index> keys1; keys1 += 3, 4, 5, 9, 7, 8;
  (void)IndexConditional(keys1, 3);

//  // Frontals not sorted
//  vector<Index> keys2; keys2 += 3, 5, 4, 9, 7, 8;
//  (void)IndexConditional::FromRange(keys2.begin(), keys2.end(), 3);

//  // Frontals not before parents
//  vector<Index> keys3; keys3 += 3, 4, 5, 1, 7, 8;
//  (void)IndexConditional::FromRange(keys3.begin(), keys3.end(), 3);
}

/* ************************************************************************* */
#ifdef TRACK_ELIMINATE
TEST(SymbolicFactor, eliminate) {
  vector<Index> keys; keys += 2, 3, 4, 6, 7, 9, 10, 11;
  IndexFactor actual(keys.begin(), keys.end());
  BayesNet<IndexConditional> fragment = *actual.eliminate(3);

  IndexFactor expected(keys.begin()+3, keys.end());
  IndexConditional::shared_ptr expected0 = IndexConditional::FromRange(keys.begin(), keys.end(), 1);
  IndexConditional::shared_ptr expected1 = IndexConditional::FromRange(keys.begin()+1, keys.end(), 1);
  IndexConditional::shared_ptr expected2 = IndexConditional::FromRange(keys.begin()+2, keys.end(), 1);

  CHECK(assert_equal(fragment.size(), size_t(3)));
  CHECK(assert_equal(expected, actual));
  BayesNet<IndexConditional>::const_iterator fragmentCond = fragment.begin();
  CHECK(assert_equal(**fragmentCond++, *expected0));
  CHECK(assert_equal(**fragmentCond++, *expected1));
  CHECK(assert_equal(**fragmentCond++, *expected2));
}
#endif
/* ************************************************************************* */
TEST(SymbolicFactor, EliminateSymbolic) {
  SymbolicFactorGraph factors;
  factors.push_factor(2,4,6);
  factors.push_factor(1,2,5);
  factors.push_factor(0,3);

  IndexFactor expectedFactor(4,5,6);
  std::vector<Index> keys; keys += 0,1,2,3,4,5,6;
  IndexConditional::shared_ptr expectedConditional(new IndexConditional(keys, 4));

  IndexFactor::shared_ptr actualFactor;
  IndexConditional::shared_ptr actualConditional;
  boost::tie(actualConditional, actualFactor) = EliminateSymbolic(factors, 4);

  CHECK(assert_equal(*expectedConditional, *actualConditional));
  CHECK(assert_equal(expectedFactor, *actualFactor));

//  BayesNet<IndexConditional> expected_bn;
//  vector<Index> parents;
//
//  parents.clear(); parents += 1,2,3,4,5,6;
//  expected_bn.push_back(IndexConditional::shared_ptr(new IndexConditional(0, parents)));
//
//  parents.clear(); parents += 2,3,4,5,6;
//  expected_bn.push_back(IndexConditional::shared_ptr(new IndexConditional(1, parents)));
//
//  parents.clear(); parents += 3,4,5,6;
//  expected_bn.push_back(IndexConditional::shared_ptr(new IndexConditional(2, parents)));
//
//  parents.clear(); parents += 4,5,6;
//  expected_bn.push_back(IndexConditional::shared_ptr(new IndexConditional(3, parents)));
//
//  BayesNet<IndexConditional>::shared_ptr actual_bn;
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
