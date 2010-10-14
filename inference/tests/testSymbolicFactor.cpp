/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicFactor.cpp
 * @brief   Unit tests for a symbolic Factor
 * @author  Frank Dellaert
 */

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/inference/Factor-inl.h>
#include <gtsam/inference/Conditional.h>

#include <boost/assign/std/vector.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(SymbolicFactor, eliminate) {
  vector<Index> keys; keys += 2, 3, 4, 6, 7, 9, 10, 11;
  Factor actual(keys.begin(), keys.end());
  BayesNet<Conditional> fragment = *actual.eliminate(3);

  Factor expected(keys.begin()+3, keys.end());
  Conditional::shared_ptr expected0 = Conditional::FromRange(keys.begin(), keys.end(), 1);
  Conditional::shared_ptr expected1 = Conditional::FromRange(keys.begin()+1, keys.end(), 1);
  Conditional::shared_ptr expected2 = Conditional::FromRange(keys.begin()+2, keys.end(), 1);

  CHECK(assert_equal(fragment.size(), size_t(3)));
  CHECK(assert_equal(expected, actual));
  BayesNet<Conditional>::const_iterator fragmentCond = fragment.begin();
  CHECK(assert_equal(**fragmentCond++, *expected0));
  CHECK(assert_equal(**fragmentCond++, *expected1));
  CHECK(assert_equal(**fragmentCond++, *expected2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
