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
#include <gtsam/symbolic/SymbolicFactor.h>
#include <gtsam/symbolic/SymbolicConditional.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>

#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
#ifdef TRACK_ELIMINATE
TEST(SymbolicFactor, eliminate) {
  KeyVector keys {2, 3, 4, 6, 7, 9, 10, 11};
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
TEST(SymbolicFactor, Constructors)
{
  SymbolicFactor expected(3, 4);

  SymbolicFactor actual1 = SymbolicFactor::FromKeys(expected.keys());
  SymbolicFactor actual2 = SymbolicFactor::FromIterators(expected.begin(), expected.end());
  SymbolicFactor actual3 = *SymbolicFactor::FromKeysShared(expected.keys());
  SymbolicFactor actual4 = *SymbolicFactor::FromIteratorsShared(expected.begin(), expected.end());

  EXPECT(assert_equal(expected, actual1));
  EXPECT(assert_equal(expected, actual2));
  EXPECT(assert_equal(expected, actual3));
  EXPECT(assert_equal(expected, actual4));
}

/* ************************************************************************* */
TEST(SymbolicFactor, EliminateSymbolic)
{
  const SymbolicFactorGraph factors = {
      std::make_shared<SymbolicFactor>(2, 4, 6),
      std::make_shared<SymbolicFactor>(1, 2, 5),
      std::make_shared<SymbolicFactor>(0, 3)};

  const SymbolicFactor expectedFactor(4,5,6);
  const SymbolicConditional expectedConditional =
    SymbolicConditional::FromKeys(KeyVector{0,1,2,3,4,5,6}, 4);

  SymbolicFactor::shared_ptr actualFactor;
  SymbolicConditional::shared_ptr actualConditional;
  boost::tie(actualConditional, actualFactor) =
      EliminateSymbolic(factors, Ordering{0, 1, 2, 3});

  CHECK(assert_equal(expectedConditional, *actualConditional));
  CHECK(assert_equal(expectedFactor, *actualFactor));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
