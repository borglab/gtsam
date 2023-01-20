/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testSymbolicConditional.cpp
 * @brief   Unit tests for SymbolicConditional class
 * @author  Frank Dellaert
 */

#include <boost/make_shared.hpp>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/symbolic/SymbolicConditional.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicConditional, empty )
{
  SymbolicConditional c0;
  LONGS_EQUAL(0, (long)c0.nrFrontals());
  LONGS_EQUAL(0, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, noParents )
{
  SymbolicConditional c0(0);
  LONGS_EQUAL(1, (long)c0.nrFrontals());
  LONGS_EQUAL(0, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, oneParents )
{
  SymbolicConditional c0(0,1);
  LONGS_EQUAL(1, (long)c0.nrFrontals());
  LONGS_EQUAL(1, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, twoParents )
{
  SymbolicConditional c0(0,1,2);
  LONGS_EQUAL(1, (long)c0.nrFrontals());
  LONGS_EQUAL(2, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, threeParents )
{
  SymbolicConditional c0(0,1,2,3);
  LONGS_EQUAL(1, (long)c0.nrFrontals());
  LONGS_EQUAL(3, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, fourParents )
{
  auto c0 = SymbolicConditional::FromKeys(KeyVector{0, 1, 2, 3, 4}, 1);
  LONGS_EQUAL(1, (long)c0.nrFrontals());
  LONGS_EQUAL(4, (long)c0.nrParents());
}

/* ************************************************************************* */
TEST( SymbolicConditional, FromRange )
{
  auto c0 = boost::make_shared<SymbolicConditional>(
      SymbolicConditional::FromKeys(KeyVector{1, 2, 3, 4, 5}, 2));
  LONGS_EQUAL(2, (long)c0->nrFrontals());
  LONGS_EQUAL(3, (long)c0->nrParents());
}

/* ************************************************************************* */
TEST(SymbolicConditional, Constructors)
{
  SymbolicConditional expected(3, 4);

  SymbolicConditional actual1 = SymbolicConditional::FromKeys(expected.keys(), 1);
  SymbolicConditional actual2 = SymbolicConditional::FromIterators(expected.begin(), expected.end(), 1);
  SymbolicConditional actual3 = *SymbolicConditional::FromKeysShared(expected.keys(), 1);
  SymbolicConditional actual4 = *SymbolicConditional::FromIteratorsShared(expected.begin(), expected.end(), 1);

  EXPECT(assert_equal(expected, actual1));
  EXPECT(assert_equal(expected, actual2));
  EXPECT(assert_equal(expected, actual3));
  EXPECT(assert_equal(expected, actual4));
}

/* ************************************************************************* */
TEST( SymbolicConditional, equals )
{
  SymbolicConditional c0(0, 1, 2), c1(0, 1, 2), c2(1, 2, 3), c3(3,4);
  CHECK(c0.equals(c1));
  CHECK(c1.equals(c0));
  CHECK(!c0.equals(c2));
  CHECK(!c2.equals(c0));
  CHECK(!c0.equals(c3));
  CHECK(!c3.equals(c0));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
