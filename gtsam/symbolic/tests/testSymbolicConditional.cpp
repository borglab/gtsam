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

#include <boost/assign/list_of.hpp>
using namespace boost::assign;
#include <boost/make_shared.hpp>

#include <CppUnitLite/TestHarness.h>
#include <gtsam/symbolic/SymbolicConditionalUnordered.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( SymbolicConditional, empty )
{
  SymbolicConditionalUnordered c0;
  LONGS_EQUAL(0,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, noParents )
{
  SymbolicConditionalUnordered c0(0);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, oneParents )
{
  SymbolicConditionalUnordered c0(0,1);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(1,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, twoParents )
{
  SymbolicConditionalUnordered c0(0,1,2);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(2,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, threeParents )
{
  SymbolicConditionalUnordered c0(0,1,2,3);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(3,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, fourParents )
{
  SymbolicConditionalUnordered c0 = SymbolicConditionalUnordered::FromKeys(
    list_of(0)(1)(2)(3)(4), 1);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(4,c0.nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, FromRange )
{
  SymbolicConditionalUnordered::shared_ptr c0 =
    boost::make_shared<SymbolicConditionalUnordered>(
    SymbolicConditionalUnordered::FromKeys(list_of(1)(2)(3)(4)(5), 2));
  LONGS_EQUAL(2,c0->nrFrontals())
  LONGS_EQUAL(3,c0->nrParents())
}

/* ************************************************************************* */
TEST( SymbolicConditional, equals )
{
  SymbolicConditionalUnordered c0(0, 1, 2), c1(0, 1, 2), c2(1, 2, 3), c3(3,4);
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
