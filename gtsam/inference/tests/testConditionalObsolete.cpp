/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testConditional.cpp
 * @brief   Unit tests for IndexConditional class
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/inference/IndexConditionalOrdered.h>
#include <gtsam/inference/IndexFactorOrdered.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( IndexConditionalOrdered, empty )
{
  IndexConditionalOrdered c0;
  LONGS_EQUAL(0,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, noParents )
{
  IndexConditionalOrdered c0(0);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, oneParents )
{
  IndexConditionalOrdered c0(0,1);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(1,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, twoParents )
{
  IndexConditionalOrdered c0(0,1,2);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(2,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, threeParents )
{
  IndexConditionalOrdered c0(0,1,2,3);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(3,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, fourParents )
{
  vector<Index> parents;
  parents += 1,2,3,4;
  IndexConditionalOrdered c0(0,parents);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(4,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, FromRange )
{
  vector<Index> keys;
  keys += 1,2,3,4,5;
  IndexConditionalOrdered::shared_ptr c0(new IndexConditionalOrdered(keys,2));
  LONGS_EQUAL(2,c0->nrFrontals())
  LONGS_EQUAL(3,c0->nrParents())
}

/* ************************************************************************* */
TEST( IndexConditionalOrdered, equals )
{
  IndexConditionalOrdered c0(0, 1, 2), c1(0, 1, 2), c2(1, 2, 3), c3(3,4);
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
