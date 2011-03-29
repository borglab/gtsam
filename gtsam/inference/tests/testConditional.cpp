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
#include <gtsam/inference/IndexConditional.h>
#include <gtsam/inference/IndexFactor.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST( IndexConditional, empty )
{
  IndexConditional c0;
  LONGS_EQUAL(0,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, noParents )
{
  IndexConditional c0(0);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(0,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, oneParents )
{
  IndexConditional c0(0,1);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(1,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, twoParents )
{
  IndexConditional c0(0,1,2);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(2,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, threeParents )
{
  IndexConditional c0(0,1,2,3);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(3,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, fourParents )
{
	vector<Index> parents;
	parents += 1,2,3,4;
  IndexConditional c0(0,parents);
  LONGS_EQUAL(1,c0.nrFrontals())
  LONGS_EQUAL(4,c0.nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, FromRange )
{
	vector<Index> keys;
	keys += 1,2,3,4,5;
  IndexConditional::shared_ptr c0(new IndexConditional(keys,2));
  LONGS_EQUAL(2,c0->nrFrontals())
  LONGS_EQUAL(3,c0->nrParents())
}

/* ************************************************************************* */
TEST( IndexConditional, equals )
{
  IndexConditional c0(0, 1, 2), c1(0, 1, 2), c2(1, 2, 3), c3(3,4);
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
