/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testJunctionTree.cpp
 * @brief   Unit tests for Junction Tree
 * @author  Kai Ni
 * @author  Frank Dellaert
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/vector.hpp> // for operator +=
#include <boost/assign/std/set.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>

#include <gtsam/symbolic/SymbolicFactorGraphUnordered.h>
#include <gtsam/symbolic/SymbolicEliminationTreeUnordered.h>
#include <gtsam/symbolic/SymbolicJunctionTreeUnordered.h>

#include "symbolicExampleGraphs.h"

using namespace gtsam;
using namespace std;

/* ************************************************************************* *
 * 1 - 0 - 2 - 3
 * 2 3
 *   0 1 : 2
 ****************************************************************************/
TEST( JunctionTree, constructor )
{
  OrderingUnordered order; order += 0, 1, 2, 3;

  SymbolicJunctionTreeUnordered actual(SymbolicEliminationTreeUnordered(simpleChain, order));

  vector<Index> frontal1; frontal1 += 3, 2;
  vector<Index> frontal2; frontal2 += 1, 0;
  vector<Index> sep1;
  vector<Index> sep2; sep2 += 2;
  EXPECT(assert_equal(frontal1, actual.roots().front()->keys));
  //EXPECT(assert_equal(sep1,     actual.roots().front()->separator));
  LONGS_EQUAL(1,                actual.roots().front()->factors.size());
  EXPECT(assert_equal(frontal2, actual.roots().front()->children.front()->keys));
  //EXPECT(assert_equal(sep2,     actual.roots().front()->children.front()->separator));
  LONGS_EQUAL(2,                actual.roots().front()->children.front()->factors.size());
  EXPECT(assert_equal(*simpleChain[2],   *actual.roots().front()->factors[0]));
  EXPECT(assert_equal(*simpleChain[0],   *actual.roots().front()->children.front()->factors[0]));
  EXPECT(assert_equal(*simpleChain[1],   *actual.roots().front()->children.front()->factors[1]));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
