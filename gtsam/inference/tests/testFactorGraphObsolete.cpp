/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testFactorgraph.cpp
 *  @brief  Unit tests for IndexFactor Graphs
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <list>
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/set.hpp> // for operator +=
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <gtsam/inference/IndexConditionalOrdered.h>
#include <gtsam/inference/SymbolicFactorGraphOrdered.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(FactorGraphOrdered, eliminateFrontals) {

  SymbolicFactorGraphOrdered sfgOrig;
  sfgOrig.push_factor(0,1);
  sfgOrig.push_factor(0,2);
  sfgOrig.push_factor(1,3);
  sfgOrig.push_factor(1,4);
  sfgOrig.push_factor(2,3);
  sfgOrig.push_factor(4,5);

  IndexConditionalOrdered::shared_ptr actualCond;
  SymbolicFactorGraphOrdered actualSfg;
  boost::tie(actualCond, actualSfg) = sfgOrig.eliminateFrontals(2);

  vector<Index> condIndices;
  condIndices += 0,1,2,3,4;
  IndexConditionalOrdered expectedCond(condIndices, 2);

  SymbolicFactorGraphOrdered expectedSfg;
  expectedSfg.push_factor(2,3);
  expectedSfg.push_factor(4,5);
  expectedSfg.push_factor(2,3,4);

  EXPECT(assert_equal(expectedSfg, actualSfg));
  EXPECT(assert_equal(expectedCond, *actualCond));
}

/* ************************************************************************* */
int main() {  TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
