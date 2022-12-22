/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDiscreteFactor.cpp
 *
 *  @date Feb 14, 2011
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DiscreteFactor.h>

#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;
using namespace gtsam::serializationTestHelpers;

/* ************************************************************************* */
TEST(DisreteKeys, Serialization) {
  DiscreteKeys keys;
  keys& DiscreteKey(0, 2);
  keys& DiscreteKey(1, 3);
  keys& DiscreteKey(2, 4);

  EXPECT(equalsObj<DiscreteKeys>(keys));
  EXPECT(equalsXML<DiscreteKeys>(keys));
  EXPECT(equalsBinary<DiscreteKeys>(keys));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
