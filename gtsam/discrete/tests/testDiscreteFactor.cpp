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

#include <gtsam/base/Testable.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/map.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

