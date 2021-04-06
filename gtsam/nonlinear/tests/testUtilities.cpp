/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testUtilities.cpp
 * @author Gerry Chen
 * @author Frank Dellaert
 */

#include <gtsam/nonlinear/utilities.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>

#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST( utilities, extractVector )
{
  auto values = Values();
  values.insert(0, (Vector(6) << 1,2,3,4,5,6).finished());
  values.insert(1, (Vector(3) << 7,8,9).finished());
  values.insert(2, (Vector(6) << 13,14,15,16,17,18).finished());
  values.insert(3, Pose3());
  auto actual = utilities::extractVector(values);
  auto expected =
      (Vector(15) << 1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 14, 15, 16, 17, 18)
          .finished();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
