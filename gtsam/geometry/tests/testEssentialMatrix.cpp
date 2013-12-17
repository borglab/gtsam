/*
 * @file testEssentialMatrix.cpp
 * @brief Test EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//*************************************************************************
// Create two cameras and corresponding essential matrix E
Rot3 aRb = Rot3::yaw(M_PI_2);
Point3 aTb(0.1, 0, 0);

//*************************************************************************
TEST (EssentialMatrix, equality) {
  EssentialMatrix actual(aRb, aTb), expected(aRb, aTb);
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract1) {
  EssentialMatrix expected(aRb.retract((Vector(3) << 0.1, 0, 0)), aTb);
  EssentialMatrix trueE(aRb, aTb);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0.1, 0, 0, 0, 0));
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract2) {
  EssentialMatrix expected(aRb, Sphere2(aTb).retract((Vector(2) << 0.1, 0)));
  EssentialMatrix trueE(aRb, aTb);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0, 0, 0, 0.1, 0));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

