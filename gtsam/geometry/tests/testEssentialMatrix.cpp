/*
 * @file testEssentialMatrix.cpp
 * @brief Test EssentialMatrix class
 * @author Frank Dellaert
 * @date December 17, 2013
 */

#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(EssentialMatrix)
GTSAM_CONCEPT_MANIFOLD_INST(EssentialMatrix)

//*************************************************************************
// Create two cameras and corresponding essential matrix E
Rot3 c1Rc2 = Rot3::yaw(M_PI_2);
Point3 c1Tc2(0.1, 0, 0);
EssentialMatrix trueE(c1Rc2, c1Tc2);

//*************************************************************************
TEST (EssentialMatrix, equality) {
  EssentialMatrix actual(c1Rc2, c1Tc2), expected(c1Rc2, c1Tc2);
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract1) {
  EssentialMatrix expected(c1Rc2.retract((Vector(3) << 0.1, 0, 0)), c1Tc2);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0.1, 0, 0, 0, 0));
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract2) {
  EssentialMatrix expected(c1Rc2,
      Sphere2(c1Tc2).retract((Vector(2) << 0.1, 0)));
  EssentialMatrix actual = trueE.retract((Vector(5) << 0, 0, 0, 0.1, 0));
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
Point3 transform_to_(const EssentialMatrix& E, const Point3& point) {
  return E.transform_to(point);
}
TEST (EssentialMatrix, transform_to) {
  // test with a more complicated EssentialMatrix
  Rot3 aRb2 = Rot3::yaw(M_PI / 3.0) * Rot3::pitch(M_PI_4)
      * Rot3::roll(M_PI / 6.0);
  Point3 aTb2(19.2, 3.7, 5.9);
  EssentialMatrix E(aRb2, aTb2);
  //EssentialMatrix E(aRb, Sphere2(aTb).retract((Vector(2) << 0.1, 0)));
  static Point3 P(0.2, 0.7, -2);
  Matrix actH1, actH2;
  E.transform_to(P, actH1, actH2);
  Matrix expH1 = numericalDerivative21(transform_to_, E, P), expH2 =
      numericalDerivative22(transform_to_, E, P);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

//*************************************************************************

/**
 * Given essential matrix E in camera frame B, convert to body frame C
 * @param cRb rotation from body frame to camera frame
 * @param E essential matrix E in camera frame C
 */
EssentialMatrix operator*(const Rot3& cRb, const EssentialMatrix& E) {
  Rot3 b1Rb2 = E.rotation();
  Sphere2 b1Tb2 = E.direction();
  Rot3  c1Rc2 = b1Rb2.conjugate(cRb);
  Sphere2 c1Tc2 = cRb * b1Tb2;
  return EssentialMatrix(c1Rc2, c1Tc2);
}

TEST (EssentialMatrix, rotate) {
  // Suppose the essential matrix is specified in a body coordinate frame B
  // which is rotated with respect to the camera frame C, via rotation bRc.
  // The rotation between body and camera is:
  Point3 bX(1, 0, 0), bY(0, 1, 0), bZ(0, 0, 1);
  Rot3 bRc(bX, bZ, -bY), cRb = bRc.inverse();

  // Let's compute the ground truth E in body frame:
  Rot3 b1Rb2 = bRc * c1Rc2 * cRb;
  Point3 b1Tb2 = bRc * c1Tc2;
  EssentialMatrix bodyE(b1Rb2, b1Tb2);
  EXPECT(assert_equal(bodyE, bRc * trueE, 1e-8));

  // Let's go back to camera frame:
  EXPECT(assert_equal(trueE, cRb * bodyE, 1e-8));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

