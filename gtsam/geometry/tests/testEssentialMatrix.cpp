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
#include <sstream>

using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(EssentialMatrix)
GTSAM_CONCEPT_MANIFOLD_INST(EssentialMatrix)

//*************************************************************************
// Create two cameras and corresponding essential matrix E
Rot3 trueRotation = Rot3::Yaw(M_PI_2);
Point3 trueTranslation(0.1, 0, 0);
Unit3 trueDirection(trueTranslation);
EssentialMatrix trueE(trueRotation, trueDirection);

//*************************************************************************
TEST (EssentialMatrix, equality) {
  EssentialMatrix actual(trueRotation, trueDirection), expected(trueRotation, trueDirection);
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST(EssentialMatrix, FromRotationAndDirection) {
  Matrix actualH1, actualH2;
  EXPECT(assert_equal(
      trueE, EssentialMatrix::FromRotationAndDirection(trueRotation, trueDirection, actualH1, actualH2),
      1e-8));

  Matrix expectedH1 = numericalDerivative11<EssentialMatrix, Rot3>(
      boost::bind(EssentialMatrix::FromRotationAndDirection, _1, trueDirection, boost::none,
                  boost::none),
      trueRotation);
  EXPECT(assert_equal(expectedH1, actualH1, 1e-7));

  Matrix expectedH2 = numericalDerivative11<EssentialMatrix, Unit3>(
      boost::bind(EssentialMatrix::FromRotationAndDirection, trueRotation, _1, boost::none,
                  boost::none),
                  trueDirection);
  EXPECT(assert_equal(expectedH2, actualH2, 1e-7));
}

//*************************************************************************
TEST (EssentialMatrix, FromPose3) {
  EssentialMatrix expected(trueRotation, trueDirection);
  Pose3 pose(trueRotation, trueTranslation);
  EssentialMatrix actual = EssentialMatrix::FromPose3(pose);
  EXPECT(assert_equal(expected, actual));
}

//*******************************************************************************
TEST(EssentialMatrix, localCoordinates0) {
  EssentialMatrix E;
  Vector expected = Z_5x1;
  Vector actual = E.localCoordinates(E);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST (EssentialMatrix, localCoordinates) {

  // Pose between two cameras
  Pose3 pose(trueRotation, trueTranslation);
  EssentialMatrix hx = EssentialMatrix::FromPose3(pose);
  Vector actual = hx.localCoordinates(EssentialMatrix::FromPose3(pose));
  EXPECT(assert_equal(Z_5x1, actual, 1e-8));

  Vector6 d;
  d << 0.1, 0.2, 0.3, 0, 0, 0;
  Vector actual2 = hx.localCoordinates(
      EssentialMatrix::FromPose3(pose.retract(d)));
  EXPECT(assert_equal(d.head(5), actual2, 1e-8));
}

//*************************************************************************
TEST (EssentialMatrix, retract0) {
  EssentialMatrix actual = trueE.retract(Z_5x1);
  EXPECT(assert_equal(trueE, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract1) {
  EssentialMatrix expected(trueRotation.retract(Vector3(0.1, 0, 0)), trueDirection);
  EssentialMatrix actual = trueE.retract((Vector(5) << 0.1, 0, 0, 0, 0).finished());
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, retract2) {
  EssentialMatrix expected(trueRotation,
      trueDirection.retract(Vector2(0.1, 0)));
  EssentialMatrix actual = trueE.retract((Vector(5) << 0, 0, 0, 0.1, 0).finished());
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, RoundTrip) {
  Vector5 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5;
  EssentialMatrix e, hx = e.retract(d);
  Vector actual = e.localCoordinates(hx);
  EXPECT(assert_equal(d, actual, 1e-8));
}

//*************************************************************************
Point3 transform_to_(const EssentialMatrix& E, const Point3& point) {
  return E.transformTo(point);
}
TEST (EssentialMatrix, transformTo) {
  // test with a more complicated EssentialMatrix
  Rot3 aRb2 = Rot3::Yaw(M_PI / 3.0) * Rot3::Pitch(M_PI_4)
      * Rot3::Roll(M_PI / 6.0);
  Point3 aTb2(19.2, 3.7, 5.9);
  EssentialMatrix E(aRb2, Unit3(aTb2));
  //EssentialMatrix E(aRb, Unit3(aTb).retract(Vector2(0.1, 0)));
  static Point3 P(0.2, 0.7, -2);
  Matrix actH1, actH2;
  E.transformTo(P, actH1, actH2);
  Matrix expH1 = numericalDerivative21(transform_to_, E, P), //
  expH2 = numericalDerivative22(transform_to_, E, P);
  EXPECT(assert_equal(expH1, actH1, 1e-8));
  EXPECT(assert_equal(expH2, actH2, 1e-8));
}

//*************************************************************************
EssentialMatrix rotate_(const EssentialMatrix& E, const Rot3& cRb) {
  return E.rotate(cRb);
}
TEST (EssentialMatrix, rotate) {
  // Suppose the essential matrix is specified in a body coordinate frame B
  // which is rotated with respect to the camera frame C, via rotation bRc.
  // The rotation between body and camera is:
  Point3 bX(1, 0, 0), bY(0, 1, 0), bZ(0, 0, 1);
  Rot3 bRc(bX, bZ, -bY), cRb = bRc.inverse();

  // Let's compute the ground truth E in body frame:
  Rot3 b1Rb2 = bRc * trueRotation * cRb;
  Point3 b1Tb2 = bRc * trueTranslation;
  EssentialMatrix bodyE(b1Rb2, Unit3(b1Tb2));
  EXPECT(assert_equal(bodyE, bRc * trueE, 1e-8));
  EXPECT(assert_equal(bodyE, trueE.rotate(bRc), 1e-8));

  // Let's go back to camera frame:
  EXPECT(assert_equal(trueE, cRb * bodyE, 1e-8));
  EXPECT(assert_equal(trueE, bodyE.rotate(cRb), 1e-8));

  // Derivatives
  Matrix actH1, actH2;
  try {
    bodyE.rotate(cRb, actH1, actH2);
  } catch (exception& e) {
  } // avoid exception
  Matrix expH1 = numericalDerivative21(rotate_, bodyE, cRb), //
  expH2 = numericalDerivative22(rotate_, bodyE, cRb);
  EXPECT(assert_equal(expH1, actH1, 1e-7));
  // Does not work yet EXPECT(assert_equal(expH2, actH2, 1e-8));
}

//*************************************************************************
TEST (EssentialMatrix, FromPose3_a) {
  Matrix actualH;
  Pose3 pose(trueRotation, trueTranslation); // Pose between two cameras
  EXPECT(assert_equal(trueE, EssentialMatrix::FromPose3(pose, actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<EssentialMatrix, Pose3>(
      boost::bind(EssentialMatrix::FromPose3, _1, boost::none), pose);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));
}

//*************************************************************************
TEST (EssentialMatrix, FromPose3_b) {
  Matrix actualH;
  Rot3 c1Rc2 = Rot3::Ypr(0.1, -0.2, 0.3);
  Point3 c1Tc2(0.4, 0.5, 0.6);
  EssentialMatrix E(c1Rc2, Unit3(c1Tc2));
  Pose3 pose(c1Rc2, c1Tc2); // Pose between two cameras
  EXPECT(assert_equal(E, EssentialMatrix::FromPose3(pose, actualH), 1e-8));
  Matrix expectedH = numericalDerivative11<EssentialMatrix, Pose3>(
      boost::bind(EssentialMatrix::FromPose3, _1, boost::none), pose);
  EXPECT(assert_equal(expectedH, actualH, 1e-5));
}

//*************************************************************************
TEST (EssentialMatrix, streaming) {
  EssentialMatrix expected(trueRotation, trueDirection), actual;
  stringstream ss;
  ss << expected;
  ss >> actual;
  EXPECT(assert_equal(expected, actual));
}

//*************************************************************************
TEST (EssentialMatrix, epipoles) {
  // Create an E
  Rot3 c1Rc2 = Rot3::Ypr(0.1, -0.2, 0.3);
  Point3 c1Tc2(0.4, 0.5, 0.6);
  EssentialMatrix E(c1Rc2, Unit3(c1Tc2));

  // Calculate expected values through SVD
  Matrix U, V;
  Vector S;
  gtsam::svd(E.matrix(), U, S, V);

  // take care of SVD sign ambiguity
  if (U(0, 2) > 0) {
    U = -U;
    V = -V;
  }

  // check rank 2 constraint
  CHECK(std::abs(S(2))<1e-10);

  // check epipoles not at infinity
  CHECK(std::abs(U(2,2))>1e-10 && std::abs(V(2,2))>1e-10);

  // Check epipoles

  // Epipole in image 1 is just E.direction()
  Unit3 e1(-U(0, 2), -U(1, 2), -U(2, 2));
  Unit3 actual = E.epipole_a();
  EXPECT(assert_equal(e1, actual));

  // take care of SVD sign ambiguity
  if (V(0, 2) < 0) {
    U = -U;
    V = -V;
  }

  // Epipole in image 2 is E.rotation().unrotate(E.direction())
  Unit3 e2(V(0, 2), V(1, 2), V(2, 2));
  EXPECT(assert_equal(e2, E.epipole_b()));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

