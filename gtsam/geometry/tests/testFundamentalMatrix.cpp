/*
 * @file testFundamentalMatrix.cpp
 * @brief Test FundamentalMatrix classes
 * @author Frank Dellaert
 * @date October 23, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(GeneralFundamentalMatrix)
GTSAM_CONCEPT_MANIFOLD_INST(GeneralFundamentalMatrix)

//*************************************************************************
// Create two rotations and corresponding fundamental matrix F
Rot3 trueU = Rot3::Yaw(M_PI_2);
Rot3 trueV = Rot3::Yaw(M_PI_4);
double trueS = 0.5;
GeneralFundamentalMatrix trueF(trueU, trueS, trueV);

//*************************************************************************
TEST(GeneralFundamentalMatrix, localCoordinates) {
  Vector expected = Z_7x1;  // Assuming 7 dimensions for U, V, and s
  Vector actual = trueF.localCoordinates(trueF);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST(GeneralFundamentalMatrix, retract) {
  GeneralFundamentalMatrix actual = trueF.retract(Z_7x1);
  EXPECT(assert_equal(trueF, actual));
}

//*************************************************************************
TEST(GeneralFundamentalMatrix, RoundTrip) {
  Vector7 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  GeneralFundamentalMatrix hx = trueF.retract(d);
  Vector actual = trueF.localCoordinates(hx);
  EXPECT(assert_equal(d, actual, 1e-8));
}

//*************************************************************************
// Create the simplest SimpleFundamentalMatrix, a stereo pair
EssentialMatrix defaultE(Rot3(), Unit3(1, 0, 0));
Point2 zero(0.0, 0.0);
SimpleFundamentalMatrix stereoF(defaultE, 1.0, 1.0, zero, zero);

//*************************************************************************
TEST(SimpleStereo, Conversion) {
  GeneralFundamentalMatrix convertedF(stereoF.matrix());
  EXPECT(assert_equal(stereoF.matrix(), convertedF.matrix(), 1e-8));
}

//*************************************************************************
TEST(SimpleStereo, localCoordinates) {
  Vector expected = Z_7x1;
  Vector actual = stereoF.localCoordinates(stereoF);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST(SimpleStereo, retract) {
  SimpleFundamentalMatrix actual = stereoF.retract(Z_9x1);
  EXPECT(assert_equal(stereoF, actual));
}

//*************************************************************************
TEST(SimpleStereo, RoundTrip) {
  Vector7 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  SimpleFundamentalMatrix hx = stereoF.retract(d);
  Vector actual = stereoF.localCoordinates(hx);
  EXPECT(assert_equal(d, actual, 1e-8));
}

//*************************************************************************
TEST(SimpleStereo, EpipolarLine) {
  // Create a point in b
  Point3 p_b(0, 2, 1);
  // Convert the point to a horizontal line in a
  Vector3 l_a = stereoF.matrix() * p_b;
  // Check if the line is horizontal at height 2
  EXPECT(assert_equal(Vector3(0, -1, 2), l_a));
}

//*************************************************************************
// Create a stereo pair, but in pixels not normalized coordinates.
// We're still using zero principal points here.
double fa = 1000;
double fb = 1000;
SimpleFundamentalMatrix pixelStereo(defaultE, fa, fb, zero, zero);

//*************************************************************************
TEST(PixelStereo, Conversion) {
  auto expected = pixelStereo.matrix();

  GeneralFundamentalMatrix convertedF(pixelStereo.matrix());

  // Check equality of F-matrices up to a scale
  auto actual = convertedF.matrix();
  actual *= expected(1, 2) / actual(1, 2);
  EXPECT(assert_equal(expected, actual, 1e-5));
}

//*************************************************************************
TEST(PixelStereo, PointInBToHorizontalLineInA) {
  // Create a point in b
  Point3 p_b = Point3(0, 300, 1);
  // Convert the point to a horizontal line in a
  Vector3 l_a = pixelStereo.matrix() * p_b;
  // Check if the line is horizontal at height 2
  EXPECT(assert_equal(Vector3(0, -0.001, 0.3), l_a));
}

//*************************************************************************
// Create a stereo pair with the right camera rotated 90 degrees
Rot3 aRb = Rot3::Rz(M_PI_2);  // Rotate 90 degrees around the Z-axis
EssentialMatrix rotatedE(aRb, Unit3(1, 0, 0));
SimpleFundamentalMatrix rotatedPixelStereo(rotatedE, fa, fb, zero, zero);

//*************************************************************************
TEST(RotatedPixelStereo, Conversion) {
  auto expected = rotatedPixelStereo.matrix();

  GeneralFundamentalMatrix convertedF(rotatedPixelStereo.matrix());

  // Check equality of F-matrices up to a scale
  auto actual = convertedF.matrix();
  actual *= expected(1, 2) / actual(1, 2);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

//*************************************************************************
TEST(RotatedPixelStereo, PointInBToHorizontalLineInA) {
  // Create a point in b
  Point3 p_b = Point3(300, 0, 1);
  // Convert the point to a horizontal line in a
  Vector3 l_a = rotatedPixelStereo.matrix() * p_b;
  // Check if the line is horizontal at height 2
  EXPECT(assert_equal(Vector3(0, -0.001, 0.3), l_a));
}

//*************************************************************************
// Now check that principal points also survive conversion
SimpleFundamentalMatrix stereoWithPrincipalPoints(rotatedE, fa, fb, zero, zero);

//*************************************************************************
TEST(stereoWithPrincipalPoints, Conversion) {
  auto expected = stereoWithPrincipalPoints.matrix();

  GeneralFundamentalMatrix convertedF(stereoWithPrincipalPoints.matrix());

  // Check equality of F-matrices up to a scale
  auto actual = convertedF.matrix();
  actual *= expected(1, 2) / actual(1, 2);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
