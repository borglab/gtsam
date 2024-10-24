/*
 * @file testFundamentalMatrix.cpp
 * @brief Test FundamentalMatrix classes
 * @author Frank Dellaert
 * @date October 23, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/geometry/Unit3.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(FundamentalMatrix)
GTSAM_CONCEPT_MANIFOLD_INST(FundamentalMatrix)

//*************************************************************************
// Create two rotations and corresponding fundamental matrix F
Rot3 trueU = Rot3::Yaw(M_PI_2);
Rot3 trueV = Rot3::Yaw(M_PI_4);
double trueS = 0.5;
FundamentalMatrix trueF(trueU, trueS, trueV);

//*************************************************************************
TEST(FundamentalMatrix, localCoordinates) {
  Vector expected = Z_7x1;  // Assuming 7 dimensions for U, V, and s
  Vector actual = trueF.localCoordinates(trueF);
  EXPECT(assert_equal(expected, actual, 1e-8));
}

//*************************************************************************
TEST(FundamentalMatrix, retract) {
  FundamentalMatrix actual = trueF.retract(Z_7x1);
  EXPECT(assert_equal(trueF, actual));
}

//*************************************************************************
TEST(FundamentalMatrix, RoundTrip) {
  Vector7 d;
  d << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7;
  FundamentalMatrix hx = trueF.retract(d);
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
  FundamentalMatrix convertedF(stereoF.matrix());
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
double focalLength = 1000;
SimpleFundamentalMatrix pixelStereo(defaultE, focalLength, focalLength, zero,
                                    zero);

//*************************************************************************
TEST(PixelStereo, Conversion) {
  auto expected = pixelStereo.matrix();

  FundamentalMatrix convertedF(pixelStereo.matrix());

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
SimpleFundamentalMatrix rotatedPixelStereo(rotatedE, focalLength, focalLength,
                                           zero, zero);

//*************************************************************************
TEST(RotatedPixelStereo, Conversion) {
  auto expected = rotatedPixelStereo.matrix();

  FundamentalMatrix convertedF(rotatedPixelStereo.matrix());

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
Point2 principalPoint(640 / 2, 480 / 2);
SimpleFundamentalMatrix stereoWithPrincipalPoints(rotatedE, focalLength,
                                                  focalLength, principalPoint,
                                                  principalPoint);

//*************************************************************************
TEST(stereoWithPrincipalPoints, Conversion) {
  auto expected = stereoWithPrincipalPoints.matrix();

  FundamentalMatrix convertedF(stereoWithPrincipalPoints.matrix());

  // Check equality of F-matrices up to a scale
  auto actual = convertedF.matrix();
  actual *= expected(1, 2) / actual(1, 2);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

//*************************************************************************
/// Generate three cameras on a circle, looking in
std::array<Pose3, 3> generateCameraPoses() {
  std::array<Pose3, 3> cameraPoses;
  const double radius = 1.0;
  for (int i = 0; i < 3; ++i) {
    double angle = i * 2.0 * M_PI / 3.0;
    double c = cos(angle), s = sin(angle);
    Rot3 aRb({-s, c, 0}, {0, 0, -1}, {-c, -s, 0});
    cameraPoses[i] = {aRb, Point3(radius * c, radius * s, 0)};
  }
  return cameraPoses;
}

//*************************************************************************
/// Function to generate a TripleF from camera poses
TripleF<SimpleFundamentalMatrix> generateTripleF(
    const std::array<Pose3, 3>& cameraPoses) {
  std::array<SimpleFundamentalMatrix, 3> F;
  for (size_t i = 0; i < 3; ++i) {
    size_t j = (i + 1) % 3;
    const Pose3 iPj = cameraPoses[i].between(cameraPoses[j]);
    EssentialMatrix E(iPj.rotation(), Unit3(iPj.translation()));
    F[i] = {E, focalLength, focalLength, principalPoint, principalPoint};
  }
  return {F[0], F[1], F[2]};  // Return a TripleF instance
}

//*************************************************************************

struct TripletFactor {
  using F = FundamentalMatrix;
  using SF = SimpleFundamentalMatrix;
  Point2 p0, p1, p2;

  /// vector of errors returns 6D vector
  Vector evaluateError(const SF& F01, const SF& F12, const SF& F20,  //
                       Matrix* H01, Matrix* H12, Matrix* H20) const {
    Vector error(6);
    std::function<Vector6(SF, SF, SF)> fn = [&](const SF& F01, const SF& F12,
                                                const SF& F20) {
      Vector6 error;
      error << F::transfer(F01.matrix(), p1, F20.matrix().transpose(), p2) - p0,
          F::transfer(F01.matrix().transpose(), p0, F12.matrix(), p2) - p1,
          F::transfer(F20.matrix(), p0, F12.matrix().transpose(), p1) - p2;
      return error;
    };
    if (H01)
      *H01 = numericalDerivative31<Vector6, SF, SF, SF>(fn, F01, F12, F20);
    if (H12)
      *H12 = numericalDerivative32<Vector6, SF, SF, SF>(fn, F01, F12, F20);
    if (H20)
      *H20 = numericalDerivative33<Vector6, SF, SF, SF>(fn, F01, F12, F20);
    return fn(F01, F12, F20);
  }
};

//*************************************************************************
TEST(TripleF, Transfer) {
  // Generate cameras on a circle, as well as fundamental matrices
  auto cameraPoses = generateCameraPoses();
  auto triplet = generateTripleF(cameraPoses);

  // Check that they are all equal
  EXPECT(triplet.Fab.equals(triplet.Fbc, 1e-9));
  EXPECT(triplet.Fbc.equals(triplet.Fca, 1e-9));
  EXPECT(triplet.Fca.equals(triplet.Fab, 1e-9));

  // Now project a point into the three cameras
  const Point3 P(0.1, 0.2, 0.3);
  const Cal3_S2 K(focalLength, focalLength, 0.0,  //
                  principalPoint.x(), principalPoint.y());

  std::array<Point2, 3> p;
  for (size_t i = 0; i < 3; ++i) {
    // Project the point into each camera
    PinholeCameraCal3_S2 camera(cameraPoses[i], K);
    p[i] = camera.project(P);
  }

  // Check that transfer works
  EXPECT(assert_equal<Point2>(p[0], triplet.transferToA(p[1], p[2]), 1e-9));
  EXPECT(assert_equal<Point2>(p[1], triplet.transferToB(p[0], p[2]), 1e-9));
  EXPECT(assert_equal<Point2>(p[2], triplet.transferToC(p[0], p[1]), 1e-9));
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*************************************************************************
