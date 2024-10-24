/*
 * @file testTransferFactor.cpp
 * @brief Test TransferFactor class
 * @author Your Name
 * @date October 23, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sfm/TransferFactor.h>

using namespace gtsam;

double focalLength = 1000;
Point2 principalPoint(640 / 2, 480 / 2);

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
// Function to generate a TripleF from camera poses
TripleF<SimpleFundamentalMatrix> generateTripleF(
    const std::array<Pose3, 3>& cameraPoses) {
  std::array<SimpleFundamentalMatrix, 3> F;
  for (size_t i = 0; i < 3; ++i) {
    size_t j = (i + 1) % 3;
    const Pose3 iPj = cameraPoses[i].between(cameraPoses[j]);
    EssentialMatrix E(iPj.rotation(), Unit3(iPj.translation()));
    F[i] = {E, 1000.0, 1000.0, Point2(640 / 2, 480 / 2),
            Point2(640 / 2, 480 / 2)};
  }
  return {F[0], F[1], F[2]};  // Return a TripleF instance
}

//*************************************************************************
// Test for TransferFactor
TEST(TransferFactor, Jacobians) {
  // Generate cameras on a circle
  std::array<Pose3, 3> cameraPoses = generateCameraPoses();
  auto triplet = generateTripleF(cameraPoses);

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

  // Create a TransferFactor
  EdgeKey key01(0, 1), key12(1, 2), key20(2, 0);
  TransferFactor<SimpleFundamentalMatrix>  //
      factor0{key01, key20, p[1], p[2], p[0]},
      factor1{key12, key01, p[2], p[0], p[1]},
      factor2{key20, key12, p[0], p[1], p[2]};

  // Check that getMatrices is correct
  auto [Fki, Fkj] = factor2.getMatrices(key20, triplet.Fca, key12, triplet.Fbc);
  EXPECT(assert_equal<Matrix3>(triplet.Fca.matrix(), Fki));
  EXPECT(assert_equal<Matrix3>(triplet.Fbc.matrix().transpose(), Fkj));

  // Create Values with edge keys
  Values values;
  values.insert(key01, triplet.Fab);
  values.insert(key12, triplet.Fbc);
  values.insert(key20, triplet.Fca);

  // Check error and Jacobians
  for (auto&& f : {factor0, factor1, factor2}) {
    Vector error = f.unwhitenedError(values);
    EXPECT(assert_equal<Vector>(Z_2x1, error));
    EXPECT_CORRECT_FACTOR_JACOBIANS(f, values, 1e-5, 1e-7);
  }
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*************************************************************************
