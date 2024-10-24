/*
 * @file testTransferFactor.cpp
 * @brief Test TransferFactor class
 * @author Your Name
 * @date October 23, 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sfm/TransferFactor.h>

using namespace gtsam;

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

double focalLength = 1000;
Point2 principalPoint(640 / 2, 480 / 2);

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
  TripletError<SimpleFundamentalMatrix> error{p[0], p[1], p[2]};
  Matrix H01, H12, H20;
  Vector e = error.evaluateError(triplet.F01, triplet.F12, triplet.F20, &H01,
                                 &H12, &H20);
  std::cout << "Error: " << e << std::endl;
  std::cout << H01 << std::endl << std::endl;
  std::cout << H12 << std::endl << std::endl;
  std::cout << H20 << std::endl;

  // Create a TransferFactor
  TransferFactor<SimpleFundamentalMatrix> factor{p[0], p[1], p[2]};
  Matrix H0, H1;
  Vector e2 = factor.evaluateError(triplet.F12, triplet.F20, &H0, &H1);
  std::cout << "Error: " << e2 << std::endl;
  std::cout << H0 << std::endl << std::endl;
  std::cout << H1 << std::endl << std::endl;

  // Check Jacobians
  Values values;
  values.insert(1, triplet.F12);
  values.insert(2, triplet.F20);
  // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-7);
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*************************************************************************
