/*
 * @file testTransferFactor.cpp
 * @brief Test TransferFactor classes
 * @author Frank Dellaert
 * @date October 2024
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/sfm/TransferFactor.h>

#include <memory>

using namespace gtsam;
using symbol_shorthand::K;

//*************************************************************************
/// Generate three cameras on a circle, looking inwards
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
namespace fixture {
// Generate cameras on a circle
std::array<Pose3, 3> cameraPoses = generateCameraPoses();
auto triplet = generateTripleF(cameraPoses);
double focalLength = 1000;
Point2 principalPoint(640 / 2, 480 / 2);
const Cal3_S2 cal(focalLength, focalLength, 0.0,  //
                  principalPoint.x(), principalPoint.y());
// Create cameras
auto f = [](const Pose3& pose) { return PinholeCamera<Cal3_S2>(pose, cal); };
std::array<PinholeCamera<Cal3_S2>, 3> cameras = {
    f(cameraPoses[0]), f(cameraPoses[1]), f(cameraPoses[2])};
}  // namespace fixture

//*************************************************************************
// Test for getMatrices
TEST(TransferFactor, GetMatrices) {
  using namespace fixture;
  TransferFactor<SimpleFundamentalMatrix> factor{{2, 0}, {1, 2}, {}};

  // Check that getMatrices is correct
  auto [Fki, Fkj] = factor.getMatrices(triplet.Fca, triplet.Fbc);
  EXPECT(assert_equal<Matrix3>(triplet.Fca.matrix(), Fki));
  EXPECT(assert_equal<Matrix3>(triplet.Fbc.matrix().transpose(), Fkj));
}

//*************************************************************************
// Test for TransferFactor
TEST(TransferFactor, Jacobians) {
  using namespace fixture;

  // Now project a point into the three cameras
  const Point3 P(0.1, 0.2, 0.3);
  std::array<Point2, 3> p;
  for (size_t i = 0; i < 3; ++i) {
    p[i] = cameras[i].project(P);
  }

  // Create a TransferFactor
  EdgeKey key01(0, 1), key12(1, 2), key20(2, 0);
  TransferFactor<SimpleFundamentalMatrix>  //
      factor0{key01, key20, {{p[1], p[2], p[0]}}},
      factor1{key12, key01, {{p[2], p[0], p[1]}}},
      factor2{key20, key12, {{p[0], p[1], p[2]}}};

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
// Test for TransferFactor with multiple tuples
TEST(TransferFactor, MultipleTuples) {
  using namespace fixture;

  // Now project multiple points into the three cameras
  const size_t numPoints = 5;  // Number of points to test
  std::vector<std::array<Point2, 3>> projectedPoints;

  // Generate random 3D points and project them
  for (size_t n = 0; n < numPoints; ++n) {
    Point3 P(0.1 * n, 0.2 * n, 0.3 + 0.1 * n);
    std::array<Point2, 3> p;
    for (size_t i = 0; i < 3; ++i) {
      p[i] = cameras[i].project(P);
    }
    projectedPoints.push_back(p);
  }

  // Create a vector of tuples for the TransferFactor
  EdgeKey key01(0, 1), key12(1, 2), key20(2, 0);
  std::vector<std::tuple<Point2, Point2, Point2>> tuples;

  for (size_t n = 0; n < numPoints; ++n) {
    const auto& p = projectedPoints[n];
    tuples.emplace_back(p[1], p[2], p[0]);
  }

  // Create TransferFactors using the new constructor
  TransferFactor<SimpleFundamentalMatrix> factor{key01, key20, tuples};

  // Create Values with edge keys
  Values values;
  values.insert(key01, triplet.Fab);
  values.insert(key12, triplet.Fbc);
  values.insert(key20, triplet.Fca);

  // Check error and Jacobians for multiple tuples
  Vector error = factor.unwhitenedError(values);
  // The error vector should be of size 2 * numPoints
  EXPECT_LONGS_EQUAL(2 * numPoints, error.size());
  // Since the points are perfectly matched, the error should be zero
  EXPECT(assert_equal<Vector>(Vector::Zero(2 * numPoints), error, 1e-9));

  // Check the Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-7);
}

//*************************************************************************
// Test for EssentialTransferFactorK
TEST(EssentialTransferFactor, Jacobians) {
  using namespace fixture;

  // Create EssentialMatrices between cameras
  EssentialMatrix E01 =
      EssentialMatrix::FromPose3(cameraPoses[0].between(cameraPoses[1]));
  EssentialMatrix E02 =
      EssentialMatrix::FromPose3(cameraPoses[0].between(cameraPoses[2]));

  // Project a point into the three cameras
  const Point3 P(0.1, 0.2, 0.3);
  std::array<Point2, 3> p;
  for (size_t i = 0; i < 3; ++i) {
    p[i] = cameras[i].project(P);
  }

  // Create EdgeKeys
  EdgeKey key01(0, 1);
  EdgeKey key02(0, 2);

  // Create an EssentialTransferFactor
  auto sharedCal = std::make_shared<Cal3_S2>(cal);
  EssentialTransferFactor<Cal3_S2> factor(key01, key02, {{p[1], p[2], p[0]}},
                                          sharedCal);

  // Create Values and insert variables
  Values values;
  values.insert(key01, E01);  // Essential matrix between views 0 and 1
  values.insert(key02, E02);  // Essential matrix between views 0 and 2

  // Check error
  Matrix H1, H2, H3, H4, H5;
  Vector error = factor.evaluateError(E01, E02,  //
                                      &H1, &H2);
  EXPECT(H1.rows() == 2 && H1.cols() == 5)
  EXPECT(H2.rows() == 2 && H2.cols() == 5)
  EXPECT(assert_equal(Vector::Zero(2), error, 1e-9))

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-7);
}

//*************************************************************************
// Test for EssentialTransferFactorK
TEST(EssentialTransferFactorK, Jacobians) {
  using namespace fixture;

  // Create EssentialMatrices between cameras
  EssentialMatrix E01 =
      EssentialMatrix::FromPose3(cameraPoses[0].between(cameraPoses[1]));
  EssentialMatrix E02 =
      EssentialMatrix::FromPose3(cameraPoses[0].between(cameraPoses[2]));

  // Project a point into the three cameras
  const Point3 P(0.1, 0.2, 0.3);
  std::array<Point2, 3> p;
  for (size_t i = 0; i < 3; ++i) {
    p[i] = cameras[i].project(P);
  }

  // Create EdgeKeys
  EdgeKey key01(0, 1);
  EdgeKey key02(0, 2);

  // Create an EssentialTransferFactorK
  EssentialTransferFactorK<Cal3_S2> factor(key01, key02, {{p[1], p[2], p[0]}});

  // Create Values and insert variables
  Values values;
  values.insert(key01, E01);  // Essential matrix between views 0 and 1
  values.insert(key02, E02);  // Essential matrix between views 0 and 2
  values.insert(K(1), cal);   // Calibration for view A (view 1)
  values.insert(K(2), cal);   // Calibration for view B (view 2)
  values.insert(K(0), cal);   // Calibration for view C (view 0)

  // Check error
  Matrix H1, H2, H3, H4, H5;
  Vector error = factor.evaluateError(E01, E02,       //
                                      cal, cal, cal,  //
                                      &H1, &H2, &H3, &H4, &H5);
  EXPECT(H1.rows() == 2 && H1.cols() == 5)
  EXPECT(H2.rows() == 2 && H2.cols() == 5)
  EXPECT(H3.rows() == 2 && H3.cols() == 5)
  EXPECT(H4.rows() == 2 && H4.cols() == 5)
  EXPECT(H5.rows() == 2 && H5.cols() == 5)
  EXPECT(assert_equal(Vector::Zero(2), error, 1e-9))

  // Check Jacobians
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-7);
}

//*************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//*************************************************************************