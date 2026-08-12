/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testWnoaInterpolator.cpp
 * @brief   Unit tests for post-solve interpolator
 * @author  Zi Cong Guo
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/WnoaFactor.h>
#include <gtsam/nonlinear/WnoaInterpolator.h>

using namespace std;
using namespace gtsam;

namespace {

struct WnoaInterpolatorFixture {
  Vector1 Q_p1{0.9};
  Vector2 Q_p2{0.9, 0.8};
  Vector3 Q_p3{0.9, 0.8, 0.7};
  Vector3 Q_se2{0.9, 0.8, 0.7};
  Vector6 Q_se3{0.9, 0.8, 0.7, 0.6, 0.5, 0.4};
  double timestep = 0.1;

  /**** Point1 Test Variables*****/
  Point1 p0_p1{1.0};
  Vector1 v0_p1{1.0};
  Point1 p1_p1{-2.0};
  Vector1 v1_p1{3.0};

  /**** Point2 Test Variables*****/
  Point2 p0_p2{1.0, 2.0};
  Vector2 v0_p2{1.0, 2.0};
  Point2 p1_p2{-3.0, -4.0};
  Vector2 v1_p2{-2.0, -1.0};

  /**** Point3 Test Variables*****/
  Point3 p0_p3{1.0, 2.0, 3.0};
  Vector3 v0_p3{1.0, 2.0, 3.0};
  Point3 p1_p3{-4.0, -5.0, -6.0};
  Vector3 v1_p3{-2.0, -1.0, 0.0};

  /**** SE(2) Test Variables*****/
  Pose2 p0_se2{0.3, 0.2, 0.5};
  Vector3 v0_se2{0.8, 0.4, 0.1};
  Pose2 p1_se2{-0.2, 0.1, 0.6};
  Vector3 v1_se2{0.5, -0.6, 0.2};

  /***** SE(3) Test Variables******/
  Pose3 p0_se3 = Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(0.3, 0.2, 0.1));
  Vector6 v0_se3{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  Pose3 p1_se3 = Pose3(Rot3::RzRyRx(0.4, 0.5, 0.6), Point3(-0.1, -0.2, -0.3));
  Vector6 v1_se3{0.2, -0.1, 0.4, 0.6, -0.5, 0.3};
};

const WnoaInterpolatorFixture& fixture() {
  static const WnoaInterpolatorFixture kFixture;
  return kFixture;
}

const Vector1 Q_p1 = fixture().Q_p1;
const Vector2 Q_p2 = fixture().Q_p2;
const Vector3 Q_p3 = fixture().Q_p3;
const Vector3 Q_se2 = fixture().Q_se2;
const Vector6 Q_se3 = fixture().Q_se3;
const double timestep = fixture().timestep;

const Point1 p0_p1 = fixture().p0_p1;
const Vector1 v0_p1 = fixture().v0_p1;
const Point1 p1_p1 = fixture().p1_p1;
const Vector1 v1_p1 = fixture().v1_p1;

const Point2 p0_p2 = fixture().p0_p2;
const Vector2 v0_p2 = fixture().v0_p2;
const Point2 p1_p2 = fixture().p1_p2;
const Vector2 v1_p2 = fixture().v1_p2;

const Point3 p0_p3 = fixture().p0_p3;
const Vector3 v0_p3 = fixture().v0_p3;
const Point3 p1_p3 = fixture().p1_p3;
const Vector3 v1_p3 = fixture().v1_p3;

const Pose2 p0_se2 = fixture().p0_se2;
const Vector3 v0_se2 = fixture().v0_se2;
const Pose2 p1_se2 = fixture().p1_se2;
const Vector3 v1_se2 = fixture().v1_se2;

const Pose3 p0_se3 = fixture().p0_se3;
const Vector6 v0_se3 = fixture().v0_se3;
const Pose3 p1_se3 = fixture().p1_se3;
const Vector6 v1_se3 = fixture().v1_se3;

}  // namespace

using symbol_shorthand::P;
using symbol_shorthand::V;

// Helper function to generate a diagonally dominant SPD covariance matrix
Matrix randomCovariance(int n) {
  // random symmetric matrix
  Matrix R = Matrix::Random(n, n);
  Matrix sym = (R + R.transpose()) / 2.0;

  // make diagonally dominant
  Matrix cov = sym;
  for (int i = 0; i < n; i++) {
    cov(i, i) = std::abs(sym.row(i).sum()) + 1.0;
  }
  return cov;
}

/* ************************************************************************* */
TEST(Interpolator, Constructor) {
  // Test that constructors work for different types of poses
  Interpolator<Point1> interpolatorP1(Q_p1);
  Interpolator<Point2> interpolatorP2(Q_p2);
  Interpolator<Point3> interpolatorP3(Q_p3);
  Interpolator<Pose2> interpolatorSE2(Q_se2);
  Interpolator<Pose3> interpolatorSE3(Q_se3);
}

/* ************************************************************************* */
/* COVARIANCE TESTS
 * Simple test to check that covariance is computed and has correct size.
 */

TEST(Interpolator, CovarianceP1) {
  Interpolator<Point1> interpolator(Q_p1);
  Matrix covariance;
  // generate a random SPD covariance matrix for (p0, v0, p1, v1)
  Matrix mainSolveMarginal = randomCovariance(4);
  auto mainSolveMarginalPtr = std::make_shared<Matrix>(mainSolveMarginal);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, 0.0),
      TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, timestep), 0.05, nullptr,
      mainSolveMarginalPtr, &covariance);
  CHECK(covariance.rows() == 2 && covariance.cols() == 2);
}
/* ************************************************************************* */
TEST(Interpolator, CovarianceP2) {
  Interpolator<Point2> interpolator(Q_p2);
  Matrix covariance;
  // generate a random SPD covariance matrix for (p0, v0, p1, v1)
  Matrix mainSolveMarginal = randomCovariance(8);
  auto mainSolveMarginalPtr = std::make_shared<Matrix>(mainSolveMarginal);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, 0.0),
      TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, timestep), 0.05, nullptr,
      mainSolveMarginalPtr, &covariance);
  CHECK(covariance.rows() == 4 && covariance.cols() == 4);
}
/* ************************************************************************* */
TEST(Interpolator, CovarianceP3) {
  Interpolator<Point3> interpolator(Q_p3);
  Matrix covariance;
  // generate a random SPD covariance matrix for (p0, v0, p1, v1)
  Matrix mainSolveMarginal = randomCovariance(12);
  auto mainSolveMarginalPtr = std::make_shared<Matrix>(mainSolveMarginal);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0),
      TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep), 0.05, nullptr,
      mainSolveMarginalPtr, &covariance);
  CHECK(covariance.rows() == 6 && covariance.cols() == 6);
}
/* ************************************************************************* */
TEST(Interpolator, CovarianceSE2) {
  Interpolator<Pose2> interpolator(Q_se2);
  Matrix covariance;
  // generate a random SPD covariance matrix for (p0, v0, p1, v1)
  Matrix mainSolveMarginal = randomCovariance(12);
  auto mainSolveMarginalPtr = std::make_shared<Matrix>(mainSolveMarginal);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, 0.0),
      TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, timestep), 0.05, nullptr,
      mainSolveMarginalPtr, &covariance);
  CHECK(covariance.rows() == 6 && covariance.cols() == 6);
}
/* ************************************************************************* */
TEST(Interpolator, CovarianceSE3) {
  Interpolator<Pose3> interpolator(Q_se3);
  Matrix covariance;
  // generate a random SPD covariance matrix for (p0, v0, p1, v1)
  Matrix mainSolveMarginal = randomCovariance(24);
  auto mainSolveMarginalPtr = std::make_shared<Matrix>(mainSolveMarginal);
  (void)interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
      TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, timestep), 0.05, nullptr,
      mainSolveMarginalPtr, &covariance);
  CHECK(covariance.rows() == 12 && covariance.cols() == 12);
}

/* ************************************************************************* */
/* COMMON-VELOCITY TESTS
 * These tests check that if the velocities are the same at the two endpoints,
 * and the second pose is obtained by applying the common velocity to the first,
 * then the interpolated velocity should be the same as well, and the pose
 * should be linear interpolation in the tangent space.
 */

TEST(Interpolator, InterpolatePoseAndVelocityP1) {
  Interpolator<Point1> interpolator(Q_p1);
  Vector1 v_common(2.0);
  Point1 p0_p1_common_v = p0_p1;
  Point1 p1_p1_common_v = p0_p1_common_v + timestep * v_common;
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_p1_common_v, v_common, 0.0),
        TimestampedPoseVelocity<Point1>(p1_p1_common_v, v_common, timestep),
        timestep * ratio);
    Point1 expectedPose = p0_p1_common_v + ratio * timestep * v_common;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v_common, pvtau.vel));
  }
}
/* ************************************************************************* */
TEST(Interpolator, InterpolatePoseAndVelocityP2) {
  Interpolator<Point2> interpolator(Q_p2);
  Vector2 v_common(0.5, -0.5);
  Point2 p0_p2_common_v = p0_p2;
  Point2 p1_p2_common_v = p0_p2_common_v + timestep * v_common;
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_p2_common_v, v_common, 0.0),
        TimestampedPoseVelocity<Point2>(p1_p2_common_v, v_common, timestep),
        timestep * ratio);
    Point2 expectedPose = p0_p2_common_v + ratio * timestep * v_common;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v_common, pvtau.vel));
  }
}
/* ************************************************************************* */
TEST(Interpolator, InterpolatePoseAndVelocityP3) {
  Interpolator<Point3> interpolator(Q_p3);
  Vector3 v_common(0.5, -0.5, 0.5);
  Point3 p0_p3_common_v = p0_p3;
  Point3 p1_p3_common_v = p0_p3_common_v + timestep * v_common;
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_p3_common_v, v_common, 0.0),
        TimestampedPoseVelocity<Point3>(p1_p3_common_v, v_common, timestep),
        timestep * ratio);
    Point3 expectedPose = p0_p3_common_v + ratio * timestep * v_common;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v_common, pvtau.vel));
  }
}
/* ************************************************************************* */

TEST(Interpolator, InterpolatePoseAndVelocitySE2) {
  Interpolator<Pose2> interpolator(Q_se2);
  Vector3 v_common(0.65, -0.1, 0.15);
  Pose2 p0_se2_common_v = p0_se2;
  Pose2 p1_se2_common_v = p0_se2_common_v.expmap(timestep * v_common);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2_common_v, v_common, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_se2_common_v, v_common, timestep),
        timestep * ratio);
    Pose2 expectedPose = p0_se2_common_v.expmap(ratio * timestep * v_common);
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v_common, pvtau.vel));
  }
}

/* ************************************************************************* */
TEST(Interpolator, InterpolatePoseAndVelocitySE3) {
  Interpolator<Pose3> interpolator(Q_se3);
  Vector6 v_common(0.15, -0.25, 0.35, 0.45, -0.55, 0.65);
  Pose3 p0_se3_common_v = p0_se3;
  Pose3 p1_se3_common_v = p0_se3_common_v.expmap(timestep * v_common);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3_common_v, v_common, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_se3_common_v, v_common, timestep),
        timestep * ratio);
    Pose3 expectedPose = p0_se3_common_v.expmap(ratio * timestep * v_common);
    double tol = 1e-8;  // larger tolerance since Lie groups have approximations
    CHECK(assert_equal(expectedPose, pvtau.pose, tol));
    CHECK(assert_equal(v_common, pvtau.vel, tol));
  }
}

/* ************************************************************************* */
/* EXTRAPOLATION TESTS
 * These tests check that extrapolation works correctly when the query time is
 * outside the interval. The extrapolated pose should be the same as if we
 * applied the velocity to the last pose.
 */
TEST(Interpolator, ExtrapolatePoseAndVelocityP1) {
  Interpolator<Point1> interpolator(Q_p1);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, 0.0),
        TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, timestep),
        -timestep * ratio);
    Point1 expectedPose = p0_p1 + (-ratio) * timestep * v0_p1;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v0_p1, pvtau.vel));

    pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, -timestep),
        TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, 0.0), timestep * ratio);
    expectedPose = p1_p1 + (ratio)*timestep * v1_p1;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v1_p1, pvtau.vel));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ExtrapolatePoseAndVelocityP2) {
  Interpolator<Point2> interpolator(Q_p2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, 0.0),
        TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, timestep),
        -timestep * ratio);
    Point2 expectedPose = p0_p2 + (-ratio) * timestep * v0_p2;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v0_p2, pvtau.vel));

    pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, -timestep),
        TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, 0.0), timestep * ratio);
    expectedPose = p1_p2 + (ratio)*timestep * v1_p2;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v1_p2, pvtau.vel));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ExtrapolatePoseAndVelocityP3) {
  Interpolator<Point3> interpolator(Q_p3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0),
        TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep),
        -timestep * ratio);
    Point3 expectedPose = p0_p3 + (-ratio) * timestep * v0_p3;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v0_p3, pvtau.vel));

    pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, -timestep),
        TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, 0.0), timestep * ratio);
    expectedPose = p1_p3 + (ratio)*timestep * v1_p3;
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v1_p3, pvtau.vel));
  }
}

/* ************************************************************************* */
TEST(Interpolator, ExtrapolatePoseAndVelocitySE2) {
  Interpolator<Pose2> interpolator(Q_se2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, timestep),
        -timestep * ratio);
    Pose2 expectedPose = p0_se2.expmap(-ratio * timestep * v0_se2);
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v0_se2, pvtau.vel));

    pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, -timestep),
        TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, 0.0), timestep * ratio);
    expectedPose = p1_se2.expmap(ratio * timestep * v1_se2);
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v1_se2, pvtau.vel));
  }
}

/* ************************************************************************* */
TEST(Interpolator, ExtrapolatePoseAndVelocitySE3) {
  Interpolator<Pose3> interpolator(Q_se3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, timestep),
        -timestep * ratio);
    Pose3 expectedPose = p0_se3.expmap(-ratio * timestep * v0_se3);
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v0_se3, pvtau.vel));

    pvtau = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, -timestep),
        TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, 0.0), timestep * ratio);
    expectedPose = p1_se3.expmap(ratio * timestep * v1_se3);
    CHECK(assert_equal(expectedPose, pvtau.pose));
    CHECK(assert_equal(v1_se3, pvtau.vel));
  }
}

/* ************************************************************************* */
/* FORWARD-BACKWARD TESTS
 * These tests check that interpolating forward is the same as interpolating
 * backward with negative velocities. Should be exact for vector spaces (Point1,
 * Point2, Point3), and approximate for Lie groups (Pose2, Pose3).
 */
TEST(Interpolator, ForwardBackwardP1) {
  Interpolator<Point1> interpolator(Q_p1);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, 0.0),
        TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, timestep),
        timestep * ratio);
    // swap poses and velocities, make velocities negative
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p1_p1, -v1_p1, 0.0),
        TimestampedPoseVelocity<Point1>(p0_p1, -v0_p1, timestep),
        timestep * (1 - ratio));
    double tol = 1e-8;
    CHECK(assert_equal(pvtau1.pose, pvtau2.pose, tol));
    CHECK(assert_equal(pvtau1.vel, (-pvtau2.vel).eval(), tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ForwardBackwardP2) {
  Interpolator<Point2> interpolator(Q_p2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, 0.0),
        TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, timestep),
        timestep * ratio);
    // swap poses and velocities, make velocities negative
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p1_p2, -v1_p2, 0.0),
        TimestampedPoseVelocity<Point2>(p0_p2, -v0_p2, timestep),
        timestep * (1 - ratio));
    double tol = 1e-8;
    CHECK(assert_equal(pvtau1.pose, pvtau2.pose, tol));
    CHECK(assert_equal(pvtau1.vel, (-pvtau2.vel).eval(), tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ForwardBackwardP3) {
  Interpolator<Point3> interpolator(Q_p3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0),
        TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep),
        timestep * ratio);
    // swap poses and velocities, make velocities negative
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p1_p3, -v1_p3, 0.0),
        TimestampedPoseVelocity<Point3>(p0_p3, -v0_p3, timestep),
        timestep * (1 - ratio));
    double tol = 1e-8;
    CHECK(assert_equal(pvtau1.pose, pvtau2.pose, tol));
    CHECK(assert_equal(pvtau1.vel, (-pvtau2.vel).eval(), tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ForwardBackwardSE2) {
  Interpolator<Pose2> interpolator(Q_se2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, timestep),
        timestep * ratio);
    // swap poses and velocities, make velocities negative
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p1_se2, -v1_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p0_se2, -v0_se2, timestep),
        timestep * (1 - ratio));
    double tol = 1e-3;  // larger tolerance since Lie groups have approximations
    CHECK(assert_equal(pvtau1.pose, pvtau2.pose, tol));
    CHECK(assert_equal(pvtau1.vel, (-pvtau2.vel).eval(), tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, ForwardBackwardSE3) {
  Interpolator<Pose3> interpolator(Q_se3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, timestep),
        timestep * ratio);
    // swap poses and velocities, make velocities negative
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p1_se3, -v1_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p0_se3, -v0_se3, timestep),
        timestep * (1 - ratio));
    double tol = 1e-2;  // larger tolerance since Lie groups have approximations
    CHECK(assert_equal(pvtau1.pose, pvtau2.pose, tol));
    CHECK(assert_equal(pvtau1.vel, (-pvtau2.vel).eval(), tol));
  }
}

/* ************************************************************************* */
/* FRAME TRANSFORMATION TESTS
 * These tests check that the interpolator is agnostic to the frame of
 * reference. The test is done by transforming the poses and velocities to a
 * different frame and checking if the interpolation is consistent. Note: we
 * separate rotation and translation in SE2 and SE3 to avoid coupling effects.
 */
TEST(Interpolator, FrameTranslationP1) {
  Point1 T_frame(0.5);  // simple translation
  Point1 p0_transformed = p0_p1 + T_frame;
  Point1 p1_transformed = p1_p1 + T_frame;
  Interpolator<Point1> interpolator(Q_p1);

  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, 0.0),
        TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, timestep),
        timestep * ratio);

    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point1>(p0_transformed, v0_p1, 0.0),
        TimestampedPoseVelocity<Point1>(p1_transformed, v1_p1, timestep),
        timestep * ratio);

    // Transform pvtau2 back to the original frame
    Point1 pvtau2_pose = pvtau2.pose - T_frame;
    Vector1 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant

    double tol = 1e-8;  // should be exact for Point1
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, FrameTranslationP2) {
  Point2 T_frame(0.5, -0.3);  // simple translation
  Point2 p0_transformed = p0_p2 + T_frame;
  Point2 p1_transformed = p1_p2 + T_frame;
  Interpolator<Point2> interpolator(Q_p2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, 0.0),
        TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, timestep),
        timestep * ratio);

    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point2>(p0_transformed, v0_p2, 0.0),
        TimestampedPoseVelocity<Point2>(p1_transformed, v1_p2, timestep),
        timestep * ratio);
    // Transform pvtau2 back to the original frame
    Point2 pvtau2_pose = pvtau2.pose - T_frame;
    Vector2 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant
    double tol = 1e-8;                     // should be exact for Point2
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, FrameTranslationP3) {
  Point3 T_frame(0.5, -0.3, 0.2);  // simple translation
  Point3 p0_transformed = p0_p3 + T_frame;
  Point3 p1_transformed = p1_p3 + T_frame;
  Interpolator<Point3> interpolator(Q_p3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0),
        TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep),
        timestep * ratio);
    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Point3>(p0_transformed, v0_p3, 0.0),
        TimestampedPoseVelocity<Point3>(p1_transformed, v1_p3, timestep),
        timestep * ratio);
    // Transform pvtau2 back to the original frame
    Point3 pvtau2_pose = pvtau2.pose - T_frame;
    Vector3 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant
    double tol = 1e-8;                     // should be exact for Point3
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, FrameTranslationSE2) {
  Point2 T_frame(0.5, -0.3);  // simple translation
  Pose2 p0_transformed = Pose2(Rot2::Identity(), T_frame).compose(p0_se2);
  Pose2 p1_transformed = Pose2(Rot2::Identity(), T_frame).compose(p1_se2);
  Interpolator<Pose2> interpolator(Q_se2);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, timestep),
        timestep * ratio);
    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_transformed, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_transformed, v1_se2, timestep),
        timestep * ratio);
    // Transform pvtau2 back to the original frame
    Pose2 pvtau2_pose = Pose2(Rot2::Identity(), -T_frame).compose(pvtau2.pose);
    Vector3 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant
    double tol = 1e-8;  // should be exact for SE(2) translation
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, FrameTranslationSE3) {
  Point3 T_frame(0.5, -0.3, 0.2);  // simple translation
  Pose3 p0_transformed = Pose3(Rot3::Identity(), T_frame).compose(p0_se3);
  Pose3 p1_transformed = Pose3(Rot3::Identity(), T_frame).compose(p1_se3);
  Interpolator<Pose3> interpolator(Q_se3);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, timestep),
        timestep * ratio);
    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_transformed, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_transformed, v1_se3, timestep),
        timestep * ratio);
    // Transform pvtau2 back to the original frame
    Pose3 pvtau2_pose = Pose3(Rot3::Identity(), -T_frame).compose(pvtau2.pose);
    Vector6 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant
    double tol = 1e-8;  // should be exact for SE(3) translation
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}
/* ************************************************************************* */
TEST(Interpolator, FrameRotationSE2) {
  Pose2 T_frame(Rot2(0.5), Point2(0.0, 0.0));
  Pose2 p0_transformed = T_frame.compose(p0_se2);
  Pose2 p1_transformed = T_frame.compose(p1_se2);
  Vector3 Q_se2_isotropic = Vector3::Ones() * 0.8;  // isotropic noise for SE(2)
  // Use an isotropic noise to avoid a non-isotropic noise in the new frame
  Interpolator<Pose2> interpolator(Q_se2_isotropic);

  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_se2, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_se2, v1_se2, timestep),
        timestep * ratio);

    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose2>(p0_transformed, v0_se2, 0.0),
        TimestampedPoseVelocity<Pose2>(p1_transformed, v1_se2, timestep),
        timestep * ratio);

    // Transform pvtau2 back to the original frame
    Pose2 pvtau2_pose = T_frame.inverse().compose(pvtau2.pose);
    Vector3 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant

    double tol = 1e-3;  // larger tolerance since Lie groups have approximations
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}

/* ************************************************************************* */
TEST(Interpolator, FrameRotationSE3) {
  Pose3 T_frame(Rot3::RzRyRx(0.3, 0.2, 0.1), Point3(0.0, 0.0, 0.0));
  Pose3 p0_transformed = T_frame.compose(p0_se3);
  Pose3 p1_transformed = T_frame.compose(p1_se3);
  Vector6 Q_se3_isotropic = Vector6::Ones() * 0.8;  // isotropic noise for SE(3)
  // Use an isotropic noise to avoid a non-isotropic noise in the new frame
  Interpolator<Pose3> interpolator(Q_se3_isotropic);
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    // pvtau1: Interpolate in the original frame
    auto pvtau1 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_se3, v1_se3, timestep),
        timestep * ratio);

    // pvtau2: Interpolate in the transformed frame
    auto pvtau2 = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0_transformed, v0_se3, 0.0),
        TimestampedPoseVelocity<Pose3>(p1_transformed, v1_se3, timestep),
        timestep * ratio);
    // Transform pvtau2 back to the original frame
    Pose3 pvtau2_pose = T_frame.inverse().compose(pvtau2.pose);
    Vector6 pvtau2_velocity = pvtau2.vel;  // body-frame velocity is invariant
    double tol = 1e-2;  // larger tolerance since Lie groups have approximations
    CHECK(assert_equal(pvtau1.pose, pvtau2_pose, tol));
    CHECK(assert_equal(pvtau1.vel, pvtau2_velocity, tol));
  }
}

/* *************************************************************************
 */

TEST(Interpolator, PoseJacobians) {
  // Redefine poses along a smooth trajectory
  const Pose3 p0_se3 = Pose3::Expmap(Vector6(0.5, 0.0, 0.0, 0.0, 0.0, 0.0));
  const Vector6 v0_se3(1, 0.0, 0.5, 0.1, 0.0, 0.0);
  const Pose3 p1_se3 = p0_se3.expmap(timestep * v0_se3);
  // const Vector6 v1_se3 = v0_se3;
  const Pose3 p2_se3 = p0_se3.expmap(2 * timestep * v0_se3);
  const Vector6 v2_se3 = v0_se3;

  // Create Interpolator
  Interpolator<Pose3> interpolator(Vector6::Ones());

  // Get analytic Jacobians
  vector<Matrix> H(8);
  auto [pose_est, vel_est] = interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
      TimestampedPoseVelocity<Pose3>(p2_se3, v2_se3, 2 * timestep), timestep,
      &H);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p2, auto& v2) {
    auto result = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0, v0, 0.0),
        TimestampedPoseVelocity<Pose3>(p2, v2, 2 * timestep), timestep);

    return p1_se3.logmap(result.pose);
  };

  // Compute numerical derivatives
  double delta = 1e-6;
  Matrix J_p0_num =
      numericalDerivative41<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_v0_num =
      numericalDerivative42<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_p2_num =
      numericalDerivative43<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_v2_num =
      numericalDerivative44<Vector, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);

  double tol = 1e-3;
  EXPECT(assert_equal(J_p0_num, H[0], tol));
  EXPECT(assert_equal(J_v0_num, H[1], tol));
  EXPECT(assert_equal(J_p2_num, H[2], tol));
  EXPECT(assert_equal(J_v2_num, H[3], tol));
}

/* *************************************************************************
 */

TEST(Interpolator, VelJacobians) {
  // Redefine poses along a smooth trajectory
  const Pose3 p0_se3 = Pose3::Expmap(Vector6(0.5, 0.0, 0.0, 0.0, 0.0, 0.0));
  const Vector6 v0_se3(1, 0.1, 0.5, 0.1, 0.1, 0.1);
  // const Pose3 p1_se3 = p0_se3.expmap(timestep * v0_se3);
  const Vector6 v1_se3 = v0_se3;
  const Pose3 p2_se3 = p0_se3.expmap(2 * timestep * v0_se3);
  const Vector6 v2_se3 = v0_se3;

  // Create Interpolator
  Interpolator<Pose3> interpolator(Vector6::Ones());

  // Get analytic Jacobians
  vector<Matrix> H(8);
  auto [pose_est, vel_est] = interpolator.interpolatePoseAndVelocity(
      TimestampedPoseVelocity<Pose3>(p0_se3, v0_se3, 0.0),
      TimestampedPoseVelocity<Pose3>(p2_se3, v2_se3, 2 * timestep), timestep,
      &H);

  // define lambda function for derivatives
  auto f = [&](auto& p0, auto& v0, auto& p2, auto& v2) {
    auto result = interpolator.interpolatePoseAndVelocity(
        TimestampedPoseVelocity<Pose3>(p0, v0, 0.0),
        TimestampedPoseVelocity<Pose3>(p2, v2, 2 * timestep), timestep);
    Vector6 err = result.vel - v1_se3;
    return err;
  };

  // Compute numerical derivatives
  double delta = 1e-5;  // delta
  double tol = 1e-2;    // tolerance
  Matrix J_p0_num =
      numericalDerivative41<Vector6, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_v0_num =
      numericalDerivative42<Vector6, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_p2_num =
      numericalDerivative43<Vector6, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  Matrix J_v2_num =
      numericalDerivative44<Vector6, Pose3, Vector6, Pose3, Vector6>(
          f, p0_se3, v0_se3, p2_se3, v2_se3, delta);
  EXPECT(assert_equal(J_v0_num, H[5], tol));
  EXPECT(assert_equal(J_p2_num, H[6], tol));
  EXPECT(assert_equal(J_p0_num, H[4], tol));
  EXPECT(assert_equal(J_v2_num, H[7], tol));
}

/* ************************************************************************* */
/* LAMBDA AND PSI TESTS
check that computation of psi and lambda using Eq. (11.41) in the book is
consistent with Eq. (5.23) in the paper. */

TEST(Interpolator, LambdaPsiConsistencyP1) {
  Interpolator<Point1> interpolator(Q_p1);
  for (double ratio = 0.1; ratio <= 0.9; ratio += 0.1) {
    // get lambda and psi using Eq. (11.41) in the book
    auto [Lambda_book, Psi_book] =
        interpolator.getLambdaPsi(0.0, timestep, timestep * ratio);

    // get lambda and psi using Eq. (5.23) in the paper
    auto tpvk = TimestampedPoseVelocity<Point1>(p0_p1, v0_p1, 0.0);
    auto tpvkp1 = TimestampedPoseVelocity<Point1>(p1_p1, v1_p1, timestep);
    auto pvtau =
        interpolator.interpolatePoseAndVelocity(tpvk, tpvkp1, timestep * ratio);
    auto tpvtau = TimestampedPoseVelocity<Point1>(pvtau, timestep * ratio);
    Matrix Lambda_paper, Psi_paper;
    (void)interpolator.computeConditionalCov(tpvk, tpvkp1, tpvtau,
                                             &Lambda_paper, &Psi_paper);
    EXPECT(assert_equal(Lambda_paper, Lambda_book));
    EXPECT(assert_equal(Psi_paper, Psi_book));
  }
}

/* ************************************************************************* */
TEST(Interpolator, LambdaPsiConsistencyP2) {
  Interpolator<Point2> interpolator(Q_p2);
  for (double ratio = 0.1; ratio <= 0.9; ratio += 0.1) {
    // get lambda and psi using Eq. (11.41) in the book
    auto [Lambda_book, Psi_book] =
        interpolator.getLambdaPsi(0.0, timestep, timestep * ratio);

    // get lambda and psi using Eq. (5.23) in the paper
    auto tpvk = TimestampedPoseVelocity<Point2>(p0_p2, v0_p2, 0.0);
    auto tpvkp1 = TimestampedPoseVelocity<Point2>(p1_p2, v1_p2, timestep);
    auto pvtau =
        interpolator.interpolatePoseAndVelocity(tpvk, tpvkp1, timestep * ratio);
    auto tpvtau = TimestampedPoseVelocity<Point2>(pvtau, timestep * ratio);
    Matrix Lambda_paper, Psi_paper;
    (void)interpolator.computeConditionalCov(tpvk, tpvkp1, tpvtau,
                                             &Lambda_paper, &Psi_paper);
    EXPECT(assert_equal(Lambda_paper, Lambda_book));
    EXPECT(assert_equal(Psi_paper, Psi_book));
  }
}

/* ************************************************************************* */
TEST(Interpolator, LambdaPsiConsistencyP3) {
  Interpolator<Point3> interpolator(Q_p3);
  for (double ratio = 0.1; ratio <= 0.9; ratio += 0.1) {
    // get lambda and psi using Eq. (11.41) in the book
    auto [Lambda_book, Psi_book] =
        interpolator.getLambdaPsi(0.0, timestep, timestep * ratio);

    // get lambda and psi using Eq. (5.23) in the paper
    auto tpvk = TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0);
    auto tpvkp1 = TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep);
    auto pvtau =
        interpolator.interpolatePoseAndVelocity(tpvk, tpvkp1, timestep * ratio);
    auto tpvtau = TimestampedPoseVelocity<Point3>(pvtau, timestep * ratio);
    Matrix Lambda_paper, Psi_paper;
    (void)interpolator.computeConditionalCov(tpvk, tpvkp1, tpvtau,
                                             &Lambda_paper, &Psi_paper);
    EXPECT(assert_equal(Lambda_paper, Lambda_book));
    EXPECT(assert_equal(Psi_paper, Psi_book));
  }
}

/* ************************************************************************* */
TEST(Interpolator, LambdaPsiExternal) {
  Interpolator<Point3> interpolator(Q_p3);
  using MatPair = Interpolator<Point3>::LambdaPsiMats;
  double ratio = 0.5;
  // get lambda and psi
  auto lambda_psi_ptr = std::make_shared<MatPair>(
      interpolator.getLambdaPsi(0.0, timestep, timestep * ratio));

  // get lambda and psi using Eq. (5.23) in the paper
  auto tpvk = TimestampedPoseVelocity<Point3>(p0_p3, v0_p3, 0.0);
  auto tpvkp1 = TimestampedPoseVelocity<Point3>(p1_p3, v1_p3, timestep);
  auto pvtau =
      interpolator.interpolatePoseAndVelocity(tpvk, tpvkp1, timestep * ratio);
  auto pvtau_alt = interpolator.interpolatePoseAndVelocity(
      tpvk, tpvkp1, timestep * ratio, nullptr, nullptr, nullptr,
      lambda_psi_ptr);

  EXPECT(assert_equal(pvtau.pose, pvtau_alt.pose));
  EXPECT(assert_equal(pvtau.vel, pvtau_alt.vel));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
