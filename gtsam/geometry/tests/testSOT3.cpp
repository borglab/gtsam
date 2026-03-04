/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSOT3.cpp
 * @brief  Unit tests for SOT3, the scaled orthogonal transforms SO(3) x R>0
 * @author Rohan Bansal
 * @date   2026
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/SOT3.h>

#include <cmath>
#include <functional>

using namespace std;
using namespace gtsam;


namespace {
const SOT3 id;

// base scenario, a small rotation around z-axis and scale = 2
const Vector3 z_axis(0, 0, 1);
const SOT3 Q1(SO3(Eigen::AngleAxisd(0.1, z_axis)), 2.0);
const SOT3 Q2(SO3(Eigen::AngleAxisd(0.2, z_axis)), 3.0);
const SOT3 Q3(SO3(Eigen::AngleAxisd(0.3, Vector3(1, 1, 0).normalized())), 0.5);
}  // namespace

//******************************************************************************
TEST(SOT3, Identity) {
  EXPECT_LONGS_EQUAL(4, SOT3::dimension);
  EXPECT_LONGS_EQUAL(4, SOT3::Dim());
  EXPECT_LONGS_EQUAL(4, id.dim());
  EXPECT_LONGS_EQUAL(4, traits<SOT3>::GetDimension(id));
  EXPECT_LONGS_EQUAL(4, id.matrix().rows());
  EXPECT_LONGS_EQUAL(4, id.matrix().cols());

  // Identity matrix should be 4x4 identity
  EXPECT(assert_equal(Matrix(Eigen::Matrix4d::Identity()), Matrix(id.matrix())));
}

//******************************************************************************
TEST(SOT3, Constructors) {
  // from (R, c)
  const SO3 R(Eigen::AngleAxisd(0.3, z_axis));
  const SOT3 Q(R, 1.5);
  EXPECT(assert_equal(R, Q.rotation()));
  DOUBLES_EQUAL(1.5, Q.scalar(), 1e-9);

  // from 4x4 matrix
  const Eigen::Matrix4d M = Q.matrix();
  const SOT3 Q2_from_mat(M);
  EXPECT(assert_equal(Q, Q2_from_mat));
}

//******************************************************************************
TEST(SOT3, Matrix) {
  const SO3 R(Eigen::AngleAxisd(0.5, Vector3(0, 1, 0)));
  const double c = 2.5;
  const SOT3 Q(R, c);
  const Eigen::Matrix4d M = Q.matrix();

  // top-left 3x3 should be rot matrix
  EXPECT(assert_equal(Matrix(R.matrix()), Matrix(M.topLeftCorner<3, 3>())));
  // top-right should be zero
  EXPECT(assert_equal(Vector(Vector3::Zero()), Vector(M.topRightCorner<3, 1>())));
  // bottom-left should be zero
  EXPECT(assert_equal(Matrix(Eigen::RowVector3d::Zero()),
                      Matrix(M.bottomLeftCorner<1, 3>())));
  // bottom-right scalar should be c
  DOUBLES_EQUAL(c, M(3, 3), 1e-9);
}

//******************************************************************************
TEST(SOT3, HatVee) {
  // goal is to test round-trip: Vee(Hat(xi)) == xi
  Vector4 xi1;
  xi1 << 0.1, 0.2, -0.3, 0.5;
  EXPECT(assert_equal(xi1, SOT3::Vee(SOT3::Hat(xi1))));

  Vector4 xi2;
  xi2 << 0.0, 0.0, 0.0, 1.0;
  EXPECT(assert_equal(xi2, SOT3::Vee(SOT3::Hat(xi2))));

  Vector4 xi3;
  xi3 << -1.0, 2.0, 0.5, -0.2;
  EXPECT(assert_equal(xi3, SOT3::Vee(SOT3::Hat(xi3))));

  Vector4 xi4;
  xi4 << 1.0, 0.0, 0.0, 0.7;
  const Eigen::Matrix4d X = SOT3::Hat(xi4);
  // check skew-symmetry of 3x3 block
  EXPECT(assert_equal(Matrix3(-X.topLeftCorner<3, 3>()),
                      Matrix3(X.topLeftCorner<3, 3>().transpose())));
  // check scalar entry
  DOUBLES_EQUAL(0.7, X(3, 3), 1e-9);
  // check off-diag 4th row/col are zero
  EXPECT(assert_equal(Vector(Vector3::Zero()), Vector(X.topRightCorner<3, 1>())));
  EXPECT(assert_equal(Matrix(Eigen::RowVector3d::Zero()),
                      Matrix(X.bottomLeftCorner<1, 3>())));
}

//******************************************************************************
TEST(SOT3, ExpmapLogmap) {
  // test round-trip: Logmap(Expmap(xi)) == xi
  Vector4 xi;
  xi << 0.1, 0.2, -0.3, 0.5;
  EXPECT(assert_equal(xi, SOT3::Logmap(SOT3::Expmap(xi)), 1e-9));

  // test round-trip: Expmap(Logmap(Q)) == Q
  EXPECT(assert_equal(Q1, SOT3::Expmap(SOT3::Logmap(Q1)), 1e-9));
  EXPECT(assert_equal(Q2, SOT3::Expmap(SOT3::Logmap(Q2)), 1e-9));
  EXPECT(assert_equal(Q3, SOT3::Expmap(SOT3::Logmap(Q3)), 1e-9));

  // identity: exp(0) = identity
  EXPECT(assert_equal(id, SOT3::Expmap(Vector4::Zero()), 1e-9));

  // scale only: Expmap(0, 0, 0, s) = (I, exp(s))
  Vector4 xi_scale;
  xi_scale << 0, 0, 0, std::log(2.0);
  const SOT3 Q_scale = SOT3::Expmap(xi_scale);
  EXPECT(assert_equal(SO3(), Q_scale.rotation(), 1e-9));
  DOUBLES_EQUAL(2.0, Q_scale.scalar(), 1e-9);
}

//******************************************************************************
TEST(SOT3, Multiply) {
  // (R1,c1)*(R2,c2) = (R1*R2, c1*c2)
  const SOT3 Q12 = Q1 * Q2;
  EXPECT(assert_equal(Q1.rotation() * Q2.rotation(), Q12.rotation(), 1e-9));
  DOUBLES_EQUAL(Q1.scalar() * Q2.scalar(), Q12.scalar(), 1e-9);

  EXPECT(assert_equal(Q1, id * Q1));
  EXPECT(assert_equal(Q1, Q1 * id));
}

//******************************************************************************
TEST(SOT3, Inverse) {
  // Q * Q^{-1} = identity
  EXPECT(assert_equal(id, Q1 * Q1.inverse(), 1e-9));
  EXPECT(assert_equal(id, Q2 * Q2.inverse(), 1e-9));
  EXPECT(assert_equal(id, Q3 * Q3.inverse(), 1e-9));

  // Q^{-1} * Q = identity
  EXPECT(assert_equal(id, Q1.inverse() * Q1, 1e-9));

  const SOT3 Qi = Q1.inverse();
  EXPECT(assert_equal(Q1.rotation().inverse(), Qi.rotation(), 1e-9));
  DOUBLES_EQUAL(1.0 / Q1.scalar(), Qi.scalar(), 1e-9);
}

//******************************************************************************
TEST(SOT3, Action) {
  // Qp = c * R * p
  const Vector3 p(1.0, 0.0, 0.0);
  const SO3 R(Eigen::AngleAxisd(M_PI / 2.0, z_axis));
  const SOT3 Q(R, 3.0);

  const Vector3 expected = 3.0 * (R.matrix() * p);
  EXPECT(assert_equal(expected, Q * p, 1e-9));

  EXPECT(assert_equal(p, Q.applyInverse(Q * p), 1e-9));
  EXPECT(assert_equal(p, Q * Q.applyInverse(p), 1e-9));
}

//******************************************************************************
TEST(SOT3, AdjointMap) {
  // Ad_Q should be block-diagonal: [[R, 0], [0, 1]]
  const SOT3 Q(SO3(Eigen::AngleAxisd(0.4, Vector3(1, 0, 0).normalized())), 2.0);
  const Eigen::Matrix4d Ad = Q.AdjointMap();

  // top-left 3x3 = R
  EXPECT(assert_equal(Q.rotation().matrix(), Matrix3(Ad.topLeftCorner<3, 3>())));
  // cross terms = 0
  EXPECT(assert_equal(Vector(Vector3::Zero()), Vector(Ad.topRightCorner<3, 1>())));
  EXPECT(assert_equal(Matrix(Eigen::RowVector3d::Zero()),
                      Matrix(Ad.bottomLeftCorner<1, 3>())));
  // bottom-right = 1
  DOUBLES_EQUAL(1.0, Ad(3, 3), 1e-9);

  // consistency check with generic MatrixLieGroup AdjointMap
  const Eigen::Matrix4d Ad_generic =
      static_cast<const MatrixLieGroup<SOT3, 4, 4>*>(&Q)->AdjointMap();
  EXPECT(assert_equal(Ad_generic, Ad, 1e-9));
}

//******************************************************************************
TEST(SOT3, Invariants) {
  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, Q1));
  EXPECT(check_group_invariants(Q1, id));
  EXPECT(check_group_invariants(Q1, Q2));
  EXPECT(check_group_invariants(Q2, Q3));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, Q1));
  EXPECT(check_manifold_invariants(Q1, id));
  EXPECT(check_manifold_invariants(Q1, Q2));
  EXPECT(check_manifold_invariants(Q2, Q3));
}

//******************************************************************************
TEST(SOT3, LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, Q1);
  CHECK_LIE_GROUP_DERIVATIVES(Q1, id);
  CHECK_LIE_GROUP_DERIVATIVES(Q1, Q2);
}

//******************************************************************************
TEST(SOT3, ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(id, id);
  CHECK_CHART_DERIVATIVES(id, Q1);
  CHECK_CHART_DERIVATIVES(Q1, id);
  CHECK_CHART_DERIVATIVES(Q1, Q2);
}

//******************************************************************************
TEST(SOT3, ExpmapJacobian) {
  // test d(Expmap)/d(xi) against numerical derivative
  Vector4 xi;
  xi << 0.1, 0.2, -0.3, 0.4;

  Matrix4 H_analytical;
  SOT3::Expmap(xi, H_analytical);

  const std::function<SOT3(const Vector4&)> expmap_func =
      [](const Vector4& v) { return SOT3::Expmap(v); };
  const Matrix H_numerical = numericalDerivative11<SOT3, Vector4>(
      expmap_func, xi);

  EXPECT(assert_equal(H_numerical, H_analytical, 1e-5));
}

//******************************************************************************
TEST(SOT3, LogmapJacobian) {
  // test d(Logmap)/d(Q) against numerical derivative, evaluated at Q1
  Matrix4 H_analytical;
  SOT3::Logmap(Q1, H_analytical);

  const std::function<Vector4(const SOT3&)> logmap_func =
      [](const SOT3& Q) { return SOT3::Logmap(Q); };
  const Matrix H_numerical = numericalDerivative11<Vector4, SOT3>(
      logmap_func, Q1);

  EXPECT(assert_equal(H_numerical, H_analytical, 1e-5));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
