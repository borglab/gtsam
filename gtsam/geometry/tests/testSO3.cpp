/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testSO3.cpp
 * @brief  Unit tests for SO3, as a GTSAM-adapted Lie Group
 * @author Frank Dellaert
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/SO3.h>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

//******************************************************************************
TEST(SO3, Identity) {
  const SO3 R;
  EXPECT_LONGS_EQUAL(3, R.rows());
  EXPECT_LONGS_EQUAL(3, SO3::dimension);
  EXPECT_LONGS_EQUAL(3, SO3::Dim());
  EXPECT_LONGS_EQUAL(3, R.dim());
  EXPECT_LONGS_EQUAL(3, traits<SO3>::GetDimension(R));
}

//******************************************************************************
TEST(SO3, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<SO3>)
  GTSAM_CONCEPT_ASSERT(IsManifold<SO3>)
  GTSAM_CONCEPT_ASSERT(IsLieGroup<SO3>)
}

//******************************************************************************
TEST(SO3, Constructors) {
  const Vector3 axis(0, 0, 1);
  const double angle = 2.5;
  SO3 Q(Eigen::AngleAxisd(angle, axis));
  SO3 R = SO3::AxisAngle(axis, angle);
  EXPECT(assert_equal(Q, R));
}

/* ************************************************************************* */
TEST(SO3, ClosestTo) {
  Matrix3 M;
  M << 0.79067393, 0.6051136, -0.0930814,   //
      0.4155925, -0.64214347, -0.64324489,  //
      -0.44948549, 0.47046326, -0.75917576;

  Matrix expected(3, 3);
  expected << 0.790687, 0.605096, -0.0931312,  //
      0.415746, -0.642355, -0.643844,          //
      -0.449411, 0.47036, -0.759468;

  auto actual = SO3::ClosestTo(3 * M);
  EXPECT(assert_equal(expected, actual.matrix(), 1e-6));
}

//******************************************************************************
namespace {
SO3 id;
Vector3 z_axis(0, 0, 1), v2(1, 2, 0), v3(1, 2, 3);
SO3 R1(Eigen::AngleAxisd(0.1, z_axis));
SO3 R2(Eigen::AngleAxisd(0.2, z_axis));
}  // namespace

/* ************************************************************************* */
TEST(SO3, ChordalMean) {
  std::vector<SO3> rotations = {R1, R1.inverse()};
  EXPECT(assert_equal(SO3(), SO3::ChordalMean(rotations)));
}

//******************************************************************************
// Check that Hat specialization is equal to dynamic version
TEST(SO3, Hat) {
  EXPECT(assert_equal(SO3::Hat(z_axis), SOn::Hat(z_axis)));
  EXPECT(assert_equal(SO3::Hat(v2), SOn::Hat(v2)));
  EXPECT(assert_equal(SO3::Hat(v3), SOn::Hat(v3)));
}

//******************************************************************************
// Check that Hat specialization is equal to dynamic version
TEST(SO3, Vee) {
  auto X1 = SOn::Hat(z_axis), X2 = SOn::Hat(v2), X3 = SOn::Hat(v3);
  EXPECT(assert_equal(SO3::Vee(X1), SOn::Vee(X1)));
  EXPECT(assert_equal(SO3::Vee(X2), SOn::Vee(X2)));
  EXPECT(assert_equal(SO3::Vee(X3), SOn::Vee(X3)));
}

//******************************************************************************
TEST(SO3, Local) {
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = traits<SO3>::Local(R1, R2);
  EXPECT(assert_equal((Vector)expected, actual));
}

//******************************************************************************
TEST(SO3, Retract) {
  Vector3 v(0, 0, 0.1);
  SO3 actual = traits<SO3>::Retract(R1, v);
  EXPECT(assert_equal(R2, actual));
}

//******************************************************************************
TEST(SO3, Logmap) {
  Vector3 expected(0, 0, 0.1);
  Vector3 actual = SO3::Logmap(R1.between(R2));
  EXPECT(assert_equal((Vector)expected, actual));
}

//******************************************************************************
TEST(SO3, Expmap) {
  Vector3 v(0, 0, 0.1);
  SO3 actual = R1 * SO3::Expmap(v);
  EXPECT(assert_equal(R2, actual));
}

//******************************************************************************
TEST(SO3, Invariants) {
  EXPECT(check_group_invariants(id, id));
  EXPECT(check_group_invariants(id, R1));
  EXPECT(check_group_invariants(R2, id));
  EXPECT(check_group_invariants(R2, R1));

  EXPECT(check_manifold_invariants(id, id));
  EXPECT(check_manifold_invariants(id, R1));
  EXPECT(check_manifold_invariants(R2, id));
  EXPECT(check_manifold_invariants(R2, R1));
}

//******************************************************************************
TEST(SO3, LieGroupDerivatives) {
  CHECK_LIE_GROUP_DERIVATIVES(id, id);
  CHECK_LIE_GROUP_DERIVATIVES(id, R2);
  CHECK_LIE_GROUP_DERIVATIVES(R2, id);
  CHECK_LIE_GROUP_DERIVATIVES(R2, R1);
}

//******************************************************************************
TEST(SO3, ChartDerivatives) {
  CHECK_CHART_DERIVATIVES(id, id);
  CHECK_CHART_DERIVATIVES(id, R2);
  CHECK_CHART_DERIVATIVES(R2, id);
  CHECK_CHART_DERIVATIVES(R2, R1);
}

/* ************************************************************************* */
TEST(SO3, ExpmapFunctor) {
  Vector axis = Vector3(0., 1., 0.);  // rotation around Y
  double angle = 3.14 / 4.0;
  Matrix expected(3,3);
  expected << 0.707388, 0, 0.706825, 0, 1, 0, -0.706825, 0, 0.707388;

  // axis angle version
  so3::ExpmapFunctor f1(axis, angle);
  SO3 actual1 = f1.expmap();
  CHECK(assert_equal(expected, actual1.matrix(), 1e-5));

  // axis angle version, negative angle
  so3::ExpmapFunctor f2(axis, angle - 2*M_PI);
  SO3 actual2 = f2.expmap();
  CHECK(assert_equal(expected, actual2.matrix(), 1e-5));

  // omega version
  so3::ExpmapFunctor f3(axis * angle);
  SO3 actual3 = f3.expmap();
  CHECK(assert_equal(expected, actual3.matrix(), 1e-5));

  // omega version, negative angle
  so3::ExpmapFunctor f4(axis * (angle - 2*M_PI));
  SO3 actual4 = f4.expmap();
  CHECK(assert_equal(expected, actual4.matrix(), 1e-5));
}

/* ************************************************************************* */
namespace exmap_derivative {
static const Vector3 w(0.1, 0.27, -0.2);
}
// Left trivialized Derivative of exp(w) wrpt w:
// How does exp(w) change when w changes?
// We find a y such that: exp(w) exp(y) = exp(w + dw) for dw --> 0
// => y = log (exp(-w) * exp(w+dw))
Vector3 testDexpL(const Vector3& dw) {
  using exmap_derivative::w;
  return SO3::Logmap(SO3::Expmap(-w) * SO3::Expmap(w + dw));
}

TEST(SO3, ExpmapDerivative) {
  using exmap_derivative::w;
  const Matrix actualDexpL = SO3::ExpmapDerivative(w);
  const Matrix expectedDexpL =
      numericalDerivative11<Vector3, Vector3>(testDexpL, Vector3::Zero(), 1e-2);
  EXPECT(assert_equal(expectedDexpL, actualDexpL, 1e-7));

  const Matrix actualDexpInvL = SO3::LogmapDerivative(w);
  EXPECT(assert_equal(expectedDexpL.inverse(), actualDexpInvL, 1e-7));
}

//******************************************************************************
TEST(SO3, ExpmapDerivative2) {
  const Vector3 theta(0.1, 0, 0.1);
  const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
      std::bind(&SO3::Expmap, std::placeholders::_1, nullptr), theta);

  CHECK(assert_equal(Jexpected, SO3::ExpmapDerivative(theta)));
  CHECK(assert_equal(Matrix3(Jexpected.transpose()),
                     SO3::ExpmapDerivative(-theta)));
}

//******************************************************************************
TEST(SO3, ExpmapDerivative3) {
  const Vector3 theta(10, 20, 30);
  const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
      std::bind(&SO3::Expmap, std::placeholders::_1, nullptr), theta);

  CHECK(assert_equal(Jexpected, SO3::ExpmapDerivative(theta)));
  CHECK(assert_equal(Matrix3(Jexpected.transpose()),
                     SO3::ExpmapDerivative(-theta)));
}

//******************************************************************************
TEST(SO3, ExpmapDerivative4) {
  // Iserles05an (Lie-group Methods) says:
  // scalar is easy: d exp(a(t)) / dt = exp(a(t)) a'(t)
  // matrix is hard: d exp(A(t)) / dt = exp(A(t)) dexp[-A(t)] A'(t)
  // where A(t): R -> so(3) is a trajectory in the tangent space of SO(3)
  // and dexp[A] is a linear map from 3*3 to 3*3 derivatives of se(3)
  // Hence, the above matrix equation is typed: 3*3 = SO(3) * linear_map(3*3)

  // In GTSAM, we don't work with the skew-symmetric matrices A directly, but
  // with 3-vectors.
  // omega is easy: d Expmap(w(t)) / dt = ExmapDerivative[w(t)] * w'(t)

  // Let's verify the above formula.

  auto w = [](double t) { return Vector3(2 * t, sin(t), 4 * t * t); };
  auto w_dot = [](double t) { return Vector3(2, cos(t), 8 * t); };

  // We define a function R
  auto R = [w](double t) { return SO3::Expmap(w(t)); };

  for (double t = -2.0; t < 2.0; t += 0.3) {
    const Matrix expected = numericalDerivative11<SO3, double>(R, t);
    const Matrix actual = SO3::ExpmapDerivative(w(t)) * w_dot(t);
    CHECK(assert_equal(expected, actual, 1e-7));
  }
}

//******************************************************************************
TEST(SO3, ExpmapDerivative5) {
  auto w = [](double t) { return Vector3(2 * t, sin(t), 4 * t * t); };
  auto w_dot = [](double t) { return Vector3(2, cos(t), 8 * t); };

  // Now define R as mapping local coordinates to neighborhood around R0.
  const SO3 R0 = SO3::Expmap(Vector3(0.1, 0.4, 0.2));
  auto R = [R0, w](double t) { return R0.expmap(w(t)); };

  for (double t = -2.0; t < 2.0; t += 0.3) {
    const Matrix expected = numericalDerivative11<SO3, double>(R, t);
    const Matrix actual = SO3::ExpmapDerivative(w(t)) * w_dot(t);
    CHECK(assert_equal(expected, actual, 1e-7));
  }
}

//******************************************************************************
TEST(SO3, ExpmapDerivative6) {
  const Vector3 thetahat(0.1, 0, 0.1);
  const Matrix Jexpected = numericalDerivative11<SO3, Vector3>(
      std::bind(&SO3::Expmap, std::placeholders::_1, nullptr), thetahat);
  Matrix3 Jactual;
  SO3::Expmap(thetahat, Jactual);
  EXPECT(assert_equal(Jexpected, Jactual));
}

//******************************************************************************
TEST(SO3, LogmapDerivative) {
  const Vector3 thetahat(0.1, 0, 0.1);
  const SO3 R = SO3::Expmap(thetahat);  // some rotation
  const Matrix Jexpected = numericalDerivative11<Vector, SO3>(
      std::bind(&SO3::Logmap, std::placeholders::_1, nullptr), R);
  const Matrix3 Jactual = SO3::LogmapDerivative(thetahat);
  EXPECT(assert_equal(Jexpected, Jactual));
}

//******************************************************************************
TEST(SO3, JacobianLogmap) {
  const Vector3 thetahat(0.1, 0, 0.1);
  const SO3 R = SO3::Expmap(thetahat);  // some rotation
  const Matrix Jexpected = numericalDerivative11<Vector, SO3>(
      std::bind(&SO3::Logmap, std::placeholders::_1, nullptr), R);
  Matrix3 Jactual;
  SO3::Logmap(R, Jactual);
  EXPECT(assert_equal(Jexpected, Jactual));
}

//******************************************************************************
TEST(SO3, ApplyDexp) {
  Matrix aH1, aH2;
  for (bool nearZeroApprox : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
        [=](const Vector3& omega, const Vector3& v) {
          return so3::DexpFunctor(omega, nearZeroApprox).applyDexp(v);
        };
    for (Vector3 omega : {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0),
                          Vector3(0, 0, 1), Vector3(0.1, 0.2, 0.3)}) {
      so3::DexpFunctor local(omega, nearZeroApprox);
      for (Vector3 v : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
                        Vector3(0.4, 0.3, 0.2)}) {
        EXPECT(assert_equal(Vector3(local.dexp() * v),
                            local.applyDexp(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(local.dexp(), aH2));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyInvDexp) {
  Matrix aH1, aH2;
  for (bool nearZeroApprox : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
        [=](const Vector3& omega, const Vector3& v) {
          return so3::DexpFunctor(omega, nearZeroApprox).applyInvDexp(v);
        };
    for (Vector3 omega : {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(0, 1, 0),
                          Vector3(0, 0, 1), Vector3(0.1, 0.2, 0.3)}) {
      so3::DexpFunctor local(omega, nearZeroApprox);
      Matrix invDexp = local.dexp().inverse();
      for (Vector3 v : {Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
                        Vector3(0.4, 0.3, 0.2)}) {
        EXPECT(assert_equal(Vector3(invDexp * v),
                            local.applyInvDexp(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(invDexp, aH2));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, vec) {
  const Vector9 expected = Eigen::Map<const Vector9>(R2.matrix().data());
  Matrix actualH;
  const Vector9 actual = R2.vec(actualH);
  CHECK(assert_equal(expected, actual));
  std::function<Vector9(const SO3&)> f = [](const SO3& Q) { return Q.vec(); };
  const Matrix numericalH = numericalDerivative11(f, R2, 1e-5);
  CHECK(assert_equal(numericalH, actualH));
}

//******************************************************************************
TEST(Matrix, compose) {
  Matrix3 M;
  M << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  SO3 R = SO3::Expmap(Vector3(1, 2, 3));
  const Matrix3 expected = M * R.matrix();
  Matrix actualH;
  const Matrix3 actual = so3::compose(M, R, actualH);
  CHECK(assert_equal(expected, actual));
  std::function<Matrix3(const Matrix3&)> f = [R](const Matrix3& M) {
    return so3::compose(M, R);
  };
  Matrix numericalH = numericalDerivative11(f, M, 1e-2);
  CHECK(assert_equal(numericalH, actualH));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
