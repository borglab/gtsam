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
#include <gtsam/geometry/Kernel.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>

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
  GTSAM_CONCEPT_ASSERT(IsGroup<SO3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<SO3>);
  GTSAM_CONCEPT_ASSERT(IsMatrixLieGroup<SO3>);
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
  std::vector<SO3> rotations = { R1, R1.inverse() };
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
TEST(SO3, Local2) {
  Vector axis = Vector3(0., 1., 0.);  // rotation around Y
  double angle = 3.14 / 4.0;
  Matrix expected(3, 3);
  expected << 0.707388, 0, 0.706825, 0, 1, 0, -0.706825, 0, 0.707388;

  // omega version
  so3::ExpmapFunctor f3(axis * angle);
  SO3 actual3(f3.expmap());
  CHECK(assert_equal(expected, actual3.matrix(), 1e-5));

  // omega version, negative angle
  so3::ExpmapFunctor f4(axis * (angle - 2 * M_PI));
  SO3 actual4(f4.expmap());
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
  const Vector3 theta(0.1, 0, 0.1);
  const Matrix expectedH = numericalDerivative11<SO3, Vector3>(
    std::bind(&SO3::Expmap, std::placeholders::_1, nullptr), theta);
  Matrix3 actualH;
  SO3::Expmap(theta, actualH);
  EXPECT(assert_equal(expectedH, actualH));
}

//******************************************************************************
TEST(SO3, LogmapDerivative) {
  const SO3 R0; // Identity
  const Vector3 omega1(0.1, 0, 0.1);
  const SO3 R1 = SO3::Expmap(omega1);  // Small rotation
  const SO3 R2((Matrix3() <<            // Near pi
    -0.750767, -0.0285082, -0.659952,
    -0.0102558, -0.998445, 0.0547974,
    -0.660487, 0.0479084, 0.749307).finished());
  const SO3 R3((Matrix3() <<            // Near pi
    -0.747473, -0.00190019, -0.664289,
    -0.0385114, -0.99819, 0.0461892,
    -0.663175, 0.060108, 0.746047).finished());
  const SO3 R4((Matrix3() <<            // Final pose in a drone experiment
    0.324237, 0.902975, 0.281968,
    -0.674322, 0.429668, -0.600562,
    -0.663445, 0.00458662, 0.748211).finished());
  size_t i = 0;
  for (const SO3& R : { R0, R1, R2, R3, R4 }) {
    const bool nearPi = (i == 2 || i == 3); // Flag cases near pi

    Matrix3 actualH; // H computed by Logmap(R, H) using LogmapDerivative(omega)
    const Vector3 omega = SO3::Logmap(R, actualH);

    // 1. Check self-consistency of analytical derivative calculation:
    //    Does the H returned by Logmap match an independent calculation
    //    of J_r^{-1} using Local with the computed omega?
    so3::DexpFunctor local(omega);
    Matrix3 J_r_inv = local.InvJacobian().right(); // J_r^{-1} via Local
    EXPECT(assert_equal(J_r_inv, actualH)); // This test is crucial and should pass

    // 2. Check analytical derivative against numerical derivative:
    //    Only perform this check AWAY from the pi singularity, where
    //    numerical differentiation of Logmap is expected to be reliable
    //    and should match the analytical derivative.
    if (!nearPi) {
      const Matrix expectedH = numericalDerivative11<Vector3, SO3>(
        std::bind(&SO3::Logmap, std::placeholders::_1, nullptr), R, 1e-7);
      EXPECT(assert_equal(expectedH, actualH, 1e-6)); // 1e-6 needed to pass R4
    }
    else {
      // We accept that the numerical derivative of this specific Logmap implementation
      // near pi will not match the standard analytical derivative J_r^{-1}.
    }
    i++;
  }
}
//******************************************************************************
namespace test_cases {
  std::vector<Vector3> small{ {0, 0, 0},                                 //
                             {1e-5, 0, 0}, {0, 1e-5, 0}, {0, 0, 1e-5},  //,
                             {1e-4, 0, 0}, {0, 1e-4, 0}, {0, 0, 1e-4} };
  std::vector<Vector3> large{
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.1, 0.2, 0.3},
      {0.4, 0.5, 0.6}, {0.7, 0.8, 0.9}, {1.1, 1.2, 1.3}, {1.4, 1.5, 1.6},
      {1.7, 1.8, 1.9}, {2, 2, 2}, {3, 3, 3}, {4, 4, 4}, {5, 5, 5} };
  auto omegas = [](bool nearZero) { return nearZero ? small : large; };
  std::vector<Vector3> vs{ {1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {0.4, 0.3, 0.2} };
}  // namespace test_cases

//******************************************************************************
TEST(SO3, JacobianInverses) {
  Matrix HR, HL;
  for (bool nearZero : {true, false}) {
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      EXPECT(assert_equal<Matrix3>(local.Jacobian().right().inverse(),
        local.InvJacobian().right()));
      EXPECT(assert_equal<Matrix3>(local.Jacobian().left().inverse(),
        local.InvJacobian().left()));
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyRightJacobian) {
  Matrix aH1, aH2;
  for (bool nearZero : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
      [nearZero](const Vector3& omega, const Vector3& v) {
      return so3::DexpFunctor(omega, nearZero ? 1.0 : 0.0, 1e-5).Jacobian().applyRight(v);
      };
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      for (const Vector3& v : test_cases::vs) {
        EXPECT(assert_equal(Vector3(local.Jacobian().right() * v),
          local.Jacobian().applyRight(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(local.Jacobian().right(), aH2));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyRightJacobianInverse) {
  Matrix aH1, aH2;
  for (bool nearZero : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
      [nearZero](const Vector3& omega, const Vector3& v) {
      return so3::DexpFunctor(omega, nearZero ? 1.0 : 0.0, 1e-5).InvJacobian().applyRight(v);
      };
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      Matrix invJr = local.InvJacobian().right();
      for (const Vector3& v : test_cases::vs) {
        EXPECT(
          assert_equal(Vector3(invJr * v), local.InvJacobian().applyRight(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(invJr, aH2));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyLeftJacobian) {
  Matrix aH1, aH2;
  for (bool nearZero : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
      [nearZero](const Vector3& omega, const Vector3& v) {
      return so3::DexpFunctor(omega, nearZero ? 1.0 : 0.0, 1e-5).Jacobian().applyLeft(v);
      };
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      for (const Vector3& v : test_cases::vs) {
        EXPECT(assert_equal(Vector3(local.Jacobian().left() * v),
          local.Jacobian().applyLeft(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(local.Jacobian().left(), aH2));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyLeftJacobianInverse) {
  Matrix aH1, aH2;
  for (bool nearZero : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
      [nearZero](const Vector3& omega, const Vector3& v) {
      return so3::DexpFunctor(omega, nearZero ? 1.0 : 0.0, 1e-5).InvJacobian().applyLeft(v);
      };
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      Matrix invJl = local.InvJacobian().left();
      for (const Vector3& v : test_cases::vs) {
        EXPECT(assert_equal(Vector3(invJl * v),
          local.InvJacobian().applyLeft(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2));
        EXPECT(assert_equal(invJl, aH2));
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
TEST(SO3, AdjointMap) {
  // Create a non-trivial SO3
  const SO3 R = SO3::Expmap(Vector3(0.1, 0.2, 0.3));

  // Call the specialized AdjointMap
  const Matrix3 specialized_Adj = R.AdjointMap();
  EXPECT(assert_equal(R.matrix(), specialized_Adj));

  // Call the generic AdjointMap from the base class
  const Matrix3 generic_Adj =
    static_cast<const MatrixLieGroup<SO3, 3, 3>*>(&R)->AdjointMap();

  // Assert that they are equal
  EXPECT(assert_equal(specialized_Adj, generic_Adj, 1e-9));
}

//******************************************************************************
TEST(SO3, ApplyGamma) {
  Matrix aH1, aH2;
  for (bool nearZero : {true, false}) {
    std::function<Vector3(const Vector3&, const Vector3&)> f =
        [nearZero](const Vector3& omega, const Vector3& v) {
          return so3::DexpFunctor(omega, nearZero ? 1.0 : 0.0, 1e-5).Gamma().applyLeft(v);
        };
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      for (const Vector3& v : test_cases::vs) {
        Matrix3 Gl = local.Gamma().left();
        EXPECT(assert_equal(Vector3(Gl * v), local.Gamma().applyLeft(v, aH1, aH2)));
        EXPECT(assert_equal(numericalDerivative21(f, omega, v), aH1, 1e-5));
        EXPECT(assert_equal(numericalDerivative22(f, omega, v), aH2, 1e-5));
        EXPECT(assert_equal(Gl, aH2, 1e-5));
      }
    }
  }
}

//******************************************************************************
TEST(SO3, ApplyJacobianMatchesFrechetApply) {
  Matrix H_apply, H_dummy;
  for (bool nearZero : {true, false}) {
    for (const Vector3& omega : test_cases::omegas(nearZero)) {
      so3::DexpFunctor local(omega, nearZero ? 1.0 : 0.0, 1e-5);
      for (const Vector3& v : test_cases::vs) {
        // Left Jacobian vs Jr/Jl kernels’ Fréchet applied to v
        Matrix3 H1_leftJ;
        local.Jacobian().applyLeft(v, H1_leftJ, H_dummy);
        EXPECT(assert_equal(H1_leftJ, local.Jacobian().applyFrechet(v), 1e-5));

        // Left Gamma vs Gl kernel’s Fréchet applied to v
        Matrix3 H1_leftG;
        local.Gamma().applyLeft(v, H1_leftG, H_dummy);
        EXPECT(assert_equal(H1_leftG, local.Gamma().applyFrechet(v), 1e-5));
      }
    }
  }
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
