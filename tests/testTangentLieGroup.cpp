/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testTangentLieGroup.cpp
 * @date June, 2026
 * @author Alessandro Fornasier
 * @brief Tangent Lie group TG = G ⋉ 𝔤 via the semidirect ProductLieGroup.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TangentLieGroup.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

/* ************************************************************************* */
namespace tangent_lie_group_fixture {

constexpr double kTol = 1e-9;
constexpr double kNumTol = 1e-6;

// GTSAM provides Vector4/Vector6 but not these fixed sizes; define locally.
using Vector12 = Eigen::Matrix<double, 12, 1>;
using Vector20 = Eigen::Matrix<double, 20, 1>;

using TGSO3 = TangentLieGroup<Rot3>;
using TGSE3 = TangentLieGroup<Pose3>;
using TGGal3 = TangentLieGroup<Gal3>;
using TGTGSE3 = TangentLieGroup<TGSE3>;

TGSE3 tgse3State() {
  return TGSE3(Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0)),
               (Vector6() << 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished());
}
TGSE3 tgse3State2() {
  return TGSE3(Pose3(Rot3::RzRyRx(-0.2, 0.1, 0.15), Point3(-0.75, 0.4, 1.2)),
               (Vector6() << -0.3, 0.2, 0.1, 0.5, -0.4, 0.25).finished());
}
Vector6 tgso3Xi() {
  return (Vector6() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6).finished();
}
Vector12 tgse3Xi() {
  Vector12 v;
  v << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.05, 0.15, -0.25, 0.3, -0.35, 0.1;
  return v;
}
TGSE3 expmapTGSE3(const Vector12& v) { return TGSE3::Expmap(v); }
Vector12 logmapTGSE3(const TGSE3& x) { return TGSE3::Logmap(x); }
TGSE3 composeTGSE3(const TGSE3& a, const TGSE3& b) { return a.compose(b); }
TGSE3 betweenTGSE3(const TGSE3& a, const TGSE3& b) { return a.between(b); }
TGSE3 inverseTGSE3(const TGSE3& a) { return a.inverse(); }

Vector20 tggal3Xi() {
  Vector20 v;
  v << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.15, -0.25, 0.3, 0.2,  // base (gal3)
      0.05, 0.1, -0.15, 0.2, -0.05, 0.1, 0.12, -0.08, 0.18, -0.1;  // algebra
  return v;
}
TGGal3 expmapTGGal3(const Vector20& v) { return TGGal3::Expmap(v); }
Vector20 logmapTGGal3(const TGGal3& x) { return TGGal3::Logmap(x); }

// Verifies Lie-group concepts and dimensions for representative tangent groups.
TEST(TangentLieGroup, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsGroup<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGTGSE3>);
  EXPECT_LONGS_EQUAL(12, TGSE3::dimension);
  EXPECT_LONGS_EQUAL(6, TGSO3::dimension);
  EXPECT_LONGS_EQUAL(24, TGTGSE3::dimension);
}

// Verifies identity, inverse, and the tangent-group multiplication law.
TEST(TangentLieGroup, IdentityComposeInverse) {
  const TGSE3 x = tgse3State();
  EXPECT(assert_equal(TGSE3::Identity(), x * x.inverse(), kTol));
  EXPECT(assert_equal(TGSE3::Identity(), x.inverse() * x, kTol));
  // group law: (g1,v1)*(g2,v2) = (g1 g2, v1 + Ad_{g1} v2)
  const TGSE3 a(Pose3(Rot3::Rz(0.2), Point3(1, 0, 0)),
                (Vector6() << 0, 0, 0.1, 0.2, 0, 0).finished());
  const TGSE3 b(Pose3(Rot3::Ry(0.1), Point3(0, 1, 0)),
                (Vector6() << 0.05, 0, 0, 0, 0.3, 0).finished());
  const TGSE3 ab = a * b;
  EXPECT(assert_equal(a.first * b.first, ab.first, kTol));
  EXPECT(assert_equal(Vector(a.second + a.first.AdjointMap() * b.second),
                      Vector(ab.second), kTol));
}

// AdjointAction obeys the left-action law and its generator matches a finite
// difference of Ad_{Exp(εu)}·ξ.
TEST(TangentLieGroup, AdjointActionLawAndGenerator) {
  const AdjointAction<Pose3> phi;
  const Pose3 g1 = Pose3(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(0.4, -0.5, 0.6));
  const Pose3 g2 = Pose3(Rot3::RzRyRx(-0.2, 0.05, 0.1), Point3(-0.3, 0.2, 0.1));
  const Vector6 xi = (Vector6() << 0.2, -0.1, 0.3, 0.4, 0.5, -0.6).finished();
  EXPECT_LEFT_ACTION(phi, g1, g2, xi);

  const Vector6 u = (Vector6() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  const double eps = 1e-7;
  const Vector6 genFd = (Pose3::Expmap(eps * u).AdjointMap() * xi - xi) / eps;
  EXPECT(assert_equal(Vector(AdjointAction<Pose3>::generator(u) * xi),
                      Vector(genFd), 1e-6));
}

// Verifies Expmap and Logmap are mutual inverses for SO(3) and SE(3) bases.
TEST(TangentLieGroup, ExpLogRoundTrip) {
  const Vector6 xi = tgso3Xi();
  EXPECT(assert_equal(xi, TGSO3::Logmap(TGSO3::Expmap(xi)), kTol));
  const TGSE3 x = tgse3State();
  EXPECT(assert_equal(x, TGSE3::Expmap(TGSE3::Logmap(x)), kTol));
}

// Expmap([u; xi]).second == J_l^G(u) * xi, with J_l(u) = Ad_{Exp(u)} * J_r(u)
// and J_r(u) = the Jacobian returned by Expmap.
TEST(TangentLieGroup, TransportBlockIsLeftJacobian) {
  const Vector6 u = (Vector6() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  const Vector6 xi = (Vector6() << -0.2, 0.1, 0.05, 0.3, -0.1, 0.2).finished();
  Vector12 tv;
  tv << u, xi;
  Matrix6 Jr;
  const Pose3 g = Pose3::Expmap(u, Jr);
  const Matrix6 Jl = g.AdjointMap() * Jr;
  const TGSE3 x = TGSE3::Expmap(tv);
  EXPECT(assert_equal(g, x.first, kTol));
  EXPECT(assert_equal(Vector(Jl * xi), Vector(x.second), kTol));
}

// Checks Expmap and Logmap Jacobians against numerical derivatives.
TEST(TangentLieGroup, ExpmapLogmapJacobians) {
  const Vector12 xi = tgse3Xi();
  Matrix expH;
  TGSE3::Expmap(xi, expH);
  EXPECT(assert_equal(numericalDerivative11(expmapTGSE3, xi), expH, kNumTol));

  const TGSE3 state = tgse3State2();
  Matrix logH;
  TGSE3::Logmap(state, logH);
  EXPECT(
      assert_equal(numericalDerivative11(logmapTGSE3, state), logH, kNumTol));
}

// Checks compose, between, and inverse Jacobians numerically.
TEST(TangentLieGroup, ComposeBetweenInverseJacobians) {
  const TGSE3 a = tgse3State(), b = tgse3State2();
  Matrix H1, H2;

  a.compose(b, H1, H2);
  EXPECT(assert_equal(numericalDerivative21(composeTGSE3, a, b), H1, kNumTol));
  EXPECT(assert_equal(numericalDerivative22(composeTGSE3, a, b), H2, kNumTol));

  a.between(b, H1, H2);
  EXPECT(assert_equal(numericalDerivative21(betweenTGSE3, a, b), H1, kNumTol));
  EXPECT(assert_equal(numericalDerivative22(betweenTGSE3, a, b), H2, kNumTol));

  Matrix H;
  a.inverse(H);
  EXPECT(assert_equal(numericalDerivative11(inverseTGSE3, a), H, kNumTol));
}

// Ad_{(g,ξ)} = [[Ad_g, 0], [ad_ξ·Ad_g, Ad_g]].
TEST(TangentLieGroup, AdjointMap) {
  const TGSE3 x = tgse3State();
  const Matrix6 Ad = x.first.AdjointMap();
  const Matrix6 adXi = Pose3::adjointMap(x.second);
  Eigen::Matrix<double, 12, 12> expected =
      Eigen::Matrix<double, 12, 12>::Zero();
  expected.topLeftCorner<6, 6>() = Ad;
  expected.bottomRightCorner<6, 6>() = Ad;
  expected.bottomLeftCorner<6, 6>() = adXi * Ad;
  EXPECT(assert_equal(Matrix(expected), Matrix(x.AdjointMap()), kTol));
}

// Checks ad_(u,v) and the recursive generator used by T(T(SE(3))).
TEST(TangentLieGroup, AlgebraAdjoint) {
  const Vector12 xi = tgse3Xi();
  const Matrix6 adU = Pose3::adjointMap(xi.head<6>());
  const Matrix6 adV = Pose3::adjointMap(xi.tail<6>());
  Eigen::Matrix<double, 12, 12> expected =
      Eigen::Matrix<double, 12, 12>::Zero();
  expected.topLeftCorner<6, 6>() = adU;
  expected.bottomLeftCorner<6, 6>() = adV;
  expected.bottomRightCorner<6, 6>() = adU;

  EXPECT(assert_equal(Matrix(expected), Matrix(TGSE3::adjointMap(xi)), kTol));
  EXPECT(assert_equal(Matrix(expected),
                      Matrix(AdjointAction<TGSE3>::generator(xi)), kTol));
}

// Verifies the tangent group of Gal(3) satisfies the Lie-group concept.
TEST(TangentLieGroup, TGGal3Concepts) {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGGal3>);
  EXPECT_LONGS_EQUAL(20, TGGal3::dimension);
}

// Checks the Gal(3) tangent group's round trip and inverse axiom.
TEST(TangentLieGroup, TGGal3RoundTripAndAxioms) {
  const Vector20 xi = tggal3Xi();
  const TGGal3 x = TGGal3::Expmap(xi);
  EXPECT(assert_equal(xi, TGGal3::Logmap(x), kTol));
  EXPECT(assert_equal(TGGal3::Identity(), x * x.inverse(), kTol));
}

// Checks Gal(3) tangent-group Expmap and Logmap Jacobians numerically.
TEST(TangentLieGroup, TGGal3Jacobians) {
  const Vector20 xi = tggal3Xi();
  Matrix expH;
  TGGal3::Expmap(xi, expH);
  EXPECT(assert_equal(numericalDerivative11(expmapTGGal3, xi), expH, kNumTol));

  const TGGal3 state = TGGal3::Expmap(tggal3Xi() * 0.5);
  Matrix logH;
  TGGal3::Logmap(state, logH);
  EXPECT(
      assert_equal(numericalDerivative11(logmapTGGal3, state), logH, kNumTol));
}

}  // namespace tangent_lie_group_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
