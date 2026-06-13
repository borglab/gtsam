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
#include <gtsam/geometry/Gal3SemiDirect.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

constexpr double kTol = 1e-9;
constexpr double kNumTol = 1e-6;

// GTSAM provides Vector4/Vector6 but not these fixed sizes; define locally.
using Vector10 = Eigen::Matrix<double, 10, 1>;
using Vector12 = Eigen::Matrix<double, 12, 1>;
using Vector20 = Eigen::Matrix<double, 20, 1>;

using TSO3 = TangentLieGroup<Rot3>;
using TSE3 = TangentLieGroup<Pose3>;
using TGal3 = TangentLieGroup<SemidirectGal3>;  // recursive: base is a product
using TGal3Native = TangentLieGroup<Gal3>;      // native base, cross-check

namespace {

TSE3 tse3State() {
  return TSE3(Pose3(Rot3::RzRyRx(0.1, 0.2, 0.3), Point3(1.0, 2.0, 3.0)),
              (Vector6() << 0.4, 0.5, 0.6, 0.7, 0.8, 0.9).finished());
}
TSE3 tse3State2() {
  return TSE3(Pose3(Rot3::RzRyRx(-0.2, 0.1, 0.15), Point3(-0.75, 0.4, 1.2)),
              (Vector6() << -0.3, 0.2, 0.1, 0.5, -0.4, 0.25).finished());
}
Vector6 tso3Xi() {
  return (Vector6() << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6).finished();
}
Vector12 tse3Xi() {
  Vector12 v;
  v << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.05, 0.15, -0.25, 0.3, -0.35, 0.1;
  return v;
}
TSE3 expmapTSE3(const Vector12& v) { return TSE3::Expmap(v); }
Vector12 logmapTSE3(const TSE3& x) { return TSE3::Logmap(x); }
TSE3 composeTSE3(const TSE3& a, const TSE3& b) { return a.compose(b); }
TSE3 betweenTSE3(const TSE3& a, const TSE3& b) { return a.between(b); }
TSE3 inverseTSE3(const TSE3& a) { return a.inverse(); }

// Convert a SemidirectGal3 element (Pose3 = (R, velocity), H = (position,time))
// to the native Gal3 oracle (R, position, velocity, time).
Gal3 asGal3(const SemidirectGal3& s) {
  // Pose3.translation() holds the velocity-boost; H.head<3>() is the position.
  return Gal3(s.first.rotation(), s.second.head<3>(), s.first.translation(),
              s.second(3));
}
// Tangent layouts coincide: [ω; ν; p; s] (semidirect) == [ω; ν; ρ; τ] (native).
Vector10 gal3Xi() {
  Vector10 xi;
  xi << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.15, -0.25, 0.3, 0.2;
  return xi;
}
SemidirectGal3 expmapSGal3(const Vector10& v) {
  return SemidirectGal3::Expmap(v);
}
Vector10 logmapSGal3(const SemidirectGal3& s) {
  return SemidirectGal3::Logmap(s);
}

Vector20 tgal3Xi() {
  Vector20 v;
  v << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.15, -0.25, 0.3, 0.2,    // base (gal3)
      0.05, 0.1, -0.15, 0.2, -0.05, 0.1, 0.12, -0.08, 0.18, -0.1;  // algebra
  return v;
}
TGal3 expmapTGal3(const Vector20& v) { return TGal3::Expmap(v); }
Vector20 logmapTGal3(const TGal3& x) { return TGal3::Logmap(x); }

}  // namespace

/* ************************************************************************* */
TEST(TangentLieGroup, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsGroup<TSE3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<TSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TSE3>);
  EXPECT_LONGS_EQUAL(12, TSE3::dimension);
  EXPECT_LONGS_EQUAL(6, TSO3::dimension);
}

/* ************************************************************************* */
TEST(TangentLieGroup, IdentityComposeInverse) {
  const TSE3 x = tse3State();
  EXPECT(assert_equal(TSE3::Identity(), x * x.inverse(), kTol));
  EXPECT(assert_equal(TSE3::Identity(), x.inverse() * x, kTol));
  // group law: (g1,v1)*(g2,v2) = (g1 g2, v1 + Ad_{g1} v2)
  const TSE3 a(Pose3(Rot3::Rz(0.2), Point3(1, 0, 0)),
               (Vector6() << 0, 0, 0.1, 0.2, 0, 0).finished());
  const TSE3 b(Pose3(Rot3::Ry(0.1), Point3(0, 1, 0)),
               (Vector6() << 0.05, 0, 0, 0, 0.3, 0).finished());
  const TSE3 ab = a * b;
  EXPECT(assert_equal(a.first * b.first, ab.first, kTol));
  EXPECT(assert_equal(Vector(a.second + a.first.AdjointMap() * b.second),
                      Vector(ab.second), kTol));
}

/* ************************************************************************* */
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

/* ************************************************************************* */
TEST(TangentLieGroup, ExpLogRoundTrip) {
  const Vector6 xi = tso3Xi();
  EXPECT(assert_equal(xi, TSO3::Logmap(TSO3::Expmap(xi)), kTol));
  const TSE3 x = tse3State();
  EXPECT(assert_equal(x, TSE3::Expmap(TSE3::Logmap(x)), kTol));
}

/* ************************************************************************* */
// Transport block: Expmap([u; xi]).second == J_l^G(u) * xi, with
// J_l(u) = Ad_{Exp(u)} * J_r(u) and J_r(u) = the Jacobian returned by Expmap.
TEST(TangentLieGroup, TransportBlockIsLeftJacobian) {
  const Vector6 u = (Vector6() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  const Vector6 xi = (Vector6() << -0.2, 0.1, 0.05, 0.3, -0.1, 0.2).finished();
  Vector12 tv;
  tv << u, xi;
  Matrix6 Jr;
  const Pose3 g = Pose3::Expmap(u, Jr);
  const Matrix6 Jl = g.AdjointMap() * Jr;
  const TSE3 x = TSE3::Expmap(tv);
  EXPECT(assert_equal(g, x.first, kTol));
  EXPECT(assert_equal(Vector(Jl * xi), Vector(x.second), kTol));
}

/* ************************************************************************* */
TEST(TangentLieGroup, ExpmapLogmapJacobians) {
  const Vector12 xi = tse3Xi();
  Matrix expH;
  TSE3::Expmap(xi, expH);
  EXPECT(assert_equal(numericalDerivative11(expmapTSE3, xi), expH, kNumTol));

  const TSE3 state = tse3State2();
  Matrix logH;
  TSE3::Logmap(state, logH);
  EXPECT(assert_equal(numericalDerivative11(logmapTSE3, state), logH, kNumTol));
}

/* ************************************************************************* */
TEST(TangentLieGroup, ComposeBetweenInverseJacobians) {
  const TSE3 a = tse3State(), b = tse3State2();
  Matrix H1, H2;

  a.compose(b, H1, H2);
  EXPECT(assert_equal(numericalDerivative21(composeTSE3, a, b), H1, kNumTol));
  EXPECT(assert_equal(numericalDerivative22(composeTSE3, a, b), H2, kNumTol));

  a.between(b, H1, H2);
  EXPECT(assert_equal(numericalDerivative21(betweenTSE3, a, b), H1, kNumTol));
  EXPECT(assert_equal(numericalDerivative22(betweenTSE3, a, b), H2, kNumTol));

  Matrix H;
  a.inverse(H);
  EXPECT(assert_equal(numericalDerivative11(inverseTSE3, a), H, kNumTol));
}

/* ************************************************************************* */
// Ad_{(g,ξ)} = [[Ad_g, 0], [ad_ξ·Ad_g, Ad_g]].
TEST(TangentLieGroup, AdjointMap) {
  const TSE3 x = tse3State();
  const Matrix6 Ad = x.first.AdjointMap();
  const Matrix6 adXi = Pose3::adjointMap(x.second);
  Eigen::Matrix<double, 12, 12> expected = Eigen::Matrix<double, 12, 12>::Zero();
  expected.topLeftCorner<6, 6>() = Ad;
  expected.bottomRightCorner<6, 6>() = Ad;
  expected.bottomLeftCorner<6, 6>() = adXi * Ad;
  EXPECT(assert_equal(Matrix(expected), Matrix(x.AdjointMap()), kTol));
}

/* ************************************************************************* */
TEST(SemidirectGal3, MatchesNativeGal3) {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<SemidirectGal3>);
  EXPECT_LONGS_EQUAL(10, SemidirectGal3::dimension);

  const Vector10 xi = gal3Xi();
  const SemidirectGal3 x = SemidirectGal3::Expmap(xi);
  // Expmap value matches native Gal3 (orderings coincide).
  EXPECT(assert_equal(Gal3::Expmap(xi), asGal3(x), kTol));
  // Logmap round-trips through native ordering.
  EXPECT(assert_equal(xi, SemidirectGal3::Logmap(x), kTol));
  // AdjointMap matches native.
  EXPECT(assert_equal(Matrix(Gal3::Expmap(xi).AdjointMap()),
                      Matrix(x.AdjointMap()), kTol));
}

/* ************************************************************************* */
TEST(SemidirectGal3, Jacobians) {
  const Vector10 xi = gal3Xi();
  Matrix expH;
  SemidirectGal3::Expmap(xi, expH);
  EXPECT(assert_equal(numericalDerivative11(expmapSGal3, xi), expH, kNumTol));

  const SemidirectGal3 state = SemidirectGal3::Expmap(gal3Xi() * 0.5);
  Matrix logH;
  SemidirectGal3::Logmap(state, logH);
  EXPECT(assert_equal(numericalDerivative11(logmapSGal3, state), logH, kNumTol));
}

/* ************************************************************************* */
// Static algebra adjoint of the semidirect product matches native Gal3::ad.
TEST(SemidirectGal3, AlgebraAdjoint) {
  const Vector10 xi = gal3Xi();
  EXPECT(assert_equal(Matrix(Gal3::adjointMap(xi)),
                      Matrix(SemidirectGal3::adjointMap(xi)), kTol));
}

/* ************************************************************************* */
TEST(TangentLieGroup, TGal3Concepts) {
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGal3>);
  EXPECT_LONGS_EQUAL(20, TGal3::dimension);
}

/* ************************************************************************* */
TEST(TangentLieGroup, TGal3RoundTripAndAxioms) {
  const Vector20 xi = tgal3Xi();
  const TGal3 x = TGal3::Expmap(xi);
  EXPECT(assert_equal(xi, TGal3::Logmap(x), kTol));
  EXPECT(assert_equal(TGal3::Identity(), x * x.inverse(), kTol));
}

/* ************************************************************************* */
TEST(TangentLieGroup, TGal3Jacobians) {
  const Vector20 xi = tgal3Xi();
  Matrix expH;
  TGal3::Expmap(xi, expH);
  EXPECT(assert_equal(numericalDerivative11(expmapTGal3, xi), expH, kNumTol));

  const TGal3 state = TGal3::Expmap(tgal3Xi() * 0.5);
  Matrix logH;
  TGal3::Logmap(state, logH);
  EXPECT(assert_equal(numericalDerivative11(logmapTGal3, state), logH, kNumTol));
}

/* ************************************************************************* */
// Cross-check: native-base tangent group reproduces the semidirect-base one
// (tangent orderings coincide, so all blocks compare directly).
TEST(TangentLieGroup, TGal3NativeCrossCheck) {
  const Vector20 xi = tgal3Xi();
  const TGal3 x = TGal3::Expmap(xi);
  const TGal3Native xn = TGal3Native::Expmap(xi);
  EXPECT(assert_equal(xn.first, asGal3(x.first), kTol));            // base parts
  EXPECT(assert_equal(Vector(xn.second), Vector(x.second), kTol));  // algebra
  EXPECT(assert_equal(Matrix(xn.AdjointMap()), Matrix(x.AdjointMap()), kTol));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
