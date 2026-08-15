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
 * @brief Tangent Lie group TG = G ⋉ 𝔤 under the adjoint action.
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/ProductLieGroup.h>
#include <gtsam/base/TangentLieGroup.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Pose2.h>
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
using Vector24 = Eigen::Matrix<double, 24, 1>;

using TGSO3 = TangentLieGroup<Rot3>;
using TGSE3 = TangentLieGroup<Pose3>;
using TGGal3 = TangentLieGroup<Gal3>;
using TGTGSE3 = TangentLieGroup<TGSE3>;
using DirectProduct = ProductLieGroup<Rot3, Pose2>;
using TGDirectProduct = TangentLieGroup<DirectProduct>;

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
Vector20 tggal3Xi() {
  Vector20 v;
  v << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.15, -0.25, 0.3, 0.2,  // base (gal3)
      0.05, 0.1, -0.15, 0.2, -0.05, 0.1, 0.12, -0.08, 0.18, -0.1;  // algebra
  return v;
}
// Verifies Lie-group concepts and dimensions for representative tangent groups.
TEST(TangentLieGroup, Concepts) {
  GTSAM_CONCEPT_ASSERT(IsGroup<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGTGSE3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<TGDirectProduct>);
  EXPECT_LONGS_EQUAL(12, TGSE3::dimension);
  EXPECT_LONGS_EQUAL(6, TGSO3::dimension);
  EXPECT_LONGS_EQUAL(24, TGTGSE3::dimension);
  EXPECT_LONGS_EQUAL(12, TGDirectProduct::dimension);
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

// Verifies a tangent group can use a direct ProductLieGroup as its base.
TEST(TangentLieGroup, DirectProductBaseRoundTrip) {
  Vector12 xi;
  xi << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.05, 0.15, -0.25, 0.3, -0.35, 0.1;
  EXPECT(assert_equal(xi, TGDirectProduct::Logmap(TGDirectProduct::Expmap(xi)),
                      kTol));
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

// TSO(3) is SE(3) with its algebra component as translation, so their
// closed-form exponential values and right Jacobians must agree exactly.
TEST(TangentLieGroup, TSO3UsesSE3ClosedForm) {
  const Vector6 xi = tgso3Xi();
  const Vector3 omega = xi.head<3>();
  const Vector3 v = xi.tail<3>();
  Matrix6 tangentH, poseH;
  const TGSO3 tangent = TGSO3::Expmap(xi, tangentH);
  const Pose3 pose = Pose3::Expmap(xi, poseH);

  EXPECT(assert_equal(pose.rotation(), tangent.first, kTol));
  EXPECT(
      assert_equal(Vector(pose.translation()), Vector(tangent.second), kTol));
  EXPECT(assert_equal(poseH, tangentH, kTol));

  Matrix splitH1, splitH2, H1Only, H2Only;
  const TGSO3 split = TGSO3::Expmap(omega, v, splitH1, splitH2);
  const TGSO3 splitFirst = TGSO3::Expmap(omega, v, H1Only, {});
  const TGSO3 splitSecond = TGSO3::Expmap(omega, v, {}, H2Only);
  const TGSO3 noJacobian = TGSO3::Expmap(omega, v);
  EXPECT(assert_equal(tangent, split, kTol));
  EXPECT(assert_equal(tangent, splitFirst, kTol));
  EXPECT(assert_equal(tangent, splitSecond, kTol));
  EXPECT(assert_equal(tangent, noJacobian, kTol));
  EXPECT(assert_equal(Matrix(poseH.leftCols<3>()), splitH1, kTol));
  EXPECT(assert_equal(Matrix(poseH.rightCols<3>()), splitH2, kTol));
  EXPECT(assert_equal(splitH1, H1Only, kTol));
  EXPECT(assert_equal(splitH2, H2Only, kTol));
}

// Checks Expmap and Logmap Jacobians against numerical derivatives.
TEST(TangentLieGroup, ExpmapLogmapJacobians) {
  const Vector12 xi = tgse3Xi();
  Matrix expH;
  TGSE3::Expmap(xi, expH);
  const auto expmap = [](const Vector12& v) { return TGSE3::Expmap(v); };
  EXPECT(assert_equal(numericalDerivative11(expmap, xi), expH, kNumTol));

  const TGSE3 state = tgse3State2();
  Matrix logH;
  TGSE3::Logmap(state, logH);
  const auto logmap = [](const TGSE3& value) { return TGSE3::Logmap(value); };
  EXPECT(assert_equal(numericalDerivative11(logmap, state), logH, kNumTol));
}

// Checks compose, between, and inverse Jacobians numerically.
TEST(TangentLieGroup, ComposeBetweenInverseJacobians) {
  const TGSE3 a = tgse3State(), b = tgse3State2();
  Matrix H1, H2;

  a.compose(b, H1, H2);
  EXPECT(assert_equal(numericalDerivative21(&TGSE3::operator*, a, b), H1,
                      kNumTol));
  EXPECT(assert_equal(numericalDerivative22(&TGSE3::operator*, a, b), H2,
                      kNumTol));

  a.between(b, H1, H2);
  const auto between = [](const TGSE3& x, const TGSE3& y) {
    return x.between(y);
  };
  EXPECT(assert_equal(numericalDerivative21(between, a, b), H1, kNumTol));
  EXPECT(assert_equal(numericalDerivative22(between, a, b), H2, kNumTol));

  Matrix H;
  a.inverse(H);
  const auto inverse = [](const TGSE3& value) { return value.inverse(); };
  EXPECT(assert_equal(numericalDerivative11(inverse, a), H, kNumTol));
}

// Exercises the centered operations and origin retract inherited from LieGroup,
// including static Retract on a recursively nested tangent group.
TEST(TangentLieGroup, InheritedLieGroupOperations) {
  const TGSE3 state = tgse3State();
  const Vector12 delta = 0.25 * tgse3Xi();

  Matrix H1, H2;
  const TGSE3 updated = state.expmap(delta, H1, H2);
  EXPECT(assert_equal(state.expmap(delta), updated, kTol));
  const auto expmap = [](const TGSE3& value, const Vector12& v) {
    return value.expmap(v);
  };
  EXPECT(
      assert_equal(numericalDerivative21(expmap, state, delta), H1, kNumTol));
  EXPECT(
      assert_equal(numericalDerivative22(expmap, state, delta), H2, kNumTol));

  Matrix L1, L2;
  EXPECT(assert_equal(delta, state.logmap(updated, L1, L2), kTol));
  const auto logmap = [](const TGSE3& value, const TGSE3& other) {
    return value.logmap(other);
  };
  EXPECT(
      assert_equal(numericalDerivative21(logmap, state, updated), L1, kNumTol));
  EXPECT(
      assert_equal(numericalDerivative22(logmap, state, updated), L2, kNumTol));

  Matrix actualH, expectedH;
  const TGSE3 actual = TGSE3::Retract(delta, actualH);
  const TGSE3 expected = TGSE3::Identity().retract(delta, {}, expectedH);
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(expectedH, actualH, kTol));

  Vector24 nestedDelta;
  nestedDelta << delta, -0.5 * delta;
  const TGTGSE3 nested = TGTGSE3::Retract(nestedDelta);
  EXPECT(assert_equal(nestedDelta, TGTGSE3::Logmap(nested), kTol));
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

  const Vector12 nestedFiber = tgse3Xi();
  const TGTGSE3 nested(x, nestedFiber);
  const Eigen::Matrix<double, 12, 12> nestedAd = x.AdjointMap();
  const Eigen::Matrix<double, 12, 12> nestedAdXi =
      TGSE3::adjointMap(nestedFiber);
  Eigen::Matrix<double, 24, 24> nestedExpected =
      Eigen::Matrix<double, 24, 24>::Zero();
  nestedExpected.topLeftCorner<12, 12>() = nestedAd;
  nestedExpected.bottomRightCorner<12, 12>() = nestedAd;
  nestedExpected.bottomLeftCorner<12, 12>() = nestedAdXi * nestedAd;
  EXPECT(
      assert_equal(Matrix(nestedExpected), Matrix(nested.AdjointMap()), kTol));
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

  Vector24 nestedXi;
  nestedXi << xi, -0.5 * xi;
  const Eigen::Matrix<double, 12, 12> nestedAdU = TGSE3::adjointMap(xi);
  const Eigen::Matrix<double, 12, 12> nestedAdV = TGSE3::adjointMap(-0.5 * xi);
  Eigen::Matrix<double, 24, 24> nestedExpected =
      Eigen::Matrix<double, 24, 24>::Zero();
  nestedExpected.topLeftCorner<12, 12>() = nestedAdU;
  nestedExpected.bottomRightCorner<12, 12>() = nestedAdU;
  nestedExpected.bottomLeftCorner<12, 12>() = nestedAdV;
  EXPECT(assert_equal(Matrix(nestedExpected),
                      Matrix(TGTGSE3::adjointMap(nestedXi)), kTol));
}

// Checks that Expmap and Logmap use the right Jacobian derived from the full
// tangent-group algebra adjoint.
TEST(TangentLieGroup, RightJacobianFromAlgebraAdjoint) {
  const Vector12 xi = tgse3Xi();
  Matrix expH;
  const TGSE3 state = TGSE3::Expmap(xi, expH);

  Eigen::Matrix<double, 24, 24> block = Eigen::Matrix<double, 24, 24>::Zero();
  block.topLeftCorner<12, 12>() = -TGSE3::adjointMap(xi);
  block.topRightCorner<12, 12>().setIdentity();
  const Eigen::Matrix<double, 24, 24> expBlock = block.exp();
  const Eigen::Matrix<double, 12, 12> expectedExpH =
      expBlock.topRightCorner<12, 12>();
  EXPECT(assert_equal(Matrix(expectedExpH), expH, kTol));

  Matrix logH;
  TGSE3::Logmap(state, logH);
  EXPECT(assert_equal(Matrix(expectedExpH.inverse()), logH, kTol));
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
  const auto expmap = [](const Vector20& v) { return TGGal3::Expmap(v); };
  EXPECT(assert_equal(numericalDerivative11(expmap, xi), expH, kNumTol));

  const TGGal3 state = TGGal3::Expmap(tggal3Xi() * 0.5);
  Matrix logH;
  TGGal3::Logmap(state, logH);
  const auto logmap = [](const TGGal3& value) { return TGGal3::Logmap(value); };
  EXPECT(assert_equal(numericalDerivative11(logmap, state), logH, kNumTol));
}

}  // namespace tangent_lie_group_fixture
/* ************************************************************************* */

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
