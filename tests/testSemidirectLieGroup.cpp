/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * --------------------------------------------------------------------------
 */

/**
 * @file testSemidirectLieGroup.cpp
 * @date April, 2026
 * @author Rohan Bansal
 * @author Jennifer Oum
 * @brief unit tests for semidirect Lie groups
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/SemidirectLieGroup.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/testLie.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>

using namespace gtsam;

/* ************************************************************************* */
namespace semidirect_product_fixture {

constexpr double kTol = 1e-9;

using Vector10 = Eigen::Matrix<double, 10, 1>;

// Rot2 acting on Point2 by rotation: φ(R, t) = R·t.
// The infinitesimal generator is Aφ(u) = Hat(u), a 2x2 skew matrix.
struct Rot2PointAction : public GroupAction<Rot2PointAction, Rot2, Point2> {
  static constexpr ActionType type = ActionType::Left;

  Point2 operator()(const Rot2& R, const Point2& t,
                    OptionalJacobian<2, 1> HR = {},
                    OptionalJacobian<2, 2> Ht = {}) const {
    return R.rotate(t, HR, Ht);
  }

  static Matrix2 generator(const Vector1& u) { return Rot2::Hat(u); }
};

// Rot3 acting on Vector3 by rotation: φ(R, t) = R·t.
// The infinitesimal generator Aφ(u)·t = d/dt(exp(tu∧)·t)|₀ = u∧·t,
// so generator(u) = u∧ (skew-symmetric matrix). SemidirectLieGroup derives
// Expmap and Logmap automatically via φ₁(u∧) = SO(3) left Jacobian.
struct Rot3VectorAction : public GroupAction<Rot3VectorAction, Rot3, Vector3> {
  static constexpr ActionType type = ActionType::Left;

  Vector3 operator()(const Rot3& R, const Vector3& t,
                     OptionalJacobian<3, 3> HR = {},
                     OptionalJacobian<3, 3> Ht = {}) const {
    return R.rotate(t, HR, Ht);
  }

  static Matrix3 generator(const Vector3& u) { return skewSymmetric(u); }
};

// SE(3) acting on ℝ⁴=(position p, time s): φ((R,ν),[p;s]) = [R·p+ν·s; s], the
// 4×4 homogeneous matrix M(g)=[[R,ν],[0,1]] times [p;s]. The Pose3 translation
// slot holds the velocity-boost ν. Reconstructs Gal(3) = SE(3) ⋉ ℝ⁴.
struct SE3Vector4Action : public GroupAction<SE3Vector4Action, Pose3, Vector4> {
  static constexpr ActionType type = ActionType::Left;

  Vector4 operator()(const Pose3& g, const Vector4& h,
                     OptionalJacobian<4, 6> Hg = {},
                     OptionalJacobian<4, 4> Hh = {}) const {
    const Rot3& R = g.rotation();
    const Point3 nu = g.translation();  // velocity-boost
    const Vector3 p = h.head<3>();
    const double s = h(3);

    Matrix3 D_Rp_R, D_Rp_p;
    const Point3 Rp =
        R.rotate(p, Hg ? &D_Rp_R : nullptr, Hh ? &D_Rp_p : nullptr);

    Vector4 out;
    out << (Rp + s * nu), s;

    if (Hg) {
      // g·Exp([ω;ρ]) ≈ (R·Exp(ω^), ν + R·ρ): ∂/∂ω = -R·p^ (=D_Rp_R), ∂/∂ρ =
      // s·R.
      Hg->setZero();
      Hg->topLeftCorner<3, 3>() = D_Rp_R;
      Hg->block<3, 3>(0, 3) = s * R.matrix();
    }
    if (Hh) {
      // ∂φ/∂h = M(g) = [[R, ν],[0,1]].
      Hh->setZero();
      Hh->topLeftCorner<3, 3>() = D_Rp_p;  // ∂/∂p = R
      Hh->block<3, 1>(0, 3) = nu;          // ∂/∂s = ν
      (*Hh)(3, 3) = 1.0;
    }
    return out;
  }

  /// Generator Aφ([ω;ρ]) = 4×4 se(3) hat = [[ω^, ρ],[0,0]].
  static Matrix4 generator(const Vector6& u) {
    Matrix4 A = Matrix4::Zero();
    A.topLeftCorner<3, 3>() = skewSymmetric(u.head<3>());
    A.block<3, 1>(0, 3) = u.tail<3>();
    return A;
  }
};

using Semidirect2 = SemidirectLieGroup<Rot2, Point2, Rot2PointAction>;
using Semidirect = SemidirectLieGroup<Rot3, Vector3, Rot3VectorAction>;
using SemidirectGal3 = SemidirectLieGroup<Pose3, Vector4, SE3Vector4Action>;

template <typename Product, typename BaseTangent, typename ActionTangent>
bool expmapRequestCombinationsAreConsistent(
    const BaseTangent& baseTangent, const ActionTangent& actionTangent) {
  Matrix bothH1, bothH2, onlyH1, onlyH2;
  const Product both =
      Product::Expmap(baseTangent, actionTangent, bothH1, bothH2);
  const Product firstOnly =
      Product::Expmap(baseTangent, actionTangent, onlyH1, {});
  const Product secondOnly =
      Product::Expmap(baseTangent, actionTangent, {}, onlyH2);
  const Product neither = Product::Expmap(baseTangent, actionTangent);

  return assert_equal(both, firstOnly, kTol) &&
         assert_equal(both, secondOnly, kTol) &&
         assert_equal(both, neither, kTol) &&
         assert_equal(bothH1, onlyH1, kTol) &&
         assert_equal(bothH2, onlyH2, kTol);
}

Semidirect2 semidirect2State() {
  return Semidirect2(Rot2::fromAngle(0.35), Point2(0.4, -0.6));
}

Vector3 semidirect2Xi() { return Vector3{0.25, 0.3, -0.2}; }

Vector3 pose2XiFromSemidirect2(const Vector3& xi) {
  return Vector3{xi(1), xi(2), xi(0)};
}

Vector3 semidirect2XiFromPose2(const Vector3& xi) {
  return Vector3{xi(2), xi(0), xi(1)};
}

Pose2 asPose2(const Semidirect2& state) {
  return Pose2(state.first, state.second);
}

Semidirect semidirectState1() {
  return Semidirect(Rot3::RzRyRx(0.1, 0.2, -0.3), Vector3(1.0, -0.5, 0.25));
}

Semidirect semidirectState2() {
  return Semidirect(Rot3::RzRyRx(-0.2, 0.1, 0.15), Vector3(-0.75, 0.4, 1.2));
}

Semidirect semidirectState3() {
  return Semidirect(Rot3::RzRyRx(0.2, -0.1, 0.05), Vector3(0.3, -0.6, 0.8));
}

Semidirect semidirectState4() {
  return Semidirect(Rot3::RzRyRx(0.1, -0.2, 0.3), Vector3(0.4, -0.1, 0.2));
}

Vector6 semidirectXi() { return Vector6{0.1, -0.2, 0.3, 0.4, -0.1, 0.2}; }

Vector6 retractDelta() { return Vector6{0.05, -0.04, 0.03, 0.1, -0.2, 0.05}; }

Pose3 asPose3(const Semidirect& state) {
  return Pose3(state.first, state.second);
}

Gal3 asGal3(const SemidirectGal3& s) {
  // Pose3.translation() holds the velocity-boost; H.head<3>() is the position.
  return Gal3(s.first.rotation(), s.second.head<3>(), s.first.translation(),
              s.second(3));
}

Vector10 gal3Xi() {
  Vector10 xi;
  xi << 0.1, -0.2, 0.3, 0.4, -0.1, 0.2, 0.15, -0.25, 0.3, 0.2;
  return xi;
}

SemidirectGal3 gal3State1() {
  return SemidirectGal3(
      Pose3(Rot3::RzRyRx(0.1, 0.2, -0.3), Point3(1.0, -0.5, 0.25)),
      (Vector4() << 0.4, -0.2, 0.7, 0.3).finished());
}

SemidirectGal3 gal3State2() {
  return SemidirectGal3(
      Pose3(Rot3::RzRyRx(-0.2, 0.1, 0.15), Point3(-0.75, 0.4, 1.2)),
      (Vector4() << -0.3, 0.5, -0.1, 0.6).finished());
}

SemidirectGal3 gal3State3() {
  return SemidirectGal3(
      Pose3(Rot3::RzRyRx(0.2, -0.1, 0.05), Point3(0.3, -0.6, 0.8)),
      (Vector4() << 0.2, 0.1, -0.4, 0.2).finished());
}

SemidirectGal3 gal3State4() {
  return SemidirectGal3(
      Pose3(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(0.4, -0.1, 0.2)),
      (Vector4() << 0.15, -0.25, 0.3, 0.2).finished());
}

Vector10 gal3RetractDelta() {
  Vector10 delta;
  delta << 0.05, -0.04, 0.03, 0.1, -0.2, 0.05, 0.02, -0.03, 0.04, 0.01;
  return delta;
}

// A second semidirect instance, Rot2 ⋉ R², checks the generic kernel path in
// a different dimension with Pose2 as the oracle.
TEST(Lie, SemidirectLieGroupAction2D) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Semidirect2>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Semidirect2>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Semidirect2>);

  const Rot2PointAction action;
  const Rot2 R1 = Rot2::fromAngle(0.3);
  const Rot2 R2 = Rot2::fromAngle(-0.2);
  const Point2 t(0.4, -0.5);
  EXPECT_LEFT_ACTION(action, R1, R2, t);

  const Vector1 u{0.35};
  const Point2 p(0.2, -0.7);
  const double eps = 1e-7;
  const Point2 generatorAction = Rot2PointAction::generator(u) * p;
  const Point2 generatorFiniteDifference =
      (Rot2::Expmap((Vector1() << eps * u(0)).finished()).rotate(p) - p) / eps;
  EXPECT(assert_equal(generatorAction, generatorFiniteDifference, 1e-6));

  const Vector3 xi = semidirect2Xi();
  const Semidirect2 actual = Semidirect2::Expmap(xi);
  EXPECT(assert_equal(Pose2::Expmap(pose2XiFromSemidirect2(xi)),
                      asPose2(actual), kTol));
  EXPECT(assert_equal(
      xi, semidirect2XiFromPose2(Pose2::Logmap(asPose2(actual))), kTol));
}

// Check Expmap/Logmap values and Jacobians for a second semidirect product,
// independent of the Pose3-specific ordering conventions.
TEST(testActionProduct, ExpmapLogmap2D) {
  const Vector3 xi = semidirect2Xi();

  Matrix expH;
  const Semidirect2 actual = Semidirect2::Expmap(xi, expH);
  const auto expmap = [](const Vector3& v) { return Semidirect2::Expmap(v); };
  const Matrix numericExpH = numericalDerivative11(expmap, xi);

  EXPECT(assert_equal(Pose2::Expmap(pose2XiFromSemidirect2(xi)),
                      asPose2(actual), kTol));
  EXPECT(assert_equal(numericExpH, expH, 1e-6));

  const Semidirect2 state = semidirect2State();
  Matrix logH;
  const Vector3 actualLog = Semidirect2::Logmap(state, logH);
  const auto logmap = [](const Semidirect2& value) {
    return Semidirect2::Logmap(value);
  };
  const Matrix numericLogH = numericalDerivative11(logmap, state);

  EXPECT(assert_equal(semidirect2XiFromPose2(Pose2::Logmap(asPose2(state))),
                      actualLog, kTol));
  EXPECT(assert_equal(numericLogH, logH, 1e-6));
}

// Verify the semidirect product obeys the left action law and matches Pose3
// behavior.
TEST(Lie, SemidirectLieGroupAction) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Semidirect>);
  GTSAM_CONCEPT_ASSERT(IsManifold<Semidirect>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<Semidirect>);

  const Rot3VectorAction action;
  const Rot3 R1 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Rot3 R2 = Rot3::RzRyRx(-0.2, 0.05, 0.1);
  const Vector3 t(0.4, -0.5, 0.6);
  EXPECT_LEFT_ACTION(action, R1, R2, t);

  const Semidirect identity;
  const Vector6 xi = semidirectXi();
  const Semidirect actual = identity.expmap(xi);
  EXPECT(assert_equal(Pose3::Expmap(xi), asPose3(actual), kTol));
  EXPECT(assert_equal(xi, identity.logmap(actual), kTol));

  const Semidirect a =
      Semidirect(Rot3::RzRyRx(0.1, 0.2, -0.1), Vector3(1.0, -2.0, 0.5));
  const Semidirect b =
      Semidirect(Rot3::RzRyRx(-0.3, 0.15, 0.2), Vector3(-0.25, 0.4, 1.5));
  const Semidirect c = semidirectState3();
  EXPECT(assert_equal(asPose3((a * b) * c), asPose3(a * (b * c)), kTol));
  EXPECT(assert_equal(asPose3(a * a.inverse()), Pose3(), kTol));
  EXPECT(assert_equal(asPose3(a * b), asPose3(a) * asPose3(b), kTol));
}

// Check semidirect compose values and Jacobians against Pose3 and numerical
// derivatives.
TEST(testActionProduct, compose) {
  const Semidirect state1 = semidirectState1();
  const Semidirect state2 = semidirectState2();

  Matrix actH1, actH2;
  const Semidirect actual = state1.compose(state2, actH1, actH2);
  const Matrix numericH1 =
      numericalDerivative21(&Semidirect::operator*, state1, state2);
  const Matrix numericH2 =
      numericalDerivative22(&Semidirect::operator*, state1, state2);

  EXPECT(
      assert_equal(asPose3(actual), asPose3(state1) * asPose3(state2), kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check semidirect between values and Jacobians against Pose3 and numerical
// derivatives.
TEST(testActionProduct, between) {
  const Semidirect state1 = semidirectState1();
  const Semidirect state2 = semidirectState2();

  Matrix actH1, actH2;
  const Semidirect actual = state1.between(state2, actH1, actH2);
  const auto between = [](const Semidirect& a, const Semidirect& b) {
    return a.between(b);
  };
  const Matrix numericH1 = numericalDerivative21(between, state1, state2);
  const Matrix numericH2 = numericalDerivative22(between, state1, state2);

  EXPECT(assert_equal(asPose3(actual), asPose3(state1).between(asPose3(state2)),
                      kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check the semidirect inverse and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, inverse) {
  const Semidirect state =
      Semidirect(Rot3::RzRyRx(0.2, -0.1, 0.05), Vector3(0.5, -1.2, 0.8));

  Matrix actH;
  const Semidirect actual = state.inverse(actH);
  const auto inverse = [](const Semidirect& value) { return value.inverse(); };
  const Matrix numericH = numericalDerivative11(inverse, state);

  EXPECT(assert_equal(asPose3(actual), asPose3(state).inverse(), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check the semidirect Expmap and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, Expmap) {
  const Vector6 xi = semidirectXi();

  Matrix actH;
  const Semidirect actual = Semidirect::Expmap(xi, actH);
  const auto expmap = [](const Vector6& v) { return Semidirect::Expmap(v); };
  const Matrix numericH = numericalDerivative11(expmap, xi);

  const Vector3 omega = xi.head<3>();
  const Vector3 velocity = xi.tail<3>();
  Matrix H2Only;
  const Semidirect splitActual =
      Semidirect::Expmap(omega, velocity, {}, H2Only);

  EXPECT(assert_equal(Pose3::Expmap(xi), asPose3(actual), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
  EXPECT(assert_equal(actual, splitActual, kTol));
  EXPECT(assert_equal(actH.rightCols<3>(), H2Only, 1e-6));
}

// Every optional-Jacobian combination returns the same value and matching
// requested columns for representative semidirect actions.
TEST(testActionProduct, ExpmapRequestCombinations) {
  const Vector3 xi2 = semidirect2Xi();
  EXPECT(expmapRequestCombinationsAreConsistent<Semidirect2>(xi2.head<1>(),
                                                             xi2.tail<2>()));

  const Vector6 xi3 = semidirectXi();
  EXPECT(expmapRequestCombinationsAreConsistent<Semidirect>(xi3.head<3>(),
                                                            xi3.tail<3>()));

  const Vector10 xiGal3 = gal3Xi();
  EXPECT(expmapRequestCombinationsAreConsistent<SemidirectGal3>(
      xiGal3.head<6>(), xiGal3.tail<4>()));
}

// Check the semidirect Logmap and its Jacobian against Pose3 and numerical
// derivatives.
TEST(testActionProduct, Logmap) {
  const Semidirect state = semidirectState4();

  Matrix actH;
  const Vector6 actual = Semidirect::Logmap(state, actH);
  const auto logmap = [](const Semidirect& value) {
    return Semidirect::Logmap(value);
  };
  const Matrix numericH = numericalDerivative11(logmap, state);

  EXPECT(assert_equal(Pose3::Logmap(asPose3(state)), actual, kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

/* ************************************************************************* */
// Exercises the centered operations and origin retract inherited from LieGroup.
TEST(testActionProduct, InheritedLieGroupOperations) {
  const Semidirect state = semidirectState1();
  const Vector6 delta = retractDelta();

  Matrix H1, H2;
  const Semidirect updated = state.expmap(delta, H1, H2);
  EXPECT(assert_equal(state.expmap(delta), updated, kTol));
  const auto expmap = [](const Semidirect& value, const Vector6& v) {
    return value.expmap(v);
  };
  EXPECT(assert_equal(numericalDerivative21(expmap, state, delta), H1, kTol));
  EXPECT(assert_equal(numericalDerivative22(expmap, state, delta), H2, kTol));

  Matrix L1, L2;
  EXPECT(assert_equal(delta, state.logmap(updated, L1, L2), kTol));
  const auto logmap = [](const Semidirect& value, const Semidirect& other) {
    return value.logmap(other);
  };
  EXPECT(assert_equal(numericalDerivative21(logmap, state, updated), L1, kTol));
  EXPECT(assert_equal(numericalDerivative22(logmap, state, updated), L2, kTol));

  Matrix actualH, expectedH;
  const Semidirect actual = Semidirect::Retract(delta, actualH);
  const Semidirect expected =
      Semidirect::Identity().retract(delta, {}, expectedH);
  EXPECT(assert_equal(expected, actual, kTol));
  EXPECT(assert_equal(expectedH, actualH, kTol));
}

// Check that the semidirect adjoint matches the Pose3 adjoint.
TEST(testActionProduct, AdjointMap) {
  const Semidirect state = semidirectState4();

  EXPECT(assert_equal(asPose3(state).AdjointMap(), state.AdjointMap(), kTol));
}

// Check Expmap-based retract/localCoordinates consistency and both Jacobians
// against numerical derivatives.
TEST(testActionProduct, retractAndLocalCoordinates) {
  const Semidirect state = semidirectState4();
  const Vector6 delta = retractDelta();

  Matrix retractH1, retractH2, localH1, localH2;
  const Semidirect updated = state.retract(delta, retractH1, retractH2);
  const Vector6 recovered = state.localCoordinates(updated, localH1, localH2);

  EXPECT(assert_equal(asPose3(updated),
                      asPose3(state).compose(Pose3::Expmap(delta)), kTol));
  EXPECT(assert_equal(delta, recovered, kTol));

  const auto retract = [](const Semidirect& value, const Vector6& v) {
    return value.retract(v);
  };
  const auto local = [](const Semidirect& value, const Semidirect& other) {
    return value.localCoordinates(other);
  };
  const Matrix numericRetractH1 = numericalDerivative21(retract, state, delta);
  const Matrix numericRetractH2 = numericalDerivative22(retract, state, delta);
  const Matrix numericLocalH1 = numericalDerivative21(local, state, updated);
  const Matrix numericLocalH2 = numericalDerivative22(local, state, updated);

  EXPECT(assert_equal(numericRetractH1, retractH1, 1e-6));
  EXPECT(assert_equal(numericRetractH2, retractH2, 1e-6));
  EXPECT(assert_equal(numericLocalH1, localH1, 1e-6));
  EXPECT(assert_equal(numericLocalH2, localH2, 1e-6));
}

// A third semidirect instance, SE(3) ⋉ ℝ⁴, reconstructs Gal(3) and exercises a
// non-trivial (Pose3) base with a 4-D action, using the native Gal3 as oracle.
TEST(Lie, SemidirectLieGroupActionGal3) {
  GTSAM_CONCEPT_ASSERT(IsGroup<SemidirectGal3>);
  GTSAM_CONCEPT_ASSERT(IsManifold<SemidirectGal3>);
  GTSAM_CONCEPT_ASSERT(IsLieGroup<SemidirectGal3>);
  EXPECT_LONGS_EQUAL(10, SemidirectGal3::dimension);

  const SE3Vector4Action action;
  const Pose3 g1 = Pose3(Rot3::RzRyRx(0.1, -0.2, 0.3), Point3(0.4, -0.5, 0.6));
  const Pose3 g2 = Pose3(Rot3::RzRyRx(-0.2, 0.05, 0.1), Point3(-0.3, 0.2, 0.1));
  const Vector4 h = (Vector4() << 0.4, -0.5, 0.6, 0.3).finished();
  EXPECT_LEFT_ACTION(action, g1, g2, h);

  const Vector6 u = (Vector6() << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6).finished();
  const Vector4 p = (Vector4() << 0.2, -0.7, 0.5, 0.4).finished();
  const double eps = 1e-7;
  const Vector4 generatorAction = SE3Vector4Action::generator(u) * p;
  const Vector4 generatorFiniteDifference =
      (action(Pose3::Expmap(eps * u), p) - p) / eps;
  EXPECT(assert_equal(generatorAction, generatorFiniteDifference, 1e-6));

  const Vector10 xi = gal3Xi();
  const SemidirectGal3 actual = SemidirectGal3::Expmap(xi);
  EXPECT(assert_equal(Gal3::Expmap(xi), asGal3(actual), kTol));
  EXPECT(assert_equal(xi, SemidirectGal3::Logmap(actual), kTol));

  const SemidirectGal3 a = gal3State1();
  const SemidirectGal3 b = gal3State2();
  const SemidirectGal3 c = gal3State3();
  EXPECT(assert_equal(asGal3((a * b) * c), asGal3(a * (b * c)), kTol));
  EXPECT(assert_equal(asGal3(a * a.inverse()), Gal3(), kTol));
  EXPECT(assert_equal(asGal3(a * b), asGal3(a) * asGal3(b), kTol));
}

// Check Gal(3) compose values and Jacobians against native Gal3 and numerical
// derivatives.
TEST(testActionProduct, composeGal3) {
  const SemidirectGal3 state1 = gal3State1();
  const SemidirectGal3 state2 = gal3State2();

  Matrix actH1, actH2;
  const SemidirectGal3 actual = state1.compose(state2, actH1, actH2);
  const Matrix numericH1 =
      numericalDerivative21(&SemidirectGal3::operator*, state1, state2);
  const Matrix numericH2 =
      numericalDerivative22(&SemidirectGal3::operator*, state1, state2);

  EXPECT(assert_equal(asGal3(actual), asGal3(state1) * asGal3(state2), kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check Gal(3) between values and Jacobians against native Gal3 and numerical
// derivatives.
TEST(testActionProduct, betweenGal3) {
  const SemidirectGal3 state1 = gal3State1();
  const SemidirectGal3 state2 = gal3State2();

  Matrix actH1, actH2;
  const SemidirectGal3 actual = state1.between(state2, actH1, actH2);
  const auto between = [](const SemidirectGal3& a, const SemidirectGal3& b) {
    return a.between(b);
  };
  const Matrix numericH1 = numericalDerivative21(between, state1, state2);
  const Matrix numericH2 = numericalDerivative22(between, state1, state2);

  EXPECT(assert_equal(asGal3(actual), asGal3(state1).between(asGal3(state2)),
                      kTol));
  EXPECT(assert_equal(numericH1, actH1, 1e-6));
  EXPECT(assert_equal(numericH2, actH2, 1e-6));
}

// Check the Gal(3) inverse and its Jacobian against native Gal3 and numerical
// derivatives.
TEST(testActionProduct, inverseGal3) {
  const SemidirectGal3 state = gal3State4();

  Matrix actH;
  const SemidirectGal3 actual = state.inverse(actH);
  const auto inverse = [](const SemidirectGal3& value) {
    return value.inverse();
  };
  const Matrix numericH = numericalDerivative11(inverse, state);

  EXPECT(assert_equal(asGal3(actual), asGal3(state).inverse(), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check the Gal(3) Expmap and its Jacobian against native Gal3 and numerical
// derivatives.
TEST(testActionProduct, ExpmapGal3) {
  const Vector10 xi = gal3Xi();

  Matrix actH;
  const SemidirectGal3 actual = SemidirectGal3::Expmap(xi, actH);
  const auto expmap = [](const Vector10& v) {
    return SemidirectGal3::Expmap(v);
  };
  const Matrix numericH = numericalDerivative11(expmap, xi);

  EXPECT(assert_equal(Gal3::Expmap(xi), asGal3(actual), kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check the Gal(3) Logmap and its Jacobian against native Gal3 and numerical
// derivatives.
TEST(testActionProduct, LogmapGal3) {
  const SemidirectGal3 state = gal3State4();

  Matrix actH;
  const Vector10 actual = SemidirectGal3::Logmap(state, actH);
  const auto logmap = [](const SemidirectGal3& value) {
    return SemidirectGal3::Logmap(value);
  };
  const Matrix numericH = numericalDerivative11(logmap, state);

  EXPECT(assert_equal(Gal3::Logmap(asGal3(state)), actual, kTol));
  EXPECT(assert_equal(numericH, actH, 1e-6));
}

// Check that the Gal(3) adjoint matches the native Gal3 adjoint.
TEST(testActionProduct, AdjointMapGal3) {
  const SemidirectGal3 state = gal3State4();

  EXPECT(assert_equal(Matrix(asGal3(state).AdjointMap()),
                      Matrix(state.AdjointMap()), kTol));
}

// Static algebra adjoint of the semidirect product matches native Gal3::ad.
TEST(testActionProduct, AlgebraAdjointGal3) {
  const Vector10 xi = gal3Xi();
  EXPECT(assert_equal(Matrix(Gal3::adjointMap(xi)),
                      Matrix(SemidirectGal3::adjointMap(xi)), kTol));
}

// Check Expmap-based retract/localCoordinates consistency and both Jacobians
// against numerical derivatives.
TEST(testActionProduct, retractAndLocalCoordinatesGal3) {
  const SemidirectGal3 state = gal3State4();
  const Vector10 delta = gal3RetractDelta();

  Matrix retractH1, retractH2, localH1, localH2;
  const SemidirectGal3 updated = state.retract(delta, retractH1, retractH2);
  const Vector10 recovered = state.localCoordinates(updated, localH1, localH2);

  EXPECT(assert_equal(asGal3(updated),
                      asGal3(state).compose(Gal3::Expmap(delta)), kTol));
  EXPECT(assert_equal(delta, recovered, kTol));

  const auto retract = [](const SemidirectGal3& value, const Vector10& v) {
    return value.retract(v);
  };
  const auto local = [](const SemidirectGal3& value,
                        const SemidirectGal3& other) {
    return value.localCoordinates(other);
  };
  const Matrix numericRetractH1 = numericalDerivative21(retract, state, delta);
  const Matrix numericRetractH2 = numericalDerivative22(retract, state, delta);
  const Matrix numericLocalH1 = numericalDerivative21(local, state, updated);
  const Matrix numericLocalH2 = numericalDerivative22(local, state, updated);

  EXPECT(assert_equal(numericRetractH1, retractH1, 1e-6));
  EXPECT(assert_equal(numericRetractH2, retractH2, 1e-6));
  EXPECT(assert_equal(numericLocalH1, localH1, 1e-6));
  EXPECT(assert_equal(numericLocalH2, localH2, 1e-6));
}

}  // namespace semidirect_product_fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace frechet_fallback_fixture {

constexpr double kTol = 1e-9;
constexpr double kNumericalTolerance = 1e-6;

/** SO(2) wrapper intentionally omitting static adjointMap(). */
class Rot2WithoutAlgebraAdjoint
    : public LieGroup<Rot2WithoutAlgebraAdjoint, 1> {
  Rot2 rotation_;

 public:
  Rot2WithoutAlgebraAdjoint() = default;
  explicit Rot2WithoutAlgebraAdjoint(const Rot2& rotation)
      : rotation_(rotation) {}

  static Rot2WithoutAlgebraAdjoint Identity() {
    return Rot2WithoutAlgebraAdjoint();
  }

  Rot2WithoutAlgebraAdjoint operator*(
      const Rot2WithoutAlgebraAdjoint& other) const {
    return Rot2WithoutAlgebraAdjoint(rotation_ * other.rotation_);
  }

  Rot2WithoutAlgebraAdjoint inverse() const {
    return Rot2WithoutAlgebraAdjoint(rotation_.inverse());
  }

  using LieGroup<Rot2WithoutAlgebraAdjoint, 1>::inverse;

  static Rot2WithoutAlgebraAdjoint Expmap(const Vector1& tangent,
                                          ChartJacobian H = {}) {
    return Rot2WithoutAlgebraAdjoint(Rot2::Expmap(tangent, H));
  }

  static Vector1 Logmap(const Rot2WithoutAlgebraAdjoint& rotation,
                        ChartJacobian H = {}) {
    return Rot2::Logmap(rotation.rotation_, H);
  }

  Matrix1 AdjointMap() const { return I_1x1; }

  struct ChartAtOrigin {
    static Rot2WithoutAlgebraAdjoint Retract(const Vector1& tangent,
                                             ChartJacobian H = {}) {
      return Expmap(tangent, H);
    }

    static Vector1 Local(const Rot2WithoutAlgebraAdjoint& rotation,
                         ChartJacobian H = {}) {
      return Logmap(rotation, H);
    }
  };

  const Rot2& rotation() const { return rotation_; }

  void print(const std::string& label = "") const { rotation_.print(label); }

  bool equals(const Rot2WithoutAlgebraAdjoint& other,
              double tolerance = 1e-9) const {
    return rotation_.equals(other.rotation_, tolerance);
  }
};

}  // namespace frechet_fallback_fixture
/* ************************************************************************* */

namespace gtsam {

template <>
struct traits<frechet_fallback_fixture::Rot2WithoutAlgebraAdjoint>
    : internal::LieGroup<frechet_fallback_fixture::Rot2WithoutAlgebraAdjoint> {
};

template <>
struct traits<const frechet_fallback_fixture::Rot2WithoutAlgebraAdjoint>
    : traits<frechet_fallback_fixture::Rot2WithoutAlgebraAdjoint> {};

}  // namespace gtsam

/* ************************************************************************* */
namespace frechet_fallback_fixture {

struct RotationAction
    : public GroupAction<RotationAction, Rot2WithoutAlgebraAdjoint, Point2> {
  static constexpr ActionType type = ActionType::Left;

  Point2 operator()(const Rot2WithoutAlgebraAdjoint& rotation,
                    const Point2& point, OptionalJacobian<2, 1> Hrotation = {},
                    OptionalJacobian<2, 2> Hpoint = {}) const {
    return rotation.rotation().rotate(point, Hrotation, Hpoint);
  }

  static Matrix2 generator(const Vector1& tangent) {
    return Rot2::Hat(tangent);
  }
};

using FallbackProduct =
    SemidirectLieGroup<Rot2WithoutAlgebraAdjoint, Point2, RotationAction>;

// A base without static adjointMap exercises the reduced vector-valued
// Fréchet fallback for both Expmap and Logmap Jacobians.
TEST(testActionProduct, ReducedFrechetFallback) {
  const Vector3 tangent = (Vector3() << 0.25, 0.3, -0.2).finished();
  Matrix expmapJacobian;
  const FallbackProduct product =
      FallbackProduct::Expmap(tangent, expmapJacobian);
  const auto expmap = [](const Vector3& v) {
    return FallbackProduct::Expmap(v);
  };
  EXPECT(assert_equal(numericalDerivative11(expmap, tangent), expmapJacobian,
                      kNumericalTolerance));

  Matrix logmapJacobian;
  const Vector3 recovered = FallbackProduct::Logmap(product, logmapJacobian);
  EXPECT(assert_equal(tangent, recovered, kTol));
  const auto logmap = [](const FallbackProduct& value) {
    return FallbackProduct::Logmap(value);
  };
  EXPECT(assert_equal(numericalDerivative11(logmap, product), logmapJacobian,
                      kNumericalTolerance));
}

}  // namespace frechet_fallback_fixture
/* ************************************************************************* */

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
