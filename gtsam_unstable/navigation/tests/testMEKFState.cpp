/**
 * @file  testMEKFState.cpp
 * @brief Unit tests for the MEKFState manifold (Retract/Local, identity).
 *
 * Tangent block order: R, v, p, b_gyro, b_accel.
 * Body/right multiplicative attitude error: R_est = R_ref * Exp(d_theta).
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/MEKFGeometry.h>

using namespace gtsam::mekf;
using namespace gtsam;

static constexpr double kTol = 1e-9;

using Tangent = Eigen::Matrix<double, 15, 1>;

// assert_equal overloads take dynamic gtsam::Vector; explicit cast resolves
// ambiguity for fixed-size Eigen types.
static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}

// ---------------------------------------------------------------------------
// identity
// ---------------------------------------------------------------------------

TEST(MEKFState, IdentityIsZero) {
  MEKFState xi = MEKFState::identity();
  EXPECT(xi.R.equals(Rot3::Identity(), kTol));
  EXPECT(veq(xi.v, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.p, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.b_gyro, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.b_accel, Eigen::Vector3d::Zero()));
}

TEST(MEKFState, DirectConstructor) {
  const Rot3 R = Rot3::Rz(0.4);
  const Eigen::Vector3d v(1, 2, 3), p(4, 5, 6), bg(0.1, 0.2, 0.3),
      ba(-0.1, 0.0, 0.1);
  MEKFState xi(R, v, p, bg, ba);
  EXPECT(xi.R.equals(R, kTol));
  EXPECT(veq(xi.v, v));
  EXPECT(veq(xi.p, p));
  EXPECT(veq(xi.b_gyro, bg));
  EXPECT(veq(xi.b_accel, ba));
}

// ---------------------------------------------------------------------------
// Equals
// ---------------------------------------------------------------------------

TEST(MEKFState, EqualsIdentities) {
  EXPECT(traits<MEKFState>::Equals(MEKFState::identity(), MEKFState::identity(),
                                   kTol));
}

// Each field must contribute to equality: perturb one block at a time.
TEST(MEKFState, EqualsDistinguishesEveryBlock) {
  const MEKFState base = MEKFState::identity();
  {
    MEKFState b = base;
    b.R = Rot3::Rx(0.01);
    EXPECT(!traits<MEKFState>::Equals(base, b, kTol));
  }
  {
    MEKFState b = base;
    b.v = Eigen::Vector3d(1, 0, 0);
    EXPECT(!traits<MEKFState>::Equals(base, b, kTol));
  }
  {
    MEKFState b = base;
    b.p = Eigen::Vector3d(0, 1, 0);
    EXPECT(!traits<MEKFState>::Equals(base, b, kTol));
  }
  {
    MEKFState b = base;
    b.b_gyro = Eigen::Vector3d(0, 0, 1);
    EXPECT(!traits<MEKFState>::Equals(base, b, kTol));
  }
  {
    MEKFState b = base;
    b.b_accel = Eigen::Vector3d(0, 1, 0);
    EXPECT(!traits<MEKFState>::Equals(base, b, kTol));
  }
}

// ---------------------------------------------------------------------------
// Retract / Local
// ---------------------------------------------------------------------------

// Retract(xi, 0) == xi for a non-identity state.
TEST(MEKFState, RetractZeroDeltaIsNoop) {
  MEKFState xi(Rot3::Ry(0.3), Eigen::Vector3d(0.1, 0.2, 0.3),
               Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(0.01, 0.02, 0.03),
               Eigen::Vector3d(-0.01, 0.0, 0.01));
  MEKFState result = traits<MEKFState>::Retract(xi, Tangent::Zero());
  EXPECT(traits<MEKFState>::Equals(xi, result, kTol));
}

// Verify each tangent block maps to the right state field (ordering contract).
TEST(MEKFState, RetractBlockOrdering) {
  const MEKFState xi = MEKFState::identity();
  Tangent d = Tangent::Zero();
  d.segment<3>(0) = Eigen::Vector3d(0.1, 0.0, 0.0);      // d_theta
  d.segment<3>(3) = Eigen::Vector3d(1.0, 2.0, 3.0);      // d_v
  d.segment<3>(6) = Eigen::Vector3d(4.0, 5.0, 6.0);      // d_p
  d.segment<3>(9) = Eigen::Vector3d(0.7, 0.8, 0.9);      // d_b_gyro
  d.segment<3>(12) = Eigen::Vector3d(-0.1, -0.2, -0.3);  // d_b_accel

  MEKFState r = traits<MEKFState>::Retract(xi, d);
  EXPECT(r.R.equals(Rot3::Expmap(d.segment<3>(0)), kTol));
  EXPECT(veq(r.v, d.segment<3>(3)));
  EXPECT(veq(r.p, d.segment<3>(6)));
  EXPECT(veq(r.b_gyro, d.segment<3>(9)));
  EXPECT(veq(r.b_accel, d.segment<3>(12)));
}

// Attitude retraction is right-multiplicative: R_est = R_ref * Exp(d_theta).
TEST(MEKFState, RetractAttitudeIsRightMultiplicative) {
  const Rot3 R0 = Rot3::Rz(0.5) * Rot3::Rx(0.2);
  MEKFState xi(R0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::Vector3d dtheta(0.05, -0.1, 0.02);
  Tangent d = Tangent::Zero();
  d.segment<3>(0) = dtheta;
  MEKFState r = traits<MEKFState>::Retract(xi, d);
  EXPECT(r.R.equals(R0 * Rot3::Expmap(dtheta), kTol));
}

// Local(ref, Retract(ref, delta)) == delta at the identity origin.
TEST(MEKFState, RetractLocalRoundTripFromIdentity) {
  Tangent delta;
  delta << 0.1, -0.2, 0.05, 1.0, 2.0, 3.0, -1.0, 0.5, 0.2, 0.01, 0.02, 0.03,
      -0.01, 0.0, 0.01;
  const MEKFState ref = MEKFState::identity();
  MEKFState ret = traits<MEKFState>::Retract(ref, delta);
  Tangent rec = traits<MEKFState>::Local(ref, ret);
  EXPECT(assert_equal((Vector)delta, (Vector)rec, kTol));
}

// Local(ref, Retract(ref, delta)) == delta for a non-identity reference.
TEST(MEKFState, RetractLocalRoundTripFromNonIdentity) {
  MEKFState ref(Rot3::Rz(0.5) * Rot3::Rx(0.3), Eigen::Vector3d(1.0, 0.5, -0.2),
                Eigen::Vector3d(10.0, -5.0, 2.0),
                Eigen::Vector3d(0.01, -0.01, 0.02),
                Eigen::Vector3d(-0.1, 0.0, 0.1));
  Tangent delta;
  delta << 0.05, -0.1, 0.02, 0.5, -0.5, 1.0, -0.1, 0.2, 0.0, 0.005, 0.0, -0.005,
      -0.01, 0.0, 0.01;
  MEKFState pert = traits<MEKFState>::Retract(ref, delta);
  Tangent rec = traits<MEKFState>::Local(ref, pert);
  EXPECT(assert_equal((Vector)delta, (Vector)rec, 1e-7));
}

// Local is the difference: Local(a, b) with a==b is zero.
TEST(MEKFState, LocalOfEqualStatesIsZero) {
  MEKFState a(Rot3::Ry(0.2), Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2),
              Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0, 0.1, 0));
  Tangent rec = traits<MEKFState>::Local(a, a);
  EXPECT(assert_equal((Vector)Tangent::Zero(), (Vector)rec, kTol));
}

// Print smoke test (must not crash).
TEST(MEKFState, PrintDoesNotCrash) {
  traits<MEKFState>::Print(MEKFState::identity(), "identity: ");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
