/**
 * @file  testMekfState.cpp
 * @brief Unit tests for the MekfState manifold (Retract/Local, identity).
 *
 * Tangent block order: R, v, p, b_gyro, b_accel.
 * Body/right multiplicative attitude error: R_est = R_ref * Exp(d_theta).
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/MEKFGeometry.h>

using namespace mekf;
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

TEST(MekfState, IdentityIsZero) {
  MekfState xi = MekfState::identity();
  EXPECT(xi.R.equals(Rot3::Identity(), kTol));
  EXPECT(veq(xi.v, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.p, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.b_gyro, Eigen::Vector3d::Zero()));
  EXPECT(veq(xi.b_accel, Eigen::Vector3d::Zero()));
}

TEST(MekfState, DirectConstructor) {
  const Rot3 R = Rot3::Rz(0.4);
  const Eigen::Vector3d v(1, 2, 3), p(4, 5, 6), bg(0.1, 0.2, 0.3),
      ba(-0.1, 0.0, 0.1);
  MekfState xi(R, v, p, bg, ba);
  EXPECT(xi.R.equals(R, kTol));
  EXPECT(veq(xi.v, v));
  EXPECT(veq(xi.p, p));
  EXPECT(veq(xi.b_gyro, bg));
  EXPECT(veq(xi.b_accel, ba));
}

// ---------------------------------------------------------------------------
// Equals
// ---------------------------------------------------------------------------

TEST(MekfState, EqualsIdentities) {
  EXPECT(traits<MekfState>::Equals(MekfState::identity(), MekfState::identity(),
                                   kTol));
}

// Each field must contribute to equality: perturb one block at a time.
TEST(MekfState, EqualsDistinguishesEveryBlock) {
  const MekfState base = MekfState::identity();
  {
    MekfState b = base;
    b.R = Rot3::Rx(0.01);
    EXPECT(!traits<MekfState>::Equals(base, b, kTol));
  }
  {
    MekfState b = base;
    b.v = Eigen::Vector3d(1, 0, 0);
    EXPECT(!traits<MekfState>::Equals(base, b, kTol));
  }
  {
    MekfState b = base;
    b.p = Eigen::Vector3d(0, 1, 0);
    EXPECT(!traits<MekfState>::Equals(base, b, kTol));
  }
  {
    MekfState b = base;
    b.b_gyro = Eigen::Vector3d(0, 0, 1);
    EXPECT(!traits<MekfState>::Equals(base, b, kTol));
  }
  {
    MekfState b = base;
    b.b_accel = Eigen::Vector3d(0, 1, 0);
    EXPECT(!traits<MekfState>::Equals(base, b, kTol));
  }
}

// ---------------------------------------------------------------------------
// Retract / Local
// ---------------------------------------------------------------------------

// Retract(xi, 0) == xi for a non-identity state.
TEST(MekfState, RetractZeroDeltaIsNoop) {
  MekfState xi(Rot3::Ry(0.3), Eigen::Vector3d(0.1, 0.2, 0.3),
               Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(0.01, 0.02, 0.03),
               Eigen::Vector3d(-0.01, 0.0, 0.01));
  MekfState result = traits<MekfState>::Retract(xi, Tangent::Zero());
  EXPECT(traits<MekfState>::Equals(xi, result, kTol));
}

// Verify each tangent block maps to the right state field (ordering contract).
TEST(MekfState, RetractBlockOrdering) {
  const MekfState xi = MekfState::identity();
  Tangent d = Tangent::Zero();
  d.segment<3>(0) = Eigen::Vector3d(0.1, 0.0, 0.0);      // d_theta
  d.segment<3>(3) = Eigen::Vector3d(1.0, 2.0, 3.0);      // d_v
  d.segment<3>(6) = Eigen::Vector3d(4.0, 5.0, 6.0);      // d_p
  d.segment<3>(9) = Eigen::Vector3d(0.7, 0.8, 0.9);      // d_b_gyro
  d.segment<3>(12) = Eigen::Vector3d(-0.1, -0.2, -0.3);  // d_b_accel

  MekfState r = traits<MekfState>::Retract(xi, d);
  EXPECT(r.R.equals(Rot3::Expmap(d.segment<3>(0)), kTol));
  EXPECT(veq(r.v, d.segment<3>(3)));
  EXPECT(veq(r.p, d.segment<3>(6)));
  EXPECT(veq(r.b_gyro, d.segment<3>(9)));
  EXPECT(veq(r.b_accel, d.segment<3>(12)));
}

// Attitude retraction is right-multiplicative: R_est = R_ref * Exp(d_theta).
TEST(MekfState, RetractAttitudeIsRightMultiplicative) {
  const Rot3 R0 = Rot3::Rz(0.5) * Rot3::Rx(0.2);
  MekfState xi(R0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
               Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const Eigen::Vector3d dtheta(0.05, -0.1, 0.02);
  Tangent d = Tangent::Zero();
  d.segment<3>(0) = dtheta;
  MekfState r = traits<MekfState>::Retract(xi, d);
  EXPECT(r.R.equals(R0 * Rot3::Expmap(dtheta), kTol));
}

// Local(ref, Retract(ref, delta)) == delta at the identity origin.
TEST(MekfState, RetractLocalRoundTripFromIdentity) {
  Tangent delta;
  delta << 0.1, -0.2, 0.05, 1.0, 2.0, 3.0, -1.0, 0.5, 0.2, 0.01, 0.02, 0.03,
      -0.01, 0.0, 0.01;
  const MekfState ref = MekfState::identity();
  MekfState ret = traits<MekfState>::Retract(ref, delta);
  Tangent rec = traits<MekfState>::Local(ref, ret);
  EXPECT(assert_equal((Vector)delta, (Vector)rec, kTol));
}

// Local(ref, Retract(ref, delta)) == delta for a non-identity reference.
TEST(MekfState, RetractLocalRoundTripFromNonIdentity) {
  MekfState ref(Rot3::Rz(0.5) * Rot3::Rx(0.3), Eigen::Vector3d(1.0, 0.5, -0.2),
                Eigen::Vector3d(10.0, -5.0, 2.0),
                Eigen::Vector3d(0.01, -0.01, 0.02),
                Eigen::Vector3d(-0.1, 0.0, 0.1));
  Tangent delta;
  delta << 0.05, -0.1, 0.02, 0.5, -0.5, 1.0, -0.1, 0.2, 0.0, 0.005, 0.0, -0.005,
      -0.01, 0.0, 0.01;
  MekfState pert = traits<MekfState>::Retract(ref, delta);
  Tangent rec = traits<MekfState>::Local(ref, pert);
  EXPECT(assert_equal((Vector)delta, (Vector)rec, 1e-7));
}

// Local is the difference: Local(a, b) with a==b is zero.
TEST(MekfState, LocalOfEqualStatesIsZero) {
  MekfState a(Rot3::Ry(0.2), Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2),
              Eigen::Vector3d(0.1, 0, 0), Eigen::Vector3d(0, 0.1, 0));
  Tangent rec = traits<MekfState>::Local(a, a);
  EXPECT(assert_equal((Vector)Tangent::Zero(), (Vector)rec, kTol));
}

// Print smoke test (must not crash).
TEST(MekfState, PrintDoesNotCrash) {
  traits<MekfState>::Print(MekfState::identity(), "identity: ");
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
