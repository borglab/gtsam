/**
 * @file  testTFGGroup.cpp
 * @brief Unit tests for the Two-Frames Group G_TF = SO(3) |x (R^6 + R^6).
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/tfg_inekf/TFGGeometry.h>

using namespace tfg;
using namespace gtsam;

static constexpr double kTol = 1e-9;
static constexpr double kTolL = 1e-7;  // looser for trig-composed operations

using Vec15 = Eigen::Matrix<double, 15, 1>;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}
static bool meq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                double tol = kTol) {
  return assert_equal((Matrix)a, (Matrix)b, tol);
}

static TwoFrameGroup makeX() {
  return TwoFrameGroup(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Eigen::Vector3d(0.3, 0.1, -0.4),
      Eigen::Vector3d(1.0, -2.0, 0.5), Eigen::Vector3d(0.1, -0.1, 0.05),
      Eigen::Vector3d(-0.2, 0.3, 0.0));
}
static TwoFrameGroup makeY() {
  return TwoFrameGroup(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Eigen::Vector3d(0.2, -0.3, 0.1),
      Eigen::Vector3d(-0.5, 1.0, 2.0), Eigen::Vector3d(0.05, 0.0, -0.05),
      Eigen::Vector3d(0.1, -0.1, 0.2));
}

// ---------------------------------------------------------------------------
// Identity and matrix realisation
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, Identity) {
  auto I = TwoFrameGroup::Identity();
  EXPECT(I.R.equals(Rot3::Identity(), kTol));
  EXPECT(veq(I.v, Eigen::Vector3d::Zero()));
  EXPECT(veq(I.p, Eigen::Vector3d::Zero()));
  EXPECT(veq(I.gamma_omega, Eigen::Vector3d::Zero()));
  EXPECT(veq(I.gamma_accel, Eigen::Vector3d::Zero()));
}

TEST(TwoFrameGroup, CMatrixBlocks) {
  auto X = makeX();
  auto C = X.C_matrix();
  EXPECT(meq(X.R.matrix(), C.block<3, 3>(0, 0)));
  EXPECT(veq(X.v, C.block<3, 1>(0, 3)));
  EXPECT(veq(X.p, C.block<3, 1>(0, 4)));
  DOUBLES_EQUAL(1.0, C(3, 3), kTol);
  DOUBLES_EQUAL(1.0, C(4, 4), kTol);
  EXPECT(meq(C.block<2, 3>(3, 0), Eigen::Matrix<double, 2, 3>::Zero()));
}

// ---------------------------------------------------------------------------
// Group axioms
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, IdentityNeutral) {
  auto X = makeX();
  auto I = TwoFrameGroup::Identity();
  EXPECT(traits<TwoFrameGroup>::Equals(I * X, X, kTol));
  EXPECT(traits<TwoFrameGroup>::Equals(X * I, X, kTol));
}

TEST(TwoFrameGroup, Inverse) {
  auto X = makeX();
  auto I = TwoFrameGroup::Identity();
  EXPECT(traits<TwoFrameGroup>::Equals(X.inverse() * X, I, kTolL));
  EXPECT(traits<TwoFrameGroup>::Equals(X * X.inverse(), I, kTolL));
}

TEST(TwoFrameGroup, Associativity) {
  auto X = makeX(), Y = makeY();
  auto Z = TwoFrameGroup(
      Rot3::Rx(0.5), Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.2, -0.1, 0.0),
      Eigen::Vector3d(0.0, 0.1, 0.0), Eigen::Vector3d(-0.1, 0.0, 0.2));
  EXPECT(traits<TwoFrameGroup>::Equals((X * Y) * Z, X * (Y * Z), kTolL));
}

// ---------------------------------------------------------------------------
// State <-> group coordinates  (Eq. B.31:  b = A^{-1} * (-gamma))
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, FromStateBiasRoundTrip) {
  Rot3 R = Rot3::Rz(0.4) * Rot3::Rx(0.2);
  Eigen::Vector3d v(0.3, 0.1, -0.4), p(1.0, -2.0, 0.5);
  Eigen::Vector3d bw(0.02, -0.01, 0.03), ba(-0.04, 0.05, 0.0);
  auto X = TwoFrameGroup::FromState(R, v, p, bw, ba);
  EXPECT(X.R.equals(R, kTol));
  EXPECT(veq(X.v, v));
  EXPECT(veq(X.p, p));
  EXPECT(veq(X.bias_omega(), bw, kTolL));
  EXPECT(veq(X.bias_accel(), ba, kTolL));
}

// ---------------------------------------------------------------------------
// Expmap / Logmap (exact)
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, ExpmapAtZeroIsIdentity) {
  auto X = TwoFrameGroup::Expmap(Vec15::Zero());
  EXPECT(traits<TwoFrameGroup>::Equals(X, TwoFrameGroup::Identity(), kTol));
}

TEST(TwoFrameGroup, LogmapOfIdentityIsZero) {
  EXPECT(assert_equal((Vector)Vec15::Zero(),
                      (Vector)TwoFrameGroup::Identity().Logmap(), kTol));
}

TEST(TwoFrameGroup, ExpLogRoundTrip) {
  Vec15 xi;
  xi << 0.2, -0.3, 0.1, 0.3, -0.5, 0.2, -0.1, 0.4, -0.2, 0.05, 0.1, -0.05, -0.2,
      0.3, 0.0;
  auto rec = TwoFrameGroup::Expmap(xi).Logmap();
  EXPECT(assert_equal((Vector)xi, (Vector)rec, kTolL));
}

// ---------------------------------------------------------------------------
// Adjoint
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, AdjointAtIdentityIsI15) {
  auto Ad = traits<TwoFrameGroup>::AdjointMap(TwoFrameGroup::Identity());
  EXPECT(meq(Ad, Eigen::Matrix<double, 15, 15>::Identity()));
}

// Ad is a homomorphism: Ad_{XY} == Ad_X Ad_Y.
TEST(TwoFrameGroup, AdjointHomomorphism) {
  auto X = makeX(), Y = makeY();
  auto lhs = traits<TwoFrameGroup>::AdjointMap(X * Y);
  auto rhs = traits<TwoFrameGroup>::AdjointMap(X) *
             traits<TwoFrameGroup>::AdjointMap(Y);
  EXPECT(meq(lhs, rhs, kTolL));
}

// Defining property of the adjoint:  Exp(Ad_X xi) == X Exp(xi) X^{-1}.
TEST(TwoFrameGroup, AdjointConjugationIdentity) {
  auto X = makeX();
  Vec15 xi;
  xi << 0.05, -0.1, 0.03, 0.2, -0.3, 0.1, -0.1, 0.2, -0.05, 0.01, 0.02, -0.01,
      0.05, 0.0, -0.05;
  auto lhs = TwoFrameGroup::Expmap(traits<TwoFrameGroup>::AdjointMap(X) * xi);
  auto rhs = X * TwoFrameGroup::Expmap(xi) * X.inverse();
  EXPECT(traits<TwoFrameGroup>::Equals(lhs, rhs, kTolL));
}

// ---------------------------------------------------------------------------
// Expmap / Logmap chart Jacobians (right chart) vs numerical derivative
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, ExpmapJacobian) {
  Vec15 xi;
  xi << 0.2, -0.3, 0.1, 0.3, -0.5, 0.2, -0.1, 0.4, -0.2, 0.05, 0.1, -0.05, -0.2,
      0.3, 0.0;
  Eigen::Matrix<double, 15, 15> H;
  traits<TwoFrameGroup>::Expmap(xi, H);
  auto num = numericalDerivative11<TwoFrameGroup, Vec15>(
      [](const Vec15& x) { return traits<TwoFrameGroup>::Expmap(x); }, xi);
  EXPECT(meq(H, num, 1e-6));
}

TEST(TwoFrameGroup, LogmapJacobian) {
  Vec15 xi;
  xi << 0.2, -0.3, 0.1, 0.3, -0.5, 0.2, -0.1, 0.4, -0.2, 0.05, 0.1, -0.05, -0.2,
      0.3, 0.0;
  auto X = TwoFrameGroup::Expmap(xi);
  Eigen::Matrix<double, 15, 15> H;
  traits<TwoFrameGroup>::Logmap(X, H);
  auto num = numericalDerivative11<Vec15, TwoFrameGroup>(
      [](const TwoFrameGroup& Y) { return traits<TwoFrameGroup>::Logmap(Y); },
      X);
  EXPECT(meq(H, num, 1e-6));
}

// ---------------------------------------------------------------------------
// Retract / Local round-trip
// ---------------------------------------------------------------------------

TEST(TwoFrameGroup, RetractLocalFromIdentity) {
  Vec15 d;
  d << 0.05, -0.1, 0.03, 0.2, -0.3, 0.1, -0.1, 0.2, -0.05, 0.01, 0.02, -0.01,
      0.05, 0.0, -0.05;
  auto I = TwoFrameGroup::Identity();
  auto X = traits<TwoFrameGroup>::Retract(I, d);
  EXPECT(assert_equal((Vector)d, (Vector)traits<TwoFrameGroup>::Local(I, X),
                      kTolL));
}

TEST(TwoFrameGroup, RetractLocalFromNonIdentity) {
  auto X = makeX();
  Vec15 d;
  d << 0.02, -0.05, 0.01, 0.1, -0.1, 0.05, -0.05, 0.1, 0.0, 0.005, 0.0, -0.005,
      0.01, -0.01, 0.0;
  auto Y = traits<TwoFrameGroup>::Retract(X, d);
  EXPECT(assert_equal((Vector)d, (Vector)traits<TwoFrameGroup>::Local(X, Y),
                      kTolL));
}

// ---------------------------------------------------------------------------
// Independent ground-truth checks (catch compensating bugs the axiom/round-trip
// tests miss).
// ---------------------------------------------------------------------------

// Nav part composes exactly as the 5x5 SE_2(3) matrix product, and the gamma
// blocks compose by pure rotation: gamma_XY = gamma_X + R_X gamma_Y.
TEST(TwoFrameGroup, ComposeMatchesMatrixRealisation) {
  auto X = makeX(), Y = makeY();
  auto XY = X * Y;
  EXPECT(meq(XY.C_matrix(), X.C_matrix() * Y.C_matrix(), kTolL));
  EXPECT(veq(XY.gamma_omega, X.gamma_omega + X.R * Y.gamma_omega, kTolL));
  EXPECT(veq(XY.gamma_accel, X.gamma_accel + X.R * Y.gamma_accel, kTolL));
}

// Inverse nav part equals the inverse 5x5 matrix; gamma block = -R^T gamma.
TEST(TwoFrameGroup, InverseMatchesMatrixRealisation) {
  auto X = makeX();
  auto Xi = X.inverse();
  EXPECT(meq(Xi.C_matrix(), X.C_matrix().inverse(), kTolL));
  EXPECT(veq(Xi.gamma_omega, -(X.R.unrotate(X.gamma_omega)), kTolL));
  EXPECT(veq(Xi.gamma_accel, -(X.R.unrotate(X.gamma_accel)), kTolL));
}

// Adjoint block structure (closed form): diagonal A on every block, and the
// d_theta column couples each vector block u_i via skew(u_i) * A.
TEST(TwoFrameGroup, AdjointMapBlockStructure) {
  auto X = makeX();
  auto Ad = traits<TwoFrameGroup>::AdjointMap(X);
  const Matrix3 A = X.R.matrix();

  // Diagonal rotation blocks
  for (int b : {0, 3, 6, 9, 12}) EXPECT(meq(Ad.block<3, 3>(b, b), A));
  // Coupling columns under d_theta
  EXPECT(meq(Ad.block<3, 3>(3, 0), skewSymmetric(X.v) * A, kTolL));
  EXPECT(meq(Ad.block<3, 3>(6, 0), skewSymmetric(X.p) * A, kTolL));
  EXPECT(meq(Ad.block<3, 3>(9, 0), skewSymmetric(X.gamma_omega) * A, kTolL));
  EXPECT(meq(Ad.block<3, 3>(12, 0), skewSymmetric(X.gamma_accel) * A, kTolL));
  // Off-diagonal vector-to-vector blocks are zero (no inter-block coupling)
  EXPECT(meq(Ad.block<3, 3>(3, 6), Matrix3::Zero()));
  EXPECT(meq(Ad.block<3, 3>(9, 12), Matrix3::Zero()));
}

// Expmap closed-form spot checks: pure rotation, and pure translation (theta=0
// => left Jacobian = I, so vector blocks pass through unchanged).
TEST(TwoFrameGroup, ExpmapKnownValues) {
  // Pure rotation: only R set, all vectors zero.
  Vec15 r = Vec15::Zero();
  r.segment<3>(0) = Eigen::Vector3d(0.1, -0.2, 0.3);
  auto Xr = TwoFrameGroup::Expmap(r);
  EXPECT(Xr.R.equals(Rot3::Expmap(r.segment<3>(0)), kTol));
  EXPECT(veq(Xr.v, Eigen::Vector3d::Zero()));
  EXPECT(veq(Xr.gamma_accel, Eigen::Vector3d::Zero()));

  // Zero rotation: vector blocks map identically (Jl = I).
  Vec15 t = Vec15::Zero();
  t.segment<3>(3) = Eigen::Vector3d(1.0, 2.0, 3.0);
  t.segment<3>(6) = Eigen::Vector3d(-1.0, 0.5, 0.2);
  t.segment<3>(9) = Eigen::Vector3d(0.1, 0.0, -0.1);
  t.segment<3>(12) = Eigen::Vector3d(0.0, -0.3, 0.4);
  auto Xt = TwoFrameGroup::Expmap(t);
  EXPECT(Xt.R.equals(Rot3::Identity(), kTol));
  EXPECT(veq(Xt.v, t.segment<3>(3)));
  EXPECT(veq(Xt.p, t.segment<3>(6)));
  EXPECT(veq(Xt.gamma_omega, t.segment<3>(9)));
  EXPECT(veq(Xt.gamma_accel, t.segment<3>(12)));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
