/**
 * @file  testTGDepthOutput.cpp
 * @brief The non-equivariant depth output h(xi) = e_3^T p and its C matrix.
 *
 * The sibling output tests start from an equivariant output action and build a
 * C* that no numerical derivative can check. This output has no action, so its
 * C is checked directly instead, and the missing-action tests are replaced by
 * the demonstration that no action could exist.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/tg_eqf/TGEqF.h>

#include <cmath>

using namespace gtsam::tgeqf;
using namespace gtsam;

/* ************************************************************************* */
namespace fixture {

constexpr double kTol = 1e-9;

using Tangent = Eigen::Matrix<double, 18, 1>;

State makeXi() {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = Vector3(1.0, -2.0, 0.5);
  vp.col(1) = Vector3(0.3, 0.1, -0.4);
  State xi;
  xi.T = ExtendedPose3<2>(Rot3::Rz(0.3) * Rot3::Rx(0.1), vp);
  xi.b = (Vector9() << 0.01, -0.02, 0.03, -0.1, 0.05, 0.0, 0.0, 0.1, -0.05)
             .finished();
  return xi;
}

TGElement makeX() {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = Vector3(1.0, -2.0, 0.5);
  vp.col(1) = Vector3(0.3, 0.1, -0.4);
  const Vector9 a =
      (Vector9() << 0.1, -0.1, 0.05, -0.2, 0.3, 0.0, 0.0, 0.1, -0.1).finished();
  return TGElement(ExtendedPose3<2>(Rot3::Rz(0.4) * Rot3::Rx(0.2), vp), a);
}

}  // namespace fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace output_matrix {

// predict() reads the raw vertical position, e_3^T p, with no transform.
TEST(DepthOutput, PredictReadsVerticalPosition) {
  const State xi = fixture::makeXi();
  EXPECT_DOUBLES_EQUAL(xi.T.x(1).z(), DepthMeasurement::predict(xi),
                       fixture::kTol);
}

// At the group identity C is the plain chart derivative of predict().
TEST(DepthOutput, CMatchesNumericalAtTheOrigin) {
  const State xi_ref = fixture::makeXi();
  const Matrix H_num = numericalDerivative11<double, State>(
      [](const State& xi) { return DepthMeasurement::predict(xi); }, xi_ref);

  EXPECT(assert_equal(
      (Matrix)DepthMeasurement::jacobian_C(xi_ref, TGElement::Identity()),
      H_num, 1e-5));
}

// C is an honest first derivative, so it is checked directly. The
// non-identity xi_ref is the only case exercising the n = R0^T e_3 factor.
TEST(DepthOutput, CMatchesNumericalAtANonIdentityEstimate) {
  const TGElement X = fixture::makeX();

  auto check = [&](const State& xi_ref) {
    const Matrix H_num = numericalDerivative11<double, State>(
        [&X](const State& e) { return DepthMeasurement::predict(phi(X, e)); },
        xi_ref);
    EXPECT(assert_equal((Matrix)DepthMeasurement::jacobian_C(xi_ref, X), H_num,
                        1e-5));
  };

  check(State::identity());
  check(fixture::makeXi());
}

}  // namespace output_matrix
/* ************************************************************************* */

/* ************************************************************************* */
namespace equivariance {

// An output action rho(X, y) would send two states with the same y = h(xi) to
// the same rho(X, y). Same p, different R, one X: the outputs disagree by the
// R-dependent term e_3^T R p_A, so no such action exists.
TEST(DepthOutput, HasNoEquivariantOutputAction) {
  State xi_a = fixture::makeXi();
  State xi_b = xi_a;
  Eigen::Matrix<double, 3, 2> vp_b;
  vp_b.col(0) = xi_a.T.x(0);
  vp_b.col(1) = xi_a.T.x(1);
  // Different R, same p, so same h(xi).
  xi_b.T = ExtendedPose3<2>(Rot3::Ry(0.7) * xi_a.T.rotation(), vp_b);

  EXPECT_DOUBLES_EQUAL(DepthMeasurement::predict(xi_a),
                       DepthMeasurement::predict(xi_b), fixture::kTol);

  const TGElement X = fixture::makeX();  // X.p != 0
  const double y_a = DepthMeasurement::predict(phi(X, xi_a));
  const double y_b = DepthMeasurement::predict(phi(X, xi_b));

  EXPECT(std::abs(y_a - y_b) > 1e-3);
}

}  // namespace equivariance
/* ************************************************************************* */

/* ************************************************************************* */
namespace accuracy {

const State kXiRef = fixture::makeXi();
const TGElement kG = fixture::makeX();

/// Depth reading of the true state at eps, xi_true = phi(kG, Retract(xi_ref,
/// eps)).
double measurementAt(const fixture::Tangent& eps) {
  return DepthMeasurement::predict(
      phi(kG, traits<State>::Retract(kXiRef, eps)));
}

/// What C linearizes: h(xi_true(eps)) - h(xi_hat). Increasing in p, so the
/// sign is opposite to testTGPositionOutput.cpp's residual.
double residual(const fixture::Tangent& eps) {
  return measurementAt(eps) - DepthMeasurement::predict(phi(kG, kXiRef));
}

double errorC(const fixture::Tangent& eps) {
  return std::abs(residual(eps) -
                  (DepthMeasurement::jacobian_C(kXiRef, kG) * eps)(0));
}

/// Ratio of the linearization error at half amplitude to that at full: ~1/4
/// for a second-order matrix.
double halvingRatio(double (*error)(const fixture::Tangent&),
                    const fixture::Tangent& d) {
  return error(0.1 * d) / error(0.2 * d);
}

// Halving the attitude error cuts the linearization error by only ~4, against
// ~8 for PositionOutput's third-order C*. That gap is the cost of dropping the
// pseudo-position construction.
TEST(DepthOutput, CIsSecondOrderInAttitudeError) {
  fixture::Tangent d = fixture::Tangent::Zero();
  d.segment<3>(0) = Vector3(0.6, -0.5, 0.62).normalized();

  // The matrix does err at this amplitude, so the ratio is not vacuous.
  EXPECT(errorC(0.2 * d) > 1e-6);

  const double ratio = halvingRatio(errorC, d);
  EXPECT(ratio > 0.20);  // ~1/4
  EXPECT(ratio < 0.35);
}

}  // namespace accuracy
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
