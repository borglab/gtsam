/**
 * @file  testTGBodyVelocityOutput.cpp
 * @brief The DVL body-velocity output h(xi) = R^T v, its output action, and the
 *        order of accuracy of C0 against C*.
 *
 * Layer above testTGSymmetry.cpp: the equivariant measurement model. The order
 * of accuracy fixture at the bottom is the reason C* exists, and it is the only
 * test here that separates C* from C0 -- they coincide at convergence.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/tg_eqf/TGEqF.h>

using namespace gtsam::tgeqf;
using namespace gtsam;

/* ************************************************************************* */
namespace fixture {

constexpr double kTol = 1e-9;
constexpr double kTolL = 1e-7;

using Tangent = Eigen::Matrix<double, 18, 1>;

bool veq(const Vector3& a, const Vector3& b, double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}

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

TGElement makeElement(const Rot3& R, const Vector3& v, const Vector3& p,
                      const Vector9& a) {
  Eigen::Matrix<double, 3, 2> vp;
  vp.col(0) = v;
  vp.col(1) = p;
  return TGElement(ExtendedPose3<2>(R, vp), a);
}

TGElement makeX() {
  return makeElement(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Vector3(1.0, -2.0, 0.5),
      Vector3(0.3, 0.1, -0.4),
      (Vector9() << 0.1, -0.1, 0.05, -0.2, 0.3, 0.0, 0.0, 0.1, -0.1)
          .finished());
}

TGElement makeY() {
  return makeElement(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Vector3(-0.5, 1.0, 2.0),
      Vector3(0.2, -0.3, 0.1),
      (Vector9() << 0.05, 0.0, -0.05, 0.1, -0.1, 0.2, -0.1, 0.0, 0.05)
          .finished());
}

}  // namespace fixture
/* ************************************************************************* */

/* ************************************************************************* */
namespace output_action {

// psi is a right action on the output space, the axiom that lets the filter
// carry a measurement from the estimate's body frame to the origin.
TEST(BodyVelocityOutput, IsARightAction) {
  const Vector3 y(0.3, -0.1, 0.2);
  const TGElement X = fixture::makeX(), Y = fixture::makeY();

  EXPECT(
      fixture::veq(y, DVLMeasurement::output_action(TGElement::Identity(), y)));
  EXPECT(fixture::veq(
      DVLMeasurement::output_action(X * Y, y),
      DVLMeasurement::output_action(Y, DVLMeasurement::output_action(X, y)),
      fixture::kTolL));
}

// inverse_output_action is psi_{X^-1}, which for a group action is psi_X^-1.
// It exists only to skip building the fiber part of X.inverse().
TEST(BodyVelocityOutput, InverseActionIsTheActionOfTheInverse) {
  const Vector3 y(0.3, -0.1, 0.2);
  const TGElement X = fixture::makeX();

  EXPECT(fixture::veq(DVLMeasurement::output_action(X.inverse(), y),
                      DVLMeasurement::inverse_output_action(X, y)));
  EXPECT(fixture::veq(y, DVLMeasurement::inverse_output_action(
                             X, DVLMeasurement::output_action(X, y))));
}

// Output equivariance h(phi(X, xi)) = psi_X(h(xi)). Without it the origin-chart
// comparison the filter makes is not the measurement residual at all.
TEST(BodyVelocityOutput, IsEquivariant) {
  const State xi = fixture::makeXi();

  Eigen::Matrix<double, 18, 1> logX;
  logX << 0.15, -0.1, 0.05, 1.0, -2.0, 0.5, 0.3, 0.1, -0.4, 0.1, -0.1, 0.05,
      -0.2, 0.3, 0.0, 0.0, 0.1, -0.1;

  for (const TGElement& X :
       {TGElement::Identity(), fixture::makeX(), TGElement::Expmap(logX)}) {
    EXPECT(fixture::veq(
        DVLMeasurement::predict(phi(X, xi)),
        DVLMeasurement::output_action(X, DVLMeasurement::predict(xi)),
        fixture::kTolL));
  }
}

}  // namespace output_action
/* ************************************************************************* */

/* ************************************************************************* */
namespace output_matrix {

// C0 is the chart derivative of the output map at the reference state.
TEST(BodyVelocityOutput, C0MatchesNumerical) {
  const State xi_ref = fixture::makeXi();
  const Eigen::Matrix<double, 3, 18> H_num =
      numericalDerivative11<Vector3, State>(
          [](const State& x) { return DVLMeasurement::predict(x); }, xi_ref);

  EXPECT(assert_equal((Matrix)DVLMeasurement::jacobian_C0(xi_ref),
                      (Matrix)H_num, 1e-5));
}

// At convergence the transported measurement is the origin output itself, so
// the C* midpoint collapses onto C0. The accuracy fixture below therefore works
// away from convergence, where the two differ.
TEST(BodyVelocityOutput, CstarMeetsC0AtConvergence) {
  const State xi_ref = fixture::makeXi();
  const TGElement g = fixture::makeX();
  const Vector3 z = DVLMeasurement::predict(phi(g, xi_ref));

  EXPECT(assert_equal((Matrix)DVLMeasurement::jacobian_C0(xi_ref),
                      (Matrix)DVLMeasurement::jacobian_Cstar(xi_ref, g, z),
                      fixture::kTol));
}

}  // namespace output_matrix
/* ************************************************************************* */

/* ************************************************************************* */
namespace accuracy {

const State kXiRef = fixture::makeXi();
const TGElement kG = fixture::makeX();

/// Body velocity the true state at eps reports to the filter.
Vector3 measurementAt(const fixture::Tangent& eps) {
  return DVLMeasurement::predict(phi(kG, traits<State>::Retract(kXiRef, eps)));
}

/**
 * The residual the filter linearizes, as a function of the error.
 *
 * TGEqF::update_dvl compares predict(xi_ref) against psi_X^{-1}(z), so an
 * output matrix C is correct to the extent that
 *
 *   C eps  ~=  psi_X^{-1}(z(eps)) - predict(xi_ref),
 *
 * with eps parametrising the true state as xi_true = phi(g, Retract(xi_ref,
 * eps)). The measurement moves with eps, which makes this the innovation
 * Jacobian rather than the Jacobian of the output map alone.
 */
Vector3 residual(const fixture::Tangent& eps) {
  return DVLMeasurement::inverse_output_action(kG, measurementAt(eps)) -
         DVLMeasurement::predict(kXiRef);
}

double errorC0(const fixture::Tangent& eps) {
  return (residual(eps) - DVLMeasurement::jacobian_C0(kXiRef) * eps).norm();
}

double errorCstar(const fixture::Tangent& eps) {
  return (residual(eps) -
          DVLMeasurement::jacobian_Cstar(kXiRef, kG, measurementAt(eps)) * eps)
      .norm();
}

/// Ratio of the linearization error at half amplitude to that at full: ~1/4 for
/// a second-order matrix, ~1/8 for a third-order one.
double halvingRatio(double (*error)(const fixture::Tangent&),
                    const fixture::Tangent& d) {
  return error(0.1 * d) / error(0.2 * d);
}

// Halving the error cuts C*'s linearization error by ~8 and C0's by ~4. C*
// takes the output differential at the midpoint of the origin output and the
// transported measurement, which follows the residual to third order; C0 takes
// it at the origin output alone and reaches only second.
TEST(BodyVelocityOutput, CstarIsThirdOrderInAttitudeError) {
  fixture::Tangent d = fixture::Tangent::Zero();
  d.segment<3>(0) = Vector3(0.6, -0.5, 0.62).normalized();

  // Both matrices do err at this amplitude, so the ratios are not vacuous.
  EXPECT(errorC0(0.2 * d) > 1e-4);
  EXPECT(errorCstar(0.2 * d) > 1e-8);
  EXPECT(errorCstar(0.1 * d) < errorC0(0.1 * d));

  EXPECT(halvingRatio(errorC0, d) > 0.20);  // ~1/4
  EXPECT(halvingRatio(errorC0, d) < 0.35);
  EXPECT(halvingRatio(errorCstar, d) > 0.05);  // ~1/8
  EXPECT(halvingRatio(errorCstar, d) < 0.20);
}

// Third order survives a mixed attitude-velocity error. This is what the
// SE_2(3) logarithm chart buys: the second-order cross term the left Jacobian
// of the exponential contributes cancels the one the C* midpoint contributes.
TEST(BodyVelocityOutput, CstarIsThirdOrderInMixedError) {
  fixture::Tangent d = fixture::Tangent::Zero();
  d.segment<3>(0) = Vector3(0.4, -0.3, 0.5);
  d.segment<3>(3) = Vector3(-0.5, 0.2, 0.35);
  d.normalize();

  EXPECT(errorCstar(0.2 * d) > 1e-8);
  EXPECT(halvingRatio(errorCstar, d) > 0.05);  // ~1/8
  EXPECT(halvingRatio(errorCstar, d) < 0.20);
  EXPECT(errorCstar(0.2 * d) < 0.5 * errorC0(0.2 * d));
}

}  // namespace accuracy
/* ************************************************************************* */

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
